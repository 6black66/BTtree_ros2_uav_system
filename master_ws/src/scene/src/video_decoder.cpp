#include <video_gateway_ros2/video_decoder.hpp>
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}
#include <chrono>
#include <mutex>
#include <stdexcept>

namespace uav::scene {
namespace {
int64_t monotonic_us() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
void checked(int result, const char* operation) {
  if(result >= 0) return;
  char message[AV_ERROR_MAX_STRING_SIZE]{};
  av_strerror(result, message, sizeof(message));
  throw std::runtime_error(std::string(operation) + ": " + message);
}
AVPixelFormat output_format(video::PixelFormat format) {
  switch(format) {
    case video::PixelFormat::bgr8: return AV_PIX_FMT_BGR24;
    case video::PixelFormat::rgb8: return AV_PIX_FMT_RGB24;
    case video::PixelFormat::mono8: return AV_PIX_FMT_GRAY8;
  }
  throw std::invalid_argument("unsupported output format");
}
}
struct VideoDecoder::Impl {
  DecoderConfig config;
  std::atomic<bool>& stopping;
  int64_t deadline_us = 0;  // FFmpeg calls the interrupt callback on the worker thread.
  AVFormatContext* input = nullptr;
  AVCodecContext* codec = nullptr;
  AVPacket* packet = nullptr;
  AVFrame* frame = nullptr;
  SwsContext* converter = nullptr;
  int video_index = -1;
  bool draining = false;
  double fps = 0;
  Impl(const DecoderConfig& settings, std::atomic<bool>& stop) : config(settings), stopping(stop) {}
  ~Impl() {
    deadline_us = 1;
    sws_freeContext(converter);
    av_frame_free(&frame);
    av_packet_free(&packet);
    avcodec_free_context(&codec);
    avformat_close_input(&input);
  }
  static int interrupt(void* opaque) {
    auto& self = *static_cast<Impl*>(opaque);
    return self.stopping.load() || (self.deadline_us && monotonic_us() >= self.deadline_us);
  }
  void deadline(int timeout_ms) { deadline_us = monotonic_us() + int64_t(timeout_ms) * 1000; }
  void convert(DecodedFrame& out) {
    const int width = frame->width, height = frame->height;
    if(width <= 0 || height <= 0 || width > config.max_width || height > config.max_height)
      throw std::runtime_error("decoded dimensions exceed configured shared memory capacity");
    if(frame->format < 0) throw std::runtime_error("decoder returned invalid pixel format");
    const auto format = output_format(config.format);
    auto source_format = static_cast<AVPixelFormat>(frame->format);
    bool full_range = frame->color_range == AVCOL_RANGE_JPEG;
    // Normalize deprecated JPEG pixel formats so the converter can be reused.
    switch(source_format) {
      case AV_PIX_FMT_YUVJ420P: source_format = AV_PIX_FMT_YUV420P; full_range = true; break;
      case AV_PIX_FMT_YUVJ422P: source_format = AV_PIX_FMT_YUV422P; full_range = true; break;
      case AV_PIX_FMT_YUVJ444P: source_format = AV_PIX_FMT_YUV444P; full_range = true; break;
      case AV_PIX_FMT_YUVJ440P: source_format = AV_PIX_FMT_YUV440P; full_range = true; break;
      default: break;
    }
    converter = sws_getCachedContext(converter, width, height, source_format,
        width, height, format, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if(!converter) throw std::runtime_error("cannot create pixel converter");
    int matrix = SWS_CS_DEFAULT;
    if(frame->colorspace == AVCOL_SPC_BT709) matrix = SWS_CS_ITU709;
    else if(frame->colorspace == AVCOL_SPC_BT2020_NCL || frame->colorspace == AVCOL_SPC_BT2020_CL) matrix = SWS_CS_BT2020;
    const int* coefficients = sws_getCoefficients(matrix);
    checked(sws_setColorspaceDetails(converter, coefficients, full_range ? 1 : 0,
        coefficients, 1, 0, 1 << 16, 1 << 16), "pixel color range");
    const int size = av_image_get_buffer_size(format, width, height, 1);
    checked(size, "image buffer size");
    out.pixels.resize(size);
    uint8_t* destination[4]{};
    int stride[4]{};
    checked(av_image_fill_arrays(destination, stride, out.pixels.data(), format, width, height, 1), "image layout");
    if(sws_scale(converter, frame->data, frame->linesize, 0, height, destination, stride) != height)
      throw std::runtime_error("incomplete pixel conversion");
    out.metadata = {};
    out.metadata.width = width; out.metadata.height = height; out.metadata.step = stride[0];
    out.metadata.data_size = size; out.metadata.format = config.format;
    const int64_t pts = frame->best_effort_timestamp;
    if(pts != AV_NOPTS_VALUE) {
      out.metadata.source_pts_valid = 1;
      out.metadata.source_pts_ns = av_rescale_q(pts, input->streams[video_index]->time_base, AVRational{1,1000000000});
    }
  }
};
VideoDecoder::VideoDecoder(const DecoderConfig& config, std::atomic<bool>& stop)
: impl_(std::make_unique<Impl>(config, stop)) {}
VideoDecoder::~VideoDecoder() = default;
void VideoDecoder::open() {
  auto& self = *impl_;
  static std::once_flag network;
  std::call_once(network, [] { checked(avformat_network_init(), "network initialization"); });
  self.input = avformat_alloc_context();
  if(!self.input) throw std::bad_alloc();
  self.input->interrupt_callback = {Impl::interrupt, &self};
  AVDictionary* options = nullptr;
  struct OptionsCleanup { AVDictionary** p; ~OptionsCleanup() { av_dict_free(p); } } cleanup{&options};
  if(self.config.input_type == "rtsp") av_dict_set(&options, "rtsp_transport", self.config.rtsp_transport.c_str(), 0);
  av_dict_set(&options, "rw_timeout", std::to_string(int64_t(self.config.read_timeout_ms) * 1000).c_str(), 0);
  const AVInputFormat* format = nullptr;
  if(!self.config.input_format.empty()) {
    format = av_find_input_format(self.config.input_format.c_str());
    if(!format) throw std::invalid_argument("unknown FFmpeg input_format");
  }
  self.deadline(self.config.open_timeout_ms);
  checked(avformat_open_input(&self.input, self.config.source_url.c_str(), format, &options), "open video input");
  self.deadline(self.config.open_timeout_ms);
  checked(avformat_find_stream_info(self.input, nullptr), "probe video stream");
  self.video_index = av_find_best_stream(self.input, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  checked(self.video_index, "find video stream");
  auto* stream = self.input->streams[self.video_index];
  const AVCodec* decoder = avcodec_find_decoder(stream->codecpar->codec_id);
  if(!decoder) throw std::runtime_error("video codec is unavailable in this FFmpeg build");
  self.codec = avcodec_alloc_context3(decoder);
  if(!self.codec) throw std::bad_alloc();
  checked(avcodec_parameters_to_context(self.codec, stream->codecpar), "decoder parameters");
  if(self.codec->width > self.config.max_width || self.codec->height > self.config.max_height)
    throw std::runtime_error("video dimensions exceed configured maximum");
  // Allow codec border/scratch surfaces in addition to visible pixels.
  // The output dimensions are checked separately for every decoded frame.
  self.codec->max_pixels = int64_t(self.config.max_width + 512) * (self.config.max_height + 512);
  self.codec->thread_count = 2;
  checked(avcodec_open2(self.codec, decoder, nullptr), "open decoder");
  self.packet = av_packet_alloc();
  self.frame = av_frame_alloc();
  if(!self.packet || !self.frame) throw std::bad_alloc();
  const auto rate = av_guess_frame_rate(self.input, stream, nullptr);
  self.fps = rate.num > 0 && rate.den > 0 ? av_q2d(rate) : 25.0;
  self.deadline_us = 0;
}
double VideoDecoder::nominal_fps() const { return impl_->fps; }
bool VideoDecoder::next(DecodedFrame& out) {
  auto& self = *impl_;
  while(!self.stopping.load()) {
    const int decoded = avcodec_receive_frame(self.codec, self.frame);
    if(decoded == 0) {
      self.convert(out);
      av_frame_unref(self.frame);
      return true;
    }
    if(decoded == AVERROR_EOF) return false;
    if(decoded != AVERROR(EAGAIN)) checked(decoded, "decode frame");
    if(self.draining) return false;
    while(!self.stopping.load()) {
      self.deadline(self.config.read_timeout_ms);
      const int received = av_read_frame(self.input, self.packet);
      self.deadline_us = 0;
      if(received == AVERROR_EOF) {
        self.draining = true;
        checked(avcodec_send_packet(self.codec, nullptr), "drain decoder");
        break;
      }
      checked(received, "read video packet");
      if(self.packet->stream_index != self.video_index) { av_packet_unref(self.packet); continue; }
      const int sent = avcodec_send_packet(self.codec, self.packet);
      av_packet_unref(self.packet);
      checked(sent, "submit video packet");
      break;
    }
  }
  return false;
}
}  // namespace uav::scene
