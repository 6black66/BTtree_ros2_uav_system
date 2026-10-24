// Deterministic MPEG-TS fixture using the installed encoder libraries; no ffmpeg CLI needed.
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
}
#include <cstring>
#include <iostream>
#include <stdexcept>
void check(int result) { if(result < 0) throw std::runtime_error("FFmpeg fixture operation failed: " + std::to_string(result)); }
int main(int argc, char** argv) {
  if(argc != 2) return 2;
  AVFormatContext* output = nullptr; AVCodecContext* codec = nullptr; AVFrame* frame = nullptr; AVPacket* packet = nullptr;
  int result = 0;
  try {
    check(avformat_alloc_output_context2(&output, nullptr, "mpegts", argv[1]));
    const AVCodec* encoder = avcodec_find_encoder(AV_CODEC_ID_MPEG2VIDEO);
    if(!encoder) throw std::runtime_error("MPEG2 fixture encoder unavailable");
    auto* stream = avformat_new_stream(output, nullptr);
    codec = avcodec_alloc_context3(encoder);
    if(!stream || !codec) throw std::bad_alloc();
    codec->width = 64; codec->height = 48; codec->pix_fmt = AV_PIX_FMT_YUV420P;
    codec->time_base = {1,25}; codec->framerate = {25,1}; codec->gop_size = 12; codec->max_b_frames = 0;
    codec->bit_rate = 200000;
    check(avcodec_open2(codec, encoder, nullptr));
    check(avcodec_parameters_from_context(stream->codecpar, codec));
    stream->time_base = codec->time_base;
    check(avio_open(&output->pb, argv[1], AVIO_FLAG_WRITE));
    check(avformat_write_header(output, nullptr));
    frame = av_frame_alloc(); packet = av_packet_alloc();
    if(!frame || !packet) throw std::bad_alloc();
    frame->format = codec->pix_fmt; frame->width = 64; frame->height = 48;
    check(av_frame_get_buffer(frame, 32));
    auto drain = [&] {
      for(;;) {
        const int received = avcodec_receive_packet(codec, packet);
        if(received == AVERROR(EAGAIN) || received == AVERROR_EOF) break;
        check(received);
        av_packet_rescale_ts(packet, codec->time_base, stream->time_base);
        packet->stream_index = stream->index;
        check(av_interleaved_write_frame(output, packet)); av_packet_unref(packet);
      }
    };
    for(int i = 0; i < 100; ++i) {
      check(av_frame_make_writable(frame));
      for(int y = 0; y < 48; ++y) std::memset(frame->data[0] + y * frame->linesize[0], 82, 64);
      for(int y = 0; y < 24; ++y) {
        std::memset(frame->data[1] + y * frame->linesize[1], 90, 32);
        std::memset(frame->data[2] + y * frame->linesize[2], 240, 32);
      }
      frame->pts = i;
      check(avcodec_send_frame(codec, frame)); drain();
    }
    check(avcodec_send_frame(codec, nullptr)); drain();
    check(av_write_trailer(output));
  } catch(const std::exception& error) { std::cerr << error.what() << '\n'; result = 1; }
  av_packet_free(&packet); av_frame_free(&frame); avcodec_free_context(&codec);
  if(output) { if(output->pb) avio_closep(&output->pb); avformat_free_context(output); }
  return result;
}
