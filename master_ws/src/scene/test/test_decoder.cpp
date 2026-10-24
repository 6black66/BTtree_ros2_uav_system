#include <video_gateway_ros2/video_decoder.hpp>
#include <iostream>
#include <stdexcept>
using namespace uav::scene;
void check(bool ok, const char* why) { if(!ok) throw std::runtime_error(why); }
int main(int argc, char** argv) {
  if(argc != 2 && argc != 3) return 2;
  try {
    std::atomic<bool> stop{false};
    if(argc == 3 && std::string(argv[2]) == "--probe") {
      DecoderConfig sample; sample.input_type = "file"; sample.source_url = argv[1];
      VideoDecoder probe(sample, stop); probe.open(); DecodedFrame image;
      for(int i = 0; i < 10; ++i) {
        check(probe.next(image), "recorded sample decode");
        check(image.pixels.size() == uint64_t(image.metadata.step) * image.metadata.height, "recorded frame dimensions");
      }
      std::cout << "Recorded sample decoded: " << image.metadata.width << 'x' << image.metadata.height << " BGR8, 10 frames\n";
      return 0;
    }
    DecoderConfig config; config.input_type = "file"; config.source_url = argv[1]; config.max_width = 64; config.max_height = 48;
    VideoDecoder decoder(config, stop); decoder.open();
    DecodedFrame frame; int count = 0;
    while(decoder.next(frame)) {
      check(frame.metadata.width == 64 && frame.metadata.height == 48 && frame.metadata.step == 192, "dimensions");
      check(frame.pixels.size() == 9216, "pixel size");
      check(frame.pixels[0] < 10 && frame.pixels[1] < 10 && frame.pixels[2] > 240, "BGR red pixel conversion");
      check(frame.metadata.source_pts_valid, "source PTS");
      ++count;
    }
    check(count == 100, "all frames, including delayed decoder frames, are drained");
    config.format = uav::video::PixelFormat::mono8;
    VideoDecoder gray(config, stop); gray.open(); check(gray.next(frame) && frame.pixels.size() == 3072, "mono conversion");
    config.max_width = 32;
    bool failed = false;
    try { VideoDecoder small(config, stop); small.open(); small.next(frame); } catch(const std::runtime_error&) { failed = true; }
    check(failed, "capacity limit enforced");
    std::cout << "Decoder checks passed (100 MPEG-TS frames, pixel colors, PTS, mono, capacity)\n";
  } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
