#include <uav_shared_video/frame_ring.hpp>
#include "shm_layout.hpp"
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <stdexcept>
#include <functional>

using namespace uav::video;
void check(bool value, const char* why) { if(!value) throw std::runtime_error(why); }
void rejects(const std::function<void()>& action) {
  try { action(); } catch(const std::exception&) { return; }
  throw std::runtime_error("expected failure");
}
int main() {
  try {
    rejects([] { FrameWriter bad("invalid", 3, 12); });
    rejects([] { FrameWriter bad("/invalid", 1, 12); });
    std::string name;
    {
      FrameWriter writer("/uav_ring_test", 2, 12);
      name = writer.name();
      FrameReader reader(name);
      FrameMetadata metadata;
      metadata.width = 2; metadata.height = 2; metadata.step = 6; metadata.received_stamp_ns = 1234;
      std::vector<uint8_t> pixels(12, 42);
      auto first = writer.write(pixels.data(), pixels.size(), metadata);
      check(first.has_value(), "first frame");
      Frame frame;
      check(reader.read(*first, frame) && frame.pixels == pixels && frame.metadata.received_stamp_ns == 1234, "pixel/metadata roundtrip");
      auto wrong = *first; ++wrong.session_id;
      check(!reader.read(wrong, frame), "session check");
      wrong = *first; wrong.slot = 100;
      check(!reader.read(wrong, frame), "slot bounds");
      const pid_t child = fork();
      check(child >= 0, "fork");
      if(child == 0) {
        try {
          FrameReader other(name); Frame out;
          _exit(other.read(*first, out) && out.pixels == pixels ? 0 : 1);
        } catch(...) { _exit(2); }
      }
      int status = 0; waitpid(child, &status, 0);
      check(WIFEXITED(status) && WEXITSTATUS(status) == 0, "cross process consumer");
      writer.write(pixels.data(), pixels.size(), metadata);
      writer.write(pixels.data(), pixels.size(), metadata);
      check(!reader.read(*first, frame), "overwritten reference rejected");
      rejects([&] { writer.write(pixels.data(), 11, metadata); });
      // Simulate a consumer dying while holding a process-shared slot lock.
      const int fd = shm_open(name.c_str(), O_RDWR, 0);
      struct stat info{}; fstat(fd, &info);
      void* mapped = mmap(nullptr, info.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
      check(mapped != MAP_FAILED, "map test ring");
      auto& header = *static_cast<detail::Header*>(mapped);
      auto* slot = detail::slot(mapped, header, 0);
      auto* other = detail::slot(mapped, header, 1);
      // Both slots busy: producer must skip immediately instead of blocking.
      check(pthread_mutex_lock(&slot->mutex) == 0, "lock first slot");
      check(pthread_mutex_lock(&other->mutex) == 0, "lock second slot");
      check(!writer.write(pixels.data(), pixels.size(), metadata), "nonblocking producer with slow consumers");
      pthread_mutex_unlock(&other->mutex); pthread_mutex_unlock(&slot->mutex);
      const pid_t owner = fork();
      check(owner >= 0, "fork lock owner");
      if(owner == 0) { _exit(pthread_mutex_lock(&slot->mutex) == 0 ? 0 : 1); }
      waitpid(owner, &status, 0);
      check(WIFEXITED(status) && WEXITSTATUS(status) == 0, "dead consumer acquired lock");
      // Next write starts at slot 1; the following one recovers dead slot 0.
      check(writer.write(pixels.data(), pixels.size(), metadata).has_value(), "write after owner death");
      auto recovered = writer.write(pixels.data(), pixels.size(), metadata);
      check(recovered && reader.read(*recovered, frame) && frame.pixels == pixels, "robust mutex recovery");
      munmap(mapped, info.st_size); close(fd);
    }
    check(shm_open(name.c_str(), O_RDWR, 0) == -1, "producer unlinks on exit");
    std::cout << "Frame ring checks passed\n";
  } catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
