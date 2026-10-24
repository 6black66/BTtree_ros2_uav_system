#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace uav::video {

// Native ABI for local Linux processes using the same library. No ROS dependency.
enum class PixelFormat : uint32_t { bgr8 = 1, rgb8 = 2, mono8 = 3 };
uint32_t channels(PixelFormat format);
std::string encoding(PixelFormat format);
PixelFormat pixel_format(const std::string& name);

struct FrameMetadata {
  uint64_t sequence = 0;
  int64_t received_stamp_ns = 0;  // Producer ROS clock, not camera capture time.
  int64_t source_pts_ns = 0;      // Stream-relative presentation time, when valid.
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t step = 0;
  uint32_t data_size = 0;
  PixelFormat format = PixelFormat::bgr8;
  uint32_t source_pts_valid = 0;
};

struct FrameReference {
  std::string shm_name;
  uint64_t session_id = 0;
  uint32_t slot = 0;
  FrameMetadata metadata;
};

struct Frame {
  FrameMetadata metadata;
  std::vector<uint8_t> pixels;
};

class FrameWriter {
public:
  // A unique suffix identifies this producer lifetime. The actual name is in references.
  FrameWriter(const std::string& base_name, uint32_t slots, uint32_t capacity_bytes);
  ~FrameWriter();
  FrameWriter(const FrameWriter&) = delete;
  FrameWriter& operator=(const FrameWriter&) = delete;
  // Never waits for slow consumers; returns nullopt if every slot is busy.
  std::optional<FrameReference> write(const uint8_t* data, size_t size, FrameMetadata metadata);
  const std::string& name() const;
  uint64_t session_id() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class FrameReader {
public:
  explicit FrameReader(const std::string& shm_name);
  ~FrameReader();
  FrameReader(const FrameReader&) = delete;
  FrameReader& operator=(const FrameReader&) = delete;
  // Copies under a process-shared mutex. False means busy, overwritten, or wrong session.
  bool read(const FrameReference& reference, Frame& frame);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace uav::video
