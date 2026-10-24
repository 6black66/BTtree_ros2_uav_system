#pragma once
#include <uav_shared_video/frame_ring.hpp>
#include <pthread.h>
#include <cstdint>

namespace uav::video::detail {
constexpr uint64_t magic = 0x5541564652414D45ULL;
constexpr uint32_t version = 1;
struct alignas(64) Header {
  uint64_t magic_number;
  uint32_t abi_version;
  uint32_t header_size;
  uint64_t session;
  uint64_t mapped_size;
  uint64_t slot_stride;
  uint32_t slot_count;
  uint32_t capacity;
};
struct alignas(64) Slot {
  pthread_mutex_t mutex;
  FrameMetadata metadata;
};
inline uint64_t align64(uint64_t value) { return (value + 63) & ~uint64_t(63); }
inline Slot* slot(void* base, const Header& header, uint32_t index) {
  return reinterpret_cast<Slot*>(static_cast<uint8_t*>(base) + header.header_size + index * header.slot_stride);
}
}  // namespace uav::video::detail
