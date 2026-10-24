#include <uav_shared_video/frame_ring.hpp>
#include "shm_layout.hpp"

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace uav::video {
namespace {
void checked(int error, const char* operation) {
  if(error) throw std::system_error(error, std::generic_category(), operation);
}
void validate_name(const std::string& name) {
  if(name.size() < 2 || name.size() > 180 || name.front() != '/' || name.find('/', 1) != std::string::npos)
    throw std::invalid_argument("shared memory name must be /name (2..180 bytes, no additional slash)");
  for(size_t i = 1; i < name.size(); ++i) {
    const char c = name[i];
    if(!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-'))
      throw std::invalid_argument("shared memory name permits letters, digits, underscore and hyphen");
  }
}
struct Mapping {
  int fd = -1;
  void* base = MAP_FAILED;
  size_t size = 0;
  ~Mapping() {
    if(base != MAP_FAILED) munmap(base, size);
    if(fd >= 0) close(fd);
  }
};
// A dead producer may leave a partially written slot: invalidate it before recovery.
bool lock_slot(detail::Slot& slot) {
  const int rc = pthread_mutex_trylock(&slot.mutex);
  if(rc == EBUSY) return false;
  if(rc == EOWNERDEAD) {
    slot.metadata.sequence = 0;
    checked(pthread_mutex_consistent(&slot.mutex), "pthread_mutex_consistent");
    return true;
  }
  checked(rc, "pthread_mutex_trylock");
  return true;
}
struct Unlock {
  pthread_mutex_t* mutex;
  ~Unlock() { pthread_mutex_unlock(mutex); }
};
}
uint32_t channels(PixelFormat format) {
  switch(format) {
    case PixelFormat::bgr8: case PixelFormat::rgb8: return 3;
    case PixelFormat::mono8: return 1;
  }
  throw std::invalid_argument("invalid pixel format");
}
std::string encoding(PixelFormat format) {
  switch(format) {
    case PixelFormat::bgr8: return "bgr8";
    case PixelFormat::rgb8: return "rgb8";
    case PixelFormat::mono8: return "mono8";
  }
  throw std::invalid_argument("invalid pixel format");
}
PixelFormat pixel_format(const std::string& name) {
  if(name == "bgr8") return PixelFormat::bgr8;
  if(name == "rgb8") return PixelFormat::rgb8;
  if(name == "mono8") return PixelFormat::mono8;
  throw std::invalid_argument("output_encoding must be bgr8, rgb8 or mono8");
}
struct FrameWriter::Impl {
  Mapping mapping;
  std::string name;
  detail::Header* header = nullptr;
  uint32_t next_slot = 0;
  uint64_t next_sequence = 0;
  ~Impl() {
    // Existing readers retain their mappings. New producers use a different name/session.
    if(!name.empty()) shm_unlink(name.c_str());
  }
};
FrameWriter::FrameWriter(const std::string& base, uint32_t slots, uint32_t capacity)
: impl_(std::make_unique<Impl>()) {
  validate_name(base);
  if(slots < 2 || slots > 32 || !capacity) throw std::invalid_argument("slots must be 2..32 and capacity positive");
  const uint64_t stride = detail::align64(sizeof(detail::Slot) + uint64_t(capacity));
  const uint64_t total = sizeof(detail::Header) + stride * slots;
  if(total > 2ULL * 1024 * 1024 * 1024) throw std::invalid_argument("shared memory allocation exceeds 2GiB");
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  const uint64_t session = uint64_t(ticks) ^ (uint64_t(getpid()) << 32);
  const std::string name = base + "_" + std::to_string(getpid()) + "_" + std::to_string(session);
  auto& map = impl_->mapping;
  map.fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0600);
  if(map.fd < 0) checked(errno, "shm_open producer");
  impl_->name = name;  // Unlink on every subsequent constructor failure too.
  if(ftruncate(map.fd, static_cast<off_t>(total)) < 0) checked(errno, "ftruncate shared memory");
  // Reserve tmpfs space now, avoiding SIGBUS later when /dev/shm is full.
  checked(posix_fallocate(map.fd, 0, static_cast<off_t>(total)), "reserve shared memory space");
  map.size = total;
  map.base = mmap(nullptr, map.size, PROT_READ | PROT_WRITE, MAP_SHARED, map.fd, 0);
  if(map.base == MAP_FAILED) checked(errno, "mmap producer");
  impl_->header = new(map.base) detail::Header{};
  auto& h = *impl_->header;
  h.magic_number = detail::magic; h.abi_version = detail::version; h.header_size = sizeof(h);
  h.session = session; h.mapped_size = total; h.slot_stride = stride; h.slot_count = slots; h.capacity = capacity;
  pthread_mutexattr_t attributes;
  checked(pthread_mutexattr_init(&attributes), "pthread_mutexattr_init");
  struct AttributesCleanup { pthread_mutexattr_t* p; ~AttributesCleanup() { pthread_mutexattr_destroy(p); } } cleanup{&attributes};
  checked(pthread_mutexattr_setpshared(&attributes, PTHREAD_PROCESS_SHARED), "process shared mutex");
  checked(pthread_mutexattr_setrobust(&attributes, PTHREAD_MUTEX_ROBUST), "robust mutex");
  for(uint32_t i = 0; i < slots; ++i) {
    auto* slot = new(detail::slot(map.base, h, i)) detail::Slot{};
    checked(pthread_mutex_init(&slot->mutex, &attributes), "pthread_mutex_init");
  }
}
FrameWriter::~FrameWriter() = default;
const std::string& FrameWriter::name() const { return impl_->name; }
uint64_t FrameWriter::session_id() const { return impl_->header->session; }
std::optional<FrameReference> FrameWriter::write(const uint8_t* data, size_t size, FrameMetadata metadata) {
  auto& h = *impl_->header;
  if(!data || !metadata.width || !metadata.height || metadata.step < uint64_t(metadata.width) * channels(metadata.format) ||
     uint64_t(metadata.step) * metadata.height != size || size > h.capacity)
    throw std::invalid_argument("invalid frame dimensions, stride or capacity");
  metadata.data_size = static_cast<uint32_t>(size);
  for(uint32_t attempt = 0; attempt < h.slot_count; ++attempt) {
    const uint32_t index = (impl_->next_slot + attempt) % h.slot_count;
    auto* slot = detail::slot(impl_->mapping.base, h, index);
    if(!lock_slot(*slot)) continue;
    Unlock unlock{&slot->mutex};
    slot->metadata.sequence = 0;
    std::memcpy(reinterpret_cast<uint8_t*>(slot) + sizeof(detail::Slot), data, size);
    metadata.sequence = ++impl_->next_sequence;
    slot->metadata = metadata;
    impl_->next_slot = (index + 1) % h.slot_count;
    return FrameReference{impl_->name, h.session, index, metadata};
  }
  return std::nullopt;
}
struct FrameReader::Impl {
  Mapping mapping;
  std::string name;
  detail::Header header{};  // Immutable layout snapshot, never trust mutable offsets.
};
FrameReader::FrameReader(const std::string& name) : impl_(std::make_unique<Impl>()) {
  // Names include the writer lifetime suffix, which may exceed base-name limit.
  if(name.size() < 2 || name.size() > 255 || name.front() != '/' || name.find('/',1) != std::string::npos)
    throw std::invalid_argument("invalid shared memory name");
  impl_->name = name;
  auto& m = impl_->mapping;
  // Readers need write permission solely for process-shared mutex operations.
  m.fd = shm_open(name.c_str(), O_RDWR | O_CLOEXEC, 0);
  if(m.fd < 0) checked(errno, "shm_open consumer");
  struct stat info{};
  if(fstat(m.fd, &info) < 0) checked(errno, "fstat shared memory");
  if(info.st_size < static_cast<off_t>(sizeof(detail::Header)) || info.st_size > 2LL * 1024 * 1024 * 1024)
    throw std::runtime_error("invalid shared memory size");
  m.size = info.st_size;
  m.base = mmap(nullptr, m.size, PROT_READ | PROT_WRITE, MAP_SHARED, m.fd, 0);
  if(m.base == MAP_FAILED) checked(errno, "mmap consumer");
  std::memcpy(&impl_->header, m.base, sizeof(detail::Header));
  const auto& h = impl_->header;
  if(h.magic_number != detail::magic || h.abi_version != detail::version || h.header_size != sizeof(h) ||
     h.slot_count < 2 || h.slot_count > 32 || !h.capacity || h.mapped_size != m.size ||
     h.slot_stride != detail::align64(sizeof(detail::Slot) + uint64_t(h.capacity)) ||
     uint64_t(h.header_size) + h.slot_stride * h.slot_count != m.size)
    throw std::runtime_error("incompatible or corrupt frame ring ABI");
}
FrameReader::~FrameReader() = default;
bool FrameReader::read(const FrameReference& ref, Frame& frame) {
  const auto& h = impl_->header;
  if(ref.shm_name != impl_->name || ref.session_id != h.session || ref.slot >= h.slot_count || !ref.metadata.sequence)
    return false;
  auto* slot = detail::slot(impl_->mapping.base, h, ref.slot);
  if(!lock_slot(*slot)) return false;
  Unlock unlock{&slot->mutex};
  auto metadata = slot->metadata;
  if(metadata.sequence != ref.metadata.sequence) return false;
  if(!metadata.width || !metadata.height || metadata.data_size > h.capacity ||
     metadata.step < uint64_t(metadata.width) * channels(metadata.format) ||
     uint64_t(metadata.step) * metadata.height != metadata.data_size)
    throw std::runtime_error("invalid shared frame metadata");
  frame.pixels.resize(metadata.data_size);
  std::memcpy(frame.pixels.data(), reinterpret_cast<const uint8_t*>(slot) + sizeof(detail::Slot), metadata.data_size);
  frame.metadata = metadata;
  return true;
}
}  // namespace uav::video
