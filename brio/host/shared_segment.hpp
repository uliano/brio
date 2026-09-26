/*
 * shared_segment.hpp (host)
 *
 * A named region of memory two processes can both reach, owned by one of
 * them. Everything the simulator shares with a viewer goes through this
 * - the framebuffer one way, the panel's inputs the other - so the rules
 * that make it portable are written once here instead of twice.
 *
 * THE NAME IS THE CONTRACT, NEVER A PATH. Linux exposes these objects
 * under /dev/shm and macOS exposes them nowhere in the filesystem, so a
 * viewer attaches with shm_open by the same name and the path is a
 * debugging convenience on one platform only. Two more rules come from
 * there: a name is SHORT, because macOS caps the whole of it at 31
 * characters; and a segment is SIZED ONCE at creation, because a second
 * ftruncate fails there - so a change of geometry is a new segment and
 * never a resize.
 *
 * THE OWNER IS THE PROGRAM, NOT THE VIEWER, in both directions. A viewer
 * comes and goes; the program does not, and a segment that exists as
 * long as the program does removes every question about what happens
 * when no one is watching. The input segment then simply reads as
 * "nothing pressed" until a viewer writes into it.
 *
 * THE BOOT ID IS LOAD-BEARING. After a segment is unlinked and remade
 * under the same name, a viewer's old mapping still points at the old
 * object, which stays alive as long as it is referenced: a counter
 * INSIDE that object could never tell it anything had changed. So every
 * segment carries an id drawn afresh at creation, and a viewer that
 * re-opens by name and finds a different one knows to remap.
 *
 * THE BYTE COUNT IS IN THE HEADER, NOT IN THE OBJECT. What fstat reports
 * for a shared object is exact on Linux and rounded up to a page on
 * macOS (16 KB on Apple Silicon), so a viewer that sized itself from the
 * object would read a different count on each. A segment's size is
 * therefore known from its LAYOUT - the framebuffer's from its header,
 * the panel's a constant - the object's size only bounds what a viewer
 * may map, and a header claiming more than the object holds is another
 * layout's, not ours to read.
 *
 * Host only, and failures throw: a bench tool that cannot map its own
 * memory has nothing useful left to do (docs/host/simulator.md).
 */

#pragma once

#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <random>
#include <stdexcept>
#include <string>

namespace brio {

/// The longest a name may be, whole: macOS's cap, obeyed everywhere so
/// that what works here works there.
inline constexpr size_t shared_segment_name_max = 31;

/// An id no two creations share, so a viewer can tell one segment from
/// its replacement under the same name.
inline uint64_t fresh_boot_id() {
    std::random_device rd;
    return (static_cast<uint64_t>(rd()) << 32) ^ static_cast<uint64_t>(rd());
}

/**
 * Creates a segment, owns it, unlinks it when it goes. Move-only in
 * spirit and not copyable at all: two owners of one name would each
 * unlink it.
 */
class SharedSegment {
public:
    SharedSegment(const std::string& prefix, const std::string& name,
                  size_t bytes)
        : name_(prefix + name), bytes_(bytes) {
        if (name_.size() > shared_segment_name_max) {
            throw std::runtime_error("shared segment name too long: " + name_);
        }
        // A stale segment from a crash would refuse the one ftruncate
        // macOS allows, so start from nothing every time.
        ::shm_unlink(name_.c_str());
        fd_ = ::shm_open(name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
        if (fd_ < 0) {
            throw std::runtime_error("shm_open failed for " + name_);
        }
        if (::ftruncate(fd_, static_cast<off_t>(bytes_)) != 0) {
            drop();
            throw std::runtime_error("ftruncate failed for " + name_);
        }
        void* base =
            ::mmap(nullptr, bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (base == MAP_FAILED) {
            drop();
            throw std::runtime_error("mmap failed for " + name_);
        }
        base_ = static_cast<uint8_t*>(base);
        memset(base_, 0, bytes_);
    }

    ~SharedSegment() {
        if (base_ != nullptr) {
            ::munmap(base_, bytes_);
            base_ = nullptr;
        }
        drop();
    }

    SharedSegment(const SharedSegment&) = delete;
    SharedSegment& operator=(const SharedSegment&) = delete;

    uint8_t* base() { return base_; }
    const uint8_t* base() const { return base_; }
    size_t bytes() const { return bytes_; }

    /// The whole name, prefix included - what a viewer opens.
    const std::string& name() const { return name_; }

private:
    void drop() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        if (!name_.empty()) {
            ::shm_unlink(name_.c_str());
        }
    }

    std::string name_;
    size_t bytes_;
    int fd_{-1};
    uint8_t* base_{nullptr};
};

} // namespace brio
