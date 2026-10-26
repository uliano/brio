// A writer whose stage is longer than 255 bytes does not compile, however
// long the transport's ring: a stage's length is a byte. (The same
// writer with `.long_size = 255` compiles.)
// refused: a stage's length is a byte: at most 255
#include <span>
#include "host/platform.hpp"
#include "util/line_writer.hpp"

struct LongRing {
    static constexpr uint32_t tx_capacity() { return 1024; }
    static uint32_t tx_room() { return 1024; }
    static bool write_byte(uint8_t) { return true; }
    static uint32_t write_bulk(std::span<const uint8_t> run) { return static_cast<uint32_t>(run.size()); }
};

using W = brio::LineWriter<LongRing, brio::HostPlatform, brio::LineWriterOptions{.long_size = 300}>;

void f() { brio::print(W{}, "a line", brio::crlf); }
