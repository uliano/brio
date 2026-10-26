// A writer whose stage is longer than its transport's ring does not
// compile: a source that may wait spins until its whole line fits the
// room, and a ring of 64 bytes never holds a line of the default long
// stage's 192. (The same writer with `.long_size = 64, .small_size = 32`
// compiles.)
// refused: a stage larger than the transport's ring could never be written whole
#include <span>
#include "host/platform.hpp"
#include "util/line_writer.hpp"

struct ShortRing {
    static constexpr uint32_t tx_capacity() { return 64; }
    static uint32_t tx_room() { return 64; }
    static bool write_byte(uint8_t) { return true; }
    static uint32_t write_bulk(std::span<const uint8_t> run) { return static_cast<uint32_t>(run.size()); }
};

using W = brio::LineWriter<ShortRing, brio::HostPlatform>;

void f() { brio::print(W{}, "a line", brio::crlf); }
