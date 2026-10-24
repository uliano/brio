// A print whose bound the transport's ring could never hold does not
// compile: a ring of 16 bytes and a line of two 32-bit integers, eleven
// bytes each at their worst, would wait for ever.
// refused: this line's bound exceeds what the transport's ring holds
#include <span>
#include "host/platform.hpp"
#include "util/line_writer.hpp"

struct ShortRing {
    static constexpr uint32_t tx_capacity() { return 16; }
    static uint32_t tx_room() { return 16; }
    static bool write_byte(uint8_t) { return true; }
    static bool tx_drained() { return false; }
};

using W = brio::LineWriter<ShortRing, brio::HostPlatform>;

void f() { (void)brio::print(W{}, int32_t{1}, int32_t{2}, brio::crlf); }
