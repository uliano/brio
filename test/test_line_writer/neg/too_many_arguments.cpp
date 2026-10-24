// A print whose packed arguments exceed the writer's message does not
// compile: the default slot of 32 bytes holds 16 of arguments on the host
// (a 64-bit pointer and the four bytes of fields before it, padded to
// eight), and a text packs nine bytes - its pointer and its length - so
// three of them are 27.
// refused: these arguments do not fit one message
#include <span>
#include "host/platform.hpp"
#include "util/line_writer.hpp"

struct Sink {
    static constexpr uint32_t tx_capacity() { return 255; }
    static uint32_t tx_room() { return 255; }
    static bool write_byte(uint8_t) { return true; }
    static bool tx_drained() { return false; }
};

using W = brio::LineWriter<Sink, brio::HostPlatform>;

void f() { (void)brio::print(W{}, "one", "two", "three", brio::crlf); }
