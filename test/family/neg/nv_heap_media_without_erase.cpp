// mcu: avr128db48 avr128da28
// The FlashMedia contract is a concept, not a comment: a media that
// cannot erase cannot back a heap (every map version and every block
// starts with an erase), and the heap must say so at the point of
// declaration rather than fail somewhere inside a template.
#include <array>
#include <span>
#include "util/nv_heap.hpp"
using namespace brio;

struct HalfMedia {
    static constexpr uint32_t erase_size = 512;
    static constexpr uint32_t write_cell = 2;
    static constexpr uint32_t flash_end = 0x20000u;
    static constexpr uint8_t zone_count = 1;
    static std::array<FlashZone, 1> zones() { return {}; }
    static void read(uint32_t, std::span<uint8_t>) {}
    static bool program(uint32_t, std::span<const uint8_t>) { return false; }
    static uint32_t build_id() { return 0; }
    // erase() deliberately missing.
};

NvHeap<HalfMedia> heap;
