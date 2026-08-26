// Flash block allocator family smoke TU: every package must compile
// this (instantiation only - nothing here is ever executed).
//
// The allocator itself (util/nv_heap.hpp) is target-independent and is
// tested on the host; what has to hold on every package is the BACKEND -
// avrdx/nvm_flash.hpp - and the fact that a heap over it instantiates
// with the same geometry everywhere. NVMCTRL is byte-identical across
// all eight AVR128DA/DB packages and the flash geometry is the same
// 128 KB in 512-byte pages, so there are no package tiers here: what
// this TU proves is that the media satisfies the concept, that the
// derived constants come out right, and that every verb of the heap
// compiles against a real target.
#include "avrdx/nvm_flash.hpp"
#include "util/nv_heap.hpp"

using namespace brio;

// The media IS the flash of this family, and its granularities are the
// device header's - not a copy of them.
static_assert(FlashMedia<NvmFlash>);
static_assert(NvmFlash::erase_size == PROGMEM_PAGE_SIZE);
static_assert(NvmFlash::write_cell == 2);
static_assert(NvmFlash::flash_end == PROGMEM_SIZE);
static_assert(NvmFlash::zone_count == 2);

using Heap = NvHeap<NvmFlash, 8, 2>;

// The map home is the last two pages of the part, an address anchored to
// the silicon and not to the linker.
static_assert(Heap::map_home == PROGMEM_SIZE - 2u * PROGMEM_PAGE_SIZE);
static_assert(Heap::map_bytes == 16 + 14 * 8 + 2);
static_assert(Heap::map_image_bytes == 130);        // already even: no padding
static_assert(Heap::map_image_bytes <= PROGMEM_PAGE_SIZE);

// A wider heap still fits one page on this media, and a narrower one
// pads its map image up to the program unit.
static_assert(NvHeap<NvmFlash, 35>::map_image_bytes <= PROGMEM_PAGE_SIZE);
static_assert(NvHeap<NvmFlash, 1>::map_image_bytes == 32);

volatile uint8_t heap_sink;
volatile uint32_t heap_sink32;

Heap heap;
uint8_t heap_buf[64];

void nv_heap_mount() {
    const Heap::MountReport& r = heap.mount();
    heap_sink = static_cast<uint8_t>(r.status);
    heap_sink = r.survivors;
    heap_sink = r.lost;
    heap_sink32 = r.seq;
    heap_sink32 = r.build_id;
    heap_sink = static_cast<uint8_t>(r.mounted());
    for (uint8_t i = 0; i < r.survivors; ++i) {
        heap_sink = static_cast<uint8_t>(r.survivor_ids[i]);
    }
    for (uint8_t i = 0; i < r.lost; ++i) {
        heap_sink = static_cast<uint8_t>(r.lost_ids[i]);
    }
    heap_sink = static_cast<uint8_t>(heap.mounted());
    heap_sink32 = heap.sequence();
    heap_sink = heap.map_page();
    heap_sink = heap.count();
    if (heap.count() != 0) {
        heap_sink = static_cast<uint8_t>(heap.entry(0).size_pages);
    }
    heap_sink = static_cast<uint8_t>(heap.free_pages(0));
    heap_sink32 = heap.zone(1).floor;
}

void nv_heap_write() {
    if (std::optional<Heap::Writer> w = heap.alloc(7, 600)) {
        heap_sink32 = w->address();
        heap_sink32 = w->capacity();
        heap_sink32 = w->written();
        heap_sink = static_cast<uint8_t>(w->record_id());
        heap_sink = static_cast<uint8_t>(w->failed());
        heap_sink = static_cast<uint8_t>(
            w->append(std::span<const uint8_t>(heap_buf, sizeof heap_buf)));
        heap_sink = static_cast<uint8_t>(w->seal());
        heap_sink = static_cast<uint8_t>(w->sealed());
    }
    if (std::optional<Heap::Writer> w = heap.rewrite(7)) {
        heap_sink = static_cast<uint8_t>(
            w->append(std::span<const uint8_t>(heap_buf, 3)));
        heap_sink = static_cast<uint8_t>(w->seal());
    }
}

void nv_heap_read() {
    if (const std::optional<NvBlock<NvmFlash>> b = heap.find(7)) {
        heap_sink32 = b->address;
        heap_sink32 = b->length;
        heap_sink = static_cast<uint8_t>(
            b->read(0, std::span<uint8_t>(heap_buf, sizeof heap_buf)));
    }
}

// The media's own verbs, in the shape the heap calls them.
void nv_heap_media() {
    const std::array<FlashZone, 2> z = NvmFlash::zones();
    heap_sink32 = z[0].ceiling;
    heap_sink32 = z[0].floor;
    heap_sink32 = z[1].size();
    heap_sink = static_cast<uint8_t>(z[1].empty());
    NvmFlash::read(0x18000u, std::span<uint8_t>(heap_buf, sizeof heap_buf));
    heap_sink = static_cast<uint8_t>(
        NvmFlash::program(0x10000u, std::span<const uint8_t>(heap_buf, 4)));
    heap_sink = static_cast<uint8_t>(NvmFlash::erase(0x10000u));
    heap_sink32 = NvmFlash::build_id();
}
