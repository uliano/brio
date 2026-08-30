// Family smoke TU for samc/mtb.hpp: every verb must COMPILE on the E, G
// and J 18A headers (tools/check_samc.sh sweeps all three).
//
// The MTB is core-private: one instance, one register map, identical on
// every member of the family (and on every Cortex-M0+ that has one). So
// what this fixture pins is the BUFFER GEOMETRY arithmetic - MASTER.MASK
// is log2(bytes) - 4 and nothing in the data sheet says so, the device
// header being the only local authority - and the packet decode.

#include <stdint.h>
#include <span>

#include "samc/mtb.hpp"

using namespace brio;

static_assert(Mtb::pac_id == 36, "the id table 12-3 leaves blank");
static_assert(Mtb::packet_bytes == 8);

// The event users, and the disagreement they carry: table 12-3 numbers
// them 44 and 45, the device header 45 and 46. These are the header's.
static_assert(Mtb::ev_user_start == 45);
static_assert(Mtb::ev_user_stop == 46);
static_assert(Mtb::ev_user_stop == Mtb::ev_user_start + 1);

// MASK spans 16 bytes at 0 and doubles from there.
static_assert(Mtb::mask_for(16) == 0);
static_assert(Mtb::mask_for(32) == 1);
static_assert(Mtb::mask_for(1024) == 6);
static_assert(Mtb::mask_for(32768) == 11, "the whole SRAM of the 18A");
static_assert(Mtb::size_for(0) == 16);
static_assert(Mtb::size_for(6) == 1024);
static_assert(Mtb::size_for(Mtb::mask_for(4096)) == 4096);

// Anything that is not a power of two of at least 16 bytes has no MASK,
// and 32 is the impossible value the configuration check refuses.
static_assert(Mtb::mask_for(0) == 32);
static_assert(Mtb::mask_for(8) == 32);
static_assert(Mtb::mask_for(1000) == 32);
static_assert(Mtb::size_for(32) == 0);

static_assert(Mtb::packets_for(1024) == 128);

// Bit 0 of each word is a flag and not part of the address.
static_assert(MtbPacket{0x00001235UL, 0x00002460UL}.source() == 0x00001234UL);
static_assert(MtbPacket{0x00001235UL, 0x00002460UL}.destination() == 0x00002460UL);
static_assert(MtbPacket{0x00001235UL, 0x00002460UL}.source_flag());
static_assert(!MtbPacket{0x00001235UL, 0x00002460UL}.destination_flag());

alignas(1024) uint32_t trace_buffer[256];

void verbs() {
    (void)Mtb::sram_base();
    (void)Mtb::buffer_valid(trace_buffer, sizeof(trace_buffer));

    constexpr MtbConfig cfg{
        .start_on_event = true,
        .stop_on_event = true,
        .auto_stop = true,
        .auto_halt = false,
        .registers_privileged = false,
        .ram_privileged = false,
        .watermark_packets = 64,
    };
    (void)Mtb::configure(trace_buffer, sizeof(trace_buffer), cfg);
    (void)Mtb::configure(trace_buffer, sizeof(trace_buffer));

    Mtb::enable(true);
    (void)Mtb::enabled();
    Mtb::enable(false);

    (void)Mtb::master();
    (void)Mtb::flow();
    (void)Mtb::position();
    (void)Mtb::mask();
    (void)Mtb::write_offset();
    (void)Mtb::wrapped();
    (void)Mtb::packets_written(trace_buffer, sizeof(trace_buffer));

    const MtbPacket p = Mtb::packet(trace_buffer, 0);
    (void)p.source();
    (void)p.destination();

    Mtb::release();
    (void)Mtb::regs().MTB_BASE;
}

// The post-mortem pair: the freeze that must come first, and the
// bounded oldest-first copy of the tail.
void tail() {
    (void)Mtb::freeze();

    MtbPacket kept[8]{};
    (void)Mtb::snapshot(trace_buffer, sizeof(trace_buffer),
                        std::span<MtbPacket>(kept, 8));
    (void)Mtb::snapshot(trace_buffer, sizeof(trace_buffer),
                        std::span<MtbPacket>(kept, 0));
}
