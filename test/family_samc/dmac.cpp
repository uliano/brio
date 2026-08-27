// DMAC family smoke TU: the block resource, the channel above it, the
// descriptor builder between them, and the two peripheral engines.
//
// NOTHING ABOUT THE DMAC VARIES ACROSS THIS FAMILY - twelve channels and
// four arbitration levels on the E, G and J alike, one AHB mask bit, one
// NVIC line, and a register set that is byte-identical in all three
// device headers. So this TU is not about package gating (the neg/ TUs
// state the one refusal there is, a channel index past DMAC_CH_NUM); it
// is where THE END-ADDRESS ARITHMETIC IS PROVEN WITHOUT A CHIP.
//
// That arithmetic is the peripheral's one trap: for an incrementing side
// the descriptor's address field holds one beat PAST the last, not the
// start (DS60001479M 25.6.2.7), and the two halves of the data sheet do
// not print the same formula for it (25.10.3 adds a stray "+ 1"; see the
// driver's file header). Every static_assert below is a claim the bench
// suite then re-proves by moving real bytes and looking at where they
// landed.
#include "samc/dmac.hpp"

using namespace brio;

// ---- what the device header says --------------------------------------------
static_assert(Dmac::channel_count == 12, "every SAM C21 variant has twelve channels");
static_assert(Dmac::channel_count == DMAC_CH_NUM, "the device header is the authority");
static_assert(Dmac::level_count == 4);
static_assert(Dmac::irq() == DMAC_IRQn);

// The descriptor is 128 bits and mirrors the device header's own type
// (the driver static_asserts the field offsets; this restates the size
// because 25.6.2.3's "Size = 128 bits x (m + 1)" is what sizes the tables).
static_assert(sizeof(DmaDescriptor) == 16);
static_assert(sizeof(dmac_descriptor_registers_t) == 16);

// ---- beat widths and step factors -------------------------------------------
// The BEATSIZE code is not the width: 0/1/2 mean 1/2/4 bytes. Reading it
// as a byte count is exactly the mistake that makes 25.10.3's "+ 1" look
// as though it meant something.
static_assert(dma_beat_bytes(DmaBeat::byte) == 1);
static_assert(dma_beat_bytes(DmaBeat::hword) == 2);
static_assert(dma_beat_bytes(DmaBeat::word) == 4);
static_assert(dma_step_factor(DmaStep::x1) == 1);
static_assert(dma_step_factor(DmaStep::x2) == 2);
static_assert(dma_step_factor(DmaStep::x128) == 128);

// ---- THE END-ADDRESS QUIRK ---------------------------------------------------
// An incrementing side ends at start + beats x beat_bytes x step.
static_assert(dma_end_address(0x20000100, 16, DmaBeat::byte, true, 1) == 0x20000110);
static_assert(dma_end_address(0x20000100, 16, DmaBeat::hword, true, 1) == 0x20000120);
static_assert(dma_end_address(0x20000100, 16, DmaBeat::word, true, 1) == 0x20000140);
static_assert(dma_end_address(0x20000100, 1, DmaBeat::byte, true, 1) == 0x20000101);
// A step multiplies the stride, and therefore the end.
static_assert(dma_end_address(0x20000100, 8, DmaBeat::byte, true, 4) == 0x20000120);
// A STATIC side is the plain address - a peripheral's DATA register does
// not move, and adding a length to it would aim the transfer at whatever
// register sits above it.
static_assert(dma_end_address(0x42001828, 16, DmaBeat::byte, false, 1) == 0x42001828);
static_assert(dma_end_address(0x42001828, 16, DmaBeat::word, false, 8) == 0x42001828);
// Zero beats: nothing to add, whichever way the side is configured.
static_assert(dma_end_address(0x20000100, 0, DmaBeat::word, true, 8) == 0x20000100);
// The width is named: 65535 word beats at a x128 step is past 2^24 and
// the arithmetic must not have wrapped on the way.
static_assert(dma_end_address(0x20000000, 65535, DmaBeat::word, true, 128) ==
              0x20000000UL + 65535UL * 4UL * 128UL);

// ---- the control word --------------------------------------------------------
constexpr DmaTransfer m2m_shape{
    .beats = 32,
    .beat = DmaBeat::byte,
    .source_increment = true,
    .destination_increment = true,
};
static_assert((dma_btctrl(m2m_shape) & DMAC_BTCTRL_VALID_Msk) != 0);
static_assert((dma_btctrl(m2m_shape) & DMAC_BTCTRL_SRCINC_Msk) != 0);
static_assert((dma_btctrl(m2m_shape) & DMAC_BTCTRL_DSTINC_Msk) != 0);
static_assert((dma_btctrl(m2m_shape) & DMAC_BTCTRL_BEATSIZE_Msk) ==
              DMAC_BTCTRL_BEATSIZE_BYTE);
// The default block action is INT, not NOACT - with NOACT the TCMPL flag
// is not raised at all (25.8.20), so a completion nobody can observe is
// what a defaulted-none would have shipped.
static_assert((dma_btctrl(m2m_shape) & DMAC_BTCTRL_BLOCKACT_Msk) ==
              DMAC_BTCTRL_BLOCKACT_INT);
static_assert((dma_btctrl(m2m_shape) & DMAC_BTCTRL_EVOSEL_Msk) ==
              DMAC_BTCTRL_EVOSEL_DISABLE);
static_assert((dma_btctrl({.beats = 1, .valid = false}) & DMAC_BTCTRL_VALID_Msk) == 0);
// STEPSEL names the side the step applies to; the other always moves one
// beat (25.10.1).
static_assert((dma_btctrl({.beats = 1, .step_side = DmaStepSide::source}) &
               DMAC_BTCTRL_STEPSEL_Msk) == DMAC_BTCTRL_STEPSEL_SRC);
static_assert((dma_btctrl({.beats = 1, .step_side = DmaStepSide::destination}) &
               DMAC_BTCTRL_STEPSEL_Msk) == DMAC_BTCTRL_STEPSEL_DST);

// ---- whole descriptors, at compile time --------------------------------------
// Memory to memory: both ends walk, both end addresses are start + length.
constexpr DmaDescriptor m2m = dma_descriptor_at(0x20000100, 0x20000200, m2m_shape);
static_assert(m2m.btcnt == 32);
static_assert(m2m.srcaddr == 0x20000120);
static_assert(m2m.dstaddr == 0x20000220);
static_assert(m2m.descaddr == 0);
static_assert(m2m.valid_bit());

// Memory to peripheral (the TX engine's shape): the source walks, the
// destination is the fixed DATA register.
constexpr DmaTransfer tx_shape{
    .beats = 64,
    .beat = DmaBeat::byte,
    .source_increment = true,
    .destination_increment = false,
};
constexpr DmaDescriptor tx = dma_descriptor_at(0x20000400, 0x42001828, tx_shape);
static_assert(tx.srcaddr == 0x20000440);
static_assert(tx.dstaddr == 0x42001828);

// Peripheral to memory (the RX engine's shape): the mirror image.
constexpr DmaTransfer rx_shape{
    .beats = 64,
    .beat = DmaBeat::byte,
    .source_increment = false,
    .destination_increment = true,
};
constexpr DmaDescriptor rx = dma_descriptor_at(0x42001828, 0x20000400, rx_shape);
static_assert(rx.srcaddr == 0x42001828);
static_assert(rx.dstaddr == 0x20000440);

// A shape that describes nothing yields an invalid descriptor, not a
// word aimed somewhere.
static_assert(dma_descriptor_at(0x20000100, 0x20000200, {.beats = 0}) == DmaDescriptor{});
static_assert(!dma_descriptor_at(0x20000100, 0x20000200, {.beats = 0}).valid_bit());
static_assert(!dma_transfer_valid({.beats = 0}));
static_assert(!dma_transfer_valid({.destination = nullptr, .beats = 4}));

// The invariant half of BTCTRL is what a write-back must still match:
// VALID is the one bit the controller legitimately clears.
static_assert(m2m.invariant_control() == (m2m.btctrl & ~DMAC_BTCTRL_VALID_Msk));
static_assert(m2m.invariant_control() != m2m.btctrl);

// ---- trigger sources ---------------------------------------------------------
// Read off the device header per instance, never computed - the same rule
// the GCLK channel ids follow in samc/clock.hpp.
static_assert(dma_trigger_none == 0);
static_assert(dma_trigger_sercom_rx<0>() == SERCOM0_DMAC_ID_RX);
static_assert(dma_trigger_sercom_tx<0>() == SERCOM0_DMAC_ID_TX);
static_assert(dma_trigger_sercom_rx<3>() == SERCOM3_DMAC_ID_RX);
static_assert(dma_trigger_sercom_tx<3>() == SERCOM3_DMAC_ID_TX);
// A SERCOM's two codes are adjacent, RX first - table 25-2's shape, and
// the reason a driver could be tempted to compute them. It does not.
static_assert(dma_trigger_sercom_tx<0>() == dma_trigger_sercom_rx<0>() + 1);

// ---- the channel control word ------------------------------------------------
constexpr DmaChannelConfig beat_cfg{
    .trigger = SERCOM0_DMAC_ID_TX,
    .action = DmaTriggerAction::beat,
    .priority = DmaPriority::level2,
};
static_assert((dma_chctrlb(beat_cfg) & DMAC_CHCTRLB_TRIGACT_Msk) ==
              DMAC_CHCTRLB_TRIGACT_BEAT);
static_assert((dma_chctrlb(beat_cfg) & DMAC_CHCTRLB_LVL_Msk) == DMAC_CHCTRLB_LVL_LVL2);
static_assert((dma_chctrlb(beat_cfg) & DMAC_CHCTRLB_CMD_Msk) == 0);
static_assert((dma_chctrlb({}) & DMAC_CHCTRLB_TRIGSRC_Msk) == 0);
static_assert((dma_chctrlb({}) & DMAC_CHCTRLB_TRIGACT_Msk) == DMAC_CHCTRLB_TRIGACT_BLOCK);

// ---- the resource and channel verbs -------------------------------------------
void block_verbs() {
    (void)Dmac::init();
    (void)Dmac::init({.levels = 0x03, .round_robin = 0x01, .debug_run = true});
    Dmac::bus_clock(true);
    (void)Dmac::enabled();
    (void)Dmac::enable(false);
    (void)Dmac::reset();

    (void)Dmac::interrupt_status();
    (void)Dmac::busy_channels();
    (void)Dmac::pending_channels();
    (void)Dmac::active();
    (void)Dmac::interrupt_pending();
    (void)Dmac::take_pending();

    (void)Dmac::descriptor(0).DMAC_BTCNT;
    (void)Dmac::read_write_back(0);
    Dmac::write_descriptor(Dmac::descriptor(1), m2m);
    Dmac::release();
}

void channel_verbs() {
    using Ch = DmaChannel<0>;
    static_assert(Ch::index == 0);
    static_assert(Ch::mask == 1u);

    Ch::configure(beat_cfg);
    (void)Ch::reset();
    Ch::load(m2m);
    (void)Ch::load(DmaTransfer{});
    (void)Ch::loaded();

    Ch::enable(true);
    (void)Ch::enabled();
    Ch::trigger();
    (void)Ch::trigger_lost();
    Ch::clear_trigger_lost();
    Ch::suspend();
    Ch::resume();

    (void)Ch::flags();
    (void)Ch::take_flags();
    Ch::clear_flags(DmaFlag::all);
    Ch::arm(DmaFlag::complete, true);
    (void)Ch::armed();
    (void)Ch::status();
    (void)Ch::busy();
    (void)Ch::pending();
    (void)Ch::fetch_error();

    (void)Ch::harvest();
    (void)Ch::consistent(m2m);
    (void)Ch::violations();
    (void)Ch::suspend_timeouts();
    Ch::clear_counters();

    // The highest channel this device has.
    using Last = DmaChannel<Dmac::channel_count - 1>;
    static_assert(Last::mask == 0x800u);
    (void)Last::status();
}

// ---- the peripheral engines ---------------------------------------------------
// They live in this header, not in sercom.hpp, so that sercom.hpp never
// includes this one: a program that names no engine cannot reach the
// descriptor tables and does not carry them. What a Uart does with an
// engine is the SERCOM fixture's business (test/family_samc/sercom.cpp).
static_assert(DmaTxEngine<0>::present && DmaRxEngine<1>::present);
static_assert(DmaTxEngine<7>::channel == 7);
static_assert(DmaRxEngine<Dmac::channel_count - 1>::channel == 11);

void engine_verbs() {
    volatile uint32_t data = 0;
    DmaTxEngine<0>::arm(&data, dma_trigger_sercom_tx<0>());
    static uint8_t out[8];
    (void)DmaTxEngine<0>::start(out, sizeof(out));
    (void)DmaTxEngine<0>::busy();
    (void)DmaTxEngine<0>::in_flight();
    (void)DmaTxEngine<0>::complete();
    DmaTxEngine<0>::stop();

    DmaRxEngine<1>::arm(&data, dma_trigger_sercom_rx<0>());
    static uint8_t in[8];
    (void)DmaRxEngine<1>::start(in, sizeof(in));
    (void)DmaRxEngine<1>::take();
    (void)DmaRxEngine<1>::full();
    (void)DmaRxEngine<1>::capacity();
    (void)DmaRxEngine<1>::taken();
    DmaRxEngine<1>::stop();
}
