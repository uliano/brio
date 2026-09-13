// bxCAN family smoke TU (RM0090 ch. 32, RM0390 ch. 30). Three things
// differ across the family and all three are asked of the header here:
// WHICH instances exist (a pair, a trio, or none), WHOSE filter block each
// one filters through, and how many banks that block has - the last being
// the one fact this project declines to state for CAN3. The bit-timing
// search above them is pure arithmetic and is checked on every part.
#include "stm32f4/can.hpp"

using namespace brio;

// ---- the vocabulary, on every part ------------------------------------------------

static_assert(can_std_id_max == 0x7FFu && can_ext_id_max == 0x1FFFFFFFu);
static_assert(can_frame_valid(CanFrame{0x7FF, false, false, 8, {}, 0, 0}));
static_assert(!can_frame_valid(CanFrame{0x800, false, false, 0, {}, 0, 0}), "twelve bits standard");
static_assert(can_frame_valid(CanFrame{0x1FFFFFFF, true, false, 0, {}, 0, 0}));
static_assert(!can_frame_valid(CanFrame{0x20000000, true, false, 0, {}, 0, 0}));
static_assert(!can_frame_valid(CanFrame{0, false, false, 9, {}, 0, 0}), "eight bytes at most");

// ---- the bit timing ------------------------------------------------------------------

static_assert(can_timing_quanta(CanTiming{1, 12, 2, 2}) == 15u, "1 + TS1 + TS2");
static_assert(can_sample_point_of(CanTiming{1, 12, 2, 2}) == 866u, "13 of 15 quanta");
static_assert(can_sample_point_of(CanTiming{1, 13, 2, 2}) == 875u, "14 of 16 - exactly 87.5%");
static_assert(can_timing_valid(CanTiming{1024, 16, 8, 4}), "the widest legal timing");
static_assert(!can_timing_valid(CanTiming{1025, 16, 8, 4}), "BRP is ten bits");
static_assert(!can_timing_valid(CanTiming{1, 17, 8, 4}), "TS1 is four bits");
static_assert(!can_timing_valid(CanTiming{1, 16, 9, 4}), "TS2 is three bits");
static_assert(!can_timing_valid(CanTiming{1, 16, 8, 5}), "SJW is two bits");
static_assert(!can_timing_valid(CanTiming{1, 16, 2, 3}), "SJW may not exceed TS2");
static_assert(!can_timing_valid(CanTiming{}), "an empty timing is not a timing");

// The four classic rates from an APB1 of 45 MHz - the F446's and the
// F42x/F43x's at their top core rate - each EXACT and each within 1% of
// the 87.5% sample point the CiA recommends.
static_assert(can_timing_valid(can_timing_for(45'000'000u, 1'000'000u)));
static_assert(can_bitrate_of(45'000'000u, can_timing_for(45'000'000u, 1'000'000u)) == 1'000'000u);
static_assert(can_bitrate_of(45'000'000u, can_timing_for(45'000'000u, 500'000u)) == 500'000u);
static_assert(can_bitrate_of(45'000'000u, can_timing_for(45'000'000u, 250'000u)) == 250'000u);
static_assert(can_bitrate_of(45'000'000u, can_timing_for(45'000'000u, 125'000u)) == 125'000u);
static_assert(can_sample_point_of(can_timing_for(45'000'000u, 500'000u)) >= 860u &&
              can_sample_point_of(can_timing_for(45'000'000u, 500'000u)) <= 890u);

// The same from 42 MHz (the F405 class's APB1 at 168 MHz) and from 25 MHz
// (the F411's at 100 MHz - a part with no CAN, which is why the search is
// checked on the number and not on a part).
static_assert(can_bitrate_of(42'000'000u, can_timing_for(42'000'000u, 500'000u)) == 500'000u);
static_assert(can_bitrate_of(25'000'000u, can_timing_for(25'000'000u, 125'000u)) == 125'000u);

// AN EXACT RATE OR NOTHING. 45 MHz cannot make 800 kbit/s with any legal
// quanta count, and the search says so instead of rounding.
static_assert(!can_timing_valid(can_timing_for(45'000'000u, 800'000u)));
static_assert(!can_timing_valid(can_timing_for(0u, 500'000u)));
static_assert(!can_timing_valid(can_timing_for(45'000'000u, 0u)));

// A sample point the caller names is honoured where the segments allow it.
static_assert(can_sample_point_of(can_timing_for(45'000'000u, 500'000u, 750u)) <= 800u);

static_assert(can_frame_bits(0, false) == 44u && can_frame_bits(8, false) == 108u);
static_assert(can_frame_bits(8, true) == 128u, "figure 396: 64 + 8N with an extended identifier");

// ---- the filter words ------------------------------------------------------------------

// Figure 391's 32-bit mapping: the standard identifier at the top, IDE in
// bit 2 and RTR in bit 1.
static_assert(can_filter32(0x123u, false, false) == (0x123u << 21));
static_assert(can_filter32(0x123u, false, true) == ((0x123u << 21) | 0x2u));
static_assert(can_filter32(0x1FFFFFFFu, true, false) == ((0x1FFFFFFFu << 3) | 0x4u));
static_assert(can_filter32(0u, true, true) == 0x6u);
static_assert(can_filter32_flags_mask(true, true) == 0x6u);
static_assert(can_filter32_flags_mask(true, false) == 0x4u);

// The 16-bit mapping: eleven identifier bits at the top, then RTR, IDE and
// the three extended bits a half filter can see.
static_assert(can_filter16(0x123u, false, false) == (0x123u << 5));
static_assert(can_filter16(0x123u, false, true) == ((0x123u << 5) | (1u << 4)));
static_assert(can_filter16(0x1FFFFFFFu, true, false) == 0xFFEFu,
              "eleven ones, RTR clear, IDE set, three ones");
static_assert(can_filter16(0x1FFFFFFFu, true, true) == 0xFFFFu, "and RTR set fills the word");
static_assert(can_filter16_pair(0x1234u, 0xABCDu) == 0xABCD1234u);

constexpr CanFilter all = can_filter_accept_all(0);
static_assert(all.bank == 0 && all.fifo == 0 && all.r1 == 0 && all.r2 == 0 && all.active);
static_assert(all.scale == CanFilterScale::single32 && all.mode == CanFilterMode::mask);

// ---- what the device header says exists ---------------------------------------------

// CAN1 and CAN2 come as a PAIR: no header of this pack declares one
// without the other, which is what makes `can_dual()` a question about the
// part and not about the instance.
static_assert(can_present(1) == can_present(2), "CAN1 and CAN2 come together");
static_assert(can_dual() == can_present(2));
static_assert(!can_present(0) && !can_present(4));
static_assert(can_present(3) ? can_present(1) : true, "CAN3 never comes alone");

// The filter master: CAN1's block serves the pair, CAN3 has its own.
static_assert(can_filter_master(1) == (can_present(1) ? 1u : 0u));
static_assert(can_filter_master(2) == (can_present(2) ? 1u : 0u));
static_assert(can_filter_master(3) == (can_present(3) ? 3u : 0u));

// The bank count: twenty-eight for the pair, and REFUSED for CAN3, whose
// manual this project has not read.
static_assert(can_filter_banks(1) == (can_present(1) ? 28u : 0u));
static_assert(can_filter_banks(2) == (can_present(2) ? 28u : 0u));
static_assert(can_filter_banks(3) == 0u, "RM0430 is not on the desk");

// The gates and the vectors come and go with the instance, and the four
// vectors of an instance are four distinct lines.
static_assert(can_present(1) == (can_clock_mask(1) != 0u));
static_assert(can_present(2) == (can_clock_mask(2) != 0u));
static_assert(can_present(3) == (can_clock_mask(3) != 0u));
static_assert(can_present(1) == (can_reset_mask(1) != 0u));

#if defined(CAN1_BASE)
static_assert(can_tx_irq(1) != can_rx_irq(1, 0) && can_rx_irq(1, 0) != can_rx_irq(1, 1) &&
              can_rx_irq(1, 1) != can_sce_irq(1));
static_assert(can_tx_irq(4) == NonMaskableInt_IRQn, "no such instance, no vector");
#endif

// The erratum is a fact of the part class and only of the classes read.
static_assert(!can_ttcm_erratum() || can_present(1),
              "the erratum belongs to part classes that have the peripheral");

// ---- the instances -----------------------------------------------------------------

#if defined(CAN1_BASE)

using Bus = Can<1>;
static_assert(Bus::instance == 1 && Bus::filter_master == 1 && Bus::filter_banks == 28);
static_assert(Bus::time_triggered_supported() == !can_ttcm_erratum());

constexpr CanPins bus_pads{.rx = {'A', 11, PinFunction::af9}, .tx = {'A', 12, PinFunction::af9}};
static_assert(can_pins_valid(bus_pads));
static_assert(!can_pins_valid(CanPins{.rx = {'A', 11, PinFunction::af9},
                                      .tx = {'A', 11, PinFunction::af9}}),
              "one pad cannot be both");

void can_block_verbs() {
    Bus::clock(true);
    (void)Bus::clock();
    Bus::filter_clock(true);
    (void)Bus::filter_clock();
    (void)Bus::regs().MCR;
    (void)Bus::filter_regs().FMR;
    Bus::claim_pads<bus_pads>();
    Bus::claim_rx_pad<bus_pads.rx>();

    (void)Bus::init_mode(true);
    (void)Bus::in_init();
    (void)Bus::in_sleep();
    (void)Bus::in_normal();
    (void)Bus::sleep(false);
    Bus::master_reset();
    (void)Bus::options(CanOptions{.auto_bus_off = true, .no_retransmit = true});
    (void)Bus::options().auto_bus_off;

    constexpr CanTiming t = can_timing_for(45'000'000u, 500'000u);
    (void)Bus::timing(t, true, true);
    (void)Bus::timing().brp;
    (void)Bus::loopback();
    (void)Bus::silent();
    (void)Bus::start();
}

void can_transfer_verbs() {
    CanFrame f{};
    f.id = 0x123;
    f.dlc = 8;
    (void)Bus::transmit(f);
    (void)Bus::mailbox_empty(0);
    (void)Bus::next_mailbox();
    (void)Bus::free_mailboxes();
    (void)Bus::lowest_priority(2);
    (void)Bus::abort(0);
    (void)Bus::result(1).ok;
    (void)Bus::clear_result(1);

    (void)Bus::pending(0);
    (void)Bus::full(1);
    (void)Bus::overrun(0);
    Bus::clear_full(0);
    Bus::clear_overrun(1);
    (void)Bus::peek(0);
    (void)Bus::receive(1);
    (void)Bus::release(0);

    // A mailbox and a FIFO the block has not got: every verb answers, none
    // of them reaches past the register file.
    (void)Bus::mailbox_empty(7);
    (void)Bus::abort(7);
    (void)Bus::clear_result(7);
    (void)Bus::peek(2);
    (void)Bus::release(2);
}

void can_error_and_interrupt_verbs() {
    (void)Bus::tec();
    (void)Bus::rec();
    (void)Bus::error_warning();
    (void)Bus::error_passive();
    (void)Bus::bus_off();
    (void)Bus::last_error();
    Bus::mark_error();
    (void)Bus::request_bus_off_recovery();

    (void)Bus::rx_level();
    (void)Bus::last_sample();
    (void)Bus::receiving();
    (void)Bus::transmitting();
    (void)Bus::error_flag();
    (void)Bus::wakeup_flag();
    (void)Bus::sleep_ack_flag();
    Bus::clear_error_flag();
    Bus::clear_wakeup_flag();
    Bus::clear_sleep_ack_flag();

    Bus::interrupt(CanInterrupt::fifo0_pending, true);
    (void)Bus::interrupt(CanInterrupt::fifo0_pending);
    Bus::interrupt(CanInterrupt::bus_off, true);
    Bus::interrupt(CanInterrupt::wakeup, true);
    Bus::interrupts_off();

    const auto tx = Bus::tx_isr();
    (void)tx.mailbox[0].ok;
    const auto rx = Bus::rx_isr(0);
    (void)rx.pending;
    (void)Bus::rx_isr(2).pending;
    const auto sce = Bus::sce_isr();
    (void)sce.bus_off;
    (void)sce.last_error;
}

void can_filter_verbs() {
    (void)Bus::filter_init(true);
    (void)Bus::filter_initializing();
    (void)Bus::start_bank(14);
    (void)Bus::start_bank();
    (void)Bus::first_bank();
    (void)Bus::bank_limit();
    (void)Bus::bank_ok(0);
    (void)Bus::filter(can_filter_accept_all(0));
    (void)Bus::filter(CanFilter{1, CanFilterScale::dual16, CanFilterMode::list, 1,
                                can_filter16_pair(can_filter16(0x100u, false, false),
                                                  can_filter16(0x101u, false, false)),
                                can_filter16_pair(can_filter16(0x102u, false, false),
                                                  can_filter16(0x103u, false, false)),
                                true});
    (void)Bus::filter(0);
    (void)Bus::filter_active(0, false);
    (void)Bus::filter_active(0);
    (void)Bus::filters_off();
    (void)Bus::filter_init(false);

    // A bank past the block's end is refused rather than written.
    (void)Bus::filter(can_filter_accept_all(28));
    (void)Bus::filter(200);
    Bus::release_instance();
}

#endif

#if defined(CAN2_BASE)

// The slave: its filter registers are CAN1's, its banks start at CAN2SB,
// and its four vectors are its own.
using Slave = Can<2>;
static_assert(Slave::filter_master == 1, "CAN1 owns the block CAN2 filters through");
static_assert(Slave::filter_banks == 28);
static_assert(Slave::tx_irq() != Can<1>::tx_irq());

void can2_verbs() {
    Slave::clock(true);
    Slave::filter_clock(true);
    (void)Slave::init_mode(true);
    (void)Slave::first_bank();
    (void)Slave::bank_limit();
    (void)Slave::filter(can_filter_accept_all(14, 0));
    (void)Slave::start();
    Slave::release_instance();
}

#endif

#if defined(CAN3_BASE)

// The third instance, where it exists, is a master of its own - and its
// filter block is the one this project has no manual for, so every filter
// verb answers false and the bank range is empty.
using Third = Can<3>;
static_assert(Third::filter_master == 3);
static_assert(Third::filter_banks == 0u);

void can3_verbs() {
    Third::clock(true);
    (void)Third::init_mode(true);
    (void)Third::filter_init(true);
    (void)Third::bank_ok(0);
    (void)Third::filter(can_filter_accept_all(0));
    (void)Third::filters_off();
    (void)Third::start_bank(1);
    (void)Third::start();
    Third::release_instance();
}

#endif
