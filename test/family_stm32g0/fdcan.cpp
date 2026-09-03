// Family smoke TU: stm32g0/fdcan.hpp on each of the three headers the
// desk's boards span. Instantiation only - no main(), no hardware.
//
// WHAT THIS FIXTURE IS REALLY FOR. FDCAN is the first chapter of this
// stratum that a part can be MISSING ENTIRELY: table 1 gives the two
// modules to the G0B1/G0C1 class alone, and the smaller headers declare
// no base, no struct and no interrupt enumerator - so `Fdcan<n>` does
// not exist there at all (the negatives prove it) while every line of
// the protocol ARITHMETIC still has to compile, because a bit timing and
// a DLC are properties of CAN and not of a peripheral. This file is
// therefore in two halves: the arithmetic and the codecs pinned against
// tables 212..223 on all three headers, and the register-facing verbs
// instantiated behind the same gate the driver uses.

#include <stdint.h>

#include "stm32g0/fdcan.hpp"

using namespace brio;

// ---- presence, as the three headers state it -------------------------------
#if defined(STM32G0B1xx)
static_assert(fdcan_present(1) && fdcan_present(2),
              "table 1: the G0B1/G0C1 class carries both modules");
static_assert(fdcan_instances() == 2);
static_assert(fdcan_base(1) == 0x40006400u && fdcan_base(2) == 0x40006800u,
              "the memory map's own addresses");
static_assert(fdcan_config_base() == 0x40006500u,
              "the subsystem's CKDIV sits BETWEEN the two modules");
static_assert(fdcan_ram_base(1) == 0x4000B400u,
              "SRAMCAN, the message RAM");
static_assert(fdcan_ram_base(2) == fdcan_ram_base(1) + 0x350u,
              "36.3.6: FDCAN2 starts where FDCAN1 ends + 4");
static_assert(fdcan_clock_mask() == RCC_APBENR1_FDCANEN &&
                  fdcan_reset_mask() == RCC_APBRSTR1_FDCANRST,
              "ONE enable and ONE reset for the whole subsystem");
static_assert(fdcan_clock_select_pos() == RCC_CCIPR2_FDCANSEL_Pos,
              "5.4.22: the kernel clock is chosen in CCIPR2 and not in CCIPR");
static_assert(fdcan_irq(0) == TIM16_FDCAN_IT0_IRQn &&
                  fdcan_irq(1) == TIM17_FDCAN_IT1_IRQn,
              "table 61: one line for TIM16 and BOTH instances' intr0, one for "
              "TIM17 and both intr1");
static_assert(fdcan_irq(0) == tim_irq(16) && fdcan_irq(1) == tim_irq(17),
              "and that sharing is the timers' own vectors, said twice");
#else
static_assert(!fdcan_present(1) && !fdcan_present(2),
              "no smaller STM32G0 has an FDCAN at all");
static_assert(fdcan_instances() == 0);
static_assert(fdcan_base(1) == 0 && fdcan_ram_base(1) == 0 &&
                  fdcan_config_base() == 0);
static_assert(fdcan_clock_mask() == 0 && fdcan_reset_mask() == 0);
static_assert(fdcan_clock_select_pos() == 0xFF,
              "and no CCIPR2 to select a kernel clock in");
#endif
static_assert(!fdcan_present(0) && !fdcan_present(3),
              "there is no FDCAN3 anywhere in this family");

// ---- the clock divider (36.4.37) -------------------------------------------
static_assert(fdcan_divider_value(FdcanClockDivider::div1) == 1);
static_assert(fdcan_divider_value(FdcanClockDivider::div2) == 2);
static_assert(fdcan_divider_value(FdcanClockDivider::div6) == 6);
static_assert(fdcan_divider_value(FdcanClockDivider::div30) == 30,
              "code n is /2n, and code 0 alone is /1");

// ---- DLC coding (table 212) ------------------------------------------------
static_assert(fdcan_dlc_to_length(0, false) == 0);
static_assert(fdcan_dlc_to_length(8, false) == 8);
static_assert(fdcan_dlc_to_length(15, false) == 8,
              "codes 9..15 are eight bytes in CLASSIC CAN");
static_assert(fdcan_dlc_to_length(9, true) == 12);
static_assert(fdcan_dlc_to_length(10, true) == 16);
static_assert(fdcan_dlc_to_length(11, true) == 20);
static_assert(fdcan_dlc_to_length(12, true) == 24);
static_assert(fdcan_dlc_to_length(13, true) == 32);
static_assert(fdcan_dlc_to_length(14, true) == 48);
static_assert(fdcan_dlc_to_length(15, true) == 64);
static_assert(fdcan_length_to_dlc(0) == 0 && fdcan_length_to_dlc(8) == 8);
static_assert(fdcan_length_to_dlc(12) == 9 && fdcan_length_to_dlc(64) == 15);
static_assert(fdcan_length_to_dlc(13) == 0xFF,
              "thirteen bytes is not a CAN frame in either format");
static_assert(fdcan_length_to_dlc(9) == 0xFF && fdcan_length_to_dlc(65) == 0xFF);
static_assert(fdcan_length_valid(48) && !fdcan_length_valid(40));

// ---- bit timing (36.3.3, 36.4.3, 36.4.7) -----------------------------------
// THE MANUAL'S OWN TWO EXAMPLES, pinned as register values.
// NBTP reset = 0x0600_0A03: NSJW 3, NBRP 0, NTSEG1 10, NTSEG2 3.
constexpr FdcanBitTiming nbtp_reset{0, 10, 3, 3};
static_assert(nbtp_reset.tq_per_bit() == 16,
              "1 + (10 + 1) + (3 + 1)");
static_assert(fdcan_bit_hz(48'000'000u, nbtp_reset) == 3'000'000u,
              "36.4.7's note: 3 Mbit/s from a 48 MHz kernel clock");
static_assert(fdcan_bit_hz(64'000'000u, nbtp_reset) == 4'000'000u,
              "...and 4 Mbit/s from this board's 64 MHz");
static_assert(nbtp_reset.sample_point_permille() == 750);
// DBTP reset = 0x0000_0A33: DBRP 0, DTSEG1 10, DTSEG2 3, DSJW 3.
constexpr FdcanBitTiming dbtp_reset{0, 10, 3, 3};
static_assert(fdcan_bit_hz(8'000'000u, dbtp_reset) == 500'000u,
              "36.4.3's note: 500 kbit/s from an 8 MHz clock");

static_assert(fdcan_nominal_timing_valid(nbtp_reset));
static_assert(fdcan_data_timing_valid(dbtp_reset));
static_assert(!fdcan_nominal_timing_valid(FdcanBitTiming{0, 0, 0, 0}),
              "3 tq is below 36.4.7's window of 4..81");
static_assert(!fdcan_nominal_timing_valid(FdcanBitTiming{0, 60, 20, 0}),
              "82 tq is above it");
static_assert(!fdcan_nominal_timing_valid(FdcanBitTiming{512, 10, 3, 3}),
              "NBRP is nine bits");
static_assert(!fdcan_data_timing_valid(FdcanBitTiming{32, 10, 3, 3}),
              "DBRP is FIVE bits - the same struct, far narrower fields");
static_assert(!fdcan_data_timing_valid(FdcanBitTiming{0, 32, 3, 3}));
static_assert(!fdcan_data_timing_valid(FdcanBitTiming{0, 10, 3, 4}),
              "SJW may never exceed BS2");

// The chooser, at this board's own rates.
static_assert(fdcan_bit_timing_for(64'000'000u, 500'000u, 875).has_value());
static_assert(fdcan_bit_timing_for(64'000'000u, 500'000u, 875)->brp == 1);
static_assert(fdcan_bit_timing_for(64'000'000u, 500'000u, 875)->tq_per_bit() == 64,
              "the smallest prescaler that fits the window - the finest grid");
static_assert(
    fdcan_bit_timing_for(64'000'000u, 500'000u, 875)->sample_point_permille() == 875);
static_assert(fdcan_bit_hz(64'000'000u, *fdcan_bit_timing_for(64'000'000u, 125'000u)) ==
              125'000u);
static_assert(fdcan_bit_hz(64'000'000u, *fdcan_bit_timing_for(64'000'000u, 1'000'000u)) ==
              1'000'000u);
static_assert(!fdcan_bit_timing_for(64'000'000u, 3'000'000u).has_value(),
              "64 is not a multiple of 3: the chooser is EXACT or nothing");
static_assert(!fdcan_bit_timing_for(64'000'000u, 0u).has_value());
static_assert(fdcan_data_timing_for(64'000'000u, 8'000'000u).has_value());
static_assert(fdcan_data_timing_for(64'000'000u, 8'000'000u)->tq_per_bit() == 8);
static_assert(!fdcan_data_timing_for(64'000'000u, 5'000'000u).has_value());
static_assert(fdcan_data_timing_for(64'000'000u, 125'000u).has_value(),
              "a slow DATA phase is legal arithmetic - DBRP's five bits reach it, "
              "and 36.4.3's 'not slower than the nominal' rule is the CONFIG's job");
static_assert(!fdcan_data_timing_for(64'000'000u, 100u).has_value(),
              "...but nothing DBTP can hold reaches 100 bit/s");

// ---- the ILS grouping (36.4.17) --------------------------------------------
static_assert(fdcan_group_of(FdcanFlag::rx_fifo0_new) == FdcanGroup::rx_fifo0);
static_assert(fdcan_group_of(FdcanFlag::rx_fifo1_lost) == FdcanGroup::rx_fifo1);
static_assert(fdcan_group_of(FdcanFlag::transmission_completed) ==
              FdcanGroup::status_message);
static_assert(fdcan_group_of(FdcanFlag::tx_event_new) == FdcanGroup::tx_fifo_error);
static_assert(fdcan_group_of(FdcanFlag::timeout) == FdcanGroup::misc);
static_assert(fdcan_group_of(FdcanFlag::error_passive) == FdcanGroup::bit_line_error);
static_assert(fdcan_group_of(FdcanFlag::bus_off) == FdcanGroup::protocol_error,
              "36.4.17 puts BO with the PROTOCOL group and EP with the BIT one - "
              "which is not where a reader would guess");
static_assert(fdcan_group_of(1u << 24) == 0, "and nothing above bit 23 is a flag");
static_assert((fdcan_flags_of(FdcanGroup::all) & FdcanFlag::all) == FdcanFlag::all,
              "every implemented flag belongs to exactly one group");
static_assert(fdcan_flags_of(FdcanGroup::rx_fifo0) ==
              (FdcanFlag::rx_fifo0_new | FdcanFlag::rx_fifo0_full |
               FdcanFlag::rx_fifo0_lost));

// ---- the element codecs, against hand-built words (tables 214..219) --------
// A classic 11-bit data frame, ID 0x123, four bytes.
constexpr FdcanFrame classic_frame{
    .id = 0x123, .length = 4, .store_event = true, .marker = 0x5A,
    .data = {0xDE, 0xAD, 0xBE, 0xEF}};
constexpr auto classic_words = *fdcan_encode_tx(classic_frame);
static_assert(classic_words.count == 3, "two headers and one data word");
static_assert(classic_words.w[0] == (0x123u << 18),
              "table 217: a STANDARD identifier is written to ID[28:18]");
static_assert(classic_words.w[1] == ((4u << 16) | (1u << 23) | (0x5Au << 24)),
              "DLC 4, EFC set, MM = 0x5A");
static_assert(classic_words.w[2] == 0xEFBEADDEu,
              "table 216: data byte 0 in bits 7:0, byte 3 in bits 31:24");
constexpr FdcanFrame classic_back = fdcan_decode_rx(classic_words);
static_assert(classic_back.id == 0x123 && classic_back.length == 4);
static_assert(!classic_back.extended && !classic_back.remote && !classic_back.fd);
static_assert(classic_back.data[0] == 0xDE && classic_back.data[3] == 0xEF);

// An extended FD frame with bit rate switching, 64 bytes.
constexpr FdcanFrame fd_frame{.id = 0x1ABCDEF0u & 0x1FFFFFFFu, .length = 64,
                              .extended = true, .fd = true,
                              .bit_rate_switch = true,
                              .error_state_indicator = true};
constexpr auto fd_words = *fdcan_encode_tx(fd_frame);
static_assert(fd_words.count == 18, "two headers and sixteen data words");
static_assert(fd_words.w[0] == ((0x1ABCDEF0u & 0x1FFFFFFFu) | (1u << 30) | (1u << 31)),
              "an EXTENDED identifier occupies ID[28:0] whole, XTD and ESI above it");
static_assert(fd_words.w[1] == ((15u << 16) | (1u << 20) | (1u << 21) | (1u << 23)),
              "DLC 15 = 64 bytes, BRS and FDF set");
static_assert(fdcan_decode_rx(fd_words).length == 64);
static_assert(fdcan_decode_rx(fd_words).extended);
static_assert(fdcan_decode_rx(fd_words).id == (0x1ABCDEF0u & 0x1FFFFFFFu));

// A remote frame carries no data whatever its DLC says.
constexpr FdcanFrame remote_frame{.id = 0x7FF, .length = 8, .remote = true};
static_assert((*fdcan_encode_tx(remote_frame)).w[0] == ((0x7FFu << 18) | (1u << 29)));
static_assert(fdcan_decode_rx(*fdcan_encode_tx(remote_frame)).length == 0,
              "the decode gives a remote frame no bytes at all");
static_assert(fdcan_decode_rx(*fdcan_encode_tx(remote_frame)).remote);

// A length no DLC codes is refused before a word is built.
static_assert(!fdcan_encode_tx(FdcanFrame{.id = 1, .length = 13, .fd = true}).has_value());
static_assert(!fdcan_encode_tx(FdcanFrame{.id = 1, .length = 12}).has_value(),
              "and a CLASSIC frame is refused past eight bytes even at a legal DLC");

// The Rx-only fields of R1 (table 215).
constexpr FdcanElementWords rx_words{
    {(0x321u << 18), (0x1234u | (2u << 16) | (5u << 24) | (1u << 31)), 0}, 3};
static_assert(fdcan_decode_rx(rx_words).timestamp == 0x1234);
static_assert(fdcan_decode_rx(rx_words).filter_index == 5);
static_assert(fdcan_decode_rx(rx_words).non_matching,
              "ANMF: no filter matched, so FIDX means nothing");

// The Tx event element (tables 218 and 219).
static_assert(fdcan_decode_event((0x123u << 18), (0x0102u | (3u << 16) |
                                                  (2u << 22) | (0x77u << 24)))
                  .event_type == 2,
              "ET = 10: transmission in spite of cancellation");
static_assert(fdcan_decode_event(0, (0x0102u | (3u << 16))).timestamp == 0x0102);
static_assert(fdcan_decode_event(0, (3u << 16) | (0x77u << 24)).marker == 0x77);

// The filter elements (tables 220 and 222).
static_assert(fdcan_standard_filter_word({FdcanFilterType::classic,
                                          FdcanFilterAction::store_fifo0,
                                          0x123, 0x7FF}) ==
                  ((2u << 30) | (1u << 27) | (0x123u << 16) | 0x7FFu));
static_assert(fdcan_standard_filter_word({FdcanFilterType::range,
                                          FdcanFilterAction::reject, 0x100, 0x1FF}) ==
              ((0u << 30) | (3u << 27) | (0x100u << 16) | 0x1FFu));
static_assert(fdcan_extended_filter_word0({FdcanExtFilterType::range_no_mask,
                                           FdcanFilterAction::store_fifo1,
                                           0x1234567, 0x89ABCDE}) ==
              ((2u << 29) | 0x1234567u));
static_assert(fdcan_extended_filter_word1({FdcanExtFilterType::range_no_mask,
                                           FdcanFilterAction::store_fifo1,
                                           0x1234567, 0x89ABCDE}) ==
                  ((3u << 30) | 0x89ABCDEu),
              "EFT 11 is a SECOND range filter - the one XIDAM is not applied to");
static_assert(fdcan_standard_filter_valid({FdcanFilterType::dual,
                                           FdcanFilterAction::store_fifo0, 0x7FF, 0}));
static_assert(!fdcan_standard_filter_valid({FdcanFilterType::dual,
                                            FdcanFilterAction::store_fifo0, 0x800, 0}),
              "a standard identifier is eleven bits");
static_assert(!fdcan_extended_filter_valid({FdcanExtFilterType::range,
                                            FdcanFilterAction::store_fifo0,
                                            0x20000000u, 0}));
static_assert(!fdcan_standard_filter_valid(
                  {FdcanFilterType::classic, static_cast<FdcanFilterAction>(7), 0, 0}),
              "SFEC 111 is 'not used' (table 221)");

// ---- the configuration checker, rule by rule -------------------------------
constexpr FdcanBitTiming nominal500 = *fdcan_bit_timing_for(64'000'000u, 500'000u);
constexpr FdcanBitTiming data2m = *fdcan_data_timing_for(64'000'000u, 2'000'000u);

static_assert(fdcan_config_valid(FdcanConfig{.nominal = nominal500}));
static_assert(!fdcan_config_valid(FdcanConfig{}),
              "a default-constructed timing is 3 tq and outside 36.4.7's window");
static_assert(!fdcan_config_valid(FdcanConfig{
                  .nominal = nominal500,
                  .mode = FdcanMode::internal_loop_back,
                  .restricted = true}),
              "36.3.4's note: restricted operation must not be combined with either "
              "loop-back");
static_assert(!fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                              .mode = FdcanMode::external_loop_back,
                                              .restricted = true}));
static_assert(fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                             .mode = FdcanMode::bus_monitor,
                                             .restricted = true}),
              "...but ASM on top of bus monitoring is legal, because ASM is its own bit");
static_assert(fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                             .data = data2m,
                                             .fd = FdcanFd::on_with_bit_rate_switch}));
static_assert(!fdcan_config_valid(FdcanConfig{.nominal = data2m,
                                              .data = nominal500,
                                              .fd = FdcanFd::on_with_bit_rate_switch}),
              "36.4.3's note: the data phase may not be SLOWER than the nominal one");
constexpr FdcanBitTiming data500 = *fdcan_data_timing_for(64'000'000u, 500'000u);
static_assert(data500.clocks_per_bit() == nominal500.clocks_per_bit(),
              "the same 500 kbit/s out of DBTP's narrower fields - 32 tq of a "
              "prescaler of 4 where NBTP takes 64 tq of 2");
static_assert(fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                             .data = data500,
                                             .fd = FdcanFd::on}),
              "equal rates are legal - the note says 'higher than or equal'");
static_assert(!fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                              .data = data2m,
                                              .tdc_offset = 128,
                                              .fd = FdcanFd::on}),
              "TDCO is seven bits");
static_assert(!fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                              .standard_filters = 29}),
              "36.4.19: 28 standard filter elements and no more");
static_assert(!fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                              .extended_filters = 9}));
static_assert(!fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                              .extended_mask = 0x20000000u}),
              "XIDAM is 29 bits");
static_assert(!fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                              .timestamp_prescaler = 17}),
              "36.4.8: TCP counts 1..16 CAN bit times");
static_assert(!fdcan_config_valid(FdcanConfig{.nominal = nominal500,
                                              .timestamp_prescaler = 0}));

#if defined(FDCAN1_BASE)

// ---- every verb of the resource, on both instances -------------------------
template <uint8_t n>
void exercise_resource() {
    using C = Fdcan<n>;
    static_assert(C::index == n);
    static_assert(C::ram_words == 212 && C::element_words == 18);
    static_assert(C::rx_fifo0_start == 44 && C::rx_fifo1_start == 98,
                  "figure 399's own byte offsets, as words");
    static_assert(C::tx_event_start == 152 && C::tx_buffer_start == 158);
    static_assert(C::standard_filter_count == 28 && C::extended_filter_count == 8);
    static_assert(C::irq0() == fdcan_irq(0) && C::irq1() == fdcan_irq(1));
    (void)&C::regs();
    (void)C::ram();
    (void)&C::config_regs();

    C::bus_clock(true);
    (void)C::bus_clock();
    C::reset();
    (void)C::kernel_clock(FdcanClock::pclk);
    (void)C::kernel_clock();
    (void)C::clock_divider(FdcanClockDivider::div2);
    (void)C::clock_divider();
    (void)C::tq_hz(64'000'000u);

    (void)C::init_mode(true);
    (void)C::in_init();
    (void)C::configuration(true);
    (void)C::configuration();
    (void)C::configurable();

    (void)C::nominal_timing(nominal500);
    (void)C::nominal_timing();
    (void)C::data_timing(data2m, true);
    (void)C::data_timing();
    (void)C::delay_compensation();
    (void)C::delay_compensation_offsets(16, 8);

    (void)C::bus_monitor(true);
    (void)C::bus_monitor();
    (void)C::restricted(false);
    (void)C::restricted();
    (void)C::auto_retransmit(false);
    (void)C::auto_retransmit();
    (void)C::test_mode(true);
    (void)C::test_mode();
    (void)C::fd_mode(FdcanFd::on_with_bit_rate_switch);
    (void)C::fd_mode();
    (void)C::non_iso(true);
    (void)C::non_iso();
    (void)C::transmit_pause(true);
    (void)C::transmit_pause();
    (void)C::edge_filtering(true);
    (void)C::edge_filtering();
    (void)C::protocol_exception_disable(true);
    (void)C::protocol_exception_disable();

    (void)C::loop_back(true);
    (void)C::loop_back();
    (void)C::tx_pin(FdcanTxPin::sample_point);
    (void)C::tx_pin();
    (void)C::rx_pin();

    C::power_down(true);
    (void)C::power_down();
    (void)C::power_down_acked();

    (void)C::ram_word(0);
    (void)C::set_ram_word(0, 0);
    C::clear_ram();
    (void)C::ram_watchdog(0x20);
    (void)C::ram_watchdog();
    (void)C::ram_watchdog_value();

    (void)C::standard_filter(0, FdcanStandardFilter{});
    (void)C::standard_filter(0);
    (void)C::extended_filter(0, FdcanExtendedFilter{});
    (void)C::filter_lists(1, 1);
    (void)C::standard_filter_list();
    (void)C::extended_filter_list();
    (void)C::non_matching(FdcanNonMatching::reject, FdcanNonMatching::fifo1);
    (void)C::reject_remote(true, true);
    (void)C::fifo_overwrite(true, false);
    (void)C::fifo_overwrite(uint8_t{0});
    (void)C::global_filter_config();
    (void)C::extended_mask(0x1FFFFFFFu);
    (void)C::extended_mask();
    (void)C::high_priority();

    FdcanFrame frame{};
    (void)C::rx_status(0);
    (void)C::rx_available(0);
    (void)C::rx_full(1);
    (void)C::rx_lost(0);
    (void)C::rx_get_index(0);
    (void)C::rx_put_index(1);
    (void)C::rx_peek(0, 0, frame);
    (void)C::rx_acknowledge(0, 0);
    (void)C::rx_read(0, frame);
    (void)C::rx_read_overwrite(1, frame);

    (void)C::tx_queue_mode(true);
    (void)C::tx_queue_mode();
    (void)C::tx_free();
    (void)C::tx_full();
    (void)C::tx_put_index();
    (void)C::tx_get_index();
    (void)C::tx_put_buffer(0, frame);
    (void)C::tx_request(0x1);
    (void)C::tx_put(frame);
    (void)C::tx_cancel(0x1);
    (void)C::tx_pending();
    (void)C::tx_occurred();
    (void)C::tx_cancelled();
    (void)C::tx_cancel_requested();
    (void)C::tx_buffer_interrupts(0x7);
    (void)C::tx_buffer_interrupts();
    (void)C::tx_cancel_interrupts(0x7);
    (void)C::tx_cancel_interrupts();

    FdcanTxEvent event{};
    (void)C::events_available();
    (void)C::events_full();
    (void)C::events_lost();
    (void)C::event_get_index();
    (void)C::event_put_index();
    (void)C::event_read(event);

    (void)C::timestamp(FdcanTimestamp::external_tim3, 1);
    (void)C::timestamp();
    (void)C::timestamp_prescaler();
    (void)C::timestamp_value();
    C::reset_timestamp();
    (void)C::timeout(FdcanTimeoutMode::rx_fifo0, 100, true);
    (void)C::timeout_enabled();
    (void)C::timeout_mode();
    (void)C::timeout_period();
    (void)C::timeout_value();
    C::reset_timeout();

    (void)C::status();
    (void)C::error_counters();
    (void)C::core_release();
    (void)C::endianness();

    (void)C::flags();
    C::clear_flags(FdcanFlag::all);
    (void)C::interrupts(FdcanFlag::rx_fifo0_new, true);
    (void)C::interrupts();
    (void)C::interrupt_line(FdcanGroup::rx_fifo0, 1);
    (void)C::interrupt_line();
    C::interrupt_lines(true, true);
    (void)C::interrupt_line_enabled(0);
    (void)C::flags_on_line(1);
    (void)C::isr(0);
    (void)C::isr0();
    (void)C::isr1();

    (void)C::enter(FdcanConfig{.nominal = nominal500});
    (void)C::template enter<FdcanConfig{.nominal = nominal500,
                                        .data = data2m,
                                        .mode = FdcanMode::internal_loop_back,
                                        .fd = FdcanFd::on_with_bit_rate_switch}>();
    (void)C::start();
    (void)C::stop();
    C::release();
}

// ---- the pads, one of each signal on each instance -------------------------
// Every FDCAN pad of this part is AF3 (DS13560 tables 14, 15, 17, 18).
using Rx1 = FdcanPad<PinSel{'B', 8, PinFunction::af3}>;    // FDCAN1_RX
using Tx1 = FdcanPad<PinSel{'B', 9, PinFunction::af3}>;    // FDCAN1_TX
using Rx1b = FdcanPad<PinSel{'A', 11, PinFunction::af3}>;
using Tx1b = FdcanPad<PinSel{'C', 5, PinFunction::af3}>;
using Rx2 = FdcanPad<PinSel{'B', 5, PinFunction::af3}>;    // FDCAN2_RX
using Tx2 = FdcanPad<PinSel{'B', 6, PinFunction::af3}>;    // FDCAN2_TX
using Rx2b = FdcanPad<PinSel{'C', 2, PinFunction::af3}>;
using Tx2b = FdcanPad<PinSel{'B', 13, PinFunction::af3}>;

void exercise_pads() {
    Rx1::claim_rx(PinPull::up);
    Rx1::pull(PinPull::down);
    Tx1::claim_tx();
    Tx1::claim_tx(PinSpeed::low);
    Rx1::release();
    Tx1::release();
    Rx1b::claim_rx();
    Tx1b::claim_tx();
    Rx2::claim_rx();
    Tx2::claim_tx();
    Rx2b::claim_rx();
    Tx2b::claim_tx();
}

void exercise_all() {
    exercise_resource<1>();
    exercise_resource<2>();
    exercise_pads();
}

#endif // FDCAN1_BASE
