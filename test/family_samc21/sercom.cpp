// SERCOM (USART mode) family smoke TU: the Sercom<n> resource, the Uart
// task above it and the baud arithmetic between them.
//
// The instance COUNT is what differs across this family - four on the E
// package, six on the G and J - so this TU exercises SERCOM0, which
// every variant has, and neg/sercom_no_instance_5.cpp states the other
// side of that fact for the E. The pads are PA04/PA05 (SERCOM0 PAD[0]
// and PAD[1] through PMUX function D), bonded on all three variants; the
// console's own PB30/PB31 pair exists only on the 64-pin J and is
// therefore an app-level fact, not one this fixture can compile.
#include "samc21/clock.hpp"
#include "samc21/sercom.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 48'000'000>;

static_assert(sercom_count >= 4, "every variant in the pack has at least SERCOM0..3");

// ---- the pad rules, at compile time ----------------------------------------
// CTRLA.TXPO puts TxD on PAD[0] or PAD[2] and nowhere else, and the two
// directions share a pad only as 31.6.3.8's loop-back - one pin named twice.
static_assert(uart_tx_pad_exists(SercomPad::pad0));
static_assert(uart_tx_pad_exists(SercomPad::pad2));
static_assert(!uart_tx_pad_exists(SercomPad::pad1));
static_assert(!uart_tx_pad_exists(SercomPad::pad3));
static_assert(uart_txpo(SercomPad::pad0) == 0);
static_assert(uart_txpo(SercomPad::pad2) == 1);   // the header calls this code PAD1
static_assert(uart_rxpo(SercomPad::pad3) == 3);

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},   // PA04 = SERCOM0/PAD[0], function D
    .rx_pin = {'A', 5, PinFunction::d},   // PA05 = SERCOM0/PAD[1], function D
};
static_assert(uart_pads_valid(pads));
static_assert(!uart_pads_valid({.tx = SercomPad::pad1, .rx = SercomPad::pad0}));
static_assert(uart_pads_valid({.tx = SercomPad::pad0,
                               .rx = SercomPad::pad0,
                               .tx_pin = {'A', 4, PinFunction::d},
                               .rx_pin = {'A', 4, PinFunction::d}}));
static_assert(!uart_pads_valid({.tx = SercomPad::pad0,
                                .rx = SercomPad::pad0,
                                .tx_pin = {'A', 4, PinFunction::d},
                                .rx_pin = {'A', 5, PinFunction::d}}));
static_assert(!uart_pads_valid({.tx = SercomPad::pad0,
                                .rx = SercomPad::pad1,
                                .tx_pin = {'C', 0, PinFunction::d}}));

// The device header must agree that PA04/PA05 reach SERCOM0's first two
// pads through function D - the one half of the pad-to-pin claim that IS
// checkable without a per-package pad table.
static_assert(MUX_PA04D_SERCOM0_PAD0 == static_cast<uint8_t>(PinFunction::d));
static_assert(MUX_PA05D_SERCOM0_PAD1 == static_cast<uint8_t>(PinFunction::d));

// ---- the baud arithmetic ----------------------------------------------------
// Table 30-2, 16x arithmetic: BAUD = 65536 x (1 - 16 x f_baud / f_ref).
static_assert(sercom_baud_reg(48'000'000, 115200).value() == 63019);
static_assert(sercom_baud_reg(48'000'000, 9600).value() == 65326);
static_assert(sercom_baud_reg(48'000'000, 3'000'000).value() == 0);   // f_ref/16: BAUD 0 is LEGAL
static_assert(sercom_baud_reg(48'000'000, 3'000'001).has_value() == false);  // above f_ref/16
static_assert(sercom_baud_reg(0, 115200).has_value() == false);
static_assert(sercom_baud_reg(48'000'000, 0).has_value() == false);
// Round trip: what the generator really produces, within its own step.
static_assert(sercom_actual_baud(48'000'000, 63019) == 115219);
static_assert(sercom_actual_baud(48'000'000, 0) == 3'000'000);
static_assert(sercom_min_ref_hz(115200) == 1'843'200);

// ---- every sampling regime (CTRLA.SAMPR, table 30-2) -------------------------
static_assert(uart_samples(UartSampling::x16_arithmetic) == 16 &&
              uart_samples(UartSampling::x16_fractional) == 16 &&
              uart_samples(UartSampling::x8_arithmetic) == 8 &&
              uart_samples(UartSampling::x8_fractional) == 8 &&
              uart_samples(UartSampling::x3_arithmetic) == 3);
static_assert(uart_fractional(UartSampling::x16_fractional) &&
              uart_fractional(UartSampling::x8_fractional) &&
              !uart_fractional(UartSampling::x16_arithmetic) &&
              !uart_fractional(UartSampling::x3_arithmetic));
static_assert(static_cast<uint8_t>(UartSampling::x3_arithmetic) == 0x4);
// The arithmetic generator on a 24 MHz crystal reference: the console rate
// and the rate letter x of test_avr_serial needs, where it is 0.1 % off.
static_assert(sercom_baud_reg(24'000'000, 115200).value() == 60503);
static_assert(sercom_actual_baud(24'000'000, 60503) == 115196);
static_assert(sercom_baud_reg(24'000'000, 3750).value() == 65372);
static_assert(sercom_actual_baud(24'000'000, 65372) == 3753);
// 8x arithmetic: the condition is f_ref / 8.
static_assert(sercom_uart_baud_reg(24'000'000, 3'000'000, UartSampling::x8_arithmetic).value() == 0);
static_assert(!sercom_uart_baud_reg(24'000'000, 3'000'001, UartSampling::x8_arithmetic));
static_assert(!sercom_uart_baud_reg(24'000'000, 1'500'001, UartSampling::x16_arithmetic));
// 3x arithmetic: f_ref / 3.
static_assert(sercom_uart_baud_reg(24'000'000, 8'000'000, UartSampling::x3_arithmetic).value() == 0);
// The FRACTIONAL generator: 8 x BAUD + FP = 8 f_ref / (S f_baud), FP in
// bits 15:13. Exact where S f_baud divides 8 f_ref.
static_assert(sercom_fractional_baud_reg(24'000'000, 3750).value() == 400);   // 400.0
static_assert(sercom_fractional_actual_baud(24'000'000, 400) == 3750);
static_assert(sercom_fractional_baud_reg(24'000'000, 9600).value() == ((2u << 13) | 156u));  // 156.25
static_assert(sercom_fractional_actual_baud(24'000'000, (2u << 13) | 156u) == 9600);
static_assert(sercom_fractional_baud_reg(24'000'000, 2'000'000, 8).value() == ((4u << 13) | 1u));
static_assert(sercom_fractional_actual_baud(24'000'000, (4u << 13) | 1u, 8) == 2'000'000);
static_assert(sercom_fractional_baud_reg(24'000'000, 1'500'000).value() == 1);   // the 16x ceiling
static_assert(!sercom_fractional_baud_reg(24'000'000, 1'500'001));
static_assert(!sercom_fractional_baud_reg(24'000'000, 183));   // integer part past 8191
static_assert(sercom_fractional_baud_reg(24'000'000, 184).has_value());
static_assert(sercom_fractional_actual_baud(24'000'000, 0) == 0);   // integer part 0: not legal
static_assert(!sercom_fractional_baud_reg(600'000'000, 9600));      // past the 32-bit guard
// The closest of the two generators, per rate.
static_assert(sercom_closest_baud(24'000'000, 3750)->sampling == UartSampling::x16_fractional);
static_assert(sercom_closest_baud(24'000'000, 3750)->reg == 400);
static_assert(sercom_closest_baud(24'000'000, 115200)->sampling == UartSampling::x16_arithmetic);
static_assert(sercom_closest_baud(24'000'000, 921600)->sampling == UartSampling::x16_arithmetic);
static_assert(sercom_closest_baud(24'000'000, 1'000'000)->sampling == UartSampling::x16_fractional);
static_assert(sercom_closest_baud(24'000'000, 2'000'000, 8)->sampling == UartSampling::x8_fractional);
static_assert(!sercom_closest_baud(24'000'000, 2'000'000, 16));
static_assert(!sercom_closest_baud(24'000'000, 9600, 3));   // no fractional generator at 3x
// The synchronous host: f_xck = f_ref / (2 (BAUD + 1)), eight bits.
static_assert(sercom_sync_baud_reg(24'000'000, 100'000).value() == 119);
static_assert(sercom_sync_baud_reg(24'000'000, 1'000'000).value() == 11);
static_assert(sercom_sync_baud_reg(24'000'000, 12'000'000).value() == 0);
static_assert(!sercom_sync_baud_reg(24'000'000, 12'000'001));
static_assert(sercom_sync_baud_reg(24'000'000, 46'875).value() == 255);
static_assert(!sercom_sync_baud_reg(24'000'000, 46'000));
static_assert(sercom_sync_actual_hz(24'000'000, 11) == 1'000'000);

// ---- the configuration's own rules -------------------------------------------
// The crossed pair the samc21 usart_peer uses: TxD PAD[2], RxD PAD[0],
// XCK on PAD[3] beside it.
constexpr UartPads link_pads{
    .tx = SercomPad::pad2,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 6, PinFunction::d},
    .rx_pin = {'A', 4, PinFunction::d},
};
static_assert(uart_xck_pad(SercomPad::pad2) == SercomPad::pad3);
static_assert(uart_xck_pad(SercomPad::pad0) == SercomPad::pad1);
constexpr SercomUartConfig sync_host{.pads = link_pads, .baud = 11, .synchronous = true};
constexpr SercomUartConfig sync_client{
    .pads = link_pads, .synchronous = true, .external_clock = true, .invert_clock = true};
constexpr SercomUartConfig ir{.pads = link_pads, .baud = 60503, .irda = true, .rx_pulse = 19};
constexpr SercomUartConfig frac{.pads = link_pads, .baud = 400,
                                .sampling = UartSampling::x16_fractional, .sample_adjust = 3};
static_assert(sercom_uart_config_valid(sync_host) && sercom_uart_config_valid(sync_client) &&
              sercom_uart_config_valid(ir) && sercom_uart_config_valid(frac));
static_assert(!sercom_uart_config_valid({.pads = link_pads, .sample_adjust = 4}));
static_assert(!sercom_uart_config_valid({.pads = link_pads, .baud = 256, .synchronous = true}));
static_assert(sercom_uart_config_valid(
    {.pads = link_pads, .baud = 256, .synchronous = true, .external_clock = true}));
static_assert(!sercom_uart_config_valid(
    {.pads = {.tx = SercomPad::pad2, .rx = SercomPad::pad3, .tx_pin = {'A', 6, PinFunction::d},
              .rx_pin = {'A', 7, PinFunction::d}},
     .synchronous = true}));
static_assert(!sercom_uart_config_valid(
    {.pads = link_pads, .sampling = UartSampling::x8_arithmetic, .synchronous = true}));
static_assert(!sercom_uart_config_valid(
    {.pads = link_pads, .baud = 0x2000, .sampling = UartSampling::x16_fractional}));
static_assert(!sercom_uart_config_valid(
    {.pads = link_pads, .sampling = UartSampling::x8_arithmetic, .irda = true}));
static_assert(!sercom_uart_config_valid({.pads = link_pads, .synchronous = true, .irda = true}));
static_assert(!sercom_uart_config_valid({.pads = link_pads, .rx_pulse = 1}));
// The register words carry them.
static_assert((sercom_uart_ctrla(sync_host) & SERCOM_USART_INT_CTRLA_CMODE_Msk) != 0);
static_assert((sercom_uart_ctrla(sync_host) & SERCOM_USART_INT_CTRLA_MODE_Msk) ==
              SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK);
static_assert((sercom_uart_ctrla(sync_client) & SERCOM_USART_INT_CTRLA_MODE_Msk) == 0);
static_assert((sercom_uart_ctrla(sync_client) & SERCOM_USART_INT_CTRLA_CPOL_Msk) != 0);
static_assert((sercom_uart_ctrla(sync_host) & SERCOM_USART_INT_CTRLA_TXPO_Msk) ==
              SERCOM_USART_INT_CTRLA_TXPO(1));
static_assert((sercom_uart_ctrla(frac) & SERCOM_USART_INT_CTRLA_SAMPR_Msk) ==
              SERCOM_USART_INT_CTRLA_SAMPR(1));
static_assert((sercom_uart_ctrla(frac) & SERCOM_USART_INT_CTRLA_SAMPA_Msk) ==
              SERCOM_USART_INT_CTRLA_SAMPA(3));
static_assert((sercom_uart_ctrlb(ir) & SERCOM_USART_INT_CTRLB_ENC_Msk) != 0);
static_assert((sercom_uart_ctrlb(frac) & SERCOM_USART_INT_CTRLB_ENC_Msk) == 0);

// ---- the pinouts beside TxD (TXPO 0x2 and 0x3) -------------------------------
// Both put TxD on PAD[0] and an output (RTS or TE) on PAD[2]; CTS is PAD[3].
constexpr UartPads pad0_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad0,
    .tx_pin = {'A', 4, PinFunction::d},
    .rx_pin = {'A', 4, PinFunction::d},
};
constexpr SercomUartConfig rs485{.pads = pad0_pads, .baud = 60503, .pinout = UartPinout::rs485,
                                 .guard_time = 7};
constexpr SercomUartConfig flow{.pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad1,
                                         .tx_pin = {'A', 4, PinFunction::d},
                                         .rx_pin = {'A', 5, PinFunction::d}},
                                .baud = 60503, .pinout = UartPinout::flow_control};
static_assert(sercom_uart_config_valid(rs485) && sercom_uart_config_valid(flow));
static_assert(sercom_uart_txpo(rs485) == 3 && sercom_uart_txpo(flow) == 2);
static_assert(sercom_uart_txpo(sync_host) == 1 && sercom_uart_txpo({.pads = pad0_pads}) == 0);
static_assert(uart_control_pad(rs485) == SercomPad::pad2 && !uart_control_pad(sync_host));
static_assert((sercom_uart_ctrla(rs485) & SERCOM_USART_INT_CTRLA_TXPO_Msk) ==
              SERCOM_USART_INT_CTRLA_TXPO(3));
static_assert(sercom_uart_ctrlc(rs485) == SERCOM_USART_INT_CTRLC_GTIME(7));
// TxD on PAD[2] has no such pinout; RxD cannot sit on the output pad.
static_assert(!sercom_uart_config_valid({.pads = link_pads, .pinout = UartPinout::rs485}));
static_assert(!sercom_uart_config_valid(
    {.pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad2, .tx_pin = {'A', 4, PinFunction::d},
              .rx_pin = {'A', 6, PinFunction::d}},
     .pinout = UartPinout::rs485}));
static_assert(!sercom_uart_config_valid(
    {.pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad3, .tx_pin = {'A', 4, PinFunction::d},
              .rx_pin = {'A', 7, PinFunction::d}},
     .pinout = UartPinout::flow_control}));
static_assert(sercom_uart_config_valid(
    {.pads = {.tx = SercomPad::pad0, .rx = SercomPad::pad3, .tx_pin = {'A', 4, PinFunction::d},
              .rx_pin = {'A', 7, PinFunction::d}},
     .pinout = UartPinout::rs485}));
// Handshaking is asynchronous on the internal clock (31.6.3.2).
static_assert(!sercom_uart_config_valid(
    {.pads = flow.pads, .baud = 11, .synchronous = true, .pinout = UartPinout::flow_control}));
static_assert(!sercom_uart_config_valid(
    {.pads = flow.pads, .external_clock = true, .pinout = UartPinout::flow_control}));
// GTIME: three bits, RS-485 alone.
static_assert(!sercom_uart_config_valid({.pads = pad0_pads, .pinout = UartPinout::rs485,
                                         .guard_time = 8}));
static_assert(!sercom_uart_config_valid({.pads = pad0_pads, .guard_time = 1}));

// ---- the LIN families (FORM 0x2, 0x4, 0x5) ----------------------------------
constexpr uint16_t frac19200 = sercom_fractional_baud_reg(24'000'000, 19'200).value();
static_assert(frac19200 == ((1u << 13) | 78u));   // 78.125
constexpr SercomUartConfig lin_host{.pads = pad0_pads, .baud = frac19200,
                                    .sampling = UartSampling::x16_fractional,
                                    .form = UartForm::lin_host, .lin_break = 3,
                                    .lin_header_delay = 2};
constexpr SercomUartConfig lin_client{.pads = link_pads, .baud = frac19200,
                                      .sampling = UartSampling::x16_fractional,
                                      .form = UartForm::auto_baud};
constexpr SercomUartConfig ab_parity{.pads = link_pads, .format = {.parity = UartParity::odd},
                                     .baud = frac19200, .sampling = UartSampling::x16_fractional,
                                     .form = UartForm::auto_baud};
static_assert(sercom_uart_config_valid(lin_host) && sercom_uart_config_valid(lin_client) &&
              sercom_uart_config_valid(ab_parity));
static_assert(sercom_uart_form(lin_host) == 0x2 && sercom_uart_form(lin_client) == 0x4 &&
              sercom_uart_form(ab_parity) == 0x5 && sercom_uart_form(frac) == 0x0 &&
              sercom_uart_form({.pads = link_pads, .format = {.parity = UartParity::even}}) == 0x1);
static_assert(sercom_uart_ctrlc(lin_host) ==
              (SERCOM_USART_INT_CTRLC_BRKLEN(3) | SERCOM_USART_INT_CTRLC_HDRDLY(2)));
static_assert(lin_break_bits(0) == 13 && lin_break_bits(1) == 17 && lin_break_bits(2) == 21 &&
              lin_break_bits(3) == 26);
static_assert(lin_break_to_sync_bits(0) == 1 && lin_break_to_sync_bits(3) == 14 &&
              lin_sync_to_id_gap_bits(0) == 0 && lin_sync_to_id_gap_bits(2) == 7 &&
              lin_sync_to_id_gap_bits(3) == 13);
// 16x fractional, asynchronous; no parity for the host; BRKLEN/HDRDLY under it alone.
static_assert(!sercom_uart_config_valid({.pads = link_pads, .baud = 60503,
                                         .form = UartForm::auto_baud}));
static_assert(!sercom_uart_config_valid({.pads = link_pads, .baud = 400,
                                         .sampling = UartSampling::x8_fractional,
                                         .form = UartForm::lin_host}));
static_assert(!sercom_uart_config_valid({.pads = link_pads, .format = {.parity = UartParity::even},
                                         .baud = frac19200,
                                         .sampling = UartSampling::x16_fractional,
                                         .form = UartForm::lin_host}));
static_assert(!sercom_uart_config_valid({.pads = link_pads, .baud = frac19200,
                                         .sampling = UartSampling::x16_fractional,
                                         .lin_break = 1}));
static_assert(!sercom_uart_config_valid({.pads = pad0_pads, .baud = frac19200,
                                         .sampling = UartSampling::x16_fractional,
                                         .pinout = UartPinout::rs485,
                                         .form = UartForm::auto_baud}));
// Erratum 1.17.2: revisions B..E (1..4) do not flag a missing stop bit.
static_assert(!sercom_autobaud_checks_stop(1) && !sercom_autobaud_checks_stop(4) &&
              sercom_autobaud_checks_stop(5));

// ---- collision detection and start-of-frame detection -----------------------
constexpr SercomUartConfig one_wire{.pads = pad0_pads, .baud = 60503, .collision_detect = true};
static_assert(sercom_uart_config_valid(one_wire));
static_assert((sercom_uart_ctrlb(one_wire) & SERCOM_USART_INT_CTRLB_COLDEN_Msk) != 0);
static_assert(!sercom_uart_config_valid({.pads = pad0_pads, .rx = false,
                                         .collision_detect = true}));
// Erratum 1.17.12: the APB clock higher than the core clock.
static_assert(sercom_uart_clocks_valid(one_wire, {48'000'000, 24'000'000}));
static_assert(!sercom_uart_clocks_valid(one_wire, {48'000'000, 48'000'000}));
static_assert(sercom_uart_clocks_valid(frac, {24'000'000, 48'000'000}));   // no COLDEN, no rule
constexpr SercomUartConfig sof{.pads = link_pads, .baud = 60503,
                               .start_of_frame = UartStartOfFrame::wake_on_rxs};
static_assert((sercom_uart_ctrlb(sof) & SERCOM_USART_INT_CTRLB_SFDE_Msk) != 0);
static_assert(sercom_uart_sof_armed(sof) == SercomFlag::rxs);
static_assert(sercom_uart_sof_armed({.start_of_frame = UartStartOfFrame::wake_on_both}) ==
              (SercomFlag::rxs | SercomFlag::rxc));
static_assert(sercom_uart_sof_armed(frac) == 0 &&
              (sercom_uart_ctrlb(frac) & SERCOM_USART_INT_CTRLB_SFDE_Msk) == 0);

void link_resource_verbs() {
    using Sc = Sercom<1>;
    (void)Sc::configure<sync_host>();
    (void)Sc::configure<sync_client>();
    (void)Sc::configure<ir>();
    (void)Sc::configure<frac>();
    (void)Sc::configure<rs485>();
    (void)Sc::configure<flow>();
    (void)Sc::configure<lin_host>();
    (void)Sc::configure<lin_client>();
    (void)Sc::configure<sof>();
    (void)Sc::configure<one_wire, SercomClocks{48'000'000, 24'000'000}>();
    (void)Sc::configure(one_wire, SercomClocks{48'000'000, 24'000'000});
    (void)Sc::configure(one_wire);   // refused at run time: no clocks stated
    (void)Sc::lin_command(LinCommand::header_on_write);
    (void)Sc::lin_command(LinCommand::break_on_write);
    (void)Sc::enable_tx(false);
    (void)Sc::enable_tx(true);
    (void)Sc::enable_rx(false);
    (void)Sc::enable_rx(true, 100);
    (void)Sc::tx_enabled();
    (void)Sc::rx_enabled();
}

using Serial = Uart<0, pads>;
static_assert(ByteTransport<Serial> && BulkSink<Serial> && SpanSource<Serial>);
static_assert(ClockUser<Serial>);
static_assert(Serial::generator == 0);
static_assert(Serial::min_hz_for(115200) == 1'843'200);
static_assert(Serial::can_baud(48'000'000, 115200));
static_assert(!Serial::can_baud(1'000'000, 115200));
void serial_set_baud() { (void)Serial::set_baud(48'000'000u, 9'600u); }

// ---- the register words -----------------------------------------------------
constexpr SercomUartConfig console_cfg{
    .pads = pads,
    .format = {.bits = UartBits::eight, .parity = UartParity::none},
    .baud = 63019,
};
static_assert((sercom_uart_ctrla(console_cfg) & SERCOM_USART_INT_CTRLA_MODE_Msk) ==
              SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK);
static_assert((sercom_uart_ctrla(console_cfg) & SERCOM_USART_INT_CTRLA_ENABLE_Msk) == 0);
// A DEFAULT frame must be LSB-first: DORD's reset value is MSB-first
// and a defaulted-false lsb_first shipped every byte bit-reversed
// (bench-caught on the first live banner).
static_assert((sercom_uart_ctrla({.pads = pads, .baud = 63019}) &
               SERCOM_USART_INT_CTRLA_DORD_Msk) != 0);
static_assert((sercom_uart_ctrlb(console_cfg) &
               (SERCOM_USART_INT_CTRLB_TXEN_Msk | SERCOM_USART_INT_CTRLB_RXEN_Msk)) ==
              (SERCOM_USART_INT_CTRLB_TXEN_Msk | SERCOM_USART_INT_CTRLB_RXEN_Msk));

void resource_verbs() {
    using Sc = Sercom<0>;
    static_assert(Sc::index == 0);
    static_assert(Sc::gclk_core_id() == SERCOM0_GCLK_ID_CORE);
    static_assert(Sc::apb_mask() == MCLK_APBCMASK_SERCOM0_Msk);
    static_assert(Sc::irq() == SERCOM0_IRQn);

    Sc::bus_clock(true);
    (void)Sc::core_clock(0);
    (void)Sc::reset();
    (void)Sc::configure(console_cfg);
    (void)Sc::enable(true);
    (void)Sc::enabled();
    (void)Sc::sync_busy(SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk);
    (void)Sc::wait_sync(SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk);

    (void)Sc::baud_reg();
    Sc::baud_reg(63019);

    (void)Sc::pending();
    (void)Sc::flags();
    (void)Sc::armed();
    Sc::clear_flags(SercomFlag::txc);
    Sc::enable_interrupt(SercomFlag::rxc | SercomFlag::dre, true);
    Sc::enable_dre_interrupt(false);
    Sc::enable_rxc_interrupt(true);
    Sc::enable_txc_interrupt(false);
    (void)Sc::dre_flag();
    (void)Sc::rxc_flag();
    (void)Sc::txc_flag();

    (void)Sc::status();
    Sc::clear_status(SercomStatus::receive_errors);
    (void)Sc::data();
    Sc::data(0x55);
    Sc::flush_rx();
    Sc::release();

    // The highest instance this device has, whichever that is.
    using Last = Sercom<sercom_count - 1>;
    (void)Last::irq();
    (void)Last::gclk_core_id();
}

void task_verbs() {
    constexpr SysClock clock;
    (void)Serial::init(clock, 115200);
    (void)Serial::init(clock, 9600, {.bits = UartBits::seven, .parity = UartParity::even,
                                     .two_stop = true});
    Serial::rebase(24'000'000);
    (void)Serial::actual_baud(48'000'000);

    (void)Serial::isr();

    uint8_t byte = 0;
    (void)Serial::write_byte('x');
    (void)Serial::read_byte(byte);
    const uint8_t text[] = {'h', 'i'};
    (void)Serial::write_bulk(text);
    (void)Serial::read_span();
    Serial::consume(1);

    (void)Serial::rx_pending();
    (void)Serial::tx_idle();
    (void)Serial::rx_overruns();
    (void)Serial::frame_errors();
    (void)Serial::parity_errors();
    (void)Serial::hw_overruns();
    Serial::clear_errors();
    Serial::release();

    // A second instantiation on the same instance but the other legal
    // TxD pad, with rings of a size that takes the guarded path on the
    // AVR: this core reads a word atomically, so the lock-free path holds
    // at any size here.
    constexpr UartPads wide_pads{
        .tx = SercomPad::pad2,
        .rx = SercomPad::pad3,
        .tx_pin = {'A', 6, PinFunction::d},
        .rx_pin = {'A', 7, PinFunction::d},
    };
    using Wide = Uart<0, wide_pads, 1024, 1024>;
    static_assert(Ring<uint8_t, 1024, SamPlatform>::lock_free);
    (void)Wide::init(clock, 460800);

    // The loop-back (31.6.3.8): the receiver on its own transmitter's pad,
    // one pin named for both directions.
    constexpr UartPads loop_pads{
        .tx = SercomPad::pad0,
        .rx = SercomPad::pad0,
        .tx_pin = {'A', 4, PinFunction::d},
        .rx_pin = {'A', 4, PinFunction::d},
    };
    using Loop = Uart<0, loop_pads>;
    (void)Loop::init(clock, 3'000'000);
    (void)Loop::tx_idle();
    Loop::release();
}

// ---- the optional DMA engine -------------------------------------------------
// The engine lives in samc21/dmac.hpp, not in sercom.hpp: including the
// DMAC from the SERCOM would give every program with a serial port the
// descriptor sections. So an application that wants it includes both
// headers and names it, and the Uart takes it in its ONE slot - the
// transmit one - which DEFAULTS to NoDmaEngine, so every use above
// compiles unchanged. A receive slot does not exist
// (neg/uart_receive_engine_slot.cpp).
#include "samc21/dmac.hpp"

using TxEngined = Uart<0, pads, 64, 256, DmaTxEngine>;

static_assert(!NoDmaEngine::present);
static_assert(!Serial::has_tx_engine && TxEngined::has_tx_engine);
static_assert(DmaTxEngine::present && DmaTxEngine::channel == 0);
// The concepts hold whether or not the engine is named.
static_assert(ByteTransport<TxEngined> && BulkSink<TxEngined> && SkippingSource<TxEngined> &&
              ClockUser<TxEngined>);
// The refused claim's breadcrumb names the refused owner.
static_assert(dma_claim_refused_context(6) == 0xD6);

void engined_uart_verbs() {
    constexpr SysClock clock;
    (void)TxEngined::init(clock, 9600);
    (void)TxEngined::isr();          // the receiver's RXC, still an interrupt
    TxEngined::dma_isr();            // the DMAC's one vector
    (void)TxEngined::write_byte('x');
    (void)TxEngined::write_bulk(std::span<const uint8_t>());
    (void)TxEngined::dma_faults();
    (void)TxEngined::tx_idle();
    TxEngined::release();
    // The claim, as a verb: one owner at a time.
    (void)DmaTxEngine::claim(1);
    (void)DmaTxEngine::owner();
    DmaTxEngine::release(1);
    (void)Serial::dma_faults();      // engineless: 0, and free
}

// ---- the placement (util/placement.hpp) -----------------------------------------
// The engined port's last parameter: InRam places the transmit engine's
// completion (dma_isr()) as ONE instance in `.fast_text.brio`; the per-byte
// isr() stays inline in the vector, which the program places itself - a
// `.fast_text` function in the same TU as the COMDAT instance (the
// section-type trap util/placement.hpp states). An unknown tag, and InRam
// on a port with no engine, are refused (neg/uart_placement_unknown.cpp,
// neg/uart_in_ram_without_engine.cpp).
using RamEngined = Uart<0, pads, 64, 256, DmaTxEngine, InRam>;
static_assert(RamEngined::has_tx_engine);

extern "C" [[gnu::section(".fast_text")]] void fixture_placed_sercom_vector() {
    (void)RamEngined::isr();
    (void)Serial::isr();
}

void ram_engined_uart_verbs() {
    constexpr SysClock clock;
    (void)RamEngined::init(clock, 9600);
    RamEngined::dma_isr();           // the completion, in SRAM
    (void)RamEngined::write_byte('x');
    fixture_placed_sercom_vector();
    RamEngined::release();
}

// ---- the ring's bulk API, which the TX engine drains through ----------------
// Added for the engines and used by them: the contiguous run each side
// owns, so a whole block goes out in one transfer instead of one byte
// per interrupt. design/ring.md carries the contract.
void ring_span_verbs() {
    static Ring<uint8_t, 256, SamPlatform> r;
    const auto w = r.write_span();
    if (!w.empty()) {
        w[0] = 0x5A;
        r.publish(1);
    }
    const auto rd = r.read_span();
    r.consume(static_cast<uint8_t>(rd.size()));
}
