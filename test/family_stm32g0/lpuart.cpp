// LPUART family smoke TU (RM0444 ch. 34): the Lpuart<n> resource, the
// shared UartTask above it under the name LpUart, and the baud
// arithmetic between them - on LPUART1, which every G0 has; LPUART2 is
// the negatives' business. The pads are PC1/PC0 (LPUART1 TX/RX on AF1,
// DS13560 table 17); PA2/PA3 AF6 are the same instance on the console's
// own pads, which is what the bench suite's console-swap letter uses.
#include "stm32g0/clock.hpp"
#include "stm32g0/lpuart.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 64'000'000>;

static_assert(lpuart_present(1));
static_assert(!lpuart_present(0) && !lpuart_present(3));
static_assert(!lpuart_bus_clock(1).apb2 && lpuart_bus_clock(1).mask == RCC_APBENR1_LPUART1EN);
static_assert(lpuart_clock_select_pos(1) == RCC_CCIPR_LPUART1SEL_Pos);
static_assert(lpuart_exti_line(1) == 28);
#if defined(STM32G0B1xx)
static_assert(lpuart_present(2));
static_assert(!lpuart_bus_clock(2).apb2 && lpuart_bus_clock(2).mask == RCC_APBENR1_LPUART2EN);
static_assert(lpuart_clock_select_pos(2) == RCC_CCIPR_LPUART2SEL_Pos);
static_assert(lpuart_exti_line(2) == 35);
static_assert(lpuart_irq(2) == USART2_LPUART2_IRQn);
static_assert(lpuart_irq(1) == USART3_4_5_6_LPUART1_IRQn);
#elif defined(STM32G071xx)
static_assert(!lpuart_present(2));
static_assert(lpuart_irq(1) == USART3_4_LPUART1_IRQn);
#else
static_assert(!lpuart_present(2));
static_assert(lpuart_irq(1) == LPUART1_IRQn);
#endif

// 34.4.7: LPUARTDIV = 256 x fck / baud, floor 0x300, twenty bits - and
// the two rules of that section are ONE rule, since 256 x 3 = 0x300 and
// 256 x 4096 is one past the field.
static_assert(lpuart_div_min == 0x300 && lpuart_div_max == 0xFFFFF);
static_assert(lpuart_min_hz(9600) == 28800);
static_assert(lpuart_max_hz(9600) == 39'321'600);

// TABLE 198 (lpuart_ker_ck_pres = 32.768 kHz) and TABLE 199 (100 MHz),
// with brio's own value beside the manual's. THE MANUAL TRUNCATES and
// this driver rounds to nearest, so six of the sixteen rows differ by
// one - always in brio's favour, and the second static_assert of each
// pair is the manual's own number, kept so the difference is a
// documented fact and not a drift.
static_assert(lpuart_brr(32768, 300).value() == 0x6D3A);     // manual 0x6D3A
static_assert(lpuart_brr(32768, 600).value() == 0x369D);     // manual 0x369D
static_assert(lpuart_brr(32768, 1200).value() == 6991);      // manual 0x1B4E = 6990
static_assert(lpuart_brr(32768, 2400).value() == 0xDA7);     // manual 0xDA7
static_assert(lpuart_brr(32768, 4800).value() == 1748);      // manual 0x6D3 = 1747
static_assert(lpuart_brr(32768, 9600).value() == 874);       // manual 0x369 = 873
static_assert(lpuart_brr(100'000'000, 38400).value() == 666667);   // manual 666666
static_assert(lpuart_brr(100'000'000, 57600).value() == 0x6C81C);
static_assert(lpuart_brr(100'000'000, 115200).value() == 0x3640E);
static_assert(lpuart_brr(100'000'000, 230400).value() == 0x1B207);
static_assert(lpuart_brr(100'000'000, 460800).value() == 55556);    // manual 55555
static_assert(lpuart_brr(100'000'000, 921600).value() == 27778);    // manual 27777
static_assert(lpuart_brr(100'000'000, 4'000'000).value() == 0x1900);
static_assert(lpuart_brr(100'000'000, 10'000'000).value() == 0xA00);
static_assert(lpuart_brr(100'000'000, 20'000'000).value() == 0x500);
static_assert(lpuart_brr(100'000'000, 33'000'000).value() == 776);  // manual 0x307 = 775

// The window, both ends. 9600 IS the LSE's ceiling (34.4.7 says so in
// as many words) and 19200 is past it; 64 MHz / 3 is 21.3 Mbaud.
static_assert(!lpuart_brr(32768, 19200).has_value());
static_assert(lpuart_brr(64'000'000, 21'333'333).has_value());
static_assert(!lpuart_brr(64'000'000, 22'000'000).has_value());
static_assert(!lpuart_brr(64'000'000, 100).has_value());   // past twenty bits
static_assert(!lpuart_brr(0, 9600).has_value());
static_assert(!lpuart_brr(32768, 0).has_value());
static_assert(lpuart_actual_baud(32768, 874) == 9597);
static_assert(lpuart_actual_baud(32768, 873) == 9608);     // the manual's own row
static_assert(lpuart_actual_baud(32768, 0x2FF) == 0);      // below the floor: refused

static_assert(Lpuart<1>::is_lpuart);
static_assert(!Lpuart<1>::is_full);            // it is LP, not FULL
static_assert(Lpuart<1>::has_prescaler);       // but PRESC is there
static_assert(Lpuart<1>::has_clock_select);    // and so is the multiplexer
static_assert(!Lpuart<1>::has_oversampling8);
static_assert(!Lpuart<1>::has_synchronous_mode && !Lpuart<1>::has_receiver_timeout);
static_assert(!Lpuart<1>::has_lin_mode);
static_assert(Lpuart<1>::fifo_depth == 8);
// Table 55 calls LPUART1's two rows `LPUART_RX` and `LPUART_TX` with no
// index at all, at 14 and 15 - not with the USARTs.
static_assert(Lpuart<1>::dma_rx_request() == 14 && Lpuart<1>::dma_tx_request() == 15);
#if defined(STM32G0B1xx)
static_assert(Lpuart<2>::dma_rx_request() == 64 && Lpuart<2>::dma_tx_request() == 65);
#endif

constexpr UartPins lp1{.tx = {'C', 1, PinFunction::af1},
                       .rx = {'C', 0, PinFunction::af1}};
constexpr UartOptions lse_opts{.kernel_clock = UsartClock::lse};
constexpr UartOptions hsi_fifo{.kernel_clock = UsartClock::hsi16, .fifo = true,
                               .rx_threshold = UartFifoThreshold::half};
constexpr UartOptions wake_opts{.kernel_clock = UsartClock::lse,
                                .wake_from_stop = UsartWakeSource::receive_ready};

using Lp = LpUart<1, lp1>;
using LpLse = LpUart<1, lp1, 64, 64, NoDmaEngine, NoDmaEngine, lse_opts>;
using LpFifo = LpUart<1, lp1, 64, 64, NoDmaEngine, NoDmaEngine, hsi_fifo>;
using LpWake = LpUart<1, lp1, 64, 64, NoDmaEngine, NoDmaEngine, wake_opts>;
static_assert(LpLse::kernel_hz<SysClock>() == 32768);
static_assert(LpFifo::kernel_hz<SysClock>() == 16'000'000);
static_assert(LpLse::min_hz_for(9600) == 28800);

void lpuart_verbs() {
    constexpr SysClock clock;
    (void)Lp::init(clock, 115200);
    (void)LpLse::init(clock, 9600);
    (void)LpFifo::init(clock, 115200);
    (void)LpWake::init(clock, 9600);
    (void)Lp::isr();
    (void)LpFifo::isr();
    (void)LpWake::wakes();
    (void)Lp::write_byte(0x55);
    uint8_t b = 0;
    (void)Lp::read_byte(b);
    const uint8_t buf[4] = {1, 2, 3, 4};
    (void)Lp::write(buf, 4);
    (void)Lp::write_bulk(buf);
    uint8_t out[8];
    (void)Lp::read_bulk(out);
    (void)Lp::rx_pending();
    (void)Lp::tx_idle();
    (void)Lp::hw_overruns();
    Lp::clear_errors();
    (void)Lp::actual_baud(SysClock::pclk_hz);
    (void)Lp::can_baud(SysClock::pclk_hz, 9600);
    (void)Lp::set_baud(SysClock::pclk_hz, 9600);
    Lp::rebase(16'000'000);
    LpLse::rebase(16'000'000);   // a kernel clock that does not follow: a no-op
    Lp::release();
    LpLse::release();
    LpFifo::release();
    LpWake::release();

    using L = Lpuart<1>;
    L::bus_clock(true);
    (void)L::kernel_clock(UsartClock::lse);
    (void)L::kernel_clock();
    (void)L::configure({}, 0x369);
    (void)L::brr();
    L::set_brr(0x369);
    L::enable(true);
    (void)L::enabled();
    (void)L::has_fifo();
    (void)L::has_half_duplex();
    (void)L::has_flow_control();
    (void)L::has_driver_enable();
    (void)L::has_wake_from_stop();
    (void)L::has_autobaud();
    (void)L::has_lin();
    (void)L::has_irda();
    (void)L::has_smartcard();
    (void)L::has_synchronous();
    (void)L::oversampling(false);
    (void)L::oversampling();
    (void)L::fifo(true);
    (void)L::fifo();
    (void)L::fifo_thresholds(UartFifoThreshold::half, UartFifoThreshold::none);
    (void)L::prescaler(UsartPrescaler::div4);
    (void)L::prescaler();
    (void)L::swap(true);
    (void)L::swap();
    (void)L::invert(true, true, true);
    (void)L::msb_first(true);
    (void)L::half_duplex(true);
    (void)L::half_duplex();
    (void)L::one_bit_sampling(false);
    (void)L::overrun_disable(true);
    (void)L::driver_enable({.assertion = 1, .deassertion = 1});
    (void)L::driver_enable_off();
    (void)L::flow_control(true, true);
    (void)L::mute_mode({.wake = MuteWake::idle_line});
    (void)L::mute_mode_off();
    (void)L::muted();
    (void)L::character_match('\n');
    (void)L::wake_from_stop(UsartWakeSource::start_bit);
    (void)L::wake_line(true);
    (void)L::wake_line();
    L::request(UsartRequest::flush_transmit);
    L::send_break();
    (void)L::status();
    (void)L::flag(UsartFlag::wuf);
    L::clear_flags(UsartClear::all);
    (void)L::read_data();
    (void)L::read_word();
    L::write_data(0);
    L::write_word(0x100);
    (void)L::tx_data_address();
    (void)L::rx_data_address();
    (void)L::dma_transmit(true);
    (void)L::dma_receive(true);
    L::rxne_interrupt(true);
    L::txe_interrupt(true);
    (void)L::txe_interrupt();
    L::interrupts(UsartInterrupt::idle, true);
    (void)L::interrupts();
    L::error_interrupt(true);
    L::cts_interrupt(true);
    L::rx_threshold_interrupt(true);
    L::tx_threshold_interrupt(true);
    (void)L::brr_for(32768, 9600);
    (void)L::baud_for(32768, 874);
    (void)L::min_hz_for(9600);
    (void)L::irq();
    L::reset();
}
