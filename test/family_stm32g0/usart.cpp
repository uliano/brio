// USART family smoke TU: the Usart<n> resource, the Uart task above it
// and the baud arithmetic between them - on USART1 and USART2, which
// every G0 has; the higher instances are the negatives' business. The
// pads are PA9/PA10 (USART1 AF1) and PA2/PA3 (USART2 AF1), DS13560
// table 13 - bonded on every package that carries these ports.
#include "stm32g0/clock.hpp"
#include "stm32g0/usart.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 64'000'000>;

static_assert(usart_present(1) && usart_present(2));
static_assert(!usart_present(7) && !usart_present(0));
static_assert(usart_bus_clock(1).apb2 && usart_bus_clock(1).mask == RCC_APBENR2_USART1EN);
static_assert(!usart_bus_clock(2).apb2 && usart_bus_clock(2).mask == RCC_APBENR1_USART2EN);
static_assert(usart_has_clock_select(1));
static_assert(!usart_has_clock_select(6));

// 33.5.7, OVER8 = 0: BRR = USARTDIV = f / baud, nearest, >= 16.
static_assert(usart_brr(64'000'000, 115200).value() == 556);
static_assert(usart_brr(16'000'000, 115200).value() == 139);
static_assert(usart_brr(8'000'000, 9600).value() == 833);      // the chapter's own example
static_assert(usart_brr(48'000'000, 921'600).value() == 52);   // and its second
static_assert(usart_brr(64'000'000, 4'000'000).value() == 16); // USARTDIV floor: legal
static_assert(!usart_brr(64'000'000, 5'000'000).has_value());  // below 16: refused
static_assert(!usart_brr(0, 115200).has_value());
static_assert(!usart_brr(64'000'000, 0).has_value());
static_assert(usart_actual_baud(64'000'000, 556) == 115107);
static_assert(usart_actual_baud(16'000'000, 139) == 115107);
static_assert(usart_min_hz(115200) == 1'843'200);

constexpr UartPins u2{.tx = {'A', 2, PinFunction::af1}, .rx = {'A', 3, PinFunction::af1}};
constexpr UartPins u1{.tx = {'A', 9, PinFunction::af1}, .rx = {'A', 10, PinFunction::af1}};
static_assert(uart_pins_valid(u2));
static_assert(!uart_pins_valid({.tx = {'A', 2, PinFunction::af1}, .rx = {'A', 2, PinFunction::af1}}));
static_assert(!uart_pins_valid({.tx = {'G', 2, PinFunction::af1}, .rx = {'A', 3, PinFunction::af1}}));

using Console = Uart<2, u2>;
using Aux = Uart<1, u1, 128, 512>;

void usart_verbs() {
    constexpr SysClock clock;
    (void)Console::init(clock, 115200);
    (void)Aux::init(clock, 9600, {.bits = UartBits::eight, .parity = UartParity::even, .stop_bits = 2});
    (void)Console::isr();
    (void)Aux::isr();
    (void)Console::write_byte(0x55);
    uint8_t b;
    (void)Console::read_byte(b);
    const uint8_t buf[4] = {1, 2, 3, 4};
    (void)Console::write(buf, 4);
    (void)Console::write_bulk(buf);
    uint8_t out[8];
    (void)Console::read_bulk(out);
    (void)Console::rx_pending();
    (void)Console::tx_idle();
    (void)Console::rx_overruns();
    (void)Console::frame_errors();
    (void)Console::parity_errors();
    (void)Console::noise_errors();
    (void)Console::hw_overruns();
    Console::clear_errors();
    (void)Console::actual_baud(SysClock::pclk_hz);
    (void)Console::can_baud(SysClock::pclk_hz, 3'000'000);
    Console::rebase(16'000'000);
    Console::release();

    Usart<2>::bus_clock(true);
    (void)Usart<2>::kernel_clock(UsartClock::pclk);
    (void)Usart<2>::configure({}, 556);
    Usart<2>::enable(true);
    (void)Usart<2>::enabled();
    (void)Usart<2>::status();
    Usart<2>::clear_flags(USART_ICR_ORECF);
    (void)Usart<2>::read_data();
    Usart<2>::write_data(0);
    Usart<2>::rxne_interrupt(false);
    Usart<2>::txe_interrupt(false);
    (void)Usart<2>::txe_interrupt();
    Usart<2>::reset();
    (void)Usart<2>::irq();
}

// ---- the chapter's long tail (campaign 8b) -------------------------------------

// Table 183's FULL/BASIC split, as the reserve states it and as the
// device header's own pointer-comparison macros mean it. The runtime
// has_*() verbs cannot be checked here (they are pointer comparisons);
// the bench suite's letter a does that against the silicon.
static_assert(usart_is_full(1));
static_assert(!usart_is_full(7));
#if defined(STM32G0B1xx)
static_assert(usart_is_full(2) && usart_is_full(3));
static_assert(!usart_is_full(4) && !usart_is_full(5) && !usart_is_full(6));
static_assert(usart_exti_line(1) == 25 && usart_exti_line(2) == 26 &&
              usart_exti_line(3) == 24);
static_assert(usart_exti_line(4) == 0xFF);
#elif defined(STM32G071xx)
static_assert(usart_is_full(2) && !usart_is_full(3) && !usart_is_full(4));
static_assert(usart_exti_line(3) == 0xFF);
#else
static_assert(!usart_is_full(2));
static_assert(usart_exti_line(2) == 0xFF);
#endif
static_assert(Usart<1>::is_full && Usart<1>::has_prescaler);
// The row of table 184 that is NOT the FULL/BASIC split: every USART of
// this family has synchronous mode (the header's IS_USART_INSTANCE lists
// all six, 33.8.3's CLKEN note agrees, and letter a asks the silicon),
// while LIN and the receiver time-out really are the FULL ones' alone.
static_assert(Usart<1>::has_synchronous_mode && Usart<1>::has_lin_mode);
#if defined(STM32G0B1xx)
static_assert(Usart<4>::has_synchronous_mode && !Usart<4>::has_lin_mode);
static_assert(!Usart<4>::has_receiver_timeout && !Usart<4>::has_fifo_mode);
#endif
static_assert(Usart<1>::fifo_depth == 8);
static_assert(!Usart<1>::is_lpuart && Usart<1>::has_oversampling8);

// 33.5.7 in BOTH oversamplings, with the chapter's own two examples.
// OVER8 = 1: BRR[15:4] = USARTDIV[15:4], BRR[2:0] = USARTDIV[3:0] >> 1,
// BRR[3] clear.
static_assert(usart_brr_over8(8'000'000, 9600).value() == 0x681);
static_assert(usart_brr_over8(48'000'000, 921'600).value() == 0x64);
static_assert(usart_brr_over8(64'000'000, 8'000'000).value() == 16);  // the floor
static_assert(!usart_brr_over8(64'000'000, 9'000'000).has_value());
static_assert(usart_actual_baud_over8(48'000'000, 0x64) == 923'076);
static_assert(usart_min_hz_over8(115200) == 921'600);
static_assert(usart_min_hz(115200) == 2u * usart_min_hz_over8(115200));

// 33.8.14's twelve codes and the reserved rest.
static_assert(usart_prescaler_divisor(UsartPrescaler::div1) == 1);
static_assert(usart_prescaler_divisor(UsartPrescaler::div6) == 6);
static_assert(usart_prescaler_divisor(UsartPrescaler::div256) == 256);
static_assert(usart_prescaler_valid(UsartPrescaler::div256));
static_assert(!usart_prescaler_valid(static_cast<UsartPrescaler>(12)));
static_assert(usart_kernel_hz(64'000'000, UsartPrescaler::div16) == 4'000'000);
static_assert(usart_kernel_clock_hz<SysClock>(UsartClock::hsi16) == 16'000'000);
static_assert(usart_kernel_clock_hz<SysClock>(UsartClock::lse) == 32768);
static_assert(usart_kernel_clock_hz<SysClock>(UsartClock::pclk) == SysClock::pclk_hz);

// 33.8.4's threshold codes, and brio's own "leave it alone".
static_assert(uart_fifo_threshold_valid(UartFifoThreshold::full_or_empty));
static_assert(uart_fifo_threshold_valid(UartFifoThreshold::none));
static_assert(!uart_fifo_threshold_valid(static_cast<UartFifoThreshold>(6)));

static_assert(driver_enable_valid({.assertion = 31, .deassertion = 31}));
static_assert(!driver_enable_valid({.assertion = 32}));
static_assert(irda_valid({.prescaler = 1}));
static_assert(!irda_valid({.prescaler = 0}));       // the ENDEC needs PSC != 0
static_assert(smartcard_valid({.retries = 7, .clock_prescaler = 31}));
static_assert(!smartcard_valid({.retries = 8}));
static_assert(!smartcard_valid({.clock_prescaler = 0, .clock_output = true}));

// The options struct and its makers.
constexpr UartOptions fifo_opts{
    .kernel_clock = UsartClock::hsi16,
    .prescaler = UsartPrescaler::div2,
    .fifo = true,
    .rx_threshold = UartFifoThreshold::half,
    .tx_threshold = UartFifoThreshold::full_or_empty,
};
constexpr UartOptions loop_opts{.over8 = true, .half_duplex = true, .one_bit = true};
constexpr UartOptions wake_opts{.kernel_clock = UsartClock::hsi16,
                                .wake_from_stop = UsartWakeSource::start_bit};
static_assert(uart_half_duplex().half_duplex);
static_assert(uart_with_driver_enable({}, {'A', 12, PinFunction::af1}, 3, 4).driver_enable);

using Fifo = Uart<1, u1, 128, 512, NoDmaEngine, NoDmaEngine, fifo_opts>;
using Loop = Uart<1, u1, 64, 64, NoDmaEngine, NoDmaEngine, loop_opts>;
using Waker = Uart<1, u1, 64, 64, NoDmaEngine, NoDmaEngine, wake_opts>;
constexpr PinSel de_pad{'A', 12, PinFunction::af1};   // USART1_RTS_DE, DS13560 table 13
using Rs485Link = Rs485<1, u1, de_pad, 8, 8>;
static_assert(Fifo::kernel_hz<SysClock>() == 8'000'000);
static_assert(Loop::min_hz_for(115200) == 921'600);   // OVER8 halves the floor

constexpr PinSel ck_pad{'A', 8, PinFunction::af0};    // USART1_CK is not bonded here;
                                                      // the pad is a placeholder for a
                                                      // compile check only
using Sync = SyncHost<1, u1, ck_pad>;
using Irda = IrdaLink<1, u1>;
using Ab = AutoBaud<1>;
constexpr PinSel card_pad{'A', 9, PinFunction::af1};
using Card = Smartcard<1, card_pad>;

void usart_tail_verbs() {
    constexpr SysClock clock;
    (void)Fifo::init(clock, 115200);
    (void)Loop::init(clock, 9600);
    (void)Waker::init(clock, 9600);
    (void)Rs485Link::init(clock, 9600);
    (void)Fifo::isr();
    (void)Loop::isr();
    (void)Waker::wakes();
    Loop::rebase(16'000'000);
    (void)Loop::actual_baud(SysClock::pclk_hz);
    (void)Loop::can_baud(SysClock::pclk_hz, 9600);
    (void)Loop::set_baud(SysClock::pclk_hz, 4800);
    Fifo::release();
    Loop::release();
    Waker::release();
    Rs485Link::release();

    using U = Usart<1>;
    (void)U::has_fifo();
    (void)U::has_autobaud();
    (void)U::has_lin();
    (void)U::has_irda();
    (void)U::has_smartcard();
    (void)U::has_half_duplex();
    (void)U::has_flow_control();
    (void)U::has_driver_enable();
    (void)U::has_wake_from_stop();
    (void)U::has_synchronous();       // IS_USART_INSTANCE, the header's own
    (void)U::kernel_clock();
    (void)U::stop_bits(UartStop::one_and_half);
    (void)U::oversampling(true);
    (void)U::oversampling();
    (void)U::prescaler(UsartPrescaler::div8);
    (void)U::prescaler();
    (void)U::fifo(true);
    (void)U::fifo();
    (void)U::fifo_thresholds(UartFifoThreshold::quarter, UartFifoThreshold::none);
    (void)U::swap(true);
    (void)U::swap();
    (void)U::invert(true, false, true);
    (void)U::msb_first(true);
    (void)U::half_duplex(true);
    (void)U::half_duplex();
    (void)U::one_bit_sampling(true);
    (void)U::overrun_disable(true);
    (void)U::flow_control(true, true);
    (void)U::driver_enable({.assertion = 2, .deassertion = 3, .active_low = true});
    (void)U::driver_enable_off();
    (void)U::mute_mode({.wake = MuteWake::address_mark, .address_7bit = true,
                        .address = 0x41});
    (void)U::mute_mode_off();
    (void)U::muted();
    (void)U::character_match('\n');
    (void)U::receiver_timeout_enable(true);
    (void)U::receiver_timeout(22);
    (void)U::receiver_timeout();
    (void)U::block_length(4);
    (void)U::lin({.break_11bit = true, .break_interrupt = true});
    (void)U::lin_off();
    (void)U::synchronous({.clock_idle_high = true, .last_bit_clock = true});
    (void)U::synchronous_off();
    (void)U::smartcard({.retries = 3, .guard_time = 16, .clock_prescaler = 4,
                        .clock_output = true});
    (void)U::smartcard_off();
    U::stop_retries();
    (void)U::irda({.low_power = true, .prescaler = 6});
    (void)U::irda_off();
    (void)U::auto_baud(AutoBaudMode::frame_55);
    (void)U::auto_baud_off();
    U::auto_baud_restart();
    (void)U::wake_from_stop(UsartWakeSource::address_match);
    (void)U::wake_line(true);
    (void)U::guard_time(8);
    U::request(UsartRequest::flush_receive);
    U::send_break();
    (void)U::flag(UsartFlag::lbdf | UsartFlag::cmf | UsartFlag::rtof | UsartFlag::wuf);
    U::clear_flags(UsartClear::all);
    (void)U::read_word();
    U::write_word(0x155);
    U::interrupts(UsartInterrupt::character_match | UsartInterrupt::receiver_timeout,
                  true);
    (void)U::interrupts();
    U::error_interrupt(true);
    U::cts_interrupt(true);
    U::rx_threshold_interrupt(true);
    U::tx_threshold_interrupt(true);
    (void)U::brr_for(64'000'000, 115200);
    (void)U::brr_for_over8(64'000'000, 115200);
    (void)U::baud_for(64'000'000, 556);
    (void)U::baud_for_over8(64'000'000, 0x64);
    (void)U::min_hz_for(9600);
    (void)U::min_hz_for_over8(9600);
    U::set_brr(556);

    (void)Sync::init(clock, 1'000'000, {.clock_idle_high = true});
    (void)Sync::send(0x55);
    (void)Sync::drain();
    Sync::release();

    (void)Irda::init(clock, 9600, {.low_power = false, .prescaler = 1});
    (void)Irda::send(0x00);
    (void)Irda::drain();
    uint32_t flags = 0;
    (void)Irda::receive(flags);
    Irda::release();

    (void)Ab::arm(AutoBaudMode::frame_7f);
    Ab::restart();
    (void)Ab::done();
    (void)Ab::failed();
    (void)Ab::learned_brr();
    (void)Ab::learned_baud(64'000'000);
    (void)Ab::wait(10);
    (void)Ab::disarm();

    (void)Card::init(clock, 10'000, {.retries = 2, .guard_time = 16});
    (void)Card::send(0x3B);
    (void)Card::drain(10);
    (void)Card::complete_before_guard_time();
    (void)Card::receive(flags);
    (void)Card::end_of_block();
    Card::release();
}
