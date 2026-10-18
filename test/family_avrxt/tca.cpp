// TCA family smoke TU: every package must compile this. The route
// table comes from the device header (tca_route_code mirrors its
// PORTMUX codes); the split-mode escape (reset with CMDEN), the CTRLC
// verbs and the split flag/interrupt surface must instantiate
// everywhere.
#include "avrxt/tca.hpp"
#include "avrxt/evsys.hpp"

using namespace brio;

void tca_common() {
    Tca<0>::init<TcaConfig{.mode = TcaMode::single_slope, .period = 999,
                           .compare0 = 250, .outputs = 0x01, .route = 'C'}>();
    Tca<0>::output_value<0>(true);
    (void)Tca<0>::output_value<0>();
    Tca<0>::reset();                          // CMDEN both: works from split too
    TcaPwm<0, 'C'>::init();
    TcaPwm<0, 'C'>::duty<3>(128);
    Tca<0>::enable_hunf_interrupt(true);
    (void)Tca<0>::hunf_flag(); Tca<0>::clear_hunf();
    (void)Tca<0>::lcmp_flag<1>(); Tca<0>::enable_lcmp_interrupt<1>(false);
    EventChannel<1>::source(Tca<0>::HunfEvent{});
    (void)TcaPwm16<0, 'D', 24000>::max;
    (void)TcaPwmCentered<0, 'D', 12000>::init();
    TcaPwmCentered<0, 'D', 12000>::duty<0>(3000);
    FrequencyGenerator<0, 'D'>::rebase(12'000'000);
}

#if defined(__AVR_AVR128DB48__) || defined(__AVR_AVR128DB64__) || \
    defined(__AVR_AVR128DA48__) || defined(__AVR_AVR128DA64__)
void tca1_exists() {
    Tca<1>::init<TcaConfig{.mode = TcaMode::normal, .period = 0xFFFF, .route = 'B'}>();
    static_assert(tca_route_code(1, 'C') != 0xFF);      // three channels, pins 4..6
    static_assert(tca_wo_pin(1, 'C', 2) == 6);
    (void)TcaPwmCentered<1, 'C', 24000>::init();        // centered PWM on a three-channel route
}
#endif

#if defined(__AVR_AVR128DB64__)
// 64-pin: the PORTG routes and TCA1 -> PORTE exist (device header).
void tca_big_package() {
    Tca<0>::init<TcaConfig{.mode = TcaMode::normal, .period = 100, .route = 'G'}>();
    TcaPwm<1, 'G'>::init();
    static_assert(tca_route_code(1, 'E') != 0xFF);
    static_assert(tca_wo_pin(1, 'E', 0) == 4);          // WO0 = PE4
}
#endif

#if defined(__AVR_AVR128DA64__)
// 64-pin DA: TCA0's PORTG route exists; TCA1's PORTE and PORTG, which the
// header defines, are refused (DS80000882C 2.11.1: not functional).
void tca_big_package() {
    Tca<0>::init<TcaConfig{.mode = TcaMode::normal, .period = 100, .route = 'G'}>();
    static_assert(tca_route_code(1, 'E') == 0xFF);
    static_assert(tca_route_code(1, 'G') == 0xFF);
}
#endif

#if defined(__AVR_AVR128DB28__) || defined(__AVR_AVR128DB32__) || \
    defined(__AVR_AVR128DA28__) || defined(__AVR_AVR128DA32__)
// 28/32-pin: no PORTB/PORTE/PORTG routes at all (header omits them).
static_assert(tca_route_code(0, 'B') == 0xFF);
static_assert(tca_route_code(0, 'E') == 0xFF);
static_assert(tca_route_code(0, 'G') == 0xFF);
#endif

#if defined(__AVR_AVR64EA28__) || defined(__AVR_AVR64EA32__) || defined(__AVR_AVR64EA48__)
// The AVR EA: TCA1 on every package, its PORTA and PORTD routes three
// channels on pins 4..6 (DS40002443A 17.3.7); PORTB (six) and PORTC
// (three) on the 48-pin part only - the 28/32-pin header keeps PORTB's
// code as "not connected" and names no PORTC.
static_assert(tca_route(1, 'A').channels == 3 && tca_route(1, 'A').first_pin == 4);
static_assert(tca_route(1, 'D').channels == 3 && tca_wo_pin(1, 'D', 2) == 6);
static_assert(tca_route_code(1, 'A') == PORTMUX_TCA1_PORTA_gc);
static_assert(tca_route_code(1, 'D') == PORTMUX_TCA1_PORTD_gc);
static_assert(tca_route(0, 'A').channels == 6 && tca_wo_pin(0, 'A', 5) == 5);
static_assert(tca_route_code(1, 'E') == 0xFF && tca_route_code(1, 'G') == 0xFF);
void tca_ea() {
    Tca<1>::init<TcaConfig{.mode = TcaMode::single_slope, .period = 999, .outputs = 0x07,
                           .route = 'A'}>();
    (void)TcaPwmCentered<1, 'D', 12000>::init();
    TcaPwmCentered<1, 'D', 12000>::duty<2>(3000);
    (void)TcaPwm16<1, 'A', 20000>::max;
    FrequencyGenerator<1, 'D'>::rebase(10'000'000);
    EventChannel<0>::source(Tca<1>::OvfEvent{});
}
#if defined(__AVR_AVR64EA48__)
static_assert(tca_route(1, 'B').channels == 6);
static_assert(tca_route(1, 'C').channels == 3 && tca_wo_pin(1, 'C', 0) == 4);
#else
static_assert(tca_route_code(1, 'B') == 0xFF && tca_route_code(1, 'C') == 0xFF);
#endif
#else
// The AVR DA/DB: no TCA1 route to PORTA or PORTD on any package.
static_assert(tca_route_code(1, 'A') == 0xFF && tca_route_code(1, 'D') == 0xFF);
#endif
