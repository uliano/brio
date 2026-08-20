// TCA family smoke TU: every package must compile this. The route
// table comes from the device header (tca_route_code mirrors its
// PORTMUX codes); the split-mode escape (reset with CMDEN), the CTRLC
// verbs and the split flag/interrupt surface must instantiate
// everywhere.
#include "avrdx/tca.hpp"
#include "avrdx/evsys.hpp"

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

#if defined(__AVR_AVR128DB64__) || defined(__AVR_AVR128DA64__)
// 64-pin: the PORTG routes and TCA1 -> PORTE exist (device header).
void tca_big_package() {
    Tca<0>::init<TcaConfig{.mode = TcaMode::normal, .period = 100, .route = 'G'}>();
    TcaPwm<1, 'G'>::init();
    static_assert(tca_route_code(1, 'E') != 0xFF);
    static_assert(tca_wo_pin(1, 'E', 0) == 4);          // WO0 = PE4
}
#endif

#if defined(__AVR_AVR128DB28__) || defined(__AVR_AVR128DB32__) || \
    defined(__AVR_AVR128DA28__) || defined(__AVR_AVR128DA32__)
// 28/32-pin: no PORTB/PORTE/PORTG routes at all (header omits them).
static_assert(tca_route_code(0, 'B') == 0xFF);
static_assert(tca_route_code(0, 'E') == 0xFF);
static_assert(tca_route_code(0, 'G') == 0xFF);
#endif
