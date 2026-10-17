// The reserve's smoke TU: the part table (die and package, keyed on the
// compiler's device macro) checked against what the device header says
// independently, so a row copied wrong fails here on the part it
// describes; and the feature probes checked against the two families
// they separate today.
#include "avrdx/device_tables.hpp"

using namespace brio;

// The package against the ports and instances the header declares.
static_assert(port_exists('B') == (package_pins >= 48));
static_assert(port_exists('E') == (package_pins >= 48));
static_assert(port_exists('G') == (package_pins == 64));
static_assert(tca_count == (package_pins >= 48 ? 2 : 1));
static_assert(tcb_count == (package_pins == 64 ? 5 : package_pins == 48 ? 4 : 3));
static_assert(usart_count == (package_pins == 64 ? 6 : package_pins == 48 ? 5 : 3));
static_assert(twi_count == (package_pins >= 32 ? 2 : 1));
static_assert(lut_count == (package_pins >= 48 ? 6 : 4));
static_assert(event_channels == (package_pins >= 48 ? 10 : 8));

// The die against the features only one of the two families has.
static_assert(avr_die == AvrDie::avr128da || avr_die == AvrDie::avr128db);
static_assert(has_mvio == (avr_die == AvrDie::avr128db));
static_assert(has_xoschf == (avr_die == AvrDie::avr128db));
static_assert(has_inlvl == (avr_die == AvrDie::avr128db));
static_assert((opamp_count != 0) == (avr_die == AvrDie::avr128db));

// The features both families share.
static_assert(has_pll && has_oschf_frqsel && has_vreg && has_vref_adc0ref && has_tcd);
static_assert(!has_clock_timebase && !has_port_evgen && !has_pit_evgen && !has_adc_pga &&
              !has_nvm_page_buffer);
static_assert(has_rampz && has_flmap);
static_assert(spi_count == 2 && ac_count == 3);
static_assert(has_ac_profile2 && has_ac_upsel2);
static_assert(zcd_count == (package_pins <= 32 ? 1
                            : package_pins == 48 && avr_die == AvrDie::avr128da ? 2 : 3));

// The route fields: two bits per instance, USART4/5 in USARTROUTEB.
static_assert(usart_route_gm(0) == 0x03 && usart_route_gm(1) == 0x0C && usart_route_gm(2) == 0x30);
static_assert(!usart_route_in_b(0) && !usart_route_in_b(1) && !usart_route_in_b(2));
#if defined(USART3)
static_assert(usart_route_gm(3) == 0xC0 && !usart_route_in_b(3));
#endif
#if defined(USART4)
static_assert(usart_route_gm(4) == 0x03 && usart_route_in_b(4));
#endif
#if defined(USART5)
static_assert(usart_route_gm(5) == 0x0C && usart_route_in_b(5));
#endif
static_assert(spi_route_gm(0) == 0x03 && spi_route_gm(1) == 0x0C);
static_assert(twi_route_gm(0) == 0x03);
static_assert(tca_route_gm(0) == 0x07);

// The errata keyed on the die.
static_assert(erratum_da_evsys_pins == (avr_die == AvrDie::avr128da));
static_assert(erratum_da_tca1_alt23 == (avr_die == AvrDie::avr128da));
static_assert(erratum_da_twi_sdahold_swap == (avr_die == AvrDie::avr128da));
static_assert(erratum_da_window == (avr_die == AvrDie::avr128da));
static_assert(usart2_alt1_has_xdir == (avr_die == AvrDie::avr128db));
