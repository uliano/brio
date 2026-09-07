// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// An LSI-clocked LptimTicker takes the rate the program states, and a
// statement outside DS13560 table 46's 29.5..34 kHz is not this
// oscillator: refused where the config is spelled.
#include "stm32g0/lptim_ticker.hpp"
using Bad = brio::LptimTicker<brio::LptimTickerConfig{
    .source = brio::LptimTickerSource::lsi, .lsi_hz = 36'000}>;
uint32_t f() { return Bad::ticks(); }
