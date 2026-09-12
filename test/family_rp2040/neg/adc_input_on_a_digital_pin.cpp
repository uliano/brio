// Must FAIL: GPIO 25 is no ADC input (only 26..29 are).
#include "rp2040/adc.hpp"

void up() { brio::AnalogIn<brio::Pin<25>>::claim(); }
