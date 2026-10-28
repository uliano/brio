/*
 * delay.hpp
 *
 * A wait shorter than a tick, for a driver's settling time or a bus's
 * setup time: a microsecond busy-wait on the STK counter of the CH32V203
 * and the CH32V303, which spins the calling context for the time asked
 * and returns. `delay_us`, `delay_rate` and `delay_rates` are the QingKe
 * core stratum's (qingke/delay.hpp, whose account they are, the
 * measurements on the two bench parts among them), over this family's
 * device header. A call, half a millisecond:
 *
 *     (void)delay_us(clock, 500u);
 *
 * `clock` is the clock task's tag (ch32vx03/clock.hpp). This family's
 * facts: the counter is the QingKe V4's 64-bit one, of which the wait
 * reads the low half; the core has the M extension, and still no
 * division runs at wait time.
 */

#pragma once

#include "ch32vx03/device.hpp"
#include "qingke/delay.hpp"
