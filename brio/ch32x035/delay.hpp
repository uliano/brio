/*
 * delay.hpp
 *
 * A wait shorter than a tick, for a driver's settling time or a bus's
 * setup time: a microsecond busy-wait on the STK counter, which spins
 * the calling context for the time asked and returns. `delay_us`,
 * `delay_rate` and `delay_rates` are the QingKe core stratum's
 * (qingke/delay.hpp, whose account they are, the measurement on the
 * CH32X035F8U6 among them), over this family's device header.
 *
 *     (void)brio::delay_us(clock, 200);   // at least 200 us, the clock task's tag
 *
 * `clock` is the clock task's tag (ch32x035/clock.hpp). This family's
 * facts: the counter is the QingKe V4's 64-bit one, of which the wait
 * reads the low half; below 1 MHz of HCLK - the ladder's three lowest
 * rungs - the factor rounds up to one cycle per microsecond, so the
 * wait is longer than asked, never shorter.
 */

#pragma once

#include "ch32x035/device.hpp"
#include "qingke/delay.hpp"
