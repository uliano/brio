/*
 * delay.hpp
 *
 * Microsecond busy-waits on the STK counter of the CH32V006 and the
 * CH32V003, for a wait shorter than a tick: a pad's settling time, a
 * peripheral's start-up. `delay_us`, `delay_rate` and `delay_rates` are
 * the QingKe core stratum's (qingke/delay.hpp, whose account they are,
 * the measurements on the CH32V006K8U6 among them), over this family's
 * device header. The smallest use, after the clock task and the ticker
 * are initialized:
 *
 *     if (!brio::delay_us(clock, 100)) { ... }   // refused: no time spent
 *
 * `clock` is the clock task's tag (ch32v00x/clock.hpp). This family's
 * facts: the counter is the QingKe V2's 32-bit one, and the core has no
 * divide instruction, which is why no division runs at wait time.
 */

#pragma once

#include "ch32v00x/device.hpp"
#include "qingke/delay.hpp"
