/*
 * stk.hpp - the CORE stratum: the QingKe system counter's two widths.
 *
 * The STK is the core's own counter - the CH32V00X reference manual
 * 6.5.4 for the V2, the QingKe V4 manual ch. 5 for the V4 (with the
 * CH32X035's reference manual 7.5.5) - which qingke/ticker.hpp makes the
 * kernel's timebase and qingke/delay.hpp reads for its microsecond
 * wait. Its logic is one on every core: it counts up from zero to a
 * compare value and, with STRE set, restarts from zero there, raising a
 * flag in a status register the handler must clear. Its WIDTH is not:
 * the V2's counter and compare are 32-bit, CNT and CMP; the V4's are
 * 64-bit, each a register pair, CNTL/CNTH and CMPLR/CMPHR. This file
 * tells them apart, and it asks the family's register struct, not a
 * flag: a `StkRegs` with CNTL is the V4's. qingke/ticker.hpp and
 * qingke/delay.hpp ask the concept below and spell both widths'
 * register names in their own verbs, under the struct's type made a
 * template parameter so that only the branch it selects is checked; the
 * registers are read and written in place there, not through a helper
 * that returns the word: with such a helper gcc orders the operands of
 * the code around it otherwise - an equal instruction for an equal one -
 * and these files keep the instruction stream the direct spelling gives.
 *
 * Only the low words are ever read. A period is HCLK/tps - at most
 * 144 000 cycles at the 1000 Hz tick and the fastest clock in view, the
 * CH32V203's and the CH32V303's 144 MHz - far below 2^32, so on the V4
 * the high halves are zeroed when the counter restarts and never move
 * inside one tick.
 *
 * Internal to the core stratum; nothing outside it names the concept.
 */

#pragma once

#include <stdint.h>

static_assert(sizeof(brio::StkRegs) != 0u,
              "qingke/stk.hpp: include the family's device header first - stk() and StkRegs are its");

namespace brio {

/// The V4's sixty-four-bit STK, read from the register struct.
template <class R>
concept Stk64 = requires(R& r) {
    r.CNTL;
    r.CNTH;
    r.CMPLR;
    r.CMPHR;
};

} // namespace brio
