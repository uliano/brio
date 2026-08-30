# DIVAS - Divide and Square Root Accelerator (SAM C21)

> **PROVISIONAL.** The whole of a small chapter is built and measured,
> but the decision the measurements exist for - whether to make this
> block the toolchain's division - is deliberately NOT taken here. See
> "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 14 - and
errata DS80000740S, which has **no DIVAS section**. Driver:
`samc/divas.hpp`. Family fixture `test/family_samc/divas.cpp`; the bench
suite is `test_samc_debug`.

A 32-bit signed/unsigned integer divider and a 32-bit unsigned square
root engine, on the bus matrix because the Cortex-M0+ core has neither.
Every `a / b` in C++ on this core is otherwise a call to `__aeabi_uidiv`
or `__aeabi_idiv` in libgcc.

## What the silicon does

**The operation starts on the operand write.** Writing DIVIDEND does
nothing; writing DIVISOR starts a division and writing SQRNUM starts a
square root (14.6.2.2). There is no start bit and no command register,
which is also why the order of the two operand writes is a requirement
and not a convention.

**Two buses, and the difference is a real choice.** The block answers at
0x48000000 on the high-speed bus and at 0x60000200 on the CPU's local
single-cycle IOBUS. On the AHB path a read of RESULT while the engine is
busy INSERTS WAIT STATES and returns the finished answer, so no polling
is needed; the IOBUS cannot wait, so a caller there must poll
STATUS.BUSY first or read a stale result (14.5.10).

**The IOBUS address comes from the data sheet, not the header.** The
device header defines DIVAS at 0x48000000 and says nothing about the
alias; table 9-1 puts the IOBUS region at 0x60000000 and figure 8-3
places PORT at its base with DIVAS at +0x200 (nothing else is in the
region - 0x60000220 onward is Reserved). Where both documents speak the
header wins; where the header is silent, as here, the data sheet is
spelled out and the bench arbitrates. It does: the same quotients come
back through both paths.

**Writing an operand while busy is an error.** 14.5.8 write-protects
CTRLA, DIVIDEND, DIVISOR and SQRNUM for the duration of an operation and
says an access "will result in an error" without saying where the error
appears. Measured, it appears in PAC.INTFLAGAHB.DIVAS - the block's own
bit as an AHB client (11.7.5), and the only candidate on this device.

**Divide by zero is not a fault.** 14.6.2.5: the quotient comes back
zero, the remainder equal to the dividend, STATUS.DBZ is set and stays
set until written back. Nothing traps.

**The signed overflow has no indication either.** 14.6.2.4: the most
negative number divided by minus one overflows the signed range and
"will return the maximum negative number with no indication of the
overflow".

**Leading-zero optimization trades determinism for speed.** CTRLA.DLZ = 0
(the reset value) lets the engine skip the dividend's leading zeros, so a
division takes 2 to 16 cycles depending on the operands; DLZ = 1 forces
every 32-bit division to sixteen.

**No interrupt, no events, no DMA, and no sleep.** 14.5.4/5/6 are all
"Not applicable", and 14.5.2 is blunter than any other chapter in this
family: "The DIVAS will not operate in any sleep mode." It is also not a
peripheral on an APB bridge - it has no `ID_` macro, no PAC write
protection and no bit in any STATUSn - only the AHB-client flag above.

## Types and verbs

`DivasBus` - which address a verb goes through: `ahb` (the default, the
one that cannot be got wrong) or `iobus`.

`DivasResult` / `DivasSignedResult` - `result` and `remainder`, unsigned
and in two's complement.

`Divas` - the monostate resource. There is no init() beyond the bus clock
and no enable bit: the block is live whenever it is clocked, and its
clock is on out of reset (table 12-3, AHB index 12).

- Constants: `ahb_base`, `iobus_base`.
- Register access: `regs()`, `io_regs()`, `at<bus>()`.
- Clocks: `bus_clock` (set and read).
- Configuration: `configure(signed, disable_leading_zero)`, `ctrla`,
  `signed_division`, `leading_zero_disabled`.
- Status: `status`, `busy`, `divide_by_zero`, `clear_divide_by_zero`,
  `wait_idle`.
- Operations, each templated on the bus: `divide_unsigned`,
  `divide_signed`, `square_root`.

`configure()` is separate from the operations on purpose: writing CTRLA
costs a store most callers do not need, and a write while busy is an
error. `divide_unsigned()` on a block left in signed mode divides signed
- the register is the truth and this header does not paper over it.

The operation verbs do NOT poll before writing their operands. The
chapter's own flow does not, and a poll would cost more than the division
- a CPU cannot issue two stores faster than the engine finishes.
`wait_idle()` is there for a caller who wants the guarantee anyway, for
instance before dividing inside an interrupt.

## How to use it

An unsigned division with its remainder:

    Divas::bus_clock(true);
    Divas::configure(false);
    const auto r = Divas::divide_unsigned(numerator, denominator);
    // r.result, r.remainder

Signed, with the language's own truncating semantics:

    Divas::configure(true);
    const auto r = Divas::divide_signed(-1000, 7);   // -142 remainder -6

An integer square root:

    const auto s = Divas::square_root(value);        // s.result, s.remainder = value - root^2

Deterministic timing, at the cost of the fast path:

    Divas::configure(false, true);                   // DLZ = 1: always 16 cycles

Divide by zero, which does not trap:

    Divas::clear_divide_by_zero();
    const auto r = Divas::divide_unsigned(x, y);
    if (Divas::divide_by_zero()) { ... }             // r.result 0, r.remainder x

## Bench findings

`test_samc_debug` letters g and h on the ATSAMC21J18A rev F, wireless.
The compiler is the reference throughout: every quotient and remainder is
compared with what gcc's own software division produces for the same
operands, through volatiles so nothing folds away.

**Nine unsigned divisions** - including 0xFFFFFFFF / 1, 0xFFFFFFFF /
0xFFFFFFFF, 1 / 0xFFFFFFFF and 0 / 5 - agree in quotient AND remainder,
and **so do the same nine through the IOBUS alias at 0x60000200**. **Six
signed divisions** match the language's truncating semantics, remainders
included.

**The documented overflow, observed:** 0x80000000 / -1 returns
-2147483648 with remainder 0 and no status bit anywhere.

**Divide by zero** gives quotient 0 and remainder = the dividend, sets
STATUS.DBZ, and the bit **latches across a later good division** until it
is written back.

**Seven square roots** are exact, with REMAINDER = n - root^2; the widest
input 0xFFFFFFFF gives 65535 remainder 131070. The square root **ignores
CTRLA.SIGNED**, which has no meaning for it.

**14.5.8's unnamed error is PAC.INTFLAGAHB.DIVAS.** Two hundred
deliberately overlapped operand writes through the IOBUS leave
INTFLAGAHB = 0x80, the DIVAS bit and nothing else - and the engine
divides correctly afterwards.

**What it costs**, at 48 MHz, as the difference of two 4000-iteration
loops (the loop with the operation minus an otherwise identical one), in
cycles per operation:

| operation | 0xFF / 7 | 0xFFFF / 7 | 0xFFFFFFFF / 7 |
|-----------|---------:|-----------:|---------------:|
| gcc `a / b` | 101.2 | 181.9 | 332.4 |
| gcc `a / b` and `a % b` | 238.8 | 389.2 | 683.4 |
| DIVAS, quotient only, AHB | 7.5 | 11.5 | 19.5 |
| DIVAS, quotient + remainder, AHB | 10.5 | 14.6 | 22.6 |
| DIVAS, quotient + remainder, IOBUS | 13.6 | 18.5 | 23.5 |

So the accelerator is **17x** faster than gcc's division on the widest
operands for a quotient alone, and **30x** when the remainder is wanted
too - because a software `a / b` and `a % b` is two calls where the
hardware leaves both answers in registers.

**The IOBUS is not the faster path here.** The AHB's wait-state stall
costs less than the explicit STATUS.BUSY poll the IOBUS requires, at
every operand size. (The poll goes through the IOBUS itself; reaching for
the AHB status register in the middle of an IOBUS division measured
worse still.)

**Leading-zero optimization, confirmed both ways.** With DLZ = 0 a small
dividend costs 8.2..8.3 cycles against 20.7..20.8 for a full-width one -
**12.4 cycles apart**; with DLZ = 1 both cost 20.7..20.8 and the pair is
apart by **less than a tenth of a cycle** across runs. That is the
deterministic timing 14.6.2.6 offers, and the price of it. (The
second verdict is a band on the ABSOLUTE difference for a reason: two
measurements that are meant to be EQUAL cannot be compared by ordering,
because the stopwatch's own noise decides which is larger.)

**A square root of 0xFFFFFFFF costs 20.8 cycles**, the same as a
full-width division.

**The AHB clock is on out of reset**, as table 12-3's index 12 says.

## Not covered yet

Driver gaps - features of ch. 14 not built:

- **Adopting the block as the toolchain's division: RULED OUT
  (2026-08-30).** gcc lets `__aeabi_uidiv`/`__aeabi_idiv` be
  overridden and the numbers above were the decision's input - and the
  decision is NO. A global override is a whole-image invariant whose
  one hazard - the block has ONE set of operand registers, so a
  division taken in an interrupt corrupts one in progress below it -
  could be held off only by discipline (or by paying a critical
  section on every division, eating the gain exactly where divisions
  are short), and brio's house rule is that a guarantee is enforced or
  it is not stated. The gain matters only in division-heavy code,
  which can spend it deliberately: `Divas`'s explicit verbs are the
  offer, and the mechanism-not-policy split stands.
- **16-bit operation.** The chapter's cycle counts mention it (14.6.2.6)
  and no register ever does: the operands are one 32-bit width, and the
  "16-bit division" of that sentence is a 32-bit division whose dividend
  has sixteen leading zeros - which is exactly the fast case the table
  above measures.

Implemented but not bench-verified:

- **`wait_idle()` as a guard against a concurrent operation.** Nothing in
  this stratum divides from an interrupt yet, so the re-entrancy hazard
  is stated and not exercised.
- **The block with its AHB clock off.** `bus_clock(false)` is written and
  never tried against a division.
