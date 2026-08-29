# PAC - Peripheral Access Controller (SAM C21)

> **PROVISIONAL.** The chapter's whole register surface is built, but the
> driver is MECHANISM ONLY: there is no guard type, no policy and no
> `util/` contract, and nothing in brio turns write protection on. What
> is missing and why is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 11 - and
errata DS80000740S, which has **no PAC section at all**: not one of items
1.1..1.25 names this chapter. What exists instead is a set of items in
OTHER chapters, five of them live at revision F, each saying the PAC does
not behave as ch. 11 promises for one particular peripheral. Driver:
`samc/pac.hpp`. Family fixture `test/family_samc/pac.cpp` plus two
negatives under `tools/check_samc.sh`; the bench suite is
`test_samc_debug`.

The block can write-protect any peripheral on any APB bridge and reports
every access violation: a protected write, a write to an unimplemented
register, a write while a synchronization is running, an access to an
address no client answers.

## What the silicon does

**WRCTRL is a keyed, word-wise store.** One 32-bit write carries the
peripheral identifier in the low sixteen bits and the operation in KEY:
1 = clear, 2 = set, 3 = set-and-lock. 11.5.2.6 is explicit that ONLY a
word-wise write takes effect and that any other access is itself an
error, flagged in INTFLAGA.PAC. There is no read-back of a request:
STATUSn is where the outcome is read.

**PERID = 32 x bridge + index.** Bridge A is 0, B is 1, C is 2; the
index is the peripheral's position in that bridge's list (ch. 12's "PAC,
Index" column), which is also its bit position in INTFLAGn and STATUSn.
The device header states the whole table as `ID_<PERIPHERAL>` macros, so
no driver has to compute it. The C21N has a fourth bridge; this register
map stops at C, and an identifier that would land on a fourth is refused.

**Protection is off out of reset - with exactly one exception.** Every
driver in this stratum says so in its comments, resting on 11.5.2.2's
"After reset, the PAC is enabled" (which is about the PAC, not about
anything being protected). Measured, it holds for bridges A and C's
documented peripherals, and **the DSU comes out of reset already
protected**: STATUSB's reset value is 0x00000002 and table 12-3's "Prot
at Reset" column carries exactly one Y, on the DSU row. `samc/dsu.hpp`
deals with it.

**The lock is until the next reset - of any kind.** Key 3 sets
protection and locks it; a later clear (or a later lock) of a locked
peripheral is an error and the protection stands. 11.5.2.5 says the lock
"will only be cleared by a hardware reset" and 11.5.2.2 that "only a
hardware reset will reset the PAC module", and neither sentence says
which resets count as hardware - table 18-1, which answers that question
for every other block on this device, has no PAC row. Measured: a system
reset (SYSRESETREQ) clears it, and so does a watchdog reset.

**The balance rule is deliberate, and it is what makes a naive scoped
guard wrong.** 11.5.2.6: setting protection on an already-protected
peripheral is an error, and so is clearing it twice, so that "the
application follows the intended program flow by always following a
write protect with an unprotect and conversely". A guard nested twice
therefore raises a PAC error on the way in and again on the way out.

**Four flag banks, all outside protection.** INTFLAGAHB reports
client-bus errors (an access to an address no bridge or peripheral
answers, plus a bit of its own for the DIVAS); INTFLAGA/B/C carry one
bit per peripheral of that bridge. All are write-one-to-clear, and
11.4.8 excepts them - together with WRCTRL - from PAC write protection by
name, so a fully protected system can still unprotect itself and clear
its own errors.

**One interrupt, on the shared line.** PAC_IRQn is line 0, shared with
PM, MCLK, OSCCTRL, OSC32KCTRL and SUPC. There is one enable bit for all
four banks, so an interrupt says only "something violated something" and
the handler must read the banks to tell them apart.

**One event generator**, ACCERR (code 86), raised whenever any flag bank
sets a bit. `samc/evsys.hpp` owns the fabric; this is the code this
peripheral publishes into it.

## The design position: mechanism, no concept

There is no RAII guard here, no policy, no `util/` contract and no
protect-everything-at-boot ceremony, and the reasons are three.

1. **The framework has no user yet.** Nothing in brio turns protection
   on. A guard designed before its first caller would be designed
   against an imagined one, and the shape it should take - scoped,
   reference-counted, per-peripheral, or a boot-time policy table - is a
   question the first safety-minded application gets to answer.
2. **The double-write rule makes nesting a design decision.** Getting a
   nestable guard right needs either a count or a read of STATUS, and
   which is correct depends on whether an interrupt shares the
   peripheral - a question 11.5.2.6 raises itself and does not settle.
3. **Protection is not protection from everything.** Erratum 1.13.3
   (live here, measured below) lets an IOBUS write straight past a
   protected PORT. A `util/` concept promising "protected" would promise
   something this silicon does not deliver uniformly, and the house rule
   is never to state what is not enforced.

Each driver publishes its own `pac_id` where it needs one - `Tsens`,
`Ccl`, `Pac`, `Dsu` and `Mtb` do - and any other driver can grow one the
day something protects it.

## Types and verbs

`PacKey` - WRCTRL.KEY: `off` (the reset value, written by nobody),
`clear`, `set`, `lock`.

`PacAhbFlag` - the INTFLAGAHB bits by name: `flash`, `sram_cm0p`,
`sram_dsu`, `bridge_a`, `bridge_b`, `bridge_c`, `lpram_dmac`, `divas`,
`all`.

`Pac` - the monostate resource.

- Constants: `pac_id` (its own identifier, 0), `bridge_count` (3 here),
  `ev_gen_error` (the ACCERR generator code), `int_error`, `irq()`.
- The identifier arithmetic, all constant-evaluable: `bridge_of`,
  `bit_of`, `perid_of`, `id_valid`.
- Clocks: `bus_clock` (on out of reset; the verb exists for
  completeness).
- Protection: `write_control` (the one request verb, a word-wise store),
  `protect`, `unprotect`, `lock`, `is_protected`, `status` per bridge.
  There is no unlock verb because there is no unlock.
- Flags: `flags`/`clear_flags` per bridge, `flagged`/`clear_flag` per
  peripheral, `ahb_flags`/`clear_ahb_flags`, `clear_all_flags`,
  `any_error`.
- Interrupt and event: `arm`, `armed`, `event_output` (EVCTRL.ERREO),
  `isr()` - which returns a `Report` of all four banks and clears them,
  since the flags are the interrupt's only level.

`is_protected()` cannot distinguish "set" from "set and locked": the lock
is invisible except by attempting a clear and watching the flag.

## How to use it

Protect a peripheral and put it back:

    Pac::protect(Tsens::pac_id);
    ...
    Pac::unprotect(Tsens::pac_id);       // never twice - see the balance rule

Ask whether a write was refused, right after making it:

    Pac::clear_flag(Ccl::pac_id);
    CCL_REGS->CCL_CTRL = 0;              // whatever the driver does
    if (Pac::flagged(Ccl::pac_id)) { ... }

Lock a peripheral's configuration for the rest of this power-on:

    Pac::lock(Nvm::pac_id);              // cleared by the next reset

Handle the shared interrupt:

    extern "C" void PAC_Handler() {
        const auto r = Pac::isr();       // reads and clears all four banks
        if (r.any()) { ... }
    }

## Bench findings

`test_samc_debug` letters a, b and c on the ATSAMC21J18A rev F. Letter c
sits outside `z` because it reboots the board twice.

**The reset state, and a bit neither document has.** STATUSA reads 0,
STATUSB reads 0x00000002 (the DSU, alone), and **STATUSC reads
0x02000000** - bit 25, which is outside 11.7.12's drawing (that stops at
the CCL, bit 23) AND outside the device header's own `PAC_STATUSC_Msk` of
0x00FFFFFF. PERID 64 + 25 = 89, past the header's `ID_PERIPH_MAX` of 87.
It behaves like any other protection bit: a CLEAR aimed at PERID 89
clears it with no error, a SET brings it back. So this device protects
something at reset that no table names.

**Sixteen peripherals across three bridges, and all sixteen report.**
Each of MCLK, RTC, EIC, FREQM, TSENS, PORT, DSU, NVMCTRL, MTB, EVSYS,
SERCOM0, TCC0, TC0, AC, DAC and CCL had a PAC-write-protected register
written back to itself, once unprotected (the control) and once
protected. Every control was clean and **every protected write raised
that peripheral's own INTFLAGn bit**. Five of them (FREQM.CFGA,
PORT.CTRL, DSU.ADDR, MTB.FLOW, EVSYS.CHANNEL11) were also written with a
CHANGED value, each with its own unprotected control, and all five were
**dropped**. So on this silicon the ordinary case is: refused and
reported.

**The two poles, measured side by side.**

- **Erratum 1.19.1's silence is a fact about the REGISTER, not the
  peripheral.** A write to a PAC-protected `TSENS.CTRLB` raises no flag,
  while `TSENS.CTRLA` - the same peripheral, in the map above - does.
  (`test_samc_tsens` letter p is where the write is also shown not to
  take effect.)
- **Erratum 1.7.4 confirmed again**: writing `CCL.CTRL.SWRST` raises the
  CCL's PAC flag **with no protection set anywhere**.

Together: an ABSENT flag is not evidence that a write landed, and a
PRESENT one is not proof that anything was violated.

**Illegal access, with a control on both sides.** A read of MCLK offset
0x00 - which chapter 17 does not implement, and which is the register
erratum 1.23.1 names as "MCLK.CTRLA" - raises the illegal-access error
11.5.2.4 promises, protected or not. The same illegal access on the PORT
(a read past both register groups, at PORT + 0x100) raises nothing, in
either state: **erratum 1.13.2 confirmed**, and confirmed against a
peripheral that does flag it.

**Erratum 1.13.3 confirmed, with the control the claim needs.** The PORT
answers on the APB at 0x41000000 and on the CPU's single-cycle IOBUS at
0x60000000 (the device header defines both). With the PORT protected, the
same `DIRTGL` write **lands through the IOBUS with no flag** and is
**dropped and flagged through the APB**. PAC write protection has a back
door on that bus.

**And the IOBUS window is not a plain mirror**, which no document says:
DIR (offset 0x00) and OUT (0x10) read the same through both buses, while
IN (0x20) and CTRL (0x24) read **zero** through the IOBUS whatever the
APB holds. The single-cycle window carries the write-side registers.

**A byte-wise WRCTRL write changes no protection and is itself flagged**
in INTFLAGA.PAC, exactly as 11.5.2.6 says. `KEY = OFF` does nothing and
is not an error. The balance rule is real: a double clear and a double
set each raise INTFLAGA.PAC - **the PAC's own bit, not the named
peripheral's**.

**The PAC can protect itself, and WRCTRL keeps working while it does**,
as 11.4.8 requires; so do the flag banks.

**A PAC lock lasts until the next reset of any kind.** Locked, a
peripheral refuses a CLEAR (flagged in INTFLAGA.PAC) and refuses a second
LOCK. After a SYSRESETREQ the lock is gone and the peripheral takes a
normal set/clear round trip with no error; after a watchdog reset,
likewise. So 11.5.2.2's "hardware reset" includes the CPU's own reset
request, and a lock is not a until-power-on measure.

## Not covered yet

Driver gaps - features of ch. 11 not built:

- **No guard type and no policy.** See "The design position" above.
  Whatever brio grows here is born with its first user.
- **No `util/` concept.** Nothing target-independent speaks write
  protection, and nothing here proposes a contract until a second family
  says what the portable shape is - the more so because erratum 1.13.3
  shows the guarantee is not uniform even on this one.
- **The ERR interrupt is exposed and never used.** Flags, an enable and
  an ISR body exist; nothing in this stratum binds the shared vector for
  the PAC, and every violation in the suite is read by polling, which is
  the only way to attribute one to the write that caused it.
- **The fourth bridge (D)**, which the C21N alone has. Its identifiers
  are refused; neither its header nor a board is here.

Implemented but not bench-verified:

- **The ACCERR event output.** EVCTRL.ERREO is written and read back, but
  no channel has ever carried the event to a user.
- **Sleep.** 11.5.6 says the PAC keeps catching errors in sleep while a
  bus host runs; nothing has slept with a violation waiting.

Open, and outside this driver:

- **What PERID 89 is.** The bit exists, answers WRCTRL and comes up set;
  no document in this project names it.
