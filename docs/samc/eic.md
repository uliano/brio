# EIC - the External Interrupt Controller (SAM C21)

> **PROVISIONAL.** The whole chapter is built and bench-verified except
> the debouncer, which exists only on the C20/C21 **N** variants and is
> not even declared by this family's device header, and sleep/wake
> behaviour, which belongs to the power pass. The list is in "Not covered
> yet".

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 26 - and
errata DS80000740S items 1.11.1 to 1.11.6, of which **exactly one is
this silicon** (see below). Driver: `samc/eic.hpp`. Family fixture
`test/family_samc/eic.cpp` plus three negatives under
`tools/check_samc.sh`; the bench suite is `test_samc_eic`.

## What the silicon does

**This is where this family keeps its pin interrupts.** PORT has none -
`port.md` says so and stops there - so everything the AVR spells in
`PINnCTRL.ISC` lives in a peripheral of its own, with its own clock, its
own enable, its own event outputs and its own NMI. A pad reaches it
through peripheral function **A**.

**Sixteen lines plus one NMI.** Each EXTINT line has a three-bit sense
(none / rising / falling / both / high level / low level), a
majority-of-three filter, an asynchronous-detection bit and an event
output enable. The NMI has the same vocabulary in a register of its own
and is a line apart in every way: always enabled, taken at any priority,
unmaskable, and **enabled by NMISENSE alone - "the EIC module is not
required to be enabled"** (26.6.4.1).

**The pad-to-line map is not a formula.** PA16 is line 0, PA24 is line
12, PA27 is line 15, PB30 is line 14; up to four pads share one line, and
which pads a package bonds differs (25 on the E, 37 on the G, 51 on the
J). The device header carries one `PIN_P<pad>A_EIC_EXTINT_NUM` per bonded
pad and that is the entire authority here. If two pads of the same line
are muxed to the EIC at once, **only one is active - "the first one
programmed"** (26.6.6 note 2), and which one won is not readable
anywhere.

**One NVIC vector for all sixteen lines.** 26.6.6 says "The EIC has one
interrupt request line for each external interrupt (EXTINTx)", but the
device header gives this part a single `EIC_IRQn` - the same shape the
SERCOM has here, and the same discipline: the ISR body masks INTFLAG with
INTENSET and dispatches on the result.

**The clock is optional, and knowing when is the whole trick** (26.5.3,
26.6.3). Level detection with no filter is asynchronous and needs no
clock; so is edge detection with `ASYNCH` set. Filtering and *synchronous*
edge detection need GCLK_EIC or CLK_ULP32K, selected by `CTRLA.CKSEL` -
and in those modes the pin is sampled, so "pulses with duration lower
than two EIC clock periods may not be properly detected". Table 26-2's
worst cases:

| Detection mode | Latency (worst case) |
|----------------|----------------------|
| level, no filter | 5 CLK_EIC_APB |
| level + filter | 4 EIC clocks + 5 CLK_EIC_APB |
| edge, no filter | 4 EIC clocks + 5 CLK_EIC_APB |
| edge + filter | 6 EIC clocks + 5 CLK_EIC_APB |

**A cleared level flag comes straight back** while the pin still matches
(26.6.3); a cleared edge flag waits for the next edge.

**Enable-protection is the structural rule.** CONFIGn, ASYNCH, EVCTRL
and CTRLA.CKSEL are writable only while CTRLA.ENABLE is zero (26.6.2.1),
so every verb that touches one of them **refuses** while the block is
enabled rather than storing into a register the silicon ignores.

**Every line drives an EVSYS generator.** EVCTRL.EXTINTEO is sixteen bits
wide and ch. 29's generator table lists EXTINT0..EXTINT15 at codes
0x0E..0x1D. 26.6.7's prose says "External event from pin (EXTINT0-7)"; it
is wrong, and the bench says so (below).

## The errata: read the row, not the column

Of six EIC items, **one applies to this chip** (E/G/J, silicon rev F).
This is the chapter where the standing trap bites hardest, because the
item everyone would reach for - 1.11.5 - is N-family only.

- **1.11.1 NMI Exception** (an NMI fires as soon as the config is written,
  with a pull-up and a rising-edge sense): revision **B only**.
- **1.11.2 ASYNCH is not write-protected**: revision **B only**.
- **1.11.3 Spurious Flag** (a flag appears at CTRLA.ENABLE for a
  low/rising/both line with the filter on): revisions **B..E**. Not this
  chip, so `enable()` does **not** quietly clear INTFLAG; on an affected
  part the caller clears INTFLAG after enabling and before arming, which
  is the erratum's own workaround.
- **1.11.4 False NMI Interrupt** on an on-the-fly NMISENSE change:
  revisions **B..E**. `nmi_configure()` likewise leaves NMIFLAG alone and
  says so.
- **1.11.5 Edge Detection** (SYNCBUSY.ENABLE released three EIC clocks
  before edge detection actually works): the **N family only**.
- **1.11.6 Edge Detection in Standby**: **every revision of E/G/J, so
  live here.** With ASYNCH set and the device in Standby only the *first*
  edge is detected; the rest are ignored until it wakes. Microchip's
  workaround for this family is blunt - "asynchronous edge detection
  doesn't work, instead use the synchronous edge detection" - with
  CLK_ULP32K as the cheap clock for it. The driver cannot enforce this,
  because it does not know whether the application ever sleeps:
  `EicLineConfig::asynchronous` carries the obligation in its own
  comment, and the power pass owns the rest.

## Types and verbs

**`EicSense`** - `none`, `rising`, `falling`, `both`, `high`, `low` (the
same encoding serves CONFIGn.SENSEx and NMICTRL.NMISENSE; the two
reserved codes are not spellable). **`EicClock`** - `gclk`, `ulp32k`.

**`EicLineConfig`** - `sense`, `filter`, `asynchronous`, `event_out`:
one line's SENSEx/FILTENx plus its ASYNCH and EVCTRL bits, because those
three registers describe one line between them and a caller who sets them
separately can only get them out of step.
`eic_line_config_valid()` refuses the filter with asynchronous detection
(26.8.10's own note). `eic_needs_clock()` answers the chapter's clock
question from a configuration.

**`EicNmiConfig`** - `sense`, `filter`, `asynchronous`, with the same two
helpers.

**`Eic`** - the block, monostate. `line_count` (16), `config_regs` (2),
`gclk_id`, `irq()`, `event_generator(line)`, `bus_clock`, `reset`,
`enable`/`enabled`, `clock_select` (both ways), `clock(generator)`,
`init`, `release`; `configure_line`, `line_config`, `release_line`;
`flags`/`clear_flags`/`arm`/`disarm`/`armed`/`flag`/`clear_flag`/
`line_mask`, `isr()`; `nmi_configure`, `nmi_config`, `nmi_flag`,
`clear_nmi_flag`, `take_nmi()`.

**`ExtInt<Pin>`** - one line reached through the pad that carries it:
`line`, `mask`, `event_generator`, `claim(pull)`, `release()`, and the
flag/arm/configure verbs bound to that line. A pad the device does not
bond fails to compile, which is the per-package gate with no hand-kept
table behind it. **`ExtNmi<Pin>`** is the same shape for the NMI pad
(PA08 on this family, again from the header).

`eic_extint_line(port, pin)` and `eic_nmi_pad(port, pin)` are the
constexpr map itself, returning -1 / false for a pad this device does not
bond.

## How to use it

**A button on a line, interrupt-driven:**

```cpp
using Button = brio::Pin<'B', 22>;
using ButtonInt = brio::ExtInt<Button>;          // EXTINT6, from the header

Eic::init();                                     // APB clock + reset, left DISABLED
Eic::clock_select(EicClock::ulp32k);             // enable-protected: while disabled
ButtonInt::claim(brio::PinPull::up);
Eic::configure_line(ButtonInt::line,
                    {.sense = EicSense::falling, .filter = true});
Eic::enable(true);                               // check the return - see below
ButtonInt::arm(true);
Nvic::enable(Eic::irq());
```

```cpp
extern "C" void EIC_Handler() {
    const uint32_t lines = brio::Eic::isr();      // masked by INTENSET, flags cleared
    if (lines & ButtonInt::mask) { /* ... */ }
}
```

**A line as an event generator**, with `evsys.hpp` moving the fabric and
this driver supplying the code:

```cpp
Eic::configure_line(LineA::line, {.sense = EicSense::rising,
                                  .asynchronous = true,
                                  .event_out = true});
Eic::enable(true);
Evsys::connect(user, channel, EventChannelConfig{
    .generator = LineA::event_generator,          // published here, not in evsys.hpp
    .path = EventPath::asynchronous,              // works - see "Bench findings"
});
```

**The NMI**, which needs no enable and no arming - and, on a board whose
pads are not all known, an **edge** sense and nothing else: an NMI cannot
be masked, so a level sense on a pad sitting at that level is an
unbreakable loop.

```cpp
using Nmi = brio::ExtNmi<brio::Pin<'A', 8>>;
Eic::bus_clock(true);
Nmi::claim(brio::PinPull::down);
Eic::nmi_configure({.sense = EicSense::rising, .asynchronous = true});
```

```cpp
extern "C" void NonMaskableInt_Handler() { (void)brio::Eic::take_nmi(); }
```

Note the spelling: the vector is `NonMaskableInt_Handler`, the device
header's name. See "Bench findings".

## Bench findings

From `test_samc_eic`: 6 letters in `z`, **85 verdicts, 85/85 twice**,
plus `n` (the NMI, **9/9**) and `u` (the button, **4/4**) outside it.
**Nothing to wire.**

**The stimulus technique, which is itself a finding.** A chapter about
*external* pins on a board with no wires needs the chip to move its own
pad, and there are two candidates. Measured:

- **Driving the pad from PORT with the peripheral mux on does nothing.**
  With PMUXEN set for function A, DIR and OUT no longer reach the output
  driver: a HIGH-level line reads its flag as 0 with OUT=0 *and* with
  OUT=1. 28.6.1's "will override the connection between the PORT and that
  I/O pin" reaches the driver.
- **The internal pull does work, and is the stimulus this whole suite
  runs on.** PINCFG.PULLEN survives PMUXEN and the pull's *direction* is
  still the PORT OUT bit (28.6.3.2), so writing OUT walks a free pad
  between the rails through the pull resistor - a real edge on a real
  pin, seen by every sense, filter and event path in this chapter.

Every pad used is first proved electrically free (it follows its own
pull) through PORT, before the EIC sees it.

**THE ENABLE NEEDS THE CLOCK ITS LINES ASKED FOR - and no chapter draws
that consequence.** 26.6.3 says the EIC "automatically requests GCLK_EIC
or CLK_ULP32K to operate" when filtering or synchronous edge detection is
enabled; CTRLA.ENABLE is write-synchronized; what it synchronizes
*against* is that requested clock. So with GCLK_EIC disconnected:

| lines configured | `enable(true)` | CTRLA | SYNCBUSY.ENABLE | edge seen |
|---|---|---|---|---|
| asynchronous edge (requests no clock) | **true** | 0x2 | 0 | **yes** |
| synchronous edge (requests one) | **false** | 0x2 | **1** | no |

The write is *pending*, not lost: connecting the generic clock channel
afterwards, **without touching CTRLA**, completes that same enable and
the line starts detecting with no second write. `enable()` returning
false is the only warning there is, which is why it returns one. The
cheap escape is CLK_ULP32K, which needs no GCLK channel at all and is
always running on this family - measured: a synchronous edge is detected
with `EIC_GCLK_ID` disconnected and CKSEL = ULP32K.

**A HARDWARE GENERATOR DOES CROSS AN ASYNCHRONOUS EVSYS CHANNEL.** This
is the question `evsys.md` explicitly declined to answer, having only
software events to test with. An EXTINT rising edge routed through an
**asynchronous** channel moves a whole DMA block, exactly as the same
edge on a resynchronized channel does - where eight software events on an
asynchronous channel move nothing. The reading that fits both
measurements: the asynchronous path carries what has *width*, and a
register write has none.

**Every line is an event generator, not just EXTINT0-7.** 26.6.7's prose
is narrower than its own EVCTRL register and than ch. 29's generator
table. EXTINT9 (generator code 0x17) moves a DMA block through EVSYS; the
same line with EVCTRL.EXTINTEO cleared still raises its INTFLAG and moves
nothing, so the enable bit is the gate and the line is not.

**A cleared level flag comes straight back** while the pin still matches,
and a cleared edge flag does not - both measured, and the reason a
level-sensitive handler that only clears will spin.

**The filter is a real low-pass.** With CLK_ULP32K (~32 kHz, one period
~30 us) a both-edge filtered line ignores a pull-driven excursion of a
few hundred CPU cycles and catches a settled one, which is 26.6.3's "two
EIC clock periods" made visible.

**One vector, sixteen lines.** An *unarmed* line still raises its
INTFLAG but takes no interrupt; arming one line and moving both leaves
the handler's `isr()` mask naming only the armed one, with the unarmed
line's flag standing untouched.

**THE NMI FIRES WITH THE EIC DISABLED**, which is 26.6.4.1 measured:
NMISENSE alone enables it, no CTRLA.ENABLE and no INTENSET involved. And
sense NONE is the only way to turn one off, since there is no enable to
clear - verified across a further edge.

**A trap in this project's own crt, found by that NMI.** The device
header declares the core exception vectors as
`NonMaskableInt_Handler` and `SVCall_Handler`; the crt spelled them
`NMI_Handler` and `SVC_Handler` (the CMSIS-classic names), while all
thirty-one *peripheral* vectors already matched the header. An app
binding the header's name therefore compiled, linked and left the real
vector pointing at `Default_Handler`, which on this target is a silent
spin: the first NMI wedged the board with a half-printed line. The crt
now spells both the header's way. Vectors: `EIC_Handler`,
`NonMaskableInt_Handler`.

**One board fact**, from letter `u`: PB22 does **not** follow its own
internal pull and sits LOW with the internal pull-up enabled, so the
user button is not fitted the pull-up-to-ground way. That letter
therefore asserts only what the header and the block guarantee, prints
what the pad does, and offers a ten-second window whose result is
printed and not judged - a suite must not fail for want of a finger.


- **In standby** (measured in the transversal sleep pass -
  [platform.md](platform.md), "Sleep, peripheral by peripheral", where
  the hardware stimulus that makes it possible is described): an EXTINT
  wakes the device from STANDBY in 7 us; a SAMPLED line detects every
  edge of a standby on CLK_ULP32K **and** on a GCLK_EIC whose generator
  has RUNSTDBY clear, because this block has no RUNSTDBY bit of its own
  to ask with and its clock request is honoured in there anyway; and
  **ERRATUM 1.11.6 DOES NOT REPRODUCE** at revision F - an asynchronous
  line detected 100 edges of 100 offered inside ONE standby, exactly as
  a sampled one did, and with the interrupt armed all 100 woke the
  device.

## Not covered yet

Driver gaps (deliberate):

- **The debouncer** (DEBOUNCEN, DPRESCALER, PINSTATE, 26.6.4.3): SAM
  C20/C21 **N** variants only, and this family's device header does not
  declare the registers at all, so there is nothing to gate and nothing
  to write. It is also the N family's workaround for erratum 1.11.6,
  which this family answers differently.
- **A kernel event out of a pin.** `isr()` gives an app the mask; nothing
  in the framework yet turns an EIC line into a posted event the way
  `util/input_scanner.hpp` turns a polled pin into one.
- **The "first one programmed wins" rule for two pads on one line**
  (26.6.6 note 2) is documented and not policed: the driver cannot know
  which pads an application has muxed, and the silicon does not say which
  one won.

Implemented but not bench-verified:

- **Falling-edge and level senses through EVSYS** (only a rising edge has
  been routed), and `EicSense::low` as an interrupt (only `high` has been
  used as a level).
- **The NMI's filter and its synchronous detection**: the bench NMI is
  edge-sensed and asynchronous by deliberate choice, because a level NMI
  on a board whose pads are not all known cannot be recovered from
  without a reflash.
- **CTRLA.CKSEL = GCLK with a fast generator.** Every measurement here
  runs the EIC at ~32 kHz (OSCULP32K, directly or through a generator),
  which is what makes the filter's threshold visible; a filter designed
  around a megahertz GCLK_EIC is untested.
- **Operation on the E and G variants**: compile-checked only. The block
  is identical across the family; what differs is which pads are bonded,
  and that is exactly what the family fixture asserts per variant.
