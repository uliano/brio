# FD controller area network (STM32G0)

> **PROVISIONAL.** Chapter 36 is implemented whole - every register,
> every field, every mode, the message RAM and both instances - and
> almost all of it is bench-verified. What keeps the banner is a bench
> and not a driver: this desk has NO TRANSCEIVER, NO WIRE AND NO SECOND
> NODE, so everything that needs a bus - arbitration, a real
> acknowledge, a foreign frame, bus-off from traffic - is measured
> against the controller's own loop-back or is declared in "Not covered
> yet" with its reason. The frame vocabulary is deliberately
> TARGET-LOCAL and says so below.

Documents of record: RM0444 Rev 6 - FDCAN ch. 36 whole, the subsystem's
enable and reset 5.4.15/5.4.16 (`APBENR1.FDCANEN`, `APBRSTR1.FDCANRST`),
the kernel-clock multiplexer 5.4.22 (`CCIPR2.FDCANSEL` - a DIFFERENT
register from the CCIPR every other multiplexer of this stratum lives
in), the vector table 61, the memory map (FDCAN1 0x4000 6400, FDCAN2
0x4000 6800, the configuration block 0x4000 6500, the message RAM
0x4000 B400) and PWR table 27. DS13560 Rev 5 tables 14, 15, 17 and 18 for
the pads (every FDCAN signal of this part is **AF3**). Errata ES0548
Rev 3: 2.13.1 and 2.13.2, both below. Driver: `stm32g0/fdcan.hpp`; the
presence, base, mask, clock-select and vector facts come from
`stm32g0/device_tables.hpp`, and `Rcc::kernel_clock2()` in
`stm32g0/clock.hpp` is the CCIPR2 verb this chapter needed. Bench suite:
`test_stm32_fdcan` (12 letters, 96 verdicts, wireless). Family fixture
`test/family_stm32g0/fdcan.cpp` plus twelve negatives under
`tools/check_stm32g0.sh`.

## What the silicon does

A Bosch M_CAN core - ISO 11898-1:2015 and CAN FD 1.0 - and this is the
first chapter of the stratum that a part can be **missing entirely**:
table 1 gives FDCAN1 and FDCAN2 to the G0B1/G0C1 class alone, and the
G071 and G031 headers declare no base, no struct and no interrupt
enumerator. `Fdcan<n>` therefore does not exist on a part without one
(the `avrdx/opamp.hpp` precedent), while the protocol ARITHMETIC - bit
timing, the DLC coding, the element codecs - compiles everywhere,
because it is a property of CAN and not of a peripheral.

**One clock, one reset, one divider for the SUBSYSTEM.** Figure 392
draws the two modules inside one block. `RCC_APBENR1.FDCANEN` clocks
both; `RCC_APBRSTR1.FDCANRST` **resets both**, so a reset asked for
through FDCAN2 takes FDCAN1's registers back to their reset values too
(measured) - which is why `enter()` does not pulse it and `reset()` is
the caller's, once, before either instance is configured. The one
`FDCAN_CKDIV` register lives at the configuration block's own address,
is FDCAN1's by 36.4.37's note, and divides the kernel clock for both.

**INIT and CCE are the whole configuration gate** (36.3.4). A protected
register is writable only with `CCCR.INIT` and `CCCR.CCE` both set, so
every configuring verb here refuses outside that state and writes
nothing - the shape `usart.hpp`'s UE-protected verbs have. Three
corollaries the chapter states and this driver enforces: CCE can only be
SET while INIT is set and is cleared BY HARDWARE when INIT is cleared;
setting CCE **resets eight status registers** and presets the timeout
counter; and `TXBAR`/`TXBCR` are writable only with CCE **clear**, which
is the one place in the file where a verb refuses because a gate is
open. INIT itself crosses a clock domain, so `init_mode()` writes and
then WAITS for the readback (36.4.6's note), bounded.

**The message RAM is a fixed map whose content at reset is undefined.**
Figure 399, 212 words per instance: 28 standard filter words at 0x000,
8 extended filter elements of two words at 0x070, Rx FIFO 0 (three
18-word elements) at 0x0B0, Rx FIFO 1 at 0x188, a three-element Tx event
FIFO of two words at 0x260 and three 18-word Tx buffers at 0x278, with
FDCAN2's map starting where FDCAN1's ends (+0x350). Nothing in the
silicon clears it, so `enter()` zeroes the instance's 212 words first -
an un-zeroed filter list is 28 elements of whatever the last program
left. The RAM is ordinary CPU-addressable memory, which is what makes an
element codec possible at all and what makes 36.3.7's warning real:
reading an element out of turn is legal, ACKNOWLEDGING it is not.

**Test mode is the only bus this board has.** EXTERNAL loop-back
(`TEST.LBCK` alone) feeds the transmitter back to the receiver
internally, ignores acknowledge errors and **still drives the TX pad**;
INTERNAL loop-back (LBCK + `CCCR.MON`) disconnects the RX pin and holds
TX recessive. `TEST.TX = 01` puts the sample point on the pad. Writing
`FDCAN_TEST` at all needs `CCCR.TEST`, and clearing `CCCR.TEST` returns
that whole register to its reset value (36.4.4).

**The interrupt line select is seven GROUPS and not thirty flags.** On
this part `FDCAN_ILS` carries seven bits, each naming a group of IR
flags (36.4.17) - the classic M_CAN's per-flag ILS is not what this
silicon has - so `interrupt_line()` speaks groups and `fdcan_group_of()`
says which group a flag belongs to. Both lines land on a **shared
vector**: table 61 puts `fdcan1_intr0_it` and `fdcan2_intr0_it` on
TIM16's line and both `intr1_it` on TIM17's, so one handler serves two
peripherals and two instances.

## Types and verbs

The arithmetic, present on every part of the family:

- `FdcanBitTiming` holds a phase's timing **as the register does** -
  every field one less than the thing it counts - with `tq_per_bit()`,
  `sample_point_permille()` and `clocks_per_bit()` over it.
  `fdcan_nominal_timing_valid()` checks NBTP's fields and 36.4.7's
  4..81 tq window; `fdcan_data_timing_valid()` checks DBTP's far
  narrower ones and 36.3.4's 4 tq floor.
- `fdcan_bit_timing_for(tq_hz, bit_hz, sample_permille)` and
  `fdcan_data_timing_for(...)` are the choosers.
  **THE RULE IS THIS DRIVER'S AND NOT THE CHAPTER'S**: the rate must
  come out EXACT, and among the prescalers that admit an exact bit the
  SMALLEST is taken - the smallest prescaler is the finest time quantum,
  which is the most tq in the bit and therefore the finest grid the
  sample point can be placed on. The sample point is then put at the
  nearest whole tq and the answer REFUSED if that lands more than 1 %
  away, so a caller never gets a silently different sample point. SJW is
  `min(BS2, 4)`, the four quanta 36.3.3's own text names.
- `fdcan_dlc_to_length()` / `fdcan_length_to_dlc()` are table 212 both
  ways; a length no DLC codes (thirteen bytes) is `0xFF`.
- `FdcanFrame` is the frame, and `id` **IS THE NATURAL IDENTIFIER**,
  right aligned: 0..0x7FF standard, 0..0x1FFF_FFFF extended. The
  element's own field stores a standard identifier in ID[28:18] (table
  215) and the codecs do that shift, because the filters speak the
  natural 11-bit number (SFID1[10:0]) and a caller made to shift for one
  and not the other would get it wrong. `length` is BYTES and not a DLC.
- `fdcan_encode_tx()` / `fdcan_decode_rx()` / `fdcan_decode_event()` and
  the two filter-word makers are constexpr and pinned in the fixture
  against tables 214..223 by hand-built words.
- `FdcanConfig` + `fdcan_config_valid()` are every configuration rule of
  the chapter in one place, and `Fdcan<n>::enter<cfg>()` is the
  compile-time twin whose assertion names the rule that was broken.

The resource `Fdcan<n>`: the subsystem's `bus_clock()`, `reset()`,
`kernel_clock()` and `clock_divider()`; the state machine as
`init_mode()`, `configuration()`, `configurable()`, `enter()`, `start()`
and `stop()`; `nominal_timing()` / `data_timing()` /
`delay_compensation_offsets()`; the mode bits one verb each
(`bus_monitor`, `restricted`, `auto_retransmit`, `test_mode`, `fd_mode`,
`non_iso`, `transmit_pause`, `edge_filtering`,
`protocol_exception_disable`); `loop_back()`, `tx_pin()`, `rx_pin()`;
`power_down()` + `power_down_acked()`; the RAM (`ram_word()`,
`clear_ram()`, `ram_watchdog()`); the filters (`standard_filter()`,
`extended_filter()`, `filter_lists()`, `non_matching()`,
`reject_remote()`, `fifo_overwrite()`, `extended_mask()`,
`high_priority()`); receiving (`rx_available()`, `rx_read()`,
`rx_peek()`, `rx_acknowledge()`, `rx_read_overwrite()`); transmitting
(`tx_put()`, `tx_put_buffer()`, `tx_request()`, `tx_cancel()`,
`tx_pending()`, `tx_occurred()`, `tx_cancelled()`, the two per-buffer
interrupt enables); the Tx event FIFO (`events_available()`,
`event_read()`); `timestamp()` and `timeout()`; `status()` and
`error_counters()`; and the interrupt surface with `isr0()` / `isr1()`.

**Five registers destroy what they report when read** - PSR sets LEC and
DLEC to 7 and clears RESI/RBRS/REDL/PXE, ECR clears CEL - so `status()`
and `error_counters()` read each of them EXACTLY ONCE per call and hand
back the whole decoded picture. There is no per-field accessor for
either, on purpose.

**No task is built.** A CAN bus AO, a frame type shared with another
architecture's controller and the policy that goes with them are a util
design question, and a decision taken from ONE implementation is a
decision taken from the M_CAN's element layout. `FdcanFrame` is this
target's until a second silicon's CAN arrives (the `avrdx/rtc.hpp`
precedent - a task is born with its first user).

## How to use it

Bring one instance up in internal loop-back, accept everything, and send
a frame:

```cpp
using Can = brio::Fdcan<1>;
constexpr brio::FdcanConfig cfg{
    .nominal = *brio::fdcan_bit_timing_for(64'000'000, 500'000, 875),
    .mode = brio::FdcanMode::internal_loop_back,
    .standard_filters = 1,
};

Can::bus_clock(true);
Can::reset();                        // ONE reset for BOTH instances
Can::enter<cfg>();                   // clock, RAM, timing, modes; INIT kept
Can::standard_filter(0, {brio::FdcanFilterType::classic,
                         brio::FdcanFilterAction::store_fifo0, 0, 0});
Can::start();                        // INIT cleared, on the bus

brio::FdcanFrame out{.id = 0x123, .length = 2, .marker = 0x77};
out.data[0] = 0xDE;
out.data[1] = 0xAD;
Can::tx_put(out);

brio::FdcanFrame in{};
if (Can::rx_available(0) != 0) {
    Can::rx_read(0, in);             // decodes and acknowledges
}
```

On a real bus, hand the two pads over first (both AF3) and choose
`FdcanMode::normal`:

```cpp
brio::FdcanPad<brio::PinSel{'B', 8, brio::PinFunction::af3}>::claim_rx();
brio::FdcanPad<brio::PinSel{'B', 9, brio::PinFunction::af3}>::claim_tx();
```

CAN FD with bit rate switching is two more fields and a per-frame flag:

```cpp
constexpr brio::FdcanConfig fd{
    .nominal = *brio::fdcan_bit_timing_for(64'000'000, 500'000, 875),
    .data = *brio::fdcan_data_timing_for(64'000'000, 2'000'000, 875),
    .fd = brio::FdcanFd::on_with_bit_rate_switch,
    .standard_filters = 1,
};
// ... and in the frame: .fd = true, .bit_rate_switch = true
```

The two interrupt lines share their vectors with TIM16 and TIM17 and
with the other instance, so a handler calls every body that can be on
the line and each answers for its own:

```cpp
extern "C" void TIM16_FDCAN_IT0_IRQHandler() {
    const uint32_t a = brio::Fdcan<1>::isr0();
    const uint32_t b = brio::Fdcan<2>::isr0();
    // ... and TIM16's own body if the app runs it
}
```

## Bench findings

All from `test_stm32_fdcan`, **96 verdicts, 96/96 four times** (one from
a cold flash), wireless, nothing written to flash.

**The pads and the instruments.** PB8 (FDCAN1_RX) and PB9 (FDCAN1_TX),
both AF3, both proven to follow their own internal pull before either is
claimed and both left in analog mode after. FDCAN2 is exercised
**without a pad at all**, because internal loop-back needs none. Three
instruments make the chapter measurable with nothing attached: `TEST.TX
= 01` counted by a DMAMUX request generator on the pad's EXTI line with
no CPU in the loop; the CPU polling the pad through one frame in
external loop-back; and **the internal timestamp counter as a bit-rate
meter**, since TSCV counts once per CAN bit time whether or not anything
is on the bus.

**THE FDCAN'S REGISTERS ANSWER THROUGH A CLOSED APB CLOCK GATE, AND ITS
WRITES DO NOT.** With `RCC_APBENR1.FDCANEN` clear at boot, CREL reads
0x32141218, ENDN 0x87654321, CCCR 0x1 and NBTP 0x06000A03 - every one of
them the true reset value, where VREFBUF behind SYSCFG's gate reads a
number that is not its reset value at all (vref.md). A STORE made
through the same closed gate lands nowhere and is not replayed when the
clock returns (FDCAN_IE written 1 and 8 on the two instances read back 0
before and after). 5.2.17 lets a reader assume neither.

**One reset for the subsystem, measured**: NBTP written on FDCAN1, then
`Fdcan<2>::reset()`, and FDCAN1's NBTP is back at 0x06000A03.

**CKDIV is one register and FDCAN1's CCE is what opens it.** /4 written
through FDCAN1 is read back by FDCAN2; the same write attempted through
FDCAN2's own INIT + CCE, with FDCAN1 out of initialization, **does not
land**. 36.4.37 gates the write on "the CCE bit" and does not say whose;
the bench says FDCAN1's, which is what the note's ordering advice ("the
input clock divider must be modified before configuring the other FDCAN
instances") quietly implies.

**The INIT readback costs at most 79 CPU cycles** (1.2 us at 64 MHz)
over eight up-and-down transitions - so 36.4.6's note is not a formality
and it is bounded and short.

**36.3.4's CCE-reset list, register by register.** A loop-back session
that leaves two frames unread in FIFO 0, one in FIFO 1, three TXBTO bits
and three Tx events, then INIT + CCE: HPMS 0xC0, RXF0S 0x20002, RXF1S
0x10001, TXBTO 0x7 and TXEFS 0x1000003 all read their reset values
afterwards, and TOCV is **preset** to TOCC.TOP (0x1234 written, 0x1234
read) - the one entry on the list that is not a reset.

**The message RAM.** The two maps are 0x350 bytes apart and not aliased
(both instances' 212 words written with distinct patterns and read
back); `clear_ram()` zeroes one instance and leaves the other; and every
section start of figure 399 is where the figure puts it, checked by
writing at the computed word offset and reading at the byte address
(0x000, 0x070, 0x0B0, 0x188, 0x260, 0x278).

**THE BIT RATE ON THE PAD, counted with no CPU.** 125 k / 250 k / 500 k
/ 1 Mbit/s from the 64 MHz PCLK measured as 125000 / 250000 / 500000 /
1000050 bit/s, every one inside the HSI16's own 1 %. Through CKDIV /2
and /4 the same NBTP gives exactly 250000 and 125000. And the RESET
NBTP, which 36.4.7's note prices at 3 Mbit/s from 48 MHz, measures
**4000400 bit/s** here - the arithmetic the note does not do.

**WHAT `TEST.TX = 01` PUTS ON THE PAD IS A STROBE AND NOT A LEVEL.**
36.4.4 says only "sample point can be monitored at pin FDCANx_TX". The
pad is high for **1 part per thousand of an 8 us bit and 15 of a 1 us
bit**, so the pulse is a fixed width in kernel clocks and its single
edge per bit time is what the counter counts - it is not a level split
at the sample point, and its duty says nothing about where the sample
point is.

**`fdcan_tq_ck <= fdcan_pclk` (36.3.3) is unreachable by construction on
this part**: the only kernel clock that exists is PCLK itself (the other
two CCIPR2 codes are refused because nothing in `clock.hpp` starts
either) and CKDIV only divides. Stated in the driver, not enforced by
it.

**Classic CAN through the internal loop.** Every DLC from 0 to 8
byte-exact; a 29-bit identifier through ID[28:0] whole where an 11-bit
one lives in ID[28:18]; a remote frame arriving with RTR set, its
identifier intact and no data (and nothing answering it - 36.3.4:
"automated transmission on reception of remote frames is not
supported"). Table 215 read field by field off a real element: R0
0x0AA80554, R1 0x00040000, R2 0xEFBEADDE for a four-byte payload
0xDE 0xAD 0xBE 0xEF, with ESI/XTD/RTR/ANMF clear, FIDX 0, FDF and BRS
clear and DLC 4. The FIFO's bookkeeping: fill level 1, the acknowledge
moving the get index on and the level back to 0.

**IR.TC IS GATED BY TXBTIE AND 36.4.15 DOES NOT SAY SO.** The same
transmission gives IR 0x601 with TXBTIE clear and 0x681 with it set:
"transmission completed" is not raised at all unless the buffer the
frame left from has its own enable bit up, and TXBTIE resets to zero -
so a program that waits on IR.TC out of the box waits for ever, while
TXBTO and the Tx event FIFO report the same transmission either way. The
Tx event carries the message marker back (0x77 in, 0x77 out) with event
type 01.

**Internal loop-back's two promises, both measured**: the TX pad never
moved through twenty frames (zero rising edges counted by the request
generator) and reads recessive throughout, and the RX pad held hard
DOMINANT the whole time changed nothing.

**External loop-back puts the frame on the pin.** An 11-bit DLC-8 data
frame at 500 kbit/s: 60 transitions spanning 200 us = **100 bit times**
from SOF to the last edge, against 98 bits of SOF..CRC before stuffing;
the same frame came back inside byte-exact; the error counters never
moved with PB8 pulled dominant (the pin is disregarded and acknowledge
errors are ignored); and three more frames gave 78 rising edges on the
same pad with no CPU in the loop.

**The filters.** Range (0x100..0x10F), dual (0x200 or 0x2FF) and classic
(0x300 under mask 0x7F0) each with an identifier inside and one outside,
under a non-matching policy of reject; a reject element rejecting a
perfectly good match; **the first match winning** (two elements matching
0x700, #0 to FIFO 0 and #1 to FIFO 1 - the frame is in FIFO 0 with FIDX
0 and #1 was never reached); a priority element raising IR.HPM and
filling HPMS (list standard, FIDX 0, MSI = 10 "stored in FIFO 0", BIDX
1); ANFS routing an unmatched frame to FIFO 1 with the element's own
ANMF set; RRFS rejecting a remote frame before the list is reached while
the frame still goes out; and XIDAM AND-ed with a received 29-bit
identifier before the list runs, so 0x11000010 masked by 0x0FFFFFFF met
element #0's range and never reached element #1, whose EFT = 11 asks for
the raw identifier.

**36.4.19's clamp is IN THE REGISTER**: LSS written 31 by hand reads back
28. The driver refuses 31 anyway, because a program that asks for 31
filter elements has a bug and a silent clamp hides it.

**A FILTER EDITED WHILE THE MODULE IS ON THE BUS TAKES EFFECT AT THE
NEXT FRAME.** 36.3.6 executes the list from element #0 for every
message and the RAM is not a protected register, so what is accepted can
be changed with no INIT at all (0x456 rejected, the element rewritten,
0x456 accepted).

**The Rx FIFOs.** Blocking: the fourth frame into a full FIFO is
discarded, the three already there are untouched, RF0F and RF0L both
raised. Overwrite: the fourth takes the OLDEST one's place, BOTH indices
move on (get 1, put 1), the FIFO still holds three - and **RF0L is NOT
raised**, so a discarded message and an overwritten one are different
events. And **36.3.6's own safe read costs one more message**: starting
at get index + 1, as the chapter requires while the FIFO is full, skips
the oldest element and acknowledging the one actually read carries the
get index past it, so two elements came back where three were stored -
which the chapter asks for and never says.

**The Tx order.** Three frames with DESCENDING identifiers written to
buffers 0, 1, 2 and requested in ONE `TXBAR` write leave as 0, 1, 2 with
`TFQM = 0` (a FIFO keeps the insertion order) and as 2, 1, 0 with
`TFQM = 1` (a queue sends the lowest identifier first) - the exact
reverse, from the same request. 36.4.27's footnote is literal: TFFL and
TFGI **read as zero** in queue mode, so a queue's room is TFQF and not
the free level.

**Cancellation.** A buffer whose transmission had not started: TXBRP
cleared, TXBCF set, TXBTO untouched, IR.TCF raised, and the other buffer
went out normally. One already on the wire: the transmission finishes,
TXBTO and TXBCF are BOTH set and the Tx event carries **type 10**,
"transmission in spite of cancellation". The Tx event FIFO is three deep
and the fourth event is discarded with TEFF and TEFL.

**CCCR.TXP is exactly two CAN bit times.** Two back-to-back frames start
51 bit times apart with the bit clear and 53 with it set, read out of
the Tx events' own timestamps.

**CAN FD, AND THE 0xCC QUESTION SETTLED.** 36.3.4 says that with DLC > 8
"the first eight bytes are transmitted as configured while the remaining
part of the data field is padded with 0xCC", and that a received frame's
"remaining bytes are discarded". **That paragraph does not describe this
part.** Sixty-four DISTINCT bytes (0x40..0x7F) sent in one FD frame with
bit rate switching came back **64 of 64 byte-exact, none of them 0xCC**.
Figure 399 allocates eighteen words - 64 bytes - to every Rx and Tx
element and the element really does carry them; the paragraph belongs to
an M_CAN configured with an eight-byte data field, which this one is
not. All seven long DLC codes (12, 16, 20, 24, 32, 48, 64) are
byte-exact.

Also in FD: ESI forced recessive in the Tx element arrives set in the
element and in PSR.RESI (table 217's footnote); NISO and ISO both loop
perfectly, which is exactly what a loop-back CANNOT tell apart since
both ends change together; and **bit rate switching is visible as
time** - the same 64-byte frame occupies **322 us, 178..179 us and 107
us** of pad as the data phase goes 2, 4 and 8 Mbit/s, with the loop
byte-exact at all three. TDC in loop-back: with TDCO = 16 mtq, PSR.TDCV
reads 18, so **the internal loop delay is 2 mtq**.

**Timestamps.** The internal counter runs on the BIT CLOCK and not on
traffic: 10000 counts in 20 ms on an idle bus at 500 kbit/s (TCP = 1),
and 625 in the same window at TCP = 16. Two back-to-back DLC-8 frames
are 124 bit times apart at RXTS, which is the frame plus the interframe
space. IR.TSW lands at 2097151 us against 65536 x 16 bit times = 2097
ms. And **TSCV is TIM3's counter** when TSS = 10 (the one external
source this part wires up): the register tracked tim3_cnt to within a
microsecond of the timer's own reading, and an element's RXTS is that
counter captured at start of frame (29 us after the request at a 1 MHz
TIM3).

**The timeout counter** counts down in the same units the timestamp
counter counts up: continuous, TOP = 1000 at TCP = 1, IR.TOO after 1999
us against 2000. Under a FIFO, an empty FIFO holds it preset at 1000
(no TOO in 5 ms) and the first element stored starts it.

**THE ERROR MACHINE WITH NO NODE.** Normal mode with the RX pad on its
own pull-up is a recessive "bus": PSR.ACT reads 0 (synchronizing) the
instant INIT is cleared and 1 (idle) 500 us later, so 36.3.4's
integration is measurable. Then every dominant bit the node sends is
read back recessive, so every one of them is a BIT0 error (LEC = 5) and
**TEC moves only in multiples of eight** - the active error flag it
answers with is six more dominant bits that fail the same way, which is
why the whole ladder is walked faster than a 64 MHz CPU can read every
rung of it. **EW first seen at TEC 96..104 and EP at 128..136 across
runs** - the poll reads PSR before ECR, so the counter it reports can
only be at or past the level that raised the flag, never below it -
**and BUS-OFF after 846..847 us**, with IR.EW, IR.EP and IR.BO all
raised and INIT set by hardware.

**The bus-off recovery sequence runs on a pull-up**, and it settles a
disagreement inside the chapter: BO cleared **2842 us = 1421 bit times**
after INIT was cleared, where figure 398 says "128 x 11 recessive bits"
(1408) and 36.4.13's note says the device "waits for 129 occurrences of
bus-idle" (1419). The measurement lands on the note. Both error counters
came back at zero.

**A STUCK-DOMINANT LINE IS NOT AN ERROR, IT IS A WAIT.** With the RX pad
pulled down the module never sees the eleven recessive bits that end
integration: ACT stays synchronizing, the transmission request stays
pending for ever and NEITHER error counter moves (TEC 0, REC 0, CEL 0).

**DAR is exactly one attempt** - TXBRP cleared, TXBCF set, TXBTO clear,
and the node does NOT reach bus-off where a retransmitting one got there
in three milliseconds. The one attempt still cost 136 of TEC, because an
error-ACTIVE transmitter answers its own bit error with a six-bit
dominant error flag that fails the same way until it turns
error-PASSIVE at 128 and its error flags go recessive.

**RESTRICTED OPERATION: 36.3.4 BEATS 36.4.6.** The two descriptions
contradict each other - the functional section says a restricted node
"does not send data frames, remote frames, active error frames, or
overload frames", the ASM bit's own description says it "is able to
transmit and receive data and remote frames" - and the pad settles it:
**not one edge left it** through a whole DLC-8 request, which stayed
pending. The error counters are frozen either way (36.4.12's own
sentence). And 36.3.4's note is enforced on the LIVE verbs in both
directions: with ASM standing `loop_back(true)` is refused, and with
LBCK standing `restricted(true)` is refused.

**Bus monitoring**: 36.3.4's "the FDCAN_TXBRP register is held in reset
state" is literal - the TXBAR write is accepted by the bus and the
pending bit never appears, so the transmission simply does not happen
and nothing reaches the pad.

**The interrupts.** ILS moved for the "status message" group carries TC,
TCF and HPM together and nothing can be split off from its group; one
frame then raised RF0N on line 0 and TC on line 1, each body serving
exactly its own mask. `ILE` gates the LINE and not the flag (with EINT0
clear the interrupt never reached the NVIC while IR still showed 0x601).
TXBTIE is per buffer. And **both modules loop at once and never hear
each other**: FDCAN1 received 0x101 and FDCAN2 received 0x202, each out
of its own message RAM, with one vector serving TIM16 and both
instances and the timer silent.

**Power-down.** On an idle module CSA rises immediately and INIT is set
by the handshake; clearing CSR clears CSA and leaves INIT standing. With
a DLC-8 frame in flight the acknowledge **waited out the whole frame**
(CSA after 251 us) and the transmission occurred. Through a **Stop 1**
the block and its message RAM keep every bit - NBTP, RXGFC and a filter
word written into the RAM all identical afterwards - which is the useful
half of table 27's row, whose other half is that the FDCAN has no Stop
functionality and no wake-up capability of its own.

**Errata.**

- **2.13.1 (desynchronization with edge filtering enabled)** STAGED:
  1500 CAN FD frames with bit rate switching in each arm, with EFBI set
  and with it clear. Every frame arrived, every one byte-exact, PEA and
  PED zero in both. **NOT REPRODUCED**, and recorded as that and not as
  a disproof: the coincidence the item needs is an end of integration
  landing on a falling FDCAN_RX edge, and a loop-back whose RX pin is
  disconnected may never offer one. The bit is a verb with the
  obligation stated on it; nothing a driver can do.
- **2.13.2 (Tx FIFO messages inverted)** is **UNREACHABLE BY
  CONSTRUCTION on this silicon, and TXBC is the evidence**: written
  0xFFFFFFFF it reads back 0x01000000, so the only implemented bit is
  TFQM. The erratum needs "both a dedicated Tx buffer and a Tx FIFO" and
  its own workaround names Tx buffers 4 and 5; this M_CAN is configured
  with THREE Tx elements and one mode bit, so the whole area is either a
  FIFO or a queue and a dedicated buffer cannot be declared at all.

**One console artifact, recorded and not explained.** In the suite's
letter `k`, about six bytes of one informational line come out of USART2
corrupted - the same bytes on every run, on a freshly reset board, with
two independent capture tools (a pyserial reader and `cat`), while every
other line of every other letter is intact. Moving the print after the
power-down is released does not move it; inserting text before it moves
it by exactly that many bytes; draining the console first does not help.
No verdict rests on that line and the letter scores 4/4.

## Not covered yet

Driver gaps - nothing in `fdcan.hpp` reaches these:

- **No task and no util vocabulary.** A CAN bus AO and a shared frame
  type are deliberately not designed from one implementation (above).
- **The PLLQ and HSE kernel clocks** (CCIPR2 codes 01 and 10) are named
  and REFUSED, because `clock.hpp` builds neither the PLL's Q output nor
  HSE on this board.

Implemented but not bench-verified, each with the reason:

- **Everything that needs a second node**: arbitration, a real
  acknowledge, bus-off from traffic rather than from a bit error against
  a pull-up, a foreign frame through the filters, the ESI a genuinely
  error-passive transmitter sends, and whether NISO and ISO really
  disagree on the wire (a loop-back cannot tell, since both ends change
  together). All of it needs a transceiver, a bus and a peer, and this
  desk has none.
- **The RAM watchdog's fault** (`RWD` and `IR.WDI`): the counter is
  written and read back and the register works, but the fault it watches
  for is a message RAM that does not answer, which nothing here can
  provoke.
- **`IR.MRAF`** and the automatic entry into restricted operation it
  causes: the same problem - the Tx handler has to fail to read the RAM
  in time.
- **`IR.ARA`** (access to a reserved address) and the protocol exception
  path (`PXHD`, `PSR.PXE`): the first wants a deliberately malformed bus
  access, the second a frame with FDF and res both recessive, which only
  a second node can send.
- **`TDCF`**, the delay compensation filter window: written and read
  back, but its subject is a dominant glitch inside a received FDF bit,
  which needs a real transceiver's loop.
- **Timestamps from TIM3 across a wrap**, and the timestamp counter in
  CAN FD, where 36.4.8 warns the internal counter is not a constant time
  base because the bit time changes inside the frame.
- **Four of the five pads per signal**: PB8/PB9 are what this suite
  claims; PA11/PA12, PC4/PC5, PD0/PD1, PD12/PD13 for FDCAN1 and
  PB0/PB1, PB5/PB6, PB12/PB13, PC2/PC3, PD14/PD15 for FDCAN2 are
  compile-only here (and the port D pads are not bonded on this
  package).
