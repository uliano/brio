# bxCAN - the controller area network (STM32F4)

Documents of record: RM0390 Rev 6 ch. 30 (RM0090 Rev 22 ch. 32 is the
same chapter for the F42x/F43x class), the datasheets' alternate-function
tables for the pads (DS10693 Rev 11 table 11 for the F446, DocID024030
Rev 10 table 12 for the F429 - CAN1 and CAN2 are AF9 on both), and ONE
LIVE ERRATUM: ES0298 2.15.1 and ES0206 2.13.1, "bxCAN time-triggered
communication mode not supported", every revision of the F446 and of the
F427/F437/F429/F439, with no workaround. Driver: `stm32f4/can.hpp`
(`Can<1|2|3>`, `CanFrame`, `CanTiming`, `CanFilter`, `CanPins`,
`CanOptions`, `CanError`, `CanInterrupt`), over the reserve's
`can_base`/`can_present`/`can_clock_mask`/`can_reset_mask`/
`can_filter_master`/`can_filter_banks`/`can_dual`/`can_tx_irq`/
`can_rx_irq`/`can_sce_irq`/`can_ttcm_erratum`
(`stm32f4/device_tables.hpp`). The family fixture is
`test/family_stm32f4/can.cpp` with five negatives. Bench:
`test_stm32f4_misc`, letters `c` to `k`, on an STM32F446 with no
transceiver and no peer.

## No util vocabulary, on purpose

brio has a shared vocabulary for SPI and for I2C because two strata
implement each of them; it has none for CAN, and the design note that
says so is the open item "the SAM's CAN (two transceivers and the util
vocabulary a shared frame type must carry)". A vocabulary invented from
one realization is a vocabulary shaped by one silicon's accidents - a
bxCAN frame carries a filter match index and a 16-bit time stamp, an
M_CAN frame carries neither and carries a message-RAM element index
instead. So `CanFrame` is a plain struct of this stratum's, there is no
AO, no `BusMaster` policy and no concept, and the shared type gets
written when a second family brings CAN.

## What the silicon does

**Three instances at most, and one filter block for two of them.** CAN1
and CAN2 come as a PAIR on every part that has CAN at all (the F405
class and up, less the F401, F410 and F411, which have none); the
F413/F423 add a third. CAN1 is the MASTER: the filter registers are its,
and CAN2's filters are banks of CAN1's block above CAN_FMR.CAN2SB - so
filtering on CAN2 means writing CAN1's registers and needing CAN1'S
CLOCK ON. CAN3, where it exists, is a master with a block of its own.
Every instance sits on APB1, so the bit time is counted in PCLK1
periods.

**Three modes, each a request with an acknowledge.** INRQ and SLEEP are
asked for in CAN_MCR; INAK and SLAK in CAN_MSR are the silicon saying it
has arrived (30.4). THE RESET STATE IS SLEEP, not normal: a block whose
clock was just turned on transmits nothing until it is woken. Leaving
initialization or sleep also costs a SYNCHRONIZATION - eleven
consecutive recessive bits on the receiver's input.

**Loopback and silent are the wireless instruments - and loopback still
wants its receive pad.** In loop back mode the node receives its own
transmissions through an internal feedback from Tx to Rx, ignores
acknowledge errors and, says 30.5.2, disregards "the actual value of the
CANRX input pin". Measured (letter `c`): that is true of the DATA and
NOT of the SYNCHRONIZATION - a node in loopback whose receive pad is
held DOMINANT never leaves initialization mode, and the same node with
the pad claimed and pulled up leaves it at once. In silent mode the node
listens without ever driving a dominant bit and CANNOT START A
TRANSMISSION AT ALL (30.5.1). The two together are the "hot self-test"
of 30.5.3.

**The bit time is the APB clock divided twice.** tq = (BRP + 1) x
tPCLK, the bit is 1 + TS1 + TS2 quanta (3..25 of them), and the sample
point sits after the first 1 + TS1 (30.7.7). CAN_BTR is writable ONLY in
initialization mode (30.9.1), and so are the two test-mode bits that
share it. `can_timing_for()` searches the whole space at COMPILE time
for an EXACT bit rate - a CAN bus with a rounded bit rate is a bus that
works until the third node joins - and reports the sample point it
reached; a rate the APB clock cannot divide into exactly comes back as
an empty timing, not a near miss.

**Three transmit mailboxes and two orders.** With more than one pending,
the scheduler picks by IDENTIFIER (lowest wins, ties by mailbox number)
or by REQUEST ORDER when MCR.TXFP is set (30.7.1). A mailbox is
write-protected unless it is empty (TME), and RQCP is the completion
flag whose clear also clears TXOK, ALST and TERR for that mailbox.

**Two receive FIFOs of three, and the overrun policy is a choice.** FMP
counts what is pending, FULL says three are, FOVR says a fourth arrived
- and MCR.RFLM decides which message is lost: with the lock clear the
newest overwrites the last one stored, with it set the newest is
discarded and the three oldest survive (30.7.3). The output mailbox is
released with RFOM, and until it is the next message is unreachable.

**Twenty-eight filter banks in four shapes.** Each bank is one 32-bit
filter or two 16-bit ones (FS1R), in mask or in list mode (FM1R),
assigned to FIFO 0 or FIFO 1 (FFA1R) and active or not (FA1R). The
FILTER MATCH INDEX a received frame carries is a number over the ACTIVE
filters of that FIFO in the chapter's priority order (32-bit before
16-bit, list before mask, low bank before high) and NOT the bank number.
Configuration is legal only with FINIT set or the bank deactivated
(30.9.1), and FINIT DEACTIVATES RECEPTION while it stands (30.4.1).

**The error counters are the fault confinement, and they are readable.**
TEC and REC in CAN_ESR, with EWGF at 96, EPVF past 127 and BOFF past 255
- and CAN_ESR carries only the LOW BYTE of a nine-bit transmit counter.
Leaving bus-off costs 128 occurrences of 11 recessive bits either way;
ABOM decides whether the hardware does it alone or waits for software to
set and clear INRQ. LEC names the last error the core saw, and its
seventh code is the one SOFTWARE writes to tell a later read that
nothing has happened since.

**Four vectors per instance**: transmit (the three mailboxes together),
FIFO 0, FIFO 1, and status-change-and-error, the last carrying the error
flags, the wake-up and the sleep acknowledge behind ERRI, WKUI and
SLAKI, which are rc_w1 (30.9.2). No header of this family shares any of
them with another peripheral.

**THE ONE ERRATUM, AND IT IS LIVE.** Time-triggered communication mode
"is not supported", no time stamp is available, and CAN_MCR.TTCM "must
be kept cleared" - every revision of the F446 and of the F427/F437/
F429/F439, no workaround. `Can<n>::options()` REFUSES a configuration
that asks for TTCM on those part classes and writes nothing;
`time_triggered_supported()` says whether it would. The TIME field a
received frame carries is left in `CanFrame` and is meaningless without
the mode. On the part classes whose errata sheet was not read (the F405,
F412, F413/F423 and F469/F479) the bit is writable.

## Types and verbs

The vocabulary, which compiles on every part of the family - including
the three with no controller, so that a portable program can name a bit
rate anywhere:

| Name | Meaning |
|------|---------|
| `CanFrame` | a message: id, extended, remote, dlc, data, and on reception the filter index and the time stamp |
| `can_frame_valid` | eleven or twenty-nine bits of identifier, at most eight bytes |
| `CanTiming` | the bit time in human units: brp 1..1024, ts1 1..16, ts2 1..8, sjw 1..4 |
| `can_timing_valid`, `can_timing_quanta`, `can_bitrate_of`, `can_sample_point_of` | what a timing is and what it produces |
| `can_timing_for` | the compile-time search: an EXACT rate, the sample point as asked |
| `can_frame_bits` | 44 + 8N standard, 64 + 8N extended, before stuffing (figure 396) |
| `CanFilterScale`, `CanFilterMode`, `CanFilter` | a bank's shape and its two registers |
| `can_filter32`, `can_filter32_flags_mask`, `can_filter16`, `can_filter16_pair`, `can_filter_accept_all` | figure 391's mappings, as arithmetic |
| `CanError` | CAN_ESR.LEC's seven codes |
| `CanTxResult` | RQCP, TXOK, ALST, TERR for one mailbox |
| `CanInterrupt` | the bits of CAN_IER, one enumeration |
| `CanPins`, `can_pins_valid` | the two pads, each with the AF the datasheet gives it |
| `CanOptions` | CAN_MCR's behaviour bits as one configuration |

The resource `Can<n>`:

| Purpose | Verbs |
|---------|-------|
| identity | `instance`, `filter_master`, `filter_banks`, `time_triggered_supported()` |
| the clock gates | `clock(bool)`, `clock()`, `filter_clock(bool)`, `filter_clock()`, `reset_block()` |
| the vectors | `tx_irq()`, `rx_irq(fifo)`, `sce_irq()` |
| the pads | `claim_pads<pins>(pull, speed)`, `claim_rx_pad<pin>(pull)` |
| the modes | `init_mode(bool)`, `start()`, `sleep(bool)`, `in_init()`, `in_sleep()`, `in_normal()`, `master_reset()` |
| the options | `options(CanOptions)`, `options()` |
| the bit time | `timing(CanTiming, loopback, silent)`, `timing()`, `loopback()`, `silent()` |
| transmitting | `transmit(frame)`, `abort(mb)`, `mailbox_empty(mb)`, `next_mailbox()`, `free_mailboxes()`, `lowest_priority(mb)`, `result(mb)`, `clear_result(mb)` |
| receiving | `pending(fifo)`, `full(fifo)`, `overrun(fifo)`, `clear_full`, `clear_overrun`, `peek(fifo)`, `receive(fifo)`, `release(fifo)` |
| the errors | `tec()`, `rec()`, `error_warning()`, `error_passive()`, `bus_off()`, `last_error()`, `mark_error()`, `request_bus_off_recovery()` |
| the status register | `rx_level()`, `last_sample()`, `receiving()`, `transmitting()`, and the three rc_w1 flags with their clears |
| the interrupts | `interrupt(CanInterrupt, bool)`, `interrupt(CanInterrupt)`, `interrupts_off()` |
| the filters | `filter_init(bool)`, `filter_initializing()`, `start_bank(n)`, `start_bank()`, `first_bank()`, `bank_limit()`, `bank_ok(n)`, `filter(CanFilter)`, `filter(n)`, `filter_active(n, bool)`, `filter_active(n)`, `filters_off()` |
| the ISR bodies | `tx_isr()`, `rx_isr(fifo)`, `sce_isr()` |
| teardown | `release_instance()` |

Two refusals are the driver's and not the silicon's, because the silicon
has none: a bank on the WRONG SIDE of CAN2SB (the write would land and
the frames it admits would arrive at the other instance), and every
filter verb on `Can<3>`, whose bank count is a fact of RM0430 and not on
this desk.

## How to use it

A node on a real bus, at 500 kbit/s:

```cpp
using Bus = brio::Can<1>;
constexpr brio::CanPins pads{.rx = {'A', 11, brio::PinFunction::af9},
                             .tx = {'A', 12, brio::PinFunction::af9}};
constexpr auto timing = brio::can_timing_for(brio::apb_hz(clock, false), 500'000);
static_assert(brio::can_timing_valid(timing), "no exact 500 kbit/s from this PCLK1");

Bus::clock(true);
Bus::filter_clock(true);
Bus::claim_pads<pads>();
(void)Bus::init_mode(true);
(void)Bus::options({.auto_bus_off = true});
(void)Bus::timing(timing);
(void)Bus::filter_init(true);
(void)Bus::filter(brio::can_filter_accept_all(0));
(void)Bus::filter_init(false);
if (!Bus::start()) {
    // eleven recessive bits never came: nothing is driving the bus
}
```

Sending and receiving:

```cpp
brio::CanFrame out{.id = 0x123, .dlc = 2};
out.data[0] = 0xA5;
out.data[1] = 0x5A;
if (const auto mb = Bus::transmit(out)) {
    // ... and later, or in the transmit vector:
    if (Bus::result(*mb).ok) { (void)Bus::clear_result(*mb); }
}
while (Bus::pending(0) != 0u) {
    if (const auto in = Bus::receive(0)) { handle(*in); }
}
```

A self-test with no transceiver and no peer - the pad claimed only so
that the receiver sees a recessive line:

```cpp
Bus::claim_rx_pad<pads.rx>(brio::PinPull::up);
(void)Bus::init_mode(true);
(void)Bus::timing(timing, /*loopback=*/true);
(void)Bus::start();                                  // the frames come back
```

Filters, in each of the four shapes:

```cpp
// one 32-bit mask: the top seven bits of the identifier must match
(void)Bus::filter({.bank = 0, .scale = brio::CanFilterScale::single32,
                   .mode = brio::CanFilterMode::mask, .fifo = 0,
                   .r1 = brio::can_filter32(0x240, false, false),
                   .r2 = brio::can_filter32(0x7F0, false, false)});
// two exact identifiers in one bank
(void)Bus::filter({.bank = 1, .scale = brio::CanFilterScale::single32,
                   .mode = brio::CanFilterMode::list, .fifo = 0,
                   .r1 = brio::can_filter32(0x111, false, false),
                   .r2 = brio::can_filter32(0x222, false, false)});
// four exact identifiers in one bank
(void)Bus::filter({.bank = 2, .scale = brio::CanFilterScale::dual16,
                   .mode = brio::CanFilterMode::list, .fifo = 1,
                   .r1 = brio::can_filter16_pair(brio::can_filter16(0x401, false, false),
                                                 brio::can_filter16(0x402, false, false)),
                   .r2 = brio::can_filter16_pair(brio::can_filter16(0x403, false, false),
                                                 brio::can_filter16(0x404, false, false))});
```

The second instance, whose filters are the first's:

```cpp
using Slave = brio::Can<2>;
Slave::clock(true);
Slave::filter_clock(true);          // CAN1's gate: the registers are its
(void)Slave::start_bank(14);        // banks 0..13 CAN1's, 14..27 CAN2's
(void)Slave::filter(brio::can_filter_accept_all(14));
```

Under the interrupts, with the app binding the four vectors:

```cpp
Bus::interrupt(brio::CanInterrupt::fifo0_pending, true);
Bus::interrupt(brio::CanInterrupt::bus_off, true);
brio::Nvic::enable(Bus::rx_irq(0));
brio::Nvic::enable(Bus::sce_irq());
// in the app's handlers:
const auto rx = Bus::rx_isr(0);     // FULL and FOVR cleared, the frames left
const auto sce = Bus::sce_isr();    // ERRI/WKUI/SLAKI cleared, ESR reported
```

## Bench findings

From `test_stm32f4_misc` letters `c` to `k` on an STM32F446 at 180 MHz,
PCLK1 45 MHz, with no transceiver, no peer and no wire - the receive
pads claimed as inputs with their pull-ups and the transmit pads never
claimed, so the chip drives nothing:

- **A NODE IN LOOPBACK STILL NEEDS ELEVEN RECESSIVE BITS ON ITS RECEIVE
  PAD.** With CAN1_RX claimed and pulled DOWN the node never leaves
  initialization mode, in loopback or in loopback with silent; with the
  same pad pulled UP it leaves at once. 30.5.2's "the actual value of
  the CANRX input pin is disregarded" holds for the DATA and not for the
  synchronization that ends initialization. (With the pad unclaimed the
  result is neither - an unclaimed pad has no level, and both outcomes
  were seen.)
- The reset state is SLEEP with SLAK set (MCR 0x00010002, MSR 0x0C0A);
  entering initialization clears it, and the master reset puts CAN_MCR
  back to its reset value.
- Every option bit of CAN_MCR is written and reads back, and **TTCM is
  refused** on this part class by the driver, which writes nothing.
- **The timing search is exact at 45 MHz** for all four classic rates:
  1 Mbit/s BRP 3, TS1 12, TS2 2 (15 tq, sample point 866 per mille);
  500 kbit/s BRP 6 and 250 kbit/s BRP 12, the same 15 tq; 125 kbit/s
  BRP 45, TS1 6, TS2 1 (8 tq, 875 per mille). CAN_BTR reads back
  0xC11B0005 for the 500 kbit/s loopback-and-silent case, and a timing
  written outside initialization mode is refused with the old one left
  standing.
- **A frame round-trips byte-exact** at every data length from zero to
  eight, with an eleven-bit identifier at both ends of its range, with a
  29-bit identifier (0x1ABCDEF1), and as a remote frame (RTR set, DLC 4,
  no data).
- **The filters, in their four shapes**: a 32-bit mask 0x240/0x7F0
  admits 0x24A and rejects 0x250; a 32-bit list bank admits exactly
  0x111 and 0x222 and reports filter match indices 0 and 1; a 16-bit
  list bank holds four identifiers (0x401..0x404) with indices 0 and 3
  at its ends; two 16-bit mask filters in one bank each admit their own
  group. With every bank deactivated the hardware discards the frame and
  the software never sees it, and a bank assigned to FIFO 1 delivers
  there and not to FIFO 0.
- **The scheduler's two orders.** Three mailboxes loaded WHILE THE BLOCK
  IS IN INITIALIZATION MODE (where nothing transmits) with identifiers
  0x300, 0x100, 0x200: with TXFP clear they arrive 0x100, 0x200, 0x300 -
  by identifier; with TXFP set they arrive 0x300, 0x100, 0x200 - in
  request order.
- **The overrun policy.** Four frames into a three-deep FIFO leave FMP
  3, FULL and FOVR set. With RFLM clear the FIFO holds 1, 2, 4 - the
  LAST STORED message is the one overwritten, so the newest survives;
  with RFLM set it holds 1, 2, 3 - the newest is discarded.
- **The frame time is the bit time.** A DLC-8 standard frame measured
  from the TXRQ store to RQCP: 109 us at 1 Mbit/s, 218 at 500 kbit/s,
  438 at 250 and 878 at 125 - 100 to 101 per cent of the 108 unstuffed
  bit times, and 8.0 times as long at 125 kbit/s as at 1 Mbit/s.
- **Silent mode cannot start a transmission.** A request in silent mode
  is still pending after 20 ms; an abort empties the mailbox, and the
  error counters never move - a node that never transmits never fails to
  be acknowledged.
- **Nobody to acknowledge.** In normal mode against the pad's own
  pull-up, one failed attempt costs the transmit error counter 8 or 16
  (a multiple of eight: one for the frame, one more for each error flag
  the node cannot get out either) and the last error code is 5, a bit
  dominant error - the transmitter sends a dominant bit and monitors the
  recessive pad, so the failure comes before the acknowledge slot and
  never reaches an acknowledge error. The warning and error-passive
  flags are passed on the way and BUS-OFF is reached in 16 one-attempt
  requests, with CAN_ESR showing 248: the low byte of a counter that is
  nine bits wide.
- **Bus-off recovery.** With ABOM clear the node is still bus-off 30 ms
  later. With ABOM set the recovery took **1407 us at 1 Mbit/s** -
  128 x 11 = 1408 bit times, to the microsecond - and left TEC and REC
  at 0.
- **The pair's filter block.** CAN_FMR.CAN2SB reads 14 out of reset, so
  CAN1 owns banks 0..13 and CAN2 banks 14..27; the split reads back
  where it is written, each instance is refused a bank on the other's
  side, and a frame round-trips on CAN2 through a bank of CAN1's block
  with CAN1 itself in sleep. **The filter match index CAN2 reports is
  relative to its own range**: a frame caught by bank 14, the first of
  CAN2's, comes back with FMI 0.

## Not covered yet

Driver gaps:
- **Everything about a real bus**: arbitration between two nodes, an
  acknowledge from a peer, a remote frame answered, the error counters
  moving under real traffic, the wake-up from bus activity (AWUM and the
  WKUI interrupt, which need a dominant edge from somewhere). This desk
  has no CAN transceiver and no second CAN node; every finding above is
  a node talking to itself or to silence.
- **CAN3's filter block** (the F413/F423's): the header gives all three
  instances the same 28-bank struct, but how many banks that separate
  block really has is a fact of RM0430, which is not on the desk, so
  `can_filter_banks(3)` is 0 and every filter verb on `Can<3>` refuses
  rather than write a bank that may not exist. One row of the reserve
  when the manual is read.
- **Time-triggered communication** (TTCM, the 16-bit counter and the
  time stamps in CAN_TDTxR/CAN_RDTxR, and TGT which sends the stamp in
  the last two data bytes): refused by the driver on the two part
  classes whose errata sheet forbids it, and not built for the others -
  no program here needs it, and the classes where it might work are the
  ones whose errata were not read.
- **A util-level CAN vocabulary, an AO, a `BusMaster` policy**: declined
  until a second family brings CAN (see "No util vocabulary" above).
- **The debug freeze in practice** (MCR.DBF and DBGMCU's per-instance
  bits): the bit is written and read back, but what a halted core does
  to a live bus is a debugger session's question, not a suite's.

Implemented, not bench-verified:
- **The four vectors and the three ISR bodies.** Every letter polls;
  nothing here binds a CAN vector, because a loopback node's traffic is
  the program's own and polling shows the same flags. What would measure
  them: a letter that arms `fifo0_pending` and counts entries against
  frames sent, and one that arms `bus_off` and catches the transition
  letter `j` provokes.
- **`claim_pads()` with both pads**, and therefore the transmit pad's
  alternate function and slew class: the suite claims receive pads only,
  on purpose. A transceiver would measure it.
- **`sleep()` leaving on bus activity** (AWUM) and the WKUI flag: the
  sleep entry and exit are measured, the wake-up EVENT is not - it needs
  a dominant edge from another node.
- **`request_bus_off_recovery()`**, the ABOM-clear path: letter `j`
  measures that the node STAYS bus-off with ABOM clear, and then
  recovers it by setting ABOM. What would measure the verb: the same
  letter asking for the software recovery instead, and timing it against
  the 1407 us the automatic one took.
- **`Can<3>`** in every respect: no part on this desk carries a third
  instance.
- **Bit rates above 1 Mbit/s** (the chapter's ceiling) and below
  125 kbit/s: the four measured are the classic ones; the search is
  exercised on others only in the fixture's constant expressions.
