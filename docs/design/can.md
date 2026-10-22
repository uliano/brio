# The CAN vocabulary

This page decides what the CAN controllers in the tree say the same
way, in one header (`util/can.hpp`): the classic frame, the bit
timing in human units with the exact search that finds it, the
protocol's error codes, the error state as one observable and the two
counters. There is no bus active object, no arbiter policy and no
concept (the last section says why), and no filters. It is written for
an embedded programmer who has configured a CAN controller's bit
timing from a vendor's calculator and wants to know what a program
writes once for every controller brio has.

The smallest use: a `CanFrame` built with a natural identifier and a
length in bytes, a `CanTiming` found by the family's
`can_timing_for(clock_hz, bitrate_hz, sample)`, and the family's `Can`
or `Fdcan` resource driven from the program's own active object. The
prerequisite that decides correctness: **the timing search is exact
and answers an empty timing for a clock that cannot divide into the
rate**; a program must check the answer rather than assume a rounded
one.

## Born at the third controller

brio shares a vocabulary between strata only once two of them realize
it, because a shape invented from one implementation is that
implementation's accidents with a `util/` name on them. For CAN the
second controller was not enough either: the second bxCAN (the
CH32V203's, which is the STM32F4's design under WCH's names) would
have confirmed every accident of the first. The third input is the
STM32G0's FDCAN, an M_CAN, whose element layout, filter list and
transmit-event FIFO share nothing with a bxCAN's mailboxes and banks,
and what survives all three is what the header holds. What each says
its own way stays in its stratum, and its document names it.

## The frame

`CanFrame` is the classic message: a natural, right-aligned identifier
(eleven bits or twenty-nine, `extended` telling which), the remote
flag, the length in bytes and eight bytes of data. The identifier is
natural because the registers disagree on where to put it (a bxCAN
mailbox in bits 31:21, an M_CAN element in ID[28:18]) and a program
made to shift for one and not the other would get it wrong; the length
is bytes and not a DLC because the DLC is the wire's coding of it,
computed on the way out. Two fields are filled on reception alone: the
index of the filter that admitted the frame and the controller's
counter at its start of frame. Both controllers publish both, in units
and under conditions their documents state (a bxCAN stamps only in its
time-triggered mode, where the STM32F4's errata forbid that mode).

The FD superset (sixty-four bytes, the FD flag, the bit-rate switch,
the error state indicator, a transmit marker) is the M_CAN stratum's
`FdcanFrame` and stays there: the classic frame is what every node on
a classic bus speaks, and an FD vocabulary written against one FD
controller would be that controller's. It is written at the second FD
controller.

## The bit timing

`CanTiming` states a bit the way a human does: a prescaler that
divides, two segments that count quanta, a jump width in quanta, each
one more than its register field holds. The arithmetic over it
(`quanta`, `bitrate_of`, `sample_point_of`) is the protocol's. What a
controller's registers allow is not the vocabulary's to know: it
travels as a value, `CanTimingLimits` (the widest divider, segments
and jump, and the quanta a bit may hold), which each stratum states
from its own chapter and binds into its own two functions,
`can_timing_for(clock_hz, bitrate_hz, sample)` and
`can_timing_valid(timing)`, the way `apb_hz(clock, bus)` binds a family
fact. A program calls the stratum's functions and never sees the
limits.

The search (`can_timing_search`) is exact: a bit rate a fraction of a
per cent off is a bus that works between two nodes of one crystal and
fails when the third joins, so a clock that cannot divide into the
rate yields an empty timing and never a rounded one. Its rule: quanta
counts from the widest down, the sample point as close to the one
asked as the segments allow, the first count whose best beats every
wider one's wins, the jump width the second segment capped at the
controller's widest.

One divergence is recorded and not unified. The STM32G0's FDCAN keeps
a timing struct of its own in register units and a search of its own
with a different rule (the smallest prescaler that admits an exact
bit, the sample point refused when it lands more than one per cent
away), because its data phase has a second set of registers with far
narrower fields and its codecs speak register values throughout. Both
rules are measured on a bus; unifying them would move measured
constants in one stratum for no bus's benefit, and stays an open item.

## Errors and the error state

`CanError` is the last-error code every ISO 11898-1 controller
publishes in one three-bit field (bxCAN's LEC, M_CAN's LEC and DLEC):
none, stuff, form, acknowledge, bit recessive, bit dominant, CRC, and
the seventh, `no_change`, which software writes so that a later read
can tell nothing has happened since (an M_CAN writes it back on every
read of its own).

`CanErrorState` is the protocol's fault confinement as one observable
(active, warning, passive, bus-off), built from the three flags a
controller raises by `can_error_state()`, the deepest standing one
winning; `CanErrorCounters` are the two counters behind it. They are
here because a bus active object, when it exists, reports the state
the way `PrepareSleep` and `WakeReport` report (it is not a return
code of any function), and a shared report needs a shared type before
the active object does.

## What stays in each stratum

- **The filters.** A bxCAN filters through banks of two registers with
  four shapes (one 32-bit or two 16-bit filters, in mask or in list
  mode, assigned to one of two FIFOs); an M_CAN through a list of
  elements each with a type (range, dual, classic mask) and an action
  (store here, reject, prioritise). These are two models of one idea,
  and a shared type would be one wearing the other's name.
- **Mailboxes and FIFOs.** Three transmit mailboxes with two
  scheduling orders against a transmit FIFO or queue with an event
  FIFO beside it; two receive FIFOs of three against two of
  configurable depth in a message RAM the program lays out.
- **The transmit outcome.** A bxCAN's mailbox flags (completed, sent
  and acknowledged, arbitration lost, error) and an M_CAN's transmit
  event (completed, or completed in spite of a cancellation, with the
  marker the program gave the frame). The bxCAN stratum carries it as
  `CanTxResult`; it becomes the vocabulary's if another controller
  reports the same four facts.
- **The modes.** Loopback, silent, the M_CAN's restricted operation
  and its external loopback: instruments of each chapter's suite.

## The bus active object, still absent

Every other shared bus in brio is served by `BusMaster`: one request,
one reply. CAN does not fit that shape, for three reasons measured on
the STM32G0's FDCAN and confirmed by the STM32F4's bxCAN:

- **Completion is not a reply.** A transmission completes, is
  cancelled, completes in spite of the cancellation, or loses
  arbitration and is retried invisibly; its status arrives in a
  separate stream with its own overflow. What "done" means to a
  requester is a decision the two controllers answer differently.
- **Reception is routing.** Frames arrive unrequested, admitted by
  filters into FIFOs with a blocking or overwrite policy and a
  high-priority side channel; there is no requester to reply to, and a
  receive active object needs the filter vocabulary this page
  declines.
- **A stuck-dominant bus is silent.** A transmission into it pends
  for ever with no flag at all, so whatever active object is written
  owes its clients the per-bus timeout `BusMaster` already has.

The active object is written when a program needs one, over this
vocabulary; until then each stratum's `Can` or `Fdcan` is a resource a
program drives from its own active object, as the suites do.

### Realizations

Common to the two: `CanError` as the last-error code, the natural
identifier and the length in bytes on whatever frame the stratum
speaks, the sample point in per mille. What differs is the frame's
reach, the timing's units and the search's rule, and how the error
state is read.

| stratum | realization | beyond the vocabulary |
|---|---|---|
| stm32g0 | `Fdcan` (`stm32g0/fdcan.hpp`) over the M_CAN | `FdcanFrame`, the FD superset (sixty-four bytes, FD, BRS, ESI, the marker, the non-matching flag) with its own codecs; `FdcanBitTiming` in register units for both phases and `fdcan_bit_timing_for` / `fdcan_data_timing_for` under the smallest-prescaler rule; the error state as three flags of `FdcanStatus` and `FdcanErrorCounters` with the receive-passive flag and the logging counter beside the two; `FdcanError` is `CanError` under the chapter's name ([the STM32G0 FDCAN page](../stm32g0/fdcan.md)) |
| stm32f4 | `Can<1|2|3>` (`stm32f4/can.hpp`) over the bxCAN | `CanFrame` and `CanTiming` as the vocabulary has them; `can_timing_limits` from RM0390 30.9.7 (a ten-bit divider, four-, three- and two-bit segments, three to twenty-five quanta) bound into `can_timing_for` / `can_timing_valid`; the error state as `error_warning()` / `error_passive()` / `bus_off()` and the counters as `tec()` / `rec()`, with `error_state()` and `error_counters()` folding them into the vocabulary's types; `CanTxResult` the mailbox's four flags; the filter banks, the two FIFOs, the three modes ([the STM32F4 CAN page](../stm32f4/can.md)) |
| ch32vx03 | none | the bxCAN of the CH32V203 and the CH32V303 (the STM32F4's design under WCH's names, one controller on each part) waits for the pass that brings every platform's CAN onto a bus with a second transceiver ([the CH32V203/CH32V303 page](../ch32vx03/README.md)) |
