# DMAC (SAM C21)

Documents of record: SAM C20/C21 data sheet DS60001479M ch. 25, and
errata DS80000740S items 1.10.1..1.10.4, whose matrix is read below for
this chip (E/G/J family, silicon rev F). Driver: `samc21/dmac.hpp` -
ONE user, the Uart's transmit engine `DmaTxEngine` ([sercom.md](sercom.md)),
on ONE channel. Every other use of the controller is declined: the
reasons are erratum 1.10.4 and the measurements below. Family fixture
`test/family_samc21/sercom.cpp` and the negatives
`dmac_one_channel.cpp` and `uart_receive_engine_slot.cpp` under
`brio check samc21`; the engine's bench letters are `test_samc_uart`'s
(sercom.md) - letter d reads its level back, letter w measures what a
release leaves the next owner.

## What the silicon does

**Twelve channels behind one register set.** CHCTRLA, CHCTRLB, the
interrupt registers and CHSTATUS talk to whichever channel CHID names
(25.8.17), and every descriptor lives in SRAM: a first-descriptor section
at BASEADDR and a WRITE-BACK section at WRBADDR, one 16-byte entry per
channel (25.6.2.3). One vector serves all twelve; INTPEND names the
lowest channel with a pending interrupt and its flags, and a write of
{flags, id} clears them (25.8.10).

**The write-back is the transfer, not a report.** "For an ongoing block
transfer, the descriptor will be fetched from the write-back memory
section" (25.6.2.6): when a block is interrupted by the arbitration or
resumed, the controller continues from what it wrote back. Whatever
corrupts that entry corrupts the transfer.

**No hardware circular mode.** 25.6.3.1 offers a self-linked descriptor
and nothing else; a channel that repeats does so from linked descriptors
or from the CPU.

**Four arbitration levels, each switched on by hand.** A channel takes
one of four levels (CHCTRLB.LVL), and while every level is enabled a
higher number wins; within a level the lower channel number wins, or a
round robin does (PRICTRL0.RRLVLENx); a level is enabled by CTRL.LVLENx,
and a channel at a level not enabled is invisible to the arbiter
(25.6.2.4; measured). The DMAC is also a host of the SRAM with a
quality of service of its own (QOSCTRL, 25.8.7; 10.4.3), a separate knob
from the channel's level. THE ENGINE ARMS AT LEVEL 2, the rule's next
for a transmit ([../design/dma.md](../design/dma.md)): a starved
transmit only leaves its line idle a while. With one channel in the
image the level ranks it against nothing, and it is the rule's so that
the day the controller admits a second user its order is already right;
QOSCTRL stays at its reset value.

**A request is the trigger line's level, and the channel keeps none.**
CHCTRLB.TRIGSRC names one peripheral line, and a SERCOM's transmit line
is a level - "set when the transmit buffer (TX DATA) is empty", "cleared
when DATA is written" (31.6.4.1). A request standing when the channel is
enabled, or when its trigger is selected onto it, is served at once: the
enable of a block fires its first beat by itself. A request that came
and went while the channel was disabled is not kept - a disabled channel
leaves the queue of pending ones and its PEND is cleared (25.6.2.4,
25.8.23) - measured below.

What the one engine relies on beyond that - the end address of an
incrementing side, the disable that drains before it clears, the silent
SWRST, BLOCKACT, the standing request served on the enable - is written
in the driver's header, each with its section.

## Erratum 1.10.4, and the revision matrix

DS80000740S 1.10.4, "Concurrent channels triggers": "When using
concurrent channels triggers, the DMAC write-back descriptors may get
corrupted." Workaround: "Multiple transfers must only be sequenced using
linked descriptors on a single channel" - which is concurrency itself
forbidden.

The matrix, read by ROW (an X marks an affected revision):

| item | E/G/J | N |
|---|---|---|
| 1.10.1 CRCDATAIN writes | B | - |
| 1.10.2 Fetch error (linked descriptors) | B, C, D | E, F |
| 1.10.3 Enabling channels (linked descriptors) | B, C, D | E, F |
| 1.10.4 Concurrent channels triggers | E, F, H | - |

On the E/G/J die the two linked-descriptor items leave at revision E,
where 1.10.4 arrives; the N die lists 1.10.2 and 1.10.3 at E and F and
not 1.10.4. The bench chip is an ATSAMC21J18A at revision F: 1.10.4 is
live and the other three are not. THE TRAP: for 1.10.2 and 1.10.3 it is
the N row that carries marks under E and F - read the row, not the
column.

## Why the controller is refused, measured (ATSAMC21J18A, rev F)

- **The Uart's two engines on one SERCOM**, echoing at 115200: the
  erratum struck in nineteen echoes of twenty. The transmit channel ran a
  write-back holding the RECEIVE descriptor (BTCTRL 0x0809, DATA as both
  addresses) and wrote received bytes into the SERCOM's own registers
  below DATA - INTENSET scribbled, an interrupt storm, the transmitter
  wedged, the board once silent. Its first-descriptor slot was intact:
  the corruption is the live copy's alone.
- **One transmit engine beside two memory-to-memory channels** triggered
  without a wait: the engine's own state was found in another channel's
  write-back. The erratum reaches across owners - no driver sees the
  pair it corrupts.
- **The SPI host's full-duplex pair**, in loop-back, 5 to 256 frames at
  1.5 to 12 MHz, both channel orders, every request judged (completion,
  both write-backs, BUFOVF, every byte): ALONE, zero faults in 24.8
  million requests (below 1.2 x 10^-7 a request at 95 %). Beside ONE
  more active channel - a memory copy re-armed without pause, or a
  timer-paced loop at 100 kHz or 1 MHz - 4.3, 6.4 and 7.8 % of requests
  hung, up to nearly all long ones at 12 MHz: the victim channel enabled
  with no PEND, no BUSY, no FERR and no TERR, its write-back holding a
  higher-numbered channel's BTCTRL and SRCADDR over its own DSTADDR; and
  367 requests COMPLETED reporting success with their bytes missing.
  The third channel was never the victim.
- **The count of channels, not the pair.** With two active channels,
  every shape measured was clean - the SPI transmit channel alone beside
  a copy or a paced loop (zero in 10.2 million write-only requests), and
  two channels of any other owners. With THREE or more, every shape
  faulted, the SPI absent included (a memory copy corrupted in 4.3 % of
  its blocks beside two paced loops); the ONE-channel write-only SPI
  path hung beside a copy and a loop, its write-back carrying the loop's
  descriptor; the receive-only SPI shape beside the same two hung in
  7.7 % of its requests. The highest-numbered active channel was the one
  never hit.
- So the hazard is any second channel a program adds beside two, of any
  owner, and no software rule across owners can be enforced: a DMA that
  could serve one user per image does not pay for the code and the
  guards it would need. One channel alone is outside the erratum by its
  own text, and it is the vendor's workaround.

## The one use admitted, the rule, and how it is enforced

THE USE: the Uart's TRANSMIT direction - the bulk direction, the one
whose bytes the program produces - on channel 0, byte beats, one block a
contiguous run of the transmit ring. It takes the console's transmitter
off the core: at 1 Mbaud 1 % of the core (24597 cycles busy over a
4096-byte run), where the interrupt transmitter's entry of some 190
cycles is two fifths of every 480-cycle frame, and 1.02 x the wire at
3 Mbaud, where the interrupt transmitter runs 2.05 x ([sercom.md](sercom.md),
[../design/benchmark.md](../design/benchmark.md)).

THE RULE: at most ONE DMA user at a time in an image.

THE ENFORCEMENT, in three layers:
- structural: the driver offers one channel and nothing that could start
  another - no channel type, no other engine, no trigger vocabulary on
  any peripheral, and no register of the block reachable through brio
  but its NVIC line. `DmaTxEngine<1>` does not compile
  (`neg/dmac_one_channel.cpp`), nor does a receive engine in a Uart
  (`neg/uart_receive_engine_slot.cpp`), nor any engine in the SPI and
  I2C hosts (`neg/spi_engine_slot_absent.cpp`,
  `neg/i2c_engine_slot_absent.cpp`);
- the claim: two Uarts may name the engine, which would make two owners
  of one channel. The engine is claimed at `init()` by its Uart's SERCOM
  number, and an `init()` that finds it held by another owner PANICS -
  the breadcrumb written with `PanicCode::assert_failed` and the context
  `dma_claim_refused_context(owner)`, 0xD0 with the owner (SERCOM n is
  owner n + 1), then the halt: at the first boot that makes the second
  user, never silently. `release()` gives the claim back, so transports
  that take the engine IN TURN are legal. `test_samc_uart` letter d
  exercises the claim as a value, and letter x the panic and its
  breadcrumb on the board;
- the build-time form, preferred and not built: the image is one
  translation unit, but "two different instantiations name this type" is
  nothing C++ can count without stateful metaprogramming, and it would
  refuse the legal in-turn shape too.

## A released requester

What a release leaves the next owner of the channel, measured by
`test_samc_uart` letter w with no wire (the Uart's transmit engine on
SERCOM1's loop through the pad, PA16, at 1 Mbaud): the engine's trigger
standing - its last block over, the channel disabled by the block's end
(25.6.2.6), DRE up behind it - then the transport released three ways,
then the one channel bound again.

- **The channel keeps no request.** DATA filled by hand behind the
  engine's last block - the first character straight to the idle
  shifter, so DRE fell and rose again with the channel disabled, the
  second left in DATA, DRE low - and a block of one started: the loop
  read 0xC1, 0xC2, then the block's 0xB1, in order. The beat waited for
  DATA to empty; nothing kept from before the enable fired it.
- **A disable withdraws the request.** Disabled and gated, as the
  release did before it reset: the instance read through its bus clock
  alone shows INTFLAG 0x00 with CTRLA's configuration standing
  (0x40000004), and a channel bound to its trigger and enabled moved
  nothing.
- **A gated clock holds a request left standing.** Gated with ENABLE
  set: INTFLAG reads 0x03 (DRE and TXC) behind the gate, and the channel
  bound to that trigger took ONE stray beat - the first byte of its
  block - and waited, enabled, for a rise that cannot come.
- **The reset leaves nothing.** `release()` resets the SERCOM before
  the gate (CTRLA.SWRST, 31.6.2.2, 31.8.1; `samc21/sercom.hpp`):
  INTFLAG and CTRLA read zero behind it, and the channel bound to its
  trigger moved nothing.
- After each of the three, the channel bound to NO trigger (TRIGSRC 0)
  moved nothing at its enable - `arm()` resets the block and the channel
  (25.6.2.2) - and the next Uart on SERCOM1 sent its first 256 bytes
  whole and in order, no fault: the outgoing owner's release is the whole
  of the hand-over, and a Uart's own `init()` resets its SERCOM before it
  arms the engine anyway.

So the SAM C21 gives the STM32F4's answer of [../design/dma.md](../design/dma.md)
at the peripheral - a disable withdraws a raised request, a gated clock
does not - and keeps nothing in the channel, where the RP families keep
a pacing request's credits. The stray reaches only the next binding of
the SAME trigger, and the reset before the gate removes it whatever
state the transport was released in.

## Not covered yet

Driver gaps:
- Every DMA use but the Uart's transmitter - declined, erratum 1.10.4
  (above): no SPI or I2C data phase, no receive engine, no converter or
  timer stream (and so no `util/block_stream.hpp` realization on this
  family), no memory-to-memory copy, no event-triggered channel.
- The CRC engine and linked descriptors - declined with the rest; the
  CRC's data-port erratum 1.10.1 is revision B only.
- The transmit engine across a standby: its channel runs with
  CHCTRLA.RUNSTDBY = 0, and 25.6.7 makes it software's to suspend such a
  channel (its SERCOM's SYNCBUSY checked) before entering standby and to
  resume it after; nothing suspends it and no sleep vote waits for a
  block in flight. Born with the first program that sleeps to standby
  with an engined Uart ([platform.md](platform.md)).
