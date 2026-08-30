# MTB - Micro Trace Buffer (SAM C21)

> **PROVISIONAL.** The four programmable registers, the packet format and
> the post-mortem path across a reset are built and measured, but there
> is no decoder above the packet pair. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M **10.3** (a
section, not a chapter - it names the four registers and then defers to
the ARM CoreSight MTB-M0+ Technical Reference Manual, which is **not**
among this project's documents of record) plus table 12-3 for the event
users; errata DS80000740S has **no MTB section**. Because the TRM is
absent, **the device header is the only local authority on the bit
layout**, and everything it does not name is measured or left alone.
Drivers: `samc/mtb.hpp` (the block) and `samc/postmortem.hpp` (the trace
carried across a reset, beside the panic breadcrumb). Family fixtures
`test/family_samc/mtb.cpp` and `test/family_samc/postmortem.cpp` plus six
negatives under `tools/check_samc.sh`; the bench suites are
`test_samc_debug` (the block) and `test_samc_postmortem` (the
post-mortem).

The MTB records every non-sequential change of the program counter as a
pair of 32-bit words in SRAM, with no CPU cycles spent and with write
priority over the processor.

**The interesting case is the one with no debugger.** Every vendor
description of the block assumes a probe reading the buffer out through
the Debug Access Port. But the buffer is ordinary SRAM chosen by
software, so a program can point the MTB at a buffer of its own, run,
stop it and **read its own trace** - a hardware backtrace of the last N
branches, available after a fault, with nothing plugged in.

## What the silicon does

**The buffer is a power of two, aligned to itself.** MASTER.MASK holds
log2(bytes) - 4, so the smallest buffer is 16 bytes (two packets) and the
write pointer wraps within a naturally aligned region of that size. A
buffer that is not aligned to its own size does not fail - it **traces
somewhere else** - which is why the driver refuses one.

**POSITION and FLOW hold offsets from BASE, not addresses.** BASE is a
read-only register giving where the SRAM the MTB writes to starts
(0x20000000 here); POSITION.POINTER and FLOW.WATERMARK are byte offsets
from it with their low three bits implied zero, a packet being eight
bytes.

**A packet is two words: a source address and a destination address**,
each with bit 0 carrying a flag. Every instruction address on this core
is halfword aligned, so bit 0 is free; the TRM names the flags and this
project does not have it, so the driver exposes the addresses with bit 0
masked off and the flags as unnamed bits. What they DO here is measured
below.

**POSITION.WRAP says the buffer has been round.** With no watermark the
pointer wraps and overwrites the oldest packets; FLOW.AUTOSTOP stops the
trace at the watermark instead, clearing MASTER.EN itself.

**Reading the tail back is a race against the reader.** The MTB traces
the processor that reads it, so every branch the reading code takes is
another packet over the oldest thing still in the buffer - and a copy
loop's backward branch is a branch. Stopping the trace is therefore the
FIRST step of any post-mortem path and not a step in it; the driver
spells that `freeze()` and the bench measures what skipping it costs.

**MASTER.HALTREQ and FLOW.AUTOHALT ask the CORE to halt**, which on
ARMv6-M needs DHCSR.C_DEBUGEN - a bit `tools/bench.py` deliberately
clears at the end of every SAM flash so that no stray halt can stop an
unattended board. The core cannot read DHCSR itself (it is
debugger-access-only, the same fact `samc/platform_sam.hpp` records about
BKPT), so the only evidence about these two bits is behavioural.

**Two event users gate the hardware trace-start and trace-stop inputs**,
enabled by MASTER.TSTARTEN and MASTER.TSTOPEN. **The two documents
disagree about their numbers**: table 12-3 says 44 and 45, the device
header's `EVENT_ID_USER_MTB_START` / `_STOP` say 45 and 46 and leave 44
unassigned. The driver publishes the header's, per the house rule, and
the bench settles it (below).

**No clock of its own.** The MTB has no bit in any MCLK mask and no row
in table 12-3's clock columns: it is clocked with the processor and there
is nothing to enable.

**It is PAC write-protectable, and its identifier is one the data sheet
never prints.** Table 12-3's "PAC Index" column is blank for this row
while PAC.STATUSB and INTFLAGB both carry an MTB bit and the header gives
`ID_MTB` = 36. `Mtb::pac_id` is that number.

## Types and verbs

`MtbPacket` - one execution-trace packet: `source_word`,
`destination_word`, with `source()` / `destination()` (bit 0 masked off)
and `source_flag()` / `destination_flag()`.

`MtbConfig` - everything MASTER and FLOW hold besides the geometry:
`start_on_event`, `stop_on_event` (TSTARTEN / TSTOPEN), `auto_stop`,
`auto_halt`, `registers_privileged`, `ram_privileged` (SFRWPRIV /
RAMPRIV, vocabulary rather than protection in a framework that runs
everything privileged), and `watermark_packets` (0 meaning "the end of
the buffer").

`Mtb` - the monostate resource.

- Constants: `pac_id`, `ev_user_start`, `ev_user_stop`, `packet_bytes`.
- Geometry, constant-evaluable: `mask_for`, `size_for`, `packets_for`,
  `geometry_valid`; plus `buffer_valid`, which adds the placement rules
  the address is needed for, and `sram_base()`.
- Configuration: `configure(buffer, bytes, cfg)` - which points the
  block at a buffer, writes MASTER and FLOW and resets the pointer, but
  does NOT enable it, so that the moment tracing starts is a line of the
  caller's own; `enable`, `enabled`, `release`.
- Readback: `master`, `flow`, `position`, `mask`, `write_offset`,
  `wrapped`, `packets_written`, `packet(buffer, index)`.
- The post-mortem pair: `freeze()` - clear MASTER.EN and nothing else,
  one load and one store, legal with interrupts dead - and
  `snapshot(buffer, bytes, span)`, the bounded allocation-free copy of
  the LAST packets the buffer holds, **oldest first**, walking back from
  the write pointer and modulo the buffer when POSITION.WRAP says it has
  been round. A span with room for fewer packets than the buffer holds
  drops the OLDEST, which is the right end to lose; an unwrapped buffer
  answers short; a geometry that is not a trace buffer's, or a POSITION
  that does not point inside the buffer given, answers zero.

`samc/postmortem.hpp` is the glue that makes those two survive a reset,
beside the kernel's panic breadcrumb rather than inside it - a trace is
silicon this stratum happens to have, and the next target may answer
differently or not at all.

`MtbPostMortem<trace_bytes, keep_packets>` - the store. It owns the
rolling buffer (ordinary `.bss`, aligned to its own size) and a
**separate `.noinit` record**: magic word, CRC-16 (`util/crc.hpp`),
count, a `source` byte and the packets. `arm()` / `disarm()`;
`capture(source)`, which **freezes first**, refuses to overwrite a
record that already stands (the rule `hard_fault_reset()` applies to the
PanicRecord, for the same reason: a fault after a diagnosis is a
consequence of it) and is legal from a fault handler; `pending()`,
`packets()`, `take()` - the read-and-invalidate that hands the packets
and the source byte over once - `clear()`, `raw()` and `checksum()`. The
geometry rules are `static_assert`s: at least one packet kept, a
power-of-two buffer, and no more kept than the buffer can hold.

`MtbTrace` - what `take()` returns: `std::span<const MtbPacket> packets`
(oldest first) and `uint8_t source`. `trace_from_fault` and
`trace_from_panic` are the two source bytes this stratum uses.

`TracingReporter<Store, source, Next>` - a panic Reporter that captures
and then chains to another (`ResetReporter` by default).
`hard_fault_trace_reset<P, Store>()` - the HardFault body an app binds,
which captures and then does what `samc/reset.hpp`'s
`hard_fault_reset<P>()` does. Composition: nothing in `reset.hpp`
changed and nothing in it knows about the trace.

## How to use it

A trace window over a known piece of code:

    alignas(1024) uint32_t trace[256];       // 1024 bytes = 128 packets

    Mtb::configure(trace, sizeof(trace));
    Mtb::enable(true);
    suspect();
    Mtb::enable(false);

    const uint32_t n = Mtb::packets_written(trace, sizeof(trace));
    for (uint32_t i = 0; i < n; ++i) {
        const auto p = Mtb::packet(trace, i);
        print(sink, hex(p.source()), " -> ", hex(p.destination()), crlf);
    }

A rolling buffer that keeps the LAST branches before something goes
wrong - leave it enabled and let it wrap; `wrapped()` then says the
buffer is full and `write_offset()` points at the oldest packet.

Stop at a watermark instead of wrapping:

    MtbConfig cfg{};
    cfg.auto_stop = true;
    cfg.watermark_packets = 8;
    Mtb::configure(trace, sizeof(trace), cfg);
    Mtb::enable(true);                       // the hardware clears EN at packet 8

Start the trace from an event rather than from a store:

    MtbConfig cfg{};
    cfg.start_on_event = true;
    Mtb::configure(trace, sizeof(trace), cfg);
    Evsys::connect(Mtb::ev_user_start, channel, channel_cfg);

A post-mortem: where the program died, read at the next boot.

    using Trace = MtbPostMortem<256, 16>;      // 32 packets rolling, 16 kept

    extern "C" void HardFault_Handler() {
        hard_fault_trace_reset<SamPlatform, Trace>();
    }

    int main() {
        if (const auto record = take_panic_record<SamPlatform>()) {
            print(sink, "died: code ", record->code, crlf);
            if (const auto trace = Trace::take()) {
                for (const MtbPacket& p : trace->packets) {   // oldest first
                    print(sink, hex(p.source()), " -> ",
                          hex(p.destination()), crlf);
                }
            }
        }
        Trace::arm();
        ...
    }

and for a panic that is not a fault, `panic<SamPlatform,
TracingReporter<Trace>>(code, context)` - though on a board whose
DHCSR.C_DEBUGEN is clear the fault body is what runs even then (below).

## Bench findings

`test_samc_debug` letters i, j and k on the ATSAMC21J18A rev F, wireless
and with no probe attached. Letter k sits outside `z` because it asks the
core to halt and a wrong answer costs a reflash.

**A Cortex-M0+ reads its own hardware backtrace.** A window over a chain
of three noinline functions produced **12 packets** into a 1024-byte
buffer, and the decoded destinations contain all three functions'
addresses as the linker placed them. BASE reads 0x20000000; MASTER.MASK
reads back log2(bytes) - 4; POSITION holds the buffer's offset from BASE
and not its address.

**Bit 0 of the DESTINATION word marks the start of trace.** Of the 12
packets exactly one has it set, and it is the first. Bit 0 of the SOURCE
word is set on none of them, so it means something this window never
produced.

**With MASTER.EN clear the pointer does not move** - two more calls to
the same chain leave it exactly where it was, which is what makes a
window a window.

**A full buffer wraps.** Two hundred chains set POSITION.WRAP and leave
the pointer inside the buffer. With FLOW.AUTOSTOP and a watermark of
eight packets the same two hundred chains leave **MASTER.EN cleared by
the hardware**, exactly eight packets written, and WRAP clear.

**The device header's event-user numbers are right and table 12-3's are
not.** Each of users 44, 45 and 46 was wired to a channel in turn, on an
asynchronous path and on a resynchronized one with a rising edge, and a
software event fired at it: **only user 45 starts the trace** (on both
paths), and user **46 stops a running one**. So START is 45 and STOP is
46, as `EVENT_ID_USER_MTB_START` / `_STOP` say. That a SOFTWARE event on
an ASYNCHRONOUS channel reaches this user at all is consistent with what
the CCL campaign established and `samc/evsys.md` now records.

**Neither halt bit stops a core with no debugger.** FLOW.AUTOHALT
reaching its watermark, and MASTER.HALTREQ set directly, both leave the
CPU running - which is the expected reading of ARMv6-M's rule that a halt
request needs DHCSR.C_DEBUGEN, and is worth having measured on a board
`tools/bench.py` deliberately leaves with that bit clear.

## Bench findings - the post-mortem

`test_samc_postmortem` on the ATSAMC21J18A rev F, wireless and with no
probe attached, with a 256-byte rolling buffer (32 packets) keeping 16.
Letters `a`, `b` and `c` are `z` (36 verdicts); `f` and `p` reboot the
board and sit outside it.

**A Cortex-M0+ hands the next boot its own last branches.** A UDF
executed three calls deep leaves, after the reset, a validated record of
**6..7 packets** whose destinations are the three calls in the order
they were made, followed by the exception entry. The PanicRecord and the
trace are read side by side: what died and where from.

**The fault site is the faulting instruction.** The exception entry's
SOURCE word is **+8 bytes into the dying leaf** - the UDF itself - and
its destination is the address the linker gave `HardFault_Handler`.

**Bit 0 of the SOURCE word marks the exception entry**, which settles
what letter i of `test_samc_debug` could only report as absent ("set on
none of them, so it means something this window never produced"). In
every fault trace exactly one packet carries it, and it is the entry
into the fault handler. Bit 0 of the DESTINATION word still marks the
start of trace, on the first packet of a window.

**The capture costs two packets.** Between the exception entry and
`freeze()` the trace grows by two packets - the handler's own branch
into the capture - which is the whole price of the mechanism in trace
terms, and the reason 16 kept packets are comfortable.

**Freezing first is worth all of that and more.** Read with the trace
STOPPED, a window over a three-deep chain gives 9 packets and **all
three** of the chain's leaves, with no two consecutive packets alike.
Read with the trace still RUNNING, the same window comes back **16
packets, 2 leaves, and not one packet in common with the frozen read** -
and its tail is a **run of 4 identical packets**, which is the copy
loop's own backward branch written again and again. The reader wins the
race against the program it came to read.

**What a chain costs.** One three-deep chain is **9 packets** in this
suite (a single leaf call is 3); `test_samc_debug`'s own chain was 12
into a 1024-byte buffer. So 16 kept packets hold a whole chain plus the
fault path with slack, and the record is 136 bytes of `.noinit`.

**The walk is oldest-first, and that is measured and not assumed.**
After 64 chains the buffer has wrapped and the snapshot returns exactly
the 16 kept; every leaf-to-leaf step in it advances a -> b -> c and none
goes backwards. An unwrapped buffer answers short (3 of 16 asked for
after one leaf call) and the oldest packet it returns carries the
start-of-trace flag.

**A standing record is not overwritten.** A second capture over a valid
record is refused and changes nothing - packets and source byte both -
which is the same rule `hard_fault_reset()` applies to the PanicRecord
and for the same reason.

**On this board the "orderly panic" path is the fault path.** A
`panic()` through `TracingReporter` reaches the next boot with the
PanicRecord carrying **the code panic() was given** (`assert_failed`,
context intact) and a trace of 11 packets containing the chain that led
into it - but the `source` byte says **the HardFault body**, and the
trace shows why: one packet with the source flag set, landing on
`HardFault_Handler`, right after the chain. `panic()` calls
`P::break_here()` BEFORE any reporter, that is a BKPT, and with
DHCSR.C_DEBUGEN clear - which `tools/bench.py` leaves after every SAM
flash - a BKPT escalates. Nothing is lost, because the PanicRecord is
written before `break_here()`; but an app that wants a trace **must bind
the fault body**, and the reporter alone is not enough. The reporter's
own capture is proven separately, chained to a reporter that does not
end the program.

## Not covered yet

Driver gaps - features of 10.3 not built:

- **Any decoder above the packet pair.** Reconstructing a call chain
  wants the image's symbols and belongs on the host; what this driver
  hands back is source/destination addresses.
- **The CoreSight management registers** at offsets 0xF00 and above -
  claim tags, lock access, the authentication status, DEVARCH/DEVID/
  DEVTYPE and the PID/CID block. They are a probe's business; the DSU's
  own ROM table (see `samc/dsu.md`) is what advertises this component to
  one.

Implemented but not bench-verified:

- **SFRWPRIV and RAMPRIV.** Written and read back; this framework runs
  everything privileged, so neither has ever refused anything.
- **A HARDWARE generator on the trace-start user.** The event path was
  proven with software events; nothing has yet started a trace from, say,
  a comparator or a timer - which is the shape that would make the
  feature worth its wiring.
- **A trace through a POWER LOSS.** The record crosses a system reset
  and a watchdog reset in `.noinit`; carrying it into flash would be
  `util/nv_journal.hpp`'s reserve, and it does not fit: at 16 packets the
  payload is 136 bytes against the RWWEE attic's arithmetic ((`max_ids` +
  2) x entry <= half, with `entry` rounded up to the 64-byte write cell),
  which leaves room for no ids at all - and the attic is already
  partitioned for `test_samc_journal`'s own geometry. A four-packet
  truncation would fit; nothing has decided that four packets are worth
  a flash write on the way out of a crash.
- **A trace across a sleep.** The buffer is ordinary SRAM and the MTB is
  clocked with the processor; what a standby does to a trace in progress
  is untested.
- **A decoded post-mortem.** What the record hands back is addresses;
  turning them into function names is the host's job (`arm-none-eabi-
  addr2line` against the `.elf` the board is running) and no tool here
  does it yet.
