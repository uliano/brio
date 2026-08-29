# MTB - Micro Trace Buffer (SAM C21)

> **PROVISIONAL.** The four programmable registers and the packet format
> are built and measured, but there is no decoder above the packet pair
> and no integration with `panic()`. The list is in "Not covered yet".

Documents of record: SAM C20/C21 data sheet DS60001479M **10.3** (a
section, not a chapter - it names the four registers and then defers to
the ARM CoreSight MTB-M0+ Technical Reference Manual, which is **not**
among this project's documents of record) plus table 12-3 for the event
users; errata DS80000740S has **no MTB section**. Because the TRM is
absent, **the device header is the only local authority on the bit
layout**, and everything it does not name is measured or left alone.
Driver: `samc/mtb.hpp`. Family fixture `test/family_samc/mtb.cpp` plus
three negatives under `tools/check_samc.sh`; the bench suite is
`test_samc_debug`.

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

## Not covered yet

Driver gaps - features of 10.3 not built:

- **Any decoder above the packet pair.** Reconstructing a call chain
  wants the image's symbols and belongs on the host; what this driver
  hands back is source/destination addresses.
- **Integration with `panic()`.** A fault handler that dumps the last
  branches is exactly what a self-hosted trace enables, and it will be
  built with its first user - together with the question of where the
  buffer lives (`.noinit`, so it survives the reset the reporter causes).
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
- **Trace across a sleep or a reset.** The buffer is ordinary SRAM and
  would survive a system reset in `.noinit`, and nothing has tried.
