// test_samc_postmortem - the bench suite for samc/postmortem.hpp: the
// Micro Trace Buffer carried across a reset, so the next boot reads not
// only WHAT died (kernel/panic.hpp's PanicRecord, code and context) but
// WHERE FROM - the last branches before the disaster.
//
// It is a suite of its own rather than three more letters of
// test_samc_debug because two of its five letters REBOOT THE BOARD and
// because the mechanism under test is the composition of four things
// that already have suites of their own (the MTB, the panic record, the
// fault body and the reset controller). test_samc_debug's letters i, j
// and k stay what they are and its z stays 117.
//
// NOTHING TO WIRE. The MTB has no pad, the record is SRAM and the fault
// is an instruction.
//
// WHAT MAKES THE VERDICTS CHECKABLE:
//
//   - THE LINKER is the reference, as in test_samc_debug letter i: the
//     chains are noinline functions whose addresses the program can ask
//     for, and a recovered trace must contain them, oldest first, in the
//     order they were called.
//   - THE FAULT IS REAL. Letter f ends in UDF - the one instruction on
//     this core that faults whatever the compiler thinks (an unaligned
//     volatile load does not: gcc emits byte loads).
//   - THE FREEZE IS MEASURED, NOT ASSUMED. Letter b reads the same
//     window twice, once with the trace stopped and once with it
//     running, and counts what survives.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>
#include <span>

#include "kernel/panic.hpp"
#include "samc/clock.hpp"
#include "samc/mtb.hpp"
#include "samc/nvic.hpp"
#include "samc/pin.hpp"
#include "samc/platform_sam.hpp"
#include "samc/postmortem.hpp"
#include "samc/reset.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token the two rebooting letters live in
//
// In .noinit and inline, for the reasons test_samc_platform and
// test_samc_journal both give: the section must survive the crt, and gcc
// gives an inline variable with a section attribute a COMDAT group where
// a plain one gets none. Table 18-1 lists no SRAM row for any reset
// source, so every read is guarded by the magic word.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0x9C21;

struct Token {
    uint16_t magic;
    uint8_t leg;       ///< 0 = nothing pending, 1 = letter f, 2 = letter p
    uint8_t path;      ///< 0 = nothing, 1 = the reporter, 2 = the fault body
    uint8_t code;      ///< what panic() was given (letter p)
    uint8_t context;
    uint16_t pass;
    uint16_t fail;
};
[[gnu::section(".noinit")]] inline Token token;

extern "C" void HardFault_Handler();

namespace {

using namespace brio;
using brio::crlf;
using brio::print;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;

using Led = Pin<'B', 23>;

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// THE STORE, and the two numbers that are this campaign's only real
// design choice.
//
// 256 bytes of rolling buffer = 32 packets. The buffer's size does not
// change WHICH packets are kept - the tail is always the last
// keep_packets - so it is chosen for headroom and not for reach.
//
// 16 packets kept, and the justification is measured rather than
// guessed: letter c prints what one three-deep chain costs (the debug
// campaign's own chain produced 12 packets into a 1024-byte buffer) and
// letter f prints how much of the recovered trace the fault path itself
// takes up. 16 packets is 128 bytes of .noinit plus an 8-byte header.
// ---------------------------------------------------------------------------
using Trace = MtbPostMortem<256, 16>;

/// The panic path an app spells: capture, then reset.
using PanicTrace = TracingReporter<Trace>;
/// The same capture in front of a reporter that does NOT end the
/// program - which is how the composition is provable without a reboot.
using QuietTrace = TracingReporter<Trace, trace_from_panic, HaltReporter>;

// ---------------------------------------------------------------------------
// The known chains. gcc can neither fold these together (each body is
// different) nor tail-call them away (each ends with a store of its
// own), so the linker gives each a distinct address the program can ask
// for and the trace must contain.
// ---------------------------------------------------------------------------
volatile uint32_t sink = 0;

[[gnu::noinline]] void leaf_a() { sink = sink + 11u; }
[[gnu::noinline]] void leaf_b() { sink = sink * 3u + 1u; }
[[gnu::noinline]] void leaf_c() { sink = sink ^ 0x5A5Au; }

[[gnu::noinline]] void chain() {
    leaf_a();
    leaf_b();
    leaf_c();
    sink = sink + 1u;   // stops the last call being a tail call
}

/// A DIFFERENT window, so a record that had been replaced would say so.
[[gnu::noinline]] void other_chain() {
    leaf_c();
    sink = sink + 9u;
}

// The chain that dies. UDF is the only instruction on this core that is
// beyond the compiler's reach: an unaligned volatile load does NOT
// fault, because gcc emits four byte loads with shifts (the lesson
// test_samc_platform paid for).
[[gnu::noinline]] void die_here() {
    sink = sink + 7u;
    __asm volatile("udf #0" ::: "memory");
}
[[gnu::noinline]] void dying_middle() {
    sink = sink * 5u;
    die_here();
    sink = sink + 2u;
}
[[gnu::noinline]] void dying_chain() {
    sink = sink ^ 0x1234u;
    dying_middle();
    sink = sink + 3u;
}

uint32_t address_of(void (*fn)()) {
    return reinterpret_cast<uint32_t>(fn) & ~1UL;
}

uint32_t handler_address() {
    return reinterpret_cast<uint32_t>(&HardFault_Handler) & ~1UL;
}

/// Where in a packet list a destination first appears, or -1.
int32_t index_of_destination(std::span<const MtbPacket> packets, uint32_t addr) {
    for (uint32_t i = 0; i < packets.size(); ++i) {
        if (packets[i].destination() == addr) {
            return static_cast<int32_t>(i);
        }
    }
    return -1;
}

bool contains_destination(std::span<const MtbPacket> packets, uint32_t addr) {
    return index_of_destination(packets, addr) >= 0;
}

/// How many of the three leaves the trace holds.
uint32_t leaves_present(std::span<const MtbPacket> packets) {
    uint32_t n = 0;
    if (contains_destination(packets, address_of(leaf_a))) ++n;
    if (contains_destination(packets, address_of(leaf_b))) ++n;
    if (contains_destination(packets, address_of(leaf_c))) ++n;
    return n;
}

void print_packets(std::span<const MtbPacket> packets) {
    for (uint32_t i = 0; i < packets.size(); ++i) {
        print(serial, "    ", i, ": ", hex(packets[i].source()), " -> ",
              hex(packets[i].destination()), "   flags ",
              packets[i].source_flag() ? 1u : 0u, "/",
              packets[i].destination_flag() ? 1u : 0u, crlf);
    }
}

void print_chain_addresses() {
    print(serial, "  the linker put the chain at ", hex(address_of(leaf_a)),
          " ", hex(address_of(leaf_b)), " ", hex(address_of(leaf_c)),
          ", the dying one at ", hex(address_of(dying_chain)), " ",
          hex(address_of(dying_middle)), " ", hex(address_of(die_here)),
          ", HardFault_Handler at ", hex(handler_address()), crlf);
}

// The cycle stopwatch and the console drain, the test_samc_debug forms.
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < SysClock::hz / 500u) {
    }
}

const char* source_name(uint8_t s) {
    switch (s) {
        case trace_from_fault: return "the HardFault body";
        case trace_from_panic: return "a panic Reporter";
        default: return "something else";
    }
}

// =============================================================================
// Letter a - the record: capture, checksum, take, and the rule that a
// standing diagnosis is not overwritten
// =============================================================================
void ta_record() {
    Trace::clear();

    print(serial, "  the store: ", Trace::buffer_bytes, " bytes of buffer = ",
          Mtb::packets_for(Trace::buffer_bytes), " packets, keeping ",
          Trace::kept, "; the record is ",
          static_cast<uint32_t>(sizeof(Trace::Record)), " bytes of .noinit at ",
          hex(reinterpret_cast<uint32_t>(&Trace::raw())), crlf);

    bench.verdict("the buffer is a legal trace buffer - a power of two, "
                  "aligned to its own size",
                  Mtb::buffer_valid(Trace::buffer(), Trace::buffer_bytes));
    bench.verdict("and it is not the record: the rolling buffer is ordinary "
                  ".bss, what survives is the checked copy",
                  Trace::buffer() != static_cast<const void*>(&Trace::raw()));
    bench.verdict("nothing is pending after a clear()", !Trace::pending());

    // A window, then the capture. Nothing is printed between the two: a
    // console line is milliseconds of branches.
    bench.verdict("arm() points the MTB at the store's buffer and starts it",
                  Trace::arm() && Mtb::enabled());
    chain();
    const bool captured = Trace::capture(trace_from_fault);
    const bool stopped = !Mtb::enabled();

    print(serial, "  the capture kept ",
          static_cast<uint32_t>(Trace::raw().count), " packets, source byte ",
          static_cast<uint32_t>(Trace::raw().source), crlf);
    print_chain_addresses();
    print_packets(Trace::packets());

    bench.verdict("capture() takes the trace", captured);
    bench.verdict("and it FROZE the trace first - the MTB is stopped when it "
                  "returns",
                  stopped);
    bench.verdict("the record validates: magic, count and CRC-16",
                  Trace::pending());
    bench.verdict("it kept packets", Trace::raw().count > 0u);
    bench.verdict("and the source byte is the one the caller gave",
                  Trace::raw().source == trace_from_fault);
    bench.verdict("the chain the window covered is all three leaves deep",
                  leaves_present(Trace::packets()) == 3u);

    // THE CHECKSUM, on a copy - the record itself is never corrupted by
    // this suite, since a torn one is exactly what must not be believed.
    Trace::Record local = Trace::raw();
    const uint16_t good = Trace::checksum(local);
    local.packets[0].destination_word ^= 0x40UL;
    const uint16_t bad = Trace::checksum(local);
    local.packets[0].destination_word ^= 0x40UL;
    local.count = static_cast<uint8_t>(Trace::kept + 1u);
    const uint16_t impossible = Trace::checksum(local);
    bench.verdict("a single flipped bit in a packet changes the checksum",
                  bad != good);
    bench.verdict("and a count the record could not hold is refused outright",
                  impossible == 0u);

    // THE RULE: a standing record is not overwritten. The second window
    // is a DIFFERENT chain, so a record that had been replaced would say
    // so.
    const MtbPacket first_kept =
        Trace::raw().count > 0u ? Trace::raw().packets[0] : MtbPacket{0, 0};
    (void)Trace::arm();
    other_chain();
    const bool refused = !Trace::capture(trace_from_panic);
    bench.verdict("a second capture over a standing record is REFUSED - a "
                  "fault after a diagnosis is a consequence of it",
                  refused);
    bench.verdict("and it changed nothing: the first trace is still there",
                  Trace::pending() && Trace::raw().count > 0u &&
                      Trace::raw().packets[0].destination_word ==
                          first_kept.destination_word &&
                      Trace::raw().source == trace_from_fault);

    // TAKE: once.
    const auto got = Trace::take();
    bench.verdict("take() hands the trace over", got.has_value());
    bench.verdict("with the source byte beside it",
                  got && got->source == trace_from_fault);
    bench.verdict("and the packets still readable after the invalidation - "
                  "the magic word is cleared, the data is not",
                  got && got->packets.size() > 0u &&
                      leaves_present(got->packets) == 3u);
    bench.verdict("a second take() returns nothing", !Trace::take().has_value());
    bench.verdict("and pending() agrees", !Trace::pending());

    // THE REPORTER, composed with one that does not end the program.
    (void)Trace::arm();
    chain();
    QuietTrace::report(PanicCode::assert_failed, 0x33);
    bench.verdict("a panic Reporter that CAPTURES and chains to another "
                  "reporter leaves a record",
                  Trace::pending());
    bench.verdict("marked as the panic path's", Trace::raw().source == trace_from_panic);
    bench.verdict("with the same chain in it", leaves_present(Trace::packets()) == 3u);

    Trace::clear();
    Trace::disarm();
    bench.verdict("disarm() hands the block back", Mtb::master() == 0u);
}

// =============================================================================
// Letter b - why freeze() is the FIRST line: the reader overwrites what
// it came to read
// =============================================================================
void tb_freeze_first() {
    MtbPacket frozen[Trace::kept]{};
    MtbPacket running[Trace::kept]{};

    // Window one: stop the trace, then read it.
    (void)Trace::arm();
    chain();
    (void)Mtb::freeze();
    const uint32_t n_frozen = Mtb::snapshot(Trace::buffer(), Trace::buffer_bytes,
                                            std::span<MtbPacket>(frozen));

    // Window two: the same chain, read with the trace STILL RUNNING.
    (void)Trace::arm();
    chain();
    const uint32_t n_running = Mtb::snapshot(Trace::buffer(), Trace::buffer_bytes,
                                             std::span<MtbPacket>(running));
    (void)Mtb::freeze();

    const uint32_t frozen_leaves =
        leaves_present(std::span<const MtbPacket>(frozen, n_frozen));
    const uint32_t running_leaves =
        leaves_present(std::span<const MtbPacket>(running, n_running));

    uint32_t same = 0;
    for (uint32_t i = 0; i < n_frozen && i < n_running; ++i) {
        if (frozen[i].destination_word == running[i].destination_word &&
            frozen[i].source_word == running[i].source_word) {
            ++same;
        }
    }

    // The reader's own copy loop is a BACKWARD BRANCH, so it writes the
    // same packet over and over: the longest run of identical
    // consecutive packets is the signature of a trace reading itself.
    const auto longest_repeat = [](const MtbPacket* p, uint32_t n) {
        uint32_t best = n > 0u ? 1u : 0u, run = best;
        for (uint32_t i = 1; i < n; ++i) {
            run = (p[i].source_word == p[i - 1].source_word &&
                   p[i].destination_word == p[i - 1].destination_word)
                      ? run + 1u
                      : 1u;
            if (run > best) {
                best = run;
            }
        }
        return best;
    };
    const uint32_t frozen_repeat = longest_repeat(frozen, n_frozen);
    const uint32_t running_repeat = longest_repeat(running, n_running);

    print(serial, "  frozen: ", n_frozen, " packets, ", frozen_leaves,
          " of the chain's 3 leaves, longest identical run ", frozen_repeat,
          "; running: ", n_running, " packets, ", running_leaves,
          " leaves, longest identical run ", running_repeat, "; ", same,
          " packets identical", crlf);
    print(serial, "  what the RUNNING read came back with:", crlf);
    print_packets(std::span<const MtbPacket>(running, n_running));

    bench.verdict("a FROZEN buffer gives the program's own history: all "
                  "three leaves of the chain",
                  frozen_leaves == 3u);
    bench.verdict("reading a RUNNING buffer loses it - the reader's own "
                  "branches are written over the evidence",
                  running_leaves < frozen_leaves);
    bench.verdict("so the two reads disagree about most of the tail",
                  same < n_frozen);
    bench.verdict("and what fills the running one is the READER'S OWN COPY "
                  "LOOP, its backward branch written again and again",
                  running_repeat > frozen_repeat && running_repeat > 2u);
    bench.verdict("- which is why freeze() is capture()'s first line and not "
                  "a step in it",
                  frozen_leaves == 3u && running_leaves < 3u);

    Trace::disarm();
}

// =============================================================================
// Letter c - the walk: oldest first, wrapped and unwrapped, and what a
// chain costs in packets
// =============================================================================
void tc_walk() {
    MtbPacket kept[Trace::kept]{};

    // THE YOUNG BUFFER: fewer packets than the caller asked for, and the
    // start-of-trace flag on the first one (test_samc_debug letter i
    // established that bit 0 of the DESTINATION word marks it).
    (void)Trace::arm();
    leaf_a();
    (void)Mtb::freeze();
    const uint32_t written =
        Mtb::packets_written(Trace::buffer(), Trace::buffer_bytes);
    const uint32_t young = Mtb::snapshot(Trace::buffer(), Trace::buffer_bytes,
                                         std::span<MtbPacket>(kept));
    const bool young_marked = young > 0u && kept[0].destination_flag();
    print(serial, "  a young buffer: ", written, " packets written, ", young,
          " returned into room for ", Trace::kept, crlf);
    bench.verdict("an unwrapped buffer answers short rather than inventing "
                  "packets",
                  young == written && young < Trace::kept);
    bench.verdict("and the oldest packet it returns is the START OF TRACE",
                  young_marked);

    // WHAT ONE CHAIN COSTS - the number keep_packets is chosen against.
    (void)Trace::arm();
    chain();
    (void)Mtb::freeze();
    const uint32_t chain_packets =
        Mtb::packets_written(Trace::buffer(), Trace::buffer_bytes);
    print(serial, "  one three-deep chain costs ", chain_packets,
          " packets, so ", Trace::kept, " kept covers ", Trace::kept / chain_packets,
          " of them plus ", Trace::kept % chain_packets, " packets of slack",
          crlf);
    bench.verdict("a whole three-deep chain fits inside what the record "
                  "keeps",
                  chain_packets > 0u && chain_packets <= Trace::kept);

    // THE WRAPPED BUFFER: the tail is the newest, and it is in ORDER.
    // The chain calls a, b, c in that order for ever, so the leaves seen
    // in the snapshot must advance cyclically - which is what proves the
    // walk is oldest-first and not merely "some 16 packets".
    (void)Trace::arm();
    for (uint32_t i = 0; i < 64u; ++i) {
        chain();
    }
    (void)Mtb::freeze();
    const bool wrapped = Mtb::wrapped();
    const uint32_t full = Mtb::snapshot(Trace::buffer(), Trace::buffer_bytes,
                                        std::span<MtbPacket>(kept));

    const uint32_t a = address_of(leaf_a), b = address_of(leaf_b),
                   c = address_of(leaf_c);
    int32_t previous = -1;
    uint32_t steps = 0, forward = 0;
    for (uint32_t i = 0; i < full; ++i) {
        int32_t which = -1;
        if (kept[i].destination() == a) which = 0;
        else if (kept[i].destination() == b) which = 1;
        else if (kept[i].destination() == c) which = 2;
        if (which < 0) {
            continue;
        }
        if (previous >= 0) {
            ++steps;
            if (which == (previous + 1) % 3) {
                ++forward;
            }
        }
        previous = which;
    }
    print(serial, "  after 64 chains: WRAP ", wrapped, ", ", full,
          " packets returned, ", steps, " leaf-to-leaf steps of which ",
          forward, " advance a -> b -> c", crlf);
    print_packets(std::span<const MtbPacket>(kept, full));

    bench.verdict("64 chains wrap a 32-packet buffer", wrapped);
    bench.verdict("and the snapshot is exactly as long as the record keeps",
                  full == Trace::kept);
    bench.verdict("the packets come back OLDEST FIRST: every leaf-to-leaf "
                  "step advances a -> b -> c, none goes backwards",
                  steps > 0u && forward == steps);

    // A span with no room, and a geometry that is not a trace buffer's.
    bench.verdict("a snapshot with no room copies nothing",
                  Mtb::snapshot(Trace::buffer(), Trace::buffer_bytes,
                                std::span<MtbPacket>(kept, 0)) == 0u);
    bench.verdict("and a size that could not be a trace buffer is refused",
                  Mtb::snapshot(Trace::buffer(), 200,
                                std::span<MtbPacket>(kept)) == 0u);

    Trace::disarm();
}

// =============================================================================
// Letter f - a REAL HardFault, across a real reset (outside z)
// =============================================================================
void bank(uint8_t leg) {
    token.magic = token_magic;
    token.leg = leg;
    token.path = 0;
    token.pass = bench.passed();
    token.fail = bench.failed();
}

void tf_fault() {
    Trace::clear();
    SamPlatform::panic_record().magic = 0;   // nothing already diagnosed

    bench.verdict("no trace and no panic record are standing", !Trace::pending());
    print_chain_addresses();
    print(serial, "  arming the MTB and executing UDF three calls deep ...",
          crlf);

    bank(1);
    console_drain();

    (void)Trace::arm();
    dying_chain();

    // Unreachable: the fault body resets the board.
    bench.verdict("UNREACHABLE: the UDF did not fault", false);
}

void tf_resume() {
    bench.resume_tally(token.pass, token.fail);
    print(serial, crlf, "f (continued after the fault reset)", crlf);

    const ResetCause cause = Reset::cause();
    const auto record = take_panic_record<SamPlatform>();
    const auto trace = Trace::take();

    print(serial, "  reset cause = ", static_cast<uint32_t>(cause),
          ", the trace was captured by ",
          source_name(trace ? trace->source : 0u), ", ",
          static_cast<uint32_t>(trace ? trace->packets.size() : 0u),
          " packets", crlf);
    print_chain_addresses();
    if (trace) {
        print_packets(trace->packets);
    }

    bench.verdict("the board came back from the fault body's system reset",
                  cause == ResetCause::system_request);
    bench.verdict("the PanicRecord says a kernel fault",
                  record && record->code ==
                                static_cast<uint8_t>(PanicCode::kernel_fault));
    bench.verdict("AND A TRACE IS WAITING BESIDE IT", trace.has_value());
    bench.verdict("captured by the HardFault body",
                  trace && trace->source == trace_from_fault);
    bench.verdict("with packets in it",
                  trace && trace->packets.size() > 0u);

    const std::span<const MtbPacket> p =
        trace ? trace->packets : std::span<const MtbPacket>{};
    const int32_t i_outer = index_of_destination(p, address_of(dying_chain));
    const int32_t i_middle = index_of_destination(p, address_of(dying_middle));
    const int32_t i_leaf = index_of_destination(p, address_of(die_here));
    const int32_t i_handler = index_of_destination(p, handler_address());
    print(serial, "  chain indices: outer ", i_outer, ", middle ", i_middle,
          ", leaf ", i_leaf, ", handler ", i_handler, " of ",
          static_cast<uint32_t>(p.size()), crlf);

    bench.verdict("the whole dying chain is in it - the call that died, "
                  "three deep",
                  i_outer >= 0 && i_middle >= 0 && i_leaf >= 0);
    bench.verdict("OLDEST FIRST: the calls appear in the order they were "
                  "made",
                  i_outer >= 0 && i_middle > i_outer && i_leaf > i_middle);
    bench.verdict("and the exception entry is in there after them, so the "
                  "trace ends AT the fault",
                  i_handler > i_leaf);
    bench.verdict("- so the next boot reads not only what died but where "
                  "from",
                  record && trace && i_leaf > i_middle && i_handler > i_leaf);

    // THE EXCEPTION-ENTRY PACKET, and the flag test_samc_debug letter i
    // saw on nothing. Its SOURCE is the faulting instruction itself -
    // the UDF inside the leaf - and bit 0 of the source word is set on
    // it and on no other packet of the trace.
    uint32_t source_flags = 0;
    for (const MtbPacket& q : p) {
        if (q.source_flag()) {
            ++source_flags;
        }
    }
    const bool entry_from_leaf =
        i_handler >= 0 &&
        p[static_cast<uint32_t>(i_handler)].source() - address_of(die_here) <
            64UL;
    const bool entry_flagged =
        i_handler >= 0 && p[static_cast<uint32_t>(i_handler)].source_flag();
    const uint32_t after_entry =
        i_handler >= 0
            ? static_cast<uint32_t>(p.size()) - 1u -
                  static_cast<uint32_t>(i_handler)
            : 0u;
    print(serial, "  the exception entry's source is +",
          i_handler >= 0 ? p[static_cast<uint32_t>(i_handler)].source() -
                               address_of(die_here)
                         : 0UL,
          " into the dying leaf; ", source_flags,
          " packets carry the SOURCE flag; the capture itself costs ",
          after_entry, " packets after it", crlf);
    bench.verdict("the fault site is the faulting instruction: the entry "
                  "packet's SOURCE is the UDF inside the leaf",
                  entry_from_leaf);
    bench.verdict("and bit 0 of the SOURCE word - the flag test_samc_debug's "
                  "own window never produced - marks that exception entry, "
                  "and only it",
                  entry_flagged && source_flags == 1u);

    bench.verdict("nothing is left pending for the next boot",
                  !Trace::pending() && !take_panic_record<SamPlatform>());

    token.magic = 0;
    token.leg = 0;
    bench.end_letter();
}

// =============================================================================
// Letter p - an orderly panic(), across a real reset (outside z)
// =============================================================================
constexpr uint8_t panic_context = 0x5A;

/// The reporter letter p hands panic(): mark the path, then capture and
/// reset. Whether it ever runs is the letter's finding.
struct MarkedPanicTrace {
    static void report(PanicCode code, uint8_t context) {
        token.path = 1;
        PanicTrace::report(code, context);
    }
};

void tp_panic() {
    Trace::clear();
    SamPlatform::panic_record().magic = 0;

    bench.verdict("no trace and no panic record are standing", !Trace::pending());
    print_chain_addresses();
    print(serial, "  panic() through TracingReporter, three calls deep ...",
          crlf);

    bank(2);
    token.code = static_cast<uint8_t>(PanicCode::assert_failed);
    token.context = panic_context;
    console_drain();

    (void)Trace::arm();
    chain();
    panic<SamPlatform, MarkedPanicTrace>(PanicCode::assert_failed,
                                         panic_context);
}

void tp_resume() {
    bench.resume_tally(token.pass, token.fail);
    print(serial, crlf, "p (continued after the panic reset)", crlf);

    const ResetCause cause = Reset::cause();
    const auto record = take_panic_record<SamPlatform>();
    const auto trace = Trace::take();

    print(serial, "  reset cause = ", static_cast<uint32_t>(cause),
          ", the breadcrumb path was ",
          token.path == 2u ? "the HardFault body (the BKPT escalated)"
                           : "the reporter itself",
          ", the trace was captured by ",
          source_name(trace ? trace->source : 0u), crlf);
    print_chain_addresses();
    if (trace) {
        print_packets(trace->packets);
    }

    bench.verdict("the board came back from a system reset request",
                  cause == ResetCause::system_request);
    bench.verdict("the PanicRecord carries the code panic() was given - NOT "
                  "the fault's",
                  record && record->code == token.code);
    bench.verdict("and its context byte came through untouched",
                  record && record->context == token.context);
    bench.verdict("A TRACE IS WAITING BESIDE IT", trace.has_value());
    bench.verdict("with the chain that led into panic() in it",
                  trace && leaves_present(trace->packets) == 3u);

    // THE ESCALATION IS IN THE TRACE. Exactly one packet carries the
    // source flag letter f identified as the exception entry, and it
    // lands on the HardFault body - so what the trace shows is the BKPT
    // itself, taken from inside panic().
    const std::span<const MtbPacket> p =
        trace ? trace->packets : std::span<const MtbPacket>{};
    const int32_t i_handler = index_of_destination(p, handler_address());
    uint32_t source_flags = 0;
    for (const MtbPacket& q : p) {
        if (q.source_flag()) {
            ++source_flags;
        }
    }
    bench.verdict("and the escalation itself at the end of it: one exception "
                  "entry, flagged in its source word, landing on the "
                  "HardFault body",
                  i_handler > 0 && source_flags == 1u &&
                      p[static_cast<uint32_t>(i_handler)].source_flag());
    bench.verdict("after the chain, not before it - the last thing the "
                  "program did was call panic()",
                  i_handler > index_of_destination(p, address_of(leaf_c)));

    // THE FINDING, and it is a property of the debug state and not of
    // the program: with DHCSR.C_DEBUGEN cleared - which is what
    // tools/bench.py leaves after every SAM flash - panic()'s own
    // break_here() escalates to HardFault BEFORE the reporter runs, so
    // the body that captures is the fault's. Both are bound, and the
    // record says which one it was.
    bench.verdict("the two entry paths agree on the record: whichever body "
                  "ran, the trace and the panic code are both there",
                  record && trace && trace->packets.size() > 0u);
    bench.verdict("and the path is the one this board's debug state "
                  "dictates - the HardFault body, because break_here()'s "
                  "BKPT escalates before any reporter",
                  token.path == 2u && trace &&
                      trace->source == trace_from_fault);

    token.magic = 0;
    token.leg = 0;
    bench.end_letter();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf,
          "test_samc_postmortem - samc/postmortem.hpp: the MTB across a "
          "reset, clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

// ---- target glue ------------------------------------------------------------
//
// AN UNBOUND VECTOR HERE IS A SILENT DEATH: the crt's default handler is
// a spin loop, so the first console interrupt would park the CPU in it
// and the board would simply never say anything.
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

/// The fault path, which on a board with DHCSR.C_DEBUGEN cleared is also
/// where an orderly panic() ends up. It marks the path and then does the
/// two things in the order that matters: capture (freeze first), then
/// the record-preserving reset.
extern "C" void HardFault_Handler() {
    token.path = 2;
    brio::hard_fault_trace_reset<brio::SamPlatform, Trace>();
}

int main() {
    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();

    brio::enable_interrupts();

    bench.letter('a', "the record: capture, checksum, take, and the standing "
                      "diagnosis", ta_record);
    bench.letter('b', "why freeze() comes first: the reader overwrites the "
                      "evidence", tb_freeze_first);
    bench.letter('c', "the walk: oldest first, wrapped and young", tc_walk);
    bench.letter('f', "a REAL HardFault across a reset", tf_fault, false);
    bench.letter('p', "an orderly panic() across a reset", tp_panic, false);

    const bool resuming = token.magic == token_magic;
    const uint8_t leg = resuming ? token.leg : 0u;

    if (serial_ok) {
        if (leg == 1u) {
            tf_resume();
        } else if (leg == 2u) {
            tp_resume();
        } else {
            print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
                  " tick=", tick_ok ? "SysTick" : "FAILED", crlf);
            banner();
        }
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            bench.menu();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
