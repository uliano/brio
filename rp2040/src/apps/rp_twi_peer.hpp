// rp_twi_peer.hpp - THE twi_peer OF THE TWO RASPBERRY PI CHIPS, written
// once over the Synopsys DW_apb_i2c (brio/dw_apb_i2c/i2c.hpp) and bound by
// rp2040/src/apps/twi_peer.cpp and rp2350/src/apps/twi_peer.cpp: a
// scriptable second chip on the bus for the far end of a two-board I2C
// test, commanded IN BAND over the very bus both are testing through
// avrdx/src/apps/twi_link.hpp.
//
// ONE BODY, TWO CHIPS: everything below is the IP's and the protocol's;
// what is a chip's - the instance, the two pads, the clock, the console,
// the microsecond ruler, the label - the binding file states before it
// includes this one, inside its own anonymous namespace with `using
// namespace brio` in force:
//
//   SysClock, clock          the app's Clock type and object (ic_clk is
//                            clk_sys on both chips)
//   Console, console         the console UART type and object
//   Client, Host, Raw        I2cClient<n, pins>, I2cHost<n, pins>,
//                            DwApbI2c<n> on the link's pads
//   SdaPin, SclPin           the two link pads as Pin types
//   us_now()                 the microsecond ruler (the system timer)
//   peer_kind, peer_label    twi_link's kind byte and the eight-character
//                            Ident label
//   bus_text                 the pads, for the banner
//   link_pads_free()         may a link pad be driven now (the RP2350's
//                            partner-rail guard; true on the RP2040)
//
// What this silicon makes of the protocol (the DW chapter of either
// datasheet: RP2040 4.3, RP2350 12.2 - the same v2.03a block):
//
//  - THE BLOCK IS A HOST OR A CLIENT, NEVER BOTH (RP2040 4.3.10, RP2350
//    12.2.10's operation modes: IC_CON's MASTER_MODE and SLAVE_DISABLE
//    must be set as a pair). So `arb` and `host_read` SWITCH the instance
//    to host for their bounded moment and back, and `arb`'s client half is
//    REFUSED: the action is acknowledged and its host half runs, the
//    client being simply absent - the Report carries the host half alone
//    (addr_hits 0, no client flag) - which is all a race verdict needs:
//    the loser's ARB_LOST and the winner's tenure against nobody's ACK.
//  - THE CLIENT IS POLLED. The thirteen sources stay masked; the loop reads
//    IC_RAW_INTR_STAT and the FIFO levels, and the line serves the host
//    action's engine alone. A commanded hold is a wait the loop spends
//    where it stands.
//  - THE CLIENT STRETCHES AS A TRANSMITTER ONLY. A read raises RD_REQ and
//    the block holds SCL until a byte is in its transmit FIFO (the block's
//    own stretch); a written byte lands in a sixteen-deep receive FIFO and
//    SCL is held only once that is full (RX_FIFO_FULL_HLD_CTRL). So
//    `hold_us` stretches every byte of a READ - one byte given per RD_REQ,
//    after the hold - and a write not at all.
//  - A READ UNHELD IS SERVED A FIFO AHEAD: up to sixteen bytes given per
//    RD_REQ, so the far end's own pace is the wire's and not this loop's.
//    The bytes the host does not take are flushed at its closing NACK
//    (ABRT_SLVFLUSH_TXFIFO, the count in IC_TX_ABRT_SOURCE's
//    TX_FLUSH_CNT) and the tally gives them back, so the counting pattern
//    of the next read resumes where the host stopped. The `counters` query
//    hands the drops to the DUT.
//  - A DATA NACK AT BYTE n. IC_SLV_DATA_NACK_ONLY refuses every data byte
//    and takes a write only with the block disabled and its client idle -
//    never mid-tenure. So nack_at = 1 is that register, set before the
//    action; nack_at = n > 1 is a DISABLE written once byte n - 1 has been
//    taken: "if the module is receiving, it stops the current transfer at
//    the end of the current byte and does not acknowledge" (RP2040 4.3.17,
//    IC_ENABLE). The block back idle, NACK_ONLY goes on and the block is
//    enabled again, so every later byte is refused too, as the protocol's
//    cumulative count asks.
//  - NO ADDRESS-MATCH EVENT: a tenure is counted at its end, the STOP
//    (STOP_DET, raised for this client's tenures alone) or the repeated
//    START (RESTART_DET), and `addr_hits` is their sum. A general call has
//    no STOP_DET under STOP_DET_IFADDRESSED (the note on RP2040 4.3.17's
//    IC_CON bit 7): it is counted at its GEN_CALL, once a START, and makes
//    `last_addr` 0x00.
//  - THE STUCK LINE: `hold_sda` takes SDA to SIO once the wire has been
//    at rest 40 us (a tenure's longest high half is 5 us at 100 kHz),
//    driven low - the one level an I2C node may drive - and released after
//    `aux8` SCL falling edges read through SIO, which reads the pad
//    whoever owns it.
//  - THE HOST ACTION'S RENDEZVOUS is the DUT's window: SDA pulled low by
//    hand under a high SCL, a START every node on the wire sees, then the
//    STOP that closes it. This end's start() is written INSIDE the window,
//    `aux16` microseconds after it opened: a DW master holds a START while
//    the bus is busy (RP2040 4.3.8: "no other master can take control
//    until the first master sends a STOP"), so both hosts' STARTs wait on
//    the one STOP edge and race on it.
//  - THE TIMING: the client is configured for fast-mode-plus, the fastest
//    bus the DUT walks (its spike filter and SDA setup follow it); the
//    host action runs at 100 kHz.
//
// COEXISTENCE is the protocol header's argument: in command mode IC_SAR is
// 0x6B and nothing else - general call off, one address register and no
// mask - so nothing the DUT's single-board letters do wakes this board.

#pragma once

#include <stdint.h>

#include "kernel/borrowed.hpp"
#include "util/print.hpp"

// THE PROTOCOL HEADER IS SHARED, NOT COPIED: pure encoding, not one
// register, and every architecture on this link compiles the same file.
#include "../../../avrdx/src/apps/twi_link.hpp"

using twilink::Op;

constexpr uint16_t firmware_version = static_cast<uint16_t>((peer_kind << 8) | 0x01u);   ///< Ident

bool trace = false;
bool client_up = false;

twilink::Decoder decoder;
twilink::Report last_report;

uint32_t commands = 0, actions = 0, naks = 0, answers = 0, flushed_total = 0;
uint16_t last_drops = 0;

uint8_t resp_[twilink::response_bytes];
uint8_t resp_len_ = 0;
uint8_t resp_pos_ = 0;
uint8_t own_addr_ = twilink::command_addr;
bool read_done_ = false;
Op pending_ = Op::ping;
bool has_pending_ = false;
twilink::Params pending_params_{};

/// The host action's completion, set by the vector while the instance
/// wears its HOST face.
volatile bool host_live = false;
volatile bool host_done = false;
volatile uint8_t host_status = i2c_ok;
/// What reached the vector while the client was polled: counted, disarmed.
volatile uint32_t stray_vectors = 0;

// ---- waits ----------------------------------------------------------------------

/// A hold of any length on the system timer, read once a turn.
void hold_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}

bool expired(uint32_t t0, uint32_t ms) { return us_now() - t0 >= ms * 1000u; }

/// The DUT's window, on the pads (SIO reads the pad whoever owns it):
/// `stop` false waits for SDA LOW under a high SCL - the START it makes by
/// hand -, `stop` true for SDA HIGH under a high SCL - the STOP that closes
/// it. The deadline is read once every 256 polls.
bool wait_wire(uint32_t t0, uint32_t ms, bool stop) {
    uint8_t guard = 0;
    for (;;) {
        if (SdaPin::read() == stop && SclPin::read()) {
            return true;
        }
        ++guard;
        if (guard == 0u && expired(t0, ms)) {
            return false;
        }
    }
}

/// THE WIRE AT REST: both lines high without a break for `quiet_us`, within
/// `ms` of `t0` - longer than any SCL high half on this bus (5 us at
/// 100 kHz), so a tenure in flight never passes for an idle bus.
bool wait_wire_quiet(uint32_t t0, uint32_t ms, uint32_t quiet_us) {
    uint32_t since = us_now();
    while (!expired(t0, ms)) {
        if (SclPin::read() && SdaPin::read()) {
            if (us_now() - since >= quiet_us) {
                return true;
            }
        } else {
            since = us_now();
        }
    }
    return false;
}

/// The low byte of IC_STATUS - ACTIVITY, the four FIFO flags, the host's
/// and the client's activity - which is what fits the protocol's one-byte
/// status fields.
uint8_t status_byte() { return static_cast<uint8_t>(Raw::flags() & 0xFFu); }

// ---- the two standing configurations ----------------------------------------------

/// The client up at one exact address, POLLED: every source masked, the
/// line serving the host face only.
bool bring_up_client(uint8_t addr, bool general_call) {
    host_live = false;
    own_addr_ = addr;
    client_up = Client::init(clock, {.address = addr, .general_call = general_call, .speed = I2cSpeed::fast_plus_1m});
    Client::interrupts(Client::events, false);
    return client_up;
}

/// COMMAND MODE: one exact address, nothing else. Everything returns here.
bool go_command() {
    const bool ok = bring_up_client(twilink::command_addr, false);
    decoder.reset();
    resp_pos_ = 0;
    read_done_ = false;
    return ok;
}

// ---- the command channel --------------------------------------------------------

void set_response(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    twilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof resp_) {
                resp_[n++] = b;
            }
        },
        op, p, len);
    resp_len_ = n;
    resp_pos_ = 0;
    ++answers;
}

void prepare_nak(Op op, uint8_t sum) {
    const uint8_t p[2] = {twilink::byte_of(op), sum};
    set_response(Op::nak, p, 2);
    has_pending_ = false;
    ++naks;
}

/// ACK BEFORE ACT: the action starts only once the DUT has collected the
/// acknowledgement.
void prepare(const twilink::Frame& f) {
    ++commands;
    if (trace) {
        print(console, "  [cmd op=", hex(twilink::byte_of(f.op)), " len=", f.len, "]", crlf);
    }
    if (twilink::is_extended_action(f.op) && f.len < twilink::params_size) {
        prepare_nak(f.op, f.sum);
        return;
    }
    const uint8_t p[2] = {twilink::byte_of(f.op), f.sum};
    set_response(Op::ack, p, 2);
    pending_ = f.op;
    has_pending_ = true;
    if (twilink::is_extended_action(f.op)) {
        pending_params_ = twilink::get_params(f.data);
    }
}

/// One pass of the command-mode client. THE ORDER IS LOAD-BEARING: the
/// receive FIFO first, so a frame is whole before its STOP resets the
/// decoder (one write tenure is exactly one frame), and the closing NACK
/// of the response read before the read request - it is what says "the
/// answer has been collected", ack-before-act's own trigger.
void service_command() {
    while (Client::data_ready()) {
        switch (decoder.feed(Client::take())) {
            case twilink::Decoder::Result::frame:
                if (twilink::is_extended_command(decoder.frame().op)) {
                    prepare(decoder.frame());
                } else {
                    prepare_nak(decoder.frame().op, decoder.frame().sum);
                }
                break;
            case twilink::Decoder::Result::bad_checksum:
                prepare_nak(decoder.pending_op(), decoder.frame().sum);
                break;
            default: break;
        }
    }
    const uint32_t raw = Raw::raw_pending();
    if ((raw & I2cInterrupt::tx_abrt) != 0u) {
        flushed_total += Raw::flushed_entries();
        (void)Raw::clear_abort();
        return;
    }
    if ((raw & I2cInterrupt::rx_done) != 0u) {
        Client::clear_nack();
        read_done_ = true;
        return;
    }
    if ((raw & I2cInterrupt::rd_req) != 0u) {
        uint8_t v = 0x00;
        if (resp_pos_ < resp_len_) {
            v = resp_[resp_pos_];
            ++resp_pos_;
        }
        Client::give(v);
        Client::clear_read_request();
        return;
    }
    if ((raw & (I2cInterrupt::stop_det | I2cInterrupt::restart_det)) != 0u) {
        // The tenure's end: a write was exactly one frame; a read's next
        // tenure starts the answer from its first byte.
        Raw::clear_pending(raw & (I2cInterrupt::stop_det | I2cInterrupt::restart_det));
        decoder.reset();
        resp_pos_ = 0;
        return;
    }
    if ((raw & (I2cInterrupt::rx_over | I2cInterrupt::gen_call)) != 0u) {
        Raw::clear_pending(raw & (I2cInterrupt::rx_over | I2cInterrupt::gen_call));
    }
}

// ---- the actions ----------------------------------------------------------------

/// Be a client: stretch, NACK, general call, deafness - and, through
/// `coll`, the shared-address fixed-byte flavour (S4, whose third party no
/// desk of this pair carries).
twilink::Report run_serve(const twilink::Params& a, bool fixed_byte) {
    twilink::Report r{};
    const uint8_t addr = (a.flags & twilink::flag_deaf) != 0u
                             ? twilink::deaf_addr
                             : (a.addr != 0u ? a.addr : twilink::command_addr);
    if (!bring_up_client(addr, (a.flags & twilink::flag_general_call) != 0u)) {
        return r;
    }
    // nack_at = 1: every data byte refused from the first (the header).
    bool refusing = false;
    if (a.nack_at == 1u) {
        refusing = Client::acknowledge(false);
        if (refusing) {
            r.flags |= twilink::report_nacked;
        }
    }
    bool cut = false;   // the block disabled mid-tenure, refusing byte nack_at
    bool gc_seen = false;
    uint16_t rx = 0, tx = 0;
    bool first_taken = false;
    const uint32_t t0 = us_now();
    for (;;) {
        if (expired(t0, a.ms)) {
            r.flags |= twilink::report_timed_out;
            break;
        }
        if (a.count != 0u && static_cast<uint16_t>(rx + tx) >= a.count && r.stops != 0u) {
            break;
        }
        if (cut) {
            // The refusal under way: the block finishes the byte in flight,
            // does not acknowledge it, and goes idle with the host's STOP.
            if (!Raw::running()) {
                if (Raw::client_rx_data_lost()) {
                    r.flags |= twilink::report_nacked;
                }
                (void)Raw::nack_data(true);
                refusing = true;
                Raw::clear_all();
                Raw::enable();
                cut = false;
                ++r.addr_hits;
                if (r.stops < 255u) {
                    ++r.stops;
                }
            }
            continue;
        }
        if (Client::data_ready()) {
            const uint8_t v = Client::take();
            if (!first_taken) {
                r.first = v;
                first_taken = true;
            }
            r.sum = static_cast<uint16_t>(r.sum + v);
            ++rx;
            if (a.nack_at > 1u && !refusing && rx + 1u == a.nack_at) {
                // Byte nack_at is on the wire now: the disable refuses it
                // (no spin: the loop watches the block go idle).
                (void)Raw::disable(0);
                cut = true;
            }
            continue;
        }
        const uint32_t raw = Raw::raw_pending();
        if ((raw & I2cInterrupt::tx_abrt) != 0u) {
            // Bytes given ahead and not taken: flushed at the closing NACK
            // (or at the next read), and given back to the tally.
            const uint16_t m = Raw::flushed_entries();
            (void)Raw::clear_abort();
            tx = m <= tx ? static_cast<uint16_t>(tx - m) : uint16_t{0};
            last_drops = static_cast<uint16_t>(last_drops + m);
            flushed_total += m;
            continue;
        }
        if ((raw & I2cInterrupt::rd_req) != 0u) {
            if (a.hold_us != 0u) {
                hold_us(a.hold_us);
                Client::give(fixed_byte ? a.seed : twilink::pattern_value(a.pattern, a.seed, tx));
                ++tx;
            } else {
                while (Client::writable()) {
                    Client::give(fixed_byte ? a.seed : twilink::pattern_value(a.pattern, a.seed, tx));
                    ++tx;
                }
            }
            Client::clear_read_request();
            continue;
        }
        if ((raw & I2cInterrupt::rx_done) != 0u) {
            // The closing NACK of a read: the flags sampled AT it.
            r.mstatus = status_byte();
            Client::clear_nack();
            continue;
        }
        if ((raw & I2cInterrupt::start_det) != 0u) {
            gc_seen = false;
            Raw::clear_pending(I2cInterrupt::start_det);
            continue;
        }
        if ((raw & I2cInterrupt::gen_call) != 0u) {
            // A general call is a tenure of its own with NO STOP_DET (IC_CON's
            // note on STOP_DET_IFADDRESSED), counted here once: GEN_CALL
            // stands through the address's acknowledge, so a clear inside it
            // does not hold, and the next START is what re-arms the count.
            if (!gc_seen) {
                gc_seen = true;
                ++r.addr_hits;
                r.last_addr = 0x00;
            }
            Raw::clear_pending(I2cInterrupt::gen_call);
            continue;
        }
        if ((raw & I2cInterrupt::restart_det) != 0u) {
            ++r.addr_hits;
            Raw::clear_pending(I2cInterrupt::restart_det);
            continue;
        }
        if ((raw & I2cInterrupt::stop_det) != 0u) {
            ++r.addr_hits;
            if (r.stops < 255u) {
                ++r.stops;
            }
            r.last_addr = own_addr_;
            Raw::clear_pending(I2cInterrupt::stop_det);
            continue;
        }
        if ((raw & I2cInterrupt::rx_over) != 0u) {
            r.flags |= twilink::report_buserr;
            Raw::clear_pending(I2cInterrupt::rx_over);
        }
    }
    r.count = static_cast<uint16_t>(rx + tx);
    r.aux0 = static_cast<uint8_t>(rx);
    r.aux1 = static_cast<uint8_t>(tx);
    r.sstatus = status_byte();
    return r;
}

/// THE HOST ACTION, a ROLE SWITCH (the header): the client released, the
/// host brought up, the DUT's window waited for, this end's start() written
/// inside it. `arb` writes the pattern to `target` once; `host_read` reads
/// `count` bytes from it, `aux8` times back to back (four at most), and the
/// Report carries the bytes read (twi_link.hpp).
twilink::Report run_host(const twilink::Params& a, bool read) {
    twilink::Report r{};
    Client::release();
    client_up = false;
    host_live = true;
    if (!Host::init(clock)) {
        host_live = false;
        (void)go_command();
        return r;
    }
    static uint8_t burst[32];
    const uint8_t n = a.count < sizeof burst ? static_cast<uint8_t>(a.count) : static_cast<uint8_t>(sizeof burst);
    for (uint8_t i = 0; i < n; ++i) {
        burst[i] = twilink::pattern_value(a.pattern, a.seed, i);
    }
    // THE WIRE AT REST FIRST: this action starts at the closing NACK of the
    // read that collected its ack, a bit time before the DUT's STOP.
    const uint32_t t0 = us_now();
    const bool window = wait_wire_quiet(t0, a.ms, 40u) && wait_wire(t0, a.ms, false);
    if (!window) {
        r.flags |= twilink::report_timed_out;
        host_live = false;
        Host::release();
        (void)go_command();
        return r;
    }
    if (a.aux16 != 0u) {
        hold_us(a.aux16);
    }
    r.flags |= twilink::report_host_ran;
    const auto tenure = [&]() -> bool {
        host_done = false;
        Host::Request q{};
        q.addr = a.target;
        if (read) {
            q.rx = lend<Lease::reply>(static_cast<uint8_t*>(burst));
            q.rx_len = n;
        } else {
            q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(burst));
            q.tx_len = n;
        }
        q.speed = I2cSpeed::standard_100k;
        if (Host::start(q)) {
            host_status = Host::status();
            host_done = true;
        } else {
            while (!host_done && !expired(t0, a.ms)) {
            }
        }
        return host_done;
    };
    const uint8_t tenures = read ? (a.aux8 == 0u ? uint8_t{1} : (a.aux8 > 4u ? uint8_t{4} : a.aux8)) : uint8_t{1};
    for (uint8_t k = 0; k < tenures; ++k) {
        if (read) {
            for (uint8_t i = 0; i < n; ++i) {
                burst[i] = 0xEE;
            }
        }
        if (!tenure()) {
            r.flags |= twilink::report_timed_out;
            (void)Host::recover();
            break;
        }
        r.mstatus = host_status;
        if (host_status == i2c_ok) {
            r.flags |= twilink::report_host_ok;
        }
        if (host_status == i2c_arb_lost) {
            r.flags |= twilink::report_arblost;
        }
        if (host_status == i2c_bus_error) {
            r.flags |= twilink::report_buserr;
        }
        r.aux1 = n;
        if (!read || host_status != i2c_ok) {
            break;
        }
        r.count = static_cast<uint16_t>(r.count + n);
        if (k == 0u) {
            r.first = burst[0];
        }
        r.aux0 = burst[0];
        for (uint8_t i = 0; i < n; ++i) {
            r.sum = static_cast<uint16_t>(r.sum + burst[i]);
        }
    }
    if (read) {
        r.addr_hits = static_cast<uint16_t>(r.count / (n == 0u ? 1u : n));
    }
    host_live = false;
    Host::release();
    (void)go_command();
    return r;
}

/// A STUCK client, by hand: SDA held low from SIO with the I2C off the
/// pads, released after `aux8` SCL falling edges (0 = only the deadline
/// does). Only the low level is ever driven; the release is an input, the
/// RP2040's pull-ups taking the line back.
twilink::Report run_hold_sda(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    client_up = false;
    SclPin::input(PinPull::none);
    SdaPin::input(PinPull::none);
    const uint32_t t0 = us_now();
    (void)wait_wire_quiet(t0, a.ms, 40u);
    SdaPin::output(false);
    r.flags |= twilink::report_host_ran;
    uint16_t falls = 0;
    bool prev = SclPin::read();
    uint8_t guard = 0;
    for (;;) {
        const bool now = SclPin::read();
        if (prev && !now) {
            ++falls;
            if (a.aux8 != 0u && falls >= a.aux8) {
                break;
            }
        }
        prev = now;
        ++guard;
        if (guard == 0u && expired(t0, a.ms)) {
            r.flags |= twilink::report_timed_out;
            break;
        }
    }
    SdaPin::input(PinPull::none);
    hold_us(5);
    r.count = falls;
    r.aux0 = static_cast<uint8_t>(falls);
    r.aux1 = SdaPin::read() ? 1u : 0u;
    return r;
}

/// This board's I2C off the wire for the deadline: the control case.
twilink::Report run_quiet(const twilink::Params& a) {
    twilink::Report r{};
    Client::release();
    client_up = false;
    hold_us(static_cast<uint32_t>(a.ms) * 1000u);
    r.count = 1;
    return r;
}

void run_pending() {
    const Op op = pending_;
    has_pending_ = false;
    if (op == Op::ping) {
        return;
    }
    if (op == Op::ident) {
        twilink::Ident d{};
        for (uint8_t i = 0; i < 8u; ++i) {
            d.label[i] = peer_label[i];
        }
        d.xtal = xtal_ok ? 1u : 0u;
        d.sanity = twilink::ident_sanity;
        d.version = firmware_version;
        uint8_t p[twilink::ident_size];
        twilink::put_ident(p, d);
        set_response(Op::ident_data, p, twilink::ident_size);
        return;
    }
    if (op == Op::report) {
        uint8_t p[twilink::report_size];
        twilink::put_report(p, last_report);
        set_response(Op::report_data, p, twilink::report_size);
        return;
    }
    if (op == Op::counters) {
        twilink::Counters k{};
        k.drops = last_drops;
        k.drops_total = static_cast<uint16_t>(flushed_total);
        k.stray = static_cast<uint16_t>(stray_vectors);
        k.commands = static_cast<uint16_t>(commands);
        uint8_t p[twilink::counters_size];
        twilink::put_counters(p, k);
        set_response(Op::counters_data, p, twilink::counters_size);
        return;
    }
    ++actions;
    const twilink::Params a = pending_params_;
    const uint32_t started = us_now();
    last_drops = 0;
    twilink::Report r{};
    switch (op) {
        case Op::serve: r = run_serve(a, false); break;
        case Op::coll: r = run_serve(a, true); break;
        case Op::arb: r = run_host(a, false); break;
        case Op::host_read: r = run_host(a, true); break;
        case Op::hold_sda: r = run_hold_sda(a); break;
        case Op::quiet: r = run_quiet(a); break;
        default: break;
    }
    r.flags |= twilink::report_ran;
    const uint32_t took = (us_now() - started) / 1000u;
    r.ms = took > 255u ? uint8_t{255} : static_cast<uint8_t>(took);
    r.op = twilink::byte_of(op);
    last_report = r;
    (void)go_command();
}

// ---- the console ----------------------------------------------------------------

void help() { print(console, "twi_peer: ? help | i status | 0 back to command mode | 3 trace", crlf); }

void status() {
    print(console, "  ", bus_text, ": client ", client_up ? "up" : "OFF", ", IC_STATUS=", hex(Raw::flags()),
          " raw=", hex(Raw::raw_pending()), " own address=", hex(own_addr_), crlf);
    print(console, "  SDA reads ", SdaPin::read() ? "high" : "LOW", ", SCL reads ", SclPin::read() ? "high" : "LOW",
          crlf);
    print(console, "  commands=", commands, " actions=", actions, " naks=", naks, " answers=", answers,
          " flushed=", flushed_total, " stray vectors=", stray_vectors, crlf);
    print(console, "  last report: op=", hex(last_report.op), " count=", last_report.count, " rx/tx=",
          last_report.aux0, "/", last_report.aux1, " hits=", last_report.addr_hits, " stops=", last_report.stops,
          " flags=", hex(last_report.flags), crlf);
    print(console, "               sum=", hex(last_report.sum), " first=", hex(last_report.first),
          " lastaddr=", hex(last_report.last_addr), " status=", hex(last_report.sstatus),
          " at-NACK=", hex(last_report.mstatus), " ms=", last_report.ms, crlf);
}

/// The instance's ONE vector. The client is POLLED, so its only owner is
/// the host action's engine; anything else is DISARMED and nothing is
/// cleared - every flag this instrument lives on belongs to the loop.
[[gnu::always_inline]] inline void peer_vector() {
    if (host_live) {
        if (Host::isr()) {
            host_status = Host::status();
            host_done = true;
        }
        return;
    }
    stray_vectors = stray_vectors + 1u;
    Raw::interrupts_only(0);
}

/// The loop: the console, then - with the link pads free - the command
/// channel and the pending action. With them not free (the RP2350's
/// partner rail down) the client stays off, asked again once a second.
[[noreturn]] void peer_loop() {
    uint32_t asked = us_now();
    for (;;) {
        uint8_t c = 0;
        if (Console::read_byte(c) && c != '\r' && c != '\n') {
            print(console, static_cast<char>(c), crlf);
            if (c == '?') {
                help();
            } else if (c == 'i') {
                status();
            } else if (c == '0') {
                has_pending_ = false;
                if (link_pads_free()) {
                    (void)go_command();
                }
                print(console, client_up ? "  command-mode client restored" : "  the client stays off", crlf);
            } else if (c == '3') {
                trace = !trace;
                print(console, trace ? "  trace on" : "  trace off", crlf);
            } else {
                print(console, "? for help", crlf);
            }
            print(console, "> ");
        }
        if (!client_up) {
            if (us_now() - asked >= 1'000'000u) {
                asked = us_now();
                if (link_pads_free()) {
                    (void)go_command();
                    print(console, crlf, "  the link pads are free: command-mode client up", crlf, "> ");
                }
            }
            continue;
        }
        service_command();
        if (read_done_) {
            read_done_ = false;
            if (has_pending_) {
                run_pending();
            }
        }
    }
}
