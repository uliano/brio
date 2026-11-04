// rp_i2c_pair.hpp - THE TWO-BOARD I2C LETTERS OF THE TWO RASPBERRY PI
// CHIPS, written once over the Synopsys DW_apb_i2c (brio/dw_apb_i2c/
// i2c.hpp) and included by test_rp2040_i2c and test_rp2350_i2c: this chip
// the DUT and the bus host, the desk partner running `twi_peer`
// (rp_twi_peer.hpp) commanded IN BAND over the very bus under test through
// avrxt/src/apps/twi_link.hpp - the RP2040 against the RP2350's peer, the
// RP2350 (both halves) against the RP2040's.
//
// ONE BODY, TWO CHIPS: what is a chip's - the instance, the pads, the
// clock, the ruler, the platform, the partner's name and kind - the
// including suite states before it includes this file, inside its own
// anonymous namespace with `using namespace brio` in force, and having
// included kernel/, util/i2c_bus.hpp, util/power.hpp, util/testbench.hpp
// and twi_link.hpp at file scope:
//
//   SysClock, clock, P, Serial, serial, bench
//   Host, DmaHost, Client, Raw   the link instance: the pump, the host on
//                                DMA channels 4 and 5 (isr_dma_0), the
//                                client, the resource
//   DmaTx, DmaRx                 DmaHost's two engine types
//   SdaPin, SclPin               the link's pads
//   us_now()                     the microsecond ruler (the system timer)
//   xip_empty()                  every line of the XIP cache dropped (the
//                                family's Xip verb): letter g's cold read
//   partner_kind, partner_text   twi_link's kind byte of the peer, its name
//   late_cycles()                clk_sys cycles, letters u and s's fine ruler
//   late_line(on)                the link instance's interrupt line on or off,
//                                in the family's interrupt controller
//   late_flash(i)                16 KB of the flash through the XIP's uncached
//                                alias, the i-th of four
//
// and binds its I2C vector to `pair_i2c_isr()` and isr_dma_0 to
// `pair_dma_isr()`.
//
// THE LINK'S RULES (docs/boards/weact-rp2040.md, weact-rp2350b.md): the
// I2C pull-ups are 1.5 kOhm and the select's 4.7 kOhm, all to the RP2040's
// 3V3; the partner runs
// twi_peer, which is deaf but at 0x6B; nothing here drives a link pad but
// the I2C's two. The DUT pulls SDA low BY HAND for the arbitration and
// role letters - IO_BANK0's output overrides (RP2040 2.19.6.1, RP2350 9.4)
// forcing the pad's output low and its output enable on while the I2C
// keeps the pad and keeps seeing it - which is a START condition every
// node on the wire sees, and gives it back for the STOP that closes the
// window.
//
// THE RULER is the system timer, one microsecond; every time-out below is
// a deadline on it, read through `wait_until()`.

#pragma once

using twilink::Op;

/// An address nothing on the pair's bus answers: the peer listens at 0x6B
/// in command mode and at the address an action names, and twilink's
/// low_addr (0x11) is answered by nobody on this desk either.
constexpr uint8_t nobody = 0x42;
constexpr I2cSpeed link_speed = I2cSpeed::standard_100k;
/// A tenure that never answered: the suite's own code, not a wire one.
constexpr uint8_t no_answer = 0xFE;

// ---- the vector's owner ----------------------------------------------------------

enum class Owner : uint8_t { none, host, dma_host, client };
volatile Owner owner = Owner::none;
volatile uint32_t isr_entries = 0;
volatile uint32_t dma_entries = 0;
volatile bool transfer_done = false;
volatile uint8_t transfer_status = 0;
volatile bool bus_ao_live = false;

uint8_t tx_buf[256];
uint8_t rx_buf[256];

// ---- shared waits -----------------------------------------------------------------

/// Spin until `done()` or `us` microseconds have passed; true when done.
template <typename F>
bool wait_until(F done, uint32_t us) {
    const uint32_t t0 = us_now();
    while (!done()) {
        if (us_now() - t0 >= us) {
            return done();
        }
    }
    return true;
}
void spin_us(uint32_t us) {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < us) {
    }
}
void settle_ms(uint32_t ms) { spin_us(ms * 1000u); }

// ---- this board's client, served from its vector (letter l) ----------------------

uint8_t client_answers[64];
uint8_t client_heard[64];
volatile uint16_t client_given = 0;
volatile uint16_t client_count = 0;
volatile uint16_t client_stops = 0;
volatile uint16_t client_restarts = 0;
volatile uint16_t client_nacks = 0;
volatile uint16_t client_flushes = 0;
volatile uint16_t client_errors = 0;
volatile uint16_t client_gcalls = 0;
volatile uint16_t client_overruns = 0;
/// Letter m's cut: the read request numbered `client_cut_at` (from 0) is
/// not answered - its RD_REQ masked, SCL left held - and `client_cut` says
/// so; 0xFFFF never.
volatile uint16_t client_cut_at = 0xFFFFu;
volatile bool client_cut = false;

void client_reset_tally() {
    client_given = 0;
    client_count = 0;
    client_stops = 0;
    client_restarts = 0;
    client_nacks = 0;
    client_flushes = 0;
    client_errors = 0;
    client_gcalls = 0;
    client_overruns = 0;
    client_cut_at = 0xFFFFu;
    client_cut = false;
    for (uint8_t i = 0; i < sizeof client_heard; ++i) {
        client_heard[i] = 0xEE;
    }
}

/// One event of the client: a written byte stored, a read request answered
/// with ONE byte (nothing queued beyond what the host takes), the rest
/// counted.
void client_serve(I2cClientEvent ev) {
    switch (ev) {
        case I2cClientEvent::byte_received:
            while (Client::data_ready()) {
                const uint8_t b = Client::take();
                if (client_count < sizeof client_heard) {
                    client_heard[client_count] = b;
                }
                client_count = client_count + 1u;
            }
            break;
        case I2cClientEvent::byte_wanted:
            if (client_given == client_cut_at) {
                client_cut = true;
                Client::interrupts(I2cInterrupt::rd_req, false);
                break;
            }
            Client::give(client_given < sizeof client_answers ? client_answers[client_given] : uint8_t{0xFF});
            client_given = client_given + 1u;
            Client::clear_read_request();
            break;
        case I2cClientEvent::stop: client_stops = client_stops + 1u; break;
        case I2cClientEvent::restart: client_restarts = client_restarts + 1u; break;
        case I2cClientEvent::nacked: client_nacks = client_nacks + 1u; break;
        case I2cClientEvent::flushed: client_flushes = client_flushes + 1u; break;
        case I2cClientEvent::general_call: client_gcalls = client_gcalls + 1u; break;
        case I2cClientEvent::error: client_errors = client_errors + 1u; break;
        case I2cClientEvent::overrun: client_overruns = client_overruns + 1u; break;
        default: break;
    }
}

/// This board as a client at `addr` (the general call answered or not),
/// its events on its vector.
bool client_ready(uint8_t addr, bool general_call, bool hold_when_rx_full = true) {
    owner = Owner::none;
    client_reset_tally();
    const bool ok = Client::init(clock, {.address = addr,
                                         .general_call = general_call,
                                         .speed = I2cSpeed::fast_plus_1m,
                                         .hold_when_rx_full = hold_when_rx_full});
    Client::interrupts(Client::events, true);
    owner = Owner::client;
    return ok;
}

// ---- the host's tenures -------------------------------------------------------------

void host_ready() {
    owner = Owner::none;
    bus_ao_live = false;
    (void)Host::init(clock);
    owner = Owner::host;
}

/// One tenure through host H on the engine named, waited out. The status,
/// or no_answer for a tenure that never answered within `wait_us` (the
/// engine recover()ed). `dma_entries` counts the DMA line's entries over
/// it, `isr_entries` the host vector's.
template <typename H>
uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
               I2cSpeed speed = link_speed, uint32_t wait_us = 200'000u,
               I2cEngine engine = I2cEngine::interrupt) {
    typename H::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    r.engine = engine;
    transfer_done = false;
    isr_entries = 0;
    dma_entries = 0;
    if (H::start(r)) {
        return H::status();
    }
    if (!wait_until([] { return transfer_done; }, wait_us)) {
        (void)H::recover();
        return no_answer;
    }
    return transfer_status;
}

// ---- the peer's command channel (twi_link) ------------------------------------------

uint8_t frame_buf[twilink::max_payload + 4];
uint8_t resp_buf[twilink::response_bytes];
bool link_quiet = false;
I2cSpeed command_speed = link_speed;

bool send_frame(Op op, const uint8_t* p, uint8_t len) {
    uint8_t n = 0;
    twilink::write_frame(
        [&](uint8_t b) {
            if (n < sizeof frame_buf) {
                frame_buf[n++] = b;
            }
        },
        op, p, len);
    return tenure<Host>(twilink::command_addr, frame_buf, n, nullptr, 0, command_speed) == i2c_ok;
}

bool recv_frame(twilink::Frame& out) {
    twilink::Decoder dec;
    for (uint8_t i = 0; i < twilink::response_bytes; ++i) {
        resp_buf[i] = 0;
    }
    if (tenure<Host>(twilink::command_addr, nullptr, 0, resp_buf, twilink::response_bytes, command_speed) != i2c_ok) {
        return false;
    }
    for (uint8_t i = 0; i < twilink::response_bytes; ++i) {
        if (dec.feed(resp_buf[i]) == twilink::Decoder::Result::frame) {
            out = dec.frame();
            return true;
        }
    }
    return false;
}

bool command_once(Op op, const uint8_t* p, uint8_t len) {
    if (!send_frame(op, p, len)) {
        return false;
    }
    twilink::Frame f;
    if (!recv_frame(f)) {
        return false;
    }
    return f.op == Op::ack && f.len == 2 && f.data[0] == twilink::byte_of(op);
}

const uint8_t no_payload[1] = {0};

/// Three attempts, the peer's own recovery bound (an action's deadline)
/// waited between them; then arm_ms, ack before act's margin.
bool command(Op op, const uint8_t* p = no_payload, uint8_t len = 0) {
    for (uint8_t k = 0; k < 3u; ++k) {
        if (command_once(op, p, len)) {
            settle_ms(twilink::arm_ms);
            return true;
        }
        host_ready();
        settle_ms(400);
    }
    if (!link_quiet) {
        print(serial, "    LINK FAILURE op ", hex(twilink::byte_of(op)), ": ", partner_text,
              " must be running twi_peer", crlf);
    }
    return false;
}

bool query(Op op, twilink::Frame& data) { return command(op) && recv_frame(data); }

bool peer_report(twilink::Report& r) {
    for (uint8_t k = 0; k < 4u; ++k) {
        twilink::Frame f;
        if (query(Op::report, f) && f.op == Op::report_data && f.len == twilink::report_size) {
            r = twilink::get_report(f.data);
            return true;
        }
        settle_ms(60);
    }
    return false;
}

bool peer_counters(twilink::Counters& c) {
    for (uint8_t k = 0; k < 2u; ++k) {
        twilink::Frame f;
        if (query(Op::counters, f) && f.op == Op::counters_data && f.len == twilink::counters_size) {
            c = twilink::get_counters(f.data);
            return true;
        }
        settle_ms(60);
    }
    return false;
}

bool peer_act(Op op, const twilink::Params& a) {
    uint8_t p[twilink::params_size];
    twilink::put_params(p, a);
    return command(op, p, twilink::params_size);
}

/// Every wire letter asks first: the host brought up on the link, the
/// peer pinged. A silent peer is a failing verdict naming it.
bool need_peer() {
    host_ready();
    command_speed = link_speed;
    link_quiet = true;
    bool up = false;
    for (uint8_t k = 0; k < 2u && !up; ++k) {
        up = command(Op::ping);
    }
    link_quiet = false;
    if (!up) {
        bench.verdict("the peer answers on the command address 0x6B (the partner running twi_peer)", false);
    }
    return up;
}

twilink::Params serve_params(uint16_t ms, uint8_t seed) {
    twilink::Params a{};
    a.ms = ms;
    a.addr = twilink::dut_addr;
    a.seed = seed;
    a.pattern = twilink::pattern_counting;
    return a;
}

/// An action's deadline waited out, with the 100 ms the peer's re-init and
/// one more command round trip need.
void outlast(const twilink::Params& a, uint32_t since_us) {
    const uint32_t due = static_cast<uint32_t>(a.ms) * 1000u + 100'000u;
    const uint32_t spent = us_now() - since_us;
    if (spent < due) {
        spin_us(due - spent);
    }
}

/// The counting pattern's bytes `from`..`from + n - 1` against what came
/// back: the count of mismatches.
uint8_t pattern_mismatches(const uint8_t* got, uint8_t n, uint8_t seed, uint16_t from) {
    uint8_t bad = 0;
    for (uint8_t i = 0; i < n; ++i) {
        if (got[i] != twilink::pattern_value(twilink::pattern_counting, seed, static_cast<uint16_t>(from + i))) {
            ++bad;
        }
    }
    return bad;
}

void fill_pattern(uint8_t* p, uint16_t n, uint8_t seed) {
    for (uint16_t i = 0; i < n; ++i) {
        p[i] = static_cast<uint8_t>(seed + 3u * i);
    }
}
uint16_t sum_of(const uint8_t* p, uint16_t n) {
    uint16_t s = 0;
    for (uint16_t i = 0; i < n; ++i) {
        s = static_cast<uint16_t>(s + p[i]);
    }
    return s;
}

// ---- the hand-made window ------------------------------------------------------------

/// A START CONDITION BY HAND: the pad's output forced low and its enable
/// forced on, the I2C still owning the pad and seeing it.
void inject_start() {
    SdaPin::out_override(PinOverride::low);
    SdaPin::oe_override(PinOeOverride::enable);
}
/// AND THE STOP THAT CLOSES IT: the two overrides back to pass, SDA rising
/// on its pull-up while SCL is high.
void inject_stop() {
    SdaPin::oe_override(PinOeOverride::pass);
    SdaPin::out_override(PinOverride::pass);
}

// =============================================================================
// b - the scan
// =============================================================================
void tb_scan() {
    host_ready();
    uint8_t found = 0;
    uint8_t first = 0;
    uint8_t nacks = 0;
    uint8_t others = 0;
    uint32_t nobody_us = 0;
    for (uint8_t a = 0x08; a <= 0x77u; ++a) {
        const uint32_t t0 = us_now();
        const uint8_t st = tenure<Host>(a, nullptr, 0, nullptr, 0, link_speed);
        const uint32_t took = us_now() - t0;
        if (st == i2c_ok) {
            if (found == 0u) {
                first = a;
            }
            ++found;
            print(serial, "  ", hex(a), " answers (", took, " us)", a == twilink::command_addr ? "  <- twi_peer's command address" : "", crlf);
        } else if (st == i2c_nack_addr) {
            ++nacks;
            if (a == nobody) {
                nobody_us = took;
            }
        } else {
            ++others;
        }
    }
    print(serial, "  112 addresses probed at 100 kHz: ", found, " answered, ", nacks, " nobody-home, ", others,
          " other; nobody home at ", hex(nobody), " took ", nobody_us, " us", crlf);
    bench.verdict("every probe ended one way or the other (no stall, no other status)", others == 0u);
    bench.verdict("the peer's command address answered, and it alone - deaf by construction but for 0x6B",
                  found == 1u && first == twilink::command_addr);
    // A refused address ends at its NACK: 9.4 SCL periods at 100 kHz (the
    // address byte, its acknowledge, the START's 0.4) = 94 us on the wire,
    // and the wire is never faster than the counts. Above it: the disable,
    // IC_TAR and enable a new address costs (at most 957 cycles at 100 kHz on
    // the RP2040, 1836 on the RP2350, docs/rp2040/i2c.md and docs/rp2350/i2c.md),
    // start(), the one handler and the wait's own loop (under 2000 cycles),
    // the rise at 100 kHz (under 1 us over 9.4 periods) and the ruler's
    // microsecond. At 125 MHz 94 + 7.7 + 16 + 1 + 1 = 119.7; at 150 MHz
    // 94 + 12.3 + 13.4 + 1 + 1 = 121.7 - 94 to 122 us on either chip.
    bench.verdict("nobody home is i2c_nack_addr, answered at the address's NACK: 94 to 122 us at 100 kHz",
                  nacks == 111u && nobody_us >= 94u && nobody_us <= 122u);
}

// =============================================================================
// c - the peer
// =============================================================================
void tc_peer() {
    if (!need_peer()) {
        return;
    }
    bench.verdict("the peer answers a ping - every command is TWO TENURES of the engine under test", true);
    twilink::Frame f;
    const bool got = query(Op::ident, f) && f.op == Op::ident_data && f.len == twilink::ident_size;
    if (got) {
        const auto id = twilink::get_ident(f.data);
        char label[9] = {};
        for (uint8_t i = 0; i < 8u; ++i) {
            label[i] = id.label[i];
        }
        print(serial, "  peer: label '", label, "' xtal=", id.xtal, " sanity=", hex(id.sanity), " fw=", hex(id.version),
              crlf);
        bench.verdict("ident comes back and it IS twi_peer on the desk partner (the sanity byte, its kind)",
                      id.sanity == twilink::ident_sanity && twilink::peer_kind(id.version) == partner_kind);
        bench.verdict("... running on its crystal (Ident.xtal = 1: the PLL off the 12 MHz XOSC)", id.xtal == 1u);
    } else {
        bench.verdict("ident comes back", false);
    }
    uint8_t good = 0;
    for (uint8_t s = 0; s < i2c_speed_count; ++s) {
        command_speed = static_cast<I2cSpeed>(s);
        for (uint8_t i = 0; i < 10u; ++i) {
            if (command(Op::ping)) {
                ++good;
            }
        }
    }
    command_speed = link_speed;
    print(serial, "  ", good, " of 30 command round trips answered, ten at each of 100 kHz, 400 kHz and 1 MHz", crlf);
    bench.verdict("the channel is steady over ten round trips at each of the three speeds - the peer's client "
                  "configured for the fastest",
                  good == 30u);
}

// =============================================================================
// d - the tenure shapes
// =============================================================================
void td_shapes() {
    if (!need_peer()) {
        return;
    }
    const twilink::Params a = serve_params(400, 0x30);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    const uint32_t s0 = us_now();
    uint16_t served = 0;   // the counting pattern's next index
    uint8_t w8[8];
    fill_pattern(w8, 8, 0x11);
    const uint8_t ws = tenure<Host>(twilink::dut_addr, w8, 8, nullptr, 0);
    for (uint8_t i = 0; i < 8u; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t rs = tenure<Host>(twilink::dut_addr, nullptr, 0, rx_buf, 8);
    const uint8_t rmism = pattern_mismatches(rx_buf, 8, a.seed, served);
    served = static_cast<uint16_t>(served + 8u);
    uint8_t c4[4] = {0xA0, 0xA1, 0xA2, 0xA3};
    for (uint8_t i = 0; i < 4u; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t cs = tenure<Host>(twilink::dut_addr, c4, 4, rx_buf, 4);
    const uint8_t cmism = pattern_mismatches(rx_buf, 4, a.seed, served);
    served = static_cast<uint16_t>(served + 4u);
    const uint8_t ps = tenure<Host>(twilink::dut_addr, nullptr, 0, nullptr, 0);
    served = static_cast<uint16_t>(served + 1u);   // the probe's one byte, read and dropped
    // Reads that cross the peer's refill: it gives a FIFO ahead (sixteen) at
    // each read request, so a read of 17 and one of 40 take one and two
    // refills, and a read of 1 drops fifteen.
    uint8_t long_mism = 0;
    bool long_ok = true;
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}, uint8_t{17}, uint8_t{40}}) {
        for (uint8_t i = 0; i < n; ++i) {
            rx_buf[i] = 0xEE;
        }
        long_ok = tenure<Host>(twilink::dut_addr, nullptr, 0, rx_buf, n) == i2c_ok && long_ok;
        long_mism = static_cast<uint8_t>(long_mism + pattern_mismatches(rx_buf, n, a.seed, served));
        served = static_cast<uint16_t>(served + n);
    }
    fill_pattern(tx_buf, 200, 0x40);
    isr_entries = 0;
    const uint8_t w200 = tenure<Host>(twilink::dut_addr, tx_buf, 200, nullptr, 0);
    const uint32_t w200_isr = isr_entries;
    outlast(a, s0);
    twilink::Report r{};
    const bool rep = peer_report(r);
    twilink::Counters k{};
    const bool kc = peer_counters(k);
    const uint16_t want_sum = static_cast<uint16_t>(sum_of(w8, 8) + sum_of(c4, 4) + sum_of(tx_buf, 200));
    print(serial, "  write 8 ", ws, ", read 8 ", rs, " (", rmism, " wrong), write 4 then read 4 ", cs, " (", cmism,
          " wrong), probe ", ps, ", reads of 1/2/17/40 ", long_ok ? "ok" : "FAILED", " (", long_mism,
          " wrong), write 200 ", w200, " in ", w200_isr, " host interrupts", crlf);
    print(serial, "  peer: count ", r.count, " (received ", r.aux0, ", served ", r.aux1, "), sum ", hex(r.sum), " (",
          hex(want_sum), "), first ", hex(r.first), ", tenures ", r.addr_hits, ", stops ", r.stops, "; dropped ",
          k.drops, kc ? "" : " (NO counters)", crlf);
    bench.verdict("a write, a read, a write-then-read and the probe all complete i2c_ok against the second chip",
                  ws == i2c_ok && rs == i2c_ok && cs == i2c_ok && ps == i2c_ok);
    bench.verdict("the bytes read are the peer's counting pattern, byte-exact, the write-then-read's included",
                  rmism == 0u && cmism == 0u);
    bench.verdict("reads of 1, 2, 17 and 40 continue the pattern exactly: the peer gives a FIFO ahead and takes "
                  "back what the host's NACK flushed",
                  long_ok && long_mism == 0u);
    // The pump refills at half the transmit FIFO (tx_level 8): 200 entries
    // past the first sixteen are ceil(184 / 8) = 23 TX_EMPTY entries, and
    // the STOP one more - 24; two spare for a refill that lands a byte late.
    bench.verdict("a 200-byte write in ONE tenure through the pump: i2c_ok in at most 26 interrupts (24 the "
                  "arithmetic)",
                  w200 == i2c_ok && w200_isr <= 26u);
    bench.verdict("the peer accounts every byte: 212 received with their sum, the served ones as read",
                  rep && r.aux0 == 212u && r.sum == want_sum && r.first == w8[0] &&
                      r.aux1 == static_cast<uint8_t>(served));
    // Tenures as the peer counts them, each at its STOP or its repeated
    // START: write, read, write-then-read (two), probe, four reads, write.
    bench.verdict("... and the tenures: ten, the write-then-read's repeated START counted from the far end",
                  rep && r.addr_hits == 10u && r.stops == 9u);
}

// =============================================================================
// e - the vocabulary on the wire
// =============================================================================
void te_vocabulary() {
    if (!need_peer()) {
        return;
    }
    twilink::Params d = serve_params(300, 0);
    d.flags = twilink::flag_deaf;
    uint8_t deaf_w = 0xFF, deaf_p = 0xFF, deaf_r = 0xFF;
    if (peer_act(Op::serve, d)) {
        const uint32_t s0 = us_now();
        uint8_t w4[4] = {1, 2, 3, 4};
        deaf_w = tenure<Host>(twilink::dut_addr, w4, 4, nullptr, 0);
        deaf_p = tenure<Host>(twilink::dut_addr, nullptr, 0, nullptr, 0);
        deaf_r = tenure<Host>(twilink::dut_addr, nullptr, 0, rx_buf, 4);
        outlast(d, s0);
    }
    print(serial, "  the peer deaf at ", hex(twilink::deaf_addr), ": write ", deaf_w, ", probe ", deaf_p, ", read ",
          deaf_r, crlf);
    bench.verdict("an address nobody answers is i2c_nack_addr on a write, the probe and a read - on a board "
                  "that is otherwise alive",
                  deaf_w == i2c_nack_addr && deaf_p == i2c_nack_addr && deaf_r == i2c_nack_addr);

    // THE ADDRESSES NO REQUEST MAY NAME (util/i2c_bus.hpp): 0x7A, a 10-bit
    // prefix, and 0x00, the general call, each refused by start() itself -
    // true from start(), i2c_rejected, no entry written. The peer sits at
    // dut_addr ANSWERING THE GENERAL CALL (counted at its GEN_CALL), so a
    // byte of either on the wire would be an address match on its count;
    // the write to dut_addr after them is its one tenure and its two bytes.
    {
        twilink::Params g = serve_params(300, 0);
        g.flags = twilink::flag_general_call;
        if (!peer_act(Op::serve, g)) {
            bench.verdict("the peer accepted the general-call serve", false);
            return;
        }
        const uint32_t g0 = us_now();
        uint8_t gw[2] = {0x5A, 0xA5};
        Host::Request bad{};
        bad.addr = 0x7A;
        bad.tx = lend<Lease::reply>(static_cast<const uint8_t*>(gw));
        bad.tx_len = 2;
        bad.speed = link_speed;
        const bool sync_prefix = Host::start(bad);
        const uint8_t st_prefix = Host::status();
        bad.addr = 0x00;
        const bool sync_call = Host::start(bad);
        const uint8_t st_call = Host::status();
        // `dma` on this host, which has no engines bound, at the peer's
        // own address: refused the same way, nothing on the wire.
        bad.addr = twilink::dut_addr;
        bad.engine = I2cEngine::dma;
        const bool sync_dma = Host::start(bad);
        const uint8_t st_dma = Host::status();
        const uint8_t own = tenure<Host>(twilink::dut_addr, gw, 2, nullptr, 0);
        outlast(g, g0);
        twilink::Report gr{};
        const bool grep = peer_report(gr);
        print(serial, "  0x7A: ", sync_prefix ? "refused at start()" : "STARTED", " ", st_prefix, "; 0x00: ",
              sync_call ? "refused at start()" : "STARTED", " ", st_call, "; dma with no engines: ",
              sync_dma ? "refused at start()" : "STARTED", " ", st_dma, "; then dut_addr ", own,
              "; the peer: received ", gr.aux0, ", tenures ", gr.addr_hits, ", last address ", hex(gr.last_addr),
              crlf);
        bench.verdict("0x7A and the general call answered i2c_rejected inside start(), nothing on the wire: the "
                      "peer, answering the general call, took only the write to its own address",
                      sync_prefix && st_prefix == i2c_rejected && sync_call && st_call == i2c_rejected &&
                          own == i2c_ok && grep && gr.aux0 == 2u && gr.addr_hits == 1u &&
                          gr.last_addr == twilink::dut_addr);
        bench.verdict("dma on a host with no engines bound is answered i2c_rejected inside start(), nothing on the "
                      "wire (the peer's one tenure is the write after it), and that next request is exact",
                      sync_dma && st_dma == i2c_rejected && own == i2c_ok && grep && gr.aux0 == 2u &&
                          gr.addr_hits == 1u);
    }

    // The refusals: at the first data byte (IC_SLV_DATA_NACK_ONLY set before
    // the action) and at the third (a disable once the second is taken).
    for (const uint8_t at : {uint8_t{1}, uint8_t{3}}) {
        twilink::Params n = serve_params(300, 0);
        n.nack_at = at;
        if (!peer_act(Op::serve, n)) {
            bench.verdict("the peer accepted the nack-at serve", false);
            return;
        }
        const uint32_t s0 = us_now();
        uint8_t w6[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
        const uint8_t st = tenure<Host>(twilink::dut_addr, w6, 6, nullptr, 0);
        outlast(n, s0);
        twilink::Report r{};
        const bool rep = peer_report(r);
        print(serial, "  the peer refusing data byte ", at, ": status ", st, ", the peer stored ", r.aux0,
              " byte(s), flags ", hex(r.flags), crlf);
        bench.verdict(at == 1u ? "a client refusing from the FIRST data byte (IC_SLV_DATA_NACK_ONLY): i2c_nack_data, "
                                 "nothing stored"
                               : "a client refusing the THIRD data byte (its block disabled once the second was "
                                 "taken, which refuses the byte in flight): i2c_nack_data, two stored",
                      st == i2c_nack_data && rep && r.aux0 == at - 1u && (r.flags & twilink::report_nacked) != 0u);
    }

    // The stretch priced: an 8-byte read and an 8-byte write, plain, then
    // with the peer holding 100 us before every byte it gives.
    uint32_t plain_r = 0, plain_w = 0, held_r = 0, held_w = 0;
    uint8_t st_ok = 0;
    for (const uint16_t hold : {uint16_t{0}, uint16_t{100}}) {
        twilink::Params h = serve_params(300, 0x50);
        h.hold_us = hold;
        if (!peer_act(Op::serve, h)) {
            bench.verdict("the peer accepted the stretched serve", false);
            return;
        }
        const uint32_t s0 = us_now();
        uint8_t w8[8] = {1, 2, 3, 4, 5, 6, 7, 8};
        uint32_t t0 = us_now();
        st_ok = static_cast<uint8_t>(st_ok + (tenure<Host>(twilink::dut_addr, nullptr, 0, rx_buf, 8) == i2c_ok ? 1u : 0u));
        const uint32_t r_us = us_now() - t0;
        const uint8_t mism = pattern_mismatches(rx_buf, 8, h.seed, 0);
        t0 = us_now();
        st_ok = static_cast<uint8_t>(st_ok + (tenure<Host>(twilink::dut_addr, w8, 8, nullptr, 0) == i2c_ok ? 1u : 0u));
        const uint32_t w_us = us_now() - t0;
        st_ok = static_cast<uint8_t>(st_ok + (mism == 0u ? 1u : 0u));
        if (hold == 0u) {
            plain_r = r_us;
            plain_w = w_us;
        } else {
            held_r = r_us;
            held_w = w_us;
        }
        outlast(h, s0);
    }
    print(serial, "  an 8-byte read: ", plain_r, " us plain, ", held_r, " us held 100 us a byte; an 8-byte write: ",
          plain_w, " us plain, ", held_w, " us under the same serve", crlf);
    // Eight holds of 100 us on the peer's system timer - 99 to 101 us each,
    // a spin that ends when two reads a microsecond apart differ by 100 -
    // plus the loop's turn at each read request (under 2 us), but each hold
    // OVERLAPS the host's own SCL low phase, which it counts from the same
    // falling edge - LCNT + 1, 6.0 us at 100 kHz on either clock (letter a's
    // split): 8 x (99 - 6) = 744 to 8 x (101 + 2 - 6) = 776 us longer, the
    // ruler's microsecond at both ends of both reads around it - 740 to 790.
    bench.verdict("the client's clock stretch is priced on the wire: 100 us before each of eight bytes given "
                  "lengthens the read by 740 to 790 us (8 x 100, less the host's own 6 us low phase each hold "
                  "overlaps)",
                  st_ok == 6u && held_r >= plain_r + 740u && held_r <= plain_r + 790u);
    // A DW client holds SCL as a TRANSMITTER only (RD_REQ); a receiver is
    // held only by a full sixteen-deep FIFO, which eight bytes never fill:
    // the write's wire time is the same, the ruler's microsecond and the
    // handler's jitter (a few microseconds) its only difference.
    bench.verdict("... and a WRITE is not stretched by the same serve: the DW client holds SCL at a read "
                  "request only (within 10 us of the plain write)",
                  held_w + 10u >= plain_w && held_w <= plain_w + 10u);
}

// =============================================================================
// f - the three speeds, measured
// =============================================================================
void tf_speeds() {
    if (!need_peer()) {
        return;
    }
    uint8_t exact = 0;
    uint8_t in_bracket = 0;
    uint8_t accounted = 0;
    for (uint8_t s = 0; s < i2c_speed_count; ++s) {
        const auto sp = static_cast<I2cSpeed>(s);
        const twilink::Params a = serve_params(300, static_cast<uint8_t>(0x60u + s));
        if (!peer_act(Op::serve, a)) {
            bench.verdict("the peer accepted the serve command", false);
            return;
        }
        const uint32_t s0 = us_now();
        fill_pattern(tx_buf, 16, static_cast<uint8_t>(0x70u + s));
        const uint8_t w = tenure<Host>(twilink::dut_addr, tx_buf, 16, nullptr, 0, sp);
        const uint16_t w_sum = sum_of(tx_buf, 16);
        for (uint8_t i = 0; i < 16u; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t r = tenure<Host>(twilink::dut_addr, nullptr, 0, rx_buf, 16, sp);
        const bool r_ok = r == i2c_ok && pattern_mismatches(rx_buf, 16, a.seed, 0) == 0u;
        // The rate: a 64-byte write is 9 x 65 SCL periods, plus START and
        // STOP (1.4) and the software around them, which only lengthen it.
        fill_pattern(tx_buf, 64, 0x07);
        const uint32_t t0 = us_now();
        const uint8_t m = tenure<Host>(twilink::dut_addr, tx_buf, 64, nullptr, 0, sp);
        const uint32_t took = us_now() - t0;
        const uint16_t m_sum = sum_of(tx_buf, 64);
        const uint32_t scl_hz = took == 0u ? 0u : static_cast<uint32_t>(9ULL * 65u * 1'000'000u / took);
        const uint32_t asked = i2c_speed_hz(sp);
        // THE BRACKET, derived: never above the asked rate by more than the
        // ruler's microsecond over the shortest tenure (585 us at 1 MHz:
        // 0.2 %) and the two crystals' 100 ppm - 2 % is that with room; below
        // it by the rise of 1.5 kOhm into the wire's ~60 pF to the Schmitt
        // trigger's threshold (0.92 tau = 83 ns a period, 8 % at 1 MHz), the
        // START, STOP and software around the tenure (1.4 periods and some
        // 10 us: 2 % at 1 MHz) - 15 % is that with room.
        const bool w_in = m == i2c_ok && scl_hz <= asked + asked / 50u && scl_hz >= asked - (asked * 15u) / 100u;
        // And a 64-byte read, the peer giving a FIFO ahead: its four refills
        // are the far end's only pace.
        const uint32_t t1 = us_now();
        const uint8_t mr = tenure<Host>(twilink::dut_addr, nullptr, 0, rx_buf, 64, sp);
        const uint32_t took_r = us_now() - t1;
        const bool mr_ok = mr == i2c_ok && pattern_mismatches(rx_buf, 64, a.seed, 16) == 0u;
        const uint32_t scl_r = took_r == 0u ? 0u : static_cast<uint32_t>(9ULL * 65u * 1'000'000u / took_r);
        const bool r_in = mr_ok && scl_r <= asked + asked / 50u && scl_r >= asked - (asked * 15u) / 100u;
        const bool bracket = w_in && r_in;
        outlast(a, s0);
        twilink::Report rep{};
        const bool got = peer_report(rep);
        const bool acct = got && rep.aux0 == 80u && rep.sum == static_cast<uint16_t>(w_sum + m_sum);
        print(serial, "  ", asked / 1000u, " kHz: write ", w == i2c_ok ? "ok" : "FAILED", ", read ",
              r_ok ? "exact" : "WRONG", "; 64 bytes read in ", took_r, " us -> SCL ", scl_r / 1000u, " kHz",
              mr_ok ? "" : " (WRONG)", "; 64 written in ", took, " us -> SCL ", scl_hz / 1000u, " kHz (",
              Host::scl_hz(sp) / 1000u, " on an ideal wire)", bracket ? "" : "  OUT OF BRACKET", "; the peer took ",
              rep.aux0, " bytes", acct ? "" : "  NOT ACCOUNTED", crlf);
        if (w == i2c_ok && r_ok) {
            ++exact;
        }
        if (bracket) {
            ++in_bracket;
        }
        if (acct) {
            ++accounted;
        }
    }
    bench.verdict("the three speeds, 100 kHz, 400 kHz and 1 MHz: sixteen bytes byte-exact both ways at each "
                  "against the second chip",
                  exact == 3u && accounted == 3u);
    bench.verdict("the SCL rate measured on the ruler, a 64-byte write and a 64-byte read, is never more than 2 % "
                  "above the asked rate and within 15 % below it (the pull-ups' rise and the tenure's edges, derived "
                  "beside the bracket)",
                  in_bracket == 3u);
}

// =============================================================================
// r - a released requester, the same owner again
// =============================================================================
//
// The release contract (docs/design/dma.md): this instance owns its two channels
// for the whole image, so what its block holds across a release can reach only this
// instance, initialized again. The staging is the worst release: IC_DMA_CR's two
// enables set, both channels stopped, the transmit FIFO empty and so its request
// standing, and the block taken down with the enables left set.
//
//   contract  the block held in reset, this chip's gate, then the host's own
//             init(): a channel bound on the transmit request, pouring into a cell
//             of RAM and not into the block, moves nothing in 20 us, and after
//             init() again one read of 16 bytes on the engines is exact;
//   control   no reset: the block disabled and enabled again, the enables left
//             set, the same channel on the same cell moves an item, the request
//             standing and asking once (the block raises the next when the FIFO has
//             taken this one, and a cell of RAM gives it nothing). That is the request
//             a reset at init() withdraws. The cell keeps it off the bus: nothing of
//             the control's reaches the wire.
//
// Bounds: the 20 us are 2500 cycles of the channel's turn at the slowest core, which
// a standing request needs a dozen of; a read of 16 at 400 kHz takes about 400 us on a
// 20 000 us limit. Four rounds.
alignas(4) uint16_t released_cells[4] = {0x1111u, 0x2222u, 0x3333u, 0x4444u};
volatile uint16_t released_sink = 0;

/// What a channel on the transmit request moves, into a cell of RAM, in 20 us.
uint8_t released_request_moves() {
    DmaTx::arm(&released_sink, Raw::dreq_tx, false, DmaReport::errors);
    (void)DmaTx::start(released_cells, 4u);
    spin_us(20);
    const uint8_t moved = static_cast<uint8_t>(4u - DmaChannel<DmaTx::channel>::count());
    DmaTx::stop();
    return moved;
}

void released_stage() {
    (void)DmaHost::init(clock);
    owner = Owner::none;
    DmaTx::stop();
    DmaRx::stop();
    Raw::dma_requests(true, true);
    spin_us(5);
}

void tr_released_requester() {
    if (!need_peer()) {
        return;
    }
    const twilink::Params a = serve_params(400, 0x2D);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    const uint32_t s0 = us_now();
    owner = Owner::none;
    Host::release();
    uint8_t after_reset = 0;
    uint8_t exact = 0;
    uint8_t held = 0;
    uint16_t served = 0;
    for (uint8_t i = 0; i < 4u; ++i) {
        released_stage();
        Raw::hold();
        (void)DmaHost::init(clock);
        const uint8_t moved_reset = released_request_moves();
        (void)DmaHost::init(clock);
        owner = Owner::dma_host;
        for (uint8_t k = 0; k < 16u; ++k) {
            rx_buf[k] = 0xEE;
        }
        const uint8_t st = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 16, I2cSpeed::fast_400k, 20'000u,
                                           I2cEngine::dma);
        const uint8_t wrong = pattern_mismatches(rx_buf, 16, a.seed, served);
        served = static_cast<uint16_t>(served + 16u);
        owner = Owner::none;
        released_stage();
        (void)Raw::disable();
        Raw::enable();
        const uint8_t moved_held = released_request_moves();
        Raw::dma_requests(false, false);
        print(serial, "  the contract: the request's channel moved ", moved_reset, " of 4, then 16 bytes on the engines "
              "status ", st, ", ", wrong, " wrong; the control, no reset: it moved ", moved_held, " of 4", crlf);
        after_reset = static_cast<uint8_t>(after_reset + (moved_reset == 0u ? 1u : 0u));
        exact = static_cast<uint8_t>(exact + (st == i2c_ok && wrong == 0u ? 1u : 0u));
        held = static_cast<uint8_t>(held + (moved_held >= 1u ? 1u : 0u));
    }
    DmaHost::release();
    host_ready();
    outlast(a, s0);
    twilink::Report r{};
    const bool rep = peer_report(r);
    bench.verdict("THE RESET AT init(): the host released with both DMA enables set, the gate closed, initialized again - "
                  "no request stands on its channel, four of four, and its engined read is exact, four of four",
                  after_reset == 4u && exact == 4u && rep);
    bench.verdict("THE CONTROL, no reset: the block disabled and enabled with the enables left set, the request stands and "
                  "the channel moves an item at once, four of four - the held request the reset withdraws",
                  held == 4u);
}

// =============================================================================
// g - the DMA engines
// =============================================================================
//
// KEPT APART FROM EVERY BUS VERDICT ABOVE: a fault in a channel cannot mask
// one of them.
void tg_dma() {
    if (!need_peer()) {
        return;
    }
    const twilink::Params a = serve_params(600, 0x21);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    const uint32_t s0 = us_now();
    owner = Owner::none;
    Host::release();
    (void)DmaHost::init(clock);
    owner = Owner::dma_host;
    uint16_t served = 0;
    // THE FIRST ENGINED READ AFTER OTHER CODE HAS RUN is printed and not
    // judged: it fetches the host's start(), its vector's and the DMA
    // line's code through the XIP cache, which the code that ran before
    // has evicted, and runs slower than the next - the same read with
    // every line of the cache dropped first, printed third, shows the same
    // cost (docs/rp2040/platform.md, "A handler in SRAM"). The judged read
    // is the second.
    for (uint8_t i = 0; i < 64u; ++i) {
        rx_buf[i] = 0xEE;
    }
    constexpr I2cEngine dma = I2cEngine::dma;
    uint32_t t0 = us_now();
    const uint8_t st0 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 64, I2cSpeed::fast_400k, 20'000u, dma);
    const uint32_t took0 = us_now() - t0;
    const uint32_t entries0 = isr_entries;
    const uint8_t m0 = pattern_mismatches(rx_buf, 64, a.seed, served);
    served = static_cast<uint16_t>(served + 64u);
    print(serial, "  the first 64-byte engined read: status ", st0, " in ", took0, " us, ", entries0,
          " host interrupts, ", m0, " wrong", crlf);
    for (uint8_t i = 0; i < 64u; ++i) {
        rx_buf[i] = 0xEE;
    }
    t0 = us_now();
    const uint8_t st = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 64, I2cSpeed::fast_400k, 20'000u, dma);
    const uint32_t took = us_now() - t0;
    const uint32_t entries = isr_entries;
    const uint8_t m64 = pattern_mismatches(rx_buf, 64, a.seed, served);
    served = static_cast<uint16_t>(served + 64u);
    const uint32_t engined_scl = took == 0u ? 0u : static_cast<uint32_t>(9ULL * 65u * 1'000'000u / took);
    print(serial, "  64 bytes read on the engines at 400 kHz: status ", st, " in ", took, " us (SCL ",
          engined_scl / 1000u, " kHz), ", entries, " host interrupts, ", m64, " wrong", crlf);
    // The host's own vector: the STOP, and TX_EMPTY when the transmit
    // block's end found no room for the STOP entry (dma_isr() arms it) -
    // one or two; the block's end itself is the DMA line's.
    // THE SAME READ COLD: the XIP cache emptied right before it.
    for (uint8_t i = 0; i < 64u; ++i) {
        rx_buf[i] = 0xEE;
    }
    xip_empty();
    t0 = us_now();
    const uint8_t stc = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 64, I2cSpeed::fast_400k, 20'000u, dma);
    const uint32_t tookc = us_now() - t0;
    const uint32_t entriesc = isr_entries;
    const uint8_t mc = pattern_mismatches(rx_buf, 64, a.seed, served);
    served = static_cast<uint16_t>(served + 64u);
    print(serial, "  the same read with the XIP cache emptied first: status ", stc, " in ", tookc, " us, ",
          entriesc, " host interrupts, ", mc, " wrong", crlf);
    bench.verdict("a 64-byte read on the two engines: the commands poured from a fixed cell, the bytes "
                  "collected, byte-exact, the host's own vector entered once or twice (the STOP, and TX_EMPTY "
                  "for the STOP entry when the block's end found no room)",
                  st == i2c_ok && m0 == 0u && st0 == i2c_ok && m64 == 0u && entries >= 1u && entries <= 2u);
    bench.verdict("... at the rate asked: letter f's bracket, 2 % above to 15 % below 400 kHz",
                  st == i2c_ok && engined_scl <= 408'000u && engined_scl >= 340'000u);
    uint8_t w2[2] = {0x01, 0x02};
    for (uint8_t i = 0; i < 32u; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t st2 = tenure<DmaHost>(twilink::dut_addr, w2, 2, rx_buf, 32, I2cSpeed::fast_400k, 200'000u, dma);
    const uint8_t m32 = pattern_mismatches(rx_buf, 32, a.seed, served);
    served = static_cast<uint16_t>(served + 32u);
    bench.verdict("a write-then-read on an engined host: the write on the pump, the repeated START, 32 bytes "
                  "back on the engines, exact",
                  st2 == i2c_ok && m32 == 0u);
    // THE SHORTEST READS ON THE ENGINES: one byte and two, `dma` named. A
    // read of one or two has no command between the first and the last,
    // so the pump writes both and the transmit engine is not claimed; the
    // receive engine's run is the length and every byte of it moved
    // (take() counts what its channel moved since the start).
    bool short_ok = true;
    for (const uint8_t n : {uint8_t{1}, uint8_t{2}}) {
        for (uint8_t i = 0; i < n; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t sn = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, n, I2cSpeed::fast_400k, 200'000u, dma);
        const uint32_t cap = DmaRx::capacity();
        const uint32_t moved = DmaRx::take();
        const bool tx_free = !DmaTx::busy();
        const uint32_t lines = dma_entries;
        const uint8_t mn = pattern_mismatches(rx_buf, n, a.seed, served);
        served = static_cast<uint16_t>(served + n);
        print(serial, "  a ", n, "-byte read under dma: status ", sn, ", the receive run ", cap, " (", moved,
              " moved), the transmit engine ", tx_free ? "free" : "CLAIMED", ", ", lines, " DMA line entries, ",
              isr_entries, " host entries, ", mn, " wrong", crlf);
        short_ok = short_ok && sn == i2c_ok && mn == 0u && cap == n && moved == n && tx_free && lines == 0u;
    }
    bench.verdict("dma at the shortest read: one byte and two on the receive engine, its run the length and every "
                  "byte of it moved, the transmit engine not claimed (no command between the first and the last), "
                  "no DMA line entry, byte-exact",
                  short_ok);
    // A HALF THE ENGINES CANNOT CARRY, UNDER dma: a write. Its entries are
    // the request's bytes, and a byte beat lands replicated across the
    // command register (the chip's document): the write stays on the
    // pump and completes - no DMA line entry, the receive engine's run
    // untouched, the bytes counted by the peer at the letter's end. On a
    // chip whose transmit engine carries a pure write (the RP2350's zero-filling
    // alias) the one byte is still the pump's and the 32 the channel's: one
    // DMA line entry, the STOP and at most a TX_EMPTY.
    const uint32_t cap_before = DmaRx::capacity();
    uint8_t w1[1] = {0x5C};
    const uint8_t sw1 = tenure<DmaHost>(twilink::dut_addr, w1, 1, nullptr, 0, I2cSpeed::fast_400k, 200'000u, dma);
    const uint32_t lines_w1 = dma_entries;
    fill_pattern(tx_buf, 32, 0x90);
    const uint8_t sw32 = tenure<DmaHost>(twilink::dut_addr, tx_buf, 32, nullptr, 0, I2cSpeed::fast_400k, 200'000u, dma);
    const uint32_t lines_w32 = dma_entries;
    const uint32_t isr_w32 = isr_entries;
    const uint16_t written_sum = static_cast<uint16_t>(0x01u + 0x02u + 0x5Cu + sum_of(tx_buf, 32));
    print(serial, "  writes of 1 and 32 under dma: status ", sw1, " and ", sw32, ", DMA line entries ", lines_w1,
          " and ", lines_w32, ", the 32-byte write's host entries ", isr_w32, crlf);
    // The 32-byte write on the pump: sixteen entries at start(), refills of
    // at least eight on TX_EMPTY (half the FIFO): two, then the STOP - three
    // host entries, four for a refill that lands a byte late.
    if constexpr (DmaHost::dma_carries_writes) {
        bench.verdict("writes under dma: the 1 byte on the interrupt, no DMA line entry, the 32 on the transmit engine, "
                      "one DMA line entry and one or two host entries, both i2c_ok, the receive run untouched (the "
                      "peer's count and sum judged at the end)",
                      sw1 == i2c_ok && sw32 == i2c_ok && lines_w1 == 0u && lines_w32 == 1u &&
                          DmaRx::capacity() == cap_before && isr_w32 >= 1u && isr_w32 <= 2u);
    } else {
        bench.verdict("a half the engines cannot carry, under dma, completes on the interrupt: writes of 1 and 32 "
                      "bytes i2c_ok, no DMA line entry, the receive run untouched, the 32 in three or four host entries "
                      "(the peer's count and sum judged at the end)",
                      sw1 == i2c_ok && sw32 == i2c_ok && lines_w1 == 0u && lines_w32 == 0u &&
                          DmaRx::capacity() == cap_before && isr_w32 >= 3u && isr_w32 <= 4u);
    }
    const uint8_t st4 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, nullptr, 0, I2cSpeed::fast_400k, 200'000u, dma);
    const uint32_t lines_probe = dma_entries;
    served = static_cast<uint16_t>(served + 1u);
    const uint8_t st5 = tenure<DmaHost>(nobody, nullptr, 0, rx_buf, 16, I2cSpeed::fast_400k, 200'000u, dma);
    bench.verdict("the probe under dma stays on the pump (no read half: no DMA line entry); a deaf client on an "
                  "engined read is i2c_nack_addr with the engines put away",
                  st4 == i2c_ok && lines_probe == 0u && st5 == i2c_nack_addr);
    const uint8_t st6 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 16, I2cSpeed::fast_400k, 200'000u, dma);
    const uint8_t m16 = pattern_mismatches(rx_buf, 16, a.seed, served);
    served = static_cast<uint16_t>(served + 16u);
    bench.verdict("... and the next engined read after that is clean", st6 == i2c_ok && m16 == 0u);
    // THE FAULT PATH: a 64-byte engined read whose receive run points into
    // a hole in the map (0x40100000, where no peripheral answers) - the
    // receive engine's first write is a bus error, which raises the
    // channel's line whatever its binding, and dma_isr() aborts the tenure
    // (the STOP after the entry under way) and answers i2c_dma_fault. How
    // far the peer got is the abort's: the next clean read's first byte
    // says where its counting pattern stands, and the tally resumes there.
    // The bound: the error comes at the first byte's move; the handler's
    // entry is microseconds warm and at most 8600 cycles cold from the
    // flash on the RP2040 (docs/rp2040/platform.md: 69 us, the slowest of
    // the three images) - under four bytes of 22.5 us at 400 kHz - and the
    // abort ends after the entry under way, the next already in the
    // shifter: one to six bytes served (3 or 4 measured).
    auto* const hole = reinterpret_cast<uint8_t*>(0x40100000u);
    const uint8_t sf = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, hole, 64, I2cSpeed::fast_400k, 20'000u, dma);
    const uint32_t fault_entries = isr_entries;
    for (uint8_t i = 0; i < 16u; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t sa = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 16, I2cSpeed::fast_400k, 200'000u, dma);
    uint16_t skipped = 0xFFFFu;
    for (uint16_t k = 0; k < 256u && skipped == 0xFFFFu; ++k) {
        if (pattern_mismatches(rx_buf, 16, a.seed, static_cast<uint16_t>(served + k)) == 0u) {
            skipped = k;
        }
    }
    print(serial, "  an engined read into a hole in the map: status ", sf, " (i2c_dma_fault ", i2c_dma_fault, "), ",
          fault_entries, " host interrupts; the peer had served ", skipped, " byte(s) by the abort; the next read ",
          sa, crlf);
    bench.verdict("THE ENGINES' FAULT PATH: a receive run into a hole in the map is a bus error on the receive "
                  "channel, the tenure aborted and answered i2c_dma_fault - and the next engined read is clean, the "
                  "peer's pattern resumed where the abort left it",
                  sf == i2c_dma_fault && sa == i2c_ok && skipped >= 1u && skipped <= 6u);
    if (skipped != 0xFFFFu) {
        served = static_cast<uint16_t>(served + skipped + 16u);
    }
    // The bursts: the request is a level the DMA banks credits on while the
    // FIFO is empty; a burst past the FIFO's depth would drop commands
    // (TX_OVER) and shorten the tenure. Eight reads of 255 at 1 MHz.
    uint8_t exact = 0;
    bool over = false;
    for (uint8_t k = 0; k < 8u; ++k) {
        for (uint16_t i = 0; i < 255u; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t s7 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 255, I2cSpeed::fast_plus_1m, 50'000u, dma);
        over = over || (Raw::raw_pending() & I2cInterrupt::tx_over) != 0u;
        if (s7 == i2c_ok && pattern_mismatches(rx_buf, 255, a.seed, served) == 0u) {
            ++exact;
        }
        served = static_cast<uint16_t>(served + 255u);
    }
    print(serial, "  eight 255-byte reads on the engines at 1 MHz: ", exact, " exact, TX_OVER ",
          over ? "RAISED" : "never raised", crlf);
    bench.verdict("the engine's bursts never overrun the sixteen-deep FIFO: eight 255-byte reads at 1 MHz "
                  "exact against the second chip, TX_OVER never raised",
                  exact == 8u && !over);
    // AFTER THE FAULT PATH AND THE BURSTS, so that those run with the
    // engined path's code as warm as their bounds assume.
    // `interrupt`, THE DEFAULT, TOUCHES NO CHANNEL, at a length the engines
    // would carry: no DMA line entry, the receive engine's run and its
    // channel's write address as the last engined read left them. The host's
    // vector takes the 64 bytes: each entry at most the FIFO's sixteen, so
    // four at the least; RX_FULL at eight bytes, seven entries and the
    // tail's at the STOP, so eight at the most - two spare.
    for (uint8_t i = 0; i < 64u; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint32_t cap_i0 = DmaRx::capacity();
    const uint32_t wa_i0 = DmaChannel<DmaRx::channel>::write_addr();
    const uint8_t sti = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 64, I2cSpeed::fast_400k, 20'000u,
                                        I2cEngine::interrupt);
    const uint32_t lines_i = dma_entries;
    const uint32_t isr_i = isr_entries;
    const bool untouched = DmaRx::capacity() == cap_i0 && DmaChannel<DmaRx::channel>::write_addr() == wa_i0 &&
                           !DmaTx::busy();
    const uint8_t mi = pattern_mismatches(rx_buf, 64, a.seed, served);
    served = static_cast<uint16_t>(served + 64u);
    print(serial, "  a 64-byte read under interrupt on the engined host: status ", sti, ", ", isr_i,
          " host entries, ", lines_i, " DMA line entries, the channels ", untouched ? "untouched" : "TOUCHED", ", ",
          mi, " wrong", crlf);
    bench.verdict("interrupt, the default, at 64 bytes touches no channel: no DMA line entry, the receive run and "
                  "its channel's write address unchanged, four to ten host entries, byte-exact",
                  sti == i2c_ok && mi == 0u && lines_i == 0u && untouched && isr_i >= 4u && isr_i <= 10u);
    owner = Owner::none;
    DmaHost::release();
    host_ready();
    outlast(a, s0);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  the peer served ", r.aux1, " (low byte of ", served, "), received ", r.aux0, " with sum ",
          hex(r.sum), " (", hex(written_sum), ")", crlf);
    bench.verdict("the peer's tally agrees: every byte the engines read was served once",
                  rep && r.aux1 == static_cast<uint8_t>(served));
    bench.verdict("... and every byte written under dma arrived once: 35 received (the 2 + 32's two, the 1 and "
                  "the 32 of the writes), their sum exact",
                  rep && r.aux0 == 35u && r.sum == written_sum);

    // THE DATA NACK UNDER dma: a write-then-read of 6 + 16 whose third
    // written byte the client refuses (its block disabled once the second
    // is taken). start() wrote the six and the read's first command, and the
    // engines took the read half before the first byte left: the abort ends
    // the tenure with them launched, puts them away before the FIFOs come
    // out of their flushed state, and the next engined read is exact.
    twilink::Params nk = serve_params(300, 0x5A);
    nk.nack_at = 3;
    if (!peer_act(Op::serve, nk)) {
        bench.verdict("the peer accepted the nack-at serve", false);
        return;
    }
    const uint32_t n0 = us_now();
    owner = Owner::none;
    Host::release();
    (void)DmaHost::init(clock);
    owner = Owner::dma_host;
    uint8_t w6[6] = {0x10, 0x20, 0x30, 0x40, 0x50, 0x60};
    const uint8_t snk = tenure<DmaHost>(twilink::dut_addr, w6, 6, rx_buf, 16, link_speed, 200'000u, dma);
    for (uint8_t i = 0; i < 16u; ++i) {
        rx_buf[i] = 0xEE;
    }
    const uint8_t sn2 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 16, link_speed, 200'000u, dma);
    const uint8_t mn2 = pattern_mismatches(rx_buf, 16, nk.seed, 0);
    owner = Owner::none;
    DmaHost::release();
    host_ready();
    outlast(nk, n0);
    twilink::Report nr{};
    const bool nrep = peer_report(nr);
    print(serial, "  6 + 16 under dma, the third written byte refused: status ", snk, ", the peer stored ", nr.aux0,
          ", served ", nr.aux1, "; the next engined read ", sn2, ", ", mn2, " wrong", crlf);
    bench.verdict("THE DATA NACK UNDER dma: a write-then-read of 6 + 16, the read half on the engines, its third "
                  "written byte refused - i2c_nack_data, two bytes stored, and the next engined read of 16 exact",
                  snk == i2c_nack_data && sn2 == i2c_ok && mn2 == 0u && nrep && nr.aux0 == 2u && nr.aux1 == 16u);
}

// =============================================================================
// h - the kernel over the bus
// =============================================================================
namespace kl {

constexpr uint32_t timeout_ticks = ticks_from_ms<P>(20);
using I2cArb = I2cBus<Host, P, 4, BusPassThrough, timeout_ticks>;

uint8_t out_a[8];
uint8_t in_a[8];

class Probe {
public:
    using Event = std::variant<I2cDone, SleepVote>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies[8];
    static inline uint32_t reply_at[8];
    static inline uint8_t n = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;

    static void init() { clear_tally(); }
    static void clear_tally() {
        n = 0;
        rejected = 0;
        votes = 0;
        last_vote = false;
    }
    static void dispatch(const Event& e) {
        match(e,
              [](const I2cDone& d) {
                  if (n < 8u) {
                      replies[n] = d.status;
                      reply_at[n] = Ticker::ticks();
                  }
                  ++n;
                  if (d.status == i2c_rejected) {
                      ++rejected;
                  }
              },
              [](const SleepVote& v) {
                  ++votes;
                  last_vote = v.ok;
              });
    }
};

using BusKernel = Tenuto<P, Probe, I2cArb>;

void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}
void pump_until(uint8_t want, uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        pump();
        if (Probe::n >= want) {
            break;
        }
    }
    pump();
}

Host::Request request(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = I2cSpeed::fast_400k;
    r.reply = reply_to<Probe, I2cDone>();
    return r;
}

}  // namespace kl

void th_kernel() {
    if (!need_peer()) {
        return;
    }
    const twilink::Params a = serve_params(500, 0x01);
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve command", false);
        return;
    }
    const uint32_t s0 = us_now();
    kl::BusKernel::init_all();
    bus_ao_live = true;
    fill_pattern(kl::out_a, 8, 0x01);
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a, 8, nullptr, 0));
    post<kl::I2cArb>(kl::request(twilink::dut_addr, nullptr, 0, kl::in_a, 8));
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a, 2, kl::in_a, 4));
    post<kl::I2cArb>(kl::request(twilink::dut_addr, nullptr, 0, nullptr, 0));
    kl::pump_until(4, 300);
    print(serial, "  four queued tenures (write, read, write-then-read, probe): replies ", kl::Probe::n, " [",
          kl::Probe::replies[0], " ", kl::Probe::replies[1], " ", kl::Probe::replies[2], " ", kl::Probe::replies[3],
          "]", crlf);
    bench.verdict("four tenures through I2cBus against the second chip, four replies, every one i2c_ok - "
                  "util/i2c_bus.hpp and util/bus_master.hpp unchanged",
                  kl::Probe::n == 4u && kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      kl::Probe::replies[2] == i2c_ok && kl::Probe::replies[3] == i2c_ok);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a, 4, nullptr, 0));
    post<kl::I2cArb>(kl::request(nobody, kl::out_a, 4, nullptr, 0));
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a, 4, nullptr, 0));
    kl::pump_until(3, 300);
    bench.verdict("the NACK of a deaf address is delivered in its place, the tenures around it untouched",
                  kl::Probe::n == 3u && kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_nack_addr &&
                      kl::Probe::replies[2] == i2c_ok);
    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a, 8, nullptr, 0));
    }
    kl::pump_until(6, 300);
    bench.verdict("the arbiter rejects what it cannot queue, immediately, and every request is still answered "
                  "exactly once",
                  kl::Probe::rejected != 0u && kl::Probe::n == 6u);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a, 8, nullptr, 0));
    post<kl::I2cArb>(PrepareSleep{.depth = SleepDepth::standby, .reply = reply_to<kl::Probe, SleepVote>()});
    kl::pump();
    kl::pump_until(2, 100);
    bench.verdict("a BUSY bus votes against it", kl::Probe::votes == 1u && !kl::Probe::last_vote);
    bus_ao_live = false;
    outlast(a, s0);

    // THE TIMED BUS: the peer holds SCL 30 ms before the first byte of a read.
    host_ready();
    twilink::Params h = serve_params(200, 0x30);
    h.hold_us = 30'000;
    if (!peer_act(Op::serve, h)) {
        bench.verdict("the peer accepted the holding serve", false);
        return;
    }
    const uint32_t h0 = us_now();
    kl::BusKernel::init_all();
    bus_ao_live = true;
    kl::Probe::clear_tally();
    const uint32_t t0 = Ticker::ticks();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, nullptr, 0, kl::in_a, 8));
    kl::pump_until(1, 200);
    const uint32_t answered_ms = kl::Probe::n != 0u ? kl::Probe::reply_at[0] - t0 : 0u;
    const bool scl_low = !SclPin::read();
    print(serial, "  a read against a client holding SCL 30 ms: reply ", kl::Probe::n != 0u ? kl::Probe::replies[0] : 0xFFu,
          " after ", answered_ms, " ms, SCL ", scl_low ? "still low" : "released", crlf);
    // The arbiter's 20 ms is twenty ticks of 1 ms counted from the post's
    // own tick: 19 to 21 ms on the ticker, the reply's dispatch within the
    // next tick - 19 to 22.
    bench.verdict("THE TIMED BUS: a client holding the clock is answered i2c_timeout at the arbiter's 20 ms "
                  "(19 to 22 on the 1 ms ticker), SCL still held by the far end",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_timeout && answered_ms >= 19u &&
                      answered_ms <= 22u && scl_low);
    bus_ao_live = false;
    outlast(h, h0);
    // The recover()ed engine carries the next two tenures, the peer back.
    host_ready();
    const twilink::Params b = serve_params(300, 0x44);
    const bool again = peer_act(Op::serve, b);
    const uint32_t b0 = us_now();
    kl::BusKernel::init_all();
    bus_ao_live = true;
    kl::Probe::clear_tally();
    post<kl::I2cArb>(kl::request(twilink::dut_addr, kl::out_a, 8, nullptr, 0));
    post<kl::I2cArb>(kl::request(twilink::dut_addr, nullptr, 0, kl::in_a, 8));
    kl::pump_until(2, 300);
    bench.verdict("... and the recover()ed engine carries the next two tenures clean",
                  again && kl::Probe::n == 2u && kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      pattern_mismatches(kl::in_a, 8, b.seed, 0) == 0u);
    bus_ao_live = false;
    outlast(b, b0);
    host_ready();
}

// =============================================================================
// i - the stuck bus
// =============================================================================
void ti_unstick() {
    if (!need_peer()) {
        return;
    }
    const uint8_t healthy = Host::unstick();
    bench.verdict("unstick() on a healthy bus clocks nothing and says so", healthy == 0u);
    twilink::Params h{};
    h.ms = 300;
    h.aux8 = 4;
    if (!peer_act(Op::hold_sda, h)) {
        bench.verdict("the peer accepted the hold_sda command", false);
        return;
    }
    const uint32_t s0 = us_now();
    // The peer takes SDA after 40 us of quiet wire, counted from the closing
    // NACK of the ack read - inside arm_ms.
    const bool held = !SdaPin::read();
    const uint8_t pulses = Host::unstick();
    outlast(h, s0);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  SDA held by the peer: ", held ? "yes" : "no", "; unstick() clocked ", pulses,
          " pulse(s) before it let go; the peer counted ", r.aux0, " falling edge(s)", crlf);
    bench.verdict("the peer really held SDA low", held);
    // The peer lets go at its fourth SCL fall, within a turn of its loop
    // (under a microsecond); unstick() reads SDA after each pulse's second
    // half bit (5 us): the fourth pulse is the one that finds it free.
    bench.verdict("unstick() clocked until the client released: exactly the four falling edges the peer was told "
                  "to wait for",
                  pulses == 4u && rep && r.aux0 == 4u);
    // A held line nine clocks cannot free: the deadline alone releases it.
    twilink::Params s{};
    s.ms = 100;
    s.aux8 = 0;
    if (!peer_act(Op::hold_sda, s)) {
        bench.verdict("the peer accepted the second hold_sda command", false);
        return;
    }
    const uint32_t s1 = us_now();
    const uint32_t t0 = us_now();
    const uint8_t stuck = Host::unstick();
    const uint32_t pulse_us = us_now() - t0;
    outlast(s, s1);
    host_ready();
    const uint8_t after = Host::unstick();
    const uint8_t probe = tenure<Host>(twilink::command_addr, nullptr, 0, nullptr, 0);
    print(serial, "  SDA held through nine clocks: unstick() ", hex(stuck), " in ", pulse_us, " us; released: ",
          after, ", the probe after ", probe, crlf);
    bench.verdict("a line that stays low through nine clocks and a STOP is 0xFF - a short, as far as nine "
                  "clocks can tell - and 0 once the far end lets go, the bus working after it",
                  stuck == 0xFFu && after == 0u && probe == i2c_ok);
    // Nine pulses of two half bits and the STOP's two: twenty waits of 5 us,
    // 100 us; each wait on the platform's microsecond ruler comes back up to
    // 1 us late, and the pad hand-overs around it some microseconds - 100 to
    // 100 + 20 + 30 = 150 us.
    bench.verdict("THE HALF BIT IS PACED BY ITS RULER: the nine clocks and the STOP took 100 to 150 us (twenty "
                  "waits of 5 us, each up to 1 us late, and the pads handed over)",
                  pulse_us >= 100u && pulse_us <= 150u);
    bench.verdict("and the command channel is back", command(Op::ping));
}

// =============================================================================
// k - arbitration on the wire
// =============================================================================

/// ONE TENURE STARTED INTO A BUSY BUS: a START condition by hand, the
/// peer's own start() written inside it, this host's start() too - both
/// held by the busy bus -, then the STOP that frees it: both STARTs leave
/// on the one edge and the wired-AND decides.
template <typename H = Host>
uint8_t arb_tenure(uint8_t addr, const uint8_t* tx, uint8_t len, uint8_t* rx, uint8_t rx_len, uint32_t lead_us,
                   I2cEngine engine = I2cEngine::interrupt) {
    typename H::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = link_speed;
    r.engine = engine;
    transfer_done = false;
    isr_entries = 0;
    inject_start();
    spin_us(lead_us);
    const bool sync = H::start(r);
    spin_us(20);
    inject_stop();
    if (sync) {
        return H::status();
    }
    if (!wait_until([] { return transfer_done; }, 200'000u)) {
        (void)H::recover();
        return no_answer;
    }
    return transfer_status;
}

void tk_arbitration() {
    if (!need_peer()) {
        return;
    }
    struct Round {
        uint8_t dut_target;
        bool dut_should_lose;
        const char* name;
    };
    // 0x6B = 1101011 against 0x2C = 0101100: this board's first address bit
    // is the one; 0x11 = 0010001 against 0x2C: the peer's second bit is.
    const Round rounds[] = {
        {twilink::command_addr, true, "this board -> 0x6B against the peer -> 0x2C (this board must lose)"},
        {twilink::low_addr, false, "this board -> 0x11 against the peer -> 0x2C (the peer must lose)"},
    };
    uint8_t as_expected = 0;
    for (const Round& rd : rounds) {
        twilink::Params a{};
        a.ms = 600;
        a.addr = twilink::dut_addr;
        a.target = twilink::dut_addr;
        a.count = 4;
        a.seed = 0x40;
        a.aux16 = 120;
        if (!peer_act(Op::arb, a)) {
            bench.verdict("the peer accepted the arbitration command", false);
            return;
        }
        const uint32_t s0 = us_now();
        host_ready();
        fill_pattern(tx_buf, 5, 0xD0);
        // The window: 350 us, past the peer's 40 us of quiet and its 120 us
        // lead, with its start() - a disable, IC_TAR and an enable, 8 to 12 us
        // at 100 kHz - well inside.
        const uint8_t st = arb_tenure(rd.dut_target, tx_buf, 5, nullptr, 0, 350);
        outlast(a, s0);
        twilink::Report r{};
        const bool rep = peer_report(r);
        const bool dut_lost = st == i2c_arb_lost;
        const bool peer_lost = rep && (r.flags & twilink::report_arblost) != 0u;
        const bool peer_ran = rep && (r.flags & twilink::report_host_ran) != 0u;
        print(serial, "  ", rd.name, crlf, "    this board: status ", st, "; the peer: flags ", hex(r.flags),
              " status ", r.mstatus, crlf);
        if (rd.dut_should_lose ? (dut_lost && peer_ran && !peer_lost) : (!dut_lost && peer_lost)) {
            ++as_expected;
        }
        if (rd.dut_should_lose) {
            bench.verdict("aiming at the HIGHER address this host loses the wired-AND and reports i2c_arb_lost; "
                          "the peer's host keeps the bus (nobody answers 0x2C: its nack_addr)",
                          dut_lost && peer_ran && !peer_lost && r.mstatus == i2c_nack_addr);
        } else {
            bench.verdict("aiming at the LOWER address this host keeps the bus (nobody answers 0x11: i2c_nack_addr) "
                          "and it is the PEER's host that reports the loss",
                          st == i2c_nack_addr && peer_lost);
        }
    }
    bench.verdict("arbitration decided in BOTH directions on the wire: two DW_apb_i2c hosts, their STARTs "
                  "released by one STOP",
                  as_expected == 2u);
    // AND UNDER dma: the engined host's read of 16 aimed at the higher
    // address, its read half on the engines from start(), loses at the
    // first address bit; the abort puts the engines away, and the next
    // engined read is exact.
    {
        twilink::Params a{};
        a.ms = 600;
        a.addr = twilink::dut_addr;
        a.target = twilink::dut_addr;
        a.count = 4;
        a.seed = 0x40;
        a.aux16 = 120;
        if (!peer_act(Op::arb, a)) {
            bench.verdict("the peer accepted the arbitration command", false);
            return;
        }
        const uint32_t s0 = us_now();
        owner = Owner::none;
        Host::release();
        (void)DmaHost::init(clock);
        owner = Owner::dma_host;
        const uint8_t st = arb_tenure<DmaHost>(twilink::command_addr, nullptr, 0, rx_buf, 16, 350, I2cEngine::dma);
        owner = Owner::none;
        DmaHost::release();
        host_ready();
        outlast(a, s0);
        twilink::Report r{};
        const bool rep = peer_report(r);
        const bool peer_lost = rep && (r.flags & twilink::report_arblost) != 0u;
        const bool peer_ran = rep && (r.flags & twilink::report_host_ran) != 0u;
        print(serial, "  under dma, a read of 16 -> 0x6B against the peer -> 0x2C: this board ", st,
              "; the peer: flags ", hex(r.flags), " status ", r.mstatus, crlf);
        bench.verdict("an engined read (dma) aiming at the higher address loses the wired-AND: i2c_arb_lost, the "
                      "peer's host keeping the bus",
                      st == i2c_arb_lost && peer_ran && !peer_lost && r.mstatus == i2c_nack_addr);
    }
    bench.verdict("the command channel is back after the races", command(Op::ping));
}

// =============================================================================
// l - the roles invert: this board as the peer's client
// =============================================================================

/// The window for a peer's host action, this board a client: SDA held by
/// hand `us` microseconds, the peer's start() written inside, then the STOP.
void client_window(uint32_t us) {
    inject_start();
    spin_us(us);
    inject_stop();
}

void tl_roles() {
    if (!need_peer()) {
        return;
    }
    // ---- written to ----
    twilink::Params c{};
    c.ms = 800;
    c.target = twilink::dut_addr;
    c.count = 6;
    c.seed = 0xB0;
    c.aux16 = 200;
    if (!peer_act(Op::arb, c)) {
        bench.verdict("the peer accepted the host command that addresses this board", false);
        return;
    }
    uint32_t s0 = us_now();
    Host::release();
    const bool up = client_ready(twilink::dut_addr, false);
    client_window(400);
    (void)wait_until([] { return client_stops != 0u; }, 300'000u);
    const uint8_t mism = pattern_mismatches(client_heard, 6, c.seed, 0);
    const uint16_t heard = client_count;
    const uint16_t stops = client_stops;
    owner = Owner::none;
    Client::release();
    host_ready();
    outlast(c, s0);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  as a CLIENT, written to: init ", up, ", bytes ", heard, " (", mism, " wrong), stops ", stops,
          "; the peer's host: flags ", hex(r.flags), " status ", r.mstatus, crlf);
    bench.verdict("this board's client took a REAL host's write on the wire: six bytes, byte-exact, ending on "
                  "its STOP, and the peer's host saw the tenure complete",
                  up && heard == 6u && mism == 0u && stops == 1u && rep && r.mstatus == i2c_ok);

    // ---- read from: two reads of six, back to back ----
    twilink::Params rd{};
    rd.ms = 900;
    rd.target = twilink::dut_addr;
    rd.count = 6;
    rd.aux8 = 2;
    rd.aux16 = 200;
    if (!peer_act(Op::host_read, rd)) {
        bench.verdict("the peer accepted the host_read command", false);
        return;
    }
    s0 = us_now();
    Host::release();
    const bool up2 = client_ready(twilink::dut_addr, false);
    for (uint8_t i = 0; i < sizeof client_answers; ++i) {
        client_answers[i] = static_cast<uint8_t>(0xC0u + 7u * i);
    }
    client_window(400);
    (void)wait_until([] { return client_nacks >= 2u && client_stops >= 2u; }, 400'000u);
    const uint16_t gave = client_given;
    const uint16_t nacks = client_nacks;
    const uint16_t flushes = client_flushes;
    owner = Owner::none;
    Client::release();
    host_ready();
    outlast(rd, s0);
    twilink::Report rr{};
    const bool rrep = peer_report(rr);
    const uint16_t want_sum = sum_of(client_answers, 12);
    print(serial, "  as a CLIENT, read from: init ", up2, ", bytes given ", gave, ", closing NACKs ", nacks,
          ", flushes ", flushes, "; the peer read ", rr.count, " bytes in ", rr.addr_hits, " reads, status ",
          rr.mstatus, ", sum ", hex(rr.sum), " (", hex(want_sum), "), first ", hex(rr.first), ", the second read's "
          "first ", hex(rr.aux0), " (", hex(client_answers[6]), ")", crlf);
    bench.verdict("the client served a REAL host's two reads as a transmitter, one byte a read request: every byte "
                  "the peer took is the run this side gave (count, sum, both first bytes), nothing flushed",
                  up2 && rrep && rr.mstatus == i2c_ok && rr.count == 12u && rr.addr_hits == 2u && rr.sum == want_sum &&
                      rr.first == client_answers[0] && rr.aux0 == client_answers[6] && gave == 12u && nacks == 2u &&
                      flushes == 0u);

    // ---- the general call, from the far end ----
    // The peer's host refuses address 0x00 as this board's does
    // (util/i2c_bus.hpp): commanded to write the general call at this
    // board's client - its IC_ACK_GENERAL_CALL on - it answers
    // i2c_rejected and nothing reaches the wire.
    twilink::Params g{};
    g.ms = 800;
    g.target = 0x00;
    g.count = 3;
    g.seed = 0x90;
    g.aux16 = 200;
    if (!peer_act(Op::arb, g)) {
        bench.verdict("the peer accepted the general-call command", false);
        return;
    }
    s0 = us_now();
    Host::release();
    const bool up3 = client_ready(twilink::dut_addr, true);
    client_window(400);
    // No STOP_DET closes a general call (IC_CON's note): the third byte does.
    (void)wait_until([] { return client_count >= 3u; }, 300'000u);
    spin_us(200);
    const uint16_t gheard = client_count;
    const uint16_t gcalls = client_gcalls;
    const uint16_t gstops = client_stops;
    const uint8_t gmism = pattern_mismatches(client_heard, 3, g.seed, 0);
    owner = Owner::none;
    Client::release();
    host_ready();
    outlast(g, s0);
    twilink::Report gr{};
    const bool grep = peer_report(gr);
    print(serial, "  a GENERAL CALL commanded at this board's client: init ", up3, ", general-call events ", gcalls,
          ", bytes ", gheard, " (", gmism, " wrong), stops ", gstops, "; the peer's host status ", gr.mstatus, crlf);
    bench.verdict("the peer's host refuses the general call too (i2c_rejected), and this board's client, the "
                  "general call acknowledged, hears nothing",
                  up3 && gcalls == 0u && gheard == 0u && gstops == 0u && grep && gr.mstatus == i2c_rejected);
    bench.verdict("the command channel is back after the role swaps", command(Op::ping));
}

// =============================================================================
// m - the client's edges: the overrun without the hold, the disable inside
// a tenure
// =============================================================================

/// One peer host write of `count` bytes to this board's client, inside the
/// hand-made window; `hold` the client's RX_FIFO_FULL_HLD_CTRL, `drain`
/// whether its vector takes the bytes (RX_FULL unmasked), `cut_after` the
/// bytes after which this side disables the block (0: never). The bytes
/// the FIFO kept are read out after the STOP when the vector did not take
/// them. Returns the peer's report.
struct EdgeRun {
    bool up = false;
    bool rep = false;
    twilink::Report r{};
    uint16_t heard = 0;
    uint8_t wrong = 0;
    uint16_t overruns = 0;
    uint16_t stops = 0;
    bool idle = false;
    bool busy_bit = false;
    bool lost_bit = false;
};

EdgeRun client_write_run(uint8_t count, uint8_t seed, bool hold, bool drain, uint8_t cut_after,
                         uint32_t drain_after_us = 0) {
    EdgeRun e{};
    twilink::Params c{};
    c.ms = 800;
    c.target = twilink::dut_addr;
    c.count = count;
    c.seed = seed;
    c.aux16 = 200;
    if (!peer_act(Op::arb, c)) {
        return e;
    }
    const uint32_t s0 = us_now();
    Host::release();
    e.up = client_ready(twilink::dut_addr, false, hold);
    if (!drain) {
        Client::interrupts(I2cInterrupt::rx_full, false);
    }
    inject_start();
    spin_us(400);
    inject_stop();
    if (cut_after != 0u) {
        // The block disabled from here, the instant the vector has taken
        // byte `cut_after` (polled: the vector stores each byte as it lands;
        // the peer's host starts at the window's STOP, 90 us a byte).
        const bool reached = wait_until([cut_after] { return client_count >= cut_after; }, 20'000u);
        if (reached) {
            (void)Raw::disable(0);
        }
        e.idle = wait_until([] { return !Raw::running(); }, 5000u);
        e.busy_bit = Raw::client_disabled_while_busy();
        e.lost_bit = Raw::client_rx_data_lost();
        spin_us(3000);
    } else {
        if (drain_after_us != 0u) {
            spin_us(drain_after_us);
            Client::interrupts(I2cInterrupt::rx_full, true);
        }
        (void)wait_until([] { return client_stops != 0u; }, 300'000u);
        if (!drain && drain_after_us == 0u) {
            while (Client::data_ready()) {
                const uint8_t b = Client::take();
                if (client_count < sizeof client_heard) {
                    client_heard[client_count] = b;
                }
                client_count = client_count + 1u;
            }
        }
    }
    e.heard = client_count;
    e.wrong = pattern_mismatches(client_heard, static_cast<uint8_t>(e.heard < 64u ? e.heard : 64u), seed, 0);
    e.overruns = client_overruns;
    e.stops = client_stops;
    owner = Owner::none;
    Client::release();
    host_ready();
    outlast(c, s0);
    e.rep = peer_report(e.r);
    return e;
}

void tm_client_edges() {
    if (!need_peer()) {
        return;
    }
    constexpr uint8_t depth = 16;   // IC_RX_BUFFER_DEPTH on both chips (letter a reads it)
    constexpr uint8_t sent = 24;
    // ---- the overrun: no hold, nothing taken until the STOP ----
    const EdgeRun o = client_write_run(sent, 0x31, false, false, 0);
    print(serial, "  NO HOLD, the FIFO left full: ", sent, " bytes written by the peer's host (status ", o.r.mstatus,
          "), ", o.heard, " kept (", o.wrong, " wrong), overrun events ", o.overruns, ", stops ", o.stops, crlf);
    // One RX_OVER a byte lost: service() clears it in the vector within
    // microseconds, and the next lost byte is 90 us later at 100 kHz.
    bench.verdict("THE CLIENT'S OVERRUN: without RX_FIFO_FULL_HLD_CTRL the block acknowledges all 24 bytes of a real "
                  "host's write (its tenure completes), keeps the first 16 - the FIFO's depth - byte-exact, and "
                  "reports each of the 8 lost as an overrun event",
                  o.up && o.rep && o.r.mstatus == i2c_ok && o.heard == depth && o.wrong == 0u &&
                      o.overruns == sent - depth && o.stops == 1u);
    // ---- the hold, the same write: SCL held until the FIFO is drained ----
    const EdgeRun h = client_write_run(sent, 0x51, true, false, 0, 3000u);
    print(serial, "  THE HOLD, the FIFO drained 3 ms after the window: ", h.heard, " taken (", h.wrong,
          " wrong), overrun events ", h.overruns, ", the peer's host status ", h.r.mstatus, crlf);
    bench.verdict("with the hold (the default) the same write loses nothing: SCL held at the full FIFO until the "
                  "vector drains it, all 24 bytes byte-exact, no overrun, the host's tenure complete",
                  h.up && h.rep && h.r.mstatus == i2c_ok && h.heard == sent && h.wrong == 0u && h.overruns == 0u);
    // ---- the disable inside a host's write ----
    const EdgeRun w = client_write_run(sent, 0x71, true, true, 4);
    print(serial, "  DISABLED after the 4th of 24 written bytes: idle ", w.idle, ", SLV_DISABLED_WHILE_BUSY ",
          w.busy_bit, ", SLV_RX_DATA_LOST ", w.lost_bit, ", bytes taken ", w.heard, ", the peer's host status ",
          w.r.mstatus, crlf);
    // The disable lands microseconds after the vector stored byte 4; byte 5
    // takes 90 us on the wire, so it is the one in flight and refused.
    bench.verdict("a client disabled inside a host's WRITE goes idle, refuses the byte in flight - the host's "
                  "tenure ends on a data NACK after four - and reports SLV_DISABLED_WHILE_BUSY and SLV_RX_DATA_LOST",
                  w.up && w.rep && w.idle && w.busy_bit && w.lost_bit && w.r.mstatus == i2c_nack_data &&
                      w.heard == 4u);
    // ---- the disable inside a host's read: the third read request left
    // unanswered (SCL held by the block), the block disabled under it ----
    twilink::Params rd{};
    rd.ms = 900;
    rd.target = twilink::dut_addr;
    rd.count = 6;
    rd.aux8 = 1;
    rd.aux16 = 200;
    if (!peer_act(Op::host_read, rd)) {
        bench.verdict("the peer accepted the host_read command", false);
        return;
    }
    const uint32_t s0 = us_now();
    Host::release();
    const bool up = client_ready(twilink::dut_addr, false);
    for (uint8_t i = 0; i < sizeof client_answers; ++i) {
        client_answers[i] = static_cast<uint8_t>(0x40u + 5u * i);
    }
    client_cut_at = 2;
    client_window(400);
    const bool held = wait_until([] { return client_cut; }, 50'000u);
    (void)Raw::disable(0);
    const bool idle_early = wait_until([] { return !Raw::running(); }, 2000u);
    // Not idle while SCL is held for a byte: the request answered, then.
    Client::give(0xA5);
    Client::clear_read_request();
    const uint32_t t_dis = us_now();
    const bool idle = wait_until([] { return !Raw::running(); }, 50'000u);
    const uint32_t idle_us = us_now() - t_dis;
    const bool busy_bit = Raw::client_disabled_while_busy();
    const bool lost_bit = Raw::client_rx_data_lost();
    const uint16_t gave = client_given;
    owner = Owner::none;
    Client::release();
    host_ready();
    outlast(rd, s0);
    twilink::Report rr{};
    const bool rrep = peer_report(rr);
    print(serial, "  DISABLED at the 3rd read request, SCL held: held ", held, ", idle within 2 ms ", idle_early,
          "; after a byte given, idle ", idle, " in ", idle_us,
          " us, SLV_DISABLED_WHILE_BUSY ", busy_bit, ", SLV_RX_DATA_LOST ", lost_bit, ", bytes given ", gave,
          "; the peer's host: status ", rr.mstatus, ", flags ", hex(rr.flags), ", count ", rr.count, crlf);
    // What the peer read: the two bytes given, then - the pads released -
    // the pull-up's 0xFF for the other four.
    const uint16_t want_sum = static_cast<uint16_t>(client_answers[0] + client_answers[1] + 4u * 0xFFu);
    bench.verdict("A CLIENT DISABLED AT A READ REQUEST IT HOLDS SCL FOR NEVER GOES IDLE: IC_EN stays 1 through 2 ms "
                  "and, a byte given after the disable (the FIFOs held erased: not taken), through 50 ms more, "
                  "neither status bit set - the byte it waits for can never come",
                  up && held && !idle_early && !idle && !busy_bit && !lost_bit && gave == 2u);
    bench.verdict("and release() - the block reset - lets SCL go: the far host's tenure completes, the two given "
                  "bytes then the pull-up's 0xFF",
                  rrep && rr.mstatus == i2c_ok && rr.count == 6u && rr.sum == want_sum);
    bench.verdict("the command channel is back after the client's edges", command(Op::ping));
}

// =============================================================================
// u, s - THE RECEIVE CHANNEL AT THE STOP
// =============================================================================
//
// A read under `dma` has no completion of its own (the receive engine reports
// errors alone): STOP_DET ends the tenure, and STOP_DET says the WIRE is over.
// The channel may still have beats to move, up to the FIFO's sixteen, since SCL
// holds only with the FIFO full. The chapter gives no bound on a channel's lag
// (RP2040 2.5.7, RP2350 12.6.1: a round of HIGH channels, then one normal). So:
//
//   u  THE LOAD: reads of 1, 8, 16, 64 and 255 at the three speeds under each
//      load below, a channel's state read in the vector's first loads; the
//      bytes at the reply and a millisecond later, the next read behind them.
//      How often a channel was late is the load's and is printed, not judged.
//   s  THE STAGE: the entry made to find the channel counting, on every shape.
//
// RP2350 12.6.4.2 on the request: "each 1-clock pulse on the dreq signal
// increments this counter ... The counter decrements when the transfer is issued
// to the address FIFOs", so a pulse banked outlives IC_DMA_CR going down; and
// "the user must not access a FIFO currently being serviced by the DMA", so the
// core reads IC_DATA_CMD only behind a stopped channel. TRANS_COUNT updates "each
// time a write transfer completes" (both chips' register lists), BUSY falls when
// the last transfer of the sequence completes. Letter s's first rows measure both.
//
// The rulers: the suite's `late_cycles()`, clk_sys cycles. What the suite states
// for these two letters: late_cycles(), late_line(on), the interrupt controller's
// line of the link instance, and late_flash(i), a run of the flash through the
// XIP's uncached alias.

using LateRxChannel = DmaChannel<DmaRx::channel>;
using LateTxChannel = DmaChannel<DmaTx::channel>;
using LateCopyA = DmaCopyEngine<8>;
using LateCopyB = DmaCopyEngine<9>;
using LateCopyC = DmaCopyEngine<10>;
using LateCopyD = DmaCopyEngine<11>;

/// A block of the load: at most the 16 KB of the shape docs/rp2350/dma.md measures.
constexpr uint32_t late_block_max = 16384u;
alignas(4) uint8_t late_load_ram[4][2][late_block_max];
alignas(4) uint32_t late_core1_ram[256];

enum class LateSource : uint8_t { sram, bridge, flash };

struct LateLoad {
    const char* name;
    uint8_t copies;
    bool high;
    LateSource source;
    bool core1;
    uint32_t block;
};

// A block out of the flash is 256 bytes: at 16 KB, a beat there being some 150 cycles,
// the channel beside two HIGH copies waits 2.4 million cycles (16 ms) a turn and a read
// of 255 outlasts any useful limit, which says nothing more about the entry.
constexpr LateLoad late_loads[] = {
    {"nothing else", 0, false, LateSource::sram, false, late_block_max},
    {"one HIGH byte copy, SRAM", 1, true, LateSource::sram, false, late_block_max},
    {"two HIGH byte copies, SRAM", 2, true, LateSource::sram, false, late_block_max},
    {"two HIGH byte copies, SRAM, blocks of 1 KB", 2, true, LateSource::sram, false, 1024},
    {"four HIGH byte copies, SRAM", 4, true, LateSource::sram, false, late_block_max},
    {"two normal byte copies, SRAM", 2, false, LateSource::sram, false, late_block_max},
    {"two HIGH, SRAM, core 1 in SRAM", 2, true, LateSource::sram, true, late_block_max},
    {"two HIGH byte reads across the bridge", 2, true, LateSource::bridge, false, late_block_max},
    {"four HIGH byte reads across the bridge", 4, true, LateSource::bridge, false, late_block_max},
    {"four HIGH across the bridge, core 1 in SRAM", 4, true, LateSource::bridge, true, late_block_max},
    {"one HIGH byte copy, uncached flash", 1, true, LateSource::flash, false, 256},
    {"two HIGH byte copies, uncached flash", 2, true, LateSource::flash, false, 256},
    {"four HIGH byte copies, uncached flash", 4, true, LateSource::flash, false, 256},
    {"two normal byte copies, uncached flash", 2, false, LateSource::flash, false, 256},
    {"four HIGH, uncached flash, core 1 in SRAM", 4, true, LateSource::flash, true, 256},
};

volatile uint8_t late_copies = 0;
volatile LateSource late_source = LateSource::sram;
volatile uint32_t late_block = late_block_max;
volatile bool late_core1_keeps = false;
volatile bool late_core1_live = false;
volatile uint32_t late_core1_turns = 0;
uint32_t late_load_blocks = 0;

template <typename E>
void late_keep(uint8_t i) {
    if (E::busy()) {
        return;
    }
    bool started = false;
    switch (late_source) {
        case LateSource::sram:
            started = E::copy(late_load_ram[i][1], static_cast<const uint8_t*>(late_load_ram[i][0]), late_block);
            break;
        case LateSource::bridge:
            // IC_COMP_VERSION, read-only, on the APB behind the same bridge as IC_DATA_CMD.
            started = E::fill(late_load_ram[i][1],
                              reinterpret_cast<const uint8_t*>(reinterpret_cast<uintptr_t>(&Raw::regs().IC_COMP_VERSION)),
                              late_block);
            break;
        case LateSource::flash:
            started = E::copy(late_load_ram[i][1], late_flash(i), late_block);
            break;
    }
    late_load_blocks = late_load_blocks + (started ? 1u : 0u);
}

/// Every copy of the load that has run out, started again.
void late_keep_all() {
    const uint8_t n = late_copies;
    if (n > 0u) { late_keep<LateCopyA>(0); }
    if (n > 1u) { late_keep<LateCopyB>(1); }
    if (n > 2u) { late_keep<LateCopyC>(2); }
    if (n > 3u) { late_keep<LateCopyD>(3); }
}

/// The turn core 0's waits take, unless core 1 keeps the load.
void late_load_turn() {
    if (!late_core1_keeps) {
        late_keep_all();
    }
}

[[noreturn]] void late_core1_entry() {
    late_core1_live = true;
    for (;;) {
        late_keep_all();
        for (uint32_t i = 0; i < 256u; ++i) {
            late_core1_ram[i] = late_core1_ram[(i + 128u) & 255u] + 1u;
        }
        late_core1_turns = late_core1_turns + 1u;
    }
}

void late_load_on(const LateLoad& l) {
    late_source = l.source;
    late_block = l.block;
    late_copies = l.copies;
    late_load_blocks = 0;
    if (l.copies > 0u) { LateCopyA::arm(DmaReport::errors, l.high); }
    if (l.copies > 1u) { LateCopyB::arm(DmaReport::errors, l.high); }
    if (l.copies > 2u) { LateCopyC::arm(DmaReport::errors, l.high); }
    if (l.copies > 3u) { LateCopyD::arm(DmaReport::errors, l.high); }
    late_core1_keeps = false;
    if (l.core1) {
        late_core1_live = false;
        late_core1_turns = 0;
        late_core1_keeps = Core1::launch(&late_core1_entry);
    } else {
        late_load_turn();
    }
}

/// The load off and whether a copy met a bus error (the load would be no load).
bool late_load_off() {
    if (late_core1_keeps) {
        (void)Core1::reset();
        late_core1_keeps = false;
    }
    const uint32_t errors = LateCopyA::errors() | LateCopyB::errors() | LateCopyC::errors() | LateCopyD::errors();
    late_copies = 0;
    LateCopyA::stop();
    LateCopyB::stop();
    LateCopyC::stop();
    LateCopyD::stop();
    return errors == 0u;
}

// ---- the vector's first loads --------------------------------------------------------

enum class LateStage : uint8_t {
    none,
    away,    ///< IC_DMA_CR down for the tail: no pulse, the bytes standing in the FIFO
    paused,  ///< the channel's EN down for the tail: pulses banked, the channel never let go
    behind,  ///< as paused, the thread letting EN up behind the reply
    race,    ///< as paused, the vector letting EN up `burns` turns ahead of the host's body
    steal,   ///< as away, the vector popping a byte: a byte nobody has
};

struct LateWatch {
    volatile bool on = false;
    volatile LateStage stage = LateStage::none;
    volatile uint16_t burns = 0;
    volatile bool stop = false;      ///< an entry with STOP_DET up was seen
    volatile bool rx_busy = false;   ///< the receive channel's BUSY at that entry
    volatile uint16_t rx_left = 0;   ///< its TRANS_COUNT there
    volatile uint8_t level = 0;      ///< IC_RXFLR there
    volatile bool tx_busy = false;   ///< the transmit channel's BUSY there
    volatile uint8_t credits = 0;    ///< the receive channel's banked requests there
    volatile uint32_t body_on_time = 0;   ///< the longest host body, the channel done
    volatile uint32_t body_late = 0;      ///< ... the channel counting
};
LateWatch late_watch;
volatile uint32_t late_burned = 0;

/// `turns` of a loop the compiler keeps, whichever core runs it.
[[gnu::always_inline]] inline void late_burn(uint32_t turns) {
    for (uint32_t i = 0; i < turns; ++i) {
        late_burned = late_burned + 1u;
    }
}

/// Ahead of the host's body. The count and BUSY are the vector's first loads.
[[gnu::always_inline]] inline void late_before() {
    const bool rx_busy = LateRxChannel::busy();
    const uint32_t rx_left = LateRxChannel::count();
    const bool tx_busy = LateTxChannel::busy();
    const uint8_t level = Raw::rx_count();
    const uint8_t credits = static_cast<uint8_t>(LateRxChannel::debug_credits());
    if ((Raw::raw_pending() & I2cInterrupt::stop_det) == 0u) {
        return;
    }
    late_watch.credits = credits;
    late_watch.stop = true;
    late_watch.rx_busy = rx_busy;
    late_watch.rx_left = static_cast<uint16_t>(rx_left);
    late_watch.tx_busy = tx_busy;
    late_watch.level = level;
    if (late_watch.stage == LateStage::race) {
        LateRxChannel::enable(true);
        late_burn(late_watch.burns);
    } else if (late_watch.stage == LateStage::steal && level != 0u) {
        (void)Raw::pop();
    }
}

// ---- one tenure under watch ----------------------------------------------------------

struct LateRow {
    uint32_t reads = 0;
    uint32_t exact = 0;        ///< the status owed and every cell right at the reply, nothing moving after
    uint32_t status_wrong = 0;
    uint32_t unanswered = 0;
    uint32_t late = 0;         ///< the channel BUSY at the STOP entry
    uint32_t owed_max = 0;     ///< the most beats TRANS_COUNT owed there
    uint32_t level_max = 0;    ///< the most entries IC_RXFLR held there
    uint32_t credits_max = 0;  ///< the most requests the channel had banked there
    uint32_t stale = 0;        ///< reads with a cell wrong at the reply
    uint32_t stale_cells = 0;  ///< how many
    uint32_t moved = 0;        ///< reads with a cell written after the reply
    uint32_t running = 0;      ///< the channel still BUSY at the reply
    uint32_t standing = 0;     ///< entries left in the FIFO at the reply
    uint32_t tx_busy = 0;      ///< the transmit channel BUSY at the STOP entry
    uint32_t next_wrong = 0;   ///< the read behind it not exact
};

void late_row_add(LateRow& all, const LateRow& r) {
    all.reads += r.reads;
    all.exact += r.exact;
    all.status_wrong += r.status_wrong;
    all.unanswered += r.unanswered;
    all.late += r.late;
    all.owed_max = r.owed_max > all.owed_max ? r.owed_max : all.owed_max;
    all.level_max = r.level_max > all.level_max ? r.level_max : all.level_max;
    all.credits_max = r.credits_max > all.credits_max ? r.credits_max : all.credits_max;
    all.stale += r.stale;
    all.stale_cells += r.stale_cells;
    all.moved += r.moved;
    all.running += r.running;
    all.standing += r.standing;
    all.tx_busy += r.tx_busy;
    all.next_wrong += r.next_wrong;
}

uint8_t late_seed = 0;
uint8_t late_rx_at = 0;   ///< the serve's read position, modulo the pattern's period
uint8_t late_tx_at = 0;   ///< the serve's write position
uint8_t late_cells[257];
uint8_t late_held[257];

uint8_t late_rx_value(uint32_t i) {
    return twilink::pattern_value(twilink::pattern_counting, late_seed, static_cast<uint16_t>(late_rx_at + i));
}

/// What a row with no load allows over the wire, in place of a second.
uint32_t late_limit_margin_us = 1'000'000u;
/// Letter s's cold row: every line of the XIP cache dropped right ahead of start(), which
/// runs with TX_CMD_BLOCK holding its entries in the FIFO, and how many of those tenures had
/// a DMA line entry inside it.
bool late_cold_start = false;
uint32_t late_inside_start = 0;

/// THE LIMIT, nine periods a byte twice over and a second: letter u's slowest measured
/// channel beside four HIGH copies of the flash waits 2.1 ms a turn for a block of 256
/// bytes (5.2 million cycles for 16 KB at 150 MHz, over 64), and a read of 255 at one beat
/// a turn is 540 ms of holding SCL. Past this it is `no_answer`, never a verdict's margin.
uint32_t late_limit_us(uint32_t bytes, I2cSpeed s) {
    return 18'000'000u / i2c_speed_hz(s) * (bytes + 2u) + late_limit_margin_us;
}

/// start(), as the cold row wants it when it does.
void late_start(const DmaHost::Request& r) {
    if (late_cold_start) {
        Raw::command_block(true);
        xip_empty();
    }
    (void)DmaHost::start(r);
    if (late_cold_start) {
        late_inside_start += dma_entries != 0u ? 1u : 0u;
        Raw::command_block(false);
    }
}

/// One millisecond with the load kept: what a late channel has left to write lands.
void late_after_reply() {
    const uint32_t t0 = us_now();
    while (us_now() - t0 < 1000u) {
        late_load_turn();
    }
}

/// One read under `dma`, staged or not, judged at the reply and a millisecond later.
void late_read(uint8_t tx_len, uint8_t n, I2cSpeed s, LateStage stage, uint8_t tail, uint16_t burns, LateRow& row,
               uint8_t want = i2c_ok, bool clean = true) {
    // A stage's pause that reached a channel already done has left EN clear behind the
    // engine's back; the engine's next start would not know.
    if (!LateRxChannel::enabled()) {
        DmaRx::stop();
        DmaRx::arm(Raw::data_address(), Raw::dreq_rx, false, DmaReport::errors);
    }
    for (uint32_t i = 0; i < n + 1u; ++i) {
        late_cells[i] = static_cast<uint8_t>(~late_rx_value(i));
    }
    for (uint32_t i = 0; i < tx_len; ++i) {
        tx_buf[i] = twilink::pattern_value(twilink::pattern_counting, late_seed, static_cast<uint16_t>(late_tx_at + i));
    }
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(late_cells));
    r.rx_len = n;
    r.speed = s;
    r.engine = I2cEngine::dma;
    late_watch.stage = stage;
    late_watch.burns = burns;
    late_watch.stop = false;
    late_watch.rx_busy = false;
    late_watch.rx_left = 0;
    late_watch.level = 0;
    late_watch.tx_busy = false;
    late_watch.credits = 0;
    late_watch.on = true;
    transfer_done = false;
    isr_entries = 0;
    dma_entries = 0;
    const uint32_t limit = late_limit_us(static_cast<uint32_t>(tx_len) + n, s);
    const uint32_t t0 = us_now();
    late_start(r);
    if (stage != LateStage::none) {
        // The tail begins at this count. The transmit block's end lets the request up
        // again (dma_isr()), so a read that has one waits for it first.
        const uint32_t room = n < tail ? n : tail;
        while (((n > 2u && DmaTx::busy()) || LateRxChannel::count() > room) && !transfer_done &&
               us_now() - t0 < limit) {
            late_load_turn();
        }
        if (stage == LateStage::away || stage == LateStage::steal) {
            Raw::dma_requests(false, false);
        } else {
            LateRxChannel::enable(false);
        }
    }
    while (!transfer_done && us_now() - t0 < limit) {
        late_load_turn();
    }
    // THE REPLY: the first loads the requester could make.
    const bool answered = transfer_done;
    const bool running = LateRxChannel::busy();
    const uint8_t level = Raw::rx_count();
    uint32_t wrong = 0;
    for (uint32_t i = 0; i < n + 1u; ++i) {
        late_held[i] = late_cells[i];
        const uint8_t owed = i < n ? late_rx_value(i) : static_cast<uint8_t>(~late_rx_value(i));
        wrong += late_held[i] != owed ? 1u : 0u;
    }
    late_watch.on = false;
    if (stage == LateStage::behind) {
        LateRxChannel::enable(true);
    }
    late_after_reply();
    bool moved = false;
    for (uint32_t i = 0; i < n + 1u; ++i) {
        moved = moved || late_cells[i] != late_held[i];
    }
    if (!answered) {
        (void)DmaHost::recover();
    }
    const uint8_t status = answered ? transfer_status : no_answer;
    // A stolen byte is owed its status; a stage that came too late stole nothing.
    const uint8_t owed_status = stage == LateStage::steal && !late_watch.rx_busy ? uint8_t{i2c_ok} : want;
    row.reads += 1u;
    row.unanswered += answered ? 0u : 1u;
    row.status_wrong += status != owed_status ? 1u : 0u;
    row.late += late_watch.rx_busy ? 1u : 0u;
    row.owed_max = late_watch.rx_left > row.owed_max ? late_watch.rx_left : row.owed_max;
    row.level_max = late_watch.level > row.level_max ? late_watch.level : row.level_max;
    row.credits_max = late_watch.credits > row.credits_max ? late_watch.credits : row.credits_max;
    row.tx_busy += late_watch.tx_busy ? 1u : 0u;
    row.stale += wrong != 0u ? 1u : 0u;
    row.stale_cells += wrong;
    row.moved += moved ? 1u : 0u;
    row.running += running ? 1u : 0u;
    row.standing += level;
    const bool nothing_after = !moved && !running && level == 0u;
    row.exact += answered && status == owed_status && nothing_after && (wrong == 0u || owed_status != i2c_ok) ? 1u : 0u;
    late_rx_at = static_cast<uint8_t>(late_rx_at + n);
    late_tx_at = static_cast<uint8_t>(late_tx_at + tx_len);
    late_watch.stage = LateStage::none;
    // WHAT A WRONG TENURE LEFT is carried into the next read only where the caller
    // judges that (`clean` false); otherwise the FIFO is given a disable, which flushes
    // it (RP2040 4.3.10.3, RP2350 12.2.10.3), so that one wrong read counts once.
    if (clean && (running || level != 0u || Raw::rx_count() != 0u)) {
        (void)Raw::disable();
        Raw::enable();
    }
}

/// A write under `dma`, its bytes the pattern at the serve's position: the transmit
/// channel's BUSY at the STOP entry is what is judged here, the bytes by the partner.
void late_write(uint8_t n, I2cSpeed s, LateRow& row) {
    for (uint32_t i = 0; i < n; ++i) {
        tx_buf[i] = twilink::pattern_value(twilink::pattern_counting, late_seed, static_cast<uint16_t>(late_tx_at + i));
    }
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.tx_len = n;
    r.speed = s;
    r.engine = I2cEngine::dma;
    late_watch.stage = LateStage::none;
    late_watch.stop = false;
    late_watch.tx_busy = false;
    late_watch.on = true;
    transfer_done = false;
    dma_entries = 0;
    const uint32_t limit = late_limit_us(n, s);
    const uint32_t t0 = us_now();
    late_start(r);
    while (!transfer_done && us_now() - t0 < limit) {
        late_load_turn();
    }
    const bool answered = transfer_done;
    late_watch.on = false;
    if (!answered) {
        (void)DmaHost::recover();
    }
    row.reads += 1u;
    row.unanswered += answered ? 0u : 1u;
    row.status_wrong += answered && transfer_status == i2c_ok ? 0u : 1u;
    row.tx_busy += late_watch.tx_busy ? 1u : 0u;
    late_tx_at = static_cast<uint8_t>(late_tx_at + n);
}

// ---- how near, polled ----------------------------------------------------------------

struct LagRow {
    uint32_t rows = 0;
    uint32_t lag_max = 0;      ///< the channel's BUSY down, cycles from the request let up
    uint32_t window_min = 0xFFFF'FFFFu;   ///< STOP_DET, cycles from there
    int32_t margin_min = 0x7FFF'FFFF;
    uint32_t late = 0;         ///< STOP_DET first
    uint32_t turn_max = 0;
    uint32_t wrong = 0;
    uint32_t stale = 0;        ///< BUSY down and the next load not the byte
};

/// A read of one with the vector's line off and the request held down until the byte
/// stands in the FIFO, then let up: a masked poll dates BUSY's fall and STOP_DET in one
/// tenure. A turn is three loads, the poll's resolution, printed.
void late_lag_row(I2cSpeed s, LagRow& row) {
    late_cells[0] = static_cast<uint8_t>(~late_rx_value(0));
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(late_cells));
    r.rx_len = 1;
    r.speed = s;
    r.engine = I2cEngine::dma;
    const uint32_t limit = late_limit_us(1, s);
    late_line(false);
    transfer_done = false;
    (void)DmaHost::start(r);
    Raw::dma_requests(false, false);
    const uint32_t t_wait = us_now();
    while (Raw::rx_count() == 0u && us_now() - t_wait < limit) {
        late_load_turn();
    }
    uint32_t zero_at = 0;
    uint32_t stop_at = 0;
    uint32_t turn = 0;
    bool zero = false;
    bool stopped = false;
    uint8_t first = 0;
    {
        InterruptGuard guard;
        late_load_turn();
        const uint32_t t0 = late_cycles();
        Raw::dma_requests(false, true);
        uint32_t last = t0;
        for (uint32_t turns = 4000u; turns != 0u && !(zero && stopped); --turns) {
            const bool busy = LateRxChannel::busy();
            if (!zero && !busy) {
                first = *static_cast<volatile uint8_t*>(late_cells);
            }
            const uint32_t raw = Raw::raw_pending();
            const uint32_t now = late_cycles();
            turn = now - last > turn ? now - last : turn;
            last = now;
            if (!zero && !busy) {
                zero = true;
                zero_at = now - t0;
            }
            if (!stopped && (raw & I2cInterrupt::stop_det) != 0u) {
                stopped = true;
                stop_at = now - t0;
            }
        }
    }
    late_line(true);
    const uint32_t t1 = us_now();
    while (!transfer_done && us_now() - t1 < limit) {
        late_load_turn();
    }
    if (!transfer_done) {
        (void)DmaHost::recover();
    }
    row.wrong += transfer_done && transfer_status == i2c_ok && late_cells[0] == late_rx_value(0) ? 0u : 1u;
    late_rx_at = static_cast<uint8_t>(late_rx_at + 1u);
    if (!stopped) {
        return;
    }
    row.rows += 1u;
    row.turn_max = turn > row.turn_max ? turn : row.turn_max;
    row.window_min = stop_at < row.window_min ? stop_at : row.window_min;
    if (zero) {
        const uint8_t owed = static_cast<uint8_t>(late_seed + late_rx_at - 1u);
        row.stale += first != owed ? 1u : 0u;
        row.lag_max = zero_at > row.lag_max ? zero_at : row.lag_max;
        const int32_t m = static_cast<int32_t>(stop_at) - static_cast<int32_t>(zero_at);
        row.margin_min = m < row.margin_min ? m : row.margin_min;
    }
    row.late += !zero || zero_at > stop_at ? 1u : 0u;
}

// ---- the serve around a row ----------------------------------------------------------

bool late_begin(uint16_t bytes, uint8_t seed, twilink::Params& a, uint32_t& s0) {
    if (!need_peer()) {
        return false;
    }
    a = serve_params(30'000, seed);
    a.count = bytes;   // the serve ends at the STOP of the tenure that reaches it
    if (!peer_act(Op::serve, a)) {
        bench.verdict("the peer accepted the serve command", false);
        return false;
    }
    s0 = us_now();
    owner = Owner::none;
    Host::release();
    (void)DmaHost::init(clock);
    late_rx_at = 0;
    late_tx_at = 0;
    late_seed = seed;
    owner = Owner::dma_host;
    return true;
}

/// The host given back and the partner's tally. Its counted serve ended at the last STOP;
/// a tally short of the count leaves it serving, and then its deadline is waited out.
bool late_end(const twilink::Params& a, uint32_t s0, twilink::Report& r) {
    owner = Owner::none;
    DmaHost::release();
    host_ready();
    settle_ms(120);
    if (peer_report(r)) {
        return true;
    }
    outlast(a, s0);
    return peer_report(r);
}

// ---- u ------------------------------------------------------------------------------

constexpr uint8_t late_lengths[5] = {1, 8, 16, 64, 255};
constexpr uint8_t late_write_lengths[2] = {8, 64};
constexpr uint16_t late_reps = 30;
constexpr uint16_t late_lag_reps = 20;

struct LateCensus {
    uint32_t reads = 0;
    uint32_t exact = 0;
    uint32_t late = 0;
    uint32_t owed_max = 0;
    uint32_t wrong_tally = 0;
    uint32_t tx_busy = 0;
    uint32_t load_faults = 0;
};

void late_census_row(const LateLoad& l, I2cSpeed s, LateCensus& all) {
    uint16_t bytes = late_lag_reps;
    for (const uint8_t n : late_lengths) {
        bytes = static_cast<uint16_t>(bytes + late_reps * n);
    }
    uint16_t written = 0;
    for (const uint8_t n : late_write_lengths) {
        written = static_cast<uint16_t>(written + late_reps * n);
    }
    twilink::Params a{};
    uint32_t s0 = 0;
    if (!late_begin(static_cast<uint16_t>(bytes + written), static_cast<uint8_t>(0x30u + all.reads), a, s0)) {
        all.wrong_tally += 1u;
        return;
    }
    late_load_on(l);
    LateRow by[5] = {};
    LateRow writes{};
    LagRow lag{};
    late_watch.body_on_time = 0;
    late_watch.body_late = 0;
    for (uint16_t pass = 0; pass < late_reps; ++pass) {
        for (uint8_t i = 0; i < 5u; ++i) {
            late_read(0, late_lengths[i], s, LateStage::none, 0, 0, by[i]);
        }
        for (const uint8_t n : late_write_lengths) {
            late_write(n, s, writes);
        }
    }
    for (uint16_t i = 0; i < late_lag_reps; ++i) {
        late_lag_row(s, lag);
    }
    const uint32_t blocks = late_load_blocks;
    const uint32_t core1_turns = late_core1_turns;
    const bool load_clean = late_load_off();
    all.load_faults += load_clean ? 0u : 1u;
    twilink::Report r{};
    const bool rep = late_end(a, s0, r);
    LateRow sum{};
    for (const LateRow& b : by) {
        late_row_add(sum, b);
    }
    const bool tally = rep && r.aux1 == static_cast<uint8_t>(bytes) && r.aux0 == static_cast<uint8_t>(written) &&
                       r.sstatus == 0u;
    print(serial, "    ", i2c_speed_hz(s) / 1000u, " kHz: late at the entry, reads of 1, 8, 16, 64, 255, ", late_reps, " each: ",
          by[0].late, ", ", by[1].late, ", ", by[2].late, ", ", by[3].late, ", ", by[4].late, "; beats owed at most ", sum.owed_max,
          ", FIFO at most ", sum.level_max, crlf);
    print(serial, "      exact ", sum.exact, " of ", sum.reads, "; at the reply wrong ", sum.stale, " (", sum.stale_cells, " cells), running ",
          sum.running, ", FIFO ", sum.standing, "; moved after it ", sum.moved, "; unanswered ", sum.unanswered, "; status wrong ",
          sum.status_wrong, crlf);
    print(serial, "      writes ", writes.reads, " status wrong ", writes.status_wrong, ", transmit channel busy at a STOP entry ",
          sum.tx_busy + writes.tx_busy, "; partner's tally ", tally ? "exact" : "OFF", " (", r.aux1, "/", static_cast<uint8_t>(bytes),
          ", ", r.aux0, "/", static_cast<uint8_t>(written), ", ", r.sstatus, " off); copies' blocks ", blocks, ", core 1 turns ", core1_turns,
          crlf);
    print(serial, "      polled, ", lag.rows, " tenures: BUSY down at most ", lag.lag_max, " cycles from the request, STOP_DET at least ",
          lag.window_min, ", the least margin ", lag.margin_min, ", STOP_DET first in ", lag.late, ", the poll's turn at most ",
          lag.turn_max, ", a stale cell ", lag.stale, ", wrong ", lag.wrong, "; the host's body at most ", late_watch.body_on_time,
          " on time, ", late_watch.body_late, " late", crlf);
    all.reads += sum.reads;
    all.exact += sum.exact;
    all.late += sum.late;
    all.owed_max = sum.owed_max > all.owed_max ? sum.owed_max : all.owed_max;
    all.wrong_tally += tally ? 0u : 1u;
    all.tx_busy += sum.tx_busy + writes.tx_busy;
}

void tu_late_load() {
    LateCensus all{};
    for (const LateLoad& l : late_loads) {
        print(serial, "  ", l.name, crlf);
        for (const I2cSpeed s : {I2cSpeed::standard_100k, I2cSpeed::fast_400k, I2cSpeed::fast_plus_1m}) {
            late_census_row(l, s, all);
        }
    }
    print(serial, "  in all ", all.reads, " reads, exact ", all.exact, ", late at the entry ", all.late, ", beats owed at most ",
          all.owed_max, ", the partner's tally off in ", all.wrong_tally, " rows, a transmit channel busy at a STOP ", all.tx_busy,
          ", a copy's bus error in ", all.load_faults, " rows", crlf);
    // No bound to derive: each read is judged cell by cell. How often a channel was late
    // is the load's and is printed, not judged: a rate of zero says nothing of the next load.
    bench.verdict("EVERY READ EXACT UNDER EVERY LOAD, late or not: every cell at the reply, none written after it, nothing "
                  "standing in the FIFO, the channel stopped",
                  all.reads != 0u && all.exact == all.reads && all.load_faults == 0u);
    bench.verdict("... the partner's tally agrees on every row", all.wrong_tally == 0u);
    bench.verdict("THE TRANSMIT SIDE: no transmit channel is busy at a tenure's STOP (the last entry is the pump's, behind the "
                  "block's end)", all.tx_busy == 0u);
}

// ---- s ------------------------------------------------------------------------------

/// A channel polled to its end, the next load the cell: what says a beat is in memory.
struct SaysRow {
    uint32_t rows = 0;
    uint32_t stale = 0;
    uint32_t turns_max = 0;
};

/// `count`: TRANS_COUNT at zero, else BUSY down. A read of one, as in late_lag_row().
void late_says_row(bool count, I2cSpeed s, SaysRow& row) {
    late_cells[0] = static_cast<uint8_t>(~late_rx_value(0));
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(late_cells));
    r.rx_len = 1;
    r.speed = s;
    r.engine = I2cEngine::dma;
    const uint32_t limit = late_limit_us(1, s);
    late_line(false);
    transfer_done = false;
    (void)DmaHost::start(r);
    Raw::dma_requests(false, false);
    const uint32_t t_wait = us_now();
    while (Raw::rx_count() == 0u && us_now() - t_wait < limit) {
    }
    uint8_t first = 0;
    uint32_t turns = 0;
    {
        InterruptGuard guard;
        late_load_turn();
        Raw::dma_requests(false, true);
        if (count) {
            while (LateRxChannel::count() != 0u && turns < 20'000u) {
                ++turns;
            }
        } else {
            while (LateRxChannel::busy() && turns < 20'000u) {
                ++turns;
            }
        }
        first = *static_cast<volatile uint8_t*>(late_cells);
    }
    late_line(true);
    const uint32_t t1 = us_now();
    while (!transfer_done && us_now() - t1 < limit) {
    }
    if (!transfer_done) {
        (void)DmaHost::recover();
    }
    row.rows += 1u;
    row.stale += first != late_rx_value(0) ? 1u : 0u;
    row.turns_max = turns > row.turns_max ? turns : row.turns_max;
    late_rx_at = static_cast<uint8_t>(late_rx_at + 1u);
}

/// A byte standing in the FIFO over a stopped channel, then the data register read in
/// one of two orders: with IC_DMA_CR's request up, or taken down first. What is judged
/// is the next block's start with nothing on the wire: a beat there is a request a stop
/// left behind. A tenure of one, the vector's line off until the read is done.
void late_stray_row(bool request_up, I2cSpeed s, uint32_t& stood, uint32_t& strays, uint32_t& credits, uint32_t& next_wrong) {
    late_cells[0] = 0;
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(late_cells));
    r.rx_len = 1;
    r.speed = s;
    r.engine = I2cEngine::dma;
    const uint32_t limit = late_limit_us(1, s);
    late_line(false);
    transfer_done = false;
    (void)DmaHost::start(r);
    Raw::dma_requests(false, false);
    const uint32_t t_wait = us_now();
    while (Raw::rx_count() == 0u && us_now() - t_wait < limit) {
    }
    uint8_t read = 0;
    uint32_t banked = 0;
    {
        InterruptGuard guard;
        DmaRx::stop();
        DmaRx::arm(Raw::data_address(), Raw::dreq_rx, false, DmaReport::errors);
        if (request_up) {
            Raw::dma_requests(false, true);
            late_burn(20);
            banked = LateRxChannel::debug_credits();
            read = Raw::pop();
            Raw::dma_requests(false, false);
        } else {
            read = Raw::pop();
        }
    }
    stood += read == late_rx_value(0) ? 1u : 0u;
    credits = banked > credits ? banked : credits;
    late_line(true);
    const uint32_t t1 = us_now();
    while (!transfer_done && us_now() - t1 < limit) {
    }
    if (!transfer_done) {
        (void)DmaHost::recover();
    }
    late_rx_at = static_cast<uint8_t>(late_rx_at + 1u);
    // The next block's start, then the request up over an empty FIFO, as launch_dma() has it.
    static uint8_t scratch[2];
    (void)DmaRx::start(scratch, 2u);
    late_burn(100);
    const bool moved_down = LateRxChannel::count() != 2u;
    Raw::dma_requests(false, true);
    late_burn(100);
    const bool moved_up = LateRxChannel::count() != 2u;
    Raw::dma_requests(false, false);
    DmaRx::stop();
    DmaRx::arm(Raw::data_address(), Raw::dreq_rx, false, DmaReport::errors);
    strays += moved_down || moved_up ? 1u : 0u;
    // And a whole read behind it.
    LateRow behind{};
    late_read(0, 8, s, LateStage::none, 0, 0, behind);
    next_wrong += behind.exact == 1u ? 0u : 1u;
}

/// DO BANKED REQUESTS OUTLIVE IC_DMA_CR? A read of four with the channel paused from its
/// start and the vector's line off: the bytes stand and the requests bank; at STOP_DET the
/// count of them, the request down, the count again, then EN up with the request down: what
/// moves then moved on credit alone, and it leaves the FIFO by the same reads. Then the
/// host's body takes what stands.
void late_credits_row(I2cSpeed s, uint32_t& before, uint32_t& after, uint32_t& moved, uint32_t& exact) {
    for (uint32_t i = 0; i < 5u; ++i) {
        late_cells[i] = static_cast<uint8_t>(~late_rx_value(i));
    }
    DmaHost::Request r{};
    r.addr = twilink::dut_addr;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(late_cells));
    r.rx_len = 4;
    r.speed = s;
    r.engine = I2cEngine::dma;
    const uint32_t limit = late_limit_us(4, s);
    late_line(false);
    transfer_done = false;
    const uint32_t t0 = us_now();
    (void)DmaHost::start(r);
    LateRxChannel::enable(false);
    while ((Raw::raw_pending() & I2cInterrupt::stop_det) == 0u && us_now() - t0 < limit) {
    }
    uint32_t banked = 0;
    uint32_t left = 0;
    uint32_t remaining = 0;
    {
        InterruptGuard guard;
        banked = LateRxChannel::debug_credits();
        Raw::dma_requests(false, false);
        late_burn(20);
        left = LateRxChannel::debug_credits();
        LateRxChannel::enable(true);
        late_burn(100);
        remaining = LateRxChannel::count();
    }
    before = banked > before ? banked : before;
    after = left > after ? left : after;
    moved = 4u - remaining > moved ? 4u - remaining : moved;
    late_line(true);
    const uint32_t t1 = us_now();
    while (!transfer_done && us_now() - t1 < limit) {
    }
    if (!transfer_done) {
        (void)DmaHost::recover();
    }
    bool whole = transfer_done && transfer_status == i2c_ok;
    for (uint32_t i = 0; i < 4u; ++i) {
        whole = whole && late_cells[i] == late_rx_value(i);
    }
    exact += whole ? 1u : 0u;
    late_rx_at = static_cast<uint8_t>(late_rx_at + 4u);
    if (Raw::rx_count() != 0u) {
        (void)Raw::disable();
        Raw::enable();
    }
}

/// The shapes a stage runs: reads, a write-then-read.
struct LateShape {
    uint8_t tx_len;
    uint8_t rx_len;
};
constexpr LateShape late_shapes[] = {{0, 1}, {0, 2}, {0, 3}, {0, 8}, {0, 16}, {0, 17}, {0, 64}, {1, 8}};
constexpr uint8_t late_tails[] = {1, 2, 8, 16};

/// Every shape under every tail, each with a read of eight and a write of eight behind it.
void late_stage_row(LateStage stage, I2cSpeed s, uint16_t burns, uint8_t want, LateRow& row) {
    for (const uint8_t tail : late_tails) {
        for (const LateShape& shape : late_shapes) {
            late_read(shape.tx_len, shape.rx_len, s, stage, tail, burns, row, want, false);
            LateRow behind{};
            late_read(0, 8, s, LateStage::none, 0, 0, behind);
            row.next_wrong += behind.exact == 1u ? 0u : 1u;
            LateRow writes{};
            late_write(8, s, writes);
            row.next_wrong += writes.status_wrong;
        }
    }
}

uint16_t late_stage_bytes() {
    uint16_t bytes = 0;
    for (const LateShape& shape : late_shapes) {
        bytes = static_cast<uint16_t>(bytes + shape.tx_len + shape.rx_len + 8u + 8u);
    }
    return static_cast<uint16_t>(bytes * sizeof late_tails);
}

void late_row_print(const char* name, const LateRow& r) {
    print(serial, "    ", name, ": ", r.reads, " reads: exact ", r.exact, ", status wrong ", r.status_wrong, ", unanswered ", r.unanswered,
          "; the entry found the channel counting in ", r.late, ", at most ", r.owed_max, " beats, FIFO ", r.level_max, "; at the reply ",
          r.stale, " wrong (", r.stale_cells, " cells), running ", r.running, ", FIFO ", r.standing, "; moved after it ", r.moved,
          "; banked requests at the entry at most ", r.credits_max, "; the requests behind them wrong ", r.next_wrong, crlf);
}

void ts_late_stage() {
    const I2cSpeed speeds[3] = {I2cSpeed::standard_100k, I2cSpeed::fast_400k, I2cSpeed::fast_plus_1m};
    twilink::Params a{};
    uint32_t s0 = 0;
    twilink::Report r{};

    // ---- what says the last beat is in memory ----
    constexpr LateLoad copy_load{"", 2, true, LateSource::sram, false, late_block_max};
    if (!late_begin(800, 0x41, a, s0)) {
        return;
    }
    SaysRow says_busy{};
    SaysRow says_count{};
    for (uint16_t i = 0; i < 200u; ++i) {
        late_says_row(false, I2cSpeed::fast_400k, says_busy);
        late_says_row(true, I2cSpeed::fast_400k, says_count);
    }
    late_load_on(copy_load);
    for (uint16_t i = 0; i < 200u; ++i) {
        late_says_row(false, I2cSpeed::fast_400k, says_busy);
        late_says_row(true, I2cSpeed::fast_400k, says_count);
    }
    (void)late_load_off();
    bool rep = late_end(a, s0, r);
    print(serial, "  a channel polled to its end, the next load the cell, ", says_busy.rows, " rows of each, half of them under two HIGH copies: "
          "BUSY down stale ", says_busy.stale, " (at most ", says_busy.turns_max, " loads), TRANS_COUNT at zero stale ", says_count.stale,
          " (at most ", says_count.turns_max, " loads); the partner's tally ", rep ? "read" : "LOST", crlf);
    // No bound to derive: both registers' text, each row judged at its end.
    bench.verdict("BUSY DOWN AND TRANS_COUNT AT ZERO EACH SAY THE LAST BEAT IS IN MEMORY: the next load reads the byte",
                  says_busy.rows == 400u && says_busy.stale == 0u && says_count.rows == 400u && says_count.stale == 0u);

    // ---- a request left over a stopped channel ----
    if (!late_begin(static_cast<uint16_t>(400u * (1u + 8u)), 0x52, a, s0)) {
        return;
    }
    uint32_t stood_up = 0;
    uint32_t stray_up = 0;
    uint32_t credit_up = 0;
    uint32_t next_up = 0;
    uint32_t stood_down = 0;
    uint32_t stray_down = 0;
    uint32_t credit_down = 0;
    uint32_t next_down = 0;
    for (uint16_t i = 0; i < 200u; ++i) {
        late_stray_row(true, I2cSpeed::fast_400k, stood_up, stray_up, credit_up, next_up);
        late_stray_row(false, I2cSpeed::fast_400k, stood_down, stray_down, credit_down, next_down);
    }
    rep = late_end(a, s0, r);
    print(serial, "  IC_DATA_CMD read over a stopped channel, 200 rows of each: the request UP, the byte read in ", stood_up, ", credits banked at most ",
          credit_up, ", a beat at the next block's start ", stray_up, ", the read behind wrong ", next_up, "; the request DOWN, the byte in ",
          stood_down, ", a beat ", stray_down, ", the read behind wrong ", next_down, "; the partner's tally ", rep ? "read" : "LOST", crlf);
    // No bound to derive: 200 rows a run, each stray judged at zero. A beat there is the
    // stale credit begin() clears; the count not moving says the clear did its work.
    bench.verdict("NO STRAY BEAT: a data register read over a stopped channel, the request up or down, leaves the next block's "
                  "start where it was, and the read behind it exact",
                  stray_up == 0u && stray_down == 0u && next_up == 0u && next_down == 0u && stood_up == 200u && stood_down == 200u);

    // ---- banked requests against IC_DMA_CR ----
    if (!late_begin(static_cast<uint16_t>(100u * 4u), 0x71, a, s0)) {
        return;
    }
    uint32_t banked = 0;
    uint32_t banked_after = 0;
    uint32_t banked_moved = 0;
    uint32_t banked_exact = 0;
    for (uint16_t i = 0; i < 100u; ++i) {
        late_credits_row(I2cSpeed::fast_400k, banked, banked_after, banked_moved, banked_exact);
    }
    rep = late_end(a, s0, r);
    print(serial, "  a channel paused over a read of four: requests banked at STOP_DET at most ", banked, ", behind IC_DMA_CR going down at most ",
          banked_after, ", beats behind EN up with the request down at most ", banked_moved, " of 4; the read exact in ", banked_exact,
          " of 100; the partner's tally ", rep ? "read" : "LOST", crlf);
    // No bound to derive: the counts are the chapter's question and are printed. The verdict
    // is the host's order over them: every read exact in the vector whichever beats ran.
    bench.verdict("A BANKED REQUEST SPENT OR NOT, THE ENTRY TAKES WHAT STANDS: every read of four exact", banked_exact == 100u);

    // ---- a transmit block's end inside start() ----
    // Seventeen is the length whose block (fifteen beats and the first entry) fills the FIFO,
    // and 100 kHz the speed that leaves the first entry in it longest: dma_isr() then arms
    // TX_EMPTY for the last entry. Unstaged, a warm start() stores its mask first; on one run
    // of fifteen a read of 17 stalled there. The stage makes the order certain: TX_CMD_BLOCK
    // keeps the first entry in the FIFO (set with the host idle, as its register asks) and the
    // cache emptied ahead of start() stretches it past the block's end. The limit's margin is
    // 20 ms, the wire's 3.8 ms being in the rest, since nothing here holds SCL but the defect
    // itself; 100 rows of each, a read and a write. On the RP2040 a write of 17 stays on the
    // pump and a read's block ends behind the mask's store even so; the RAM image's start()
    // fetches nothing. So the arm's look is what this row judges on the RP2350's flash image.
    late_limit_margin_us = 20'000u;
    late_cold_start = true;
    late_inside_start = 0;
    if (!late_begin(static_cast<uint16_t>(200u * 17u), 0x7C, a, s0)) {
        return;
    }
    LateRow cold_reads{};
    LateRow cold_writes{};
    for (uint16_t i = 0; i < 100u; ++i) {
        late_read(0, 17, I2cSpeed::standard_100k, LateStage::none, 0, 0, cold_reads);
        late_write(17, I2cSpeed::standard_100k, cold_writes);
    }
    late_cold_start = false;
    late_limit_margin_us = 1'000'000u;
    rep = late_end(a, s0, r);
    print(serial, "  reads and writes of 17 at 100 kHz, TX_CMD_BLOCK and the XIP cache emptied over start(), 100 of each: a DMA line entry inside start() in ",
          late_inside_start, " of 200; reads exact ", cold_reads.exact, ", unanswered ", cold_reads.unanswered, "; writes unanswered ",
          cold_writes.unanswered, "; the partner's tally ", rep ? "read" : "LOST", ", ", r.sstatus, " off the pattern", crlf);
    // No bound to derive: a tenure answered or not, over the limit's arithmetic above.
    bench.verdict("A TRANSMIT BLOCK ENDING INSIDE start(): its arm for the last entry stands, every tenure answered",
                  cold_reads.unanswered == 0u && cold_writes.unanswered == 0u && cold_reads.exact == 100u && rep);

    // ---- the stages ----
    struct StageRow {
        const char* name;
        LateStage stage;
        uint16_t burns;
        uint8_t want;
    };
    const StageRow stages[] = {
        {"the request taken away for the tail", LateStage::away, 0, i2c_ok},
        {"the channel paused for the tail", LateStage::paused, 0, i2c_ok},
        {"the channel let go behind the reply", LateStage::behind, 0, i2c_ok},
        {"the channel let go in the entry, at once", LateStage::race, 0, i2c_ok},
        {"... 4 turns ahead of the body", LateStage::race, 4, i2c_ok},
        {"... 16 turns", LateStage::race, 16, i2c_ok},
        {"a byte nobody has", LateStage::steal, 0, i2c_dma_fault},
    };
    bool whole = true;
    for (const StageRow& st : stages) {
        print(serial, "  ", st.name, crlf);
        for (const I2cSpeed s : speeds) {
            if (!late_begin(late_stage_bytes(), 0x63, a, s0)) {
                return;
            }
            LateRow row{};
            late_watch.body_on_time = 0;
            late_watch.body_late = 0;
            late_stage_row(st.stage, s, st.burns, st.want, row);
            rep = late_end(a, s0, r);
            late_row_print(i2c_speed_hz(s) == 100'000u ? "100 kHz" : i2c_speed_hz(s) == 400'000u ? "400 kHz" : "1 MHz", row);
            print(serial, "      the host's body at most ", late_watch.body_on_time, " cycles on time, ", late_watch.body_late,
                  " late; the partner's tally: ", rep ? "read" : "LOST", ", served ", r.aux1, ", received ", r.aux0, ", ", r.sstatus,
                  " off the pattern", crlf);
            // 32 = 4 tails x 8 shapes. The stage's own miss is no failure, its read judged the
            // same, and the one shape it can miss on is the read of one, a tenure of 18 us at
            // 1 MHz against the thread's way out of a start() cold in the XIP cache: 4 rows of
            // the 32. Beyond those the stage has missed its point, except a race's, whose beats
            // may be done by the vector's first load.
            const bool stage_hit = st.stage == LateStage::race || row.late >= 28u;
            whole = whole && row.reads == 32u && row.exact == 32u && row.next_wrong == 0u && stage_hit && rep && r.sstatus == 0u;
        }
    }
    bench.verdict("EVERY STAGE: every byte at its place in the vector or the status that says otherwise, the channel stopped, "
                  "nothing moving after the reply, nothing left in the FIFO, the requests behind whole", whole);
}

// ---- the vector bodies ---------------------------------------------------------------

[[gnu::always_inline]] inline void pair_i2c_isr() {
    isr_entries = isr_entries + 1u;
    switch (owner) {
        case Owner::host:
            if (Host::isr()) {
                transfer_status = Host::status();
                transfer_done = true;
                if (bus_ao_live) {
                    post<kl::I2cArb>(TransferDone{Host::status()});
                }
            }
            break;
        case Owner::dma_host: {
            const bool watched = late_watch.on;
            uint32_t t0 = 0;
            if (watched) {
                late_before();
                t0 = late_cycles();
            }
            const bool ended = DmaHost::isr();
            if (watched && late_watch.stop) {
                const uint32_t took = late_cycles() - t0;
                if (late_watch.rx_busy) {
                    late_watch.body_late = took > late_watch.body_late ? took : late_watch.body_late;
                } else {
                    late_watch.body_on_time = took > late_watch.body_on_time ? took : late_watch.body_on_time;
                }
            }
            if (ended) {
                transfer_status = DmaHost::status();
                transfer_done = true;
            }
            break;
        }
        case Owner::client: client_serve(Client::service()); break;
        default:
            Raw::interrupts_only(0);
            Raw::clear_all();
            break;
    }
}

[[gnu::always_inline]] inline void pair_dma_isr() {
    dma_entries = dma_entries + 1u;
    if (owner == Owner::dma_host && DmaHost::dma_isr()) {
        transfer_status = DmaHost::status();
        transfer_done = true;
    }
}

// =============================================================================
// x - the hand-made window, traced (diagnostics, outside z)
// =============================================================================
void tx_window() {
    host_ready();
    Raw::clear_all();
    const bool sda0 = SdaPin::read();
    const bool scl0 = SclPin::read();
    inject_start();
    spin_us(10);
    const bool sda1 = SdaPin::read();
    const uint32_t raw1 = Raw::raw_pending();
    const uint32_t st1 = Raw::flags();
    inject_stop();
    spin_us(10);
    const bool sda2 = SdaPin::read();
    const uint32_t raw2 = Raw::raw_pending();
    print(serial, "  idle SDA ", sda0, " SCL ", scl0, "; held: SDA ", sda1, " raw ", hex(raw1), " status ", hex(st1),
          "; released: SDA ", sda2, " raw ", hex(raw2), crlf);
    // A tenure started inside the window, nobody racing it.
    Raw::clear_all();
    fill_pattern(tx_buf, 2, 0x10);
    Host::Request r{};
    r.addr = nobody;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(tx_buf));
    r.tx_len = 2;
    r.speed = link_speed;
    transfer_done = false;
    inject_start();
    spin_us(50);
    const uint32_t t0 = us_now();
    const bool sync = Host::start(r);
    const uint32_t start_us = us_now() - t0;
    uint8_t scl_falls = 0;
    bool prev = SclPin::read();
    const uint32_t t1 = us_now();
    while (us_now() - t1 < 300u) {
        const bool now = SclPin::read();
        if (prev && !now) {
            ++scl_falls;
        }
        prev = now;
    }
    const bool done_in_window = transfer_done;
    const uint32_t st2 = Raw::flags();
    inject_stop();
    const bool done = wait_until([] { return transfer_done; }, 200'000u);
    print(serial, "  start() inside the window: sync ", sync, " in ", start_us, " us; over the next 300 us SCL fell ",
          scl_falls, " times, done ", done_in_window, ", status ", hex(st2), "; after the STOP done ", done,
          " status ", transfer_status, crlf);
    bench.verdict("traced", true);
}
