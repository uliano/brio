// rp_i2c_pair.hpp - THE TWO-BOARD I2C LETTERS OF THE TWO RASPBERRY PI
// CHIPS, written once over the Synopsys DW_apb_i2c (brio/dw_apb_i2c/
// i2c.hpp) and included by test_rp2040_i2c and test_rp2350_i2c: this chip
// the DUT and the bus host, the desk partner running `twi_peer`
// (rp_twi_peer.hpp) commanded IN BAND over the very bus under test through
// avrdx/src/apps/twi_link.hpp - the RP2040 against the RP2350's peer, the
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
//   SdaPin, SclPin               the link's pads
//   us_now()                     the microsecond ruler (the system timer)
//   xip_empty()                  every line of the XIP cache dropped (the
//                                family's Xip verb): letter g's cold read
//   partner_kind, partner_text   twi_link's kind byte of the peer, its name
//
// and binds its I2C vector to `pair_i2c_isr()` and isr_dma_0 to
// `pair_dma_isr()`.
//
// THE LINK'S RULES (docs/boards/weact-rp2040.md, weact-rp2350b.md): the
// three pull-ups are 1.5 kOhm to the RP2040's 3V3; the partner runs
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

void client_reset_tally() {
    client_given = 0;
    client_count = 0;
    client_stops = 0;
    client_restarts = 0;
    client_nacks = 0;
    client_flushes = 0;
    client_errors = 0;
    client_gcalls = 0;
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
        default: break;
    }
}

/// This board as a client at `addr` (the general call answered or not),
/// its events on its vector.
bool client_ready(uint8_t addr, bool general_call) {
    owner = Owner::none;
    client_reset_tally();
    const bool ok = Client::init(clock, {.address = addr, .general_call = general_call, .speed = I2cSpeed::fast_plus_1m});
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

/// One tenure through host H, waited out. The status, or no_answer for a
/// tenure that never answered within `wait_us` (the engine recover()ed).
template <typename H>
uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
               I2cSpeed speed = link_speed, uint32_t wait_us = 200'000u) {
    typename H::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    transfer_done = false;
    isr_entries = 0;
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
    // IC_TAR and enable a new address costs (6 us at 100 kHz,
    // docs/design/benchmark.md's letter i), start(), the one handler and the
    // wait's own loop (under 2000 cycles, 16 us at 125 MHz), the rise at
    // 100 kHz (under 1 us over 9.4 periods) and the ruler's microsecond -
    // 94 to 118 us.
    bench.verdict("nobody home is i2c_nack_addr, answered at the address's NACK: 94 to 118 us at 100 kHz",
                  nacks == 111u && nobody_us >= 94u && nobody_us <= 118u);
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

    // The general call: the host's tenure to address 0x00.
    twilink::Params g = serve_params(300, 0);
    g.flags = twilink::flag_general_call;
    if (peer_act(Op::serve, g)) {
        const uint32_t g0 = us_now();
        uint8_t gw[2] = {0x5A, 0xA5};
        const uint8_t gs = tenure<Host>(0x00, gw, 2, nullptr, 0);
        outlast(g, g0);
        twilink::Report gr{};
        const bool grep = peer_report(gr);
        print(serial, "  general call: status ", gs, ", the peer's last address ", hex(gr.last_addr), ", received ",
              gr.aux0, ", tenures ", gr.addr_hits, ", stops ", gr.stops, crlf);
        bench.verdict("a GENERAL CALL write (the Request's address 0x00, IC_TAR's SPECIAL clear) reaches the "
                      "peer's client: one tenure at 0x00, its two bytes taken, and no STOP_DET for it (IC_CON's "
                      "note on STOP_DET_IFADDRESSED)",
                      gs == i2c_ok && grep && gr.aux0 == 2u && gr.addr_hits == 1u && gr.last_addr == 0x00u &&
                          gr.stops == 0u);
    } else {
        bench.verdict("the peer accepted the general-call serve", false);
    }
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
    print(serial, "  the peer deaf at 0x7A: write ", deaf_w, ", probe ", deaf_p, ", read ", deaf_r, crlf);
    bench.verdict("an address nobody answers is i2c_nack_addr on a write, the probe and a read - on a board "
                  "that is otherwise alive",
                  deaf_w == i2c_nack_addr && deaf_p == i2c_nack_addr && deaf_r == i2c_nack_addr);

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
    uint32_t t0 = us_now();
    const uint8_t st0 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 64, I2cSpeed::fast_400k, 20'000u);
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
    const uint8_t st = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 64, I2cSpeed::fast_400k, 20'000u);
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
    const uint8_t stc = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 64, I2cSpeed::fast_400k, 20'000u);
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
    const uint8_t st2 = tenure<DmaHost>(twilink::dut_addr, w2, 2, rx_buf, 32, I2cSpeed::fast_400k);
    const uint8_t m32 = pattern_mismatches(rx_buf, 32, a.seed, served);
    served = static_cast<uint16_t>(served + 32u);
    bench.verdict("a write-then-read on an engined host: the write on the pump, the repeated START, 32 bytes "
                  "back on the engines, exact",
                  st2 == i2c_ok && m32 == 0u);
    const uint8_t st3 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 2, I2cSpeed::fast_400k);
    const uint8_t m2 = pattern_mismatches(rx_buf, 2, a.seed, served);
    served = static_cast<uint16_t>(served + 2u);
    const uint8_t st4 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, nullptr, 0, I2cSpeed::fast_400k);
    served = static_cast<uint16_t>(served + 1u);
    const uint8_t st5 = tenure<DmaHost>(nobody, nullptr, 0, rx_buf, 16, I2cSpeed::fast_400k);
    bench.verdict("a two-byte read and the probe stay on the pump of an engined host; a deaf client on an "
                  "engined read is i2c_nack_addr with the engines put away",
                  st3 == i2c_ok && m2 == 0u && st4 == i2c_ok && st5 == i2c_nack_addr);
    const uint8_t st6 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 16, I2cSpeed::fast_400k);
    const uint8_t m16 = pattern_mismatches(rx_buf, 16, a.seed, served);
    served = static_cast<uint16_t>(served + 16u);
    bench.verdict("... and the next engined read after that is clean", st6 == i2c_ok && m16 == 0u);
    // The bursts: the request is a level the DMA banks credits on while the
    // FIFO is empty; a burst past the FIFO's depth would drop commands
    // (TX_OVER) and shorten the tenure. Eight reads of 255 at 1 MHz.
    uint8_t exact = 0;
    bool over = false;
    for (uint8_t k = 0; k < 8u; ++k) {
        for (uint16_t i = 0; i < 255u; ++i) {
            rx_buf[i] = 0xEE;
        }
        const uint8_t s7 = tenure<DmaHost>(twilink::dut_addr, nullptr, 0, rx_buf, 255, I2cSpeed::fast_plus_1m, 50'000u);
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
    owner = Owner::none;
    DmaHost::release();
    host_ready();
    outlast(a, s0);
    twilink::Report r{};
    const bool rep = peer_report(r);
    print(serial, "  the peer served ", r.aux1, " (low byte of ", served, ")", crlf);
    bench.verdict("the peer's tally agrees: every byte the engines read was served once",
                  rep && r.aux1 == static_cast<uint8_t>(served));
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
uint8_t arb_tenure(uint8_t addr, const uint8_t* tx, uint8_t len, uint32_t lead_us) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = len;
    r.speed = link_speed;
    transfer_done = false;
    isr_entries = 0;
    inject_start();
    spin_us(lead_us);
    const bool sync = Host::start(r);
    spin_us(20);
    inject_stop();
    if (sync) {
        return Host::status();
    }
    if (!wait_until([] { return transfer_done; }, 200'000u)) {
        (void)Host::recover();
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
        // lead, with its start() - a disable, IC_TAR and an enable, 6 us at
        // 100 kHz - well inside.
        const uint8_t st = arb_tenure(rd.dut_target, tx_buf, 5, 350);
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

    // ---- the client's general call ----
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
    print(serial, "  the GENERAL CALL to this board's client: init ", up3, ", general-call events ", gcalls,
          ", bytes ", gheard, " (", gmism, " wrong), stops ", gstops, "; the peer's host status ", gr.mstatus, crlf);
    bench.verdict("THE CLIENT'S GENERAL CALL - with IC_ACK_GENERAL_CALL on, a real host's write to 0x00 is "
                  "acknowledged, reported ONCE as general_call (GEN_CALL masked through the address's "
                  "acknowledge), its three bytes taken, and no stop event (IC_CON's note)",
                  up3 && gcalls == 1u && gheard == 3u && gmism == 0u && gstops == 0u && grep && gr.mstatus == i2c_ok);
    bench.verdict("the command channel is back after the role swaps", command(Op::ping));
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
        case Owner::dma_host:
            if (DmaHost::isr()) {
                transfer_status = DmaHost::status();
                transfer_done = true;
            }
            break;
        case Owner::client: client_serve(Client::service()); break;
        default:
            Raw::interrupts_only(0);
            Raw::clear_all();
            break;
    }
}

[[gnu::always_inline]] inline void pair_dma_isr() {
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
