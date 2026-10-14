// rp_uart_peer.hpp - THE uart_peer OF THE TWO RASPBERRY PI CHIPS, written
// once over the ARM PL011 (brio/pl011/uart.hpp) and bound by
// rp2040/src/apps/uart_peer.cpp and rp2350/src/apps/uart_peer.cpp: the far
// end of a serial link for the suite on the other board - a FIXED-ROLE
// firmware with no console, no kernel and no menu, whose ROLE is chosen by
// the board under test over a WIRE rather than by a command over a port.
//
// ONE BODY, TWO CHIPS: everything below is the block's and the role's;
// what is a chip's - the clock, the pads, the ruler, the mode line's read,
// the guard - the binding file states before it includes this one, inside
// its own anonymous namespace with `using namespace brio` in force:
//
//   SysClock, clock      the app's Clock type and object
//   Link                 Uart<1, ...> on the link's two pads
//   Led                  the board's LED as a Pin type
//   mode_line_up()       the mode line made an input, pulled down
//   mode_high()          the mode line read
//   us_now()             the microsecond ruler
//   link_pads_free()     may a link pad be driven now (the RP2350's
//                        partner-rail guard; true on the RP2040)
//
// THE LINK is the instance's two pads at 115200 8N1, crossed to the other
// board's UART. THE MODE LINE is an input pulled down, driven by the other
// board; the LED follows it.
//
//   mode line LOW   ECHO: every byte that arrives goes back out, unchanged
//                   and as fast as the rings allow. This is what proves a
//                   link byte-exact in both directions.
//   mode line HIGH  SEND: once per rising edge, 64 bytes of the pattern
//                   below go out and nothing is echoed (what arrives in
//                   the meantime waits in the ring, echoed once the line
//                   falls). This is what a receiver testing its own FRAME
//                   FORMAT against a known one needs - an echo cannot
//                   serve there, because an echo returns the bits it was
//                   given and a test of the wrong format has to be given
//                   bits it did not send.
//
// The pattern is the xorshift every stream tool of this repository
// generates: a 32-bit state seeded 0x12345678, the low byte of each step.
//
// THE LINK COMES UP ONLY WHILE ITS PADS MAY BE DRIVEN: the transmitter
// idles HIGH on its pad from the bring-up on, so where the binding's guard
// reads the partner unpowered the instance stays in reset and the guard is
// asked again once a second; once up it stays up.
//
// THE COST, per byte echoed: the transport's interrupt receiver and its
// transmitter (docs/rp2040/uart.md, docs/rp2350/uart.md), and between them
// one run moved a loop turn - `read_bulk` of up to 64 bytes into a buffer
// on the stack and `write_bulk` of the same run back, two copies, no call
// per byte. At 115200 that is far inside the wire's 86.8 us a frame.
//
// Nothing here is specific to the chip at the other end: it is a serial
// peer, and the board under test may be of any family.

/// The pattern both ends of every stream of this repository generate.
struct Lfsr {
    uint32_t state = 0x12345678u;
    uint8_t next() {
        uint32_t s = state;
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        state = s;
        return static_cast<uint8_t>(s & 0xFFu);
    }
};

constexpr uint32_t link_baud = 115200;

void send_burst() {
    Lfsr tx;
    uint8_t out[64];
    for (uint8_t& b : out) {
        b = tx.next();
    }
    (void)Link::write_bulk(out);
}

bool link_up() { return link_pads_free() && Link::init(clock, link_baud); }

[[noreturn]] void peer_loop() {
    mode_line_up();
    Led::output(false);
    // The first bring-up before the interrupts are on, as init() asks; a
    // later one, the guard having refused this one, runs under them -
    // init() takes its own line down first.
    bool link_ok = link_up();
    enable_interrupts();
    uint32_t asked = us_now();
    bool sending = false;
    for (;;) {
        const bool want_send = mode_high();
        Led::duty(want_send ? 1u : 0u);
        if (!link_ok) {
            if (us_now() - asked >= 1'000'000u) {
                asked = us_now();
                link_ok = link_up();
            }
            continue;
        }
        if (want_send && !sending) {
            // The rising edge: one burst, then silence until it falls.
            sending = true;
            send_burst();
        } else if (!want_send) {
            sending = false;
            uint8_t buf[64];
            const uint32_t n = Link::read_bulk(buf);
            if (n != 0u) {
                (void)Link::write_bulk(std::span<const uint8_t>(buf, n));
            }
        }
    }
}
