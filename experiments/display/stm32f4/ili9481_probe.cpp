// ili9481_probe - the 3.5" ILI9481 module on the STM32F411CE black pill,
// driven over SPI1 and judged by READING ITS FRAME MEMORY BACK.
//
// THE QUESTION THIS APP ANSWERS is not "does the panel show colours" but
// "can the program read what it wrote": a panel whose GRAM answers is
// the bench half of design/gfx.md's three planes of truth - a driver
// under development and every drawing primitive above it can be judged
// pixel for pixel against what the silicon holds, with no eye and no
// camera. So the letters establish, in order, that the controller
// answers, in WHAT FORMAT its serial read path returns bytes (a dummy
// read is a byte or a clock, the datasheet's figure does not say for
// this interface), that the read continues across commands and rows,
// what the write and read rates cost, and that a drawing made through
// brio/gfx reads back as the font says it should.
//
// ILI9481 facts used (the datasheet's 7.2 and chapter 8):
//   - native 320 columns x 480 pages, MADCTL 0x00 at reset
//   - DBI Type C, option 3 (four wires, D/CX): a command byte with D/CX
//     low, parameters and data with it high; SDA_EN = 0 at reset, so
//     the answers leave on DOUT (the module's SDO) and DIN is one way
//   - the serial interface carries 18-bit pixels ONLY: three bytes a
//     pixel, each colour in the six MSBs of its byte (COLMOD 0x66, the
//     reset default); the two LSBs are ignored on a write
//   - RAMRD (2Eh) and read_memory_continue (3Eh) return 24 bits a pixel
//     "regardless of the pixel format", after a first parameter the
//     datasheet calls a dummy read; read data is valid on the rising
//     edge of SCL with D/CX high
//   - the serial clock's limits (13.3.2): 40 ns high + 40 ns low for a
//     write (12.5 MHz), 120 + 120 ns for a read (4.17 MHz)
//   - these panels want INVON or every colour is its complement (the
//     AVR half of this experiment found it; the GRAM is unaffected)
//   - THIS MODULE scans its columns right to left and is wired BGR
//     (seen on the glass, not in the GRAM): MADCTL's B1 flips the
//     display back and the driver writes B, G, R
//   - MADCTL's B5/B6/B7 reorder the address counter's walk INSIDE the
//     window and never move the origin (letter g), so a rotation of the
//     logical surface is a window mapping plus those bits, and a read
//     through the same window comes back in logical order
//
// THE CLOCK IS 96 MHz, not the part's 100: the console is the board's own
// USB-C and the OTG controller wants 48 MHz on the main PLL's Q output.
// PCLK2 is 96 MHz too, so SPI1's ladder is 48, 24, 12, 6, 3, 1.5 ... MHz:
// the default write rate is /16 = 6 MHz, the default read rate /32 =
// 3 MHz, both inside the datasheet, and two letters climb from there.
//
// The bus is driven SYNCHRONOUSLY - polled requests straight into
// SpiHost::start(), the arbiter skipped, spi-bus.md's own allowance for
// a device that owns its bus alone - because a probe reads better as a
// sequence than as a state machine.
//
// What is exercised, letter by letter:
//   a  the controller answers: RDDID, RDDST, RDDPM, RDDMADCTL,
//      RDDCOLMOD and the device code, raw; 94 81 found in the device
//      code and where in the stream it sits
//   b  THE READ FORMAT: a block of known pixels written, its GRAM read
//      back raw, and the byte offset and bit shift that make the stream
//      match found by search - stored for every later letter
//   c  read_memory_continue (3Eh) picks up where RAMRD stopped
//   d  the whole frame written with a pattern of (x, y), read back row
//      by row and compared byte for byte; both phases timed
//   k  the whole frame through SPI1's two DMA engines at 6 and 12 MHz,
//      read back and timed: the wire's rate against the pump's
//   e  the write rate ladder above the default, judged by read-back
//   f  the read rate ladder above the default, judged the same way
//   g  the eight scan orders of MADCTL, each written and read back
//   h  brio/gfx over the panel: "brio" drawn and read back against the
//      font under each of the four rotations, then the picture in
//      landscape through the Surface verbs - the end state the panel keeps
//   o  (by name) the next rotation, a marker picture for the eye
//   r  (by name) hardware reset and the wake sequence again
//   i  (by name) toggle the inversion, to see the panel's polarity
//
// Wiring (the module has no level shifter: the board's 3.3 V rail):
//   SPI1  SCK PA5, MISO PA6 (the module's SDO), MOSI PA7 (SDI), AF5
//   CS PB2 (BOOT1 - a plain output once booted), RESET PB1, DC/RS PB0
//   VCC and LED (the backlight) on 3.3 V, GND on GND; the touch
//   controller on the same three wires with its select on PA4, held
//   HIGH here (a floating select lets its DOUT corrupt every read)
//   the console is the USB-C connector (CDC ACM, 1209:0001), the probe
//   an STLINK-V3 on the SWD header
//
// build: boards = f411ce
// build: monitor_speed = 115200

#include <stdint.h>

#include <array>
#include <optional>
#include <span>
#include <string_view>

#include "gfx/draw.hpp"
#include "gfx/font_5x7.hpp"
#include "gfx/surface.hpp"
#include "gfx/text.hpp"
#include "kernel/borrowed.hpp"
#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "stm32f4/platform.hpp"
#include "stm32f4/spi.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/usb.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"
#include "util/usb/cdc.hpp"
#include "util/usb/device.hpp"

namespace {

using namespace brio;

using P = Stm32f4Platform<>;
using SysClock = Clock<ClockSource::pll_hse, 96'000'000, 25'000'000>;
constexpr SysClock clock;
static_assert(SysClock::usb_hz == otg_clock_hz, "this rate must give the controller 48 MHz");

// ---- the console: the chip's own USB ------------------------------------------
using Usb = UsbFs;
using Serial = UsbCdcAcm<Usb, P, 0, 1, 2, 512, 2048>;   // a deep transmit ring: the dumps
constexpr Serial serial;

struct Descriptors {
    static constexpr auto device =
        usb_device_descriptor({.vendor_id = 0x1209, .product_id = 0x0001, .device_class = 2});
    static constexpr auto configuration =
        usb_concat(usb_configuration_head(9 + Serial::descriptor_bytes, Serial::interface_count),
                   Serial::descriptors);
    static constexpr auto language = usb_language_descriptor();
    static constexpr auto manufacturer = usb_string_descriptor("brio");
    static constexpr auto product = usb_string_descriptor("brio console");
    static constexpr auto serial_number = usb_string_descriptor("stm32f4");
    static std::span<const uint8_t> string(uint8_t index) {
        switch (index) {
        case 0: return language;
        case 1: return manufacturer;
        case 2: return product;
        case 3: return serial_number;
        default: return {};
        }
    }
};
using Device = UsbDevice<Usb, Descriptors, Serial>;

// ---- the wires ------------------------------------------------------------------
using Led = Pin<'C', 13>;   // lit when low
using Cs = Pin<'B', 2>;
using Rst = Pin<'B', 1>;
using Dc = Pin<'B', 0>;
/// The module's touch controller shares MISO: its select must be HIGH
/// or its DOUT talks over the panel's answers (measured: garbage in
/// every read with PA4 left floating).
using Tcs = Pin<'A', 4>;
/// A logic analyser's trigger: high around the first transaction of the
/// letters p, q and j.
using Trigger = Pin<'B', 10>;

constexpr SpiPins spi_pins{
    .sck = {'A', 5, PinFunction::af5},
    .miso = {'A', 6, PinFunction::af5},
    .mosi = {'A', 7, PinFunction::af5},
};
using Host = SpiHost<1, spi_pins>;
// The same instance under the two DMA engines RM0383 table 28 wires to
// SPI1: the transmit request on DMA2 stream 3 channel 3, the receive on
// DMA2 stream 0 channel 3. Letter k brings this host up in place of the
// pump one and hands the bus back afterwards.
using TxEngine = DmaTxEngine<2, 3, 3>;
using RxEngine = DmaRxEngine<2, 0, 3>;
using DmaHost = SpiHost<1, spi_pins, TxEngine, RxEngine>;
bool dma_host_live = false;

TestBench<Serial, 32> bench;

// ---- printing helpers -----------------------------------------------------------
void hex2(uint8_t b) {
    static constexpr char digits[] = "0123456789ABCDEF";
    print(serial, digits[b >> 4], digits[b & 0x0Fu]);
}

void dump(const uint8_t* p, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        hex2(p[i]);
        print(serial, ' ');
    }
}

void wait_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::millis();
    while (Ticker::millis() - t0 < ms) {
    }
}

// ---- the panel ------------------------------------------------------------------
namespace ili {

constexpr uint16_t width = 320;
constexpr uint16_t height = 480;
constexpr uint16_t row_bytes = width * 3u;

constexpr uint8_t rddid = 0x04;
constexpr uint8_t rddst = 0x09;
constexpr uint8_t rddpm = 0x0A;
constexpr uint8_t rddmadctl = 0x0B;
constexpr uint8_t rddcolmod = 0x0C;
constexpr uint8_t slpout = 0x11;
constexpr uint8_t invoff = 0x20;
constexpr uint8_t invon = 0x21;
constexpr uint8_t dispon = 0x29;
constexpr uint8_t caset = 0x2A;
constexpr uint8_t paset = 0x2B;
constexpr uint8_t ramwr = 0x2C;
constexpr uint8_t ramrd = 0x2E;
constexpr uint8_t madctl = 0x36;
constexpr uint8_t ramwr_continue = 0x3C;
constexpr uint8_t ramrd_continue = 0x3E;
constexpr uint8_t devcode = 0xBF;

/// The two rates letters e and f move; the datasheet's ceilings are
/// 12.5 MHz for a write and 4.17 MHz for a read, so at PCLK2 = 96 MHz
/// these are /16 = 6 MHz and /32 = 3 MHz.
SpiClock write_rate = SpiClock::div16;
SpiClock read_rate = SpiClock::div32;

/// How the read stream lines up: the byte offset of the first pixel
/// byte and the bit shift of every byte - found by letter b, assumed
/// until then to be one dummy byte and no shift.
struct ReadFormat {
    uint8_t offset = 1;
    uint8_t shift = 0;
};
ReadFormat read_format{};
bool read_format_known = false;

/// The format in force, said once by every letter that leans on it.
void note_format() {
    print(serial, "  (read format: pixel bytes from byte ", read_format.offset, ", shift ", read_format.shift,
          read_format_known ? ", found by letter b)" : ", ASSUMED - run letter b first)", crlf);
}
/// Raw bytes clocked past what a read needs: room for the offset and
/// for a shift's spill.
constexpr uint8_t read_extra = 4;

bool inverted = true;

/// TWO FACTS OF THIS MODULE, seen on the glass and not in the GRAM: the
/// panel shows COLUMN 319 AT THE LEFT (MADCTL's B1, the display's own
/// horizontal flip, does nothing here - measured), and it is wired BGR.
/// The mirror is answered in the rotation map below; the colour order in
/// the byte order the driver writes, not in MADCTL's BGR bit, because
/// that bit acts on writes alone (letter g) and would leave the
/// read-back in the other order.
constexpr uint8_t madctl_panel = 0x00;
constexpr bool bgr = true;
constexpr bool column_319_at_left = true;

/// The rotation of the LOGICAL surface over the glass. In DISPLAY
/// coordinates (d left to right, p top to bottom) a rotation is the
/// usual one; the memory column is 319 - d on this module. A run of the
/// logical surface (one row, x increasing) must be one contiguous walk
/// of the address counter, so each rotation states the MADCTL walk bits
/// it needs: B5 makes the counter step the pages first, B6 reverses its
/// walk along the columns and B7 along the pages - all three INSIDE THE
/// WINDOW, never moving the origin (letter g). A run is one row or one
/// column of the window, so only the bit along the run matters, and the
/// bits across it are left clear.
enum class Rotation : uint8_t { r0 = 0, r90 = 1, r180 = 2, r270 = 3 };
Rotation rotation = Rotation::r0;

constexpr uint8_t walk_bits(Rotation r) {
    switch (r) {
        case Rotation::r0: return column_319_at_left ? 0x40 : 0x00;    // the run walks the columns down
        case Rotation::r90: return 0x20;                                // pages first, upward count
        case Rotation::r180: return column_319_at_left ? 0x00 : 0x40;
        case Rotation::r270: return 0x20 | 0x80;                        // pages first, downward count
    }
    return 0x00;
}
constexpr uint8_t madctl_for(Rotation r) { return static_cast<uint8_t>(madctl_panel | walk_bits(r)); }
constexpr bool exchanged(Rotation r) { return r == Rotation::r90 || r == Rotation::r270; }

/// One transaction on host `H`: the command byte with D/C low, then
/// `len` bytes of data with it high - `tx` out (null = 0xFF dummies) and
/// `rx` in (null = discarded). Polled: complete on return.
template <typename H>
bool start_on(uint8_t cmd, const uint8_t* tx, uint8_t* rx, uint16_t len, SpiClock rate) {
    static uint8_t cmd_byte = 0;   // lent to the request, which completes inside start()
    cmd_byte = cmd;
    typename H::Request r{};
    r.cs = Cs::ref();
    r.dc = Dc::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&cmd_byte));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.bits = SpiDataSize::bits8;
    r.polled = true;
    return H::start(r);
}

/// The transaction on the host in force: the pump one, or the engined
/// one while letter k holds the bus.
bool transfer(uint8_t cmd, const uint8_t* tx, uint8_t* rx, uint16_t len, SpiClock rate) {
    if (dma_host_live) {
        return start_on<DmaHost>(cmd, tx, rx, len, rate);
    }
    return start_on<Host>(cmd, tx, rx, len, rate);
}

void command(uint8_t c) { (void)transfer(c, nullptr, nullptr, 0, write_rate); }

void command(uint8_t c, const uint8_t* params, uint16_t n) {
    (void)transfer(c, params, nullptr, n, write_rate);
}

void window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t p[4];
    p[0] = static_cast<uint8_t>(x0 >> 8);
    p[1] = static_cast<uint8_t>(x0);
    p[2] = static_cast<uint8_t>(x1 >> 8);
    p[3] = static_cast<uint8_t>(x1);
    command(caset, p, 4);
    p[0] = static_cast<uint8_t>(y0 >> 8);
    p[1] = static_cast<uint8_t>(y0);
    p[2] = static_cast<uint8_t>(y1 >> 8);
    p[3] = static_cast<uint8_t>(y1);
    command(paset, p, 4);
}

void set_madctl(uint8_t code) { command(madctl, &code, 1); }

void set_rotation(Rotation r) {
    rotation = r;
    set_madctl(madctl_for(r));
}

constexpr uint16_t logical_width(Rotation r) { return exchanged(r) ? height : width; }
constexpr uint16_t logical_height(Rotation r) { return exchanged(r) ? width : height; }

/// The window for a LOGICAL rectangle under the rotation in force: the
/// physical column and page ranges it covers, handed to CASET and PASET
/// in the order the walk bits expect (CASET takes the pages once B5 has
/// exchanged the axes - measured, letter g).
void window_logical(uint16_t lx, uint16_t ly, uint16_t w, uint16_t h) {
    // Display coordinates of the rectangle first, then the memory
    // column of a display column d is 319 - d on this module.
    uint16_t d0 = 0, d1 = 0, p0 = 0, p1 = 0;
    switch (rotation) {
        case Rotation::r0:
            d0 = lx; d1 = static_cast<uint16_t>(lx + w - 1u);
            p0 = ly; p1 = static_cast<uint16_t>(ly + h - 1u);
            break;
        case Rotation::r90:   // clockwise: logical x runs down the glass, logical y leftward
            d0 = static_cast<uint16_t>(width - ly - h); d1 = static_cast<uint16_t>(width - 1u - ly);
            p0 = lx; p1 = static_cast<uint16_t>(lx + w - 1u);
            break;
        case Rotation::r180:
            d0 = static_cast<uint16_t>(width - lx - w); d1 = static_cast<uint16_t>(width - 1u - lx);
            p0 = static_cast<uint16_t>(height - ly - h); p1 = static_cast<uint16_t>(height - 1u - ly);
            break;
        case Rotation::r270:  // counter-clockwise: logical x runs up the glass, logical y rightward
            d0 = ly; d1 = static_cast<uint16_t>(ly + h - 1u);
            p0 = static_cast<uint16_t>(height - lx - w); p1 = static_cast<uint16_t>(height - 1u - lx);
            break;
    }
    uint16_t c0 = d0, c1 = d1;
    if (column_319_at_left) {
        c0 = static_cast<uint16_t>(width - 1u - d1);
        c1 = static_cast<uint16_t>(width - 1u - d0);
    }
    if (exchanged(rotation)) {
        window(p0, c0, p1, c1);   // CASET takes the pages once B5 has exchanged the axes
    } else {
        window(c0, p0, c1, p1);
    }
}

void hard_reset() {
    Rst::clear();
    wait_ms(10);
    Rst::set();
    wait_ms(150);
}

/// SLPOUT, DISPON, the inversion: the sequence the AVR half proved.
void wake() {
    command(slpout);
    wait_ms(150);
    command(dispon);
    wait_ms(25);
    command(inverted ? invon : invoff);
    set_rotation(rotation);
}

/// A read register's raw answer, `n` bytes clocked after the command.
void read_raw(uint8_t cmd, uint8_t* raw, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        raw[i] = 0;
    }
    (void)transfer(cmd, nullptr, raw, n, read_rate);
}

/// The stream as the format says: byte `offset + i`, shifted left by
/// `shift` with the next byte's top bits pulled in. False when the raw
/// buffer is too short for it.
bool realign(const uint8_t* raw, uint16_t n_raw, uint8_t* out, uint16_t n, ReadFormat f) {
    const uint16_t need = static_cast<uint16_t>(f.offset + n + (f.shift != 0u ? 1u : 0u));
    if (need > n_raw) {
        return false;
    }
    for (uint16_t i = 0; i < n; ++i) {
        uint8_t b = raw[f.offset + i];
        if (f.shift != 0u) {
            b = static_cast<uint8_t>((b << f.shift) | (raw[f.offset + i + 1u] >> (8u - f.shift)));
        }
        out[i] = b;
    }
    return true;
}

/// Bytes of `a` and `b` that differ under `mask`; the first one's index
/// in `first` (n when none).
uint32_t mismatches(const uint8_t* a, const uint8_t* b, uint32_t n, uint16_t& first, uint8_t mask = 0xFC) {
    uint32_t bad = 0;
    first = static_cast<uint16_t>(n);
    for (uint32_t i = 0; i < n; ++i) {
        if (((a[i] ^ b[i]) & mask) != 0u) {
            if (bad == 0u) {
                first = static_cast<uint16_t>(i);
            }
            ++bad;
        }
    }
    return bad;
}

/// A pixel's colour from its position: every byte's six MSBs move with
/// x and y, so a stream one byte out of step disagrees at once. 0x00RRGGBB.
constexpr uint32_t pattern(uint16_t x, uint16_t y, uint32_t salt = 0) {
    uint32_t v = static_cast<uint32_t>(x) * 0x9E3779B1u;
    v ^= (static_cast<uint32_t>(y) + 1u) * 0x85EBCA77u;
    v ^= salt;
    v ^= v >> 15;
    v *= 0x2C1B3C6Du;
    v ^= v >> 12;
    return v & 0x00FCFCFCu;
}

void put_pixel(uint8_t* row, uint16_t i, uint32_t rgb) {
    row[i * 3u] = static_cast<uint8_t>(bgr ? rgb : rgb >> 16);
    row[i * 3u + 1u] = static_cast<uint8_t>(rgb >> 8);
    row[i * 3u + 2u] = static_cast<uint8_t>(bgr ? rgb >> 16 : rgb);
}

/// A LOGICAL run is up to the LONG side of the panel wide once the
/// surface is rotated, so the buffers are sized by that side and not by
/// the physical row - what sits after them in RAM is the console's own
/// state, and a row of the rotated surface overran it once.
constexpr uint16_t run_pixels_max = height > width ? height : width;
constexpr uint16_t run_bytes_max = run_pixels_max * 3u;
uint8_t row_buf[run_bytes_max + 4];
uint8_t raw_buf[run_bytes_max + read_extra + 4];
uint8_t scratch[run_bytes_max + 4];

/// Write a `w` x `h` block at (x, y) with pattern(x, y, salt): RAMWR
/// for the first row, write_memory_continue for the rest.
void write_pattern(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t salt, SpiClock rate) {
    window(x, y, static_cast<uint16_t>(x + w - 1u), static_cast<uint16_t>(y + h - 1u));
    for (uint16_t r = 0; r < h; ++r) {
        for (uint16_t c = 0; c < w; ++c) {
            put_pixel(row_buf, c, pattern(static_cast<uint16_t>(x + c), static_cast<uint16_t>(y + r), salt));
        }
        (void)transfer(r == 0u ? ramwr : ramwr_continue, row_buf, nullptr, static_cast<uint16_t>(w * 3u), rate);
    }
}

/// Read a `w` x `h` block's bytes into `out` (w*h*3 bytes), realigned by
/// the format in force. The window is set first; the read is ONE RAMRD
/// when the block fits a row buffer, else one per row.
bool read_block(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t* out, SpiClock rate) {
    const uint16_t bytes = static_cast<uint16_t>(w * 3u);
    for (uint16_t r = 0; r < h; ++r) {
        window(x, static_cast<uint16_t>(y + r), static_cast<uint16_t>(x + w - 1u), static_cast<uint16_t>(y + r));
        for (uint16_t i = 0; i < bytes + read_extra; ++i) {
            raw_buf[i] = 0;
        }
        (void)transfer(ramrd, nullptr, raw_buf, static_cast<uint16_t>(bytes + read_extra), rate);
        if (!realign(raw_buf, static_cast<uint16_t>(bytes + read_extra), out + static_cast<uint32_t>(r) * bytes,
                     bytes, read_format)) {
            return false;
        }
    }
    return true;
}

/// Compare a block against pattern(x, y, salt): mismatched bytes.
uint32_t check_pattern(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t salt, const uint8_t* got,
                       uint16_t& first) {
    for (uint16_t r = 0; r < h; ++r) {
        for (uint16_t c = 0; c < w; ++c) {
            put_pixel(scratch + static_cast<uint32_t>(r) * w * 3u, c,
                      pattern(static_cast<uint16_t>(x + c), static_cast<uint16_t>(y + r), salt));
        }
    }
    return mismatches(scratch, got, static_cast<uint32_t>(w) * h * 3u, first);
}

}  // namespace ili

// ---- the panel as a gfx Surface ---------------------------------------------------
/// 24-bit colour in a uint32_t (0x00RRGGBB); the panel keeps six bits
/// of each.
struct Rgb888 {
    using Color = uint32_t;
    static constexpr uint8_t bits = 24;
    static constexpr Color max_color = 0x00FFFFFFu;
};

class PanelSurface {
public:
    using Format = Rgb888;
    using Color = uint32_t;

    static Extent width() { return ili::logical_width(ili::rotation); }
    static Extent height() { return ili::logical_height(ili::rotation); }

    void fill_rect(Coord x, Coord y, Extent w, Extent h, Color c) {
        const std::optional<Rect> r = clip({x, y, w, h}, width(), height());
        if (!r) {
            return;
        }
        ili::window_logical(static_cast<uint16_t>(r->x), static_cast<uint16_t>(r->y), r->w, r->h);
        for (uint16_t i = 0; i < r->w; ++i) {
            ili::put_pixel(ili::row_buf, i, c);
        }
        for (uint16_t row = 0; row < r->h; ++row) {
            (void)ili::transfer(row == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr,
                                static_cast<uint16_t>(r->w * 3u), ili::write_rate);
        }
    }

    void write_run(Coord x, Coord y, std::span<const Color> run) {
        const std::optional<Rect> r = clip({x, y, static_cast<Extent>(run.size()), 1}, width(), height());
        if (!r) {
            return;
        }
        const uint16_t skip = static_cast<uint16_t>(r->x - x);
        ili::window_logical(static_cast<uint16_t>(r->x), static_cast<uint16_t>(r->y), r->w, 1);
        for (uint16_t i = 0; i < r->w; ++i) {
            ili::put_pixel(ili::row_buf, i, run[skip + i]);
        }
        (void)ili::transfer(ili::ramwr, ili::row_buf, nullptr, static_cast<uint16_t>(r->w * 3u), ili::write_rate);
    }

    /// THE ORACLE'S VERB, not a Surface one: a logical run read back in
    /// logical order, through the same window and the same walk. `out`
    /// takes w * 3 bytes in the driver's byte order.
    static bool read_run(uint16_t x, uint16_t y, uint16_t w, uint8_t* out) {
        const uint16_t bytes = static_cast<uint16_t>(w * 3u);
        ili::window_logical(x, y, w, 1);
        (void)ili::transfer(ili::ramrd, nullptr, ili::raw_buf, static_cast<uint16_t>(bytes + ili::read_extra),
                            ili::read_rate);
        return ili::realign(ili::raw_buf, static_cast<uint16_t>(bytes + ili::read_extra), out, bytes,
                            ili::read_format);
    }
};

// =============================================================================
// the letters
// =============================================================================

/// Where the pair (hi, lo) sits in `raw`, as a byte offset and a bit
/// shift of the whole stream; nullopt when nowhere.
std::optional<ili::ReadFormat> find_pair(const uint8_t* raw, uint16_t n, uint8_t hi, uint8_t lo) {
    for (uint8_t shift = 0; shift < 8u; ++shift) {
        for (uint8_t off = 0; off + 2u + (shift != 0u ? 1u : 0u) <= n; ++off) {
            uint8_t a[2];
            if (!ili::realign(raw, n, a, 2, {off, shift})) {
                break;
            }
            if (a[0] == hi && a[1] == lo) {
                return ili::ReadFormat{off, shift};
            }
        }
    }
    return std::nullopt;
}

void ta_identity() {
    struct Reg {
        uint8_t cmd;
        uint8_t len;
        const char* name;
    };
    static constexpr Reg regs[] = {
        {ili::rddid, 6, "RDDID    "},     {ili::rddst, 7, "RDDST    "},   {ili::rddpm, 4, "RDDPM    "},
        {ili::rddmadctl, 4, "RDDMADCTL"}, {ili::rddcolmod, 4, "RDDCOLMOD"}, {ili::devcode, 8, "DEVCODE  "},
    };
    uint8_t raw[8];
    uint8_t pm_raw[4] = {0, 0, 0, 0};
    uint8_t code_raw[8] = {0};
    for (const Reg& r : regs) {
        ili::read_raw(r.cmd, raw, r.len);
        print(serial, "  ");
        hex2(r.cmd);
        print(serial, ' ', r.name, " -> ");
        dump(raw, r.len);
        print(serial, crlf);
        if (r.cmd == ili::rddpm) {
            for (uint8_t i = 0; i < 4u; ++i) {
                pm_raw[i] = raw[i];
            }
        }
        if (r.cmd == ili::devcode) {
            for (uint8_t i = 0; i < 8u; ++i) {
                code_raw[i] = raw[i];
            }
        }
    }
    const std::optional<ili::ReadFormat> where = find_pair(code_raw, 8, 0x94, 0x81);
    if (where) {
        print(serial, "  94 81 at byte ", where->offset, " with a shift of ", where->shift, " bits: ",
              where->offset == 3u ? "one dummy BYTE before 02 04" :
              where->offset == 2u ? "NO dummy byte before 02 04" : "an offset the datasheet does not describe",
              crlf);
    }
    bench.verdict("the device code carries 94 81: an ILI9481 answers on SDO", where.has_value());
    const bool dummy_byte = where && where->offset == 3u && where->shift == 0u;
    const bool dummy_clock = where && where->offset == 2u && where->shift == 1u;
    bench.verdict("the device code's dummy read is one byte or one clock", dummy_byte || dummy_clock);
    // RDDPM: a single-parameter read - the datasheet gives it a dummy
    // too, and whether the silicon does is what the raw bytes say. Both
    // readings are printed; the verdict takes the first byte unless it
    // is the idle line.
    const uint8_t pm = pm_raw[0] != 0xFFu ? pm_raw[0] : pm_raw[1];
    print(serial, "  RDDPM as first byte 0x");
    hex2(pm_raw[0]);
    print(serial, ", as second byte 0x");
    hex2(pm_raw[1]);
    print(serial, ", shifted one clock 0x");
    hex2(static_cast<uint8_t>((pm_raw[0] << 1) | (pm_raw[1] >> 7)));
    print(serial, crlf);
    print(serial, "  power mode 0x");
    hex2(pm);
    print(serial, ": D7 ", (pm & 0x80u) ? "1" : "0 (the table's 'set to 0')", ", idle ", (pm & 0x40u) ? "ON" : "off",
          ", partial ", (pm & 0x20u) ? "ON" : "off", ", sleep ", (pm & 0x10u) ? "out" : "IN",
          ", normal ", (pm & 0x08u) ? "on" : "OFF", ", display ", (pm & 0x04u) ? "on" : "OFF", crlf);
    // 0xFF is the pull-up on a line nobody drove, and the datasheet keeps
    // D1 and D0 of this register at zero: an all-ones stream is not an
    // answer, it is an absent or unpowered module.
    bool all_ones = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        all_ones = all_ones && code_raw[i] == 0xFFu;
    }
    if (all_ones) {
        print(serial, "  every byte reads 0xFF: SDO is idle under the MISO pull-up - the module is not wired, "
                      "not powered, or its SDO is not on PA6", crlf);
    }
    bench.verdict("power mode: sleep out, normal mode, display on, D1 and D0 zero",
                  (pm & 0x1Fu) == 0x1Cu && !all_ones);
}

void tb_read_format() {
    constexpr uint16_t x = 16, y = 16, w = 8, h = 4;
    constexpr uint16_t n = w * h * 3u;
    ili::write_pattern(x, y, w, h, 0, ili::write_rate);
    // One RAMRD over the whole block: the window spans the rows and the
    // address counter wraps at EC by itself.
    ili::window(x, y, x + w - 1u, y + h - 1u);
    const uint16_t n_raw = n + ili::read_extra;
    ili::read_raw(ili::ramrd, ili::raw_buf, n_raw);
    print(serial, "  raw: ");
    dump(ili::raw_buf, 20);
    print(serial, "...", crlf);
    // The expected bytes.
    for (uint16_t r = 0; r < h; ++r) {
        for (uint16_t c = 0; c < w; ++c) {
            ili::put_pixel(ili::scratch + r * w * 3u, c, ili::pattern(x + c, y + r));
        }
    }
    print(serial, "  expected: ");
    dump(ili::scratch, 12);
    print(serial, "...", crlf);
    std::optional<ili::ReadFormat> found;
    uint8_t low_bits = 0;
    for (uint8_t shift = 0; shift < 8u && !found; ++shift) {
        for (uint8_t off = 0; off < 4u; ++off) {
            uint8_t out[n];
            if (!ili::realign(ili::raw_buf, n_raw, out, n, {off, shift})) {
                continue;
            }
            uint16_t first = 0;
            if (ili::mismatches(ili::scratch, out, n, first) == 0u) {
                found = ili::ReadFormat{off, shift};
                for (uint16_t i = 0; i < n; ++i) {
                    low_bits = static_cast<uint8_t>(low_bits | (out[i] & 0x03u));
                }
                break;
            }
        }
    }
    if (found) {
        print(serial, "  the stream matches at byte offset ", found->offset, ", bit shift ", found->shift,
              " - ", found->offset == 1u && found->shift == 0u ? "ONE DUMMY BYTE, then the pixels" :
              found->offset == 0u && found->shift == 0u ? "no dummy at all" : "a dummy CLOCK, not a byte",
              crlf, "  the two LSBs of every returned byte OR to 0x");
        hex2(low_bits);
        print(serial, crlf);
        ili::read_format = *found;
        ili::read_format_known = true;
    } else {
        // Print how close the default alignment gets.
        uint8_t out[n];
        (void)ili::realign(ili::raw_buf, n_raw, out, n, ili::read_format);
        uint16_t first = 0;
        const uint32_t bad = ili::mismatches(ili::scratch, out, n, first);
        print(serial, "  no alignment matches; with one dummy byte ", bad, " of ", n, " bytes differ, first at ",
              first, crlf);
    }
    bench.verdict("a block written is read back byte for byte (six MSBs)", found.has_value());
    bench.verdict("the read format is a dummy byte or a dummy clock, as the datasheet's 'dummy read' allows",
                  found && ((found->offset == 1u && found->shift == 0u) || (found->offset == 0u && found->shift == 1u)));
}

void tc_read_continue() {
    ili::note_format();
    constexpr uint16_t x = 32, y = 16, w = 24;
    constexpr uint16_t eight = 8u * 3u;
    ili::write_pattern(x, y, w, 1, 0, ili::write_rate);
    for (uint16_t c = 0; c < w; ++c) {
        ili::put_pixel(ili::scratch, c, ili::pattern(x + c, y));
    }
    // Exactly what one read of eight pixels needs in the format found:
    // a spare byte clocked would move the read pointer past a pixel.
    const uint16_t exact = static_cast<uint16_t>(ili::read_format.offset + eight +
                                                 (ili::read_format.shift != 0u ? 1u : 0u));
    uint8_t got[eight];
    uint16_t bad_at = 0;

    ili::window(x, y, x + w - 1u, y);
    ili::read_raw(ili::ramrd, ili::raw_buf, exact);
    (void)ili::realign(ili::raw_buf, exact, got, eight, ili::read_format);
    bench.verdict("RAMRD of the first eight pixels, clocked exactly", ili::mismatches(ili::scratch, got, eight, bad_at) == 0u);

    ili::read_raw(ili::ramrd_continue, ili::raw_buf, exact);
    print(serial, "  3Eh raw: ");
    dump(ili::raw_buf, 8);
    print(serial, "...", crlf);
    (void)ili::realign(ili::raw_buf, exact, got, eight, ili::read_format);
    const uint32_t bad = ili::mismatches(ili::scratch + eight, got, eight, bad_at);
    bench.verdict("read_memory_continue (3Eh), in the same format, picks up at the ninth pixel", bad == 0u);

    // The pointer counts CLOCKS: a RAMRD with four spare bytes clocked
    // (a whole pixel and one byte more), then 3Eh - where does it start?
    ili::window(x, y, x + w - 1u, y);
    ili::read_raw(ili::ramrd, ili::raw_buf, static_cast<uint16_t>(exact + 4u));
    ili::read_raw(ili::ramrd_continue, ili::raw_buf, exact);
    (void)ili::realign(ili::raw_buf, exact, got, eight, ili::read_format);
    std::optional<uint16_t> at;
    for (uint16_t j = 0; static_cast<uint32_t>(j) + eight <= static_cast<uint32_t>(w) * 3u && !at; ++j) {
        if (ili::mismatches(ili::scratch + j, got, eight, bad_at) == 0u) {
            at = j;
        }
    }
    if (at) {
        print(serial, "  after four spare bytes clocked (a pixel and a byte), 3Eh starts at pixel ", *at / 3u,
              " byte ", *at % 3u, ": ", *at == 9u * 3u ? "the pixel cut short is read again from its first byte" :
              *at == 10u * 3u ? "the pixel cut short is skipped whole" :
              *at == 9u * 3u + 1u ? "it resumes inside the pixel cut short" : "none of the three readings",
              crlf);
    } else {
        print(serial, "  after four spare bytes clocked, 3Eh's eight pixels are nowhere in the row", crlf);
    }
    bench.verdict("3Eh after a read clocked past its pixels resumes at a position the row explains", at.has_value());
}

void td_full_frame() {
    ili::note_format();
    // Write.
    uint32_t t0 = Ticker::millis();
    ili::window(0, 0, ili::width - 1u, ili::height - 1u);
    for (uint16_t y = 0; y < ili::height; ++y) {
        for (uint16_t x = 0; x < ili::width; ++x) {
            ili::put_pixel(ili::row_buf, x, ili::pattern(x, y));
        }
        (void)ili::transfer(y == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, ili::row_bytes,
                            ili::write_rate);
    }
    const uint32_t write_ms = Ticker::millis() - t0;
    constexpr uint32_t frame_bytes = static_cast<uint32_t>(ili::row_bytes) * ili::height;
    print(serial, "  written: ", frame_bytes, " bytes in ", write_ms, " ms (", frame_bytes / (write_ms ? write_ms : 1u),
          " kB/s at SCK ", Host::sck_hz(ili::write_rate) / 1000u, " kHz)", crlf);

    // Read back, one RAMRD per row (its own window each - no reliance on 3Eh).
    t0 = Ticker::millis();
    uint32_t bad = 0;
    uint32_t bad_rows = 0;
    uint16_t first_row = ili::height;
    uint16_t first_at = 0;
    uint8_t got_first[3] = {0, 0, 0};
    uint8_t want_first[3] = {0, 0, 0};
    for (uint16_t y = 0; y < ili::height; ++y) {
        if (!ili::read_block(0, y, ili::width, 1, ili::scratch, ili::read_rate)) {
            bad += ili::row_bytes;
            continue;
        }
        for (uint16_t x = 0; x < ili::width; ++x) {
            ili::put_pixel(ili::row_buf, x, ili::pattern(x, y));
        }
        uint16_t first = 0;
        const uint32_t row_bad = ili::mismatches(ili::row_buf, ili::scratch, ili::row_bytes, first);
        if (row_bad != 0u) {
            if (bad == 0u) {
                first_row = y;
                first_at = first;
                for (uint8_t i = 0; i < 3u; ++i) {
                    got_first[i] = ili::scratch[first + i];
                    want_first[i] = ili::row_buf[first + i];
                }
            }
            bad += row_bad;
            ++bad_rows;
        }
    }
    const uint32_t read_ms = Ticker::millis() - t0;
    print(serial, "  read back: ", frame_bytes, " bytes in ", read_ms, " ms (", frame_bytes / (read_ms ? read_ms : 1u),
          " kB/s at SCK ", Host::sck_hz(ili::read_rate) / 1000u, " kHz)", crlf);
    print(serial, "  mismatched bytes: ", bad, " in ", bad_rows, " rows");
    if (bad != 0u) {
        print(serial, "; first at row ", first_row, " byte ", first_at, ": got ");
        dump(got_first, 3);
        print(serial, "wanted ");
        dump(want_first, 3);
    }
    print(serial, crlf);
    bench.verdict("the whole frame reads back as written", bad == 0u);
}

/// A 64 x 8 block written at `wr` and read back at `rd`; `under_test`
/// is the one of the two the caller is climbing, for the line printed.
void rate_block(const char* what, uint16_t x, uint16_t y, uint32_t salt, SpiClock wr, SpiClock rd,
                SpiClock under_test, uint32_t& bad) {
    constexpr uint16_t w = 64, h = 8;
    constexpr uint32_t n = static_cast<uint32_t>(w) * h * 3u;
    static uint8_t got[n];
    ili::write_pattern(x, y, w, h, salt, wr);
    uint16_t first = 0;
    if (!ili::read_block(x, y, w, h, got, rd)) {
        bad = n;
    } else {
        bad = ili::check_pattern(x, y, w, h, salt, got, first);
    }
    print(serial, "  ", what, " ", Host::sck_hz(under_test) / 1000u, " kHz: ", bad, " of ", n, " bytes wrong");
    if (bad != 0u) {
        print(serial, ", first at byte ", first, ": got ");
        dump(got + first, 3);
    }
    print(serial, crlf);
}

void te_write_rates() {
    ili::note_format();
    struct Step {
        SpiClock rate;
        bool in_spec;
    };
    static constexpr Step steps[] = {
        {SpiClock::div8, true},    // 12 MHz: an 83 ns cycle against the 80 the datasheet asks
        {SpiClock::div4, false},   // 24 MHz
        {SpiClock::div2, false},   // 48 MHz
    };
    uint32_t salt = 0x1000;
    for (const Step& s : steps) {
        uint32_t bad = 0;
        rate_block("write at", 64, 64, salt, s.rate, ili::read_rate, s.rate, bad);
        if (s.in_spec) {
            bench.verdict("a write at the datasheet's ceiling reads back exact", bad == 0u);
        }
        salt += 0x1000;
    }
}

void tf_read_rates() {
    ili::note_format();
    static constexpr SpiClock steps[] = {SpiClock::div16, SpiClock::div8, SpiClock::div4};   // 6, 12, 24 MHz
    uint32_t salt = 0x5000;
    for (SpiClock rd : steps) {
        uint32_t bad = 0;
        rate_block("read at", 64, 80, salt, ili::write_rate, rd, rd, bad);
        salt += 0x1000;
    }
    bench.verdict("the read rate ladder is printed, not judged: the datasheet's 4.17 MHz stands", true);
}

// The four physical corners, each an 8 x 8 square painted with a pattern
// of its PHYSICAL coordinates under MADCTL 00 - the map every read and
// write under another code is located on.
constexpr uint16_t corner = 8;
constexpr uint32_t corner_salt = 0x7000;
struct Xy {
    uint16_t x;
    uint16_t y;
};
constexpr Xy corners[4] = {{0, 0}, {ili::width - corner, 0}, {0, ili::height - corner},
                           {ili::width - corner, ili::height - corner}};

/// The map is painted and read under the panel's base code (its own
/// display flip, no walk bits): "physical" here means that.
void set_madctl(uint8_t code) { ili::set_madctl(static_cast<uint8_t>(ili::madctl_panel | code)); }

void paint_corners() {
    for (const Xy& c : corners) {
        ili::write_pattern(c.x, c.y, corner, corner, corner_salt, ili::write_rate);
    }
}

/// Where a pixel VALUE lies on the corner map: its physical coordinates,
/// and whether it matched only with R and B exchanged.
struct Located {
    uint16_t x;
    uint16_t y;
    bool swapped;
};
std::optional<Located> locate_on_map(const uint8_t* px) {
    for (const Xy& c : corners) {
        for (uint16_t r = 0; r < corner; ++r) {
            for (uint16_t k = 0; k < corner; ++k) {
                const uint32_t v = ili::pattern(c.x + k, c.y + r, corner_salt);
                const uint8_t b0 = static_cast<uint8_t>(v >> 16), b1 = static_cast<uint8_t>(v >> 8),
                              b2 = static_cast<uint8_t>(v);
                if ((px[0] & 0xFCu) == b0 && (px[1] & 0xFCu) == b1 && (px[2] & 0xFCu) == b2) {
                    return Located{static_cast<uint16_t>(c.x + k), static_cast<uint16_t>(c.y + r), false};
                }
                if ((px[0] & 0xFCu) == b2 && (px[1] & 0xFCu) == b1 && (px[2] & 0xFCu) == b0 && b0 != b2) {
                    return Located{static_cast<uint16_t>(c.x + k), static_cast<uint16_t>(c.y + r), true};
                }
            }
        }
    }
    return std::nullopt;
}

/// Where a written pixel's VALUE landed: the corners read back under
/// MADCTL 00 (`map` = four 8 x 8 blocks in corner order).
std::optional<Located> locate_in_readback(const uint8_t* map, uint32_t want) {
    const uint8_t b0 = static_cast<uint8_t>(want >> 16), b1 = static_cast<uint8_t>(want >> 8),
                  b2 = static_cast<uint8_t>(want);
    for (uint8_t ci = 0; ci < 4u; ++ci) {
        for (uint16_t r = 0; r < corner; ++r) {
            for (uint16_t k = 0; k < corner; ++k) {
                const uint8_t* px = map + (static_cast<uint32_t>(ci) * corner * corner + r * corner + k) * 3u;
                if ((px[0] & 0xFCu) == b0 && (px[1] & 0xFCu) == b1 && (px[2] & 0xFCu) == b2) {
                    return Located{static_cast<uint16_t>(corners[ci].x + k), static_cast<uint16_t>(corners[ci].y + r), false};
                }
                if ((px[0] & 0xFCu) == b2 && (px[1] & 0xFCu) == b1 && (px[2] & 0xFCu) == b0 && b0 != b2) {
                    return Located{static_cast<uint16_t>(corners[ci].x + k), static_cast<uint16_t>(corners[ci].y + r), true};
                }
            }
        }
    }
    return std::nullopt;
}

void print_located(const std::optional<Located>& l) {
    if (!l) {
        print(serial, "(nowhere)");
        return;
    }
    print(serial, "(", l->x, ",", l->y, l->swapped ? " R/B swapped)" : ")");
}

void tg_madctl() {
    ili::note_format();
    static constexpr uint8_t codes[] = {0x00, 0x40, 0x80, 0xC0, 0x20, 0x60, 0xA0, 0xE0, 0x08};
    static constexpr const char* names[] = {"00 normal", "40 cols reversed", "80 pages reversed",
                                            "C0 both reversed", "20 exchanged", "60 exch+cols",
                                            "A0 exch+pages", "E0 exch+both", "08 BGR"};
    // The three logical pixels whose physical homes tell the mapping:
    // the origin, one step along the columns, one along the pages.
    static constexpr Xy probes[3] = {{0, 0}, {1, 0}, {0, 1}};
    static uint8_t map[4u * corner * corner * 3u];
    static uint8_t block[4u * 2u * 3u];
    print(serial, "  logical (0,0) (1,0) (0,1) -> physical, the read's then the write's", crlf);
    uint8_t i = 0;
    bool agree_00 = false;
    for (uint8_t code : codes) {
        // THE READ: the corners painted with their physical pattern under
        // 00, then a 4 x 2 window at the origin read under the code.
        set_madctl(0x00);
        paint_corners();
        set_madctl(code);
        ili::window(0, 0, 3, 1);
        const uint16_t n = static_cast<uint16_t>(sizeof(block));
        ili::read_raw(ili::ramrd, ili::raw_buf, static_cast<uint16_t>(n + ili::read_extra));
        (void)ili::realign(ili::raw_buf, static_cast<uint16_t>(n + ili::read_extra), block, n, ili::read_format);
        std::optional<Located> rd[3];
        bool all_read = true;
        for (uint8_t k = 0; k < 3u; ++k) {
            rd[k] = locate_on_map(block + (probes[k].y * 4u + probes[k].x) * 3u);
            all_read = all_read && rd[k].has_value();
        }
        // THE WRITE: a 4 x 2 block at the origin under the code, then the
        // corners read back under 00 and each written value looked for.
        ili::write_pattern(0, 0, 4, 2, 0x8000u + code, ili::write_rate);
        set_madctl(0x00);
        for (uint8_t ci = 0; ci < 4u; ++ci) {
            (void)ili::read_block(corners[ci].x, corners[ci].y, corner, corner,
                                  map + static_cast<uint32_t>(ci) * corner * corner * 3u, ili::read_rate);
        }
        std::optional<Located> wr[3];
        bool all_write = true;
        bool agree = true;
        for (uint8_t k = 0; k < 3u; ++k) {
            wr[k] = locate_in_readback(map, ili::pattern(probes[k].x, probes[k].y, 0x8000u + code));
            all_write = all_write && wr[k].has_value();
            agree = agree && rd[k] && wr[k] && rd[k]->x == wr[k]->x && rd[k]->y == wr[k]->y &&
                    rd[k]->swapped == wr[k]->swapped;
        }
        print(serial, "  MADCTL ", names[i], ": read ");
        for (uint8_t k = 0; k < 3u; ++k) {
            print_located(rd[k]);
            print(serial, ' ');
        }
        print(serial, " write ");
        for (uint8_t k = 0; k < 3u; ++k) {
            print_located(wr[k]);
            print(serial, ' ');
        }
        print(serial, agree ? " AGREE" : " DIFFER", crlf);
        bench.verdict("read under ", names[i], all_read);
        bench.verdict("write under ", names[i], all_write);
        if (code == 0x00u) {
            agree_00 = agree;
        }
        ++i;
    }
    ili::set_rotation(ili::rotation);
    bench.verdict("under 00 the write's mapping and the read's are the same", agree_00);
}

/// "brio" drawn at (8, 8) of the logical surface and read back through
/// it: mismatched bytes against the font's own rows.
uint32_t word_mismatches(PanelSurface& panel, bool& read_ok) {
    using F = Font5x7;
    constexpr uint32_t white = 0xFCFCFC, black = 0x000000;
    constexpr uint16_t cells = 4;
    constexpr uint16_t run_px = cells * F::cell_w;
    static uint8_t got[run_px * 3u];
    static uint8_t want[run_px * 3u];
    const char* word = "brio";
    text<F>(panel, 8, 8, word, white, black);
    uint32_t bad = 0;
    read_ok = true;
    for (Extent row = 0; row < F::cell_h; ++row) {
        for (uint16_t k = 0; k < cells; ++k) {
            const uint8_t bits = F::row_bits(static_cast<uint8_t>(word[k]), row);
            for (Extent col = 0; col < F::cell_w; ++col) {
                const bool on = ((bits >> (F::cell_w - 1u - col)) & 1u) != 0u;
                ili::put_pixel(want, static_cast<uint16_t>(k * F::cell_w + col), on ? white : black);
            }
        }
        read_ok = PanelSurface::read_run(8, static_cast<uint16_t>(8u + row), run_px, got) && read_ok;
        uint16_t first = 0;
        bad += ili::mismatches(want, got, sizeof(got), first);
    }
    return bad;
}

void th_gfx() {
    ili::note_format();
    PanelSurface panel;
    using F = Font5x7;
    constexpr uint32_t black = 0x000000, white = 0xFCFCFC;
    static constexpr uint32_t bars[8] = {0xFCFCFC, 0xFCFC00, 0x00FCFC, 0x00FC00, 0xFC00FC, 0xFC0000, 0x0000FC, 0x000000};

    // The word in every rotation first: the same call, the same read.
    static constexpr ili::Rotation turns[4] = {ili::Rotation::r0, ili::Rotation::r90, ili::Rotation::r180,
                                               ili::Rotation::r270};
    static constexpr const char* turn_names[4] = {"r0", "r90", "r180", "r270"};
    for (uint8_t t = 0; t < 4u; ++t) {
        ili::set_rotation(turns[t]);
        bool ok = false;
        const uint32_t bad = word_mismatches(panel, ok);
        print(serial, "  ", turn_names[t], " (", panel.width(), "x", panel.height(), ", MADCTL 0x");
        hex2(ili::madctl_for(turns[t]));
        print(serial, "): \"brio\" read back with ", bad, " bytes differing", crlf);
        bench.verdict("text reads back as the font says under ", turn_names[t], ok && bad == 0u);
    }

    // The picture, in landscape.
    ili::set_rotation(ili::Rotation::r90);
    const Extent W = panel.width(), H = panel.height();
    uint32_t t0 = Ticker::millis();
    clear(panel, black);
    const uint32_t clear_ms = Ticker::millis() - t0;
    for (uint8_t i = 0; i < 8u; ++i) {
        fill_rect(panel, static_cast<Coord>(i * (W / 8u)), 0, static_cast<Extent>(W / 8u), 64, bars[i]);
    }
    rect(panel, 0, 0, W, H, white);
    rect(panel, 2, 2, static_cast<Extent>(W - 4u), static_cast<Extent>(H - 4u), 0x00A0FC);
    fill_circle(panel, 400, 150, 48, 0xFC8000);
    circle(panel, 400, 150, 60, white);
    fill_round_rect(panel, 20, 100, 140, 90, 12, 0x2040FC);
    round_rect(panel, 20, 100, 140, 90, 12, white);
    line(panel, 20, 220, static_cast<Coord>(W - 20), static_cast<Coord>(H - 20), 0x00FC00);
    line(panel, static_cast<Coord>(W - 20), 220, 20, static_cast<Coord>(H - 20), 0xFC0000);
    Coord y = 200;
    text<F>(panel, 180, y, "brio ILI9481 probe, landscape (r90)", white, black);
    y += F::cell_h + 2;
    text<F>(panel, 180, y, "STM32F411CE black pill, SPI1 at PA5/6/7", white, black);
    y += F::cell_h + 2;
    text<F>(panel, 180, y, "the GRAM read back is the oracle", 0xFCFC00, black);
    y += F::cell_h + 2;
    text<F>(panel, 180, y, "!\"#$%&'()*+,-./0123456789:;<=>?@", 0x00FCFC, black);
    y += F::cell_h + 2;
    text<F>(panel, 180, y, "ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`", 0x00FCFC, black);
    y += F::cell_h + 2;
    text<F>(panel, 180, y, "abcdefghijklmnopqrstuvwxyz{|}~", 0x00FCFC, black);
    const uint32_t draw_ms = Ticker::millis() - t0;
    print(serial, "  landscape: clear ", clear_ms, " ms, the whole picture ", draw_ms, " ms", crlf);

    // A bar's colour, read through the surface: the yellow one, in the
    // driver's own byte order.
    uint8_t px[3] = {0, 0, 0};
    uint8_t want[3];
    ili::put_pixel(want, 0, bars[1]);
    const bool read_ok = PanelSurface::read_run(static_cast<uint16_t>(W / 8u + 4u), 40, 1, px);
    print(serial, "  the yellow bar's pixel: ");
    dump(px, 3);
    print(serial, "(wanted ");
    dump(want, 3);
    print(serial, ")", crlf);
    uint16_t first = 0;
    bench.verdict("a filled rectangle's pixel reads back its colour", read_ok && ili::mismatches(want, px, 3, first) == 0u);
}

/// The marker picture for the eye, in the rotation in force: a red
/// square at the logical origin, an arrow and TOP at the top centre, BL
/// and BR at the bottom corners, the rotation's name. Readable text
/// says "not mirrored"; where TOP points says the rotation.
void draw_marker(const char* name) {
    PanelSurface panel;
    using F = Font5x7;
    const Extent W = panel.width(), H = panel.height();
    clear(panel, 0x000000);
    rect(panel, 0, 0, W, H, 0xFCFCFC);
    fill_rect(panel, 0, 0, 40, 40, 0xFC0000);
    text<F>(panel, 48, 8, name, 0xFCFCFC, 0x000000);
    text<F>(panel, 48, 20, "red square = logical (0,0)", 0xFCFCFC, 0x000000);
    const Coord cx = static_cast<Coord>(W / 2u);
    line(panel, cx, 50, cx, 110, 0x00FC00);
    line(panel, cx, 50, static_cast<Coord>(cx - 15), 70, 0x00FC00);
    line(panel, cx, 50, static_cast<Coord>(cx + 15), 70, 0x00FC00);
    text<F>(panel, static_cast<Coord>(cx - 9), 114, "TOP", 0x00FC00, 0x000000);
    text<F>(panel, 8, static_cast<Coord>(H - 16), "BL", 0xFCFC00, 0x000000);
    text<F>(panel, static_cast<Coord>(W - 20), static_cast<Coord>(H - 16), "BR", 0xFCFC00, 0x000000);
    text<F>(panel, static_cast<Coord>(cx - 60), static_cast<Coord>(H / 2u), "brio ILI9481 orientation", 0x00FCFC, 0x000000);
}

/// (by name) the marker in the rotation in force, then the next one
/// for the next call: r0, r90, r180, r270, r0 ...
void to_turn() {
    static constexpr const char* names[4] = {"r0 (portrait)", "r90 (clockwise)", "r180", "r270 (counter-clockwise)"};
    const uint8_t cur = static_cast<uint8_t>(ili::rotation);
    draw_marker(names[cur]);
    print(serial, "  drawn: ", names[cur], " - MADCTL 0x");
    hex2(ili::madctl_for(ili::rotation));
    print(serial, "; the next call draws the next rotation", crlf);
    ili::set_rotation(static_cast<ili::Rotation>((cur + 1u) & 3u));
}

/// (by name) THE DISPLAY-SIDE BITS OF MADCTL, one step a call, the
/// memory untouched: none, B1 (horizontal flip), B0 (vertical flip),
/// B4 (line address order), B1+B0, none again. The eye says what each
/// does to the picture on the glass.
void tn_display_bits() {
    static constexpr uint8_t bits[5] = {0x00, 0x02, 0x01, 0x10, 0x03};
    static constexpr const char* names[5] = {"none", "B1 horizontal flip", "B0 vertical flip", "B4 line address order",
                                             "B1 + B0"};
    static uint8_t step = 0;
    step = static_cast<uint8_t>((step + 1u) % 5u);
    // The rotation's walk bits stay; the display bits change alone. The
    // rotation in force is the one the last marker was drawn in.
    const ili::Rotation shown = static_cast<ili::Rotation>((static_cast<uint8_t>(ili::rotation) + 3u) & 3u);
    const uint8_t code = static_cast<uint8_t>(ili::madctl_for(shown) | bits[step]);
    ili::set_madctl(code);
    print(serial, "  MADCTL display bits: ", names[step], " (0x");
    hex2(code);
    print(serial, ") - look at the glass; the next call steps on", crlf);
}

/// The frame written with pattern(x, y, salt) at `rate` on the host in
/// force, then read back at `read_at` and compared; both phases timed.
void frame_round(uint32_t salt, SpiClock rate, SpiClock read_at, uint32_t& bad, uint32_t& write_ms,
                 uint32_t& read_ms) {
    uint32_t t0 = Ticker::millis();
    ili::window(0, 0, ili::width - 1u, ili::height - 1u);
    for (uint16_t y = 0; y < ili::height; ++y) {
        for (uint16_t x = 0; x < ili::width; ++x) {
            ili::put_pixel(ili::row_buf, x, ili::pattern(x, y, salt));
        }
        (void)ili::transfer(y == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, ili::row_bytes, rate);
    }
    write_ms = Ticker::millis() - t0;
    t0 = Ticker::millis();
    bad = 0;
    for (uint16_t y = 0; y < ili::height; ++y) {
        if (!ili::read_block(0, y, ili::width, 1, ili::scratch, read_at)) {
            bad += ili::row_bytes;
            continue;
        }
        for (uint16_t x = 0; x < ili::width; ++x) {
            ili::put_pixel(ili::row_buf, x, ili::pattern(x, y, salt));
        }
        uint16_t first = 0;
        bad += ili::mismatches(ili::row_buf, ili::scratch, ili::row_bytes, first);
    }
    read_ms = Ticker::millis() - t0;
}

void tk_engines() {
    ili::note_format();
    Host::release();
    if (!DmaHost::init(clock)) {
        bench.verdict("the engined host came up", false);
        (void)Host::init(clock);
        return;
    }
    dma_host_live = true;
    constexpr uint32_t frame_bytes = static_cast<uint32_t>(ili::row_bytes) * ili::height;
    struct Step {
        SpiClock write;
        SpiClock read;
        const char* what;
    };
    static constexpr Step steps[] = {
        {SpiClock::div16, SpiClock::div32, "write 6 MHz, read 3 MHz"},
        {SpiClock::div8, SpiClock::div8, "write 12 MHz, read 12 MHz"},
    };
    uint32_t salt = 0x3000;
    for (const Step& st : steps) {
        uint32_t bad = 0, wms = 0, rms = 0;
        frame_round(salt, st.write, st.read, bad, wms, rms);
        print(serial, "  engines, ", st.what, ": written in ", wms, " ms (", frame_bytes / (wms ? wms : 1u),
              " kB/s), read back in ", rms, " ms (", frame_bytes / (rms ? rms : 1u), " kB/s), ", bad,
              " bytes wrong, engine status ", DmaHost::status(), crlf);
        bench.verdict("the frame through the engines reads back exact: ", st.what, bad == 0u);
        salt += 0x1000;
    }
    dma_host_live = false;
    DmaHost::release();
    bench.verdict("the pump host is back", Host::init(clock));
}

/// (by name) WHERE A WINDOW LANDS UNDER B5: the frame cleared, four
/// blocks of one colour each written under different MADCTL codes and
/// window shapes, then every non-black pixel located under the base
/// code. The answer is the physical (column, page) of each colour.
void tx_b5_range() {
    ili::set_madctl(ili::madctl_panel);
    PanelSurface panel;
    ili::rotation = ili::Rotation::r0;
    clear(panel, 0x000000);
    struct Probe {
        uint8_t code;
        uint16_t x0, y0, x1, y1;   // CASET x0..x1, PASET y0..y1, as written
        uint32_t colour;
        const char* name;
    };
    static constexpr Probe probes[] = {
        {0x00, 200, 300, 203, 300, 0xFCFCFC, "white: base, CASET 200..203 PASET 300"},
        {0x20, 100, 150, 103, 150, 0xFC0000, "red:   B5, CASET 100..103 PASET 150"},
        {0x20, 340, 150, 343, 150, 0x00FC00, "green: B5, CASET 340..343 PASET 150"},
        {0x20, 150, 400, 150, 403, 0x0000FC, "blue:  B5, CASET 150 PASET 400..403"},
        {0x20, 150, 250, 150, 253, 0xFCFC00, "yellow: B5, CASET 150 PASET 250..253"},
        {0x60, 340, 160, 343, 160, 0xFC00FC, "magenta: B5+B6, CASET 340..343 PASET 160"},
        {0x60, 100, 160, 103, 160, 0xFC8000, "orange: B5+B6, CASET 100..103 PASET 160"},
        {0x40, 100, 340, 103, 340, 0x00FCFC, "cyan: B6, CASET 100..103 PASET 340"},
        {0xA0, 300, 170, 303, 170, 0x8080FC, "lilac: B5+B7, CASET 300..303 PASET 170"},
    };
    for (const Probe& pr : probes) {
        ili::set_madctl(static_cast<uint8_t>(ili::madctl_panel | pr.code));
        ili::window(pr.x0, pr.y0, pr.x1, pr.y1);
        for (uint16_t i = 0; i < 4u; ++i) {
            ili::put_pixel(ili::row_buf, i, pr.colour);
        }
        (void)ili::transfer(ili::ramwr, ili::row_buf, nullptr, 12, ili::write_rate);
        print(serial, "  ", pr.name, crlf);
    }
    // Two full-height runs, one column each: under B5 alone and under
    // B5 with B6 - does the walk reach page 479?
    struct Column {
        uint8_t code;
        uint16_t column;
        uint32_t colour;
        const char* name;
    };
    static constexpr Column columns[] = {
        {0x20, 40, 0xFC4040, "a full column under B5 at column 40"},
        {0x60, 60, 0x40FC40, "a full column under B5+B6 at column 60"},
    };
    for (const Column& col : columns) {
        ili::set_madctl(static_cast<uint8_t>(ili::madctl_panel | col.code));
        ili::window(0, col.column, ili::height - 1u, col.column);
        for (uint16_t i = 0; i < 320u; ++i) {
            ili::put_pixel(ili::row_buf, i, col.colour);
        }
        (void)ili::transfer(ili::ramwr, ili::row_buf, nullptr, 960, ili::write_rate);
        (void)ili::transfer(ili::ramwr_continue, ili::row_buf, nullptr, 480, ili::write_rate);
    }
    ili::set_madctl(ili::madctl_panel);
    uint16_t col_hits[2] = {0, 0};
    uint16_t col_last[2] = {0, 0};
    uint16_t hits = 0;
    for (uint16_t y = 0; y < ili::height; ++y) {
        if (!ili::read_block(0, y, ili::width, 1, ili::scratch, ili::read_rate)) {
            continue;
        }
        for (uint16_t x = 0; x < ili::width; ++x) {
            const uint8_t* px = ili::scratch + x * 3u;
            if ((px[0] | px[1] | px[2]) == 0u) {
                continue;
            }
            uint8_t want[3];
            const char* who = "?";
            bool is_column = false;
            for (uint8_t k = 0; k < 2u; ++k) {
                ili::put_pixel(want, 0, columns[k].colour);
                if (x == columns[k].column && (px[0] & 0xFCu) == want[0] && (px[1] & 0xFCu) == want[1] &&
                    (px[2] & 0xFCu) == want[2]) {
                    ++col_hits[k];
                    col_last[k] = y;
                    is_column = true;
                }
            }
            if (is_column) {
                continue;
            }
            for (const Probe& pr : probes) {
                ili::put_pixel(want, 0, pr.colour);
                if ((px[0] & 0xFCu) == want[0] && (px[1] & 0xFCu) == want[1] && (px[2] & 0xFCu) == want[2]) {
                    who = pr.name;
                }
            }
            if (hits < 40u) {
                print(serial, "  pixel at column ", x, " page ", y, " is ", who, crlf);
            }
            ++hits;
        }
    }
    print(serial, "  ", hits, " lit pixels besides the two columns (36 written)", crlf);
    for (uint8_t k = 0; k < 2u; ++k) {
        print(serial, "  ", columns[k].name, ": ", col_hits[k], " of 480 pixels lit, the last at page ",
              col_last[k], crlf);
    }
    ili::set_rotation(ili::rotation);
}

/// (by name) the USB controller's own counters: what the console rode on.
void tu_usb() {
    const UsbLineCoding c = Serial::line_coding();
    print(serial, "  state=", static_cast<uint8_t>(Device::state()), " address=", Device::address(),
          " resets=", Device::resets(), " setups=", Device::setups(), " stalls=", Device::stalls(),
          " suspends=", Device::suspends(), " line=", c.baud, "/", c.data_bits, "/", c.parity, "/", c.stop_bits,
          " dtr=", Serial::dtr(), " rts=", Serial::rts(), " frame=", Usb::frame(), crlf,
          "  rx_overruns=", Serial::rx_overruns(), " rx_bytes=", Serial::rx_bytes(), " tx_bytes=", Serial::tx_bytes(),
          " fifo_waits=", Usb::fifo_waits(), " timeouts=", Usb::timeouts(), " mismatches=", Usb::mode_mismatches(),
          crlf);
}

/// (by name) WHAT THE FRAME HOLDS, untouched: every non-black pixel
/// counted, by colour (up to eight distinct ones, six bits a channel),
/// each with the box it spans in physical columns and pages - the way
/// to see what another program left in the GRAM.
void ty_census() {
    ili::set_madctl(ili::madctl_panel);
    struct Bin {
        uint32_t rgb;
        uint32_t count;
        uint16_t c0, c1, p0, p1;
    };
    static Bin bins[8];
    uint8_t n_bins = 0;
    uint32_t others = 0;
    uint32_t lit = 0;
    for (uint16_t y = 0; y < ili::height; ++y) {
        if (!ili::read_block(0, y, ili::width, 1, ili::scratch, ili::read_rate)) {
            continue;
        }
        for (uint16_t x = 0; x < ili::width; ++x) {
            const uint8_t* px = ili::scratch + x * 3u;
            const uint32_t rgb = (static_cast<uint32_t>(px[0] & 0xFCu) << 16) |
                                 (static_cast<uint32_t>(px[1] & 0xFCu) << 8) | (px[2] & 0xFCu);
            if (rgb == 0u) {
                continue;
            }
            ++lit;
            uint8_t i = 0;
            for (; i < n_bins; ++i) {
                if (bins[i].rgb == rgb) {
                    break;
                }
            }
            if (i == n_bins) {
                if (n_bins == 8u) {
                    ++others;
                    continue;
                }
                bins[n_bins++] = Bin{rgb, 0, x, x, y, y};
            }
            Bin& b = bins[i];
            ++b.count;
            if (x < b.c0) b.c0 = x;
            if (x > b.c1) b.c1 = x;
            if (y < b.p0) b.p0 = y;
            if (y > b.p1) b.p1 = y;
        }
    }
    print(serial, "  ", lit, " lit pixels of ", static_cast<uint32_t>(ili::width) * ili::height, crlf);
    for (uint8_t i = 0; i < n_bins; ++i) {
        print(serial, "  colour ");
        hex2(static_cast<uint8_t>(bins[i].rgb >> 16));
        hex2(static_cast<uint8_t>(bins[i].rgb >> 8));
        hex2(static_cast<uint8_t>(bins[i].rgb));
        print(serial, " (driver byte order): ", bins[i].count, " pixels, columns ", bins[i].c0, "..", bins[i].c1,
              ", pages ", bins[i].p0, "..", bins[i].p1, crlf);
    }
    if (others != 0u) {
        print(serial, "  and ", others, " pixels of other colours", crlf);
    }
    ili::set_rotation(ili::rotation);
}

/// (by name) THE INTERRUPT-STYLE PUMP: the same block written through
/// requests with polled = false, each completing off SPI1's vector,
/// then read back polled - is the ISR path's stream the polled one's?
volatile bool isr_done = false;
volatile uint32_t spi1_vectors = 0;
bool wait_reads_sr = false;   // DIAGNOSTIC: the wait loop reads SPI1's SR on the APB
bool isr_wait() {
    for (uint32_t spins = 20'000'000u; spins != 0u; --spins) {
        if (wait_reads_sr) {
            (void)Spi<1>::status();
        }
        if (isr_done) {
            return true;
        }
    }
    return false;
}
bool transfer_isr(uint8_t cmd, const uint8_t* tx, uint8_t* rx, uint16_t len, SpiClock rate) {
    static uint8_t c = 0;
    c = cmd;
    Host::Request r{};
    r.cs = Cs::ref();
    r.dc = Dc::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&c));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.polled = false;
    isr_done = false;
    if (Host::start(r)) {
        return true;   // the empty request completes synchronously
    }
    return isr_wait();
}

bool transfer_isr_dma(uint8_t cmd, const uint8_t* tx, uint8_t* rx, uint16_t len, SpiClock rate) {
    static uint8_t c = 0;
    c = cmd;
    DmaHost::Request r{};
    r.cs = Cs::ref();
    r.dc = Dc::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&c));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.polled = false;
    isr_done = false;
    if (DmaHost::start(r)) {
        return true;
    }
    return isr_wait();
}
bool transfer_dma_polled(uint8_t cmd, const uint8_t* tx, uint8_t* rx, uint16_t len, SpiClock rate) {
    static uint8_t c = 0;
    c = cmd;
    DmaHost::Request r{};
    r.cs = Cs::ref();
    r.dc = Dc::ref();
    r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&c));
    r.cmd_len = 1;
    r.tx = lend<Lease::reply>(tx);
    r.rx = lend<Lease::reply>(rx);
    r.len = len;
    r.clock = rate;
    r.mode = SpiMode::mode0;
    r.polled = true;
    return DmaHost::start(r);
}

void tv_isr_pump() {
    ili::note_format();
    constexpr uint16_t x = 100, y = 100, w = 60, h = 4;
    // First, one request at a time, with the vector counted: a 4-byte
    // CASET at the write rate, then at the slowest rate.
    {
        uint8_t q[4] = {0, 0, 1, 0x3F};
        for (uint8_t k = 0; k < 2u; ++k) {
            const SpiClock rate = k == 0u ? ili::write_rate : SpiClock::div256;
            const uint32_t before = spi1_vectors;
            const bool done = transfer_isr(ili::caset, q, nullptr, 4, rate);
            print(serial, "  CASET through the ISR at ", Host::sck_hz(rate) / 1000u, " kHz: ",
                  done ? "completed" : "NEVER COMPLETED", ", vector entered ", spi1_vectors - before,
                  " times (5 expected), NVIC ", Nvic::enabled(Spi<1>::irq) ? "enabled" : "DISABLED", " RXNEIE ",
                  Spi<1>::rxne_interrupt() ? "on" : "off", ", status ", Host::status(), ", SR 0x");
            hex2(static_cast<uint8_t>(Spi<1>::status()));
            print(serial, ", DC ", Dc::read_out() ? "high" : "LOW", ", CS ", Cs::read_out() ? "high" : "LOW", crlf);
            if (!done) {
                (void)Host::recover();
            }
        }
    }
    // The window through the ISR path, the rows through it too.
    uint8_t p[4];
    p[0] = static_cast<uint8_t>(x >> 8); p[1] = static_cast<uint8_t>(x);
    p[2] = static_cast<uint8_t>((x + w - 1u) >> 8); p[3] = static_cast<uint8_t>(x + w - 1u);
    bool ok = transfer_isr(ili::caset, p, nullptr, 4, ili::write_rate);
    p[0] = static_cast<uint8_t>(y >> 8); p[1] = static_cast<uint8_t>(y);
    p[2] = static_cast<uint8_t>((y + h - 1u) >> 8); p[3] = static_cast<uint8_t>(y + h - 1u);
    ok = transfer_isr(ili::paset, p, nullptr, 4, ili::write_rate) && ok;
    for (uint16_t r = 0; r < h; ++r) {
        for (uint16_t c = 0; c < w; ++c) {
            ili::put_pixel(ili::row_buf, c, ili::pattern(x + c, y + r, 0x4444));
        }
        ok = transfer_isr(r == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, w * 3u, ili::write_rate) && ok;
    }
    bench.verdict("every ISR-style request completed off the vector", ok);
    static uint8_t got[w * h * 3u];
    uint16_t first = 0;
    uint32_t bad = static_cast<uint32_t>(w) * h * 3u;
    if (ili::read_block(x, y, w, h, got, ili::read_rate)) {
        bad = ili::check_pattern(x, y, w, h, 0x4444, got, first);
    }
    print(serial, "  written through the ISR pump, read back polled: ", bad, " of ", static_cast<uint32_t>(w) * h * 3u,
          " bytes wrong; first bytes ");
    dump(got, 6);
    print(serial, crlf);
    bench.verdict("a block written through the ISR pump reads back exact", bad == 0u);
    // And a READ through the ISR path.
    static uint8_t got2[w * 3u + ili::read_extra];
    ili::window(x, y, x + w - 1u, y);
    ok = transfer_isr(ili::ramrd, nullptr, got2, static_cast<uint16_t>(w * 3u + ili::read_extra), ili::read_rate);
    uint8_t aligned[w * 3u];
    (void)ili::realign(got2, static_cast<uint16_t>(w * 3u + ili::read_extra), aligned, w * 3u, ili::read_format);
    for (uint16_t c = 0; c < w; ++c) {
        ili::put_pixel(ili::scratch, c, ili::pattern(x + c, y, 0x4444));
    }
    const uint32_t bad2 = ili::mismatches(ili::scratch, aligned, w * 3u, first);
    print(serial, "  the first row read through the ISR pump: ", bad2, " of ", w * 3u, " bytes wrong; raw ");
    dump(got2, 6);
    print(serial, crlf);
    bench.verdict("a row read through the ISR pump is exact", ok && bad2 == 0u);

    // The same block through the ISR pump at 12 MHz (the pump cannot
    // keep the wire busy there, but every byte must still be right).
    ok = transfer_isr(ili::caset, p, nullptr, 0, SpiClock::div8);
    uint8_t q[4];
    q[0] = static_cast<uint8_t>(x >> 8); q[1] = static_cast<uint8_t>(x);
    q[2] = static_cast<uint8_t>((x + w - 1u) >> 8); q[3] = static_cast<uint8_t>(x + w - 1u);
    ok = transfer_isr(ili::caset, q, nullptr, 4, SpiClock::div8);
    q[0] = static_cast<uint8_t>(y >> 8); q[1] = static_cast<uint8_t>(y);
    q[2] = static_cast<uint8_t>((y + h - 1u) >> 8); q[3] = static_cast<uint8_t>(y + h - 1u);
    ok = transfer_isr(ili::paset, q, nullptr, 4, SpiClock::div8) && ok;
    for (uint16_t r = 0; r < h; ++r) {
        for (uint16_t c = 0; c < w; ++c) {
            ili::put_pixel(ili::row_buf, c, ili::pattern(x + c, y + r, 0x5555));
        }
        ok = transfer_isr(r == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, w * 3u, SpiClock::div8) && ok;
    }
    bad = static_cast<uint32_t>(w) * h * 3u;
    if (ili::read_block(x, y, w, h, got, ili::read_rate)) {
        bad = ili::check_pattern(x, y, w, h, 0x5555, got, first);
    }
    print(serial, "  the block through the ISR pump at 12 MHz: ", bad, " bytes wrong; first bytes ");
    dump(got, 6);
    print(serial, crlf);
    bench.verdict("a block written through the ISR pump at 12 MHz reads back exact", ok && bad == 0u);

    // WHICH HALF FAILS AT 12 MHz: the window polled and the rows through
    // the ISR, then the window through the ISR and the rows polled.
    for (uint8_t variant = 0; variant < 2u; ++variant) {
        const uint32_t salt = 0x6000u + variant;
        const bool rows_isr = variant == 0u;
        if (rows_isr) {
            ili::window(x, y, x + w - 1u, y + h - 1u);   // polled, at the write rate
        } else {
            ok = transfer_isr(ili::caset, q, nullptr, 0, SpiClock::div8);
            q[0] = static_cast<uint8_t>(x >> 8); q[1] = static_cast<uint8_t>(x);
            q[2] = static_cast<uint8_t>((x + w - 1u) >> 8); q[3] = static_cast<uint8_t>(x + w - 1u);
            (void)transfer_isr(ili::caset, q, nullptr, 4, SpiClock::div8);
            q[0] = static_cast<uint8_t>(y >> 8); q[1] = static_cast<uint8_t>(y);
            q[2] = static_cast<uint8_t>((y + h - 1u) >> 8); q[3] = static_cast<uint8_t>(y + h - 1u);
            (void)transfer_isr(ili::paset, q, nullptr, 4, SpiClock::div8);
        }
        for (uint16_t r = 0; r < h; ++r) {
            for (uint16_t c = 0; c < w; ++c) {
                ili::put_pixel(ili::row_buf, c, ili::pattern(x + c, y + r, salt));
            }
            if (rows_isr) {
                (void)transfer_isr(r == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, w * 3u, SpiClock::div8);
            } else {
                (void)ili::transfer(r == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, w * 3u, SpiClock::div8);
            }
        }
        bad = static_cast<uint32_t>(w) * h * 3u;
        if (ili::read_block(x, y, w, h, got, ili::read_rate)) {
            bad = ili::check_pattern(x, y, w, h, salt, got, first);
        }
        print(serial, "  12 MHz, ", rows_isr ? "window polled + rows through the ISR" : "window through the ISR + rows polled",
              ": ", bad, " bytes wrong", crlf);
    }

    // The ISR at 12 MHz, the pad at very_high, with the WAIT LOOP READING
    // SR on the APB (what the polled path does between bytes).
    wait_reads_sr = true;
    {
        const uint32_t salt = 0x6100u;
        q[0] = static_cast<uint8_t>(x >> 8); q[1] = static_cast<uint8_t>(x);
        q[2] = static_cast<uint8_t>((x + w - 1u) >> 8); q[3] = static_cast<uint8_t>(x + w - 1u);
        (void)transfer_isr(ili::caset, q, nullptr, 4, SpiClock::div8);
        q[0] = static_cast<uint8_t>(y >> 8); q[1] = static_cast<uint8_t>(y);
        q[2] = static_cast<uint8_t>((y + h - 1u) >> 8); q[3] = static_cast<uint8_t>(y + h - 1u);
        (void)transfer_isr(ili::paset, q, nullptr, 4, SpiClock::div8);
        for (uint16_t r = 0; r < h; ++r) {
            for (uint16_t c = 0; c < w; ++c) {
                ili::put_pixel(ili::row_buf, c, ili::pattern(x + c, y + r, salt));
            }
            (void)transfer_isr(r == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, w * 3u, SpiClock::div8);
        }
        bad = static_cast<uint32_t>(w) * h * 3u;
        if (ili::read_block(x, y, w, h, got, ili::read_rate)) {
            bad = ili::check_pattern(x, y, w, h, salt, got, first);
        }
        print(serial, "  12 MHz through the ISR, pad very_high, the wait loop READING SR: ", bad, " bytes wrong", crlf);
    }
    wait_reads_sr = false;

    // The ISR at 12 MHz under each SCK pad speed class.
    static constexpr PinSpeed speeds[4] = {PinSpeed::low, PinSpeed::medium, PinSpeed::high, PinSpeed::very_high};
    static constexpr const char* speed_names[4] = {"low", "medium", "high", "very_high"};
    for (uint8_t k = 0; k < 4u; ++k) {
        Host::sck_speed(speeds[k]);
        const uint32_t salt = 0x7000u + k;
        q[0] = static_cast<uint8_t>(x >> 8); q[1] = static_cast<uint8_t>(x);
        q[2] = static_cast<uint8_t>((x + w - 1u) >> 8); q[3] = static_cast<uint8_t>(x + w - 1u);
        (void)transfer_isr(ili::caset, q, nullptr, 4, SpiClock::div8);
        q[0] = static_cast<uint8_t>(y >> 8); q[1] = static_cast<uint8_t>(y);
        q[2] = static_cast<uint8_t>((y + h - 1u) >> 8); q[3] = static_cast<uint8_t>(y + h - 1u);
        (void)transfer_isr(ili::paset, q, nullptr, 4, SpiClock::div8);
        for (uint16_t r = 0; r < h; ++r) {
            for (uint16_t c = 0; c < w; ++c) {
                ili::put_pixel(ili::row_buf, c, ili::pattern(x + c, y + r, salt));
            }
            (void)transfer_isr(r == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, w * 3u, SpiClock::div8);
        }
        bad = static_cast<uint32_t>(w) * h * 3u;
        if (ili::read_block(x, y, w, h, got, ili::read_rate)) {
            bad = ili::check_pattern(x, y, w, h, salt, got, first);
        }
        print(serial, "  12 MHz through the ISR with the SCK pad at ", speed_names[k], ": ", bad, " bytes wrong", crlf);
    }
    Host::sck_speed(PinSpeed::very_high);

    // A READ through the ISR at 12 MHz (the block holds the last salt).
    ili::window(x, y, x + w - 1u, y);
    ok = transfer_isr(ili::ramrd, nullptr, got2, static_cast<uint16_t>(w * 3u + ili::read_extra), SpiClock::div8);
    (void)ili::realign(got2, static_cast<uint16_t>(w * 3u + ili::read_extra), aligned, w * 3u, ili::read_format);
    for (uint16_t c = 0; c < w; ++c) {
        ili::put_pixel(ili::scratch, c, ili::pattern(x + c, y, 0x7003u));
    }
    print(serial, "  a row READ through the ISR at 12 MHz: ", ili::mismatches(ili::scratch, aligned, w * 3u, first), " of ",
          w * 3u, " bytes wrong; raw ");
    dump(got2, 6);
    print(serial, crlf);

    // The engined host, interrupt-style, at 6 and at 12 MHz.
    Host::release();
    if (!DmaHost::init(clock)) {
        print(serial, "  the engined host did not come up", crlf);
        (void)Host::init(clock);
        return;
    }
    dma_host_live = true;
    for (uint8_t k = 0; k < 3u; ++k) {
        const SpiClock rate = k == 0u ? SpiClock::div16 : SpiClock::div8;
        DmaHost::sck_speed(k == 2u ? PinSpeed::medium : PinSpeed::very_high);
        const uint32_t salt = 0x8000u + k;
        bool all = true;
        q[0] = static_cast<uint8_t>(x >> 8); q[1] = static_cast<uint8_t>(x);
        q[2] = static_cast<uint8_t>((x + w - 1u) >> 8); q[3] = static_cast<uint8_t>(x + w - 1u);
        all = transfer_isr_dma(ili::caset, q, nullptr, 4, rate) && all;
        q[0] = static_cast<uint8_t>(y >> 8); q[1] = static_cast<uint8_t>(y);
        q[2] = static_cast<uint8_t>((y + h - 1u) >> 8); q[3] = static_cast<uint8_t>(y + h - 1u);
        all = transfer_isr_dma(ili::paset, q, nullptr, 4, rate) && all;
        for (uint16_t r = 0; r < h; ++r) {
            for (uint16_t c = 0; c < w; ++c) {
                ili::put_pixel(ili::row_buf, c, ili::pattern(x + c, y + r, salt));
            }
            all = transfer_isr_dma(r == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, w * 3u, rate) && all;
        }
        // Read back through the engined host, polled.
        ili::window(x, y, x + w - 1u, y + h - 1u);   // ili::transfer goes through Host... the pump host is released:
        // so read through the engined one instead, row by row.
        bad = 0;
        for (uint16_t r = 0; r < h; ++r) {
            uint8_t wq[4];
            wq[0] = static_cast<uint8_t>(x >> 8); wq[1] = static_cast<uint8_t>(x);
            wq[2] = static_cast<uint8_t>((x + w - 1u) >> 8); wq[3] = static_cast<uint8_t>(x + w - 1u);
            (void)transfer_dma_polled(ili::caset, wq, nullptr, 4, ili::write_rate);
            wq[0] = static_cast<uint8_t>((y + r) >> 8); wq[1] = static_cast<uint8_t>(y + r);
            wq[2] = wq[0]; wq[3] = wq[1];
            (void)transfer_dma_polled(ili::paset, wq, nullptr, 4, ili::write_rate);
            (void)transfer_dma_polled(ili::ramrd, nullptr, got2, static_cast<uint16_t>(w * 3u + ili::read_extra), ili::read_rate);
            (void)ili::realign(got2, static_cast<uint16_t>(w * 3u + ili::read_extra), aligned, w * 3u, ili::read_format);
            for (uint16_t c = 0; c < w; ++c) {
                ili::put_pixel(ili::scratch, c, ili::pattern(x + c, y + r, salt));
            }
            bad += ili::mismatches(ili::scratch, aligned, w * 3u, first);
        }
        print(serial, "  the engined host, interrupt-style, at ", DmaHost::sck_hz(rate) / 1000u, " kHz with the SCK pad at ",
              k == 2u ? "medium" : "very_high", ": ", all ? "every request completed" : "A REQUEST NEVER COMPLETED", ", ",
              bad, " of ", static_cast<uint32_t>(w) * h * 3u, " bytes wrong", crlf);
        bench.verdict("the engined host's interrupt-style requests land exact at ",
                      k == 0u ? "6 MHz" : k == 1u ? "12 MHz, pad very_high" : "12 MHz, pad medium", all && bad == 0u);
    }
    dma_host_live = false;
    DmaHost::release();
    (void)Host::init(clock);
}

/// (by name) THE SAME SHORT TRANSACTION THREE WAYS, for a logic analyser
/// on SCK, MOSI, D/C and CS: a 4-pixel block at (100, 100) as CASET,
/// PASET, RAMWR of twelve bytes - then read back polled and judged.
/// One transaction per keypress; CS's falling edge is the trigger.
void la_case(bool isr, PinSpeed sck, uint32_t salt, const char* name) {
    constexpr uint16_t x = 100, y = 100, w = 4;
    Host::sck_speed(sck);
    uint8_t q[4];
    uint8_t px[w * 3u];
    for (uint16_t c = 0; c < w; ++c) {
        ili::put_pixel(px, c, ili::pattern(x + c, y, salt));
    }
    q[0] = static_cast<uint8_t>(x >> 8); q[1] = static_cast<uint8_t>(x);
    q[2] = static_cast<uint8_t>((x + w - 1u) >> 8); q[3] = static_cast<uint8_t>(x + w - 1u);
    Trigger::set();
    if (isr) { (void)transfer_isr(ili::caset, q, nullptr, 4, SpiClock::div8); }
    else     { (void)ili::transfer(ili::caset, q, nullptr, 4, SpiClock::div8); }
    Trigger::clear();
    q[0] = static_cast<uint8_t>(y >> 8); q[1] = static_cast<uint8_t>(y);
    q[2] = q[0]; q[3] = q[1];
    if (isr) { (void)transfer_isr(ili::paset, q, nullptr, 4, SpiClock::div8); }
    else     { (void)ili::transfer(ili::paset, q, nullptr, 4, SpiClock::div8); }
    if (isr) { (void)transfer_isr(ili::ramwr, px, nullptr, w * 3u, SpiClock::div8); }
    else     { (void)ili::transfer(ili::ramwr, px, nullptr, w * 3u, SpiClock::div8); }
    Host::sck_speed(PinSpeed::very_high);
    uint8_t got[w * 3u];
    uint16_t first = 0;
    uint32_t bad = w * 3u;
    if (ili::read_block(x, y, w, 1, got, ili::read_rate)) {
        bad = ili::mismatches(px, got, w * 3u, first);
    }
    print(serial, "  ", name, ": sent ");
    dump(px, 6);
    print(serial, "... read back ");
    dump(got, 6);
    print(serial, "... ", bad, " of 12 bytes wrong", crlf);
    bench.verdict(name, bad == 0u);
}
/// (by name) TWO MORE LEVERS at 12 MHz through the interrupt pump with
/// the SCK pad at very_high: the MOSI pad's slew, and the data pattern.
void tm_levers() {
    ili::note_format();
    constexpr uint16_t x = 100, y = 100, w = 60, h = 2;
    static uint8_t got[w * h * 3u];
    uint8_t q[4];
    const auto block = [&](uint32_t salt, uint8_t fixed, bool use_fixed) -> uint32_t {
        q[0] = static_cast<uint8_t>(x >> 8); q[1] = static_cast<uint8_t>(x);
        q[2] = static_cast<uint8_t>((x + w - 1u) >> 8); q[3] = static_cast<uint8_t>(x + w - 1u);
        (void)transfer_isr(ili::caset, q, nullptr, 4, SpiClock::div8);
        q[0] = static_cast<uint8_t>(y >> 8); q[1] = static_cast<uint8_t>(y);
        q[2] = static_cast<uint8_t>((y + h - 1u) >> 8); q[3] = static_cast<uint8_t>(y + h - 1u);
        (void)transfer_isr(ili::paset, q, nullptr, 4, SpiClock::div8);
        for (uint16_t r = 0; r < h; ++r) {
            for (uint16_t c = 0; c < w; ++c) {
                if (use_fixed) {
                    ili::row_buf[c * 3u] = fixed; ili::row_buf[c * 3u + 1u] = fixed; ili::row_buf[c * 3u + 2u] = fixed;
                } else {
                    ili::put_pixel(ili::row_buf, c, ili::pattern(x + c, y + r, salt));
                }
            }
            (void)transfer_isr(r == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr, w * 3u, SpiClock::div8);
        }
        uint32_t bad = static_cast<uint32_t>(w) * h * 3u;
        uint16_t first = 0;
        if (ili::read_block(x, y, w, h, got, ili::read_rate)) {
            if (use_fixed) {
                bad = 0;
                for (uint32_t i = 0; i < static_cast<uint32_t>(w) * h * 3u; ++i) {
                    if ((got[i] & 0xFCu) != (fixed & 0xFCu)) ++bad;
                }
            } else {
                bad = ili::check_pattern(x, y, w, h, salt, got, first);
            }
        }
        return bad;
    };
    Host::sck_speed(PinSpeed::very_high);
    // 1. the MOSI pad slowed to medium, SCK still very_high
    Host::mosi_speed(PinSpeed::medium);
    print(serial, "  MOSI pad medium, SCK very_high, pattern data: ", block(0xA100, 0, false), " bytes wrong", crlf);
    Host::mosi_speed(PinSpeed::very_high);
    // 2. the data pattern, MOSI back at very_high
    print(serial, "  MOSI very_high, all 0x00: ", block(0, 0x00, true), " bytes wrong", crlf);
    print(serial, "  MOSI very_high, all 0xFC: ", block(0, 0xFC, true), " bytes wrong", crlf);
    print(serial, "  MOSI very_high, all 0xA8: ", block(0, 0xA8, true), " bytes wrong", crlf);
    print(serial, "  MOSI very_high, the pattern: ", block(0xA200, 0, false), " bytes wrong", crlf);
    // 3. both pads medium
    Host::sck_speed(PinSpeed::medium);
    Host::mosi_speed(PinSpeed::medium);
    print(serial, "  MOSI medium, SCK medium, the pattern: ", block(0xA300, 0, false), " bytes wrong", crlf);
    Host::mosi_speed(PinSpeed::medium);   // the probe's own setting, restored
    Host::sck_speed(PinSpeed::very_high);
}

void tp_la_polled() { la_case(false, PinSpeed::very_high, 0x9100u + static_cast<uint32_t>(Ticker::millis() & 0xFFu), "polled, 12 MHz, SCK pad very_high"); }
void tq_la_isr() { la_case(true, PinSpeed::very_high, 0x9200u + static_cast<uint32_t>(Ticker::millis() & 0xFFu), "interrupt, 12 MHz, SCK pad very_high"); }
void tj_la_isr_medium() { la_case(true, PinSpeed::medium, 0x9300u + static_cast<uint32_t>(Ticker::millis() & 0xFFu), "interrupt, 12 MHz, SCK pad medium"); }

void tr_reset() {
    ili::hard_reset();
    ili::wake();
    ili::read_format_known = false;
    uint8_t raw[4];
    ili::read_raw(ili::rddpm, raw, 4);
    print(serial, "  RDDPM raw: ");
    dump(raw, 4);
    print(serial, crlf);
    bench.verdict("awake again after a hardware reset", (raw[0] & 0x1Fu) == 0x1Cu);
}

void ti_invert() {
    ili::inverted = !ili::inverted;
    ili::command(ili::inverted ? ili::invon : ili::invoff);
    print(serial, "  inversion ", ili::inverted ? "ON" : "OFF", crlf);
}

// ---- the simulator's assumptions, measured ----------------------------------------
/// The host's simulated panel (brio/host/sim_dcs_panel.hpp) states six
/// rules the bench had not measured, each marked ASSUMPTION there.
/// Letters w, z and s measure them in MEMORY coordinates under MADCTL
/// 0x00, with pixels that carry their own index - byte 0 = (i + 1) << 2,
/// byte 1 = 0x40, byte 2 = (63 - i) << 2 - so a read-back says WHICH
/// pixel landed WHERE. A verdict passes when the outcome is one the
/// letter can name; the finding is the text beside it.
namespace assume {

constexpr uint8_t bg[3] = {0xFC, 0x00, 0xFC};
constexpr uint16_t region_w = 16;
constexpr uint16_t region_h = 16;
uint8_t region_buf[region_w * region_h * 3u];
uint8_t px_buf[64 * 3];

void tag(uint8_t* p, uint8_t i) {
    p[0] = static_cast<uint8_t>((i + 1u) << 2);
    p[1] = 0x40;
    p[2] = static_cast<uint8_t>((63u - i) << 2);
}

/// The index a pixel carries, when it is one of ours.
std::optional<uint8_t> tag_of(const uint8_t* p) {
    if ((p[1] & 0xFCu) != 0x40u || (p[0] & 0xFCu) == 0u) {
        return std::nullopt;
    }
    const uint8_t i = static_cast<uint8_t>((p[0] >> 2) - 1u);
    if ((p[2] & 0xFCu) != static_cast<uint8_t>((63u - i) << 2)) {
        return std::nullopt;
    }
    return i;
}

bool is_bg(const uint8_t* p) {
    return (p[0] & 0xFCu) == 0xFCu && (p[1] & 0xFCu) == 0x00u && (p[2] & 0xFCu) == 0xFCu;
}

bool same_px(const uint8_t* a, const uint8_t* b) {
    return (a[0] & 0xFCu) == (b[0] & 0xFCu) && (a[1] & 0xFCu) == (b[1] & 0xFCu) && (a[2] & 0xFCu) == (b[2] & 0xFCu);
}

void fill_bg(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    ili::window(x, y, static_cast<uint16_t>(x + w - 1u), static_cast<uint16_t>(y + h - 1u));
    for (uint16_t c = 0; c < w; ++c) {
        ili::row_buf[c * 3u] = bg[0];
        ili::row_buf[c * 3u + 1u] = bg[1];
        ili::row_buf[c * 3u + 2u] = bg[2];
    }
    for (uint16_t r = 0; r < h; ++r) {
        (void)ili::transfer(r == 0u ? ili::ramwr : ili::ramwr_continue, ili::row_buf, nullptr,
                            static_cast<uint16_t>(w * 3u), ili::write_rate);
    }
}

bool read_region(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    return ili::read_block(x, y, w, h, region_buf, ili::read_rate);
}

struct Where {
    uint16_t x;
    uint16_t y;
};

/// Where tag `i` sits in the region last read (absolute coordinates).
std::optional<Where> find_in_region(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t i) {
    for (uint16_t r = 0; r < h; ++r) {
        for (uint16_t c = 0; c < w; ++c) {
            const uint8_t* p = region_buf + (static_cast<uint32_t>(r) * w + c) * 3u;
            const std::optional<uint8_t> t = tag_of(p);
            if (t && *t == i) {
                return Where{static_cast<uint16_t>(x0 + c), static_cast<uint16_t>(y0 + r)};
            }
        }
    }
    return std::nullopt;
}

/// The whole frame scanned for tag `i`, a row at a time - the fallback
/// when a region has not got it. About two seconds at 3 MHz.
std::optional<Where> find_in_frame(uint8_t i) {
    for (uint16_t y = 0; y < ili::height; ++y) {
        if (!ili::read_block(0, y, ili::width, 1, ili::scratch, ili::read_rate)) {
            return std::nullopt;
        }
        for (uint16_t x = 0; x < ili::width; ++x) {
            const std::optional<uint8_t> t = tag_of(ili::scratch + x * 3u);
            if (t && *t == i) {
                return Where{x, y};
            }
        }
    }
    return std::nullopt;
}

void print_where(const std::optional<Where>& w) {
    if (w) {
        print(serial, "(", w->x, ",", w->y, ")");
    } else {
        print(serial, "nowhere");
    }
}

void print_px(const uint8_t* p) {
    hex2(p[0]);
    print(serial, " ");
    hex2(p[1]);
    print(serial, " ");
    hex2(p[2]);
}

void describe(const uint8_t* p) {
    const std::optional<uint8_t> t = tag_of(p);
    if (t) {
        print(serial, "tag ", *t);
    } else if (is_bg(p)) {
        print(serial, "background");
    } else {
        print(serial, "bytes ");
        print_px(p);
    }
}

/// `n` pixels read with `cmd`, exactly clocked, into px_buf.
void read_pixels(uint8_t cmd, uint16_t n) {
    const uint16_t bytes = static_cast<uint16_t>(n * 3u);
    const uint16_t exact = static_cast<uint16_t>(ili::read_format.offset + bytes + (ili::read_format.shift != 0u ? 1u : 0u));
    ili::read_raw(cmd, ili::raw_buf, exact);
    (void)ili::realign(ili::raw_buf, exact, px_buf, bytes, ili::read_format);
}

/// `n` tagged pixels, `first` the first index, written with `cmd`.
void write_tags(uint8_t cmd, uint8_t first, uint8_t n) {
    for (uint8_t i = 0; i < n; ++i) {
        tag(ili::row_buf + i * 3u, static_cast<uint8_t>(first + i));
    }
    (void)ili::transfer(cmd, ili::row_buf, nullptr, static_cast<uint16_t>(n * 3u), ili::write_rate);
}

/// The pixel of the region last read at absolute (x, y).
const uint8_t* at(uint16_t x0, uint16_t y0, uint16_t w, uint16_t x, uint16_t y) {
    return region_buf + (static_cast<uint32_t>(y - y0) * w + (x - x0)) * 3u;
}

}  // namespace assume

/// ASSUMPTIONS 1, 2 AND 6: a write past the window's last pixel, a read
/// past it, and what re-issuing the window does to the two pointers.
void tw_window_edges() {
    ili::note_format();
    ili::set_madctl(0x00);
    constexpr uint16_t rx = 100, ry = 200, rh = 8;          // the background region, 16 x 8
    constexpr uint16_t wx = 104, wy = 202, ww = 4, wh = 2;  // the window inside it: eight pixels
    const auto open_window = [] { ili::window(wx, wy, wx + ww - 1u, wy + wh - 1u); };
    const auto region = [] { (void)assume::read_region(rx, ry, assume::region_w, rh); };
    const auto find = [](uint8_t i) { return assume::find_in_region(rx, ry, assume::region_w, rh, i); };
    const auto slot = [](uint8_t i) {   // the window's i-th pixel, column-first
        return assume::at(rx, ry, assume::region_w, static_cast<uint16_t>(wx + i % ww), static_cast<uint16_t>(wy + i / ww));
    };

    // 0. Eight pixels into a window of eight: the walk this letter reads by.
    assume::fill_bg(rx, ry, assume::region_w, rh);
    open_window();
    assume::write_tags(ili::ramwr, 0, 8);
    region();
    bool in_order = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        const std::optional<uint8_t> t = assume::tag_of(slot(i));
        if (!t || *t != i) {
            in_order = false;
        }
    }
    bench.verdict("eight pixels fill the window column-first, page by page (MADCTL 0x00)", in_order);

    // 1. Twelve pixels into a window of eight.
    assume::fill_bg(rx, ry, assume::region_w, rh);
    open_window();
    assume::write_tags(ili::ramwr, 0, 12);
    region();
    print(serial, "  the window's first four pixels hold ");
    for (uint8_t i = 0; i < 4u; ++i) {
        assume::describe(slot(i));
        print(serial, i < 3u ? ", " : "");
    }
    print(serial, crlf);
    std::optional<assume::Where> ninth = find(8);
    const char* outcome = nullptr;
    if (ninth) {
        outcome = ninth->x == wx && ninth->y == wy ? "it WRAPS to the window's first pixel" : "it lands in the region, off the window";
    } else {
        ninth = assume::find_in_frame(8);
        outcome = ninth ? "it continues OUTSIDE the window" : "it is DROPPED";
    }
    print(serial, "  the ninth pixel of twelve: ");
    assume::print_where(ninth);
    print(serial, crlf);
    bench.verdict("assumption 1, a write past the window's last pixel: ", outcome, true);

    // 2. Twelve pixels read from the same window of eight.
    open_window();
    assume::read_pixels(ili::ramrd, 12);
    print(serial, "  pixels 8..11 of a twelve-pixel read: ");
    for (uint8_t i = 8; i < 12u; ++i) {
        assume::print_px(assume::px_buf + i * 3u);
        print(serial, i < 11u ? " | " : "");
    }
    print(serial, crlf);
    const uint8_t* p8 = assume::px_buf + 8u * 3u;
    const char* r2 = ((p8[0] | p8[1] | p8[2]) & 0xFCu) == 0u ? "0x00"
                     : assume::same_px(p8, assume::px_buf)   ? "the window's first pixel again (a WRAP)"
                     : (p8[0] & p8[1] & p8[2] & 0xFCu) == 0xFCu ? "0xFF, the line undriven"
                                                                 : "another value (see the bytes)";
    bench.verdict("assumption 2, a read past the window's last pixel answers ", r2, true);

    // 6. The write pointer on a re-issued window, on CASET alone, on PASET alone.
    assume::fill_bg(rx, ry, assume::region_w, rh);
    open_window();
    assume::write_tags(ili::ramwr, 20, 2);
    open_window();
    assume::write_tags(ili::ramwr_continue, 22, 1);
    region();
    std::optional<assume::Where> w22 = find(22);
    const char* r6a = !w22 ? "3Ch after it writes nothing" : (w22->x == wx && w22->y == wy) ? "it REWINDS the write pointer"
                    : (w22->x == wx + 2u && w22->y == wy) ? "it leaves the write pointer where it was" : "3Ch lands elsewhere";
    print(serial, "  CASET + PASET re-issued after two pixels, then 3Ch: ");
    assume::print_where(w22);
    print(serial, crlf);
    bench.verdict("assumption 6, a window re-issued: ", r6a, true);

    // The same with a DIFFERENT window (one column to the right): does a
    // window that changes rewind, where one re-issued unchanged did not?
    assume::fill_bg(rx, ry, assume::region_w, rh);
    open_window();
    assume::write_tags(ili::ramwr, 24, 2);
    ili::window(wx + 1u, wy, wx + ww, wy + wh - 1u);
    assume::write_tags(ili::ramwr_continue, 26, 1);
    region();
    std::optional<assume::Where> w26 = find(26);
    print(serial, "  a window one column to the right after two pixels, then 3Ch: ");
    assume::print_where(w26);
    print(serial, !w26 ? " - nothing lands" : (w26->x == wx + 1u && w26->y == wy) ? " - a CHANGED window rewinds to its own origin"
          : (w26->x == wx + 2u && w26->y == wy) ? " - the pointer stays at the third pixel of the frame"
          : (w26->x == wx + 3u && w26->y == wy) ? " - the pointer stays at the third pixel of the NEW window" : " - elsewhere", crlf);

    uint8_t p[4];
    assume::fill_bg(rx, ry, assume::region_w, rh);
    open_window();
    assume::write_tags(ili::ramwr, 30, 2);
    p[0] = static_cast<uint8_t>(wx >> 8);
    p[1] = static_cast<uint8_t>(wx);
    p[2] = static_cast<uint8_t>((wx + ww - 1u) >> 8);
    p[3] = static_cast<uint8_t>(wx + ww - 1u);
    ili::command(ili::caset, p, 4);
    assume::write_tags(ili::ramwr_continue, 32, 1);
    region();
    std::optional<assume::Where> w32 = find(32);
    print(serial, "  CASET alone after two pixels, then 3Ch: ");
    assume::print_where(w32);
    print(serial, w32 && w32->x == wx && w32->y == wy ? " - CASET alone rewinds" : " - CASET alone does not rewind", crlf);

    assume::fill_bg(rx, ry, assume::region_w, rh);
    open_window();
    assume::write_tags(ili::ramwr, 40, 2);
    p[0] = static_cast<uint8_t>(wy >> 8);
    p[1] = static_cast<uint8_t>(wy);
    p[2] = static_cast<uint8_t>((wy + wh - 1u) >> 8);
    p[3] = static_cast<uint8_t>(wy + wh - 1u);
    ili::command(ili::paset, p, 4);
    assume::write_tags(ili::ramwr_continue, 42, 1);
    region();
    std::optional<assume::Where> w42 = find(42);
    print(serial, "  PASET alone after two pixels, then 3Ch: ");
    assume::print_where(w42);
    print(serial, w42 && w42->x == wx && w42->y == wy ? " - PASET alone rewinds" : " - PASET alone does not rewind", crlf);

    // 6b. The read pointer on a re-issued window.
    assume::fill_bg(rx, ry, assume::region_w, rh);
    open_window();
    assume::write_tags(ili::ramwr, 0, 8);
    open_window();
    assume::read_pixels(ili::ramrd, 2);
    open_window();
    assume::read_pixels(ili::ramrd_continue, 1);
    std::optional<uint8_t> t = assume::tag_of(assume::px_buf);
    print(serial, "  RAMRD of two, the window re-issued, 3Eh of one: ");
    assume::describe(assume::px_buf);
    print(serial, crlf);
    bench.verdict("assumption 6, the read pointer on a re-issued window: ",
                  t && *t == 0u ? "REWOUND" : t && *t == 2u ? "left where it was" : "neither reading", true);

    // 6c. Are the two pointers one? A write between two reads, a read between two writes.
    open_window();
    assume::read_pixels(ili::ramrd, 2);
    assume::write_tags(ili::ramwr, 50, 1);   // RAMWR restarts at the origin: tag 50 on pixel 0
    assume::read_pixels(ili::ramrd_continue, 1);
    t = assume::tag_of(assume::px_buf);
    print(serial, "  RAMRD of two, RAMWR of one, 3Eh of one: ");
    assume::describe(assume::px_buf);
    print(serial, t && *t == 2u ? " - the read pointer ignores the write" : t && *t == 1u ? " - ONE pointer: the read continues after the write" : t && *t == 50u ? " - the read pointer was rewound by the write" : "", crlf);

    open_window();
    assume::write_tags(ili::ramwr, 60, 1);   // write pointer at 1
    assume::read_pixels(ili::ramrd, 3);      // read pointer at 3
    assume::write_tags(ili::ramwr_continue, 61, 1);
    region();
    std::optional<assume::Where> w61 = find(61);
    print(serial, "  RAMWR of one, RAMRD of three, 3Ch of one: ");
    assume::print_where(w61);
    print(serial, w61 && w61->x == wx + 1u ? " - the write pointer ignores the read" : w61 && w61->x == wx + 3u ? " - ONE pointer: the write continues after the read" : w61 && w61->x == wx ? " - the write pointer was rewound by the read" : "", crlf);

    ili::set_rotation(ili::rotation);
}

/// ASSUMPTION 3: windows the axes cannot hold - a start beyond the
/// axis, an end beyond it, a range that runs backwards, pages beyond 479
/// and a page range that runs backwards, under MADCTL 0x00. Before each
/// a SENTINEL window is set (columns 8..11 on page 106), so that a
/// command the controller IGNORES is told from a write it DROPS: the
/// pixels of an ignored one land in the sentinel. Never more pixels than
/// the smallest window in question holds, because a write past a
/// window's end wraps to its start and hides the first pixels (letter w).
void tl_invalid_windows() {
    ili::note_format();
    ili::set_madctl(0x00);
    constexpr uint16_t py = 100, ph = 8;
    constexpr uint16_t sx = 8, sy = 106;   // the sentinel's origin
    const auto sentinel = [] { ili::window(sx, sy, sx + 3u, sy); };
    const auto regions = [] {
        assume::fill_bg(304, py, 16, ph);   // the right edge
        assume::fill_bg(0, py, 16, ph);     // the left edge, the sentinel inside it
        assume::fill_bg(200, py, 16, ph);   // the middle
    };
    const auto classify = [](const std::optional<assume::Where>& w, uint16_t cx, uint16_t cy, const char* taken,
                             const char* clamped) -> const char* {
        if (!w) {
            return "the write is DROPPED";
        }
        if (w->x == sx && w->y == sy) {
            return "the command is IGNORED, the window before it stands";
        }
        if (w->x == cx && w->y == cy) {
            return clamped;
        }
        return taken;
    };
    uint8_t p[4];
    const auto caset = [&p](uint16_t a, uint16_t b) {
        p[0] = static_cast<uint8_t>(a >> 8);
        p[1] = static_cast<uint8_t>(a);
        p[2] = static_cast<uint8_t>(b >> 8);
        p[3] = static_cast<uint8_t>(b);
        ili::command(ili::caset, p, 4);
    };
    const auto paset = [&p](uint16_t a, uint16_t b) {
        p[0] = static_cast<uint8_t>(a >> 8);
        p[1] = static_cast<uint8_t>(a);
        p[2] = static_cast<uint8_t>(b >> 8);
        p[3] = static_cast<uint8_t>(b);
        ili::command(ili::paset, p, 4);
    };

    // (a) a start beyond the axis: CASET 400..403, one pixel.
    regions();
    sentinel();
    caset(400, 403);
    assume::write_tags(ili::ramwr, 0, 1);
    std::optional<assume::Where> w = assume::find_in_frame(0);
    print(serial, "  CASET 400..403 after the sentinel: the pixel ");
    assume::print_where(w);
    print(serial, crlf);
    bench.verdict("assumption 3a, a start beyond the axis: ",
                  classify(w, 319, sy, w && w->x == 80u ? "the address WRAPS (400 - 320)" : "it lands elsewhere",
                           "it lands on the LAST column"), true);

    // (b) an end beyond the axis: CASET 316..323 on page 106 - one pixel
    //     first (ignored, dropped or taken), then six if the start was
    //     taken: four fit before the axis, and where the fifth goes says
    //     whether the end is clamped (it wraps to 316) or not.
    regions();
    sentinel();
    caset(316, 323);
    assume::write_tags(ili::ramwr, 10, 1);
    w = assume::find_in_frame(10);
    print(serial, "  CASET 316..323 after the sentinel: one pixel ");
    assume::print_where(w);
    print(serial, crlf);
    const char* r3b = classify(w, 316, sy, "it lands elsewhere", "the START is taken");
    if (w && w->x == 316u && w->y == sy) {
        regions();
        caset(316, 323);
        paset(sy, sy);
        assume::write_tags(ili::ramwr, 10, 6);
        (void)assume::read_region(304, py, 16, ph);
        const std::optional<uint8_t> at316 = assume::tag_of(assume::at(304, py, 16, 316, sy));
        std::optional<assume::Where> b14 = assume::find_in_region(304, py, 16, ph, 14);
        if (!b14) {
            b14 = assume::find_in_frame(14);
        }
        print(serial, "  CASET 316..323, six pixels: column 316 holds ");
        assume::describe(assume::at(304, py, 16, 316, sy));
        print(serial, ", the fifth pixel ");
        assume::print_where(b14);
        print(serial, crlf);
        r3b = (at316 && *at316 == 14u) ? "the start is taken and the end is CLAMPED to the axis: the fifth pixel wraps to the start"
              : (b14 && b14->x == 0u) ? "the start is taken and the walk runs past the edge into column 0"
              : b14 ? "the start is taken and the fifth pixel lands elsewhere"
                    : "the start is taken and the pixels past the axis are DROPPED";
    }
    bench.verdict("assumption 3b, an end beyond the axis: ", r3b, true);

    // (c) a range that runs backwards: CASET 210..205 on page 106, one pixel.
    regions();
    sentinel();
    caset(210, 205);
    assume::write_tags(ili::ramwr, 20, 1);
    w = assume::find_in_frame(20);
    print(serial, "  CASET 210..205 after the sentinel: the pixel ");
    assume::print_where(w);
    print(serial, crlf);
    bench.verdict("assumption 3c, a column range that runs backwards: ",
                  classify(w, 210, sy, "it lands elsewhere", "the START is taken"), true);

    // (d) pages beyond the axis: CASET 304..307 (valid) then PASET 500..501, one pixel.
    regions();
    sentinel();
    caset(304, 307);
    paset(500, 501);
    assume::write_tags(ili::ramwr, 30, 1);
    w = assume::find_in_frame(30);
    print(serial, "  CASET 304..307, PASET 500..501: the pixel ");
    assume::print_where(w);
    print(serial, crlf);
    bench.verdict("assumption 3d, pages beyond the axis: ",
                  !w ? "the write is DROPPED" : (w->x == 304u && w->y == sy) ? "PASET is IGNORED, the page window before it stands"
                  : (w->x == 304u && w->y == 479u) ? "it lands on the LAST page" : (w->x == 304u && w->y == 20u) ? "the address WRAPS (500 - 480)" : "it lands elsewhere", true);

    // (e) a page range that runs backwards: CASET 304..307 then PASET 105..103, one pixel.
    regions();
    sentinel();
    caset(304, 307);
    paset(105, 103);
    assume::write_tags(ili::ramwr, 40, 1);
    w = assume::find_in_frame(40);
    print(serial, "  CASET 304..307, PASET 105..103: the pixel ");
    assume::print_where(w);
    print(serial, crlf);
    bench.verdict("assumption 3e, a page range that runs backwards: ",
                  !w ? "the write is DROPPED" : (w->x == 304u && w->y == sy) ? "PASET is IGNORED, the page window before it stands"
                  : (w->x == 304u && w->y == 105u) ? "the START is taken" : "it lands elsewhere", true);

    // (f) a read from a window that runs backwards: what the bytes say.
    regions();
    ili::window(200, sy, 203, sy);
    assume::write_tags(ili::ramwr, 44, 4);
    caset(210, 205);
    assume::read_pixels(ili::ramrd, 2);
    print(serial, "  RAMRD of two from CASET 210..205 (200..203 written before): ");
    assume::print_px(assume::px_buf);
    print(serial, " | ");
    assume::print_px(assume::px_buf + 3);
    print(serial, crlf);
    const uint8_t* q = assume::px_buf;
    bench.verdict("assumption 3f, a read from a backwards window answers ",
                  ((q[0] | q[1] | q[2]) & 0xFCu) == 0u ? "0x00" : (q[0] & q[1] & q[2] & 0xFCu) == 0xFCu ? "0xFF"
                  : assume::tag_of(q) && *assume::tag_of(q) == 44u ? "the window before it (its first pixel)"
                  : assume::tag_of(q) ? "a pixel of the frame (see the tag)" : "another value", true);

    // (g) the write pointer as an ADDRESS: two pixels into 200..203 x 106,
    //     then a window far away (220..223 x 104..105), then three
    //     pixels through 3Ch - where do they go?
    regions();
    ili::window(200, sy, 203, sy);
    assume::write_tags(ili::ramwr, 50, 2);
    ili::window(220, py + 4u, 223, py + 5u);
    assume::write_tags(ili::ramwr_continue, 52, 3);
    (void)assume::read_region(200, py, 16, ph);
    std::optional<assume::Where> g52 = assume::find_in_region(200, py, 16, ph, 52);
    std::optional<assume::Where> g53 = assume::find_in_region(200, py, 16, ph, 53);
    std::optional<assume::Where> g54 = assume::find_in_region(200, py, 16, ph, 54);
    if (!g52) {
        g52 = assume::find_in_frame(52);
    }
    print(serial, "  pointer at (202,106), window moved to 220..223 x 104..105, 3Ch of three: ");
    assume::print_where(g52);
    print(serial, " ");
    assume::print_where(g53);
    print(serial, " ");
    assume::print_where(g54);
    print(serial, crlf);
    bench.verdict("assumption 6b, a pointer outside the new window: ",
                  !g52 ? "the write is DROPPED" : (g52->x == 202u && g52->y == sy) ? "the ADDRESS stands and the pixel lands there, the following ones tell the step"
                  : (g52->x == 220u && g52->y == py + 4u) ? "the pointer is pulled to the new window's origin" : "it lands elsewhere", true);

    ili::set_rotation(ili::rotation);
}

/// ASSUMPTIONS 4 AND 5: MADCTL written after the window, and a pixel cut
/// in two across a write and its continuation.
void ts_sequence() {
    ili::note_format();
    ili::set_madctl(0x00);

    // 4. CASET 104..107, PASET 202..203 under MADCTL 0x00, then B5, then eight pixels.
    //    Region A holds the window as written (columns 104..107 x pages
    //    202..203); region B holds it with the axes exchanged (columns
    //    202..203 x pages 104..107).
    assume::fill_bg(100, 200, 16, 8);
    assume::fill_bg(200, 100, 8, 16);
    ili::window(104, 202, 107, 203);
    ili::set_madctl(0x20);
    assume::write_tags(ili::ramwr, 0, 8);
    ili::set_madctl(0x00);
    (void)assume::read_region(100, 200, 16, 8);
    std::optional<assume::Where> a0 = assume::find_in_region(100, 200, 16, 8, 0);
    std::optional<assume::Where> a1 = assume::find_in_region(100, 200, 16, 8, 1);
    const char* r4 = nullptr;
    if (a0) {
        r4 = (a1 && a1->x == 104u && a1->y == 203u) ? "the registers keep their bytes and B5 only reorders the walk: pages first inside the window as written"
             : (a1 && a1->x == 105u && a1->y == 202u) ? "the late B5 changes nothing: columns first inside the window as written"
                                                        : "the window as written, in another order";
    } else {
        (void)assume::read_region(200, 100, 8, 16);
        a0 = assume::find_in_region(200, 100, 8, 16, 0);
        a1 = assume::find_in_region(200, 100, 8, 16, 1);
        if (a0) {
            r4 = (a1 && a1->x == 202u && a1->y == 105u) ? "the late B5 REASSIGNS the axes: CASET's bytes become the pages, and the walk steps the pages first"
                 : (a1 && a1->x == 203u && a1->y == 104u) ? "the late B5 REASSIGNS the axes: CASET's bytes become the pages, and the walk steps the columns first"
                                                            : "the axes exchanged, in another order";
        } else {
            a0 = assume::find_in_frame(0);
            r4 = a0 ? "the pixels land elsewhere in the frame" : "nothing lands";
        }
    }
    print(serial, "  window written under 0x00, MADCTL 0x20 after it, eight pixels: the first at ");
    assume::print_where(a0);
    print(serial, ", the second at ");
    assume::print_where(a1);
    print(serial, crlf);
    bench.verdict("assumption 4, MADCTL after the window: ", r4, true);

    // 5. A window of three on page 206; RAMWR carries pixel 50 and the
    //    first byte of 51; 3Ch carries the rest of 51 and the whole of 52.
    assume::fill_bg(100, 200, 16, 8);
    ili::window(104, 206, 106, 206);
    uint8_t t50[3], t51[3], t52[3];
    assume::tag(t50, 50);
    assume::tag(t51, 51);
    assume::tag(t52, 52);
    uint8_t first[4] = {t50[0], t50[1], t50[2], t51[0]};
    uint8_t rest[5] = {t51[1], t51[2], t52[0], t52[1], t52[2]};
    (void)ili::transfer(ili::ramwr, first, nullptr, 4, ili::write_rate);
    (void)ili::transfer(ili::ramwr_continue, rest, nullptr, 5, ili::write_rate);
    (void)assume::read_region(100, 200, 16, 8);
    const uint8_t* s0 = assume::at(100, 200, 16, 104, 206);
    const uint8_t* s1 = assume::at(100, 200, 16, 105, 206);
    const uint8_t* s2 = assume::at(100, 200, 16, 106, 206);
    print(serial, "  the three pixels: ");
    assume::describe(s0);
    print(serial, " | ");
    assume::describe(s1);
    print(serial, " | ");
    assume::describe(s2);
    print(serial, crlf);
    const uint8_t restart[3] = {t51[1], t51[2], t52[0]};
    const std::optional<uint8_t> k1 = assume::tag_of(s1);
    const std::optional<uint8_t> k2 = assume::tag_of(s2);
    const char* r5 = (k1 && *k1 == 51u && k2 && *k2 == 52u) ? "the partial pixel is CARRIED into the continuation"
                     : assume::same_px(s1, restart)         ? "the continuation restarts the byte phase; the byte left over is dropped"
                     : assume::is_bg(s1)                    ? "the continuation writes nothing where the partial pixel was"
                                                            : "another arrangement (see the bytes)";
    bench.verdict("assumption 5, a pixel cut across a write and its 3Ch: ", r5, assume::tag_of(s0).has_value());

    ili::set_rotation(ili::rotation);
}

void banner() {
    print(serial, crlf, "ILI9481 probe on the STM32F411CE black pill: SPI1 PA5/PA6/PA7, CS PB2, RST PB1, DC PB0",
          crlf, "sysclk ", SysClock::hz / 1000000u, " MHz, PCLK2 ", SysClock::pclk2_hz / 1000000u,
          " MHz; write SCK ", Host::sck_hz(ili::write_rate) / 1000u, " kHz, read SCK ",
          Host::sck_hz(ili::read_rate) / 1000u, " kHz; errata APB ceiling at very_high ",
          Host::errata_apb_ceiling_hz() / 1000000u, " MHz (", Host::within_errata_ceiling() ? "inside" : "ABOVE", ")",
          crlf);
    bench.menu();
}

}  // namespace

// ---- target glue ------------------------------------------------------------
extern "C" void SPI1_IRQHandler() {
    if (dma_host_live) {
        if (DmaHost::isr()) {
            isr_done = true;
        }
        return;
    }
    spi1_vectors = spi1_vectors + 1u;
    if (Host::isr()) {
        isr_done = true;
    }
}
extern "C" void DMA2_Stream3_IRQHandler() {
    if (dma_host_live && DmaHost::dma_isr()) {
        isr_done = true;
    }
}
extern "C" void DMA2_Stream0_IRQHandler() {
    if (dma_host_live && DmaHost::dma_isr()) {
        isr_done = true;
    }
}
extern "C" void OTG_FS_IRQHandler() { Device::isr(); }
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }

int main() {
    const bool clock_ok = SysClock::init();
    (void)brio::Ticker::init(clock);
    Led::output(true);   // lit when low
    brio::enable_interrupts();

    // The control lines on FAST pads: their setup before a byte's first
    // clock edge is half an SCK period, 42 ns at 12 MHz, and a pad at
    // `low` slews in that order of time (letter v's finding).
    Cs::output(true, {.speed = PinSpeed::very_high});
    Rst::output(true);
    Dc::output(true, {.speed = PinSpeed::very_high});
    Tcs::output(true);
    Trigger::output(false);
    const bool spi_ok = Host::init(clock);
    // The breadboard's rule (docs/stm32f4/spi.md): at PCLK2/8 the data
    // pad's edge is too fast for these wires, so MOSI goes out at medium
    // on both hosts and SCK keeps the slew the errata ask for.
    Host::mosi_speed(PinSpeed::medium);
    DmaHost::mosi_speed(PinSpeed::medium);
    const bool usb_ok = Usb::init(clock);
    if (usb_ok) {
        Device::start();
    }

    // The panel comes up before any host is listening: the picture
    // needs no terminal.
    ili::hard_reset();
    ili::wake();
    Led::clear();

    bench.letter('a', "the controller answers: the read registers and the device code", ta_identity);
    bench.letter('b', "the read format: a block written, its GRAM read back raw, the alignment found", tb_read_format);
    bench.letter('c', "read_memory_continue (3Eh) after a RAMRD", tc_read_continue);
    bench.letter('d', "the whole frame written, read back and compared, timed", td_full_frame);
    bench.letter('k', "the whole frame through the DMA engines, at 6 and at 12 MHz, read back", tk_engines);
    bench.letter('e', "the write rate ladder, judged by read-back", te_write_rates);
    bench.letter('f', "the read rate ladder, judged by read-back", tf_read_rates);
    bench.letter('g', "the scan orders of MADCTL, each written and read back", tg_madctl);
    bench.letter('h', "brio/gfx over the panel, one word read back against the font", th_gfx);
    bench.letter('o', "the marker picture in the rotation in force, then the next rotation", to_turn, false);
    bench.letter('n', "the display-side bits of MADCTL, one step a call, the memory untouched", tn_display_bits, false);
    bench.letter('u', "the USB controller's counters", tu_usb, false);
    bench.letter('m', "12 MHz through the interrupt pump: the MOSI pad's slew and the data pattern", tm_levers, false);
    bench.letter('p', "for the logic analyser: one short transaction, polled at 12 MHz, SCK pad very_high", tp_la_polled, false);
    bench.letter('q', "for the logic analyser: the same through the interrupt pump, SCK pad very_high", tq_la_isr, false);
    bench.letter('j', "for the logic analyser: the same through the interrupt pump, SCK pad medium", tj_la_isr_medium, false);
    bench.letter('v', "the interrupt-style pump: a block written and read through it", tv_isr_pump, false);
    bench.letter('y', "what the frame holds, untouched: a census by colour with boxes", ty_census, false);
    bench.letter('x', "where a window lands under B5: the frame scanned for four colours", tx_b5_range, false);
    bench.letter('r', "hardware reset and the wake sequence again", tr_reset, false);
    bench.letter('i', "toggle the inversion", ti_invert, false);
    bench.letter('w', "the simulator's assumptions 1, 2, 6: the window's edges and the two pointers, measured", tw_window_edges, false);
    bench.letter('l', "the simulator's assumption 3: windows the axes cannot hold, measured", tl_invalid_windows, false);
    bench.letter('s', "the simulator's assumptions 4, 5: MADCTL after the window, a pixel cut across 3Ch, measured", ts_sequence, false);

    while (!Serial::configured() || !Serial::dtr()) {
        P::CriticalSection cs;
        P::idle();
    }
    print(serial, crlf, "boot: clk=", clock_ok ? "PLL96" : "FAILED", " spi=", spi_ok ? "up" : "FAILED",
          " usb=", usb_ok ? "up" : "FAILED", crlf);
    banner();
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            P::CriticalSection cs;
            P::idle();
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "? for help", crlf);
        }
        bench.prompt();
    }
}
