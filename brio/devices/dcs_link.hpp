/*
 * dcs_link.hpp (devices)
 *
 * The link a command panel is spoken to over, as a concept - and the
 * four-wire serial realization of it (clock, data, select and a
 * data/command line) over any stratum's SpiHost. A link carries one
 * command and the bytes that belong with it as one transaction on the
 * bus, and knows nothing else. A panel driver (devices/dcs_panel.hpp) is
 * the vocabulary (devices/dcs.hpp) over a controller's traits
 * (devices/ili9481.hpp: a struct of constexpr facts about one silicon)
 * over one of these, and those four layers are the command tier of
 * docs/design/gfx.md ("The command tier"); docs/devices/dcs_link.md is
 * this file's document.
 *
 * The smallest use, a link over a family's own host (docs/devices/
 * dcs_link.md, "How to use it"): the application fills two requests of
 * its own bus as it fills any other, a write prototype and a read
 * prototype at a lower rate, and the link copies them once:
 *
 *     using PanelHost = brio::SpiHost<1, panel_pins>;
 *     using PanelLink = brio::DcsSerialLink<PanelHost>;
 *
 *     PanelHost::Request write{};
 *     write.cs = Cs::ref();
 *     write.dc = Dc::ref();
 *     write.clock = brio::SpiClock::div8;       // under the write ceiling
 *     write.mode = brio::SpiMode::mode3;        // one the panel's traits take
 *     PanelHost::Request read = write;
 *     read.clock = brio::SpiClock::div32;       // under the read ceiling
 *
 *     PanelLink link{PanelLink::Config{write, read}};
 *     link.command(brio::Dcs::slpout, {});
 *     link.write(brio::Dcs::ramwr, std::span<const uint8_t>(pixels, n * 3)); // 3 bytes a pixel
 *
 * Who calls what, and from where. The link is a plain object with no
 * interrupt body: its verbs are called from the context that owns the
 * bus - main, a test, or an active object's dispatch (an active object
 * is a component with its own event queue, served one event at a time
 * by the kernel). Each verb hands a polled request (one that runs to
 * completion inside the call) straight to `Host::start()`, so it waits
 * for its whole transfer on the wire and returns when the transaction
 * is over: a large memory write called from a dispatch holds every
 * other active object for that long. No arbiter's queue is involved:
 * the link calls `Host::start()` itself, under the same engine contract
 * the arbiter relies on, so the program owns the bus while it uses the
 * link, and a program that also drives the same host through an SpiBus
 * arbiter must keep the two apart - nothing here checks it. A `true`
 * means the host ran the transaction and its status is spi_ok; SPI has
 * no acknowledge, so nothing is confirmed by the panel. What the host masks
 * while it runs a polled transfer is the host's business (its family's
 * document). Storage: the caller's span is lent to the engine for that
 * one transaction (`Lease::reply`, kernel/borrowed.hpp) and is the
 * caller's again on return; the command byte and the two requests are
 * members of the link, which is why the link is neither copyable nor
 * movable and a panel driver holds it by reference.
 *
 * What a link is, and what it is not. It carries a command byte and the
 * bytes that belong with it, in one tenure of whatever bus it owns (a
 * tenure is the time one transaction holds the bus), and that is the
 * whole of its job. It knows no command code, no pixel format, no
 * window and no dummy byte: a read hands back the bytes as the link
 * clocked them, and what a controller makes of them - a value a bit
 * late, a value one byte in, a line nobody drove - is the traits'
 * `read_framing` and the driver's to apply. That cut is what lets one
 * driver sit over a serial bus, an 8080 bus and a DSI host without a
 * branch.
 *
 * Why a write is not a command. `command` and `write` differ in nothing
 * on a link that clocks bytes, and the serial realization below
 * implements both with one function and says so. They are two verbs
 * because on other links they are two things: a parallel bus stores a
 * pixel as one word and a parameter as a byte, and DSI sends a long
 * packet for the one and a short packet for the other. A driver that
 * spelled them the same would have to be rewritten at the second link.
 *
 * The synchronous face is the whole of this file. Every verb completes
 * on return: `true` = done and accepted, `false` = refused or failed.
 * That is the direct surface shape of the command tier - a bring-up, a
 * suite, a program that owns its bus - and it is enough for the
 * memory-mapped shape too, whose pixels never travel this way. The
 * asynchronous face is born with the tiled pipeline and not before,
 * because its shape is decided by what a tile transfer needs and not by
 * what a link could offer: on this realization it is the same Request
 * with `polled` false and a real `ReplyTo<SpiDone>`, posted to a
 * `SpiBus` arbiter instead of handed to the engine, the tile on loan
 * until the reply lands - which is why the configuration below is a
 * pair of prototype requests and not a bag of fields.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include <concepts>
#include <span>

#include "kernel/borrowed.hpp"
#include "util/spi_bus.hpp"

namespace brio {

/**
 * What a panel driver is written over. Three verbs, all complete on
 * return; `true` = done and accepted, `false` = refused or failed.
 *
 *  - `command(c, parameters)` - a command with its parameter bytes, of
 *    which there may be none.
 *  - `write(c, bytes)` - a memory write: the command, then the pixel
 *    bytes. See the header comment for why this is not `command`.
 *  - `read(c, in)` - the command, then `in.size()` bytes clocked in,
 *    RAW: dummy clock or dummy byte included, exactly as the link saw
 *    them.
 */
template <typename L>
concept DcsLink = requires(L& l, uint8_t c, std::span<const uint8_t> out, std::span<uint8_t> in) {
    { l.command(c, out) } -> std::same_as<bool>;
    { l.write(c, out) } -> std::same_as<bool>;
    { l.read(c, in) } -> std::same_as<bool>;
};

/**
 * The number of a SPI mode - CPOL in bit 1, CPHA in bit 0, the
 * numbering every data sheet uses - read off the mode enum of any
 * stratum. Every family's `SpiMode` and the simulated host's
 * `SimSpiMode` spell the four as `mode0`..`mode3` (docs/design/spi-bus.md's
 * vocabulary), and the spelling is what this reads: no underlying value
 * is assumed, so a stratum whose enumerators are register codes answers
 * right too. It is what lets a controller's traits state the modes its
 * serial interface accepts as plain numbers, and a panel driver compare
 * a link's mode against them, with neither naming a family.
 */
template <typename Mode>
constexpr uint8_t dcs_spi_mode_number(Mode m) {
    if (m == Mode::mode1) {
        return 1u;
    }
    if (m == Mode::mode2) {
        return 2u;
    }
    if (m == Mode::mode3) {
        return 3u;
    }
    return 0u;
}

/**
 * The four-wire serial link, over any stratum's `SpiHost`.
 *
 * What makes it portable verbatim. The strata do not spell a bus
 * transaction the same way - the rate is an enum on most, a baud
 * divisor on the SAM, a prescaler pair on the PL022, and only some
 * Requests carry a frame size - so this class knows none of that. The
 * configuration is two prototype requests the application fills exactly
 * as it fills any request of that bus (the select and the D/C pin, the
 * setup time, the rate, the mode, the frame size where the stratum has
 * one). The link copies each once, at construction, and supplies in
 * that copy what every tenure of its own shares - the command byte lent
 * from a member of its own, `cmd_len` of one, a null reply, `polled` -
 * and a verb then fills in place only what its transaction brings: the
 * two spans and the length, three stores, and lends the member to
 * `Host::start()`. The arbiter's contract (util/bus_master.hpp) is what
 * makes the lend enough: the request is lent for the call, and a polled
 * request completes inside it, so nothing of the member is read after
 * the return. Whatever fields a family's Request adds travel through
 * untouched.
 *
 * Why two prototypes and not one. A controller's read ceiling is not
 * its write ceiling - a third of it on the ILI9481 - so a link that
 * read at writing speed would be wrong on the first panel. The pair
 * costs two request-sized members and no branch.
 *
 * The command byte is a member, and so the link does not move. A
 * Request lends its `cmd` buffer under `Lease::reply`, and on a polled
 * request the reply is inside `start()` - so the byte must outlive the
 * call and belong to this link, not to the function that returns before
 * the loan is over and not to a static two links would share. The two
 * prototypes carry that loan from construction on, which is a pointer
 * into this object: a copy of the link would lend the original's byte,
 * so copying and moving are deleted (a copy is a compile error). A panel
 * driver holds its link by reference.
 *
 * The frame must be a byte. `cmd_len` and `len` are counted in frames
 * by every engine that has a frame size, and this link counts bytes.
 * A prototype whose frame is wider is therefore refused at
 * construction: `valid()` goes false and every verb answers false
 * without touching the bus. Where the Request has no frame size the
 * frame is a byte by construction and there is nothing to check.
 *
 * The mode is the panel's to judge. The link reads nothing of the bus's
 * settings but exposes the mode each prototype carries, as the stratum
 * spells it, because which modes a controller's serial interface takes
 * is a fact of the controller (its traits) and the panel driver is the
 * layer that has both in hand. `dcs_spi_mode_number` above is how the
 * two meet without either naming a family's enum.
 */
template <typename Host>
class DcsSerialLink {
public:
    using Request = typename Host::Request;
    /// The mode as the stratum spells it: a family's `SpiMode`, the
    /// simulated host's `SimSpiMode`.
    using Mode = decltype(Request::mode);

    /// The two tenures this link ever runs, as the application would
    /// have written them by hand.
    struct Config {
        Request write;
        Request read;
    };

    explicit DcsSerialLink(const Config& config)
        : write_(config.write),
          read_(config.read),
          write_valid_(frame_is_a_byte(config.write)),
          read_valid_(frame_is_a_byte(config.read)),
          valid_(write_valid_ && read_valid_) {
        prepare(write_);
        prepare(read_);
    }

    DcsSerialLink(const DcsSerialLink&) = delete;
    DcsSerialLink& operator=(const DcsSerialLink&) = delete;

    /// True while both prototypes carry a byte-wide frame. A verb
    /// refuses while this is false.
    bool valid() const { return valid_; }
    /// Which of the two the constructor refused.
    bool write_prototype_valid() const { return write_valid_; }
    bool read_prototype_valid() const { return read_valid_; }

    /// The mode each tenure runs in, as the application set it in the
    /// prototype - for the panel driver to hold against its
    /// controller's traits.
    Mode write_mode() const { return write_.mode; }
    Mode read_mode() const { return read_.mode; }

    /// The engine's own code for the last request it was handed - what a
    /// caller reads after a `false`. A code other than spi_ok there is a
    /// failed transfer, or a request the engine refused without moving a
    /// byte: by the engine contract (util/bus_master.hpp) start() returns
    /// true for both and the engine's code says which. Only the link's
    /// own refusals - valid() false, a span longer than 65535 bytes - run
    /// no transaction and leave this code as the previous one left it, so
    /// a caller that must tell the two apart checks valid() and the
    /// length first and reads the code with that in mind.
    uint8_t status() const { return Host::status(); }

    /// A command with its parameter bytes (an empty span for none): one
    /// tenure of the write prototype, run to completion before the
    /// return. False when the link refuses - valid() false, a length the
    /// Request's 16-bit `len` cannot hold - or when the engine's status
    /// is not spi_ok, a failed transfer or an engine's refusal alike
    /// (status() says which).
    /// The span is lent for the call and the caller's again on return.
    bool command(uint8_t c, std::span<const uint8_t> parameters) {
        return send(write_, c, parameters.data(), nullptr, parameters.size());
    }

    /// The same tenure as `command()` on this link: a byte is a byte
    /// whether it is a parameter or a pixel. The verbs stay two because
    /// other links frame the two differently (the header comment).
    bool write(uint8_t c, std::span<const uint8_t> bytes) {
        return send(write_, c, bytes.data(), nullptr, bytes.size());
    }

    /// `in.size()` bytes clocked in behind the command, raw: dummy clock
    /// or dummy byte included, as the traits' `read_framing` describes.
    /// One tenure of the read prototype, complete on return, false as for
    /// command(). The engine sends 0xFF while they come.
    bool read(uint8_t c, std::span<uint8_t> in) {
        return send(read_, c, nullptr, in.data(), in.size());
    }

private:
    /// Does this Request type carry a frame size at all?
    static constexpr bool has_frame_size = requires(Request r) { r.bits; };

    static constexpr bool frame_is_a_byte(const Request& proto) {
        if constexpr (has_frame_size) {
            return proto.bits == decltype(proto.bits)::bits8;
        } else {
            (void)proto;
            return true;
        }
    }

    /// What every tenure of a prototype shares, written into it once.
    /// The spans and the length are the verb's; they start null and
    /// zero and are overwritten before every call.
    void prepare(Request& r) {
        r.cmd = lend<Lease::reply>(static_cast<const uint8_t*>(&command_));
        r.cmd_len = 1;
        r.tx = {};
        r.rx = {};
        r.len = 0;
        r.reply = {};
        r.polled = true;
    }

    bool send(Request& r, uint8_t c, const uint8_t* tx, uint8_t* rx, size_t n) {
        if (!valid_) {
            return false;
        }
        // A length the descriptor cannot hold is a refusal and never a
        // silent truncation. Where size_t is no wider than the field
        // there is nothing to compare.
        if constexpr (sizeof(size_t) > sizeof(uint16_t)) {
            if (n > 0xFFFFu) {
                return false;
            }
        }
        command_ = c;
        // Both spans every time, one of them null: a stale loan left in
        // the member would be written by the engine on the next tenure
        // of the other verb.
        r.tx = lend<Lease::reply>(tx);
        r.rx = lend<Lease::reply>(rx);
        r.len = static_cast<uint16_t>(n);
        // A polled request completes inside start() by the arbiter's
        // contract (util/bus_master.hpp): start() answers true, a request
        // the engine refuses included, and the status carries the
        // engine's code - and the member it was lent is not read after
        // the return. The false branch of start() is the asynchronous
        // one, and no host in the tree takes it for a polled request.
        return Host::start(r) && Host::status() == spi_ok;
    }

    Request write_;
    Request read_;
    uint8_t command_ = 0;
    bool write_valid_ = false;
    bool read_valid_ = false;
    bool valid_ = false;
};

}   // namespace brio
