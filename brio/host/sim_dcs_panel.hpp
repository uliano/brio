/*
 * sim_dcs_panel.hpp
 *
 * A COMMAND PANEL MADE OF RAM: the frame memory, the address counter
 * and the walk of a DCS controller, with no wire under it and no chip
 * over it. What it is for is the middle plane of design/gfx.md's three
 * planes of truth - a driver's window arithmetic, its walk bits, its
 * packing and its read framing judged pixel for pixel before any
 * silicon is asked, and judged again afterwards against the same
 * expectations.
 *
 * TWO LAYERS, AS THE COMMAND TIER SAYS. `SimDcsPanel<Traits>` is the
 * CORE, and its seam is the DCS TRANSACTION: a command, the parameter
 * or pixel bytes in, the answer bytes out. It knows nothing of dummy
 * bytes, of a D/C line or of a clock - those belong to a link, and
 * `SimDcsSerial<Core>` is the FRAMING ADAPTER for the four-wire serial
 * interface in front of it, at the level of bytes. A simulated SPI host
 * drives that, and so can a captured trace; a second link would be a
 * second adapter in front of the same core.
 *
 * THE BENCH IS THE AUTHORITY AND THIS FILE IS NOT. Every rule below is
 * a measurement on an ILI9481 module, restated from the traits type
 * that holds it (devices/ili9481.hpp) so that the driver and the
 * simulator cannot drift apart. Where nothing was measured the choice
 * is marked ASSUMPTION in the comment beside it, and each of those is a
 * question for the bench, not a decision this file is entitled to make:
 *
 *   1. A write past the window's last pixel is DROPPED (and counted).
 *   2. A read past the window's last pixel answers 0x00.
 *   3. A window whose start is beyond its axis is INVALID and nothing
 *      of a following write lands (the bench saw a single degenerate
 *      pixel, so a test asserts "at most one" and this model writes
 *      none); an end beyond the axis is clamped to it, and a range that
 *      runs backwards is invalid too.
 *   4. CASET and PASET keep the bytes they were given and the axes are
 *      resolved when the counter walks, so a MADCTL written after a
 *      window still decides which axis is which. The bench always wrote
 *      MADCTL first, so no measurement separates this from the other
 *      reading.
 *   5. A partial pixel left at the end of a memory write is carried
 *      into the next write memory continue.
 *   6. Setting a window puts both the write and the read position back
 *      to the window's start.
 *
 * WHAT IS NOT MODELLED: time, the glass's own refresh, the tearing
 * effect line, gamma, brightness, partial and idle modes, and every
 * command the bench did not exercise - those are COUNTED as unknown and
 * otherwise ignored, which is the honest answer for a model built from
 * measurements.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include <span>

#include "devices/dcs.hpp"

namespace brio {

/**
 * The MODULE's facts, which are a board's and not a controller's: how
 * the glass is mounted on the die and what it wants of the inversion.
 * They reach the simulator the way they reach a driver - handed over,
 * like the pins.
 */
struct SimDcsGlass {
    /// The glass shows memory column `columns - 1` at its LEFT.
    bool column_mirror = false;
    /// The panel needs INVON, so the colours it shows are the stored
    /// ones only while the inversion is on.
    bool wants_inversion = false;
};

/**
 * The core: frame memory, registers and the walk, behind one seam.
 *
 * The memory is the object's own - 320 x 480 x 3 bytes for an ILI9481 -
 * so a test declares one statically rather than on a stack.
 */
template <typename Traits>
class SimDcsPanel {
public:
    using Controller = Traits;

    static constexpr uint16_t columns = Traits::columns;
    static constexpr uint16_t pages = Traits::pages;
    static constexpr uint32_t pixel_count = static_cast<uint32_t>(columns) * pages;

    /// THE FRAME MEMORY, public on purpose: a test locates a pixel here
    /// the way the bench located one through a corner map, and a
    /// simulator that hid its memory would only be tested through the
    /// code under test. The two low bits of every byte are zero, because
    /// a write drops them exactly as the silicon does.
    uint8_t gram[pages][columns][3];

    explicit SimDcsPanel(SimDcsGlass glass = {}) : glass_(glass) { hard_reset(); }

    // ---- the seam -----------------------------------------------------------

    /**
     * One DCS transaction: `command`, its parameter or pixel bytes in
     * `in`, and the bytes the panel puts on the link in `out`. Every
     * byte of `out` the panel does not drive reads 0xFF, which is a
     * link's idle level and the only thing a pull-up can say.
     *
     * `commit` is what a FRAMING ADAPTER needs and nothing else: a
     * serial link does not know how many bytes will be clocked until
     * the select rises, so it asks for the answer first with
     * `commit = false` - a pure query, no counter moved and no pointer
     * advanced - serves the bytes as they are clocked, and calls again
     * at the close with an `out` sized to exactly what was clocked. The
     * answer is a function of the state alone, so the two calls agree.
     */
    void transaction(uint8_t command, std::span<const uint8_t> in, std::span<uint8_t> out,
                     bool commit = true) {
        for (uint8_t& b : out) {
            b = 0xFF;
        }
        answer(command, out);
        if (!commit) {
            return;
        }
        ++commands_;
        apply(command, in, out.size());
    }

    // ---- the reset lines ----------------------------------------------------

    /// The RESET pin: the registers to their reset values AND the frame
    /// memory to the alternate-page pattern the bench sees after one.
    void hard_reset() {
        reset_registers();
        for (uint16_t page = 0; page < pages; ++page) {
            const uint8_t fill = (page % 2u) == 0u ? Traits::reset_fill_even_page
                                                   : Traits::reset_fill_odd_page;
            for (uint16_t column = 0; column < columns; ++column) {
                gram[page][column][0] = fill;
                gram[page][column][1] = fill;
                gram[page][column][2] = fill;
            }
        }
    }

    // ---- what the glass shows -----------------------------------------------

    /**
     * The colour the GLASS shows at display column `d` (left to right)
     * and `line` (top to bottom), as 0x00RRGGBB.
     *
     * Three things stand between the memory and the eye and no more:
     * the module's own mirror (memory column `columns - 1 - d`), B0 of
     * MADCTL, which mirrors the page, and the inversion, which shows the
     * complement of every kept bit whenever the state does not match
     * what the module wants. B1 (horizontal flip) and B4 (line address
     * order) DO NOTHING on this module - measured - so they are kept in
     * the state and applied to nothing.
     */
    uint32_t glass(uint16_t d, uint16_t line) const {
        if (d >= columns || line >= pages) {
            return 0;
        }
        const uint16_t column = glass_.column_mirror ? static_cast<uint16_t>(columns - 1u - d) : d;
        const uint16_t page = (madctl_ & DcsAddressMode::vertical_flip) != 0u
                                  ? static_cast<uint16_t>(pages - 1u - line)
                                  : line;
        const uint32_t colour = stored(column, page);
        if (inverted_ == glass_.wants_inversion) {
            return colour;
        }
        return (~colour) & 0x00FCFCFCu;
    }

    /// The colour a memory cell holds, as 0x00RRGGBB.
    uint32_t stored(uint16_t column, uint16_t page) const {
        if (column >= columns || page >= pages) {
            return 0;
        }
        return (static_cast<uint32_t>(gram[page][column][0]) << 16) |
               (static_cast<uint32_t>(gram[page][column][1]) << 8) |
               gram[page][column][2];
    }

    // ---- what a test reads --------------------------------------------------

    bool sleeping() const { return sleeping_; }
    bool display_on() const { return display_on_; }
    bool inverted() const { return inverted_; }
    bool normal_mode() const { return normal_; }
    uint8_t madctl() const { return madctl_; }
    uint8_t colmod() const { return colmod_; }
    uint32_t read_pointer() const { return read_pos_; }

    uint32_t commands_taken() const { return commands_; }
    uint32_t pixels_written() const { return pixels_; }
    uint32_t bytes_read() const { return bytes_read_; }
    uint32_t unknown_commands() const { return unknown_; }
    uint32_t writes_dropped() const { return dropped_; }

    void reset_counters() {
        commands_ = 0;
        pixels_ = 0;
        bytes_read_ = 0;
        unknown_ = 0;
        dropped_ = 0;
    }

private:
    /// A cell of the frame memory.
    struct Cell {
        uint16_t column;
        uint16_t page;
    };

    /// The window resolved against the address mode in force: which axis
    /// each of CASET and PASET addresses, where the walk starts, how
    /// long its fast run is and how many pixels it covers.
    struct Walk {
        bool valid = false;
        bool exchange = false;
        bool column_reverse = false;
        bool page_reverse = false;
        uint16_t column_lo = 0;
        uint16_t column_hi = 0;
        uint16_t page_lo = 0;
        uint16_t page_hi = 0;
        uint16_t fast_len = 0;
        uint32_t count = 0;
    };

    Walk walk() const {
        Walk w;
        w.exchange = (madctl_ & DcsAddressMode::exchange) != 0u;
        w.column_reverse = (madctl_ & DcsAddressMode::column_order) != 0u;
        w.page_reverse = (madctl_ & DcsAddressMode::page_order) != 0u;

        // Under B5 the CASET pair addresses the pages and the PASET pair
        // the columns - measured, and the whole of what B5 does to an
        // address.
        const uint16_t column_start = w.exchange ? paset_start_ : caset_start_;
        const uint16_t column_end = w.exchange ? paset_end_ : caset_end_;
        const uint16_t page_start = w.exchange ? caset_start_ : paset_start_;
        const uint16_t page_end = w.exchange ? caset_end_ : paset_end_;

        if (column_start >= columns || page_start >= pages) {
            return w;   // ASSUMPTION 3: a start beyond its axis is no window at all
        }
        w.column_lo = column_start;
        w.column_hi = column_end >= columns ? static_cast<uint16_t>(columns - 1u) : column_end;
        w.page_lo = page_start;
        w.page_hi = page_end >= pages ? static_cast<uint16_t>(pages - 1u) : page_end;
        if (w.column_hi < w.column_lo || w.page_hi < w.page_lo) {
            return w;   // ASSUMPTION 3: a range that runs backwards is no window either
        }
        const uint16_t across = static_cast<uint16_t>(w.column_hi - w.column_lo + 1u);
        const uint16_t down = static_cast<uint16_t>(w.page_hi - w.page_lo + 1u);
        w.fast_len = w.exchange ? down : across;
        w.count = static_cast<uint32_t>(across) * down;
        w.valid = true;
        return w;
    }

    /// The `n`-th pixel of the walk. The fast axis is the columns, or
    /// the pages under B5; B6 reverses the column direction and B7 the
    /// page direction, whichever of the two each has become.
    static Cell locate(const Walk& w, uint32_t n) {
        const uint16_t fast = static_cast<uint16_t>(n % w.fast_len);
        const uint16_t slow = static_cast<uint16_t>(n / w.fast_len);
        const uint16_t column_index = w.exchange ? slow : fast;
        const uint16_t page_index = w.exchange ? fast : slow;
        return {w.column_reverse ? static_cast<uint16_t>(w.column_hi - column_index)
                                 : static_cast<uint16_t>(w.column_lo + column_index),
                w.page_reverse ? static_cast<uint16_t>(w.page_hi - page_index)
                               : static_cast<uint16_t>(w.page_lo + page_index)};
    }

    // ---- the answer, which moves nothing ------------------------------------

    void answer(uint8_t command, std::span<uint8_t> out) const {
        switch (command) {
            case Dcs::rddpm:
                put(out, power_mode());
                break;
            case Dcs::rddmadctl:
                put(out, madctl_);
                break;
            case Dcs::rddcolmod:
                put(out, colmod_);
                break;
            case Dcs::rddid:
            case Dcs::rddst:
                // All ones on this module: the fill already says it.
                break;
            case Traits::device_code_command:
                for (size_t i = 0; i < out.size() && i < Traits::device_code.size(); ++i) {
                    out[i] = Traits::device_code[i];
                }
                break;
            case Dcs::ramrd:
                read_memory(0, out);
                break;
            case Dcs::ramrd_continue:
                read_memory(read_pos_ / Traits::bytes_per_pixel, out);
                break;
            default:
                break;
        }
    }

    static void put(std::span<uint8_t> out, uint8_t value) {
        if (!out.empty()) {
            out[0] = value;
        }
    }

    /// RDDPM: bit 4 sleep out, bit 3 normal mode, bit 2 display on, and
    /// zeros elsewhere - 0x1C after SLPOUT and DISPON, as the bench read
    /// it. D7 is the table's "set to 0", whatever the prose beside it
    /// says about the booster.
    uint8_t power_mode() const {
        uint8_t v = 0;
        if (!sleeping_) {
            v = static_cast<uint8_t>(v | 0x10u);
        }
        if (normal_) {
            v = static_cast<uint8_t>(v | 0x08u);
        }
        if (display_on_) {
            v = static_cast<uint8_t>(v | 0x04u);
        }
        return v;
    }

    /// Three bytes a pixel along the walk, starting at pixel
    /// `first_pixel` and at THAT PIXEL'S FIRST BYTE.
    void read_memory(uint32_t first_pixel, std::span<uint8_t> out) const {
        const Walk w = walk();
        for (size_t i = 0; i < out.size(); ++i) {
            const uint32_t n = first_pixel + static_cast<uint32_t>(i / Traits::bytes_per_pixel);
            const size_t byte = i % Traits::bytes_per_pixel;
            if (!w.valid || n >= w.count) {
                out[i] = 0x00;   // ASSUMPTION 2
                continue;
            }
            const Cell c = locate(w, n);
            out[i] = gram[c.page][c.column][byte];
        }
    }

    // ---- the state a command moves ------------------------------------------

    void apply(uint8_t command, std::span<const uint8_t> in, size_t clocked) {
        switch (command) {
            case Dcs::nop:
                break;
            case Dcs::swreset:
                reset_registers();
                break;
            case Dcs::caset:
                if (in.size() >= 4) {
                    const DcsWindowRange r = dcs_window_range(in.first<4>());
                    caset_start_ = r.start;
                    caset_end_ = r.end;
                    rewind();
                }
                break;
            case Dcs::paset:
                if (in.size() >= 4) {
                    const DcsWindowRange r = dcs_window_range(in.first<4>());
                    paset_start_ = r.start;
                    paset_end_ = r.end;
                    rewind();
                }
                break;
            case Dcs::ramwr:
                write_pos_ = 0;
                pending_ = 0;
                write_memory(in);
                break;
            case Dcs::ramwr_continue:
                write_memory(in);
                break;
            case Dcs::ramrd:
                read_pos_ = static_cast<uint32_t>(clocked);
                bytes_read_ += static_cast<uint32_t>(clocked);
                break;
            case Dcs::ramrd_continue:
                read_pos_ += static_cast<uint32_t>(clocked);
                bytes_read_ += static_cast<uint32_t>(clocked);
                break;
            case Dcs::madctl:
                if (!in.empty()) {
                    madctl_ = in[0];
                }
                break;
            case Dcs::colmod:
                if (!in.empty()) {
                    colmod_ = in[0];
                }
                break;
            case Dcs::slpin:
                sleeping_ = true;
                break;
            case Dcs::slpout:
                sleeping_ = false;
                break;
            case Dcs::dispon:
                display_on_ = true;
                break;
            case Dcs::dispoff:
                display_on_ = false;
                break;
            case Dcs::invon:
                inverted_ = true;
                break;
            case Dcs::invoff:
                inverted_ = false;
                break;
            case Dcs::noron:
                normal_ = true;
                break;
            case Dcs::rddpm:
            case Dcs::rddmadctl:
            case Dcs::rddcolmod:
            case Dcs::rddid:
            case Dcs::rddst:
            case Traits::device_code_command:
                break;
            default:
                ++unknown_;
                break;
        }
    }

    /// Pixel bytes into the memory along the walk. The BGR bit is
    /// applied ON THE WAY IN and the two low bits of every byte are
    /// dropped, both because that is what the silicon does and because
    /// it leaves the memory holding exactly what a read gives back.
    void write_memory(std::span<const uint8_t> in) {
        const Walk w = walk();
        for (uint8_t byte : in) {
            if (!w.valid || write_pos_ >= w.count) {
                ++dropped_;   // ASSUMPTION 1
                continue;
            }
            assembling_[pending_] = static_cast<uint8_t>(byte & 0xFCu);
            ++pending_;
            if (pending_ < Traits::bytes_per_pixel) {
                continue;   // ASSUMPTION 5: a partial pixel waits for the next write
            }
            pending_ = 0;
            if ((madctl_ & DcsAddressMode::bgr) != 0u) {
                const uint8_t swap = assembling_[0];
                assembling_[0] = assembling_[2];
                assembling_[2] = swap;
            }
            const Cell c = locate(w, write_pos_);
            gram[c.page][c.column][0] = assembling_[0];
            gram[c.page][c.column][1] = assembling_[1];
            gram[c.page][c.column][2] = assembling_[2];
            ++write_pos_;
            ++pixels_;
        }
    }

    /// ASSUMPTION 6: a new window is a new walk.
    void rewind() {
        write_pos_ = 0;
        read_pos_ = 0;
        pending_ = 0;
    }

    /// What SWRESET puts back, and what a hardware reset puts back
    /// before it touches the memory. The window comes up as the whole
    /// frame, which is what makes a RAMWR with no window a full-screen
    /// write.
    void reset_registers() {
        sleeping_ = true;
        display_on_ = false;
        inverted_ = false;
        normal_ = true;
        madctl_ = Traits::reset_madctl;
        colmod_ = Traits::reset_colmod;
        caset_start_ = 0;
        caset_end_ = static_cast<uint16_t>(columns - 1u);
        paset_start_ = 0;
        paset_end_ = static_cast<uint16_t>(pages - 1u);
        rewind();
    }

    SimDcsGlass glass_;

    bool sleeping_ = true;
    bool display_on_ = false;
    bool inverted_ = false;
    bool normal_ = true;
    uint8_t madctl_ = Traits::reset_madctl;
    uint8_t colmod_ = Traits::reset_colmod;

    uint16_t caset_start_ = 0;
    uint16_t caset_end_ = 0;
    uint16_t paset_start_ = 0;
    uint16_t paset_end_ = 0;

    uint32_t write_pos_ = 0;    ///< the next pixel of the walk a write fills
    uint32_t read_pos_ = 0;     ///< BYTES clocked out of the window since it was opened
    uint8_t pending_ = 0;       ///< bytes of a pixel already taken
    uint8_t assembling_[3] = {0, 0, 0};

    uint32_t commands_ = 0;
    uint32_t pixels_ = 0;
    uint32_t bytes_read_ = 0;
    uint32_t unknown_ = 0;
    uint32_t dropped_ = 0;
};

/**
 * THE FOUR-WIRE SERIAL INTERFACE, at the level of bytes: the framing
 * adapter in front of a core. Three verbs, which is what the wires are
 * - `select(true)` pulls the chip select low, `dc(false)` marks a
 * command byte, `byte(mosi)` clocks eight bits in and returns the eight
 * MISO carried for them.
 *
 * HOW A TRANSACTION IS ASSEMBLED. With the select low, a byte with D/C
 * low is a COMMAND and closes whatever command was standing (the panel
 * does that too: a command ends when the host sends another one); bytes
 * with D/C high are that command's parameters, its pixel data or the
 * clocks of its answer; the select rising closes the transaction. A
 * write is buffered whole and handed to the core at the close, so the
 * core sees one transaction and not a stream of bytes.
 *
 * HOW A READ IS SERVED. The framing is the controller's fact, taken
 * from its traits: a memory read begins with one dummy BYTE, a
 * multi-parameter register read is shifted right by one BIT across the
 * whole stream, a single-parameter one is aligned, and a command the
 * panel does not drive leaves the line at 0xFF. The bytes must be
 * served as they are clocked, before the adapter can know how many
 * there will be, so the answer is fetched once as a query that moves
 * nothing and the core is told the true count at the close - which is
 * what keeps the read pointer's rule exact.
 *
 * The buffer is fixed, as a chip's would be: bytes past it are dropped
 * and counted.
 */
template <typename Core, uint16_t Capacity = 1024>
class SimDcsSerial {
public:
    using Controller = typename Core::Controller;

    static constexpr uint8_t idle_level = 0xFF;   ///< the pull-up on a line nobody drives

    explicit SimDcsSerial(Core& core) : core_(core) {}

    /// The chip select: true asserts it (the wire goes LOW).
    void select(bool low) {
        if (low == selected_) {
            return;
        }
        selected_ = low;
        if (low) {
            open_ = false;
            in_len_ = 0;
            clocked_ = 0;
        } else {
            close();
        }
    }

    /// The D/C line: true marks data, false a command byte.
    void dc(bool high) { dc_ = high; }

    /// One byte clocked in; what MISO carried while it went.
    uint8_t byte(uint8_t mosi) {
        if (!selected_) {
            return idle_level;
        }
        if (!dc_) {
            close();
            command_ = mosi;
            framing_ = Controller::read_framing(mosi);
            open_ = true;
            in_len_ = 0;
            clocked_ = 0;
            prefetched_ = false;
            return idle_level;
        }
        if (!open_) {
            return idle_level;   // data with no command in front of it
        }
        const uint16_t k = clocked_;
        if (clocked_ < 0xFFFFu) {
            ++clocked_;
        }
        if (framing_ == DcsReadFraming::undriven) {
            if (in_len_ < Capacity) {
                in_[in_len_] = mosi;
                ++in_len_;
            } else {
                ++overruns_;
            }
            return idle_level;
        }
        if (!prefetched_) {
            core_.transaction(command_, {}, std::span<uint8_t>(answer_, Capacity), false);
            prefetched_ = true;
        }
        return served(k);
    }

    /// Bytes a transaction could not hold - its data past the buffer, or
    /// an answer clocked past it.
    uint32_t overruns() const { return overruns_; }
    /// Transactions handed to the core.
    uint32_t transactions() const { return transactions_; }
    void reset_counters() {
        overruns_ = 0;
        transactions_ = 0;
    }

private:
    uint8_t served(uint16_t k) const {
        switch (framing_) {
            case DcsReadFraming::aligned:
                return k < Capacity ? answer_[k] : idle_level;
            case DcsReadFraming::dummy_byte:
                if (k == 0u) {
                    return Controller::memory_read_dummy_byte;
                }
                return (k - 1u) < Capacity ? answer_[k - 1u] : idle_level;
            case DcsReadFraming::dummy_clock: {
                // One clock of dummy in front of the value's first bit,
                // driven LOW: the whole stream is a bit late, which is
                // how 02 04 94 81 FF reaches a host as 01 02 4A 40 FF.
                const uint8_t previous = k == 0u ? 0x00u : value_byte(static_cast<uint16_t>(k - 1u));
                const uint8_t current = value_byte(k);
                return static_cast<uint8_t>(((previous & 0x01u) << 7) | (current >> 1));
            }
            default:
                return idle_level;
        }
    }

    uint8_t value_byte(uint16_t k) const { return k < Capacity ? answer_[k] : idle_level; }

    /// How many VALUE bytes the clocks so far asked the panel for - the
    /// count the core must see, so that its read pointer advances by the
    /// bytes actually clocked and by nothing else.
    uint16_t value_bytes() const {
        uint16_t n = 0;
        switch (framing_) {
            case DcsReadFraming::aligned:
            case DcsReadFraming::dummy_clock:
                n = clocked_;
                break;
            case DcsReadFraming::dummy_byte:
                n = clocked_ > 0u ? static_cast<uint16_t>(clocked_ - 1u) : 0u;
                break;
            default:
                return 0;
        }
        return n <= Capacity ? n : Capacity;
    }

    void close() {
        if (!open_) {
            return;
        }
        open_ = false;
        const uint16_t n = value_bytes();
        ++transactions_;
        core_.transaction(command_, std::span<const uint8_t>(in_, in_len_),
                          std::span<uint8_t>(answer_, n));
    }

    Core& core_;
    bool selected_ = false;
    bool dc_ = false;
    bool open_ = false;
    bool prefetched_ = false;
    uint8_t command_ = 0;
    DcsReadFraming framing_ = DcsReadFraming::undriven;
    uint16_t in_len_ = 0;
    uint16_t clocked_ = 0;
    uint32_t overruns_ = 0;
    uint32_t transactions_ = 0;
    uint8_t in_[Capacity] = {};
    uint8_t answer_[Capacity] = {};
};

}   // namespace brio
