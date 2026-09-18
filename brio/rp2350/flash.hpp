/*
 * flash.hpp
 *
 * THE EXTERNAL QUAD-SPI CHIP THE CORE EXECUTES OUT OF, and the two
 * blocks between it and the bus: the QMI (datasheet 12.14), which turns
 * a bus access into a serial transfer, and the XIP cache with its four
 * address aliases (4.4). Three things live here:
 *
 *  - `Qmi` and `QmiWindow<0|1>`, THE RESOURCE over 12.14. The QMI is
 *    what replaces the RP2040's SSI, and it is not that peripheral
 *    renamed: it has TWO MEMORY WINDOWS of 16 MB, one per chip select,
 *    each with its own timing, its own read and write transfer FORMAT
 *    (prefix / address / suffix / dummy / data, every phase with its own
 *    bus width), its own command constants, and FOUR ADDRESS TRANSLATION
 *    panes that map a virtual 4 MB slice onto any 4 kB-aligned physical
 *    base. Beside the windows sits DIRECT MODE, a plain SPI FIFO pair
 *    that disconnects the memory windows for the duration - which is
 *    what a flash command needs and what makes a window of the operation
 *    below.
 *
 *  - `Xip`, THE CACHE (4.4.1): 16 kB, two-way, eight-byte lines. Where
 *    the RP2040 had one FLUSH register this chip has a MAINTENANCE
 *    ADDRESS WINDOW - one write per line, four operations and a fifth
 *    that PINS a line so it can be used as SRAM - and where the RP2040
 *    had one enable this one has a pair (Secure and Non-secure) beside
 *    five more access controls. The two performance counters are the
 *    RP2040's. Erratum RP2350-E11 is answered in `clean_all()`: a clean
 *    by set/way rewrites the line's tag with the maintenance address, so
 *    the sweep is driven from an address whose tag falls OUTSIDE the
 *    QMI's half of the downstream space.
 *
 *  - `Flash`, THE ENGINE, over the bootrom's own flash functions (5.4):
 *    `erase`, `program` and a raw READ command, each a WINDOW with the
 *    memory interface disconnected. The functions are found by two-
 *    character code through rp2350/bootrom.hpp, which does the right
 *    lookup on either architecture, so this file names no address and
 *    asks the preprocessor nothing.
 *
 * THERE IS NO SECOND STAGE HERE. On the RP2040 the way back into
 * execute-in-place after a flash operation was the image's own first 256
 * bytes, called as a function. This chip has no such stage: the bootrom
 * sets the interface up itself while scanning the flash, and leaves a
 * position-independent XIP SETUP FUNCTION in the first 256 bytes of boot
 * RAM (4.3, 5.2.7) that restores the mode and divisor it found. Boot RAM
 * is on the APB and is never executable, so `init()` copies those 256
 * bytes into SRAM once and the window calls the copy.
 *
 * AND THE WAY BACK IS NOT LOAD-BEARING FOR CORRECTNESS, which is the
 * other half of the difference. 5.4.8.6 states it plainly: the ROM's
 * `flash_exit_xip()` leaves a basic 03h serial read mode set up as it
 * goes, and the program and erase functions do not leave XIP
 * inaccessible - so after a window the flash is READABLE whatever
 * happens, and the setup function only restores the FAST mode. This file
 * therefore has two ways back and says which it used
 * (`xip_restore_kind()`): the copied setup function when it validated,
 * and the ROM's own `flash_enter_cmd_xip()` when it did not.
 *
 * THE WINDOW, AND WHAT IT ASKS OF THE APPLICATION. While the QMI is in
 * direct mode the memory windows are gone: an access to 0x1000_0000
 * returns a bus error (12.14.5, and 5.4.8.10 says the same of the
 * debugger and the other core). So every operation here runs from
 * `.ram_text` - an input section the linker script folds into .data, so
 * the crt copies it to SRAM with the initialized data - with INTERRUPTS
 * MASKED ON THIS CORE for its whole duration, a handler fetched from
 * flash being exactly the fault. Nothing here can enforce the rest, so
 * it is stated: THE OTHER CORE MUST NOT FETCH OR READ FLASH while a
 * window is open (park it in SRAM or leave it where the bootrom did),
 * and no DMA channel may have the window as a source. A buffer of the
 * caller's that lies in the window is refused (`in_window`).
 *
 * WHAT IS SAVED ACROSS A WINDOW. The ROM's `connect_internal_flash()`
 * puts every QSPI pad control back to its RESET state, and its
 * `flash_exit_xip()` rewrites the second window's timing, read format
 * and read command as well as the first's. Neither is this driver's to
 * lose: the six QSPI pad registers are snapshotted and written back, and
 * so is window 1's trio - but only when the runtime FLASH_DEVINFO says
 * no device is attached to chip select 1, which is the only case this
 * board can be. A program with a PSRAM on CS1 wants its own setup
 * function and does not have one here.
 *
 * THE RAW COMMAND VERB READS AND NOTHING ELSE. `command()` accepts an
 * ALLOW-LIST of opcodes (`FlashCommand`, all of them reads), refused by
 * static_assert where the opcode is a template argument and by a false
 * return where it is a value. No write enable, no status or
 * configuration register write, no erase or program opcode can travel
 * through it: those registers are one-way on many chips - a quad-enable
 * or a lock bit set by accident is a board that no longer boots - and
 * the erase and program paths that this file does offer go through the
 * bootrom's own bounds-checked functions instead.
 *
 * ADDRESSES ARE OFFSETS IN THE CHIP, from 0 - what the bootrom's
 * functions take - and the aliases add their base where a pointer is
 * wanted. The image begins at offset 0; an erase that reaches it is the
 * application's to refuse, and rp2350/nvm_flash.hpp refuses it by its
 * bounds.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <array>
#include <optional>
#include <span>

#include "rp2350/bootrom.hpp"
#include "rp2350/core.hpp"
#include "rp2350/device.hpp"

#ifndef BRIO_RP2350_FLASH_KB
#error "BRIO_RP2350_FLASH_KB names the board's flash size in KB (rp2350/CMakeLists.txt passes it)"
#endif

namespace brio {

// =============================================================================
// The address map of the XIP subsystem (4.4.1)
// =============================================================================
//
// One 26-bit downstream space, mirrored four times in the system map and
// decoded on bits 27:26. The QMI occupies its LOWER HALF: two 16 MB
// windows, one per chip select; the upper half is reserved, and is where
// a pinned cache line can live without ever aliasing a real address.

inline constexpr uint32_t xip_base = 0x1000'0000u;             ///< cached, translated
inline constexpr uint32_t xip_nocache_base = 0x1400'0000u;     ///< uncached, translated
inline constexpr uint32_t xip_maintenance_base = 0x1800'0000u; ///< cache maintenance writes
inline constexpr uint32_t xip_untranslated_base = 0x1c00'0000u;///< uncached, and no translation
inline constexpr uint32_t xip_alias_span = 0x0400'0000u;       ///< 64 MB, one alias
inline constexpr uint32_t xip_space_span = 0x1000'0000u;       ///< the four aliases together
inline constexpr uint32_t xip_window_span = 0x0100'0000u;      ///< 16 MB, one chip select
inline constexpr uint32_t xip_pane_span = 0x0040'0000u;        ///< 4 MB, one translation pane

/// Boot RAM (4.3): 1 kB on the APB, the bootrom's own, and never
/// executable. Its first 256 bytes hold the XIP setup function.
inline constexpr uint32_t bootram_base = BOOTRAM_BASE;
inline constexpr uint32_t bootram_bytes = 1024;

// =============================================================================
// The QMI's vocabulary (12.14.2, 12.14.3, 12.14.4)
// =============================================================================

/// How many lines a phase of a transfer uses. The QMI names the same
/// three everywhere: the prefix, the address, the suffix, the dummy
/// cycles and the data each carry one of these.
enum class QmiWidth : uint8_t {
    serial = 0,   ///< one line out (SD0) and one in (SD1)
    dual = 1,     ///< two lines, bidirectional
    quad = 2,     ///< four lines, bidirectional
};

/// One record pushed into DIRECT_TX (12.14.5.1). The control bits sit
/// above the data, and all-zero means "eight serial bits, answer kept",
/// which is why a plain byte works as a FIFO write.
struct QmiDirectFrame {
    uint16_t data = 0;                        ///< 8 or 16 bits, by `sixteen_bit`
    QmiWidth width = QmiWidth::serial;        ///< IWIDTH
    bool sixteen_bit = false;                 ///< DWIDTH: 16 data bits instead of 8
    bool output_enable = false;               ///< OE: drive the pads (dual/quad only)
    bool no_push = false;                     ///< NOPUSH: throw this record's answer away
};

/// The DIRECT_TX word a frame is.
constexpr uint32_t qmi_direct_word(const QmiDirectFrame& f) {
    return static_cast<uint32_t>(f.data) |
           (static_cast<uint32_t>(f.width) << QMI_DIRECT_TX_IWIDTH_LSB) |
           (f.sixteen_bit ? QMI_DIRECT_TX_DWIDTH_BITS : 0u) |
           (f.output_enable ? QMI_DIRECT_TX_OE_BITS : 0u) |
           (f.no_push ? QMI_DIRECT_TX_NOPUSH_BITS : 0u);
}

/// Where a burst is broken whatever the address sequence says, because
/// some devices forbid crossing the boundary in one transfer.
enum class QmiPageBreak : uint8_t {
    none = QMI_M0_TIMING_PAGEBREAK_VALUE_NONE,
    at_256 = QMI_M0_TIMING_PAGEBREAK_VALUE_256,
    at_1024 = QMI_M0_TIMING_PAGEBREAK_VALUE_1024,
    at_4096 = QMI_M0_TIMING_PAGEBREAK_VALUE_4096,
};

/// One window's M*_TIMING, decoded. Every field is the register's own
/// unit: `clkdiv` divides clk_sys and encodes 256 as zero, `cooldown`
/// and `max_select` count 64 system clocks, the two select times count
/// system clocks, `min_deselect` counts them too.
struct QmiTiming {
    uint8_t clkdiv = 4;                             ///< 1..255 direct, 0 means 256
    uint8_t rxdelay = 0;                            ///< sample delay, half system clocks
    uint8_t min_deselect = 0;                       ///< 5 bits
    uint8_t max_select = 0;                         ///< 6 bits, units of 64 clocks; 0 = unlimited
    uint8_t select_hold = 0;                        ///< 2 bits, extra system clocks
    uint8_t select_setup = 0;                       ///< 1 bit, one extra system clock
    QmiPageBreak page_break = QmiPageBreak::none;
    uint8_t cooldown = 1;                           ///< 2 bits, units of 64 clocks; 0 = no cooldown
};

constexpr uint32_t qmi_timing_word(const QmiTiming& t) {
    return (static_cast<uint32_t>(t.cooldown & 0x3u) << QMI_M0_TIMING_COOLDOWN_LSB) |
           (static_cast<uint32_t>(t.page_break) << QMI_M0_TIMING_PAGEBREAK_LSB) |
           (static_cast<uint32_t>(t.select_setup & 0x1u) << QMI_M0_TIMING_SELECT_SETUP_LSB) |
           (static_cast<uint32_t>(t.select_hold & 0x3u) << QMI_M0_TIMING_SELECT_HOLD_LSB) |
           (static_cast<uint32_t>(t.max_select & 0x3Fu) << QMI_M0_TIMING_MAX_SELECT_LSB) |
           (static_cast<uint32_t>(t.min_deselect & 0x1Fu) << QMI_M0_TIMING_MIN_DESELECT_LSB) |
           (static_cast<uint32_t>(t.rxdelay & 0x7u) << QMI_M0_TIMING_RXDELAY_LSB) |
           static_cast<uint32_t>(t.clkdiv);
}

constexpr QmiTiming qmi_timing_from(uint32_t w) {
    return QmiTiming{
        .clkdiv = static_cast<uint8_t>(w & QMI_M0_TIMING_CLKDIV_BITS),
        .rxdelay = static_cast<uint8_t>((w & QMI_M0_TIMING_RXDELAY_BITS) >> QMI_M0_TIMING_RXDELAY_LSB),
        .min_deselect =
            static_cast<uint8_t>((w & QMI_M0_TIMING_MIN_DESELECT_BITS) >> QMI_M0_TIMING_MIN_DESELECT_LSB),
        .max_select =
            static_cast<uint8_t>((w & QMI_M0_TIMING_MAX_SELECT_BITS) >> QMI_M0_TIMING_MAX_SELECT_LSB),
        .select_hold =
            static_cast<uint8_t>((w & QMI_M0_TIMING_SELECT_HOLD_BITS) >> QMI_M0_TIMING_SELECT_HOLD_LSB),
        .select_setup =
            static_cast<uint8_t>((w & QMI_M0_TIMING_SELECT_SETUP_BITS) >> QMI_M0_TIMING_SELECT_SETUP_LSB),
        .page_break = static_cast<QmiPageBreak>((w & QMI_M0_TIMING_PAGEBREAK_BITS) >>
                                                QMI_M0_TIMING_PAGEBREAK_LSB),
        .cooldown = static_cast<uint8_t>((w & QMI_M0_TIMING_COOLDOWN_BITS) >> QMI_M0_TIMING_COOLDOWN_LSB),
    };
}

/// The clock a window's transfers run at: clk_sys over CLKDIV, with zero
/// meaning 256 (12.14.3). `clk_sys_hz` is the caller's, because this
/// file knows no clock.
constexpr uint32_t qmi_sck_hz(uint32_t clk_sys_hz, uint8_t clkdiv) {
    return clk_sys_hz / (clkdiv == 0u ? 256u : static_cast<uint32_t>(clkdiv));
}

/// A phase's length where the register encodes a choice and not a count.
enum class QmiPrefixLen : uint8_t {
    none = QMI_M0_RFMT_PREFIX_LEN_VALUE_NONE,
    eight = QMI_M0_RFMT_PREFIX_LEN_VALUE_8,
};
enum class QmiSuffixLen : uint8_t {
    none = QMI_M0_RFMT_SUFFIX_LEN_VALUE_NONE,
    eight = QMI_M0_RFMT_SUFFIX_LEN_VALUE_8,
};

/// One window's read or write FORMAT (M*_RFMT / M*_WFMT): the five
/// phases of 12.14.2, each with its width, and the two that have a
/// length. `dummy_len` is in units of four bits, 0..7, so 28 dummy bits
/// is the most this block will insert.
struct QmiTransferFormat {
    QmiWidth prefix_width = QmiWidth::serial;
    QmiWidth addr_width = QmiWidth::serial;
    QmiWidth suffix_width = QmiWidth::serial;
    QmiWidth dummy_width = QmiWidth::serial;
    QmiWidth data_width = QmiWidth::serial;
    QmiPrefixLen prefix_len = QmiPrefixLen::eight;
    QmiSuffixLen suffix_len = QmiSuffixLen::none;
    uint8_t dummy_len = 0;                    ///< 0..7, in units of four bits
    bool dtr = false;                         ///< both clock edges carry data (12.14.3.3)
};

constexpr uint32_t qmi_format_word(const QmiTransferFormat& f) {
    return (f.dtr ? QMI_M0_RFMT_DTR_BITS : 0u) |
           (static_cast<uint32_t>(f.dummy_len & 0x7u) << QMI_M0_RFMT_DUMMY_LEN_LSB) |
           (static_cast<uint32_t>(f.suffix_len) << QMI_M0_RFMT_SUFFIX_LEN_LSB) |
           (static_cast<uint32_t>(f.prefix_len) << QMI_M0_RFMT_PREFIX_LEN_LSB) |
           (static_cast<uint32_t>(f.data_width) << QMI_M0_RFMT_DATA_WIDTH_LSB) |
           (static_cast<uint32_t>(f.dummy_width) << QMI_M0_RFMT_DUMMY_WIDTH_LSB) |
           (static_cast<uint32_t>(f.suffix_width) << QMI_M0_RFMT_SUFFIX_WIDTH_LSB) |
           (static_cast<uint32_t>(f.addr_width) << QMI_M0_RFMT_ADDR_WIDTH_LSB) |
           static_cast<uint32_t>(f.prefix_width);
}

constexpr QmiTransferFormat qmi_format_from(uint32_t w) {
    return QmiTransferFormat{
        .prefix_width = static_cast<QmiWidth>(w & QMI_M0_RFMT_PREFIX_WIDTH_BITS),
        .addr_width =
            static_cast<QmiWidth>((w & QMI_M0_RFMT_ADDR_WIDTH_BITS) >> QMI_M0_RFMT_ADDR_WIDTH_LSB),
        .suffix_width =
            static_cast<QmiWidth>((w & QMI_M0_RFMT_SUFFIX_WIDTH_BITS) >> QMI_M0_RFMT_SUFFIX_WIDTH_LSB),
        .dummy_width =
            static_cast<QmiWidth>((w & QMI_M0_RFMT_DUMMY_WIDTH_BITS) >> QMI_M0_RFMT_DUMMY_WIDTH_LSB),
        .data_width =
            static_cast<QmiWidth>((w & QMI_M0_RFMT_DATA_WIDTH_BITS) >> QMI_M0_RFMT_DATA_WIDTH_LSB),
        .prefix_len =
            static_cast<QmiPrefixLen>((w & QMI_M0_RFMT_PREFIX_LEN_BITS) >> QMI_M0_RFMT_PREFIX_LEN_LSB),
        .suffix_len =
            static_cast<QmiSuffixLen>((w & QMI_M0_RFMT_SUFFIX_LEN_BITS) >> QMI_M0_RFMT_SUFFIX_LEN_LSB),
        .dummy_len = static_cast<uint8_t>((w & QMI_M0_RFMT_DUMMY_LEN_BITS) >> QMI_M0_RFMT_DUMMY_LEN_LSB),
        .dtr = (w & QMI_M0_RFMT_DTR_BITS) != 0u,
    };
}

/// The six pads the QMI drives, in the order the QSPI pad bank lists
/// them. They are not bank 0's: no `Pin` reaches them, and the only
/// program that ought to write one is a board file that knows the flash
/// chip's timing.
enum class QspiPad : uint8_t { sclk = 0, sd0, sd1, sd2, sd3, select };

/// The two constants a window sends around the address (M*_RCMD /
/// M*_WCMD): the PREFIX before it and the SUFFIX after it.
struct QmiCommand {
    uint8_t prefix = 0x03;
    uint8_t suffix = 0x00;
};

constexpr uint32_t qmi_command_word(const QmiCommand& c) {
    return static_cast<uint32_t>(c.prefix) |
           (static_cast<uint32_t>(c.suffix) << QMI_M0_RCMD_SUFFIX_LSB);
}

constexpr QmiCommand qmi_command_from(uint32_t w) {
    return QmiCommand{
        .prefix = static_cast<uint8_t>(w & QMI_M0_RCMD_PREFIX_BITS),
        .suffix = static_cast<uint8_t>((w & QMI_M0_RCMD_SUFFIX_BITS) >> QMI_M0_RCMD_SUFFIX_LSB),
    };
}

/// One ATRANS pane (12.14.4), in BYTES. The registers count 4 kB units;
/// this type does not, because every other address in this file is a
/// byte address and two units in one header is how a sector gets erased
/// at the wrong place.
struct QmiTranslation {
    uint32_t base = 0;    ///< the physical address the pane's first byte maps to
    uint32_t size = 0;    ///< how far the pane extends; 0 maps nothing at all
};

/// The unit both ATRANS fields count in: one flash sector.
inline constexpr uint32_t qmi_translation_unit = 4096;

constexpr uint32_t qmi_atrans_word(const QmiTranslation& t) {
    return ((t.base / qmi_translation_unit) & 0xFFFu) |
           (((t.size / qmi_translation_unit) & 0x7FFu) << QMI_ATRANS0_SIZE_LSB);
}

constexpr QmiTranslation qmi_atrans_from(uint32_t w) {
    return QmiTranslation{
        .base = (w & QMI_ATRANS0_BASE_BITS) * qmi_translation_unit,
        .size = ((w & QMI_ATRANS0_SIZE_BITS) >> QMI_ATRANS0_SIZE_LSB) * qmi_translation_unit,
    };
}

/// The identity map a QMI reset leaves: four panes of 4 MB, each based
/// where it already is (figure 140).
constexpr QmiTranslation qmi_identity_pane(uint8_t pane) {
    return QmiTranslation{.base = static_cast<uint32_t>(pane) * xip_pane_span, .size = xip_pane_span};
}

/**
 * What the QMI does to an offset inside one 16 MB window, as
 * arithmetic: the pane is the offset's 4 MB slice, the offset inside the
 * pane is added to that pane's BASE, and the result wraps modulo the
 * window (12.14.4's "rolling window"). Nothing when the offset falls
 * past the pane's SIZE - which the silicon answers with a bus error, not
 * with a wrong address.
 *
 * Pure, so a test can judge the silicon's own translator against it.
 */
constexpr std::optional<uint32_t> qmi_translate(uint32_t window_offset,
                                                const std::array<QmiTranslation, 4>& panes) {
    if (window_offset >= xip_window_span) {
        return std::nullopt;
    }
    const uint32_t pane = window_offset / xip_pane_span;
    const uint32_t inside = window_offset % xip_pane_span;
    if (inside >= panes[pane].size) {
        return std::nullopt;
    }
    return (panes[pane].base + inside) % xip_window_span;
}

// =============================================================================
// Qmi: the block, and the direct mode that is the flash window's floor
// =============================================================================

/**
 * The QMI's shared half: direct mode, its FIFOs and its chip selects.
 * Per-window configuration is `QmiWindow<n>`.
 *
 * DIRECT MODE DISCONNECTS THE MEMORY WINDOWS. Nothing here is safe to
 * call from code fetched out of flash, which is why the verbs are
 * offered as MECHANISM and the one sequence that uses them lives in
 * `.ram_text` below. A program that drives the QSPI bus by hand owns the
 * same duty.
 */
struct Qmi {
    Qmi() = delete;

    static constexpr uint32_t base = QMI_BASE;

    static volatile uint32_t& csr() { return QMI->DIRECT_CSR; }

    // ---- direct mode ---------------------------------------------------------

    static bool direct_enabled() { return (csr() & QMI_DIRECT_CSR_EN_BITS) != 0u; }
    /// Enable direct mode. The caller must then wait for `busy()` to
    /// fall, which is what 12.14.5 asks: an XIP transfer in flight at
    /// the moment of the switch has to finish first.
    static void enable_direct(bool on) {
        if (on) {
            hw_set(csr(), QMI_DIRECT_CSR_EN_BITS);
        } else {
            hw_clear(csr(), QMI_DIRECT_CSR_EN_BITS);
        }
    }
    /// Shifting, or within half an SCK period of having finished - which
    /// is the flag chip select timing is taken from, not the RX level.
    static bool busy() { return (csr() & QMI_DIRECT_CSR_BUSY_BITS) != 0u; }

    static bool tx_full() { return (csr() & QMI_DIRECT_CSR_TXFULL_BITS) != 0u; }
    static bool tx_empty() { return (csr() & QMI_DIRECT_CSR_TXEMPTY_BITS) != 0u; }
    static bool rx_full() { return (csr() & QMI_DIRECT_CSR_RXFULL_BITS) != 0u; }
    static bool rx_empty() { return (csr() & QMI_DIRECT_CSR_RXEMPTY_BITS) != 0u; }
    static uint8_t tx_level() {
        return static_cast<uint8_t>((csr() & QMI_DIRECT_CSR_TXLEVEL_BITS) >> QMI_DIRECT_CSR_TXLEVEL_LSB);
    }
    static uint8_t rx_level() {
        return static_cast<uint8_t>((csr() & QMI_DIRECT_CSR_RXLEVEL_BITS) >> QMI_DIRECT_CSR_RXLEVEL_LSB);
    }

    /// Direct mode's own divisor, separate from either window's: control
    /// commands and execute-in-place reads do not share a frequency
    /// limit. Zero encodes 256.
    static uint8_t direct_clkdiv() {
        return static_cast<uint8_t>((csr() & QMI_DIRECT_CSR_CLKDIV_BITS) >> QMI_DIRECT_CSR_CLKDIV_LSB);
    }
    static void set_direct_clkdiv(uint8_t div) {
        hw_write_masked(csr(), static_cast<uint32_t>(div) << QMI_DIRECT_CSR_CLKDIV_LSB,
                        QMI_DIRECT_CSR_CLKDIV_BITS);
    }
    /// Direct mode's own sample delay, in half system clock cycles.
    static uint8_t direct_rxdelay() {
        return static_cast<uint8_t>((csr() & QMI_DIRECT_CSR_RXDELAY_BITS) >> QMI_DIRECT_CSR_RXDELAY_LSB);
    }
    static void set_direct_rxdelay(uint8_t delay) {
        hw_write_masked(csr(), static_cast<uint32_t>(delay) << QMI_DIRECT_CSR_RXDELAY_LSB,
                        QMI_DIRECT_CSR_RXDELAY_BITS);
    }

    /// Push one record. The QMI never drops a received byte: a full RX
    /// FIFO stalls the interface instead, so a caller may push freely as
    /// long as it also pops.
    static void push(uint32_t word) { QMI->DIRECT_TX = word; }
    static void push(const QmiDirectFrame& f) { push(qmi_direct_word(f)); }
    /// Pop one answer. Undefined when `rx_empty()`, which is the
    /// register's own contract and why the loops below check first.
    static uint32_t pop() { return QMI->DIRECT_RX; }

    /// The FIFO addresses a DMA channel is pointed at. The XIP auxiliary
    /// AHB port (4.4.3) reaches the same two FIFOs in one cycle where
    /// the APB port takes several.
    static volatile uint32_t& tx_fifo() { return QMI->DIRECT_TX; }
    static volatile uint32_t& rx_fifo() { return QMI->DIRECT_RX; }
    static volatile uint32_t& fast_tx_fifo() { return XIP_AUX->QMI_DIRECT_TX; }
    static volatile uint32_t& fast_rx_fifo() { return XIP_AUX->QMI_DIRECT_RX; }

    /// One QSPI pad's control word, READ-ONLY. The bootrom configures
    /// these six pads while it scans the flash, and every window here
    /// puts them back afterwards because the ROM's
    /// `connect_internal_flash()` resets them; so what a program has to
    /// do with them is READ them - the drive strength, the slew class,
    /// the input enable the chip was found to work at. Writing one
    /// belongs to a board file that knows that chip, and this file does
    /// not offer it.
    static uint32_t pad_control(QspiPad p) {
        return reg_at(PADS_QSPI_BASE, 4u + 4u * static_cast<uint32_t>(p));
    }

    /// The eight address translation panes back to the identity map
    /// (12.14.4). THE CACHE MUST BE FLUSHED AFTER THIS (12.14.4.2), and
    /// a program whose own code is reached through a translated pane
    /// must not call it at all: the ground it runs on would move.
    static void reset_address_translation() {
        for (uint8_t i = 0; i < 8; ++i) {
            reg_at(base, QMI_ATRANS0_OFFSET + 4u * i) =
                qmi_atrans_word(qmi_identity_pane(static_cast<uint8_t>(i % 4u)));
        }
    }
};

/**
 * One of the QMI's two memory windows (12.14): window 0 is chip select
 * 0, which is where a board's flash is, and window 1 is chip select 1,
 * which on many boards is a PSRAM and on this project's board is
 * nothing.
 *
 *   using Chip = brio::QmiWindow<0>;
 *   const auto t = Chip::timing();          // what the bootrom chose
 *   const auto f = Chip::read_format();     // 03h serial, or EBh quad
 *
 * EVERY WRITE HERE IS A WINDOW OF ITS OWN. Changing the format or the
 * command of the window the program executes from breaks the very
 * fetches that follow, so such a change belongs in `.ram_text` with
 * interrupts masked, exactly like a flash operation. The DIVISOR alone
 * may be changed on the fly (4.4's own note).
 */
template <uint8_t n>
struct QmiWindow {
    static_assert(n < 2, "brio RP2350 QMI: two memory windows, 0 (chip select 0) and 1");

    QmiWindow() = delete;

    static constexpr uint8_t index = n;
    /// The first byte of this window in the cached alias.
    static constexpr uint32_t window_base = xip_base + static_cast<uint32_t>(n) * xip_window_span;
    /// The stride between the two windows' register groups.
    static constexpr uint32_t reg_stride = QMI_M1_TIMING_OFFSET - QMI_M0_TIMING_OFFSET;
    static constexpr uint32_t timing_offset = QMI_M0_TIMING_OFFSET + n * reg_stride;
    static constexpr uint32_t rfmt_offset = QMI_M0_RFMT_OFFSET + n * reg_stride;
    static constexpr uint32_t rcmd_offset = QMI_M0_RCMD_OFFSET + n * reg_stride;
    static constexpr uint32_t wfmt_offset = QMI_M0_WFMT_OFFSET + n * reg_stride;
    static constexpr uint32_t wcmd_offset = QMI_M0_WCMD_OFFSET + n * reg_stride;

    static QmiTiming timing() { return qmi_timing_from(reg_at(Qmi::base, timing_offset)); }
    static void set_timing(const QmiTiming& t) { reg_at(Qmi::base, timing_offset) = qmi_timing_word(t); }
    /// The divisor alone, which 4.4 allows to be changed while the
    /// interface is running - the QMI samples it at each new byte.
    static void set_clkdiv(uint8_t div) {
        hw_write_masked(reg_at(Qmi::base, timing_offset), static_cast<uint32_t>(div),
                        QMI_M0_TIMING_CLKDIV_BITS);
    }

    static QmiTransferFormat read_format() { return qmi_format_from(reg_at(Qmi::base, rfmt_offset)); }
    static void set_read_format(const QmiTransferFormat& f) {
        reg_at(Qmi::base, rfmt_offset) = qmi_format_word(f);
    }
    static QmiTransferFormat write_format() { return qmi_format_from(reg_at(Qmi::base, wfmt_offset)); }
    static void set_write_format(const QmiTransferFormat& f) {
        reg_at(Qmi::base, wfmt_offset) = qmi_format_word(f);
    }

    static QmiCommand read_command() { return qmi_command_from(reg_at(Qmi::base, rcmd_offset)); }
    static void set_read_command(const QmiCommand& c) {
        reg_at(Qmi::base, rcmd_offset) = qmi_command_word(c);
    }
    static QmiCommand write_command() { return qmi_command_from(reg_at(Qmi::base, wcmd_offset)); }
    static void set_write_command(const QmiCommand& c) {
        reg_at(Qmi::base, wcmd_offset) = qmi_command_word(c);
    }

    // ---- address translation -------------------------------------------------

    /// This window's four panes: ATRANS0..3 for window 0, ATRANS4..7 for
    /// window 1.
    template <uint8_t pane>
    static constexpr uint32_t atrans_offset() {
        static_assert(pane < 4, "brio RP2350 QMI: four translation panes a window, 0..3");
        return QMI_ATRANS0_OFFSET + 4u * (4u * n + pane);
    }

    template <uint8_t pane>
    static QmiTranslation translation() {
        return qmi_atrans_from(reg_at(Qmi::base, atrans_offset<pane>()));
    }
    template <uint8_t pane>
    static void set_translation(const QmiTranslation& t) {
        reg_at(Qmi::base, atrans_offset<pane>()) = qmi_atrans_word(t);
    }

    /// All four, in order - the shape `translate()` and a test want.
    static std::array<QmiTranslation, 4> translations() {
        return {translation<0>(), translation<1>(), translation<2>(), translation<3>()};
    }

    /// Is this window mapped one to one, as a QMI reset leaves it?
    static bool translation_is_identity() {
        const auto panes = translations();
        for (uint8_t p = 0; p < 4; ++p) {
            const auto want = qmi_identity_pane(p);
            if (panes[p].base != want.base || panes[p].size != want.size) {
                return false;
            }
        }
        return true;
    }

    /// The physical address this window would reach for `offset`, read
    /// off the LIVE registers. Nothing when the offset falls in a hole.
    static std::optional<uint32_t> translate(uint32_t offset) {
        return qmi_translate(offset, translations());
    }

    // ---- the chip select -----------------------------------------------------

    /// Drive this window's chip select low by hand. IT APPLIES EVEN WITH
    /// DIRECT MODE OFF (12.14.5.2's own warning), so a program that sets
    /// it while an XIP transfer may run has corrupted that transfer.
    static void assert_select(bool low) {
        const uint32_t bit = n == 0 ? QMI_DIRECT_CSR_ASSERT_CS0N_BITS : QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
        if (low) {
            hw_set(Qmi::csr(), bit);
        } else {
            hw_clear(Qmi::csr(), bit);
        }
    }
    /// Let the block drive the chip select low for as long as it is
    /// busy: the other of 12.14.5.2's two arrangements, and the one that
    /// needs no timing care from the caller.
    static void auto_select(bool on) {
        const uint32_t bit = n == 0 ? QMI_DIRECT_CSR_AUTO_CS0N_BITS : QMI_DIRECT_CSR_AUTO_CS1N_BITS;
        if (on) {
            hw_set(Qmi::csr(), bit);
        } else {
            hw_clear(Qmi::csr(), bit);
        }
    }
    static bool select_asserted() {
        const uint32_t bit = n == 0 ? QMI_DIRECT_CSR_ASSERT_CS0N_BITS : QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
        return (Qmi::csr() & bit) != 0u;
    }
};

// =============================================================================
// Xip: the cache in front of the QMI (4.4.1)
// =============================================================================

/// The five operations a write into the maintenance window performs,
/// selected by the three least significant bits of the address.
enum class XipMaintenance : uint32_t {
    invalidate_by_set_way = 0,
    clean_by_set_way = 1,
    invalidate_by_address = 2,
    clean_by_address = 3,
    pin_by_set_way = 7,
};

/**
 * The XIP cache: 16 kB, two-way set-associative, eight-byte lines, one
 * cycle on a hit.
 *
 *   brio::Xip::reset_counters();
 *   // ... run the code whose locality is in question ...
 *   const uint32_t ratio = 100u * brio::Xip::hits() / brio::Xip::accesses();
 *
 * WHAT IS NOT THE RP2040'S. There is no single flush register: a flush
 * is a sweep of writes through the maintenance window, one per line.
 * There are two enables, Secure and Non-secure, and five more bits that
 * close the uncached and untranslated aliases or hand maintenance to
 * Non-secure code. A line can be PINNED, which is this chip's
 * cache-as-SRAM. And a write into the window is DOWNGRADED TO A READ
 * unless the matching WRITABLE bit is set, which is what keeps a stray
 * store from breaking a flash chip out of continuous read mode.
 */
struct Xip {
    Xip() = delete;

    static constexpr uint32_t cache_bytes = 16 * 1024;
    static constexpr uint32_t line_bytes = 8;
    static constexpr uint16_t sets = 1024;
    static constexpr uint8_t ways = 2;
    static constexpr uint32_t lines = cache_bytes / line_bytes;

    static volatile uint32_t& ctrl() { return XIP_CTRL->CTRL; }
    static volatile uint32_t& stat() { return XIP_CTRL->STAT; }

    // ---- the two enables and what sits beside them ---------------------------

    static bool enabled_secure() { return (ctrl() & XIP_CTRL_EN_SECURE_BITS) != 0u; }
    static bool enabled_nonsecure() { return (ctrl() & XIP_CTRL_EN_NONSECURE_BITS) != 0u; }
    /// Both at once: what a program that is not partitioning the chip
    /// wants, and what brio's Secure-only world means by "the cache".
    static void enable(bool on) {
        const uint32_t bits = XIP_CTRL_EN_SECURE_BITS | XIP_CTRL_EN_NONSECURE_BITS;
        if (on) {
            hw_set(ctrl(), bits);
        } else {
            hw_clear(ctrl(), bits);
        }
    }
    static void enable_secure(bool on) {
        if (on) {
            hw_set(ctrl(), XIP_CTRL_EN_SECURE_BITS);
        } else {
            hw_clear(ctrl(), XIP_CTRL_EN_SECURE_BITS);
        }
    }
    static void enable_nonsecure(bool on) {
        if (on) {
            hw_set(ctrl(), XIP_CTRL_EN_NONSECURE_BITS);
        } else {
            hw_clear(ctrl(), XIP_CTRL_EN_NONSECURE_BITS);
        }
    }

    /// The cache memories powered down: the contents stay, the cache
    /// cannot be reached, and the two enables are forced to zero by the
    /// hardware itself.
    static bool powered_down() { return (ctrl() & XIP_CTRL_POWER_DOWN_BITS) != 0u; }
    static void power_down(bool on) {
        if (on) {
            hw_set(ctrl(), XIP_CTRL_POWER_DOWN_BITS);
        } else {
            hw_clear(ctrl(), XIP_CTRL_POWER_DOWN_BITS);
        }
    }

    /// Secure accesses to way 0, Non-secure to way 1: two direct-mapped
    /// halves instead of one two-way cache. A FULL FLUSH IS REQUIRED
    /// around the change and must happen while the bit is clear, so this
    /// verb does the flush itself, in the right order.
    static bool split_ways() { return (ctrl() & XIP_CTRL_SPLIT_WAYS_BITS) != 0u; }
    static void set_split_ways(bool on) {
        if (split_ways()) {
            hw_clear(ctrl(), XIP_CTRL_SPLIT_WAYS_BITS);
        }
        invalidate_all();
        if (on) {
            hw_set(ctrl(), XIP_CTRL_SPLIT_WAYS_BITS);
        }
    }

    static bool maintenance_nonsecure() { return (ctrl() & XIP_CTRL_MAINT_NONSEC_BITS) != 0u; }
    static void allow_nonsecure_maintenance(bool on) {
        if (on) {
            hw_set(ctrl(), XIP_CTRL_MAINT_NONSEC_BITS);
        } else {
            hw_clear(ctrl(), XIP_CTRL_MAINT_NONSEC_BITS);
        }
    }

    /// The four bits that turn an alias into a bus error, per alias and
    /// per security level. A program with no Non-secure half never needs
    /// them; they are here because the chapter has them.
    static void refuse_uncached(bool secure, bool nonsecure) {
        hw_write_masked(ctrl(),
                        (secure ? XIP_CTRL_NO_UNCACHED_SEC_BITS : 0u) |
                            (nonsecure ? XIP_CTRL_NO_UNCACHED_NONSEC_BITS : 0u),
                        XIP_CTRL_NO_UNCACHED_SEC_BITS | XIP_CTRL_NO_UNCACHED_NONSEC_BITS);
    }
    static void refuse_untranslated(bool secure, bool nonsecure) {
        hw_write_masked(ctrl(),
                        (secure ? XIP_CTRL_NO_UNTRANSLATED_SEC_BITS : 0u) |
                            (nonsecure ? XIP_CTRL_NO_UNTRANSLATED_NONSEC_BITS : 0u),
                        XIP_CTRL_NO_UNTRANSLATED_SEC_BITS | XIP_CTRL_NO_UNTRANSLATED_NONSEC_BITS);
    }
    static bool uncached_refused_secure() { return (ctrl() & XIP_CTRL_NO_UNCACHED_SEC_BITS) != 0u; }
    static bool untranslated_refused_secure() {
        return (ctrl() & XIP_CTRL_NO_UNTRANSLATED_SEC_BITS) != 0u;
    }

    /// Whether a store into one window's address range reaches the
    /// device. READ-ONLY BY DEFAULT, and that default is a safety
    /// property and not a convenience: a write to a flash would appear
    /// to succeed in the cache and then, on eviction, issue a write
    /// command that can leave the chip returning garbage.
    template <uint8_t window>
    static bool window_writable() {
        static_assert(window < 2, "brio RP2350 XIP: two memory windows, 0 and 1");
        return (ctrl() & (window == 0 ? XIP_CTRL_WRITABLE_M0_BITS : XIP_CTRL_WRITABLE_M1_BITS)) != 0u;
    }
    template <uint8_t window>
    static void set_window_writable(bool on) {
        static_assert(window < 2, "brio RP2350 XIP: two memory windows, 0 and 1");
        constexpr uint32_t bit = window == 0 ? XIP_CTRL_WRITABLE_M0_BITS : XIP_CTRL_WRITABLE_M1_BITS;
        if (on) {
            hw_set(ctrl(), bit);
        } else {
            hw_clear(ctrl(), bit);
        }
    }

    // ---- the two counters (4.4.4) -------------------------------------------

    /// Accesses through any alias, and the hits among them. Both
    /// saturate at all ones and are cleared by a write of any value.
    static uint32_t accesses() { return XIP_CTRL->CTR_ACC; }
    static uint32_t hits() { return XIP_CTRL->CTR_HIT; }
    static void reset_counters() {
        XIP_CTRL->CTR_HIT = 0;
        XIP_CTRL->CTR_ACC = 0;
    }

    // ---- maintenance (4.4.1.1) ----------------------------------------------

    /// The address one maintenance operation is written to.
    static constexpr uint32_t maintenance_address(uint32_t address_or_line, XipMaintenance op) {
        return xip_maintenance_base + address_or_line + static_cast<uint32_t>(op);
    }

    /// Every line marked invalid: what the flash operations below need,
    /// because the chip changed under a cache that cannot know.
    static void invalidate_all() {
        for (uint32_t i = 0; i < cache_bytes; i += line_bytes) {
            write_maintenance(maintenance_address(i, XipMaintenance::invalidate_by_set_way));
        }
    }

    /**
     * Every dirty line written out, none invalidated.
     *
     * ERRATUM RP2350-E11 IS IN THIS VERB. A clean by set/way also
     * OVERWRITES THE LINE'S TAG with bits 25:13 of the address the
     * maintenance write itself used, so a sweep driven from 0x1800_0000
     * would leave every cleaned line claiming to hold address zero -
     * and the next read of the flash's first bytes would hit that line.
     * The sweep is therefore driven from the TOP of the maintenance
     * window, where the tag it leaves behind names the reserved upper
     * half of the downstream space, which no QMI access can ever ask
     * for. The cost is a miss on the next access to a cleaned address,
     * which is what a clean is for.
     */
    static void clean_all() {
        constexpr uint32_t sweep = xip_alias_span - cache_bytes;   // the last 16 kB of the window
        for (uint32_t i = 0; i < cache_bytes; i += line_bytes) {
            write_maintenance(maintenance_address(sweep + i, XipMaintenance::clean_by_set_way));
        }
    }

    /// One address looked up and maintained if it is allocated. `addr`
    /// is a byte offset in the 26-bit downstream space - what
    /// `Flash::address()` points at, less `xip_base`.
    static void invalidate_address(uint32_t addr) {
        write_maintenance(maintenance_address(addr & ~(line_bytes - 1u),
                                              XipMaintenance::invalidate_by_address));
    }
    static void clean_address(uint32_t addr) {
        write_maintenance(maintenance_address(addr & ~(line_bytes - 1u),
                                              XipMaintenance::clean_by_address));
    }
    /// A run of addresses, line by line: 4.4.1.1's "usually faster than
    /// a full flush" when only one page changed.
    static void invalidate_range(uint32_t addr, uint32_t bytes) {
        const uint32_t first = addr & ~(line_bytes - 1u);
        for (uint32_t a = first; a < addr + bytes; a += line_bytes) {
            invalidate_address(a);
        }
    }

    /**
     * Pin the line `addr` maps to, so it is never evicted (4.4.1.3).
     * THE OPERATION COPIES NOTHING: it marks the line and writes the
     * tag, so a pinned line that is to hold a device's contents must be
     * filled afterwards by reading the same address through the
     * UNCACHED alias. A pinned line leaves the pinned state only by an
     * invalidate, and survives a processor reset - the tag memory is in
     * the XIP power domain.
     */
    static void pin_address(uint32_t addr) {
        write_maintenance(maintenance_address(addr & ~(line_bytes - 1u), XipMaintenance::pin_by_set_way));
    }

    // ---- the streaming interface (4.4.3) ------------------------------------

    /// A linear run of words read in the background at lower priority
    /// than the program's own fetches, pushed into a small FIFO whose
    /// DREQ paces a DMA channel. `addr` is a full XIP address.
    static void stream_start(uint32_t addr, uint32_t words) {
        stream_stop();
        XIP_CTRL->STREAM_ADDR = addr & ~0x3u;
        XIP_CTRL->STREAM_CTR = words;
    }
    /// The run abandoned and the FIFO emptied - what a program does
    /// before starting another, since the count alone does not drain it.
    static void stream_stop() {
        XIP_CTRL->STREAM_CTR = 0;
        while (!stream_empty()) {
            (void)XIP_CTRL->STREAM_FIFO;
        }
    }
    /// How many words the engine has still to fetch.
    static uint32_t stream_remaining() { return XIP_CTRL->STREAM_CTR; }
    static bool stream_empty() { return (stat() & XIP_STAT_FIFO_EMPTY_BITS) != 0u; }
    static bool stream_full() { return (stat() & XIP_STAT_FIFO_FULL_BITS) != 0u; }
    /// One word, or nothing when the FIFO has not caught up yet.
    static std::optional<uint32_t> stream_pop() {
        if (stream_empty()) {
            return std::nullopt;
        }
        return XIP_CTRL->STREAM_FIFO;
    }
    /// The FIFO as an address, for a DMA channel: the APB one and the
    /// auxiliary AHB port's single-cycle alias.
    static volatile uint32_t& stream_fifo() { return XIP_CTRL->STREAM_FIFO; }
    static volatile uint32_t& fast_stream_fifo() { return XIP_AUX->STREAM; }

private:
    /// A maintenance write. The data is ignored; the ADDRESS is the
    /// whole instruction.
    static void write_maintenance(uint32_t address) {
        *reinterpret_cast<volatile uint8_t*>(static_cast<uintptr_t>(address)) = 0;
    }
};

// =============================================================================
// The flash chip: what may be asked of it, and what it answers
// =============================================================================

/**
 * THE ALLOW-LIST. Every opcode this driver will clock out, and every one
 * of them a READ. The rule behind it is the bench's: this project
 * carries one RP2350 and the chip beside it holds the image, and a
 * status or configuration register is one-way on a good many QSPI parts
 * - a quad-enable, a lock bit or a four-byte-address mode set by
 * accident is a board that no longer boots. So there is no write enable
 * here, no register write, no erase and no program opcode; the erase and
 * program this file does offer go through the bootrom's own functions,
 * which take an address and a length and not an opcode.
 */
struct FlashCommand {
    FlashCommand() = delete;

    static constexpr uint8_t read_data = 0x03;            ///< serial read, 24-bit address
    static constexpr uint8_t fast_read = 0x0B;            ///< serial read, 24-bit address, 8 dummy bits
    static constexpr uint8_t read_status1 = 0x05;         ///< busy, write-enable latch, protection
    static constexpr uint8_t read_status2 = 0x35;         ///< quad enable and the rest, where a chip has it
    static constexpr uint8_t read_status3 = 0x15;         ///< drive strength and the rest, where a chip has it
    static constexpr uint8_t read_sfdp = 0x5A;            ///< the JEDEC parameter tables, 24-bit address
    static constexpr uint8_t read_jedec_id = 0x9F;        ///< manufacturer, type, capacity
    static constexpr uint8_t read_unique_id = 0x4B;       ///< 64 bits, after 32 dummy bits
    static constexpr uint8_t read_device_id = 0x90;       ///< manufacturer and device, 24-bit address
};

/// Is `opcode` one of the reads above? The one gate every raw command
/// passes through.
constexpr bool flash_command_reads_only(uint8_t opcode) {
    return opcode == FlashCommand::read_data || opcode == FlashCommand::fast_read ||
           opcode == FlashCommand::read_status1 || opcode == FlashCommand::read_status2 ||
           opcode == FlashCommand::read_status3 || opcode == FlashCommand::read_sfdp ||
           opcode == FlashCommand::read_jedec_id || opcode == FlashCommand::read_unique_id ||
           opcode == FlashCommand::read_device_id;
}

/// What 9Fh answers: the JEDEC manufacturer, the memory type, the
/// capacity as a power of two (0x18 = 16 MB).
struct FlashJedecId {
    uint8_t manufacturer;
    uint8_t type;
    uint8_t capacity;

    static constexpr uint8_t winbond = 0xEF;
    static constexpr uint8_t zetta = 0xBA;

    constexpr uint32_t bytes() const { return capacity < 32u ? (1ul << capacity) : 0u; }
};

/// The runtime FLASH_DEVINFO the bootrom keeps in boot RAM (5.4.8.5),
/// decoded. It is what the ROM's high-level flash entry points bound
/// their addresses against, and on a board that has programmed no OTP it
/// is the ROM's own default: 16 MB on chip select 0, nothing on chip
/// select 1, no secondary chip select pin, and NO D8h BLOCK ERASE - the
/// last of which is a statement about what the ROM will do and not about
/// what the chip can do.
struct FlashDevInfo {
    uint16_t raw = 0;

    /// The size code's bytes: 0 means "none", and every other code is
    /// 8 kB shifted left by code - 1.
    static constexpr uint32_t size_bytes(uint8_t code) {
        return code == 0u ? 0u : (8u * 1024u) << (code - 1u);
    }

    constexpr uint8_t cs0_size_code() const {
        return static_cast<uint8_t>((raw & OTP_DATA_FLASH_DEVINFO_CS0_SIZE_BITS) >>
                                    OTP_DATA_FLASH_DEVINFO_CS0_SIZE_LSB);
    }
    constexpr uint8_t cs1_size_code() const {
        return static_cast<uint8_t>((raw & OTP_DATA_FLASH_DEVINFO_CS1_SIZE_BITS) >>
                                    OTP_DATA_FLASH_DEVINFO_CS1_SIZE_LSB);
    }
    constexpr uint32_t cs0_bytes() const { return size_bytes(cs0_size_code()); }
    constexpr uint32_t cs1_bytes() const { return size_bytes(cs1_size_code()); }
    /// Which bank 0 pad carries the second chip select. ERRATUM
    /// RP2350-E14 IS ABOUT THIS FIELD: on A2 silicon the ROM's
    /// `connect_internal_flash()` ignores it and always uses GPIO 0. It
    /// matters only to a board that has a device on chip select 1, and
    /// nothing in brio writes this field.
    constexpr uint8_t cs1_gpio() const {
        return static_cast<uint8_t>((raw & OTP_DATA_FLASH_DEVINFO_CS1_GPIO_BITS) >>
                                    OTP_DATA_FLASH_DEVINFO_CS1_GPIO_LSB);
    }
    constexpr bool d8h_erase_supported() const {
        return (raw & OTP_DATA_FLASH_DEVINFO_D8H_ERASE_SUPPORTED_BITS) != 0u;
    }
};

/// How an erase run may be broken up.
enum class FlashEraseGrain : uint8_t {
    sector,            ///< 20h only: 4 kB at a time, whatever the run's alignment
    sector_or_block,   ///< the 64 kB D8h command where the run is aligned to it
};

/// Which of the two ways back into execute-in-place an image has.
enum class XipRestore : uint8_t {
    none,           ///< `init()` has not run, or found no bootrom table
    setup_function, ///< the bootrom's own, copied out of boot RAM: the mode found at boot
    rom_cmd_xip,    ///< the ROM's `flash_enter_cmd_xip()`: 03h serial reads at CLKDIV 12
};

namespace detail {

/// The bootrom's flash functions, found once, and the XIP setup function
/// copied out once; then the operations that run with the memory windows
/// disconnected. Not for an application - `Flash` below is the surface.
struct FlashRom {
    FlashRom() = delete;

    using PlainFn = void (*)();
    using RangeEraseFn = void (*)(uint32_t addr, uint32_t count, uint32_t block_size, uint8_t block_cmd);
    using RangeProgramFn = void (*)(uint32_t addr, const uint8_t* data, uint32_t count);
    using TranslateFn = int32_t (*)(uint32_t addr);

    static inline PlainFn connect = nullptr;        ///< 'I','F' - the QSPI pads and the QMI back to the device
    static inline PlainFn exit_xip = nullptr;       ///< 'E','X' - direct mode, and the device out of continuous read
    static inline RangeEraseFn range_erase = nullptr;      ///< 'R','E'
    static inline RangeProgramFn range_program = nullptr;  ///< 'R','P'
    static inline PlainFn flush_cache = nullptr;    ///< 'F','C' - every cache line invalidated
    static inline PlainFn enter_cmd_xip = nullptr;  ///< 'C','X' - 03h at CLKDIV 12, the fallback way back
    static inline TranslateFn runtime_to_storage = nullptr; ///< 'F','A' - arithmetic in ROM, no window needed

    static constexpr uint32_t setup_bytes = 256;
    /// The copy the window calls. In .bss, which this linker script puts
    /// in a region the core may fetch from; boot RAM itself is on the
    /// APB and is physically not executable (4.3).
    alignas(4) static inline uint8_t setup[setup_bytes]{};
    static inline XipRestore restore = XipRestore::none;
    static inline const uint16_t* devinfo = nullptr;
    static inline bool named_by_rom = false;   ///< did 'X','F' name the setup function, or was the base taken?
    static inline bool ready = false;

    static constexpr uint32_t code_connect = bootrom_code('I', 'F');
    static constexpr uint32_t code_exit_xip = bootrom_code('E', 'X');
    static constexpr uint32_t code_range_erase = bootrom_code('R', 'E');
    static constexpr uint32_t code_range_program = bootrom_code('R', 'P');
    static constexpr uint32_t code_flush_cache = bootrom_code('F', 'C');
    static constexpr uint32_t code_enter_cmd_xip = bootrom_code('C', 'X');
    static constexpr uint32_t code_runtime_to_storage = bootrom_code('F', 'A');
    static constexpr uint32_t code_xip_setup_ptr = bootrom_code('X', 'F');
    static constexpr uint32_t code_devinfo_ptr = bootrom_code('F', 'D');

    /**
     * Where the bootrom left its XIP setup function. TWO ROUTES TO ONE
     * ADDRESS, because the datasheet gives two: 4.3 and 5.2.7 say the
     * function is written to the BASE of boot RAM, and 5.4.8.30 names a
     * ROM data entry for it.
     *
     * Both ROM DATA entries of 5.4.8 are a pointer TO a pointer - the
     * table holds the address of a word in ROM, and that word holds the
     * address wanted - so the lookup is dereferenced once and then
     * checked against boot RAM, which is the only place such a function
     * may live. When it does not answer that, the documented base is
     * used, and `named_by_rom` says which route this image took.
     */
    static const uint8_t* setup_source() {
        const auto* cell = static_cast<const uint32_t* const*>(Bootrom::lookup_data(code_xip_setup_ptr));
        if (cell != nullptr) {
            const auto address = reinterpret_cast<uintptr_t>(*cell);
            if (address >= bootram_base && address + setup_bytes <= bootram_base + bootram_bytes) {
                named_by_rom = true;
                return reinterpret_cast<const uint8_t*>(address);
            }
        }
        named_by_rom = false;
        return reinterpret_cast<const uint8_t*>(static_cast<uintptr_t>(bootram_base));
    }

    /// The boot RAM copy of FLASH_DEVINFO, by the same rule.
    static const uint16_t* devinfo_source() {
        const auto* cell = static_cast<const uint16_t* const*>(Bootrom::lookup_data(code_devinfo_ptr));
        if (cell == nullptr) {
            return nullptr;
        }
        const auto address = reinterpret_cast<uintptr_t>(*cell);
        if (address < bootram_base || address + 2u > bootram_base + bootram_bytes) {
            return nullptr;
        }
        return reinterpret_cast<const uint16_t*>(address);
    }

    /**
     * Find the six functions, copy the setup function out, note the
     * DEVINFO pointer. Once, from flash - the windows are still there.
     *
     * The setup function has no checksum of its own (unlike the
     * RP2040's second stage), so all that can be asked of it is that
     * the ROM names it, that the name points into boot RAM, and that
     * the first word is neither all zeros nor all ones. A copy that
     * fails those tests is not used, and the way back becomes the ROM's
     * `flash_enter_cmd_xip()` - slower, and correct.
     */
    static bool ensure() {
        if (ready) {
            return true;
        }
        connect = reinterpret_cast<PlainFn>(Bootrom::lookup_function(code_connect));
        exit_xip = reinterpret_cast<PlainFn>(Bootrom::lookup_function(code_exit_xip));
        range_erase = reinterpret_cast<RangeEraseFn>(Bootrom::lookup_function(code_range_erase));
        range_program = reinterpret_cast<RangeProgramFn>(Bootrom::lookup_function(code_range_program));
        flush_cache = reinterpret_cast<PlainFn>(Bootrom::lookup_function(code_flush_cache));
        enter_cmd_xip = reinterpret_cast<PlainFn>(Bootrom::lookup_function(code_enter_cmd_xip));
        runtime_to_storage =
            reinterpret_cast<TranslateFn>(Bootrom::lookup_function(code_runtime_to_storage));
        if (connect == nullptr || exit_xip == nullptr || range_erase == nullptr ||
            range_program == nullptr || flush_cache == nullptr || enter_cmd_xip == nullptr) {
            return false;
        }
        devinfo = devinfo_source();
        restore = XipRestore::rom_cmd_xip;
        if (const uint8_t* from = setup_source()) {
            memcpy(setup, from, setup_bytes);
            uint32_t first = 0;
            memcpy(&first, setup, sizeof(first));
            if (first != 0u && first != 0xFFFF'FFFFu) {
                restore = XipRestore::setup_function;
            }
        }
        ready = true;
        return true;
    }

    // ---- what runs with the memory windows disconnected ----------------------
    //
    // Every function below is in .ram_text, never inlined into a caller
    // that lives in flash, and FLATTENED so that what it calls is folded
    // in: the bootrom's functions and the setup copy are reached through
    // pointers, the register accesses are stores. No switch (its jump
    // table would be .rodata), no library call - memcpy is in flash.

    /// The way back: the bootrom's setup function if this image has a
    /// good copy, else the ROM's own 03h mode. Bit 0 is set because the
    /// Arm half needs the Thumb bit; the RISC-V half's JALR clears it
    /// again, so one spelling serves both.
    [[gnu::always_inline]] static void reenter_xip() {
        if (restore == XipRestore::setup_function) {
            const auto fn = reinterpret_cast<PlainFn>(reinterpret_cast<uintptr_t>(setup) | 1u);
            fn();
        } else {
            enter_cmd_xip();
        }
    }

    /// The six QSPI pad registers and window 1's trio, saved across a
    /// window: `connect_internal_flash()` resets the pads and
    /// `flash_exit_xip()` rewrites both windows' read configuration.
    struct Saved {
        uint32_t pads[6];
        uint32_t m1_timing;
        uint32_t m1_rfmt;
        uint32_t m1_rcmd;
        bool restore_m1;
    };

    /// One pad control of the QSPI bank: VOLTAGE_SELECT sits at offset
    /// zero and is a bank-wide fact the ROM does not touch, so the six
    /// per-pad registers start one word above it.
    [[gnu::always_inline]] static volatile uint32_t& qspi_pad(uint32_t i) {
        return reg_at(PADS_QSPI_BASE, 4u + 4u * i);
    }

    /**
     * NOTHING BELOW MAY BE VALUE-INITIALIZED. A `Saved s{}` is a nine
     * word zero fill, which at -Os is a CALL TO memset - and memset
     * lives in flash, which is the one thing a window may not touch.
     * The whole record is therefore built by aggregate initialization,
     * every member from a register read.
     */
    [[gnu::always_inline]] static Saved save() {
        return Saved{
            .pads = {qspi_pad(0), qspi_pad(1), qspi_pad(2), qspi_pad(3), qspi_pad(4), qspi_pad(5)},
            .m1_timing = QMI->M1_TIMING,
            .m1_rfmt = QMI->M1_RFMT,
            .m1_rcmd = QMI->M1_RCMD,
            // Window 1's trio goes back only when nothing is attached to
            // chip select 1: then the ROM issues no exit sequence there
            // and the old configuration is still the right one. With a
            // device attached it is not, and this driver has no setup
            // function for one.
            .restore_m1 = devinfo == nullptr ||
                          ((*devinfo & OTP_DATA_FLASH_DEVINFO_CS1_SIZE_BITS) == 0u),
        };
    }

    [[gnu::always_inline]] static void restore_state(const Saved& s) {
        for (uint32_t i = 0; i < 6; ++i) {
            qspi_pad(i) = s.pads[i];
        }
        if (s.restore_m1) {
            QMI->M1_TIMING = s.m1_timing;
            QMI->M1_RFMT = s.m1_rfmt;
            QMI->M1_RCMD = s.m1_rcmd;
        }
    }

    [[gnu::section(".ram_text"), gnu::noinline, gnu::flatten]]
    static void erase_in_ram(uint32_t addr, uint32_t count, uint32_t block_size, uint8_t block_cmd) {
        InterruptGuard guard;
        const Saved s = save();
        connect();
        exit_xip();
        range_erase(addr, count, block_size, block_cmd);
        flush_cache();
        reenter_xip();
        restore_state(s);
    }

    [[gnu::section(".ram_text"), gnu::noinline, gnu::flatten]]
    static void program_in_ram(uint32_t addr, const uint8_t* data, uint32_t count) {
        InterruptGuard guard;
        const Saved s = save();
        connect();
        exit_xip();
        range_program(addr, data, count);
        flush_cache();
        reenter_xip();
        restore_state(s);
    }

    /**
     * One raw command on chip select 0, in direct mode.
     *
     * The opcode and the `head_len - 1` bytes behind it go out with
     * NOPUSH set, so their answers never reach the RX FIFO - which is
     * what 12.14.5.1 offers the bit for, and what lets the reply land in
     * `rx` from its first byte with no offset arithmetic. Then `rx_len`
     * zero bytes are clocked out and their answers popped.
     *
     * The FIFO needs no accounting: a full RX FIFO STALLS the interface
     * rather than dropping a byte (12.14.5), so the only rule is to pop
     * while pushing, which this loop does.
     */
    [[gnu::section(".ram_text"), gnu::noinline, gnu::flatten]]
    static void command_in_ram(const uint8_t* head, uint32_t head_len, uint8_t* rx, uint32_t rx_len) {
        InterruptGuard guard;
        const Saved s = save();
        connect();
        exit_xip();

        volatile uint32_t& csr = QMI->DIRECT_CSR;
        reg_at(QMI_BASE + reg_alias_set, QMI_DIRECT_CSR_OFFSET) = QMI_DIRECT_CSR_EN_BITS;
        while ((csr & QMI_DIRECT_CSR_BUSY_BITS) != 0u) {
        }
        reg_at(QMI_BASE + reg_alias_set, QMI_DIRECT_CSR_OFFSET) = QMI_DIRECT_CSR_ASSERT_CS0N_BITS;

        for (uint32_t i = 0; i < head_len; ++i) {
            while ((csr & QMI_DIRECT_CSR_TXFULL_BITS) != 0u) {
            }
            QMI->DIRECT_TX = static_cast<uint32_t>(head[i]) | QMI_DIRECT_TX_NOPUSH_BITS;
        }
        uint32_t to_send = rx_len;
        uint32_t to_take = rx_len;
        while (to_send != 0u || to_take != 0u) {
            const uint32_t flags = csr;
            if (to_send != 0u && (flags & QMI_DIRECT_CSR_TXFULL_BITS) == 0u) {
                QMI->DIRECT_TX = 0;
                --to_send;
            }
            if (to_take != 0u && (flags & QMI_DIRECT_CSR_RXEMPTY_BITS) == 0u) {
                rx[rx_len - to_take] = static_cast<uint8_t>(QMI->DIRECT_RX);
                --to_take;
            }
        }
        // BUSY stands for half an SCK period past the last bit, which is
        // exactly the chip select hold this waits out (12.14.5.2).
        while ((csr & QMI_DIRECT_CSR_BUSY_BITS) != 0u) {
        }
        reg_at(QMI_BASE + reg_alias_clr, QMI_DIRECT_CSR_OFFSET) = QMI_DIRECT_CSR_ASSERT_CS0N_BITS;
        reg_at(QMI_BASE + reg_alias_clr, QMI_DIRECT_CSR_OFFSET) = QMI_DIRECT_CSR_EN_BITS;

        flush_cache();
        reenter_xip();
        restore_state(s);
    }
};

} // namespace detail

/**
 * The flash chip: the geometry the build states, and the bootrom's
 * operations behind bounds and alignment checks.
 *
 *   if (brio::Flash::init()) {
 *       const auto id = brio::Flash::jedec_id();          // 9Fh
 *       brio::Flash::erase_sector(0x00ff'0000);
 *       brio::Flash::program(0x00ff'0000, page);          // page in SRAM
 *   }
 *
 * Every verb that opens a window costs the whole program the window's
 * duration, with this core's interrupts masked and the other core's
 * flash access forbidden. A `read()` costs nothing: it is a memcpy
 * through the cached alias.
 */
struct Flash {
    Flash() = delete;

    static constexpr uint32_t size_bytes = static_cast<uint32_t>(BRIO_RP2350_FLASH_KB) * 1024u;
    static constexpr uint32_t page_size = 256;      ///< the program unit the ROM's 'R','P' takes
    static constexpr uint32_t sector_size = 4096;   ///< the erase unit (20h)
    static constexpr uint32_t block_size = 65536;   ///< the larger erase (D8h), where a run is aligned to it
    static constexpr uint8_t block_command = 0xD8;
    static constexpr uint32_t setup_bytes = detail::FlashRom::setup_bytes;
    static constexpr uint32_t max_command_head = 8;  ///< opcode plus address and dummy bytes
    static constexpr uint32_t window = xip_base;

    /// A block size no run of this chip can be aligned to AND long
    /// enough for, which is how `FlashEraseGrain::sector` tells the ROM
    /// to use the 4 kB command and nothing else: the ROM takes the
    /// larger command only where both hold.
    static constexpr uint32_t no_block = 0x8000'0000u;

    static_assert(size_bytes % sector_size == 0u, "a flash of whole sectors");
    static_assert(size_bytes <= xip_window_span, "one chip select reaches 16 MB (12.14.4)");

    /// Find the bootrom's flash functions and copy the XIP setup
    /// function. Every operation calls it itself; an application calls
    /// it to learn early. False when the ROM carries no table or a
    /// function is missing from it.
    static bool init() { return detail::FlashRom::ensure(); }
    static bool ready() { return detail::FlashRom::ready; }

    /// Which way back into execute-in-place this image will take out of
    /// a window - a measurement, because it decides whether the flash
    /// runs at the mode the bootrom found or at 03h and CLKDIV 12.
    static XipRestore xip_restore_kind() { return detail::FlashRom::restore; }
    /// Whether the ROM's own data table named the setup function, or the
    /// documented base of boot RAM had to serve instead - two routes to
    /// one address, and a fact worth reporting because only one of them
    /// is a promise the ROM keeps across releases.
    static bool xip_setup_named_by_rom() { return detail::FlashRom::named_by_rom; }
    /// The setup function as copied, word by word, for the record.
    static uint32_t setup_word(uint32_t index) {
        uint32_t w = 0;
        if (index * 4u + 4u <= setup_bytes) {
            memcpy(&w, detail::FlashRom::setup + index * 4u, sizeof(w));
        }
        return w;
    }

    /**
     * The QMI's address translation applied by the ROM ITSELF
     * (5.4.8.13), for a full XIP runtime address. Nothing when the ROM
     * refuses it - outside the window, or in the hole a pane's SIZE
     * leaves.
     *
     * It is pure arithmetic over the live ATRANS registers and opens no
     * window, so it costs nothing and is safe anywhere. It is also the
     * ORACLE `QmiWindow<n>::translate()` is judged against: two
     * independent readings of the same eight registers.
     *
     * THE ANSWER IS THE ROM'S OWN, IN THE ROM'S OWN BASE, and this
     * wrapper does not shift it. 5.4.8.13 says "the storage address",
     * which reads like an offset in the chip, while 5.4.8.9 expresses
     * every flash address from the window's base; the chapter does not
     * settle which, so a caller that compares this with its own
     * arithmetic establishes the base once, by asking for an address it
     * knows the answer to. `test_rp2350_flash` letter d does exactly
     * that, and prints what it found.
     */
    static std::optional<uint32_t> runtime_to_storage(uint32_t runtime_address) {
        if (!init() || detail::FlashRom::runtime_to_storage == nullptr) {
            return std::nullopt;
        }
        const int32_t r = detail::FlashRom::runtime_to_storage(runtime_address);
        if (r < 0) {
            return std::nullopt;
        }
        return static_cast<uint32_t>(r);
    }

    /**
     * Commit anything the program wrote THROUGH the cache before a
     * window opens. The ROM's `flash_flush_cache()` is an INVALIDATE of
     * every line and would drop a dirty one, so a program that has made
     * a memory window writable - which only a RAM on the QSPI bus makes
     * sense of - needs its lines cleaned first. Costs nothing at all
     * when neither window is writable, which is the reset state and the
     * only state a board with a flash and nothing else is ever in.
     *
     * It runs BEFORE the window, so it may live in flash like the rest
     * of this file: a maintenance write is not a QSPI access.
     */
    static void commit_writes() {
        if (Xip::window_writable<0>() || Xip::window_writable<1>()) {
            Xip::clean_all();
        }
    }

    /// The runtime FLASH_DEVINFO, or nothing when the ROM does not name
    /// it. READ-ONLY here: writing it would change what the ROM's own
    /// bounds checks believe about the board.
    static std::optional<FlashDevInfo> device_info() {
        if (!init() || detail::FlashRom::devinfo == nullptr) {
            return std::nullopt;
        }
        return FlashDevInfo{.raw = *detail::FlashRom::devinfo};
    }

    /// Is this pointer anywhere in the XIP address space - any window,
    /// any of the four aliases? What a window's operations refuse as a
    /// source or a destination: the bootrom would read or write it
    /// through a memory interface that is not there.
    static bool in_window(const void* p) {
        const auto a = reinterpret_cast<uintptr_t>(p);
        return a >= xip_base && a < xip_base + xip_space_span;
    }

    /// The flash byte at `offset`, through the cache.
    static const uint8_t* address(uint32_t offset) {
        return reinterpret_cast<const uint8_t*>(static_cast<uintptr_t>(window + offset));
    }
    /// The same byte with the cache neither asked nor filled: what the
    /// chip holds, not what the cache remembers.
    static const uint8_t* uncached_address(uint32_t offset) {
        return reinterpret_cast<const uint8_t*>(static_cast<uintptr_t>(xip_nocache_base + offset));
    }
    /// The same byte with the QMI's address translation bypassed too:
    /// the PHYSICAL byte, whatever the ATRANS panes say.
    static const uint8_t* untranslated_address(uint32_t offset) {
        return reinterpret_cast<const uint8_t*>(static_cast<uintptr_t>(xip_untranslated_base + offset));
    }

    /// `dst` filled from the chip through the cache; anything past the
    /// chip's end reads as erased.
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        if (addr >= size_bytes) {
            memset(dst.data(), 0xFF, dst.size());
            return;
        }
        const uint32_t n = dst.size() <= size_bytes - addr ? static_cast<uint32_t>(dst.size())
                                                           : size_bytes - addr;
        memcpy(dst.data(), address(addr), n);
        if (n < dst.size()) {
            memset(dst.data() + n, 0xFF, dst.size() - n);
        }
    }

    /**
     * Whole sectors from `addr`, `bytes` long. With
     * `FlashEraseGrain::sector_or_block` the ROM takes the 64 kB D8h
     * command wherever the run is aligned to it and long enough for it,
     * which is much faster and assumes the chip has that command; the
     * default asks for 20h and nothing else.
     *
     * THE WINDOW IS OPEN FOR THE WHOLE RUN - one call, one window,
     * however many sectors - so bound it. The chapter does not say what
     * a sector erase costs; the suite measures it.
     */
    static bool erase(uint32_t addr, uint32_t bytes, FlashEraseGrain grain = FlashEraseGrain::sector) {
        if (bytes == 0u || (addr % sector_size) != 0u || (bytes % sector_size) != 0u ||
            addr >= size_bytes || bytes > size_bytes - addr) {
            return false;
        }
        if (!init()) {
            return false;
        }
        commit_writes();
        const uint32_t block = grain == FlashEraseGrain::sector_or_block ? block_size : no_block;
        detail::FlashRom::erase_in_ram(addr, bytes, block, block_command);
        return true;
    }
    /// One sector, the smallest erase this chip's flash has.
    static bool erase_sector(uint32_t addr) { return erase(addr, sector_size); }

    /**
     * Whole pages from `addr`, `src` in SRAM. A source in the XIP space
     * is refused: the ROM would read it through a memory interface that
     * is disconnected for the duration.
     *
     * A page programs ones to zeros only; a page programmed twice
     * between erases holds the AND of the two, which is the physics the
     * chapter does not state and the suite measures.
     */
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        if (src.empty() || (addr % page_size) != 0u || (src.size() % page_size) != 0u ||
            addr >= size_bytes || src.size() > size_bytes - addr || in_window(src.data())) {
            return false;
        }
        if (!init()) {
            return false;
        }
        commit_writes();
        detail::FlashRom::program_in_ram(addr, src.data(), static_cast<uint32_t>(src.size()));
        return true;
    }

    /**
     * A raw command on chip select 0: `opcode`, then `head` (the address
     * and dummy bytes the command wants), then as many bytes clocked in
     * as `rx` is long. The opcode and the head are sent with their
     * answers discarded, so `rx` begins at the first byte of the reply.
     *
     * `opcode` MUST BE A READ (`flash_command_reads_only`), and this is
     * the run-time face of that rule; `command<opcode>()` below is the
     * compile-time one. `head` is copied to the stack, so it may live
     * anywhere, and is capped at `max_command_head`; `rx` may be any
     * length but must not lie in the XIP space.
     */
    static bool command(uint8_t opcode, std::span<const uint8_t> head, std::span<uint8_t> rx) {
        if (!flash_command_reads_only(opcode) || head.size() >= max_command_head ||
            in_window(rx.data())) {
            return false;
        }
        if (!init()) {
            return false;
        }
        commit_writes();
        uint8_t out[max_command_head];
        out[0] = opcode;
        for (uint32_t i = 0; i < head.size(); ++i) {
            out[i + 1u] = head[i];
        }
        detail::FlashRom::command_in_ram(out, static_cast<uint32_t>(head.size()) + 1u, rx.data(),
                                         static_cast<uint32_t>(rx.size()));
        return true;
    }

    /// The same, with the opcode as a template argument: an opcode that
    /// is not one of `FlashCommand`'s reads is a COMPILE error here.
    template <uint8_t opcode>
    static bool command(std::span<const uint8_t> head, std::span<uint8_t> rx) {
        static_assert(flash_command_reads_only(opcode),
                      "brio RP2350 flash: the raw command verb carries READS only - no write enable, "
                      "no status or configuration register write, no erase or program opcode");
        return command(opcode, head, rx);
    }

    /// 9Fh: manufacturer, memory type, capacity.
    static std::optional<FlashJedecId> jedec_id() {
        uint8_t rx[3] = {};
        if (!command<FlashCommand::read_jedec_id>({}, rx)) {
            return std::nullopt;
        }
        return FlashJedecId{.manufacturer = rx[0], .type = rx[1], .capacity = rx[2]};
    }

    /// 4Bh: the chip's 64-bit unique id, after four dummy bytes.
    static std::optional<std::array<uint8_t, 8>> unique_id() {
        const uint8_t head[4] = {0, 0, 0, 0};
        std::array<uint8_t, 8> id{};
        if (!command<FlashCommand::read_unique_id>(head, id)) {
            return std::nullopt;
        }
        return id;
    }

    /// 05h / 35h / 15h: the three status registers, READ. `which` is
    /// 1, 2 or 3; anything else is refused at compile time.
    template <uint8_t which>
    static std::optional<uint8_t> status_register() {
        static_assert(which >= 1 && which <= 3, "brio RP2350 flash: status registers 1, 2 and 3");
        constexpr uint8_t opcode = which == 1   ? FlashCommand::read_status1
                                   : which == 2 ? FlashCommand::read_status2
                                                : FlashCommand::read_status3;
        uint8_t rx[1] = {};
        if (!command<opcode>({}, rx)) {
            return std::nullopt;
        }
        return rx[0];
    }

    /// 5Ah: the JEDEC Serial Flash Discoverable Parameters at `addr`,
    /// after the 24-bit address and one dummy byte. The chip's own
    /// account of its geometry, which is why it is worth reading beside
    /// the build's number.
    static bool read_sfdp(uint32_t addr, std::span<uint8_t> dst) {
        const uint8_t head[4] = {static_cast<uint8_t>(addr >> 16), static_cast<uint8_t>(addr >> 8),
                                 static_cast<uint8_t>(addr), 0};
        return command<FlashCommand::read_sfdp>(head, dst);
    }
};

} // namespace brio
