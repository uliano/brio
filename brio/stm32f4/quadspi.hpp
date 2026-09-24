/*
 * quadspi.hpp
 *
 * The STM32F4's Quad-SPI memory interface (RM0386 ch. 13, RM0390 ch. 12 -
 * one register file under one name on the F412, the F413/F423, the F446
 * and the F469/F479, which the device header says with QSPI_R_BASE; the
 * rest of the pack has no block and nothing register-facing in this file
 * exists there): `Quadspi`, a MONOSTATE resource, because a part carries
 * one interface or none.
 *
 *   using namespace brio;
 *   constexpr QspiConfig cfg{.prescaler = 1, .fifo_threshold = 4,
 *                            .address_bits = 24, .cs_high_cycles = 5};
 *   Quadspi::init<cfg>();                          // HCLK/2, a 16 MB device
 *   constexpr QspiCommand read_id{.instruction = 0x9E, .data_lines = QspiLines::one};
 *   uint8_t id[20];
 *   Quadspi::read(read_id, 0, id, sizeof id);      // an indirect read
 *   Quadspi::map(fast_read_quad);                  // then plain loads
 *   const uint8_t b = Quadspi::window()[0x1000];
 *
 * WHAT THE BLOCK IS. A command engine in front of a serial flash: every
 * command is up to five phases - an instruction, an address of one to
 * four bytes, one to four alternate bytes, a run of dummy cycles and the
 * data - each phase on one, two or four lines or skipped (13.3.3), and
 * the whole thing in one of three functional modes: INDIRECT, where the
 * program writes the registers and pumps a 32-byte FIFO; AUTOMATIC
 * STATUS-POLLING, where the block repeats a read on its own and raises a
 * flag when the masked answer matches; and MEMORY-MAPPED, where a load
 * from the window at 0x9000 0000 becomes the programmed read command and
 * the flash is seen as memory (13.3.5, 13.3.6, 13.3.7). Only reads go
 * through the window; a write is always indirect.
 *
 * THE FACTS THAT SHAPE THE FILE.
 *
 * 1. A COMMAND STARTS ON THE LAST REGISTER IT NEEDS (13.3.5): on the CCR
 *    write when there is no address and no data to give, on the AR write
 *    when an address is needed and nothing is to be sent, on the first DR
 *    write when data is. The verbs below write the registers in exactly
 *    that order, ABR before CCR because the alternate bytes never start
 *    anything and must be there first, and DLR before CCR because the
 *    length is part of what the command is.
 * 2. THE CLOCK IS HCLK DIVIDED, NOT A CLOCK OF THE RCC: PRESCALER makes
 *    CLK = HCLK / (PRESCALER + 1) (13.5.1), so a device's ceiling is met by
 *    a divisor a program chooses knowing HCLK - qspi_prescaler_for() is
 *    that arithmetic. An odd divisor gives a clock that is low one cycle
 *    longer than high.
 * 3. THE FLASH'S SIZE IS PART OF THE CONTRACT. FSIZE + 1 is the number of
 *    address bits (13.3.8), and an indirect access whose address or whose
 *    address plus length passes the size raises TEF (13.3.13); in
 *    memory-mapped mode the same mistake is a BUS ERROR to the master
 *    that made it. `QspiConfig::address_bits` is that field in the
 *    manual's own unit.
 * 4. NCS'S HIGH TIME IS A DEVICE PARAMETER. CSHT + 1 cycles between two
 *    commands (13.3.8); a flash wants a deselect time in nanoseconds after
 *    a command, and qspi_cs_high_cycles_for() turns it into the field at
 *    the clock in force - refused when eight cycles are not enough.
 * 5. THE MEMORY-MAPPED WINDOW IS THE MANUAL'S AND NOT THE HEADER'S. No
 *    header of the pack declares it; the reserve states 0x9000 0000 for
 *    the two classes whose manuals are on the desk and `map()` REFUSES on
 *    the others rather than guess (device_tables.hpp's quadspi_window).
 * 6. BUSY MEANS DIFFERENT THINGS PER MODE (13.3.14): in indirect mode it
 *    falls when the command is done and the FIFO empty; in status-polling
 *    mode it stays up between the periodic reads until a match with APMS
 *    or an abort; in memory-mapped mode it stays up from the first access,
 *    prefetching, until an abort. `abort()` is the one way down that works
 *    in every mode, and it flushes the FIFO.
 *
 * THE ERRATA ARE THE SAME FIVE ON BOTH SHEETS READ - ES0321 Rev 14
 * 2.4.1..2.4.5 (the F469/F479) and ES0298 Rev 8 2.4.1..2.4.5 (the F446) -
 * and four of them are code here:
 *  - 2.4.2, "first nibble of data not written after dummy phase": an
 *    indirect WRITE with dummy cycles loses its first nibble. write()
 *    REFUSES a command with dummy cycles; a device that wants latency
 *    before the data of a write gets it as alternate bytes, the sheet's
 *    own workaround.
 *  - 2.4.3, "wrong data from memory-mapped read after an indirect mode
 *    operation" when AR's two low bits are set on entry: map() clears AR
 *    and aborts before it writes the memory-mapped command, every time.
 *  - 2.4.4, "memory-mapped read operations may fail when timeout counter
 *    is enabled": TCEN is never set - there is no timeout option in
 *    QspiConfig at all - and NCS is raised by unmap()'s abort, the sheet's
 *    workaround.
 *  - 2.4.1, "extra data written in the FIFO at the end of a read
 *    transfer" (quad, DDR, prescaler 1): read() drains whatever the FIFO
 *    still holds after the transfer completes and discards it, which
 *    costs nothing on the transfers the item does not touch.
 *  - 2.4.5, "memory-mapped access in indirect mode clearing QUADSPI_AR":
 *    no code can cover a load a PROGRAM makes from the window while the
 *    block is in indirect mode; window() is handed out by map() and the
 *    verbs write AR right before every command they start, so a program
 *    that keeps to the verbs never meets it.
 *
 * NOT HERE, BY DESIGN: the flash's own command set. This file speaks the
 * chapter's language - phases, lines, sizes - and a device's opcodes,
 * dummy counts and registers are the device's datasheet's, spelled as
 * `QspiCommand` values where the device is (the bench suite for the
 * MT25QL128 the 32F469IDISCOVERY carries). No FlashMedia stands on this
 * block: the NV stack's review (docs/design/nv-heap.md) comes first.
 */

#pragma once

#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"

namespace brio {

// =============================================================================
// The vocabulary - on every part, block or no block
// =============================================================================

/// How many lines a phase uses: IMODE/ADMODE/ABMODE/DMODE's codes (13.5.6).
enum class QspiLines : uint8_t { none = 0, one = 1, two = 2, four = 3 };

/// ADSIZE and ABSIZE: the phase's width in bytes, coded (13.5.6).
enum class QspiWidth : uint8_t { bits8 = 0, bits16 = 1, bits24 = 2, bits32 = 3 };

/**
 * One command as the chapter draws it (13.3.3): the instruction and its
 * lines, the address phase, the alternate bytes, the dummy cycles, the
 * data phase. A phase whose lines are `none` is skipped. The functional
 * mode - indirect read or write, polling, memory-mapped - is the VERB's
 * and not the command's, which is what lets one read command serve an
 * indirect read and the window alike.
 */
struct QspiCommand {
    uint8_t instruction = 0;
    QspiLines instruction_lines = QspiLines::one;
    QspiLines address_lines = QspiLines::none;
    QspiWidth address_width = QspiWidth::bits24;
    QspiLines alternate_lines = QspiLines::none;
    QspiWidth alternate_width = QspiWidth::bits8;
    uint32_t alternate = 0;
    uint8_t dummy_cycles = 0;     ///< 0..31 CLK cycles (DCYC)
    QspiLines data_lines = QspiLines::none;
    bool ddr = false;             ///< DDRM: address, alternate and data on both edges
    bool ddr_hold = false;        ///< DHHC: the DDR output delayed a quarter cycle
    bool instruction_once = false;   ///< SIOO (13.3.12)
};

/// The chapter's rules on a command: at most 31 dummy cycles, and at
/// least one of the instruction, address, alternate and data phases
/// (13.3.3).
constexpr bool qspi_command_ok(const QspiCommand& c) {
    if (c.dummy_cycles > 31u) {
        return false;
    }
    return c.instruction_lines != QspiLines::none || c.address_lines != QspiLines::none ||
           c.alternate_lines != QspiLines::none || c.data_lines != QspiLines::none;
}

/// Whether a command RECEIVES data on more than one line, which 13.3.3
/// says needs at least one dummy cycle for the turn-around.
constexpr bool qspi_read_needs_turnaround(const QspiCommand& c) {
    return (c.data_lines == QspiLines::two || c.data_lines == QspiLines::four) &&
           c.dummy_cycles == 0u;
}

/**
 * The block's configuration (13.3.10): the prescaler, the sampling shift,
 * the FIFO threshold, and the device's three parameters of DCR - its size
 * as a number of address bits, the chip-select high time in cycles, the
 * idle level of CLK.
 */
struct QspiConfig {
    uint8_t prescaler = 1;         ///< CLK = HCLK / (prescaler + 1)
    bool sample_shift = false;     ///< SSHIFT: sample half a cycle later (never with DDR)
    uint8_t fifo_threshold = 4;    ///< FTHRES + 1: 1..32 bytes
    uint8_t address_bits = 24;     ///< FSIZE + 1: the flash holds 2^address_bits bytes, 1..32
    uint8_t cs_high_cycles = 1;    ///< CSHT + 1: NCS high between commands, 1..8 cycles
    bool clock_mode3 = false;      ///< CKMODE: CLK high while NCS is high
};

constexpr bool qspi_config_ok(const QspiConfig& c) {
    return c.fifo_threshold >= 1u && c.fifo_threshold <= 32u && c.address_bits >= 1u &&
           c.address_bits <= 32u && c.cs_high_cycles >= 1u && c.cs_high_cycles <= 8u;
}

/// CLK for a prescaler code at a given HCLK (13.5.1).
constexpr uint32_t qspi_clock_hz(uint32_t hclk_hz, uint8_t prescaler) {
    return hclk_hz / (static_cast<uint32_t>(prescaler) + 1u);
}

/// The smallest prescaler code whose CLK does not exceed `max_hz`: the
/// fastest legal clock for a device with that ceiling. 255 when even
/// HCLK/256 is too fast - a device that slow is not this block's.
constexpr uint8_t qspi_prescaler_for(uint32_t hclk_hz, uint32_t max_hz) {
    if (max_hz == 0u) {
        return 255u;
    }
    for (uint32_t p = 0; p < 256u; ++p) {
        if (hclk_hz / (p + 1u) <= max_hz) {
            return static_cast<uint8_t>(p);
        }
    }
    return 255u;
}

/// The CSHT + 1 cycles that cover `ns` of deselect time at `clk_hz`,
/// rounded up; 0 when more than the field's eight are needed.
constexpr uint8_t qspi_cs_high_cycles_for(uint32_t clk_hz, uint32_t ns) {
    const uint64_t cycles = (static_cast<uint64_t>(ns) * clk_hz + 999'999'999u) / 1'000'000'000u;
    if (cycles > 8u) {
        return 0u;
    }
    return cycles == 0u ? 1u : static_cast<uint8_t>(cycles);
}

/// The address bits of a flash of `bytes` (a power of two): FSIZE + 1.
/// 0 when `bytes` is not a power of two or is not addressable.
constexpr uint8_t qspi_address_bits(uint64_t bytes) {
    if (bytes == 0u || (bytes & (bytes - 1u)) != 0u) {
        return 0u;
    }
    uint8_t bits = 0;
    while ((1ull << bits) < bytes) {
        ++bits;
    }
    return bits > 32u ? 0u : bits;
}

/// What an indirect verb reports.
enum class QspiStatus : uint8_t {
    ok,
    refused,   ///< the command or the arguments break a rule above; nothing was written
    error,     ///< TEF: the address or address + length passed the flash size
    timeout,   ///< the bounded wait ran out with BUSY still up
};

/// The five events of table 93, as a mask over SR's flag bits.
struct QspiEvents {
    bool timeout = false;
    bool status_match = false;
    bool fifo_threshold = false;
    bool transfer_complete = false;
    bool transfer_error = false;
};

#if defined(QSPI_R_BASE)

// =============================================================================
// The block
// =============================================================================

struct Quadspi {
    Quadspi() = delete;

    static QUADSPI_TypeDef& regs() { return *QUADSPI; }

    static constexpr IRQn_Type irq = QUADSPI_IRQn;

    /// The memory-mapped window, where the manual gives it.
    static constexpr QuadspiWindow window_facts = quadspi_window();

    /// A bounded wait's default: generous, because a status-polling wait
    /// for an erase is a device's hundreds of milliseconds and not the
    /// block's microseconds.
    static constexpr uint32_t default_spins = 40'000'000u;

    // ---- the clock gate and the reset line (AHB3) --------------------------------

    static void clock(bool on) { Rcc::ahb3_clock(quadspi_clock_mask(), on); }
    static bool clock() { return Rcc::ahb3_clock(quadspi_clock_mask()); }
    /// RCC_AHB3RSTR.QSPIRST: every register back to zero, an operation in
    /// flight abandoned on the block's side.
    static void reset_block() { Rcc::ahb3_reset(quadspi_reset_mask()); }

    // ---- bring-up ---------------------------------------------------------------

    /// The two-phase configuration of 13.3.10 in one verb: the gate opened,
    /// an operation in flight aborted, the block disabled, CR (prescaler,
    /// sample shift, FIFO threshold) and DCR (size, chip-select high time,
    /// clock mode) written, the block enabled. False and nothing written
    /// when the configuration breaks qspi_config_ok().
    template <QspiConfig cfg>
    static bool init() {
        static_assert(qspi_config_ok(cfg),
                      "brio Quadspi: the FIFO threshold is 1..32 bytes, the flash 1..32 address "
                      "bits, the chip-select high time 1..8 cycles (RM0386 13.5.1, 13.5.2)");
        return init(cfg);
    }

    static bool init(const QspiConfig& cfg) {
        if (!qspi_config_ok(cfg)) {
            return false;
        }
        clock(true);
        if (busy()) {
            (void)abort();
        }
        regs().CR &= ~QUADSPI_CR_EN;
        regs().CR = (static_cast<uint32_t>(cfg.prescaler) << QUADSPI_CR_PRESCALER_Pos) |
                    (cfg.sample_shift ? QUADSPI_CR_SSHIFT : 0u) |
                    (static_cast<uint32_t>(cfg.fifo_threshold - 1u) << QUADSPI_CR_FTHRES_Pos);
        regs().DCR = (static_cast<uint32_t>(cfg.address_bits - 1u) << QUADSPI_DCR_FSIZE_Pos) |
                     (static_cast<uint32_t>(cfg.cs_high_cycles - 1u) << QUADSPI_DCR_CSHT_Pos) |
                     (cfg.clock_mode3 ? QUADSPI_DCR_CKMODE : 0u);
        regs().FCR = QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;
        regs().CR |= QUADSPI_CR_EN;
        return true;
    }

    /// The block disabled and its gate closed; an operation in flight is
    /// aborted first. The pads are the program's.
    static void release() {
        if (clock()) {
            (void)abort();
            regs().CR &= ~QUADSPI_CR_EN;
            clock(false);
        }
    }

    static bool enabled() { return (regs().CR & QUADSPI_CR_EN) != 0u; }

    /// What CR holds as the block's configuration, read back.
    static uint8_t prescaler() {
        return static_cast<uint8_t>((regs().CR & QUADSPI_CR_PRESCALER_Msk) >> QUADSPI_CR_PRESCALER_Pos);
    }
    static uint8_t address_bits() {
        return static_cast<uint8_t>(((regs().DCR & QUADSPI_DCR_FSIZE_Msk) >> QUADSPI_DCR_FSIZE_Pos) + 1u);
    }
    static uint8_t cs_high_cycles() {
        return static_cast<uint8_t>(((regs().DCR & QUADSPI_DCR_CSHT_Msk) >> QUADSPI_DCR_CSHT_Pos) + 1u);
    }
    static uint8_t fifo_threshold() {
        return static_cast<uint8_t>(((regs().CR & QUADSPI_CR_FTHRES_Msk) >> QUADSPI_CR_FTHRES_Pos) + 1u);
    }

    // ---- the status register (13.5.3) --------------------------------------------

    static bool busy() { return (regs().SR & QUADSPI_SR_BUSY) != 0u; }
    /// FLEVEL: the bytes the FIFO holds, 0..32.
    static uint8_t fifo_level() {
        return static_cast<uint8_t>((regs().SR & QUADSPI_SR_FLEVEL_Msk) >> QUADSPI_SR_FLEVEL_Pos);
    }
    static bool transfer_complete() { return (regs().SR & QUADSPI_SR_TCF) != 0u; }
    static bool transfer_error() { return (regs().SR & QUADSPI_SR_TEF) != 0u; }
    static bool status_match() { return (regs().SR & QUADSPI_SR_SMF) != 0u; }
    static bool fifo_threshold_reached() { return (regs().SR & QUADSPI_SR_FTF) != 0u; }
    static bool timed_out() { return (regs().SR & QUADSPI_SR_TOF) != 0u; }

    /// The four flags that clear by a write of one to FCR (13.5.4); FTF
    /// clears itself when the threshold condition ends.
    static void clear_flags() {
        regs().FCR = QUADSPI_FCR_CTOF | QUADSPI_FCR_CSMF | QUADSPI_FCR_CTCF | QUADSPI_FCR_CTEF;
    }

    /// ABORT (13.3.14): the operation in flight stopped, BUSY and ABORT
    /// back to zero, the FIFO flushed. True when the block came to rest
    /// inside the bound.
    static bool abort(uint32_t spins = default_spins) {
        regs().CR |= QUADSPI_CR_ABORT;
        for (; spins != 0u; --spins) {
            if ((regs().CR & QUADSPI_CR_ABORT) == 0u && !busy()) {
                clear_flags();
                return true;
            }
        }
        return false;
    }

    // ---- the interrupt (13.4) ----------------------------------------------------

    /// The five enables of CR, set together.
    static void interrupts(const QspiEvents& e) {
        uint32_t v = regs().CR & ~(QUADSPI_CR_TOIE | QUADSPI_CR_SMIE | QUADSPI_CR_FTIE |
                                   QUADSPI_CR_TCIE | QUADSPI_CR_TEIE);
        if (e.timeout) v |= QUADSPI_CR_TOIE;
        if (e.status_match) v |= QUADSPI_CR_SMIE;
        if (e.fifo_threshold) v |= QUADSPI_CR_FTIE;
        if (e.transfer_complete) v |= QUADSPI_CR_TCIE;
        if (e.transfer_error) v |= QUADSPI_CR_TEIE;
        regs().CR = v;
    }

    /// The ISR body: which of the five events stand, the four that are
    /// flags cleared on the way out. The FIFO threshold is left as it is -
    /// it clears when the FIFO is served, which is the handler's business.
    [[gnu::always_inline]] static QspiEvents isr() {
        const uint32_t sr = regs().SR;
        QspiEvents e{};
        e.timeout = (sr & QUADSPI_SR_TOF) != 0u;
        e.status_match = (sr & QUADSPI_SR_SMF) != 0u;
        e.fifo_threshold = (sr & QUADSPI_SR_FTF) != 0u;
        e.transfer_complete = (sr & QUADSPI_SR_TCF) != 0u;
        e.transfer_error = (sr & QUADSPI_SR_TEF) != 0u;
        regs().FCR = sr & (QUADSPI_SR_TOF | QUADSPI_SR_SMF | QUADSPI_SR_TCF | QUADSPI_SR_TEF);
        return e;
    }

    // ---- the command word (13.5.6) -----------------------------------------------

    /// CCR from a command and a functional mode: 0 indirect write, 1
    /// indirect read, 2 automatic status polling, 3 memory-mapped. A
    /// skipped phase leaves its width field zero too, so the word says
    /// exactly what the command has.
    static constexpr uint32_t ccr_word(const QspiCommand& c, uint8_t fmode) {
        const bool has_address = c.address_lines != QspiLines::none;
        const bool has_alternate = c.alternate_lines != QspiLines::none;
        return (c.ddr ? QUADSPI_CCR_DDRM : 0u) | (c.ddr_hold ? QUADSPI_CCR_DHHC : 0u) |
               (c.instruction_once ? QUADSPI_CCR_SIOO : 0u) |
               (static_cast<uint32_t>(fmode & 3u) << QUADSPI_CCR_FMODE_Pos) |
               (static_cast<uint32_t>(c.data_lines) << QUADSPI_CCR_DMODE_Pos) |
               (static_cast<uint32_t>(c.dummy_cycles & 31u) << QUADSPI_CCR_DCYC_Pos) |
               (has_alternate ? static_cast<uint32_t>(c.alternate_width) << QUADSPI_CCR_ABSIZE_Pos : 0u) |
               (static_cast<uint32_t>(c.alternate_lines) << QUADSPI_CCR_ABMODE_Pos) |
               (has_address ? static_cast<uint32_t>(c.address_width) << QUADSPI_CCR_ADSIZE_Pos : 0u) |
               (static_cast<uint32_t>(c.address_lines) << QUADSPI_CCR_ADMODE_Pos) |
               (static_cast<uint32_t>(c.instruction_lines) << QUADSPI_CCR_IMODE_Pos) |
               static_cast<uint32_t>(c.instruction);
    }

    // ---- indirect mode (13.3.5) ---------------------------------------------------

    /**
     * A command with no data phase - WRITE ENABLE, an erase with its
     * address, a reset - started and waited for. The address phase is
     * present when the command's `address_lines` say so; `address` is
     * ignored otherwise. Refused (nothing written) when the command breaks
     * the chapter's rules or names a data phase.
     */
    static QspiStatus command(const QspiCommand& c, uint32_t address = 0,
                              uint32_t spins = default_spins) {
        if (!qspi_command_ok(c) || c.data_lines != QspiLines::none || !ready(spins)) {
            return QspiStatus::refused;
        }
        clear_flags();
        regs().ABR = c.alternate;
        regs().CCR = ccr_word(c, 0);   // starts here when there is no address
        if (c.address_lines != QspiLines::none) {
            regs().AR = address;       // ... or here
        }
        return finish(spins);
    }

    /**
     * An indirect WRITE of `len` bytes: the command's phases, then the data
     * pumped through the FIFO. Refused with dummy cycles (ES0321 2.4.2 /
     * ES0298 2.4.2: the first nibble is lost - alternate bytes are the
     * latency a write may have), with no data phase, with a zero length, or
     * with the block busy.
     */
    static QspiStatus write(const QspiCommand& c, uint32_t address, const uint8_t* data,
                            uint32_t len, uint32_t spins = default_spins) {
        if (!qspi_command_ok(c) || c.data_lines == QspiLines::none || c.dummy_cycles != 0u ||
            len == 0u || data == nullptr || !ready(spins)) {
            return QspiStatus::refused;
        }
        clear_flags();
        regs().DLR = len - 1u;
        regs().ABR = c.alternate;
        regs().CCR = ccr_word(c, 0);
        if (c.address_lines != QspiLines::none) {
            regs().AR = address;
        }
        // The first DR write is what starts the command (13.3.5); the FIFO
        // is 32 bytes and a store into a full one stalls the bus, so the
        // level is watched instead.
        volatile uint8_t& dr8 = *reinterpret_cast<volatile uint8_t*>(&regs().DR);
        for (uint32_t i = 0; i < len; ++i) {
            uint32_t guard = spins;
            while (fifo_level() >= 32u) {
                if (transfer_error()) {
                    return fail(QspiStatus::error);
                }
                if (--guard == 0u) {
                    return fail(QspiStatus::timeout);
                }
            }
            dr8 = data[i];
        }
        return finish(spins);
    }

    /**
     * An indirect READ of `len` bytes into `data`. A read on two or four
     * lines needs a dummy cycle for the turn-around (13.3.3) and is refused
     * without one. Whatever the FIFO still holds after the transfer is
     * drained and discarded (ES0321 2.4.1's extra byte, when it comes).
     */
    static QspiStatus read(const QspiCommand& c, uint32_t address, uint8_t* data, uint32_t len,
                           uint32_t spins = default_spins) {
        if (!qspi_command_ok(c) || c.data_lines == QspiLines::none ||
            qspi_read_needs_turnaround(c) || len == 0u || data == nullptr || !ready(spins)) {
            return QspiStatus::refused;
        }
        clear_flags();
        regs().DLR = len - 1u;
        regs().ABR = c.alternate;
        regs().CCR = ccr_word(c, 1);   // starts here when there is no address
        if (c.address_lines != QspiLines::none) {
            regs().AR = address;       // ... or here
        }
        // The pump takes whatever the FIFO holds in one go - the level is
        // read once per burst, not once per byte - and in words while four
        // bytes are there, which is what lets a four-line read at 90 MHz
        // (45 Mbyte/s) be the wire's pace and not the loop's.
        volatile uint8_t& dr8 = *reinterpret_cast<volatile uint8_t*>(&regs().DR);
        uint32_t i = 0;
        while (i < len) {
            uint32_t level = fifo_level();
            if (level == 0u) {
                uint32_t guard = spins;
                do {
                    if (transfer_error()) {
                        return fail(QspiStatus::error);
                    }
                    if (!busy() && fifo_level() == 0u) {
                        // The block says it is done and holds nothing
                        // more: the read ends short, which the caller
                        // learns as a timeout rather than as silently
                        // short data.
                        return fail(QspiStatus::timeout);
                    }
                    if (--guard == 0u) {
                        return fail(QspiStatus::timeout);
                    }
                    level = fifo_level();
                } while (level == 0u);
            }
            uint32_t take = len - i < level ? len - i : level;
            while (take >= 4u) {
                const uint32_t w = regs().DR;
                data[i] = static_cast<uint8_t>(w);
                data[i + 1u] = static_cast<uint8_t>(w >> 8);
                data[i + 2u] = static_cast<uint8_t>(w >> 16);
                data[i + 3u] = static_cast<uint8_t>(w >> 24);
                i += 4u;
                take -= 4u;
            }
            while (take != 0u) {
                data[i] = dr8;
                ++i;
                --take;
            }
        }
        const QspiStatus st = finish(spins);
        while (fifo_level() != 0u) {
            [[maybe_unused]] const uint8_t extra = dr8;   // 2.4.1's extra byte, or nothing
        }
        return st;
    }

    // ---- automatic status-polling mode (13.3.6) ---------------------------------

    /**
     * The block reads `bytes` (1..4) of status with `c` every `interval`
     * CLK cycles and stops on the first match (APMS): with `match_any` the
     * OR rule, otherwise AND, over the bits `mask` selects. `status` gets
     * the last bytes read. Refused when the command has no data phase or
     * asks more than four bytes; a timeout leaves the block aborted.
     */
    static QspiStatus poll(const QspiCommand& c, uint32_t address, uint8_t bytes, uint32_t mask,
                           uint32_t match, uint16_t interval, bool match_any, uint32_t& status,
                           uint32_t spins = default_spins) {
        if (!qspi_command_ok(c) || c.data_lines == QspiLines::none || bytes == 0u ||
            bytes > 4u || !ready(spins)) {
            return QspiStatus::refused;
        }
        clear_flags();
        regs().PSMKR = mask;
        regs().PSMAR = match;
        regs().PIR = interval;
        regs().DLR = bytes - 1u;
        uint32_t cr = regs().CR | QUADSPI_CR_APMS;
        cr = match_any ? (cr | QUADSPI_CR_PMM) : (cr & ~QUADSPI_CR_PMM);
        regs().CR = cr;
        regs().ABR = c.alternate;
        regs().CCR = ccr_word(c, 2);
        if (c.address_lines != QspiLines::none) {
            regs().AR = address;
        }
        for (; spins != 0u; --spins) {
            if (status_match()) {
                status = regs().DR;
                uint32_t guard = default_spins;
                while (busy() && --guard != 0u) {
                }
                clear_flags();
                return busy() ? fail(QspiStatus::timeout) : QspiStatus::ok;
            }
            if (transfer_error()) {
                return fail(QspiStatus::error);
            }
        }
        status = regs().DR;
        return fail(QspiStatus::timeout);
    }

    // ---- memory-mapped mode (13.3.7) ----------------------------------------------

    /**
     * The window opened over the flash with `c` as the read every load
     * becomes. Before it, ES0321 2.4.3's measure - AR cleared and an abort
     * issued, whatever the last indirect operation left - done so that it
     * TAKES: an abort first, because a write to AR is ignored while BUSY
     * stands (13.5.7), then the clear, which under an indirect read command
     * starts a read at address 0 (13.3.5), then the abort that stops that
     * read - the sheet's own two steps, with the one before them that makes
     * the first of them land. Refused where the manual on the desk does not
     * give this class's window address, and with a command that is not a
     * read.
     */
    static bool map(const QspiCommand& c, uint32_t spins = default_spins) {
        if (!window_facts.known || !qspi_command_ok(c) || c.data_lines == QspiLines::none ||
            qspi_read_needs_turnaround(c)) {
            return false;
        }
        if (!abort(spins)) {
            return false;
        }
        regs().AR = 0u;
        if (!abort(spins)) {
            return false;
        }
        regs().ABR = c.alternate;
        regs().CCR = ccr_word(c, 3);
        return true;
    }

    /// Whether CCR holds the memory-mapped mode.
    static bool mapped() {
        return ((regs().CCR & QUADSPI_CCR_FMODE_Msk) >> QUADSPI_CCR_FMODE_Pos) == 3u;
    }

    /// The window closed: the prefetch aborted, NCS raised (2.4.4's way of
    /// raising it, the timeout counter never being armed), the block left
    /// idle for the next indirect command.
    static bool unmap(uint32_t spins = default_spins) { return abort(spins); }

    /// The flash as memory - valid after map() answered true and until
    /// unmap(); a load through it before that is the bus error 13.3.13
    /// describes. Null where the window is not known.
    static const volatile uint8_t* window() {
        return window_facts.known ? reinterpret_cast<const volatile uint8_t*>(window_facts.base)
                                  : nullptr;
    }

private:
    /// The block idle - or an operation still running past the bound.
    static bool ready(uint32_t spins) {
        if (!enabled()) {
            return false;
        }
        for (; spins != 0u && busy(); --spins) {
        }
        return !busy();
    }

    /// The tail of every indirect command: BUSY down, which in indirect
    /// mode means the command sequence done and the FIFO empty (13.3.14),
    /// TEF read before the flags are cleared. BUSY and not TCF, on
    /// purpose: TCF is the event an interrupt handler reports and clears
    /// (isr()), and a poller that waited for it would be robbed by that
    /// handler and time out - measured: the wait ran out, the abort of the
    /// failure path set TCF anew, and the vector was entered twice for one
    /// read.
    static QspiStatus finish(uint32_t spins) {
        for (; spins != 0u; --spins) {
            if (transfer_error()) {
                return fail(QspiStatus::error);
            }
            if (!busy()) {
                clear_flags();
                return QspiStatus::ok;
            }
        }
        return fail(QspiStatus::timeout);
    }

    /// A command that did not end well is aborted so that the block is at
    /// rest for the next one, and the status handed back unchanged.
    static QspiStatus fail(QspiStatus st) {
        (void)abort();
        return st;
    }
};

#endif  // QSPI_R_BASE

}  // namespace brio
