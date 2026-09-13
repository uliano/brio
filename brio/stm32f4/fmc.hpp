/*
 * fmc.hpp
 *
 * The STM32F4's flexible memory controller (RM0090 ch. 37, RM0390
 * ch. 11) - the one peripheral of this family that does not talk to a
 * wire but ADDS ADDRESS SPACE: memory soldered outside the die, mapped
 * into the Cortex's own map and reached by a plain load or store.
 *
 *  Fmc                the block: the AHB3 gate and reset line, the one
 *                     vector every half shares, the six memory windows
 *                     of figure 457, and what this part's silicon has;
 *  FmcSdram<1|2>      the SDRAM controller of banks 5 and 6 - the
 *                     geometry and the seven timings in the chapter's
 *                     words, the command machine of 37.7.3 as one
 *                     verb, the refresh counter's arithmetic, self
 *                     refresh and power down, the status flags;
 *  FmcNorPsram<1..4>  the static controller of bank 1 - four sub-banks
 *                     with a chip select each, both timing registers
 *                     and the extended mode, every field of BCR/BTR/
 *                     BWTR;
 *  FmcSram<n>,        the two tasks over it: an asynchronous SRAM and a
 *  FmcNor<n>          NOR flash, each pinning the memory type and
 *                     leaving only what that device needs.
 *
 *   using Sdram = brio::FmcSdram<2>;                  // SDNE1/SDCKE1
 *   Sdram::initialize(clock, geometry, timing, boot);
 *   volatile uint16_t* pixels = Sdram::at<uint16_t>(0);
 *
 * TWO BLOCKS WEAR THIS CHAPTER, AND THEY ARE NOT THE SAME PERIPHERAL.
 * The F405 class, the F412 and the F413/F423 carry the FSMC: the static
 * half alone, under register names of its own and with no SDRAM
 * controller anywhere. The F42x/F43x, the F446 and the F469/F479 carry
 * the FMC: that register set plus the SDRAM banks and a few fields the
 * FSMC has not. This file is the FMC's, and on an FSMC part nothing in
 * it exists at all: `fsmc_present()` in stm32f4/device_tables.hpp is
 * what a program asks to learn which block the part really has, so that
 * an absent FMC never reads as "no external memory". The rest of the
 * pack - the F401, the F410, the F411 and the 48-pin F412 - has
 * neither.
 *
 * WHAT AN EXTERNAL MEMORY COSTS THAT AN INTERNAL ONE DOES NOT. Three
 * things shape every verb below:
 *
 * 1. THE WINDOW IS LIVE BEFORE THE MEMORY IS. The address decoder
 *    answers as soon as the bank is enabled, and the AHB has no way to
 *    say "there is nothing there". A read of an SDRAM bank whose
 *    controller has not run its initialization sequence HANGS THE
 *    SYSTEM WITH NO FAULT (ES0206 2.3.3) - not a bus error, not a hard
 *    fault, a stopped machine. That is why initialize() is one verb
 *    that either completes the whole of 37.7.3 or returns false, and
 *    why nothing here hands out a pointer before it.
 * 2. THE CONTROLLER IS SHARED AND SERIAL. One access at a time, one set
 *    of address and data pads for every bank; and on this silicon the
 *    dynamic and static halves cannot even be used in turn without
 *    care (ES0206 2.3.7: put the SDRAM in self refresh before touching
 *    a static bank, and issue a Normal command on the way back).
 * 3. THE CPU IS NOT THE ONLY MASTER, AND THE ERRATA KNOW IT. ES0206
 *    2.3.5: with the FMC holding stack, heap or variable data, an
 *    interrupt taken during a CPU read may corrupt the data or fault;
 *    reads by a DMA are not affected. 2.3.4 and 2.3.15 are the two
 *    faces of the read FIFO - one says ENABLE RBURST (a 16- or 8-bit
 *    burst read crossing a row boundary reads the next row wrong with
 *    the FIFO off), the other says the FIFO ON can serve a stale line
 *    when a burst read of one internal bank is interrupted by a read of
 *    another. They do not contradict each other so much as bracket the
 *    same race, and the driver's answer is the manual's default: the
 *    FIFO ON (SdramConfig::read_burst), because 2.3.4 has no other
 *    workaround while 2.3.15 has three.
 *
 * THE SDRAM CLOCK IS NOT A CLOCK OF THE RCC. It is HCLK divided by two
 * or three inside this block (SDCR1.SDCLK), so a program that changes
 * HCLK changes the memory's clock and every timing programmed in
 * cycles under it. This driver is deliberately NOT a ClockUser: the
 * refresh rate and the seven SDTR fields are the DEVICE's nanoseconds
 * expressed in cycles, and rebasing them on a rate change would mean
 * re-running the initialization sequence, which is a program-wide
 * decision and not a fan-out. sdram_clock_hz() and
 * sdram_refresh_count() are the arithmetic a program does once, at the
 * rate it has chosen to stay at.
 *
 * WHAT THE PADS ARE IS NOT IN THIS FILE. Every FMC signal is AF12 on
 * this family, and which pad carries which line - and which of them a
 * BOARD actually wires - is a datasheet table and a schematic, not a
 * register fact: a program hands the pads over with stm32f4/pin.hpp
 * before it calls initialize(), the way it does for every other
 * peripheral, and the pin list belongs beside the board it describes.
 *
 * CONCURRENCY. configure(), timing() and the command verbs are
 * read-modify-writes of registers with no set/clear twins and are meant
 * for setup. command() and initialize() BUSY-WAIT on SDSR.BUSY with a
 * bounded spin and answer false on a time-out rather than hanging.
 * Once a bank is initialized the memory itself is ordinary storage: a
 * load or a store, from any context, at the cost the bench findings
 * name - subject to erratum 2.3.5 above.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/nvic.hpp"
#include "util/clock.hpp"

namespace brio {

// =============================================================================
// The memory windows (37.4, figure 457)
// =============================================================================
//
// Fixed by the architecture and identical on every part that has the
// block, so they are stated here and not probed: four NOR/PSRAM
// sub-banks of 64 MB from 0x6000 0000, the NAND banks at 0x7000 0000 and
// 0x8000 0000, the PC Card at 0x9000 0000, and the two SDRAM banks of
// 256 MB at 0xC000 0000 and 0xD000 0000. A bank the part has not gets 0
// here, so a window address is never a plausible-looking lie.

/// 64 MB per NOR/PSRAM sub-bank, one chip select each (FMC_NE1..4).
constexpr uint32_t fmc_static_window_bytes = 0x0400'0000UL;
/// 256 MB per SDRAM bank, the whole of which one device rarely fills.
constexpr uint32_t fmc_sdram_window_bytes = 0x1000'0000UL;

constexpr uint32_t fmc_static_window(uint8_t bank) {
    return (fmc_static_banks() != 0u && bank >= 1u && bank <= 4u)
               ? 0x6000'0000UL + fmc_static_window_bytes * (bank - 1u)
               : 0u;
}

constexpr uint32_t fmc_sdram_window(uint8_t bank) {
    return (fmc_sdram_banks() != 0u && bank >= 1u && bank <= 2u)
               ? 0xC000'0000UL + fmc_sdram_window_bytes * (bank - 1u)
               : 0u;
}

constexpr uint32_t fmc_nand_window(uint8_t bank) {
    return fmc_nand_bank_present(bank) ? 0x7000'0000UL + 0x1000'0000UL * (bank - 2u) : 0u;
}

constexpr uint32_t fmc_pccard_window() { return fmc_pccard_present() ? 0x9000'0000UL : 0u; }

// =============================================================================
// SDRAM vocabulary (37.7.5)
// =============================================================================

/// SDCRx.NC - how many column address bits the device takes.
enum class SdramColumns : uint8_t { eight = 0, nine = 1, ten = 2, eleven = 3 };

/// SDCRx.NR - how many row address bits. Code 3 is reserved and has no
/// spelling here.
enum class SdramRows : uint8_t { eleven = 0, twelve = 1, thirteen = 2 };

/// SDCRx.MWID - the device's data bus width. Code 3 is reserved.
enum class SdramWidth : uint8_t { bits8 = 0, bits16 = 1, bits32 = 2 };

/// SDCRx.NB - the device's INTERNAL banks (the ones BA[1:0] select),
/// not the controller's two.
enum class SdramInternalBanks : uint8_t { two = 0, four = 1 };

/// SDCRx.CAS, in memory clock cycles. Code 0 is reserved, so the
/// enumerator's value IS the number of cycles.
enum class SdramCas : uint8_t { one = 1, two = 2, three = 3 };

/// SDCR1.SDCLK - the memory clock, made inside this block from HCLK.
/// Code 1 is reserved. `off` stops the clock, which the chapter asks
/// for before the frequency changes and which makes the device need a
/// full re-initialization.
enum class SdramClock : uint8_t { off = 0, hclk_div2 = 2, hclk_div3 = 3 };

/// SDCR1.RPIPE - extra HCLK cycles before the data read after the CAS
/// latency is taken. Code 3 is reserved.
enum class SdramPipe : uint8_t { none = 0, one_hclk = 1, two_hclk = 2 };

/// SDCMR.MODE - what the command machine issues. Code 7 is reserved.
enum class SdramCommand : uint8_t {
    normal = 0,
    clock_enable = 1,    ///< SDCKE high; the device's power-up clock
    precharge_all = 2,   ///< PALL
    auto_refresh = 3,    ///< NRFS + 1 consecutive refresh cycles
    load_mode = 4,       ///< the device's own mode register, from MRD
    self_refresh = 5,
    power_down = 6,
};

/// SDSR.MODES1/2 - where a bank is now.
enum class SdramState : uint8_t { normal = 0, self_refresh = 1, power_down = 2 };

/// The geometry and the reading discipline: everything in SDCRx.
///
/// THREE OF THESE FIELDS ARE NOT PER BANK. SDCLK, RBURST and RPIPE live
/// in SDCR1 for BOTH banks (37.7.5: "the corresponding bits in the
/// FMC_SDCR2 register are read only" / "don't care"), so configure()
/// writes them into SDCR1 whichever bank it is called for. A two-device
/// board therefore states the same three values twice, and the second
/// statement is not ignored - it is written to the same place.
struct SdramConfig {
    SdramColumns columns = SdramColumns::eight;
    SdramRows rows = SdramRows::twelve;
    SdramWidth width = SdramWidth::bits16;
    SdramInternalBanks internal_banks = SdramInternalBanks::four;
    SdramCas cas = SdramCas::three;
    /// SDCRx.WP: with this set every write is ignored and an AHB error
    /// is raised (37.3). It is set out of reset.
    bool write_protect = false;

    // ---- SDCR1's, for both banks ----
    SdramClock clock = SdramClock::hclk_div2;
    /// RBURST: the six-line read FIFO, filled during the CAS latency.
    /// The manual's own recommendation and ES0206 2.3.4's workaround.
    bool read_burst = true;
    SdramPipe read_pipe = SdramPipe::none;
};

/// The seven SDTRx delays, IN MEMORY CLOCK CYCLES as a human counts
/// them (1..16); the register holds one less. Every one of them is a
/// nanosecond figure of the DEVICE's data sheet divided by the memory
/// clock period and rounded up.
///
/// TWO OF THESE ARE NOT PER BANK EITHER: TRP and TRC live in SDTR1
/// (37.7.5), with the slowest device's value where there are two.
struct SdramTiming {
    uint8_t load_mode_to_active = 2;   ///< TMRD
    uint8_t exit_self_refresh = 7;     ///< TXSR
    uint8_t self_refresh = 4;          ///< TRAS, the minimum self-refresh period
    uint8_t row_cycle = 6;             ///< TRC, SDTR1 only
    uint8_t write_recovery = 2;        ///< TWR
    uint8_t row_precharge = 2;         ///< TRP, SDTR1 only
    uint8_t row_to_column = 2;         ///< TRCD
};

/// What the power-up sequence of 37.7.3 needs beyond the geometry and
/// the timings.
struct SdramInit {
    /// Step 4: the delay after the clock starts, before any command.
    /// "Typical delay is around 100 us" - the device's data sheet is
    /// what really says, and 100 us is the JEDEC floor.
    uint16_t power_up_us = 100;
    /// Step 6: consecutive auto-refresh cycles, 1..15 (NRFS + 1). The
    /// chapter's typical number is 8.
    uint8_t refresh_cycles = 8;
    /// Step 7: the DEVICE's mode register, carried in MRD.
    /// sdram_mode_register() builds the usual one.
    uint16_t mode_register = 0;
    /// Step 8: SDRTR.COUNT. Zero leaves the refresh timer alone, which
    /// means NO REFRESH AT ALL - only useful when a second bank's
    /// initialization is to set it.
    uint16_t refresh_count = 0;
    /// Issue every command to both banks at once (CTB1 | CTB2), which
    /// 37.7.3 asks for when two devices are initialized together.
    bool both_banks = false;
};

// ---- the device's own mode register ------------------------------------------
//
// NOT an FMC register: MRD is a pipe through which the controller sends
// the SDRAM its LOAD MODE REGISTER word, and the meaning of the bits is
// the memory's. The encoding below is the JEDEC SDR SDRAM one that
// every device of this generation follows; a device that departs from
// it takes its own literal in SdramInit::mode_register.

enum class SdramBurstLength : uint8_t { one = 0, two = 1, four = 2, eight = 3, full_page = 7 };
enum class SdramBurstType : uint8_t { sequential = 0, interleaved = 1 };
/// Whether a write follows the programmed burst length or always
/// touches one location (mode register bit 9).
enum class SdramWriteBurst : uint8_t { programmed = 0, single = 1 };

/// The device's mode register word. 37.7.3 step 7b: the burst length
/// must be 1 - the controller breaks every AHB burst into single memory
/// accesses itself - and the CAS latency must match SDCRx.CAS.
constexpr uint16_t sdram_mode_register(SdramCas cas,
                                       SdramBurstLength length = SdramBurstLength::one,
                                       SdramBurstType type = SdramBurstType::sequential,
                                       SdramWriteBurst write_burst = SdramWriteBurst::single) {
    return static_cast<uint16_t>((static_cast<uint16_t>(length) & 0x7u) |
                                 (static_cast<uint16_t>(type) << 3u) |
                                 ((static_cast<uint16_t>(cas) & 0x7u) << 4u) |
                                 (static_cast<uint16_t>(write_burst) << 9u));
}

// ---- the arithmetic ------------------------------------------------------------

/// How many bits of address each field carries, for the capacity sum.
constexpr uint8_t sdram_column_bits(SdramColumns c) {
    return static_cast<uint8_t>(8u + static_cast<uint8_t>(c));
}
constexpr uint8_t sdram_row_bits(SdramRows r) {
    return static_cast<uint8_t>(11u + static_cast<uint8_t>(r));
}
constexpr uint8_t sdram_width_bytes(SdramWidth w) {
    return static_cast<uint8_t>(1u << static_cast<uint8_t>(w));
}

/// How many bytes the described device holds - rows x columns x internal
/// banks x the bus width. The controller's window is 256 MB whatever
/// this says; addresses past the device are the "reserved address
/// range" whose access raises an AHB error (37.3).
constexpr uint32_t sdram_capacity_bytes(const SdramConfig& c) {
    const uint8_t bank_bits = (c.internal_banks == SdramInternalBanks::four) ? 2u : 1u;
    const uint8_t address_bits =
        static_cast<uint8_t>(sdram_row_bits(c.rows) + sdram_column_bits(c.columns) + bank_bits);
    return (1UL << address_bits) * sdram_width_bytes(c.width);
}

/// The memory clock in hertz: HCLK over the divider SDCR1.SDCLK holds.
/// Zero when the clock is off.
constexpr uint32_t sdram_clock_hz(uint32_t hclk_hz, SdramClock c) {
    return (c == SdramClock::off) ? 0u : hclk_hz / static_cast<uint32_t>(c);
}

/// SDRTR.COUNT for a device that wants every row refreshed within
/// `refresh_period_us` (37.7.5):
///
///     COUNT = refresh_period / rows x memory clock - 20
///
/// The 20 cycles are the safety margin the chapter subtracts so that a
/// refresh request arriving on top of an accepted read still fits. The
/// product is computed in 64 bits because a 64 ms period times 90 MHz
/// does not fit in 32.
constexpr uint32_t sdram_refresh_count(uint32_t sdram_hz, uint32_t refresh_period_us,
                                       uint32_t rows) {
    if (sdram_hz == 0u || rows == 0u || refresh_period_us == 0u) {
        return 0u;
    }
    const uint64_t cycles =
        (static_cast<uint64_t>(sdram_hz) * refresh_period_us) / (1'000'000ULL * rows);
    return (cycles > 20ULL) ? static_cast<uint32_t>(cycles - 20ULL) : 0u;
}

/// COUNT is 13 bits and "must be set at least to 41 SDRAM clock cycles"
/// (37.7.5). Below that the field does not behave as the chapter says
/// (see FmcSdram::refresh_rate), so the floor is enforced here rather
/// than trusted.
constexpr bool sdram_refresh_count_valid(uint32_t count) {
    return count >= 41u && count <= 0x1FFFu;
}

/// 37.7.5's last note: the programmed COUNT must not be equal to
/// TWR + TRP + TRC + TRCD + 4 memory clock cycles. The manual gives no
/// reason and no range - one value is forbidden, so one value is
/// refused.
constexpr uint32_t sdram_forbidden_count(const SdramTiming& t) {
    return static_cast<uint32_t>(t.write_recovery) + t.row_precharge + t.row_cycle +
           t.row_to_column + 4u;
}

// ---- the refusals ---------------------------------------------------------------

/// Every reserved code of SDCRx spelled out, so that a config built
/// from run-time numbers cannot reach the register.
constexpr bool sdram_config_valid(const SdramConfig& c) {
    if (static_cast<uint8_t>(c.rows) > 2u) {
        return false;   // NR = 11 reserved
    }
    if (static_cast<uint8_t>(c.width) > 2u) {
        return false;   // MWID = 11 reserved
    }
    const uint8_t cas = static_cast<uint8_t>(c.cas);
    if (cas < 1u || cas > 3u) {
        return false;   // CAS = 00 reserved
    }
    const uint8_t clk = static_cast<uint8_t>(c.clock);
    if (clk == 1u || clk > 3u) {
        return false;   // SDCLK = 01 reserved
    }
    if (static_cast<uint8_t>(c.read_pipe) > 2u) {
        return false;   // RPIPE = 11 reserved
    }
    return true;
}

/// Each SDTR field is 1..16 cycles, and TWR carries two inequalities of
/// its own (37.7.5): it must cover what is left of TRAS after TRCD, and
/// what is left of TRC after TRCD and TRP. They are stated in CYCLES,
/// which is what this struct holds.
constexpr bool sdram_timing_valid(const SdramTiming& t) {
    const uint8_t fields[] = {t.load_mode_to_active, t.exit_self_refresh, t.self_refresh,
                              t.row_cycle,           t.write_recovery,    t.row_precharge,
                              t.row_to_column};
    for (uint8_t f : fields) {
        if (f < 1u || f > 16u) {
            return false;
        }
    }
    const uint16_t twr = t.write_recovery;
    if (twr + t.row_to_column < t.self_refresh) {
        return false;   // TWR >= TRAS - TRCD
    }
    if (twr + t.row_to_column + t.row_precharge < t.row_cycle) {
        return false;   // TWR >= TRC - TRCD - TRP
    }
    return true;
}

constexpr bool sdram_init_valid(const SdramInit& i, const SdramTiming& t) {
    if (i.refresh_cycles < 1u || i.refresh_cycles > 15u) {
        return false;   // NRFS = 1111 reserved
    }
    if (i.mode_register > 0x1FFFu) {
        return false;   // MRD is 13 bits
    }
    if (i.refresh_count != 0u) {
        if (!sdram_refresh_count_valid(i.refresh_count)) {
            return false;
        }
        if (i.refresh_count == sdram_forbidden_count(t)) {
            return false;
        }
    }
    return true;
}

// =============================================================================
// NOR Flash / PSRAM / SRAM vocabulary (37.5.6)
// =============================================================================

/// BCRx.MTYP. Code 3 is reserved.
enum class StaticMemory : uint8_t { sram = 0, psram = 1, nor = 2 };

/// BCRx.MWID, the same three codes as the SDRAM's.
enum class StaticWidth : uint8_t { bits8 = 0, bits16 = 1, bits32 = 2 };

/// BTRx/BWTRx.ACCMOD - which of the chapter's four asynchronous
/// protocols the extended mode runs. Read only where EXTMOD is set;
/// without it the memory type picks mode 1 (SRAM/PSRAM) or mode 2 (NOR).
enum class StaticAccess : uint8_t { mode_a = 0, mode_b = 1, mode_c = 2, mode_d = 3 };

/// BCRx.CPSIZE - where a Cellular RAM 1.5 burst must be split. Codes
/// above 4 are reserved.
enum class CramPageSize : uint8_t {
    none = 0,
    bytes128 = 1,
    bytes256 = 2,
    bytes512 = 3,
    bytes1024 = 4,
};

/// BCRx.WAITCFG - whether NWAIT is asserted one cycle before the wait
/// state or during it.
enum class WaitTiming : uint8_t { one_cycle_before = 0, during = 1 };

/// BCRx.WAITPOL.
enum class WaitPolarity : uint8_t { active_low = 0, active_high = 1 };

/// Everything in BCRx but the enable.
///
/// THE THREE DEFAULTS THAT ARE ERRATA WORKAROUNDS, all of ES0206:
/// `wait_enable` is on and `wait_polarity` is active high (2.3.2 - with
/// the polarity matching an idle NWAIT input the system hangs; 2.3.9 -
/// mixed WAITEN across banks corrupts a burst read), and
/// `bus_turnaround` in StaticTiming is nonzero (2.3.12). A program that
/// overrides them is on its own and the header says so here.
struct StaticConfig {
    StaticMemory memory = StaticMemory::sram;
    StaticWidth width = StaticWidth::bits16;
    /// MUXEN: address and data share the bus. NOR and PSRAM only.
    bool multiplexed = false;
    /// FACCEN: NOR accesses allowed. With it clear a read or write of a
    /// NOR bank raises an AHB error (37.3).
    bool flash_access = true;
    /// WREN: with it clear a write raises an AHB error.
    bool writable = true;
    /// BURSTEN: synchronous READS.
    bool burst_read = false;
    /// CBURSTRW: synchronous WRITES, the PSRAM burst-mode twin.
    bool burst_write = false;
    /// WAITEN: NWAIT taken into account in synchronous mode.
    bool wait_enable = true;
    WaitTiming wait_timing = WaitTiming::one_cycle_before;
    WaitPolarity wait_polarity = WaitPolarity::active_high;
    /// ASYNCWAIT: NWAIT taken into account in ASYNCHRONOUS mode too.
    bool async_wait = false;
    /// WRAPMOD, where the part has it: 37.5.6 states it has no effect,
    /// because neither the CPU nor the DMA of this family generates a
    /// wrapping burst. RM0390 11.6.6 drops the field altogether and
    /// setting it is refused on the F446, the F469 and the F479.
    ///
    /// THE FIELD GOING THE OTHER WAY HAS NO MEMBER HERE. RM0390 11.6.6
    /// adds BCR1.WFDIS, which disables the write FIFO the whole
    /// controller shares; its reset value is "enabled", which is
    /// ES0298 2.3.4's own workaround (a read taken one cycle after a
    /// write returns corrupt data with the FIFO off), and no CMSIS
    /// header of the pack declares a mask for it. A verb whose one
    /// legal argument is the reset value is worse than none.
    bool wrapped_burst = false;
    /// EXTMOD: BWTRx carries the write timings, so reads and writes get
    /// different ones.
    bool extended_mode = false;
    CramPageSize page_size = CramPageSize::none;
    /// CCLKEN: FMC_CLK runs continuously instead of only during a
    /// synchronous access. BANK 1'S FIELD ALONE governs all four.
    bool continuous_clock = false;
};

/// BTRx (and, under the extended mode, BWTRx), in the units the fields
/// really count - a human number, not a register code.
///
/// The defaults are the RESET VALUES (0x0FFF FFFF): the slowest
/// everything, which is the only safe starting point for a device
/// nobody has described yet.
struct StaticTiming {
    /// ADDSET, 0..15 HCLK cycles. At least 1 in multiplexed mode or
    /// mode D (37.5.6).
    uint8_t address_setup = 15;
    /// ADDHLD, 1..15 HCLK cycles; 0 is reserved. Used in mode D and in
    /// multiplexed accesses.
    uint8_t address_hold = 15;
    /// DATAST, 1..255 HCLK cycles; 0 is reserved.
    uint8_t data_phase = 255;
    /// BUSTURN, 0..15 HCLK cycles - and nonzero on every bank when
    /// several are used (ES0206 2.3.12).
    uint8_t bus_turnaround = 15;
    /// CLKDIV, the FMC_CLK period in HCLK cycles: 2..16. The register
    /// code 0 is reserved.
    uint8_t clock_divide = 16;
    /// DATLAT, the memory clock cycles before the first burst datum:
    /// 2..17. Not HCLK cycles - FMC_CLK ones.
    uint8_t data_latency = 17;
    StaticAccess access = StaticAccess::mode_a;
};

constexpr bool static_config_valid(const StaticConfig& c, uint8_t bank) {
    if (bank < 1u || bank > 4u) {
        return false;
    }
    if (static_cast<uint8_t>(c.memory) > 2u) {
        return false;   // MTYP = 11 reserved
    }
    if (static_cast<uint8_t>(c.width) > 2u) {
        return false;   // MWID = 11 reserved
    }
    if (static_cast<uint8_t>(c.page_size) > 4u) {
        return false;   // CPSIZE codes 5..7 reserved
    }
    if (c.multiplexed && c.memory == StaticMemory::sram) {
        return false;   // 37.5.6: MUXEN is "valid only with NOR and PSRAM memories"
    }
    if (c.page_size != CramPageSize::none && c.memory != StaticMemory::psram) {
        return false;   // CPSIZE is Cellular RAM's alone
    }
    if (c.continuous_clock && bank != 1u) {
        return false;   // CCLKEN is bank 1's field for every bank
    }
    if (c.continuous_clock && !c.burst_read) {
        return false;   // 37.5.6: bank 1 must be synchronous to make the continuous clock
    }
    if (c.wrapped_burst && fmc_wrapped_burst_mask() == 0u) {
        return false;   // RM0390 11.6.6: no WRAPMOD field on this part
    }
    return true;
}

constexpr bool static_timing_valid(const StaticTiming& t, const StaticConfig& c) {
    if (t.address_hold < 1u || t.address_hold > 15u) {
        return false;
    }
    if (t.data_phase < 1u) {
        return false;
    }
    if (t.address_setup > 15u || t.bus_turnaround > 15u) {
        return false;
    }
    if (t.clock_divide < 2u || t.clock_divide > 16u) {
        return false;
    }
    if (t.data_latency < 2u || t.data_latency > 17u) {
        return false;
    }
    if ((c.multiplexed || (c.extended_mode && t.access == StaticAccess::mode_d)) &&
        t.address_setup < 1u) {
        return false;   // 37.5.6: "In Muxed mode or Mode D, the minimum value for ADDSET is 1"
    }
    return true;
}

// The register-facing half is compiled only where the device header
// declares the FMC. An FSMC part says which block it really has.
#if defined(FMC_Bank1_R_BASE)

// =============================================================================
// The block
// =============================================================================

class Fmc {
public:
    static_assert(fmc_present(),
                  "brio Fmc: this device has no FMC (the F405 class, the F412 and the F413/F423 "
                  "carry the FSMC - the same static banks under other register names and with no "
                  "SDRAM controller - and the F401, F410, F411 and F412Cx carry neither)");

    Fmc() = delete;

    /// What this part's silicon has, off the device header.
    static constexpr uint8_t static_banks = fmc_static_banks();
    static constexpr uint8_t sdram_banks = fmc_sdram_banks();
    /// The NAND half: two banks on the F42x/F43x, one on the F446 and
    /// the F469/F479. There is no driver for it here - see the
    /// document's gap list - but a program that maps a NAND has to know
    /// whether the bank exists at all.
    static constexpr uint8_t nand_banks = fmc_nand_banks();
    static constexpr bool has_pc_card = fmc_pccard_present();

    /// The one vector the whole controller shares: the NAND banks' FIFO
    /// interrupts and the SDRAM's refresh error come out of it, so a
    /// handler bound here is a dispatcher.
    static constexpr IRQn_Type irq_line = fmc_irq();

    // ---- the AHB3 gate (RM0090 7.3.12) ------------------------------------------

    /// RCC_AHB3ENR.FMCEN, clear at reset. With it clear the whole
    /// register block reads zero and swallows writes, and the memory
    /// windows answer nothing - the same trap as an unclocked GPIO
    /// port. Every configuring verb of this file opens it first.
    static void clock(bool on) { Rcc::ahb3_clock(fmc_clock_mask(), on); }
    static bool clock() { return Rcc::ahb3_clock(fmc_clock_mask()); }

    /// RCC_AHB3RSTR.FMCRST: every bank's configuration back to its reset
    /// value in one pulse. AN INITIALIZED SDRAM DOES NOT SURVIVE IT as a
    /// configuration - the sequence has to be run again before a single
    /// load or store, or the machine hangs (ES0206 2.3.3) - and it is
    /// also the ONLY thing that leaves the device unrefreshed: the
    /// memory clock stops with the block, and neither a COUNT below the
    /// floor nor SDCR1.SDCLK = 00 stops it (see FmcSdram). The DEVICE's
    /// charge survives for as long as its rows hold it, which is long
    /// enough that a reset and a re-initialization keep the contents.
    static void reset() {
        clock(true);
        Rcc::ahb3_reset(fmc_reset_mask());
    }

    static void enable_interrupt() { Nvic::enable(irq_line); }
    static void disable_interrupt() { Nvic::disable(irq_line); }

    // ---- the windows -------------------------------------------------------------

    static constexpr uint32_t static_window(uint8_t bank) { return fmc_static_window(bank); }
    static constexpr uint32_t sdram_window(uint8_t bank) { return fmc_sdram_window(bank); }
    static constexpr uint32_t nand_window(uint8_t bank) { return fmc_nand_window(bank); }
    static constexpr uint32_t pc_card_window() { return fmc_pccard_window(); }

    /// At least `cycles` core cycles with no timebase of any kind: the
    /// body is eight NOPs, so one iteration is at least eight cycles and
    /// the count is a floor. The SDRAM power-up delay is measured with
    /// this rather than with delay_us(), because a program wants its
    /// external memory before it starts SysTick.
    static void spin_at_least(uint32_t cycles) {
        for (uint32_t i = cycles / 8u + 1u; i != 0u; --i) {
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();
        }
    }
};

#if defined(FMC_Bank5_6_R_BASE)

// =============================================================================
// The SDRAM controller (37.7)
// =============================================================================

/// FmcSdram<1> is the device on FMC_SDNE0/SDCKE0 at 0xC000 0000,
/// FmcSdram<2> the one on FMC_SDNE1/SDCKE1 at 0xD000 0000. The two share
/// one command register, one refresh timer and one status register, and
/// three fields of SDCR1 and two of SDTR1 (see SdramConfig).
template <uint8_t bank>
class FmcSdram {
public:
    static_assert(bank == 1u || bank == 2u,
                  "brio FmcSdram: this family's SDRAM controller has two banks, 1 (SDNE0, "
                  "0xC0000000) and 2 (SDNE1, 0xD0000000)");
    static_assert(fmc_sdram_banks() != 0u,
                  "brio FmcSdram: this device's external memory controller has no SDRAM half");

    FmcSdram() = delete;

    /// Where the device is in the Cortex's address map, and how much of
    /// the map the bank owns whatever the device fills.
    static constexpr uint32_t base = fmc_sdram_window(bank);
    static constexpr uint32_t window_bytes = fmc_sdram_window_bytes;

    /// A pointer into the device, at a byte offset from its base. The
    /// type is the ACCESS width, not the device's: the controller splits
    /// or masks whatever the AHB asks for (37.3.1), so a byte store into
    /// a 16-bit device drives one NBL and a word load takes two memory
    /// cycles. NOTHING HERE CHECKS THAT THE BANK IS INITIALIZED - the
    /// caller does, because a read of an uninitialized bank hangs the
    /// machine with no fault (ES0206 2.3.3).
    template <typename T>
    static volatile T* at(uint32_t byte_offset = 0) {
        return reinterpret_cast<volatile T*>(base + byte_offset);
    }

    static FMC_Bank5_6_TypeDef& regs() {
        return *reinterpret_cast<FMC_Bank5_6_TypeDef*>(fmc_sdram_base());
    }

    /// How long a command verb waits for SDSR.BUSY to fall before
    /// giving up. The longest wait the chapter asks for is the TRAS
    /// that follows a self-refresh command - sixteen memory cycles -
    /// so this is four orders of magnitude of slack and a stopped
    /// controller is reported instead of hung on.
    static constexpr uint32_t busy_spins = 100'000u;

    // ---- configuration -------------------------------------------------------

    /// SDCRx from a geometry, with SDCLK, RBURST and RPIPE going into
    /// SDCR1 whichever bank this is. Refuses every reserved code and
    /// writes nothing when it does.
    static bool configure(const SdramConfig& c) {
        if (!sdram_config_valid(c)) {
            return false;
        }
        Fmc::clock(true);
        const uint32_t own = (static_cast<uint32_t>(c.columns) << FMC_SDCR1_NC_Pos) |
                             (static_cast<uint32_t>(c.rows) << FMC_SDCR1_NR_Pos) |
                             (static_cast<uint32_t>(c.width) << FMC_SDCR1_MWID_Pos) |
                             (static_cast<uint32_t>(c.internal_banks) << FMC_SDCR1_NB_Pos) |
                             (static_cast<uint32_t>(c.cas) << FMC_SDCR1_CAS_Pos) |
                             (c.write_protect ? FMC_SDCR1_WP : 0u);
        const uint32_t shared = (static_cast<uint32_t>(c.clock) << FMC_SDCR1_SDCLK_Pos) |
                                (c.read_burst ? FMC_SDCR1_RBURST : 0u) |
                                (static_cast<uint32_t>(c.read_pipe) << FMC_SDCR1_RPIPE_Pos);
        // Bank 1 carries both halves in one store; bank 2 keeps its own
        // and puts the shared three where the silicon reads them.
        if constexpr (bank == 1u) {
            regs().SDCR[0] = own | shared;
        } else {
            regs().SDCR[1] = own;
            regs().SDCR[0] = (regs().SDCR[0] & ~(FMC_SDCR1_SDCLK | FMC_SDCR1_RBURST |
                                                 FMC_SDCR1_RPIPE)) |
                             shared;
        }
        return true;
    }

    /// SDTRx from a timing, with TRP and TRC going into SDTR1 whichever
    /// bank this is. The fields hold one less than the cycle count.
    static bool timing(const SdramTiming& t) {
        if (!sdram_timing_valid(t)) {
            return false;
        }
        Fmc::clock(true);
        const uint32_t own =
            (static_cast<uint32_t>(t.load_mode_to_active - 1u) << FMC_SDTR1_TMRD_Pos) |
            (static_cast<uint32_t>(t.exit_self_refresh - 1u) << FMC_SDTR1_TXSR_Pos) |
            (static_cast<uint32_t>(t.self_refresh - 1u) << FMC_SDTR1_TRAS_Pos) |
            (static_cast<uint32_t>(t.write_recovery - 1u) << FMC_SDTR1_TWR_Pos) |
            (static_cast<uint32_t>(t.row_to_column - 1u) << FMC_SDTR1_TRCD_Pos);
        const uint32_t shared =
            (static_cast<uint32_t>(t.row_cycle - 1u) << FMC_SDTR1_TRC_Pos) |
            (static_cast<uint32_t>(t.row_precharge - 1u) << FMC_SDTR1_TRP_Pos);
        if constexpr (bank == 1u) {
            regs().SDTR[0] = own | shared;
        } else {
            regs().SDTR[1] = own;
            regs().SDTR[0] = (regs().SDTR[0] & ~(FMC_SDTR1_TRC | FMC_SDTR1_TRP)) | shared;
        }
        return true;
    }

    /// SDCRx and SDTRx as they stand, for a program that wants to check
    /// what a previous stage left (and for a suite that measures it).
    static uint32_t control_word() { Fmc::clock(true); return regs().SDCR[bank - 1u]; }
    static uint32_t timing_word() { Fmc::clock(true); return regs().SDTR[bank - 1u]; }

    /// SDCRx.WP alone: a bank made read-only without disturbing the
    /// geometry. A write to a protected bank is ignored AND raises an
    /// AHB error (37.3), which on the CPU is a hard fault.
    static void write_protect(bool on) {
        Fmc::clock(true);
        if (on) {
            regs().SDCR[bank - 1u] |= FMC_SDCR1_WP;
        } else {
            regs().SDCR[bank - 1u] &= ~FMC_SDCR1_WP;
        }
    }
    static bool write_protect() {
        Fmc::clock(true);
        return (regs().SDCR[bank - 1u] & FMC_SDCR1_WP) != 0u;
    }

    // ---- the command machine (37.7.3, 37.7.5) ----------------------------------

    static bool busy() { return (regs().SDSR & FMC_SDSR_BUSY) != 0u; }

    /// Spin until the controller will take another command. False on a
    /// time-out, which is a broken block and not a slow one.
    static bool wait_ready(uint32_t spins = busy_spins) {
        Fmc::clock(true);
        for (uint32_t i = 0; i < spins; ++i) {
            if (!busy()) {
                return true;
            }
        }
        return false;
    }

    /// One SDCMR store, bracketed by the two BUSY waits the register
    /// needs: one so the previous command has finished, one so the
    /// caller knows this one has. `refresh_cycles` is read for the
    /// auto-refresh command alone and `mode_register` for the load-mode
    /// one; both are refused out of range whichever command it is,
    /// because a field that reaches the register is a field that must
    /// be legal.
    static bool command(SdramCommand mode, uint8_t refresh_cycles = 1,
                        uint16_t mode_register = 0, bool both_banks = false) {
        if (refresh_cycles < 1u || refresh_cycles > 15u) {
            return false;
        }
        if (mode_register > 0x1FFFu) {
            return false;
        }
        if (static_cast<uint8_t>(mode) > 6u) {
            return false;   // MODE = 111 reserved
        }
        if (!wait_ready()) {
            return false;
        }
        const uint32_t targets =
            both_banks ? (FMC_SDCMR_CTB1 | FMC_SDCMR_CTB2)
                       : (bank == 1u ? FMC_SDCMR_CTB1 : FMC_SDCMR_CTB2);
        regs().SDCMR = (static_cast<uint32_t>(mode) << FMC_SDCMR_MODE_Pos) | targets |
                       (static_cast<uint32_t>(refresh_cycles - 1u) << FMC_SDCMR_NRFS_Pos) |
                       (static_cast<uint32_t>(mode_register) << FMC_SDCMR_MRD_Pos);
        return wait_ready();
    }

    /// Where this bank is, out of SDSR.
    static SdramState state() {
        Fmc::clock(true);
        const uint32_t s = regs().SDSR;
        const uint32_t code = (bank == 1u) ? ((s & FMC_SDSR_MODES1) >> FMC_SDSR_MODES1_Pos)
                                           : ((s & FMC_SDSR_MODES2) >> FMC_SDSR_MODES2_Pos);
        return static_cast<SdramState>(code);
    }

    /// The two low-power modes and the way back. In self refresh the
    /// device keeps itself alive with no clock; in power down the
    /// CONTROLLER keeps refreshing it, leaving power down for each
    /// refresh and returning. Either is left by the first access to the
    /// bank as well as by normal() - the silicon wakes the device
    /// itself and the access simply costs more.
    static bool self_refresh(bool both_banks = false) {
        return command(SdramCommand::self_refresh, 1, 0, both_banks);
    }
    static bool power_down(bool both_banks = false) {
        return command(SdramCommand::power_down, 1, 0, both_banks);
    }
    static bool normal(bool both_banks = false) {
        return command(SdramCommand::normal, 1, 0, both_banks);
    }

    // ---- the refresh timer (37.7.5) ---------------------------------------------

    /// SDRTR.COUNT: memory clock cycles between auto-refresh requests,
    /// 41..8191. Refuses the value 37.7.5 forbids for the timings in
    /// force, which is why this takes them.
    static bool refresh_rate(uint32_t count, const SdramTiming& t) {
        if (!sdram_refresh_count_valid(count) || count == sdram_forbidden_count(t)) {
            return false;
        }
        return refresh_rate(count);
    }

    /// The same without the forbidden-value check, for a caller that has
    /// made it itself (initialize() has).
    ///
    /// A COUNT BELOW THE FLOOR IS NOT A WAY TO STOP THE REFRESH.
    /// 37.7.5 says "if the value programmed in the register is 0, no
    /// refresh is carried out", and this silicon does not do that: a
    /// store of 0 into COUNT reads back as 40 and the timer keeps
    /// running at that rate (measured; docs/stm32f4/fmc.md). Anything
    /// below 41 is refused here so that no program can be built on the
    /// sentence, and memory_clock(off) is what really leaves a device
    /// unrefreshed.
    static bool refresh_rate(uint32_t count) {
        if (!sdram_refresh_count_valid(count)) {
            return false;
        }
        Fmc::clock(true);
        regs().SDRTR = (regs().SDRTR & ~FMC_SDRTR_COUNT) |
                       (static_cast<uint32_t>(count) << FMC_SDRTR_COUNT_Pos);
        return true;
    }

    static uint32_t refresh_rate() {
        Fmc::clock(true);
        return (regs().SDRTR & FMC_SDRTR_COUNT) >> FMC_SDRTR_COUNT_Pos;
    }

    /// SDCR1.SDCLK, the memory clock, for BOTH banks. 37.7.5 asks for a
    /// PALL before the field changes and this issues it; it also says
    /// the device must be RE-INITIALIZED after the change, which this
    /// does not do - `initialize()` is that, and whether the contents
    /// are worth keeping across it is the caller's question.
    ///
    /// SdramClock::off IS NOT A WAY TO STOP THE REFRESH. The field
    /// reads back cleared and 37.7.5 calls it "SDCLK clock disabled",
    /// but FMC_SDCLK's pad goes on toggling with it clear (measured;
    /// docs/stm32f4/fmc.md), so the refresh timer - which counts on
    /// that clock - goes on counting. Only the block's own reset
    /// (Fmc::reset()) leaves this device unclocked and unrefreshed.
    static bool memory_clock(SdramClock c) {
        const uint8_t code = static_cast<uint8_t>(c);
        if (code == 1u || code > 3u) {
            return false;
        }
        Fmc::clock(true);
        if (!command(SdramCommand::precharge_all)) {
            return false;
        }
        regs().SDCR[0] = (regs().SDCR[0] & ~FMC_SDCR1_SDCLK) |
                         (static_cast<uint32_t>(c) << FMC_SDCR1_SDCLK_Pos);
        return true;
    }

    static SdramClock memory_clock() {
        Fmc::clock(true);
        return static_cast<SdramClock>((regs().SDCR[0] & FMC_SDCR1_SDCLK) >>
                                       FMC_SDCR1_SDCLK_Pos);
    }

    /// SDRTR.REIE: an interrupt on Fmc::irq_line when a refresh request
    /// arrives with the previous one unserved.
    static void refresh_interrupt(bool on) {
        Fmc::clock(true);
        if (on) {
            regs().SDRTR |= FMC_SDRTR_REIE;
        } else {
            regs().SDRTR &= ~FMC_SDRTR_REIE;
        }
    }
    static bool refresh_interrupt() {
        Fmc::clock(true);
        return (regs().SDRTR & FMC_SDRTR_REIE) != 0u;
    }

    /// SDSR.RE - a refresh request that arrived before the previous one
    /// was served. It says the rate is too fast for the traffic, not
    /// that data was lost.
    static bool refresh_error() { return (regs().SDSR & FMC_SDSR_RE) != 0u; }

    /// SDRTR.CRE is write-only and sits in the register that holds
    /// COUNT, so clearing the flag means storing COUNT back - AND THE
    /// TIMER RESTARTS COUNTING FROM IT (37.7.5: "As soon as the
    /// FMC_SDRTR register is programmed, the timer starts counting").
    /// There is no other way to clear RE, so a program that clears it
    /// often postpones a refresh each time.
    static void clear_refresh_error() {
        Fmc::clock(true);
        regs().SDRTR = regs().SDRTR | FMC_SDRTR_CRE;
    }

    /// The ISR body for Fmc::irq_line: answers whether the refresh error
    /// was this interrupt's cause and clears it. The vector is shared
    /// with the NAND banks, so an app that has both calls both bodies.
    [[gnu::always_inline]] static bool refresh_isr() {
        if (!refresh_error()) {
            return false;
        }
        clear_refresh_error();
        return true;
    }

    // ---- the power-up sequence (37.7.3) ------------------------------------------

    /// The whole of 37.7.3 in one verb: the geometry and the timings,
    /// the clock started, the prescribed delay, PALL, the auto-refresh
    /// burst, the device's mode register and the refresh rate. Answers
    /// false - having written whatever it got to - the moment a step is
    /// refused or the controller stops answering; the caller must not
    /// touch the window after that.
    ///
    /// `clock` is the CORE clock, used for the power-up delay alone; the
    /// memory clock is HCLK over SdramConfig::clock and nothing here
    /// needs to know it. The delay is spun, not slept, so this runs
    /// before SysTick and before the kernel.
    template <typename Clock>
    static bool initialize(Clock clock, const SdramConfig& c, const SdramTiming& t,
                           const SdramInit& init) {
        if (!sdram_config_valid(c) || !sdram_timing_valid(t) || !sdram_init_valid(init, t)) {
            return false;
        }
        if (c.clock == SdramClock::off) {
            return false;   // step 3 has nothing to deliver
        }
        Fmc::clock(true);
        if (!configure(c) || !timing(t)) {
            return false;   // steps 1 and 2
        }
        if (!command(SdramCommand::clock_enable, 1, 0, init.both_banks)) {
            return false;   // step 3
        }
        // Step 4. The floor is a whole number of core cycles per
        // microsecond rounded UP, so the wait is never short.
        const uint32_t per_us = clock_hz(clock) / 1'000'000u + 1u;
        Fmc::spin_at_least(per_us * init.power_up_us);
        if (!command(SdramCommand::precharge_all, 1, 0, init.both_banks)) {
            return false;   // step 5
        }
        if (!command(SdramCommand::auto_refresh, init.refresh_cycles, 0, init.both_banks)) {
            return false;   // step 6
        }
        if (!command(SdramCommand::load_mode, 1, init.mode_register, init.both_banks)) {
            return false;   // step 7
        }
        if (init.refresh_count != 0u && !refresh_rate(init.refresh_count)) {
            return false;   // step 8
        }
        return true;
    }
};

#endif   // FMC_Bank5_6_R_BASE

// =============================================================================
// The NOR Flash / PSRAM / SRAM controller (37.5)
// =============================================================================

/// One of the four sub-banks of FMC bank 1, each with its own chip
/// select FMC_NE1..4, its own 64 MB window and its own three registers.
///
/// NOTHING HERE IS BENCH-MEASURED: no board on this desk carries a
/// static memory on the FMC. The register set is complete and the
/// chapter's refusals are code, but a timing that satisfies them is
/// still only a timing that satisfies them.
template <uint8_t bank>
class FmcNorPsram {
public:
    static_assert(bank >= 1u && bank <= 4u,
                  "brio FmcNorPsram: FMC bank 1 has four sub-banks, 1..4 (FMC_NE1..NE4)");

    FmcNorPsram() = delete;

    static constexpr uint32_t base = fmc_static_window(bank);
    static constexpr uint32_t window_bytes = fmc_static_window_bytes;

    template <typename T>
    static volatile T* at(uint32_t byte_offset = 0) {
        return reinterpret_cast<volatile T*>(base + byte_offset);
    }

    /// BCRx and BTRx are interleaved in one array (BTCR[8]); BWTRx lives
    /// in a block of its own with the same stride.
    static FMC_Bank1_TypeDef& regs() {
        return *reinterpret_cast<FMC_Bank1_TypeDef*>(fmc_bank1_base());
    }
    static FMC_Bank1E_TypeDef& write_regs() {
        return *reinterpret_cast<FMC_Bank1E_TypeDef*>(fmc_bank1e_base());
    }

    static constexpr uint8_t control_index = static_cast<uint8_t>(2u * (bank - 1u));
    static constexpr uint8_t read_timing_index = static_cast<uint8_t>(2u * (bank - 1u) + 1u);
    static constexpr uint8_t write_timing_index = static_cast<uint8_t>(2u * (bank - 1u));

    // ---- configuration ------------------------------------------------------------

    /// BCRx from a description, WITH THE BANK LEFT DISABLED: a window
    /// that answers before its timings are written is a window that can
    /// hang on a device that is not there. enable() is the separate
    /// step, after timing().
    ///
    /// CCLKEN is bank 1's field for all four banks, so this refuses it
    /// on banks 2..4 rather than writing a bit the silicon ignores.
    static bool configure(const StaticConfig& c) {
        if (!static_config_valid(c, bank)) {
            return false;
        }
        Fmc::clock(true);
        uint32_t v = (static_cast<uint32_t>(c.memory) << FMC_BCR1_MTYP_Pos) |
                     (static_cast<uint32_t>(c.width) << FMC_BCR1_MWID_Pos) |
                     (static_cast<uint32_t>(c.page_size) << FMC_BCR1_CPSIZE_Pos) |
                     (c.multiplexed ? FMC_BCR1_MUXEN : 0u) |
                     (c.flash_access ? FMC_BCR1_FACCEN : 0u) |
                     (c.writable ? FMC_BCR1_WREN : 0u) |
                     (c.burst_read ? FMC_BCR1_BURSTEN : 0u) |
                     (c.burst_write ? FMC_BCR1_CBURSTRW : 0u) |
                     (c.wait_enable ? FMC_BCR1_WAITEN : 0u) |
                     (c.wait_timing == WaitTiming::during ? FMC_BCR1_WAITCFG : 0u) |
                     (c.wait_polarity == WaitPolarity::active_high ? FMC_BCR1_WAITPOL : 0u) |
                     (c.async_wait ? FMC_BCR1_ASYNCWAIT : 0u) |
                     (c.wrapped_burst ? fmc_wrapped_burst_mask() : 0u) |
                     (c.extended_mode ? FMC_BCR1_EXTMOD : 0u);
        if constexpr (bank == 1u) {
            v |= c.continuous_clock ? FMC_BCR1_CCLKEN : 0u;
        }
        regs().BTCR[control_index] = v;
        return true;
    }

    /// BTRx: the READ timings, and the only ones when the extended mode
    /// is off. ACCMOD is written whatever the mode - the silicon reads
    /// it only under EXTMOD, and a field that describes the intent is
    /// worth more than a field left at its reset value.
    static bool timing(const StaticTiming& t, const StaticConfig& c) {
        if (!static_timing_valid(t, c)) {
            return false;
        }
        Fmc::clock(true);
        regs().BTCR[read_timing_index] =
            (static_cast<uint32_t>(t.address_setup) << FMC_BTR1_ADDSET_Pos) |
            (static_cast<uint32_t>(t.address_hold) << FMC_BTR1_ADDHLD_Pos) |
            (static_cast<uint32_t>(t.data_phase) << FMC_BTR1_DATAST_Pos) |
            (static_cast<uint32_t>(t.bus_turnaround) << FMC_BTR1_BUSTURN_Pos) |
            (static_cast<uint32_t>(t.clock_divide - 1u) << FMC_BTR1_CLKDIV_Pos) |
            (static_cast<uint32_t>(t.data_latency - 2u) << FMC_BTR1_DATLAT_Pos) |
            (static_cast<uint32_t>(t.access) << FMC_BTR1_ACCMOD_Pos);
        return true;
    }

    /// BWTRx: the WRITE timings, read only while BCRx.EXTMOD is set.
    /// The register has no CLKDIV and no DATLAT - a synchronous write
    /// takes bank 1's - so those two fields of the argument are ignored
    /// here and not silently written somewhere else.
    static bool write_timing(const StaticTiming& t, const StaticConfig& c) {
        if (!static_timing_valid(t, c)) {
            return false;
        }
        Fmc::clock(true);
        write_regs().BWTR[write_timing_index] =
            (static_cast<uint32_t>(t.address_setup) << FMC_BWTR1_ADDSET_Pos) |
            (static_cast<uint32_t>(t.address_hold) << FMC_BWTR1_ADDHLD_Pos) |
            (static_cast<uint32_t>(t.data_phase) << FMC_BWTR1_DATAST_Pos) |
            (static_cast<uint32_t>(t.bus_turnaround) << FMC_BWTR1_BUSTURN_Pos) |
            (static_cast<uint32_t>(t.access) << FMC_BWTR1_ACCMOD_Pos);
        return true;
    }

    // ---- the enable ----------------------------------------------------------------

    /// BCRx.MBKEN. Bank 1 comes out of reset ENABLED (its BCR1 resets to
    /// 0x0000 30DB) and banks 2..4 disabled; nothing reaches a pad until
    /// the AHB3 gate is open, so "enabled" out of reset costs nothing
    /// until Fmc::clock(true).
    static void enable() {
        Fmc::clock(true);
        regs().BTCR[control_index] |= FMC_BCR1_MBKEN;
    }
    static void disable() {
        Fmc::clock(true);
        regs().BTCR[control_index] &= ~FMC_BCR1_MBKEN;
    }
    static bool enabled() {
        Fmc::clock(true);
        return (regs().BTCR[control_index] & FMC_BCR1_MBKEN) != 0u;
    }

    static uint32_t control_word() { Fmc::clock(true); return regs().BTCR[control_index]; }
    static uint32_t timing_word() { Fmc::clock(true); return regs().BTCR[read_timing_index]; }
    static uint32_t write_timing_word() {
        Fmc::clock(true);
        return write_regs().BWTR[write_timing_index];
    }
};

// ---- the two tasks over it -------------------------------------------------------

/// An asynchronous SRAM (or ROM) on sub-bank `bank`: the memory type
/// pinned, no burst of any kind, and one timing to state. `read` is what
/// BTRx gets; pass a second timing to run the extended mode with its own
/// write timings.
template <uint8_t bank>
struct FmcSram {
    using Resource = FmcNorPsram<bank>;

    FmcSram() = delete;

    static constexpr uint32_t base = Resource::base;

    template <typename T>
    static volatile T* at(uint32_t byte_offset = 0) {
        return Resource::template at<T>(byte_offset);
    }

    static bool init(StaticWidth width, const StaticTiming& read) {
        const StaticConfig c{.memory = StaticMemory::sram, .width = width};
        return start(c, read, read);
    }

    static bool init(StaticWidth width, const StaticTiming& read, const StaticTiming& write) {
        const StaticConfig c{.memory = StaticMemory::sram, .width = width, .extended_mode = true};
        return start(c, read, write);
    }

    /// MBKEN cleared: the window stops answering. The timings and the
    /// description stay where init() put them - a bank re-enabled is
    /// the same bank, and Fmc::reset() is what wipes the block.
    static void release() {
        Resource::disable();
    }

private:
    static bool start(const StaticConfig& c, const StaticTiming& read,
                      const StaticTiming& write) {
        if (!Resource::configure(c) || !Resource::timing(read, c)) {
            return false;
        }
        if (c.extended_mode && !Resource::write_timing(write, c)) {
            return false;
        }
        Resource::enable();
        return true;
    }
};

/// A NOR flash on sub-bank `bank`: MTYP = NOR, FACCEN on (without it
/// every access to the bank is an AHB error), and the choice this device
/// really has - asynchronous, or the synchronous burst read whose data
/// latency and clock divider are BTRx's.
template <uint8_t bank>
struct FmcNor {
    using Resource = FmcNorPsram<bank>;

    FmcNor() = delete;

    static constexpr uint32_t base = Resource::base;

    template <typename T>
    static volatile T* at(uint32_t byte_offset = 0) {
        return Resource::template at<T>(byte_offset);
    }

    /// Asynchronous, non-multiplexed - the plain memory-mapped flash.
    static bool init(StaticWidth width, const StaticTiming& read) {
        const StaticConfig c{.memory = StaticMemory::nor, .width = width};
        return start(c, read);
    }

    /// Asynchronous with the address and data lines shared, which is how
    /// most NOR parts of this size are wired. ADDSET must be at least 1
    /// here and static_timing_valid() enforces it.
    static bool init_multiplexed(StaticWidth width, const StaticTiming& read) {
        const StaticConfig c{
            .memory = StaticMemory::nor, .width = width, .multiplexed = true};
        return start(c, read);
    }

    /// The synchronous burst read of 37.5.5: BURSTEN on, the clock
    /// divider and the data latency taken from `read`. WAITEN stays on
    /// and the polarity active high - ES0206 2.3.2 and 2.3.9.
    static bool init_burst(StaticWidth width, const StaticTiming& read) {
        const StaticConfig c{.memory = StaticMemory::nor, .width = width, .burst_read = true};
        return start(c, read);
    }

    /// MBKEN cleared; the description and the timings stay.
    static void release() { Resource::disable(); }

private:
    static bool start(const StaticConfig& c, const StaticTiming& read) {
        if (!Resource::configure(c) || !Resource::timing(read, c)) {
            return false;
        }
        Resource::enable();
        return true;
    }
};

#endif   // FMC_Bank1_R_BASE

} // namespace brio
