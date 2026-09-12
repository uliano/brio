/*
 * flash.hpp
 *
 * The RP2040's flash: an EXTERNAL quad-SPI chip the core executes out
 * of through the XIP window (datasheet 2.6.3, the cache and its four
 * aliases; 4.10, the SSI that drives the chip), erased and programmed
 * through THE BOOTROM'S OWN FUNCTIONS (2.8.3.1.3) - the chip has no
 * flash controller of its own, and the only code that knows how to
 * take the SSI out of its execute-in-place mode, drive a command and
 * put it back is the one in mask ROM, found by name in the ROM table
 * (2.8.3, the lookup at 0x18 over the function table at 0x14). Three
 * things here:
 *
 *  - `Flash`, THE ENGINE: `erase(addr, bytes)` by whole 4 KB sectors
 *    (the 64 KB block command taken where a run is aligned to it),
 *    `program(addr, src)` by whole 256-byte pages, `read` through the
 *    cached window, `command(tx, rx)` - a raw exchange with the chip
 *    on the SSI with the chip select forced low, how the JEDEC id
 *    (9Fh), the unique id (4Bh) and the status register (05h) are
 *    read -, and the geometry the build states: `size_bytes` from
 *    BRIO_RP2040_FLASH_KB, the same number the linker script and the
 *    storage partition (nvm_flash.hpp) are drawn from.
 *
 *  - `Xip`, THE CACHE: enable, flush, power down, the hit and access
 *    counters the suite reads a hit ratio off, and the four aliases
 *    of the window as addresses (`Flash::address` cached,
 *    `Flash::uncached_address` through the no-cache-no-allocate one).
 *
 *  - THE WINDOW OF THE OPERATION, which is what makes this driver
 *    unlike the other strata's. While the SSI is out of XIP mode THERE
 *    IS NO FLASH: a fetch from 0x1000_0000 hangs or faults. So every
 *    operation runs from `.ram_text`, an input section the linker
 *    script folds into .data (the crt copies it to SRAM with the
 *    initialized data), with INTERRUPTS MASKED for its whole duration
 *    - a handler fetched from flash would be the fault - and returns
 *    to XIP through THE SECOND STAGE ITSELF: the 256 bytes at the start
 *    of the image, copied to SRAM once by `init()` and called as a
 *    function (the stage returns to a caller whose link register is
 *    not zero, and vectors into flash when it is: src/glue/boot2_*.S),
 *    so the SSI comes back in the same quad-I/O mode it booted in,
 *    whatever chip the board carries. The bootrom's own 'CX' entry
 *    would leave it in a slow single-line mode instead. The masked
 *    window is measured by the suite: a sector erase is tens of
 *    milliseconds, a page program under a millisecond, and the ticker
 *    loses whatever ticks fall inside.
 *
 * WHAT THE WINDOW ASKS OF THE APPLICATION, stated because nothing here
 * can enforce it: nothing else may fetch from flash while it is open -
 * not the other core (a core 1 running a program out of flash must be
 * parked in the bootrom or in SRAM first; the suite runs with core 1
 * where the bootrom left it), not a DMA channel reading the window.
 * The source of a program and the buffers of a command are refused
 * when they lie in the window (`in_window`): the bootrom would read
 * them through a flash that is not there.
 *
 * ADDRESSES ARE OFFSETS IN THE CHIP, from 0 - what the bootrom's
 * functions take - and the window adds 0x1000_0000 where a pointer is
 * wanted. The first 256 bytes are the stage, the image follows: an
 * erase that reaches the image is the application's to refuse; the
 * media of nvm_flash.hpp refuse it by their bounds.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <array>
#include <optional>
#include <span>

#include "rp2040/device.hpp"

#ifndef BRIO_RP2040_FLASH_KB
#error "BRIO_RP2040_FLASH_KB names the board's flash size in KB (rp2040/CMakeLists.txt passes it)"
#endif

namespace brio {

/// The address map's fixed regions (2.2): the mask ROM and the four
/// aliases of the XIP window (the CMSIS header names the peripherals'
/// bases alone).
inline constexpr uint32_t rom_base = 0x0000'0000u;
inline constexpr uint32_t xip_base = 0x1000'0000u;                    ///< cached, allocating
inline constexpr uint32_t xip_noalloc_base = 0x1100'0000u;            ///< a hit served, a miss not filled
inline constexpr uint32_t xip_nocache_base = 0x1200'0000u;            ///< the cache bypassed, a line still filled
inline constexpr uint32_t xip_nocache_noalloc_base = 0x1300'0000u;    ///< the flash alone
inline constexpr uint32_t xip_alias_span = 0x0400'0000u;              ///< the four aliases together

/// What 9Fh answers: the JEDEC manufacturer, the memory type, the
/// capacity as a power of two (0x15 = 2 MB).
struct FlashJedecId {
    uint8_t manufacturer;
    uint8_t type;
    uint8_t capacity;

    static constexpr uint8_t winbond = 0xEF;
    static constexpr uint8_t zetta = 0xBA;

    constexpr uint32_t bytes() const { return capacity < 32u ? (1ul << capacity) : 0u; }
};

namespace detail {

/// The bootrom's flash functions, found once, and the stage copied out
/// once; the operations that run with the flash disconnected. Not for
/// an application: `Flash` below is the surface.
struct FlashRom {
    FlashRom() = delete;

    using PlainFn = void (*)();
    using RangeEraseFn = void (*)(uint32_t addr, uint32_t count, uint32_t block_size, uint8_t block_cmd);
    using RangeProgramFn = void (*)(uint32_t addr, const uint8_t* data, uint32_t count);
    using LookupFn = void* (*)(const uint16_t* table, uint32_t code);

    static inline PlainFn connect = nullptr;         ///< 'IF' - the QSPI pads and the SSI back to the flash
    static inline PlainFn exit_xip = nullptr;        ///< 'EX' - the SSI into a serial command mode, the chip out of continuous read
    static inline RangeEraseFn range_erase = nullptr;       ///< 'RE'
    static inline RangeProgramFn range_program = nullptr;   ///< 'RP'
    static inline PlainFn flush_cache = nullptr;     ///< 'FC' - the XIP cache flushed and enabled
    static inline PlainFn enter_cmd_xip = nullptr;   ///< 'CX' - the slow 03h read mode (not used; the stage is)
    alignas(4) static inline uint8_t stage[256]{};   ///< the second stage, copied out of the image
    static inline bool ready = false;

    static constexpr uint32_t stage_bytes = 256;
    static constexpr uint32_t rom_magic = 0x10;
    static constexpr uint32_t rom_func_table = 0x14;
    static constexpr uint32_t rom_lookup = 0x18;

    static constexpr uint32_t code(char a, char b) {
        return static_cast<uint32_t>(static_cast<uint8_t>(a)) | (static_cast<uint32_t>(static_cast<uint8_t>(b)) << 8);
    }

    /// A ROM address the compiler cannot fold: the table lives in the
    /// first page of the address space, which gcc takes for the null
    /// page and refuses to read through a constant.
    static uintptr_t rom_address(uint32_t at) {
        uintptr_t a = rom_base + at;
        asm volatile("" : "+r"(a));
        return a;
    }
    static const volatile uint8_t* rom_byte(uint32_t at) {
        return reinterpret_cast<const volatile uint8_t*>(rom_address(at));
    }
    static uint16_t rom_half(uint32_t at) {
        return *reinterpret_cast<const volatile uint16_t*>(rom_address(at));
    }

    /// Is this a bootrom with a table (2.8.3: 'M', 'u', and a version)?
    static bool magic_ok() {
        return *rom_byte(rom_magic) == 'M' && *rom_byte(rom_magic + 1u) == 'u';
    }
    static uint8_t rom_version() { return *rom_byte(rom_magic + 2u); }

    /// The CRC the bootrom checks the stage by (2.8.1.3.1): CRC-32,
    /// polynomial 0x04C11DB7, MSB first, seed all ones, no final XOR,
    /// over the first 252 bytes, stored little-endian in the last four.
    static uint32_t stage_crc(const uint8_t* bytes) {
        uint32_t crc = 0xFFFF'FFFFu;
        for (uint32_t i = 0; i < stage_bytes - 4u; ++i) {
            crc ^= static_cast<uint32_t>(bytes[i]) << 24;
            for (uint8_t b = 0; b < 8u; ++b) {
                crc = (crc & 0x8000'0000u) != 0u ? (crc << 1) ^ 0x04C1'1DB7u : crc << 1;
            }
        }
        return crc;
    }

    /// Find the six functions and copy the stage out: once, from flash
    /// (the window is still there), before any operation. False when
    /// the ROM carries no table, a function is missing, or the stage's
    /// CRC is not the bootrom's (an image whose first 256 bytes are not
    /// a stage would vector nowhere on the way back).
    static bool ensure() {
        if (ready) {
            return true;
        }
        if (!magic_ok()) {
            return false;
        }
        const auto* table = reinterpret_cast<const uint16_t*>(static_cast<uintptr_t>(rom_half(rom_func_table)));
        const auto lookup = reinterpret_cast<LookupFn>(static_cast<uintptr_t>(rom_half(rom_lookup)));
        connect = reinterpret_cast<PlainFn>(lookup(table, code('I', 'F')));
        exit_xip = reinterpret_cast<PlainFn>(lookup(table, code('E', 'X')));
        range_erase = reinterpret_cast<RangeEraseFn>(lookup(table, code('R', 'E')));
        range_program = reinterpret_cast<RangeProgramFn>(lookup(table, code('R', 'P')));
        flush_cache = reinterpret_cast<PlainFn>(lookup(table, code('F', 'C')));
        enter_cmd_xip = reinterpret_cast<PlainFn>(lookup(table, code('C', 'X')));
        if (connect == nullptr || exit_xip == nullptr || range_erase == nullptr || range_program == nullptr ||
            flush_cache == nullptr || enter_cmd_xip == nullptr) {
            return false;
        }
        memcpy(stage, reinterpret_cast<const void*>(static_cast<uintptr_t>(xip_base)), stage_bytes);
        uint32_t stored = 0;
        memcpy(&stored, stage + stage_bytes - 4u, 4u);
        if (stage_crc(stage) != stored) {
            return false;
        }
        ready = true;
        return true;
    }

    // ----- what runs with the flash disconnected --------------------------
    // Every function below is in .ram_text, not inlined into a caller in
    // flash, and FLATTENED so that nothing it calls is a call into flash
    // (the CMSIS intrinsics fold in; the bootrom's functions and the
    // stage are reached through pointers). No switch (its jump table
    // would be in .rodata), no library call.

    /// The stage as a function: it returns here because the link
    /// register is not zero.
    static void reenter_xip() {
        const auto stage_fn = reinterpret_cast<PlainFn>(reinterpret_cast<uintptr_t>(stage) | 1u);
        stage_fn();
    }

    [[gnu::section(".ram_text"), gnu::noinline, gnu::flatten]]
    static void erase_in_ram(uint32_t addr, uint32_t count, uint32_t block_size, uint8_t block_cmd) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        connect();
        exit_xip();
        range_erase(addr, count, block_size, block_cmd);
        flush_cache();
        reenter_xip();
        __set_PRIMASK(primask);
    }

    [[gnu::section(".ram_text"), gnu::noinline, gnu::flatten]]
    static void program_in_ram(uint32_t addr, const uint8_t* data, uint32_t count) {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        connect();
        exit_xip();
        range_program(addr, data, count);
        flush_cache();
        reenter_xip();
        __set_PRIMASK(primask);
    }

    /// A raw exchange on the SSI in the serial mode 'EX' leaves it in
    /// (eight-bit frames, one line each way), the chip select forced
    /// low through the QSPI pad's override for the whole of it; the
    /// bytes clocked out are tx[], the bytes clocked in land in rx[],
    /// as many as go out. The transmit FIFO is kept two short of full
    /// so the receive side never overflows.
    [[gnu::section(".ram_text"), gnu::noinline, gnu::flatten]]
    static void command_in_ram(const uint8_t* tx, uint8_t* rx, uint32_t n) {
        volatile uint32_t& ss = *reinterpret_cast<volatile uint32_t*>(IO_QSPI_BASE + IO_QSPI_GPIO_QSPI_SS_CTRL_OFFSET);
        volatile uint32_t& sr = *reinterpret_cast<volatile uint32_t*>(SSI_BASE + SSI_SR_OFFSET);
        volatile uint32_t& dr0 = *reinterpret_cast<volatile uint32_t*>(SSI_BASE + SSI_DR0_OFFSET);
        constexpr uint32_t in_flight_max = 16u - 2u;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        connect();
        exit_xip();
        ss = (ss & ~IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS) |
             (IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_LOW << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB);
        uint32_t to_send = n;
        uint32_t to_take = n;
        while (to_send != 0u || to_take != 0u) {
            const uint32_t flags = sr;
            if ((flags & SSI_SR_TFNF_BITS) != 0u && to_send != 0u && to_take - to_send < in_flight_max) {
                dr0 = tx[n - to_send];
                --to_send;
            }
            if ((flags & SSI_SR_RFNE_BITS) != 0u && to_take != 0u) {
                rx[n - to_take] = static_cast<uint8_t>(dr0);
                --to_take;
            }
        }
        ss = (ss & ~IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_BITS) |
             (IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_VALUE_NORMAL << IO_QSPI_GPIO_QSPI_SS_CTRL_OUTOVER_LSB);
        flush_cache();
        reenter_xip();
        __set_PRIMASK(primask);
    }
};

} // namespace detail

/// The XIP cache (2.6.3): 16 KB, two-way, in front of the window.
struct Xip {
    Xip() = delete;

    static volatile uint32_t& ctrl() { return reg_at(XIP_CTRL_BASE, XIP_CTRL_OFFSET); }
    static volatile uint32_t& stat() { return reg_at(XIP_CTRL_BASE, XIP_STAT_OFFSET); }

    static bool enabled() { return (ctrl() & XIP_CTRL_EN_BITS) != 0u; }
    /// Disabled, every access misses and goes to the flash.
    static void enable(bool on) {
        ctrl() = on ? (ctrl() | XIP_CTRL_EN_BITS) : (ctrl() & ~XIP_CTRL_EN_BITS);
    }
    /// The cache memories powered down (their contents kept); a hit
    /// while powered down is a bus fault, so the cache is disabled first.
    static void power_down(bool on) {
        if (on) {
            ctrl() = (ctrl() & ~XIP_CTRL_EN_BITS) | XIP_CTRL_POWER_DOWN_BITS;
        } else {
            ctrl() = ctrl() & ~XIP_CTRL_POWER_DOWN_BITS;
        }
    }
    static bool powered_down() { return (ctrl() & XIP_CTRL_POWER_DOWN_BITS) != 0u; }
    /// A write to any alias but the cached one faults (true) or is
    /// dropped in silence (false).
    static void fault_on_bad_write(bool on) {
        ctrl() = on ? (ctrl() | XIP_CTRL_ERR_BADWRITE_BITS) : (ctrl() & ~XIP_CTRL_ERR_BADWRITE_BITS);
    }
    static bool faults_on_bad_write() { return (ctrl() & XIP_CTRL_ERR_BADWRITE_BITS) != 0u; }

    /// Every line invalidated; a read of FLUSH holds the bus until it is
    /// done, which is how this waits.
    static void flush() {
        reg_at(XIP_CTRL_BASE, XIP_FLUSH_OFFSET) = 1u;
        const uint32_t done = reg_at(XIP_CTRL_BASE, XIP_FLUSH_OFFSET);
        (void)done;
    }
    static bool flush_ready() { return (stat() & XIP_STAT_FLUSH_READY_BITS) != 0u; }

    /// The two counters: accesses through the cached alias, and the hits
    /// among them. Saturating, cleared by any write.
    static uint32_t hits() { return reg_at(XIP_CTRL_BASE, XIP_CTR_HIT_OFFSET); }
    static uint32_t accesses() { return reg_at(XIP_CTRL_BASE, XIP_CTR_ACC_OFFSET); }
    static void reset_counters() {
        reg_at(XIP_CTRL_BASE, XIP_CTR_HIT_OFFSET) = 0u;
        reg_at(XIP_CTRL_BASE, XIP_CTR_ACC_OFFSET) = 0u;
    }
};

/// The flash chip: the geometry the build states and the bootrom's
/// operations behind bounds and alignment checks.
struct Flash {
    Flash() = delete;

    static constexpr uint32_t size_bytes = static_cast<uint32_t>(BRIO_RP2040_FLASH_KB) * 1024u;
    static constexpr uint32_t page_size = 256;      ///< the program unit (the bootrom's 'RP' takes whole pages)
    static constexpr uint32_t sector_size = 4096;   ///< the erase unit (the 20h sector)
    static constexpr uint32_t block_size = 65536;   ///< the larger erase the bootrom takes where a run is aligned to it
    static constexpr uint8_t block_command = 0xD8;
    static constexpr uint32_t stage_bytes = detail::FlashRom::stage_bytes;
    static constexpr uint32_t max_command = 64;     ///< the longest exchange command() carries
    static constexpr uint32_t window = xip_base;

    static_assert(size_bytes % sector_size == 0u, "a flash of whole sectors");
    static_assert(size_bytes >= 2u * block_size, "smaller than the smallest chip the stage is written for");

    /// Find the bootrom's functions and copy the stage out. Every
    /// operation calls it itself; an application calls it to learn
    /// early. False on a ROM with no table or a stage with no CRC.
    static bool init() { return detail::FlashRom::ensure(); }
    static bool ready() { return detail::FlashRom::ready; }
    static uint8_t rom_version() { return detail::FlashRom::rom_version(); }
    /// The stage's CRC as computed and as stored, for the record.
    static uint32_t stage_crc() { return detail::FlashRom::stage_crc(reinterpret_cast<const uint8_t*>(static_cast<uintptr_t>(window))); }
    static uint32_t stage_crc_stored() {
        uint32_t stored = 0;
        memcpy(&stored, reinterpret_cast<const void*>(static_cast<uintptr_t>(window + stage_bytes - 4u)), 4u);
        return stored;
    }

    /// Is this pointer in the window (any of its four aliases)? What the
    /// window's operations refuse as a source or a destination.
    static bool in_window(const void* p) {
        const auto a = reinterpret_cast<uintptr_t>(p);
        return a >= xip_base && a < xip_base + xip_alias_span;
    }

    /// The flash byte at `offset`, through the cache.
    static const uint8_t* address(uint32_t offset) {
        return reinterpret_cast<const uint8_t*>(static_cast<uintptr_t>(window + offset));
    }
    /// The same byte through the alias that neither hits nor fills the
    /// cache: what the flash holds, not what the cache remembers.
    static const uint8_t* uncached_address(uint32_t offset) {
        return reinterpret_cast<const uint8_t*>(static_cast<uintptr_t>(xip_nocache_noalloc_base + offset));
    }

    /// `dst` filled from the chip through the cache; the part past the
    /// chip's end, if any, reads as erased.
    static void read(uint32_t addr, std::span<uint8_t> dst) {
        if (addr >= size_bytes) {
            memset(dst.data(), 0xFF, dst.size());
            return;
        }
        const uint32_t n = dst.size() <= size_bytes - addr ? static_cast<uint32_t>(dst.size()) : size_bytes - addr;
        memcpy(dst.data(), address(addr), n);
        if (n < dst.size()) {
            memset(dst.data() + n, 0xFF, dst.size() - n);
        }
    }

    /// Whole sectors from `addr`, `bytes` long; the 64 KB block command
    /// where the run is aligned to it (the bootrom's own choice). The
    /// window is open for the whole run: bound it.
    static bool erase(uint32_t addr, uint32_t bytes) {
        if (bytes == 0u || (addr % sector_size) != 0u || (bytes % sector_size) != 0u || addr >= size_bytes ||
            bytes > size_bytes - addr) {
            return false;
        }
        if (!init()) {
            return false;
        }
        detail::FlashRom::erase_in_ram(addr, bytes, block_size, block_command);
        return true;
    }
    /// One sector.
    static bool erase_sector(uint32_t addr) { return erase(addr, sector_size); }

    /// Whole pages from `addr`, `src` in SRAM (a source in the window is
    /// refused: the bootrom would read it through a flash that is not
    /// there). A page programs 1 -> 0 only; a page programmed twice
    /// between erases holds the AND of the two, which is the physics
    /// the chapter does not state and the suite measures.
    static bool program(uint32_t addr, std::span<const uint8_t> src) {
        if (src.empty() || (addr % page_size) != 0u || (src.size() % page_size) != 0u || addr >= size_bytes ||
            src.size() > size_bytes - addr || in_window(src.data())) {
            return false;
        }
        if (!init()) {
            return false;
        }
        detail::FlashRom::program_in_ram(addr, src.data(), static_cast<uint32_t>(src.size()));
        return true;
    }

    /// A raw exchange: `tx` clocked out, as many bytes clocked into `rx`
    /// (the same length, at most `max_command`, `rx` in SRAM). The
    /// command byte, its address and dummy bytes and the zeros that
    /// clock the answer in are the caller's; `tx` may live anywhere,
    /// it is copied to the stack first.
    static bool command(std::span<const uint8_t> tx, std::span<uint8_t> rx) {
        if (tx.empty() || tx.size() > max_command || rx.size() != tx.size() || in_window(rx.data())) {
            return false;
        }
        if (!init()) {
            return false;
        }
        uint8_t out[max_command];
        memcpy(out, tx.data(), tx.size());
        detail::FlashRom::command_in_ram(out, rx.data(), static_cast<uint32_t>(tx.size()));
        return true;
    }

    /// 9Fh: manufacturer, type, capacity.
    static std::optional<FlashJedecId> jedec_id() {
        const uint8_t tx[4] = {0x9F, 0, 0, 0};
        uint8_t rx[4] = {};
        if (!command(tx, rx)) {
            return std::nullopt;
        }
        return FlashJedecId{.manufacturer = rx[1], .type = rx[2], .capacity = rx[3]};
    }

    /// 4Bh: the chip's 64-bit unique id, after four dummy bytes.
    static std::optional<std::array<uint8_t, 8>> unique_id() {
        const uint8_t tx[13] = {0x4B, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        uint8_t rx[13] = {};
        if (!command(tx, rx)) {
            return std::nullopt;
        }
        std::array<uint8_t, 8> id{};
        memcpy(id.data(), rx + 5, 8u);
        return id;
    }

    /// 05h: status register 1 (bit 0 busy, bit 1 write-enabled, the
    /// block-protect bits above).
    static std::optional<uint8_t> status_register() {
        const uint8_t tx[2] = {0x05, 0};
        uint8_t rx[2] = {};
        if (!command(tx, rx)) {
            return std::nullopt;
        }
        return rx[1];
    }
};

} // namespace brio
