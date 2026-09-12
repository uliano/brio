/*
 * pio.hpp
 *
 * The RP2040's programmable I/O (datasheet chapter 3): two blocks of
 * four STATE MACHINES each, executing sixteen-bit programs from a
 * shared memory of thirty-two instructions - nine instructions, one
 * cycle each, a delay of up to 31 cycles and a side-set of up to five
 * pins folded into every one -, each machine with two 32-bit scratch
 * registers, an output and an input shift register with their
 * counters, a four-deep FIFO each way (joinable into one of eight), a
 * 16.8 clock divider, four pin ranges (OUT, SET, IN, side-set) and a
 * pin to branch on, eight flags shared by the machines of a block of
 * which four reach the two system interrupt lines, and a DMA request
 * per FIFO. Three layers, the other strata's arrangement:
 *
 *  - THE ASSEMBLER: the nine instructions as constexpr encoders
 *    (`pio_jmp` .. `pio_set`), the side-set and delay folded in by
 *    `pio_side_delay`, a program as `PioProgram<N>` with its wrap
 *    points and side-set shape, JMP addresses relocated at load. Every
 *    program is written here in these words - there is no pioasm in
 *    this tree.
 *  - `Pio<n>` is the BLOCK: the reset gate, the instruction memory
 *    with a first-fit placer, the enables and restarts in lockstep
 *    (CTRL), the FIFO status and debug words, the eight flags, the two
 *    interrupt lines with their twelve sources, the ISR body.
 *  - `PioSm<n, sm>` is the RESOURCE of one machine: its configuration
 *    (the divider, the execution controls, the shift controls, the pin
 *    ranges), the FIFOs, an instruction executed at once (SMx_INSTR),
 *    the pins claimed through a SET executed on the side.
 *  - THE TASKS are the chapter's own programs: `PioUartTx` and
 *    `PioUartRx` (3.6.3, 3.6.4: a serial port on any pin at eight
 *    cycles a bit), `PioSquareWave` (3.2.1's four-cycle wave),
 *    `PioPwm` (3.6.8: a util/pwm_channel.hpp PwmChannel counted by a
 *    program). Any other program is the application's, written with
 *    the encoders above.
 *
 * THE PINS: any of the thirty GPIOs under function 6 (PIO0) or 7
 * (PIO1); a machine's four ranges are bases and counts modulo 32 over
 * the block's pins; on one cycle the highest-numbered machine writing
 * a pin wins, and a side-set beats an OUT or SET of the same machine
 * (3.5.6.1). An input passes a two-flip-flop synchronizer unless
 * INPUT_SYNC_BYPASS says otherwise.
 *
 * THE RELOCATION: a JMP carries an ABSOLUTE address (3.4.2), so a
 * program written from 0 is moved to its load offset by adding the
 * offset to every JMP's target - which is why programs here name
 * their targets from 0 and load() does the adding; an instruction
 * executed through OUT EXEC or MOV EXEC is the application's to
 * relocate.
 *
 * THE FIFOS AND THE DMA: a word pushed to TXFx, a word popped from
 * RXFx; the request of each rises with room or data (table 119); a
 * byte-wide read of RXFx + 3 takes the top byte of an entry, where a
 * right-shifting IN leaves eight bits (3.6.4).
 */

#pragma once

#include <stdint.h>
#include <array>
#include <optional>

#include "rp2040/device.hpp"

#include "rp2040/dma_engine.hpp"
#include "rp2040/nvic.hpp"
#include "rp2040/pin.hpp"
#include "rp2040/resets.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// The assembler
// =============================================================================

using PioInstr = uint16_t;

inline constexpr uint8_t pio_instruction_count = 32;
inline constexpr uint8_t pio_sm_count = 4;
inline constexpr uint8_t pio_fifo_depth = 4;
inline constexpr uint8_t pio_flag_count = 8;

/// JMP's condition (3.4.2).
enum class PioJmp : uint8_t { always = 0, x_zero = 1, x_dec = 2, y_zero = 3, y_dec = 4, x_ne_y = 5, pin = 6, osr_not_empty = 7 };
/// WAIT's source (3.4.3).
enum class PioWaitOn : uint8_t { gpio = 0, pin = 1, irq = 2 };
/// IN's source (3.4.4).
enum class PioIn : uint8_t { pins = 0, x = 1, y = 2, null = 3, isr = 6, osr = 7 };
/// OUT's destination (3.4.5).
enum class PioOut : uint8_t { pins = 0, x = 1, y = 2, null = 3, pindirs = 4, pc = 5, isr = 6, exec = 7 };
/// MOV's destination, operation and source (3.4.8).
enum class PioMovTo : uint8_t { pins = 0, x = 1, y = 2, exec = 4, pc = 5, isr = 6, osr = 7 };
enum class PioMovOp : uint8_t { none = 0, invert = 1, reverse = 2 };
enum class PioMovFrom : uint8_t { pins = 0, x = 1, y = 2, null = 3, status = 5, isr = 6, osr = 7 };
/// SET's destination (3.4.10).
enum class PioSetTo : uint8_t { pins = 0, x = 1, y = 2, pindirs = 4 };

constexpr PioInstr pio_jmp(PioJmp cond, uint8_t address) {
    return static_cast<PioInstr>((0u << 13) | (static_cast<uint16_t>(cond) << 5) | (address & 0x1Fu));
}
constexpr PioInstr pio_wait(bool polarity, PioWaitOn on, uint8_t index) {
    return static_cast<PioInstr>((1u << 13) | (polarity ? 1u << 7 : 0u) | (static_cast<uint16_t>(on) << 5) | (index & 0x1Fu));
}
/// A WAIT on a flag relative to the machine's number (the index's bit 4).
constexpr PioInstr pio_wait_irq(bool polarity, uint8_t flag, bool relative = false) {
    return pio_wait(polarity, PioWaitOn::irq, static_cast<uint8_t>((flag & 7u) | (relative ? 0x10u : 0u)));
}
/// Bit counts 1..32, 32 encoded as 0.
constexpr PioInstr pio_in(PioIn from, uint8_t bits) {
    return static_cast<PioInstr>((2u << 13) | (static_cast<uint16_t>(from) << 5) | (bits & 0x1Fu));
}
constexpr PioInstr pio_out(PioOut to, uint8_t bits) {
    return static_cast<PioInstr>((3u << 13) | (static_cast<uint16_t>(to) << 5) | (bits & 0x1Fu));
}
constexpr PioInstr pio_push(bool if_full = false, bool block = true) {
    return static_cast<PioInstr>((4u << 13) | (if_full ? 1u << 6 : 0u) | (block ? 1u << 5 : 0u));
}
constexpr PioInstr pio_pull(bool if_empty = false, bool block = true) {
    return static_cast<PioInstr>((4u << 13) | (1u << 7) | (if_empty ? 1u << 6 : 0u) | (block ? 1u << 5 : 0u));
}
constexpr PioInstr pio_mov(PioMovTo to, PioMovFrom from, PioMovOp op = PioMovOp::none) {
    return static_cast<PioInstr>((5u << 13) | (static_cast<uint16_t>(to) << 5) | (static_cast<uint16_t>(op) << 3) |
                                 static_cast<uint16_t>(from));
}
/// IRQ: set (wait = the machine stalls until the flag is cleared) or
/// clear the flag; `relative` adds the machine's number to the low two
/// bits.
constexpr PioInstr pio_irq(uint8_t flag, bool clear = false, bool wait = false, bool relative = false) {
    return static_cast<PioInstr>((6u << 13) | (clear ? 1u << 6 : 0u) | (wait ? 1u << 5 : 0u) | (flag & 7u) |
                                 (relative ? 0x10u : 0u));
}
constexpr PioInstr pio_set(PioSetTo to, uint8_t data) {
    return static_cast<PioInstr>((7u << 13) | (static_cast<uint16_t>(to) << 5) | (data & 0x1Fu));
}
/// The assembler's nop: MOV Y, Y.
constexpr PioInstr pio_nop() { return pio_mov(PioMovTo::y, PioMovFrom::y); }

/// How a program uses bits 12:8 of every instruction: `sideset_count`
/// MSBs are the side-set (the enable bit included when `side_en`), the
/// rest the delay.
struct PioSideSet {
    uint8_t count = 0;        ///< PINCTRL_SIDESET_COUNT, 0..5, the enable bit included
    bool optional = false;    ///< EXECCTRL_SIDE_EN: the MSB is an enable, the width one less
    bool pindirs = false;     ///< EXECCTRL_SIDE_PINDIR: directions instead of levels
    constexpr uint8_t data_bits() const { return static_cast<uint8_t>(optional ? count - 1u : count); }
    constexpr uint8_t delay_bits() const { return static_cast<uint8_t>(5u - count); }
    constexpr bool valid() const { return count <= 5u && (!optional || count >= 1u); }
};

/// The delay and the side-set folded into an instruction under `s`:
/// `side` the value for the side-set pins (nullopt = no side-set, legal
/// only when the side-set is optional), `delay` under 2^(5 - count).
constexpr PioInstr pio_side_delay(PioInstr instr, const PioSideSet& s, std::optional<uint8_t> side, uint8_t delay = 0) {
    uint16_t field = 0;
    if (side) {
        uint16_t v = static_cast<uint16_t>(*side & ((1u << s.data_bits()) - 1u));
        if (s.optional) {
            v |= static_cast<uint16_t>(1u << s.data_bits());
        }
        field = static_cast<uint16_t>(v << s.delay_bits());
    }
    field |= static_cast<uint16_t>(delay & ((1u << s.delay_bits()) - 1u));
    return static_cast<PioInstr>((instr & 0xE0FFu) | (field << 8));
}
/// A delay alone.
constexpr PioInstr pio_delay(PioInstr instr, uint8_t cycles, const PioSideSet& s = {}) {
    return pio_side_delay(instr, s, std::nullopt, cycles);
}

constexpr bool pio_is_jmp(PioInstr i) { return (i >> 13) == 0u; }
/// A JMP moved to its load offset; anything else unchanged.
constexpr PioInstr pio_relocate(PioInstr i, uint8_t offset) {
    return pio_is_jmp(i) ? static_cast<PioInstr>((i & 0xFFE0u) | ((i + offset) & 0x1Fu)) : i;
}

/// A program: its instructions written from address 0, the wrap points
/// (relative, the whole program by default) and its side-set shape.
template <uint8_t N>
struct PioProgram {
    static_assert(N >= 1u && N <= pio_instruction_count, "a PIO program is 1..32 instructions");
    std::array<PioInstr, N> code{};
    uint8_t wrap_bottom = 0;
    uint8_t wrap_top = N - 1;
    PioSideSet side{};
    static constexpr uint8_t length = N;
    constexpr bool valid() const { return wrap_bottom < N && wrap_top < N && wrap_bottom <= wrap_top && side.valid(); }
};

/// The 16.8 divider (SMx_CLKDIV): `integer` 1..65535, or 0 for 65536.
struct PioClockDiv {
    uint16_t integer = 1;
    uint8_t frac = 0;
    constexpr uint32_t reg() const { return (static_cast<uint32_t>(integer) << 16) | (static_cast<uint32_t>(frac) << 8); }
    constexpr uint32_t x256() const { return static_cast<uint32_t>(integer == 0u ? 65536u : integer) * 256u + frac; }
    constexpr bool valid() const { return integer != 0u || frac == 0u; }
    constexpr bool operator==(const PioClockDiv&) const = default;
};

/// The divider for a machine clock of `hz` at `sys_hz`, to the nearest
/// 1/256; nullopt under one or over 65536.
constexpr std::optional<PioClockDiv> pio_clock_div_for(uint32_t sys_hz, uint32_t hz) {
    if (hz == 0u || sys_hz == 0u) {
        return {};
    }
    const uint64_t x256 = (static_cast<uint64_t>(sys_hz) * 256u + hz / 2u) / hz;
    if (x256 < 256u || x256 > 65536u * 256u) {
        return {};
    }
    const uint32_t integer = static_cast<uint32_t>(x256 / 256u);
    return PioClockDiv{static_cast<uint16_t>(integer == 65536u ? 0u : integer), static_cast<uint8_t>(x256 % 256u)};
}
/// The machine's clock at `sys_hz` under a divider.
constexpr uint32_t pio_sm_hz(uint32_t sys_hz, PioClockDiv d) {
    return static_cast<uint32_t>((static_cast<uint64_t>(sys_hz) * 256u) / d.x256());
}

/// Which FIFO takes the other's storage (SHIFTCTRL.FJOIN_*).
enum class PioFifoJoin : uint8_t { none, tx, rx };
/// MOV x, STATUS's source (EXECCTRL.STATUS_SEL).
enum class PioStatus : uint8_t { tx_level = 0, rx_level = 1 };

/// One machine's configuration - the four registers a program needs
/// set before it runs.
struct PioSmConfig {
    PioClockDiv clock{};
    uint8_t wrap_bottom = 0;          ///< ABSOLUTE addresses (init() adds the offset)
    uint8_t wrap_top = 31;
    PioSideSet side{};
    uint8_t jmp_pin = 0;              ///< EXECCTRL_JMP_PIN: the GPIO JMP PIN branches on
    bool out_sticky = false;
    bool inline_out_enable = false;
    uint8_t out_enable_bit = 0;       ///< OUT_EN_SEL
    PioStatus status_select = PioStatus::tx_level;
    uint8_t status_n = 0;
    PioFifoJoin fifo_join = PioFifoJoin::none;
    uint8_t pull_threshold = 32;      ///< 1..32
    uint8_t push_threshold = 32;      ///< 1..32
    bool out_shift_right = true;
    bool in_shift_right = true;
    bool autopull = false;
    bool autopush = false;
    uint8_t out_base = 0;
    uint8_t out_count = 0;            ///< 0..32
    uint8_t set_base = 0;
    uint8_t set_count = 5;            ///< 0..5
    uint8_t sideset_base = 0;
    uint8_t in_base = 0;
};

constexpr bool pio_sm_config_valid(const PioSmConfig& c) {
    return c.clock.valid() && c.wrap_bottom < 32u && c.wrap_top < 32u && c.side.valid() && c.jmp_pin < 32u &&
           c.out_enable_bit < 32u && c.status_n < 16u && c.pull_threshold >= 1u && c.pull_threshold <= 32u &&
           c.push_threshold >= 1u && c.push_threshold <= 32u && c.out_base < 32u && c.out_count <= 32u && c.set_base < 32u &&
           c.set_count <= 5u && c.sideset_base < 32u && c.in_base < 32u;
}

constexpr uint32_t pio_execctrl_of(const PioSmConfig& c) {
    return (c.side.optional ? PIO_SM0_EXECCTRL_SIDE_EN_BITS : 0u) | (c.side.pindirs ? PIO_SM0_EXECCTRL_SIDE_PINDIR_BITS : 0u) |
           (static_cast<uint32_t>(c.jmp_pin) << PIO_SM0_EXECCTRL_JMP_PIN_LSB) |
           (static_cast<uint32_t>(c.out_enable_bit) << PIO_SM0_EXECCTRL_OUT_EN_SEL_LSB) |
           (c.inline_out_enable ? PIO_SM0_EXECCTRL_INLINE_OUT_EN_BITS : 0u) | (c.out_sticky ? PIO_SM0_EXECCTRL_OUT_STICKY_BITS : 0u) |
           (static_cast<uint32_t>(c.wrap_top) << PIO_SM0_EXECCTRL_WRAP_TOP_LSB) |
           (static_cast<uint32_t>(c.wrap_bottom) << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) |
           (static_cast<uint32_t>(c.status_select) << PIO_SM0_EXECCTRL_STATUS_SEL_LSB) |
           (static_cast<uint32_t>(c.status_n) << PIO_SM0_EXECCTRL_STATUS_N_LSB);
}
constexpr uint32_t pio_shiftctrl_of(const PioSmConfig& c) {
    return (c.fifo_join == PioFifoJoin::rx ? PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS : 0u) |
           (c.fifo_join == PioFifoJoin::tx ? PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS : 0u) |
           (static_cast<uint32_t>(c.pull_threshold & 0x1Fu) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB) |
           (static_cast<uint32_t>(c.push_threshold & 0x1Fu) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB) |
           (c.out_shift_right ? PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS : 0u) | (c.in_shift_right ? PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS : 0u) |
           (c.autopull ? PIO_SM0_SHIFTCTRL_AUTOPULL_BITS : 0u) | (c.autopush ? PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS : 0u);
}
constexpr uint32_t pio_pinctrl_of(const PioSmConfig& c) {
    return (static_cast<uint32_t>(c.side.count) << PIO_SM0_PINCTRL_SIDESET_COUNT_LSB) |
           (static_cast<uint32_t>(c.set_count) << PIO_SM0_PINCTRL_SET_COUNT_LSB) |
           (static_cast<uint32_t>(c.out_count) << PIO_SM0_PINCTRL_OUT_COUNT_LSB) |
           (static_cast<uint32_t>(c.in_base) << PIO_SM0_PINCTRL_IN_BASE_LSB) |
           (static_cast<uint32_t>(c.sideset_base) << PIO_SM0_PINCTRL_SIDESET_BASE_LSB) |
           (static_cast<uint32_t>(c.set_base) << PIO_SM0_PINCTRL_SET_BASE_LSB) |
           (static_cast<uint32_t>(c.out_base) << PIO_SM0_PINCTRL_OUT_BASE_LSB);
}

/// The twelve sources of a block's interrupt line (INTR / IRQn_INTE).
struct PioInterrupt {
    static constexpr uint32_t rx_not_empty(uint8_t sm) { return 1u << (sm & 3u); }
    static constexpr uint32_t tx_not_full(uint8_t sm) { return 1u << (4u + (sm & 3u)); }
    /// Flag 0..3 raised by a machine.
    static constexpr uint32_t flag(uint8_t f) { return 1u << (8u + (f & 3u)); }
    static constexpr uint32_t all = 0xFFFu;
};

// =============================================================================
// The block
// =============================================================================

template <uint8_t n>
struct Pio {
    static_assert(n < 2, "the RP2040 has PIO0 and PIO1");
    Pio() = delete;

    static constexpr uint8_t index = n;
    static constexpr uint32_t base = n == 0 ? PIO0_BASE : PIO1_BASE;
    static constexpr uint32_t reset_bit = n == 0 ? ResetBlock::pio0 : ResetBlock::pio1;
    static constexpr PinFunction pin_function = n == 0 ? PinFunction::pio0 : PinFunction::pio1;
    static constexpr IRQn_Type irq(uint8_t line) {
        return n == 0 ? (line == 0 ? PIO0_IRQ_0_IRQn : PIO0_IRQ_1_IRQn) : (line == 0 ? PIO1_IRQ_0_IRQn : PIO1_IRQ_1_IRQn);
    }
    static constexpr Dreq dreq_tx(uint8_t sm) { return static_cast<Dreq>(static_cast<uint8_t>(n == 0 ? Dreq::pio0_tx0 : Dreq::pio1_tx0) + (sm & 3u)); }
    static constexpr Dreq dreq_rx(uint8_t sm) { return static_cast<Dreq>(static_cast<uint8_t>(n == 0 ? Dreq::pio0_rx0 : Dreq::pio1_rx0) + (sm & 3u)); }

    static volatile uint32_t& reg(uint32_t offset) { return reg_at(base, offset); }

    /// The block from its reset state: every machine stopped, the
    /// memory as it was (the memory is not cleared by the reset), the
    /// placer's map cleared.
    static bool reset() {
        used_ = 0;
        return Resets::cycle(reset_bit);
    }
    static void hold() { Resets::hold(reset_bit); }

    // ---- the instruction memory ---------------------------------------------------

    /// Write `program` at `offset`, its JMPs relocated; the slots are
    /// marked used. Refused past the memory or over a used slot.
    template <uint8_t N>
    static bool load(const PioProgram<N>& program, uint8_t offset) {
        if (!program.valid() || offset + N > pio_instruction_count) {
            return false;
        }
        const uint32_t mask = static_cast<uint32_t>(((1ULL << N) - 1u) << offset);
        if ((used_ & mask) != 0u) {
            return false;
        }
        for (uint8_t i = 0; i < N; ++i) {
            reg(PIO_INSTR_MEM0_OFFSET + 4u * (offset + i)) = pio_relocate(program.code[i], offset);
        }
        used_ |= mask;
        return true;
    }
    /// The first free run of N slots, or nullopt.
    template <uint8_t N>
    static std::optional<uint8_t> place(const PioProgram<N>&) {
        for (uint8_t offset = 0; offset + N <= pio_instruction_count; ++offset) {
            const uint32_t mask = static_cast<uint32_t>(((1ULL << N) - 1u) << offset);
            if ((used_ & mask) == 0u) {
                return offset;
            }
        }
        return {};
    }
    /// place() then load(): the offset the program landed at.
    template <uint8_t N>
    static std::optional<uint8_t> add(const PioProgram<N>& program) {
        const auto at = place(program);
        if (!at || !load(program, *at)) {
            return {};
        }
        return at;
    }
    static void unload(uint8_t offset, uint8_t length) {
        used_ &= ~static_cast<uint32_t>(((1ULL << length) - 1u) << offset);
    }
    static uint32_t used_slots() { return used_; }

    // ---- the machines together (CTRL) ---------------------------------------------------
    static void enable(uint8_t mask) { hw_set(reg(PIO_CTRL_OFFSET), mask & 0xFu); }
    static void disable(uint8_t mask) { hw_clear(reg(PIO_CTRL_OFFSET), mask & 0xFu); }
    static uint8_t enabled() { return static_cast<uint8_t>(reg(PIO_CTRL_OFFSET) & PIO_CTRL_SM_ENABLE_BITS); }
    /// SM_RESTART: the machine's internal state cleared (the shift
    /// counters, the stalls, the delays), the PC and the FIFOs left.
    static void restart(uint8_t mask) { hw_set(reg(PIO_CTRL_OFFSET), static_cast<uint32_t>(mask & 0xFu) << PIO_CTRL_SM_RESTART_LSB); }
    /// CLKDIV_RESTART: the dividers of `mask` restarted from phase 0
    /// together - machines with one divisor then run in lockstep.
    static void restart_clocks(uint8_t mask) {
        hw_set(reg(PIO_CTRL_OFFSET), static_cast<uint32_t>(mask & 0xFu) << PIO_CTRL_CLKDIV_RESTART_LSB);
    }

    // ---- the status words ---------------------------------------------------------------
    static uint32_t fifo_status() { return reg(PIO_FSTAT_OFFSET); }
    static uint32_t fifo_debug() { return reg(PIO_FDEBUG_OFFSET); }
    static void clear_fifo_debug(uint32_t mask) { reg(PIO_FDEBUG_OFFSET) = mask; }
    static uint32_t fifo_levels() { return reg(PIO_FLEVEL_OFFSET); }
    /// DBG_CFGINFO: what the silicon was built with.
    static uint8_t memory_size() { return static_cast<uint8_t>((reg(PIO_DBG_CFGINFO_OFFSET) & PIO_DBG_CFGINFO_IMEM_SIZE_BITS) >> PIO_DBG_CFGINFO_IMEM_SIZE_LSB); }
    static uint8_t machine_count() { return static_cast<uint8_t>((reg(PIO_DBG_CFGINFO_OFFSET) & PIO_DBG_CFGINFO_SM_COUNT_BITS) >> PIO_DBG_CFGINFO_SM_COUNT_LSB); }
    static uint8_t fifo_depth() { return static_cast<uint8_t>(reg(PIO_DBG_CFGINFO_OFFSET) & PIO_DBG_CFGINFO_FIFO_DEPTH_BITS); }
    /// The synchronizer bypass, one bit per GPIO.
    static void sync_bypass(uint32_t mask) { reg(PIO_INPUT_SYNC_BYPASS_OFFSET) = mask; }

    // ---- the eight flags ----------------------------------------------------------------
    static uint8_t flags() { return static_cast<uint8_t>(reg(PIO_IRQ_OFFSET) & 0xFFu); }
    static bool flag(uint8_t f) { return (flags() & (1u << (f & 7u))) != 0u; }
    /// Write-one-to-clear.
    static void clear_flags(uint8_t mask) { reg(PIO_IRQ_OFFSET) = mask; }
    /// IRQ_FORCE: raised as a machine would, visible to the machines.
    static void raise_flags(uint8_t mask) { reg(PIO_IRQ_FORCE_OFFSET) = mask; }

    // ---- the two interrupt lines --------------------------------------------------------
    static volatile uint32_t& inte(uint8_t line) { return reg(line == 0 ? PIO_IRQ0_INTE_OFFSET : PIO_IRQ1_INTE_OFFSET); }
    static volatile uint32_t& intf(uint8_t line) { return reg(line == 0 ? PIO_IRQ0_INTF_OFFSET : PIO_IRQ1_INTF_OFFSET); }
    static volatile uint32_t& ints(uint8_t line) { return reg(line == 0 ? PIO_IRQ0_INTS_OFFSET : PIO_IRQ1_INTS_OFFSET); }
    static void interrupts(uint8_t line, uint32_t mask, bool on) {
        if (on) { hw_set(inte(line), mask & PioInterrupt::all); } else { hw_clear(inte(line), mask & PioInterrupt::all); }
    }
    static void interrupts_only(uint8_t line, uint32_t mask) { inte(line) = mask & PioInterrupt::all; }
    static uint32_t raw_pending() { return reg(PIO_INTR_OFFSET) & PioInterrupt::all; }
    static uint32_t pending(uint8_t line) { return ints(line) & PioInterrupt::all; }
    static void force(uint8_t line, uint32_t mask, bool on) {
        if (on) { hw_set(intf(line), mask & PioInterrupt::all); } else { hw_clear(intf(line), mask & PioInterrupt::all); }
    }
    /// The ISR body of `line`: the raised-and-enabled sources; a flag
    /// source is cleared here (the FIFO sources clear by moving data).
    [[gnu::always_inline]] static uint32_t isr(uint8_t line) {
        const uint32_t up = pending(line);
        if ((up & (PioInterrupt::flag(0) | PioInterrupt::flag(1) | PioInterrupt::flag(2) | PioInterrupt::flag(3))) != 0u) {
            clear_flags(static_cast<uint8_t>((up >> 8) & 0xFu));
        }
        return up;
    }

private:
    static inline uint32_t used_ = 0;
};

// =============================================================================
// The resource: one machine
// =============================================================================

template <uint8_t n, uint8_t sm>
struct PioSm {
    static_assert(sm < pio_sm_count, "a PIO block has four state machines, 0..3");
    PioSm() = delete;

    using Block = Pio<n>;
    static constexpr uint8_t index = sm;
    static constexpr uint8_t bit = static_cast<uint8_t>(1u << sm);
    static constexpr uint32_t stride = PIO_SM1_CLKDIV_OFFSET - PIO_SM0_CLKDIV_OFFSET;
    static constexpr Dreq dreq_tx = Block::dreq_tx(sm);
    static constexpr Dreq dreq_rx = Block::dreq_rx(sm);

    static volatile uint32_t& clkdiv() { return Block::reg(PIO_SM0_CLKDIV_OFFSET + stride * sm); }
    static volatile uint32_t& execctrl() { return Block::reg(PIO_SM0_EXECCTRL_OFFSET + stride * sm); }
    static volatile uint32_t& shiftctrl() { return Block::reg(PIO_SM0_SHIFTCTRL_OFFSET + stride * sm); }
    static volatile uint32_t& pinctrl() { return Block::reg(PIO_SM0_PINCTRL_OFFSET + stride * sm); }
    static volatile uint32_t& txf() { return Block::reg(PIO_TXF0_OFFSET + 4u * sm); }
    static volatile uint32_t& rxf() { return Block::reg(PIO_RXF0_OFFSET + 4u * sm); }
    static volatile void* tx_address() { return &txf(); }
    static volatile void* rx_address() { return &rxf(); }
    /// The top byte of an RX entry, for a byte-wide reader of a
    /// right-shifted eight-bit word (3.6.4).
    static volatile void* rx_top_byte_address() {
        return reinterpret_cast<volatile uint8_t*>(&rxf()) + 3;
    }

    /// The four registers written, the machine DISABLED first (the
    /// caller enables); refused for a value out of range.
    static bool configure(const PioSmConfig& c) {
        if (!pio_sm_config_valid(c)) {
            return false;
        }
        Block::disable(bit);
        clkdiv() = c.clock.reg();
        execctrl() = pio_execctrl_of(c);
        shiftctrl() = pio_shiftctrl_of(c);
        pinctrl() = pio_pinctrl_of(c);
        return true;
    }
    /// The whole start: configured with the program's wrap points at
    /// `offset`, restarted, its divider restarted, the FIFOs drained,
    /// the PC put at `offset` (a JMP executed) - still disabled.
    template <uint8_t N>
    static bool init(const PioProgram<N>& program, uint8_t offset, PioSmConfig c) {
        c.wrap_bottom = static_cast<uint8_t>(offset + program.wrap_bottom);
        c.wrap_top = static_cast<uint8_t>(offset + program.wrap_top);
        c.side = program.side;
        if (!configure(c)) {
            return false;
        }
        Block::restart(bit);
        Block::restart_clocks(bit);
        drain_tx();
        drain_rx();
        exec(pio_jmp(PioJmp::always, offset));
        return true;
    }
    static void enable(bool on) {
        if (on) { Block::enable(bit); } else { Block::disable(bit); }
    }
    static bool enabled() { return (Block::enabled() & bit) != 0u; }
    static void restart() { Block::restart(bit); }
    static void clock(PioClockDiv d) { clkdiv() = d.reg(); }

    // ---- an instruction now ------------------------------------------------------------
    /// SMx_INSTR: executed at once, the program resumed after; a JMP
    /// moves the PC. Refused while a previous one still stalls.
    static bool exec(PioInstr i) {
        if (exec_stalled()) {
            return false;
        }
        Block::reg(PIO_SM0_INSTR_OFFSET + stride * sm) = i;
        return true;
    }
    /// The same, waited for (bounded): true once it completed.
    static bool exec_wait(PioInstr i, uint32_t spins = 100'000u) {
        if (!exec(i)) {
            return false;
        }
        while (exec_stalled() && spins-- != 0u) {
        }
        return !exec_stalled();
    }
    static bool exec_stalled() { return (execctrl() & PIO_SM0_EXECCTRL_EXEC_STALLED_BITS) != 0u; }
    /// The PC, and the instruction under it.
    static uint8_t address() { return static_cast<uint8_t>(Block::reg(PIO_SM0_ADDR_OFFSET + stride * sm) & 0x1Fu); }
    static PioInstr instruction() { return static_cast<PioInstr>(Block::reg(PIO_SM0_INSTR_OFFSET + stride * sm) & 0xFFFFu); }

    /// The pins `base`..`base + count - 1` set as outputs or inputs, and
    /// driven, through a SET executed on the side with PINCTRL borrowed
    /// for the moment - the vendor's way to claim pins for a program
    /// before it runs. Up to five at a time; more take several.
    static void pin_directions(uint8_t base, uint8_t count, bool output) {
        set_range(base, count, PioSetTo::pindirs, output ? 0x1Fu : 0u);
    }
    static void pin_levels(uint8_t base, uint8_t count, bool high) {
        set_range(base, count, PioSetTo::pins, high ? 0x1Fu : 0u);
    }

    // ---- the FIFOs --------------------------------------------------------------------------
    static bool tx_full() { return (Block::fifo_status() & (PIO_FSTAT_TXFULL_BITS & (1u << (16u + sm)))) != 0u; }
    static bool tx_empty() { return (Block::fifo_status() & (1u << (24u + sm))) != 0u; }
    static bool rx_full() { return (Block::fifo_status() & (1u << sm)) != 0u; }
    static bool rx_empty() { return (Block::fifo_status() & (1u << (8u + sm))) != 0u; }
    static uint8_t tx_level() { return static_cast<uint8_t>((Block::fifo_levels() >> (8u * sm)) & 0xFu); }
    static uint8_t rx_level() { return static_cast<uint8_t>((Block::fifo_levels() >> (8u * sm + 4u)) & 0xFu); }
    static void push(uint32_t word) { txf() = word; }
    static uint32_t pop() { return rxf(); }
    /// Blocking, bounded: false when the FIFO stayed full / empty.
    static bool push_wait(uint32_t word, uint32_t spins = 1'000'000u) {
        while (tx_full() && spins-- != 0u) {
        }
        if (tx_full()) {
            return false;
        }
        push(word);
        return true;
    }
    static std::optional<uint32_t> pop_wait(uint32_t spins = 1'000'000u) {
        while (rx_empty() && spins-- != 0u) {
        }
        if (rx_empty()) {
            return {};
        }
        return pop();
    }
    static void drain_rx() {
        while (!rx_empty()) {
            (void)pop();
        }
    }
    /// The TX FIFO emptied the vendor's way: one word consumed per
    /// instruction executed on the side - an OUT NULL, 32 under
    /// autopull, a non-blocking PULL otherwise -, the machine disabled
    /// meanwhile; bounded.
    static void drain_tx() {
        const bool was = enabled();
        Block::disable(bit);
        const bool autopull = (shiftctrl() & PIO_SM0_SHIFTCTRL_AUTOPULL_BITS) != 0u;
        const PioInstr eat = autopull ? pio_out(PioOut::null, 32) : pio_pull(false, false);
        for (uint8_t spins = 0; !tx_empty() && spins < 64u; ++spins) {
            (void)exec_wait(eat, 1000);
        }
        if (was) {
            Block::enable(bit);
        }
    }
    /// FDEBUG's four sticky flags of this machine, and their clear.
    static bool tx_stalled() { return (Block::fifo_debug() & (1u << (24u + sm))) != 0u; }
    static bool tx_overflowed() { return (Block::fifo_debug() & (1u << (16u + sm))) != 0u; }
    static bool rx_underflowed() { return (Block::fifo_debug() & (1u << (8u + sm))) != 0u; }
    static bool rx_stalled() { return (Block::fifo_debug() & (1u << sm)) != 0u; }
    static void clear_fifo_debug() { Block::clear_fifo_debug((1u << (24u + sm)) | (1u << (16u + sm)) | (1u << (8u + sm)) | (1u << sm)); }

    /// The DMA requests of this machine (table 119).
    static constexpr uint32_t interrupt_rx_not_empty = PioInterrupt::rx_not_empty(sm);
    static constexpr uint32_t interrupt_tx_not_full = PioInterrupt::tx_not_full(sm);

private:
    static void set_range(uint8_t base, uint8_t count, PioSetTo what, uint8_t value) {
        const uint32_t saved = pinctrl();
        while (count > 0u) {
            const uint8_t now = count > 5u ? 5u : count;
            pinctrl() = (static_cast<uint32_t>(now) << PIO_SM0_PINCTRL_SET_COUNT_LSB) |
                        (static_cast<uint32_t>(base & 0x1Fu) << PIO_SM0_PINCTRL_SET_BASE_LSB);
            (void)exec_wait(pio_set(what, value));
            base = static_cast<uint8_t>(base + now);
            count = static_cast<uint8_t>(count - now);
        }
        pinctrl() = saved;
    }
};

// =============================================================================
// The chapter's own programs, as tasks
// =============================================================================

/// 3.6.3: an 8n1 transmitter, one bit every eight machine cycles, the
/// line high while idle (side-set on the pin), stalled on an empty
/// FIFO with the line idle.
inline constexpr PioProgram<4> pio_uart_tx_program = [] {
    constexpr PioSideSet s{.count = 2, .optional = true};
    PioProgram<4> p{};
    p.side = s;
    p.code = {
        pio_side_delay(pio_pull(false, true), s, 1, 7),            // pull side 1 [7]: the stop bit, or idle
        pio_side_delay(pio_set(PioSetTo::x, 7), s, 0, 7),          // set x, 7 side 0 [7]: the start bit
        pio_out(PioOut::pins, 1),                                  // bitloop: out pins, 1
        pio_delay(pio_jmp(PioJmp::x_dec, 2), 6, s),                // jmp x-- bitloop [6]
    };
    return p;
}();

/// 3.6.4: an 8n1 receiver with framing: the start bit waited for, eight
/// samples at the bit's middle, the stop bit checked on JMP PIN, a
/// frame that fails raising flag 4 + sm and the line waited idle.
inline constexpr PioProgram<9> pio_uart_rx_program = [] {
    PioProgram<9> p{};
    p.code = {
        pio_wait(false, PioWaitOn::pin, 0),                        // start: wait 0 pin 0
        pio_delay(pio_set(PioSetTo::x, 7), 10),                    // set x, 7 [10]
        pio_in(PioIn::pins, 1),                                    // bitloop: in pins, 1
        pio_delay(pio_jmp(PioJmp::x_dec, 2), 6),                   // jmp x-- bitloop [6]
        pio_jmp(PioJmp::pin, 8),                                   // jmp pin good_stop
        pio_irq(4, false, false, true),                            // irq 4 rel
        pio_wait(true, PioWaitOn::pin, 0),                         // wait 1 pin 0
        pio_jmp(PioJmp::always, 0),                                // jmp start
        pio_push(false, true),                                     // good_stop: push
    };
    return p;
}();

/// 3.2.1's square wave: the pin high two cycles, low two - a period of
/// four machine cycles.
inline constexpr PioProgram<3> pio_square_wave_program = [] {
    PioProgram<3> p{};
    p.code = {
        pio_delay(pio_set(PioSetTo::pins, 1), 1),                  // set pins, 1 [1]
        pio_set(PioSetTo::pins, 0),                                // set pins, 0
        pio_jmp(PioJmp::always, 0),                                // jmp again
    };
    return p;
}();

/// 3.6.8: a PWM counted by the machine - the period in the ISR, the
/// level pulled from the FIFO or kept when none arrives, the pin on
/// the side-set.
inline constexpr PioProgram<7> pio_pwm_program = [] {
    constexpr PioSideSet s{.count = 2, .optional = true};
    PioProgram<7> p{};
    p.side = s;
    p.code = {
        pio_side_delay(pio_pull(false, false), s, 0),              // pull noblock side 0
        pio_mov(PioMovTo::x, PioMovFrom::osr),                     // mov x, osr
        pio_mov(PioMovTo::y, PioMovFrom::isr),                     // mov y, isr
        pio_jmp(PioJmp::x_ne_y, 5),                                // countloop: jmp x != y noset
        pio_side_delay(pio_jmp(PioJmp::always, 6), s, 1),          // jmp skip side 1
        pio_nop(),                                                 // noset: nop
        pio_jmp(PioJmp::y_dec, 3),                                 // skip: jmp y-- countloop
    };
    return p;
}();

/**
 * PioUartTx<n, sm, pin>: a transmitter on any pin at eight machine
 * cycles a bit. The FIFO is joined into eight entries of one byte each;
 * the DMA request is `Sm::dreq_tx` on `tx_address()`.
 */
template <uint8_t n, uint8_t sm, uint8_t pin>
struct PioUartTx {
    PioUartTx() = delete;
    static_assert(pin < gpio_count, "brio PioUartTx: no such GPIO");
    using Block = Pio<n>;
    using Sm = PioSm<n, sm>;

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud) {
        const auto div = pio_clock_div_for(clock_hz(clock), baud * 8u);
        const auto at = Block::add(pio_uart_tx_program);
        if (!div || !at) {
            return false;
        }
        offset_ = *at;
        Sm::pin_levels(pin, 1, true);
        Sm::pin_directions(pin, 1, true);
        Pin<pin>::function(Block::pin_function);
        PioSmConfig c{};
        c.clock = *div;
        c.out_shift_right = true;
        c.autopull = false;
        c.pull_threshold = 32;
        c.out_base = pin;
        c.out_count = 1;
        c.sideset_base = pin;
        c.fifo_join = PioFifoJoin::tx;
        if (!Sm::init(pio_uart_tx_program, offset_, c)) {
            return false;
        }
        Sm::enable(true);
        return true;
    }
    static bool writable() { return !Sm::tx_full(); }
    static void write(uint8_t byte) { Sm::push(byte); }
    static bool write_wait(uint8_t byte) { return Sm::push_wait(byte); }
    static bool idle() { return Sm::tx_empty() && Sm::address() == offset_; }
    static volatile void* tx_address() { return Sm::tx_address(); }
    static void release() {
        Sm::enable(false);
        Block::unload(offset_, pio_uart_tx_program.length);
        Pin<pin>::release();
    }

private:
    static inline uint8_t offset_ = 0;
};

/**
 * PioUartRx<n, sm, pin>: a receiver on any pin, the byte in the top
 * eight bits of each RX entry (`read()` takes it), a frame without its
 * stop bit counted in flag 4 + sm (`frame_error()`), the FIFO joined
 * into eight; the DMA request is `Sm::dreq_rx` on
 * `rx_top_byte_address()` for a byte engine.
 */
template <uint8_t n, uint8_t sm, uint8_t pin>
struct PioUartRx {
    PioUartRx() = delete;
    static_assert(pin < gpio_count, "brio PioUartRx: no such GPIO");
    using Block = Pio<n>;
    using Sm = PioSm<n, sm>;
    static constexpr uint8_t frame_flag = static_cast<uint8_t>(4u + sm);

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, PinPull pull = PinPull::up) {
        const auto div = pio_clock_div_for(clock_hz(clock), baud * 8u);
        const auto at = Block::add(pio_uart_rx_program);
        if (!div || !at) {
            return false;
        }
        offset_ = *at;
        Sm::pin_directions(pin, 1, false);
        Pin<pin>::function(Block::pin_function, {.pull = pull});
        PioSmConfig c{};
        c.clock = *div;
        c.in_base = pin;
        c.jmp_pin = pin;
        c.in_shift_right = true;
        c.autopush = false;
        c.push_threshold = 32;
        c.fifo_join = PioFifoJoin::rx;
        if (!Sm::init(pio_uart_rx_program, offset_, c)) {
            return false;
        }
        Block::clear_flags(static_cast<uint8_t>(1u << frame_flag));
        Sm::enable(true);
        return true;
    }
    static bool readable() { return !Sm::rx_empty(); }
    static uint8_t read() { return static_cast<uint8_t>(Sm::pop() >> 24); }
    static std::optional<uint8_t> read_wait(uint32_t spins = 1'000'000u) {
        const auto w = Sm::pop_wait(spins);
        if (!w) {
            return {};
        }
        return static_cast<uint8_t>(*w >> 24);
    }
    /// A frame arrived without its stop bit (or the line broke): the
    /// flag the program raises, cleared here.
    static bool frame_error() {
        const bool up = Block::flag(frame_flag);
        if (up) {
            Block::clear_flags(static_cast<uint8_t>(1u << frame_flag));
        }
        return up;
    }
    static volatile void* rx_top_byte_address() { return Sm::rx_top_byte_address(); }
    static void release() {
        Sm::enable(false);
        Block::unload(offset_, pio_uart_rx_program.length);
        Pin<pin>::release();
    }

private:
    static inline uint8_t offset_ = 0;
};

/**
 * PioSquareWave<n, sm, pin>: a 50 % wave at a quarter of the machine's
 * clock - `hz` asked for, the divider solved for four times it.
 */
template <uint8_t n, uint8_t sm, uint8_t pin>
struct PioSquareWave {
    PioSquareWave() = delete;
    static_assert(pin < gpio_count, "brio PioSquareWave: no such GPIO");
    using Block = Pio<n>;
    using Sm = PioSm<n, sm>;

    template <typename Clock>
    static bool init(Clock clock, uint32_t hz) {
        const auto div = pio_clock_div_for(clock_hz(clock), hz * 4u);
        const auto at = Block::add(pio_square_wave_program);
        if (!div || !at) {
            return false;
        }
        offset_ = *at;
        Sm::pin_directions(pin, 1, true);
        Pin<pin>::function(Block::pin_function);
        PioSmConfig c{};
        c.clock = *div;
        c.set_base = pin;
        c.set_count = 1;
        if (!Sm::init(pio_square_wave_program, offset_, c)) {
            return false;
        }
        Sm::enable(true);
        return true;
    }
    static void release() {
        Sm::enable(false);
        Block::unload(offset_, pio_square_wave_program.length);
        Pin<pin>::release();
    }

private:
    static inline uint8_t offset_ = 0;
};

/**
 * PioPwm<n, sm, pin, period>: 3.6.8's PWM as a util/pwm_channel.hpp
 * PwmChannel - `max` = period, the level pulled from the FIFO once per
 * pulse and kept when none arrives. A pulse takes (period + 1) x 3
 * machine cycles (the count loop is three instructions).
 */
template <uint8_t n, uint8_t sm, uint8_t pin, uint16_t period = 0xFFFF>
struct PioPwm {
    PioPwm() = delete;
    static_assert(pin < gpio_count, "brio PioPwm: no such GPIO");
    static_assert(period > 0u, "a PWM period of zero has no duty to set");
    using Block = Pio<n>;
    using Sm = PioSm<n, sm>;
    static constexpr uint16_t max = period;
    static constexpr uint32_t cycles_per_pulse = (static_cast<uint32_t>(period) + 1u) * 3u;

    /// The machine at `divider`; the period loaded into the ISR through
    /// the FIFO and two instructions executed on the side (3.6.8).
    static bool init(PioClockDiv divider = {}) {
        const auto at = Block::add(pio_pwm_program);
        if (!at) {
            return false;
        }
        offset_ = *at;
        Sm::pin_directions(pin, 1, true);
        Pin<pin>::function(Block::pin_function);
        PioSmConfig c{};
        c.clock = divider;
        c.sideset_base = pin;
        if (!Sm::init(pio_pwm_program, offset_, c)) {
            return false;
        }
        Sm::push(period);
        (void)Sm::exec_wait(pio_pull(false, false));
        (void)Sm::exec_wait(pio_out(PioOut::isr, 32));
        Sm::push(0);
        Sm::enable(true);
        return true;
    }
    static void duty(uint16_t v) { Sm::push(v > max ? max : v); }
    static void release() {
        Sm::enable(false);
        Block::unload(offset_, pio_pwm_program.length);
        Pin<pin>::release();
    }

private:
    static inline uint8_t offset_ = 0;
};

// =============================================================================
// The chapter's own encodings, pinned at compile time
// =============================================================================

// Table 366's opcodes, and the examples' own words.
static_assert(pio_jmp(PioJmp::always, 0) == 0x0000u && pio_jmp(PioJmp::x_dec, 2) == 0x0042u);
static_assert(pio_wait(false, PioWaitOn::pin, 0) == 0x2020u && pio_wait(true, PioWaitOn::gpio, 15) == 0x208Fu);
static_assert(pio_in(PioIn::pins, 1) == 0x4001u && pio_in(PioIn::null, 32) == 0x4060u);
static_assert(pio_out(PioOut::pins, 1) == 0x6001u && pio_out(PioOut::isr, 32) == 0x60C0u);
static_assert(pio_push() == 0x8020u && pio_pull() == 0x80A0u && pio_pull(false, false) == 0x8080u);
static_assert(pio_mov(PioMovTo::x, PioMovFrom::osr) == 0xA027u && pio_nop() == 0xA042u);
static_assert(pio_irq(4, false, false, true) == 0xC014u && pio_irq(0, false, true) == 0xC020u && pio_irq(1, true) == 0xC041u);
static_assert(pio_set(PioSetTo::x, 7) == 0xE027u && pio_set(PioSetTo::pindirs, 1) == 0xE081u && pio_set(PioSetTo::pins, 0) == 0xE000u);
// The delay and the side-set: `set pins, 1 [1]` is 0xE101; the UART
// transmitter's `pull side 1 [7]` under an optional one-pin side-set
// (an enable bit, a data bit, three delay bits) is 0x9FA0 and its
// `set x, 7 side 0 [7]` 0xF727 - the vendor's assembled words.
static_assert(pio_delay(pio_set(PioSetTo::pins, 1), 1) == 0xE101u);
static_assert(pio_uart_tx_program.code[0] == 0x9FA0u && pio_uart_tx_program.code[1] == 0xF727u);
static_assert(pio_uart_tx_program.code[2] == 0x6001u && pio_uart_tx_program.code[3] == 0x0642u);
static_assert(pio_uart_rx_program.code[1] == 0xEA27u && pio_uart_rx_program.code[4] == 0x00C8u && pio_uart_rx_program.code[5] == 0xC014u);
static_assert(pio_relocate(pio_jmp(PioJmp::x_dec, 2), 10) == 0x004Cu && pio_relocate(pio_set(PioSetTo::x, 7), 10) == 0xE027u);
static_assert(pio_clock_div_for(125'000'000, 125'000'000)->integer == 1u && pio_clock_div_for(125'000'000, 921'600)->integer == 135u);
static_assert(pio_clock_div_for(125'000'000, 1'000'000'000) == std::nullopt);
static_assert(pio_sm_hz(125'000'000, {1, 0}) == 125'000'000u && pio_sm_hz(125'000'000, {2, 128}) == 50'000'000u);
static_assert(pio_uart_tx_program.valid() && pio_uart_rx_program.valid() && pio_square_wave_program.valid() && pio_pwm_program.valid());
static_assert(PwmChannel<PioPwm<0, 0, 17, 999>>);
static_assert(PioSm<0, 2>::dreq_rx == Dreq::pio0_rx2 && PioSm<1, 3>::dreq_tx == Dreq::pio1_tx3);

} // namespace brio
