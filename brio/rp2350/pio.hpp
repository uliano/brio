/*
 * pio.hpp
 *
 * The RP2350's programmable I/O (datasheet chapter 11): THREE blocks of
 * four STATE MACHINES each, executing sixteen-bit programs from a shared
 * memory of thirty-two instructions - nine instructions, one cycle each,
 * a delay of up to 31 cycles and a side-set of up to five pins folded
 * into every one -, each machine with two 32-bit scratch registers, an
 * output and an input shift register with their counters, a four-deep
 * FIFO each way (joinable into one of eight, or turned into four
 * randomly addressed REGISTERS), a 16.8 clock divider, four pin ranges
 * (OUT, SET, IN, side-set) and a pin to branch on, eight flags shared by
 * the machines of a block and reaching the two system interrupt lines,
 * and a DMA request per FIFO. Three layers, the other strata's
 * arrangement:
 *
 *  - THE ASSEMBLER: the nine instructions as constexpr encoders
 *    (`pio_jmp` .. `pio_set`, and this chip's `pio_put` / `pio_get`), the
 *    side-set and delay folded in by `pio_side_delay`, a program as
 *    `PioProgram<N>` with its wrap points and side-set shape, JMP
 *    addresses relocated at load. Every program is written here in these
 *    words - there is no pioasm in this tree.
 *  - `Pio<n>` is the BLOCK: the reset gate, the GPIO window, the
 *    instruction memory with a first-fit placer, the enables and restarts
 *    in lockstep (CTRL) reaching the NEIGHBOURING BLOCKS as well, the
 *    FIFO status and debug words, what the pads are being driven with,
 *    the eight flags, the two interrupt lines with their sixteen sources,
 *    the ISR body.
 *  - `PioSm<n, sm>` is the RESOURCE of one machine: its configuration
 *    (the divider, the execution controls, the shift controls, the pin
 *    ranges), the FIFOs and their register face, an instruction executed
 *    at once (SMx_INSTR), the pins claimed through a SET executed on the
 *    side.
 *  - THE TASKS are the chapter's own programs: `PioUartTx` and
 *    `PioUartRx` (11.6.3, 11.6.4: a serial port on any pin at eight
 *    cycles a bit), `PioSquareWave` (11.2.1's four-cycle wave), `PioPwm`
 *    (11.6.8: a util/pwm_channel.hpp PwmChannel counted by a program).
 *    Any other program is the application's, written with the encoders
 *    above.
 *
 * WHAT THIS CHIP ADDED, and why none of it is the RP2040's file retyped
 * (11.1.1 states the list; every item below has a verb here):
 *
 *  - A THIRD BLOCK. Twelve state machines, three interrupt line pairs,
 *    three sets of DMA requests, and PIO2 as a pin function of its own
 *    (F8). `Pio<n>` takes n < 3, and the three form a RING for the
 *    cross-block features below: the block after PIO2 is PIO0.
 *  - VERSION. DBG_CFGINFO.VERSION reads 1 here and read 0 on the RP2040,
 *    so a program can ask the silicon whether the features of this file
 *    exist at all. `Pio<n>::version()`.
 *  - GPIOBASE, and it is the one thing that changes the MEANING of every
 *    pin number in this file. A block still sees 32 GPIOs at a time, but
 *    which 32 is a register: 0 selects GPIO0..31, 16 selects GPIO16..47,
 *    and those two are the only values the bit accepts. Its own
 *    description is that it "relocates GPIO 0 FROM PIO'S POINT OF VIEW",
 *    so the four pin ranges in `PioSmConfig`, EXECCTRL's JMP_PIN and the
 *    index of a WAIT GPIO are all the BLOCK's own 0..31 and not the
 *    system's GPIO number - `Pio<n>::pin_index(gpio)` is the single
 *    conversion, and the tasks take system numbers and do it themselves,
 *    refusing a pad outside the window instead of driving the wrong one.
 *  - CTRL's NEIGHBOUR MASKS. One write to a block's CTRL can start, stop
 *    or restart the clock dividers of state machines in the block before
 *    and after it, ON THE SAME CYCLE as its own - which is the only way
 *    to put machines of different blocks in lockstep. `enable_across`,
 *    `disable_across`, `restart_clocks_across`.
 *  - ALL EIGHT FLAGS REACH THE LINES. On the RP2040 only flags 0..3 could
 *    be masked into an interrupt request; here IRQ0_INTE and IRQ1_INTE
 *    are sixteen bits wide and the upper eight are the eight flags, so
 *    `PioInterrupt::flag(f)` takes 0..7.
 *  - SHIFTCTRL.IN_COUNT masks the IN-mapped pins above a count to zero -
 *    which is what makes `MOV X, PINS` usable, that instruction having
 *    had no other way to say how many pins it meant.
 *  - THE RX FIFO AS FOUR REGISTERS. FJOIN_RX_PUT turns a machine's
 *    receive FIFO into four cells the machine writes in any order (the
 *    PUT instruction) and the system reads at RXFn_PUTGETm - a status
 *    register a program keeps up to date and no one blocks on.
 *    FJOIN_RX_GET is the mirror: the system writes, the machine reads.
 *    Both set together give the machine four more scratch registers and
 *    shut the system out. `PioFifoJoin` carries those three modes beside
 *    the two the RP2040 had.
 *  - NEW INSTRUCTION FORMS (11.4): WAIT on JMP_PIN plus an offset of 0..3
 *    (a wait whose pin is the machine's own branch pin and not the IN
 *    mapping); MOV to PINDIRS, which turns every OUT-mapped pin around in
 *    one instruction; the IRQ flags as a source for MOV X, STATUS, so a
 *    program can BRANCH on a flag instead of blocking on it; and an IRQ
 *    index that names a flag of the PREVIOUS or the NEXT block, with no
 *    cycle of delay between blocks. `PioIrqScope` is that index mode, and
 *    it is where the RP2040's single `relative` flag went: relative is
 *    one of its four values.
 *
 * THE PINS: any of the GPIOs the package bonds, under function 6 (PIO0),
 * 7 (PIO1) or 8 (PIO2) - every pin of this chip offers all three (9.4). A
 * machine's four ranges are bases and counts modulo 32 over the block's
 * window; on one cycle the highest-numbered machine writing a pin wins,
 * and a side-set beats an OUT or SET of the same machine (11.5.6.1). An
 * input passes a two-flip-flop synchronizer unless INPUT_SYNC_BYPASS says
 * otherwise.
 *
 * THE RELOCATION: a JMP carries an ABSOLUTE address (11.4.2), so a
 * program written from 0 is moved to its load offset by adding the offset
 * to every JMP's target - which is why programs here name their targets
 * from 0 and load() does the adding; an instruction executed through OUT
 * EXEC or MOV EXEC is the application's to relocate.
 *
 * THE FIFOS AND THE DMA: a word pushed to TXFx, a word popped from RXFx;
 * the request of each rises with room or data (12.6.4.1). PIO0's and
 * PIO1's sixteen requests keep the numbers they had on the RP2040 and
 * PIO2's eight are new behind them, which is what pushed every OTHER
 * peripheral's request up the table (rp2350/dma_engine.hpp). A byte-wide
 * read of RXFx + 3 takes the top byte of an entry, where a right-shifting
 * IN leaves eight bits (11.6.4).
 *
 * NO ERRATUM OF APPENDIX E NAMES THIS BLOCK. What does reach it is
 * RP2350-E9, which is the pads': a floating pad with its input buffer
 * enabled reads HIGH on stepping A2, so a program that watches an
 * undriven pin is watching the leak and not the wire (rp2350/pin.hpp).
 */

#pragma once

#include <stdint.h>
#include <array>
#include <optional>

#include "rp2350/device.hpp"

#include "rp2350/core.hpp"
#include "rp2350/dma_engine.hpp"
#include "rp2350/pin.hpp"
#include "rp2350/resets.hpp"
#include "util/clock.hpp"
#include "util/pwm_channel.hpp"

namespace brio {

// =============================================================================
// The assembler
// =============================================================================

using PioInstr = uint16_t;

/// This chip's PIO, as built (11.1, and DBG_CFGINFO reports each of them
/// back at run time).
inline constexpr uint8_t pio_block_count = 3;
inline constexpr uint8_t pio_instruction_count = 32;
inline constexpr uint8_t pio_sm_count = 4;
inline constexpr uint8_t pio_fifo_depth = 4;
inline constexpr uint8_t pio_flag_count = 8;
/// DBG_CFGINFO.VERSION: 0 was the RP2040's PIO, 1 is this one.
inline constexpr uint8_t pio_version = 1;

/// JMP's condition (11.4.2).
enum class PioJmp : uint8_t { always = 0, x_zero = 1, x_dec = 2, y_zero = 3, y_dec = 4, x_ne_y = 5, pin = 6, osr_not_empty = 7 };
/// WAIT's source (11.4.3). `jmppin` is this chip's own: the pin EXECCTRL's
/// JMP_PIN names, plus an offset of 0..3, which gives a WAIT a pin
/// mapping of its own independent of the IN range.
enum class PioWaitOn : uint8_t { gpio = 0, pin = 1, irq = 2, jmppin = 3 };
/// IN's source (11.4.4).
enum class PioIn : uint8_t { pins = 0, x = 1, y = 2, null = 3, isr = 6, osr = 7 };
/// OUT's destination (11.4.5).
enum class PioOut : uint8_t { pins = 0, x = 1, y = 2, null = 3, pindirs = 4, pc = 5, isr = 6, exec = 7 };
/// MOV's destination, operation and source (11.4.10). `pindirs` is this
/// chip's own (destination 3, reserved on the RP2040): MOV PINDIRS, NULL
/// and MOV PINDIRS, ~NULL turn every OUT-mapped pin around at once.
enum class PioMovTo : uint8_t { pins = 0, x = 1, y = 2, pindirs = 3, exec = 4, pc = 5, isr = 6, osr = 7 };
enum class PioMovOp : uint8_t { none = 0, invert = 1, reverse = 2 };
enum class PioMovFrom : uint8_t { pins = 0, x = 1, y = 2, null = 3, status = 5, isr = 6, osr = 7 };
/// SET's destination (11.4.12).
enum class PioSetTo : uint8_t { pins = 0, x = 1, y = 2, pindirs = 4 };

/// How an IRQ or a WAIT IRQ names its flag (11.4.11's IdxMode, the two
/// most significant bits of the index field). `relative` is the RP2040's
/// one modifier; the other two reach ACROSS BLOCKS, with no cycle of
/// delay, and wrap around the ring of three.
enum class PioIrqScope : uint8_t {
    here = 0,       ///< a flag of this block
    prev = 1,       ///< the block before this one (PIO0's previous is PIO2)
    relative = 2,   ///< this block, the machine's number added modulo four to the low two bits
    next = 3,       ///< the block after this one (PIO2's next is PIO0)
};

constexpr PioInstr pio_jmp(PioJmp cond, uint8_t address) {
    return static_cast<PioInstr>((0u << 13) | (static_cast<uint16_t>(cond) << 5) | (address & 0x1Fu));
}
constexpr PioInstr pio_wait(bool polarity, PioWaitOn on, uint8_t index) {
    return static_cast<PioInstr>((1u << 13) | (polarity ? 1u << 7 : 0u) | (static_cast<uint16_t>(on) << 5) | (index & 0x1Fu));
}
/// A WAIT on a flag, the scope in the index's two most significant bits.
constexpr PioInstr pio_wait_irq(bool polarity, uint8_t flag, PioIrqScope scope = PioIrqScope::here) {
    return pio_wait(polarity, PioWaitOn::irq, static_cast<uint8_t>((flag & 7u) | (static_cast<uint8_t>(scope) << 3)));
}
/// A WAIT on the machine's OWN branch pin (EXECCTRL's JMP_PIN) plus an
/// offset of 0..3 - the source this chip added.
constexpr PioInstr pio_wait_jmppin(bool polarity, uint8_t offset = 0) {
    return pio_wait(polarity, PioWaitOn::jmppin, static_cast<uint8_t>(offset & 3u));
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
/// PUT (11.4.8): the ISR written into RX FIFO entry `entry`, which wants
/// SHIFTCTRL's FJOIN_RX_PUT. The same opcode space as PUSH, told apart by
/// bit 4, which PUSH leaves clear. There are four entries and the
/// hardware indexes them by the field's TWO low bits, so 0..3 is the
/// range the chapter gives and anything above it aliases.
constexpr PioInstr pio_put(uint8_t entry) {
    return static_cast<PioInstr>((4u << 13) | (1u << 4) | (1u << 3) | (entry & 7u));
}
/// PUT with the entry taken from the two low bits of scratch Y.
constexpr PioInstr pio_put_y() { return static_cast<PioInstr>((4u << 13) | (1u << 4)); }
/// GET (11.4.9): RX FIFO entry `entry` read into the OSR, which wants
/// FJOIN_RX_GET. PULL's bit 7, and bit 4 again.
constexpr PioInstr pio_get(uint8_t entry) {
    return static_cast<PioInstr>((4u << 13) | (1u << 7) | (1u << 4) | (1u << 3) | (entry & 7u));
}
/// GET with the entry taken from the two low bits of scratch Y.
constexpr PioInstr pio_get_y() { return static_cast<PioInstr>((4u << 13) | (1u << 7) | (1u << 4)); }
constexpr PioInstr pio_mov(PioMovTo to, PioMovFrom from, PioMovOp op = PioMovOp::none) {
    return static_cast<PioInstr>((5u << 13) | (static_cast<uint16_t>(to) << 5) | (static_cast<uint16_t>(op) << 3) |
                                 static_cast<uint16_t>(from));
}
/// IRQ: set (wait = the machine stalls until the flag is cleared) or
/// clear the flag; `scope` names WHOSE flag (11.4.11).
constexpr PioInstr pio_irq(uint8_t flag, bool clear = false, bool wait = false, PioIrqScope scope = PioIrqScope::here) {
    return static_cast<PioInstr>((6u << 13) | (clear ? 1u << 6 : 0u) | (wait ? 1u << 5 : 0u) | (flag & 7u) |
                                 (static_cast<uint16_t>(static_cast<uint8_t>(scope)) << 3));
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

/// What a machine's pair of FIFOs is (SHIFTCTRL's four join bits). The
/// first three are the RP2040's; the last three are this chip's, and they
/// turn the receive FIFO's four cells into REGISTERS reached at random
/// instead of a queue (11.4.8, 11.4.9).
enum class PioFifoJoin : uint8_t {
    none,       ///< four entries each way
    tx,         ///< eight to the transmit side, no receive FIFO
    rx,         ///< eight to the receive side, no transmit FIFO
    rx_put,     ///< the machine writes the four cells (PUT), the system reads them
    rx_get,     ///< the system writes the four cells, the machine reads them (GET)
    rx_putget,  ///< the machine reads and writes them; the system cannot reach them
};

/// MOV x, STATUS's source (EXECCTRL.STATUS_SEL). `irq_flag` is this
/// chip's own: a flag as an all-ones or all-zeroes value, so a program
/// can BRANCH on a flag rather than block on it.
enum class PioStatus : uint8_t { tx_level = 0, rx_level = 1, irq_flag = 2 };

/// Whose flag MOV x, STATUS looks at when STATUS_SEL is `irq_flag`. These
/// are not the instruction's IdxMode codes - the field is STATUS_N and it
/// has no relative mode - so they are spelled apart on purpose.
enum class PioStatusIrq : uint8_t { here = 0x00, prev = 0x08, next = 0x10 };

/// The STATUS_N value that names flag `flag` of `where`.
constexpr uint8_t pio_status_irq(uint8_t flag, PioStatusIrq where = PioStatusIrq::here) {
    return static_cast<uint8_t>((flag & 7u) | static_cast<uint8_t>(where));
}

/// One machine's configuration - the four registers a program needs set
/// before it runs. EVERY PIN NUMBER HERE IS THE BLOCK'S OWN 0..31, which
/// is the system GPIO minus the block's GPIOBASE (`Pio<n>::pin_index`).
struct PioSmConfig {
    PioClockDiv clock{};
    uint8_t wrap_bottom = 0;          ///< ABSOLUTE addresses (init() adds the offset)
    uint8_t wrap_top = 31;
    PioSideSet side{};
    uint8_t jmp_pin = 0;              ///< EXECCTRL_JMP_PIN: the pin JMP PIN and WAIT JMPPIN use
    bool out_sticky = false;
    bool inline_out_enable = false;
    uint8_t out_enable_bit = 0;       ///< OUT_EN_SEL
    PioStatus status_select = PioStatus::tx_level;
    uint8_t status_n = 0;             ///< a FIFO level, or pio_status_irq()'s value
    PioFifoJoin fifo_join = PioFifoJoin::none;
    uint8_t pull_threshold = 32;      ///< 1..32
    uint8_t push_threshold = 32;      ///< 1..32
    bool out_shift_right = true;
    bool in_shift_right = true;
    bool autopull = false;
    bool autopush = false;
    uint8_t in_count = 32;            ///< 1..32: IN-mapped pins above this read 0
    uint8_t out_base = 0;
    uint8_t out_count = 0;            ///< 0..32
    uint8_t set_base = 0;
    uint8_t set_count = 5;            ///< 0..5
    uint8_t sideset_base = 0;
    uint8_t in_base = 0;
};

constexpr bool pio_sm_config_valid(const PioSmConfig& c) {
    const bool level_status = c.status_select != PioStatus::irq_flag;
    return c.clock.valid() && c.wrap_bottom < 32u && c.wrap_top < 32u && c.side.valid() && c.jmp_pin < 32u &&
           c.out_enable_bit < 32u && c.status_n < 32u && (!level_status || c.status_n <= 2u * pio_fifo_depth) &&
           c.pull_threshold >= 1u && c.pull_threshold <= 32u && c.push_threshold >= 1u && c.push_threshold <= 32u &&
           c.in_count >= 1u && c.in_count <= 32u && c.out_base < 32u && c.out_count <= 32u && c.set_base < 32u &&
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
    const bool rx = c.fifo_join == PioFifoJoin::rx;
    const bool tx = c.fifo_join == PioFifoJoin::tx;
    const bool put = c.fifo_join == PioFifoJoin::rx_put || c.fifo_join == PioFifoJoin::rx_putget;
    const bool get = c.fifo_join == PioFifoJoin::rx_get || c.fifo_join == PioFifoJoin::rx_putget;
    return (rx ? PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS : 0u) | (tx ? PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS : 0u) |
           (put ? PIO_SM0_SHIFTCTRL_FJOIN_RX_PUT_BITS : 0u) | (get ? PIO_SM0_SHIFTCTRL_FJOIN_RX_GET_BITS : 0u) |
           (static_cast<uint32_t>(c.pull_threshold & 0x1Fu) << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB) |
           (static_cast<uint32_t>(c.push_threshold & 0x1Fu) << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB) |
           (c.out_shift_right ? PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS : 0u) | (c.in_shift_right ? PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS : 0u) |
           (c.autopull ? PIO_SM0_SHIFTCTRL_AUTOPULL_BITS : 0u) | (c.autopush ? PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS : 0u) |
           (static_cast<uint32_t>(c.in_count & 0x1Fu) << PIO_SM0_SHIFTCTRL_IN_COUNT_LSB);
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

/// The SIXTEEN sources of a block's interrupt line (INTR / IRQn_INTE):
/// the four receive FIFOs, the four transmit FIFOs, and ALL EIGHT FLAGS -
/// where the RP2040 offered the lower four alone.
struct PioInterrupt {
    static constexpr uint32_t rx_not_empty(uint8_t sm) { return 1u << (sm & 3u); }
    static constexpr uint32_t tx_not_full(uint8_t sm) { return 1u << (4u + (sm & 3u)); }
    /// Flag 0..7 raised by a machine, or by IRQ_FORCE.
    static constexpr uint32_t flag(uint8_t f) { return 1u << (8u + (f & 7u)); }
    /// The eight flag sources together.
    static constexpr uint32_t all_flags = 0xFF00u;
    static constexpr uint32_t all = 0xFFFFu;
};

/// Which state machines of the block BEFORE and AFTER this one a CTRL
/// write reaches (CTRL's PREV_PIO_MASK and NEXT_PIO_MASK): four bits
/// each, and zero means the write is this block's alone.
struct PioNeighbours {
    uint8_t next = 0;
    uint8_t prev = 0;
    constexpr bool any() const { return (next | prev) != 0u; }
};

// =============================================================================
// The block
// =============================================================================

template <uint8_t n>
struct Pio {
    static_assert(n < pio_block_count, "the RP2350 has PIO0, PIO1 and PIO2 (the RP2040 had two)");
    Pio() = delete;

    static constexpr uint8_t index = n;
    /// The ring the cross-block features run round (11.4.11): the block
    /// after the last one is the first.
    static constexpr uint8_t next_index = static_cast<uint8_t>((n + 1u) % pio_block_count);
    static constexpr uint8_t prev_index = static_cast<uint8_t>((n + pio_block_count - 1u) % pio_block_count);

    static constexpr uint32_t base = n == 0 ? PIO0_BASE : (n == 1 ? PIO1_BASE : PIO2_BASE);
    static constexpr uint32_t reset_bit = n == 0 ? ResetBlock::pio0 : (n == 1 ? ResetBlock::pio1 : ResetBlock::pio2);
    static constexpr PinFunction pin_function =
        n == 0 ? PinFunction::pio0 : (n == 1 ? PinFunction::pio1 : PinFunction::pio2);
    static constexpr IRQn_Type irq(uint8_t line) {
        if constexpr (n == 0) {
            return line == 0 ? PIO0_IRQ_0_IRQn : PIO0_IRQ_1_IRQn;
        } else if constexpr (n == 1) {
            return line == 0 ? PIO1_IRQ_0_IRQn : PIO1_IRQ_1_IRQn;
        } else {
            return line == 0 ? PIO2_IRQ_0_IRQn : PIO2_IRQ_1_IRQn;
        }
    }
    static constexpr Dreq dreq_tx(uint8_t sm) {
        constexpr uint8_t first = n == 0 ? static_cast<uint8_t>(Dreq::pio0_tx0)
                                         : (n == 1 ? static_cast<uint8_t>(Dreq::pio1_tx0) : static_cast<uint8_t>(Dreq::pio2_tx0));
        return static_cast<Dreq>(first + (sm & 3u));
    }
    static constexpr Dreq dreq_rx(uint8_t sm) {
        constexpr uint8_t first = n == 0 ? static_cast<uint8_t>(Dreq::pio0_rx0)
                                         : (n == 1 ? static_cast<uint8_t>(Dreq::pio1_rx0) : static_cast<uint8_t>(Dreq::pio2_rx0));
        return static_cast<Dreq>(first + (sm & 3u));
    }

    static volatile uint32_t& reg(uint32_t offset) { return reg_at(base, offset); }

    /// The block from its reset state: every machine stopped, GPIOBASE
    /// back at zero, the memory as it was (the reset does not clear it),
    /// the placer's map cleared.
    static bool reset() {
        used_ = 0;
        return Resets::cycle(reset_bit);
    }
    static void hold() { Resets::hold(reset_bit); }

    // ---- the GPIO window (11.1.1) -------------------------------------------------
    //
    // A block sees 32 GPIOs at a time and GPIOBASE says which 32. Only 0
    // and 16 are writable values, so the two windows are GPIO0..31 and
    // GPIO16..47, and pins 16..31 are in both.

    static constexpr uint8_t gpio_window = 32;

    static uint8_t gpio_base() {
        return static_cast<uint8_t>(reg(PIO_GPIOBASE_OFFSET) & 0x10u);
    }
    /// Move the window. True when `base` is one of the two values the
    /// register accepts; nothing is written otherwise. EVERY MACHINE OF
    /// THE BLOCK follows, so this is a block-level decision and not a
    /// program's.
    static bool gpio_base(uint8_t base_gpio) {
        if (base_gpio != 0u && base_gpio != 16u) {
            return false;
        }
        reg(PIO_GPIOBASE_OFFSET) = base_gpio;
        return true;
    }
    /// The block's own index for a system GPIO: the number every pin
    /// field of PioSmConfig, JMP_PIN and WAIT GPIO is written in. Nothing
    /// for a pad this package has not got, or one outside the window.
    static std::optional<uint8_t> pin_index(uint8_t gpio) {
        if (!Gpio::bonded(gpio)) {
            return {};
        }
        const uint8_t b = gpio_base();
        if (gpio < b || gpio >= b + gpio_window) {
            return {};
        }
        return static_cast<uint8_t>(gpio - b);
    }
    /// Whether the block can reach this pad at all as it stands.
    static bool sees(uint8_t gpio) { return pin_index(gpio).has_value(); }

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
    /// SM_RESTART: the machine's internal state cleared - both shift
    /// counters, the contents of the INPUT shift register, the delay
    /// counter, the waiting-on-IRQ state, any stalled instruction written
    /// to SMx_INSTR or run by an EXEC, and a pin write OUT_STICKY was
    /// holding. THE PC, THE OSR, X AND Y AND THE FIFOS ARE LEFT, which is
    /// what makes this the way to drop a stall without losing a
    /// measurement a program left in a scratch register.
    static void restart(uint8_t mask) { hw_set(reg(PIO_CTRL_OFFSET), static_cast<uint32_t>(mask & 0xFu) << PIO_CTRL_SM_RESTART_LSB); }
    /// CLKDIV_RESTART: the dividers of `mask` restarted from phase 0
    /// together - machines with one divisor then run in lockstep.
    static void restart_clocks(uint8_t mask) {
        hw_set(reg(PIO_CTRL_OFFSET), static_cast<uint32_t>(mask & 0xFu) << PIO_CTRL_CLKDIV_RESTART_LSB);
    }

    // ---- and the machines of the neighbouring blocks with them (11.1.1) ------------------
    //
    // The three operations that have a NEXTPREV twin are all of them
    // SIMULTANEITY verbs: the point is that the enable, the disable or the
    // divider restart lands on the same cycle in two blocks, which nothing
    // else on this chip can arrange. So each is ONE bus write.
    //
    // Enable and the divider restart are pure ORs, so they go through the
    // SET alias and read nothing. A disable is not: the local SM_ENABLE
    // bits must go DOWN while the NEXTPREV bits go UP, and no single alias
    // does both - so that one reads CTRL and writes it whole, which is a
    // read-modify-write and wants no other context enabling a machine of
    // this block across it.

    static void enable_across(uint8_t mask, PioNeighbours nb) {
        hw_set(reg(PIO_CTRL_OFFSET), static_cast<uint32_t>(mask & 0xFu) | PIO_CTRL_NEXTPREV_SM_ENABLE_BITS | neighbour_bits(nb));
    }
    static void disable_across(uint8_t mask, PioNeighbours nb) {
        const uint32_t live = static_cast<uint32_t>(enabled()) & ~static_cast<uint32_t>(mask & 0xFu);
        reg(PIO_CTRL_OFFSET) = live | PIO_CTRL_NEXTPREV_SM_DISABLE_BITS | neighbour_bits(nb);
    }
    static void restart_clocks_across(uint8_t mask, PioNeighbours nb) {
        hw_set(reg(PIO_CTRL_OFFSET), (static_cast<uint32_t>(mask & 0xFu) << PIO_CTRL_CLKDIV_RESTART_LSB) |
                                         PIO_CTRL_NEXTPREV_CLKDIV_RESTART_BITS | neighbour_bits(nb));
    }

    // ---- the status words ---------------------------------------------------------------
    static uint32_t fifo_status() { return reg(PIO_FSTAT_OFFSET); }
    static uint32_t fifo_debug() { return reg(PIO_FDEBUG_OFFSET); }
    static void clear_fifo_debug(uint32_t mask) { reg(PIO_FDEBUG_OFFSET) = mask; }
    static uint32_t fifo_levels() { return reg(PIO_FLEVEL_OFFSET); }
    /// DBG_CFGINFO: what the silicon was built with. `version()` is this
    /// chip's addition and the way a program tells the two PIOs apart.
    static uint8_t version() {
        return static_cast<uint8_t>((reg(PIO_DBG_CFGINFO_OFFSET) & PIO_DBG_CFGINFO_VERSION_BITS) >> PIO_DBG_CFGINFO_VERSION_LSB);
    }
    static uint8_t memory_size() { return static_cast<uint8_t>((reg(PIO_DBG_CFGINFO_OFFSET) & PIO_DBG_CFGINFO_IMEM_SIZE_BITS) >> PIO_DBG_CFGINFO_IMEM_SIZE_LSB); }
    static uint8_t machine_count() { return static_cast<uint8_t>((reg(PIO_DBG_CFGINFO_OFFSET) & PIO_DBG_CFGINFO_SM_COUNT_BITS) >> PIO_DBG_CFGINFO_SM_COUNT_LSB); }
    static uint8_t fifo_depth() { return static_cast<uint8_t>(reg(PIO_DBG_CFGINFO_OFFSET) & PIO_DBG_CFGINFO_FIFO_DEPTH_BITS); }
    /// What the block is driving onto the pads of its window, level and
    /// direction: the one way to see a machine's output without a wire.
    static uint32_t pad_out() { return reg(PIO_DBG_PADOUT_OFFSET); }
    static uint32_t pad_oe() { return reg(PIO_DBG_PADOE_OFFSET); }
    /// The synchronizer bypass, one bit per pin of the window.
    static void sync_bypass(uint32_t mask) { reg(PIO_INPUT_SYNC_BYPASS_OFFSET) = mask; }
    static uint32_t sync_bypass() { return reg(PIO_INPUT_SYNC_BYPASS_OFFSET); }

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
        if ((up & PioInterrupt::all_flags) != 0u) {
            clear_flags(static_cast<uint8_t>((up >> 8) & 0xFFu));
        }
        return up;
    }

private:
    static constexpr uint32_t neighbour_bits(PioNeighbours nb) {
        return (static_cast<uint32_t>(nb.next & 0xFu) << PIO_CTRL_NEXT_PIO_MASK_LSB) |
               (static_cast<uint32_t>(nb.prev & 0xFu) << PIO_CTRL_PREV_PIO_MASK_LSB);
    }
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
    /// right-shifted eight-bit word (11.6.4).
    static volatile void* rx_top_byte_address() {
        return reinterpret_cast<volatile uint8_t*>(&rxf()) + 3;
    }
    /// The receive FIFO's four cells as REGISTERS (11.4.8, 11.4.9): the
    /// system's end of FJOIN_RX_PUT (read) and FJOIN_RX_GET (write). With
    /// neither join bit set, or with both, this address answers nothing
    /// useful - the storage belongs to the queue, or to the machine.
    static volatile uint32_t& putget(uint8_t entry) {
        return Block::reg(PIO_RXF0_PUTGET0_OFFSET + 16u * sm + 4u * (entry & 3u));
    }

    /// The four registers written, the machine DISABLED first (the caller
    /// enables); refused for a value out of range. CHANGING THE JOIN
    /// DISCARDS WHAT THE FIFOS HELD (11.5.3).
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
    /// `offset`, restarted, its divider restarted, the FIFOs drained, the
    /// PC put at `offset` (a JMP executed) - still disabled.
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
    /// SMx_INSTR: executed at once, the program resumed after; a JMP moves
    /// the PC. A DISABLED machine executes these and nothing else, which
    /// is what makes them the way to set a machine up. Refused while a
    /// previous one still stalls.
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

    /// The pins `base`..`base + count - 1` OF THE BLOCK'S WINDOW set as
    /// outputs or inputs, and driven, through a SET executed on the side
    /// with PINCTRL borrowed for the moment - the vendor's way to claim
    /// pins for a program before it runs. Up to five at a time; more take
    /// several. `base` is what `Pio<n>::pin_index` returns, not a system
    /// GPIO number.
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
    /// instruction executed on the side - an OUT NULL, 32 under autopull,
    /// a non-blocking PULL otherwise -, the machine disabled meanwhile;
    /// bounded.
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

    /// This machine's two interrupt sources (11.7's IRQ0_INTE).
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

/// 11.6.3: an 8n1 transmitter, one bit every eight machine cycles, the
/// line high while idle (side-set on the pin), stalled on an empty FIFO
/// with the line idle.
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

/// 11.6.4: an 8n1 receiver with framing: the start bit waited for, eight
/// samples at the bit's middle, the stop bit checked on JMP PIN, a frame
/// that fails raising flag 4 + sm and the line waited idle.
inline constexpr PioProgram<9> pio_uart_rx_program = [] {
    PioProgram<9> p{};
    p.code = {
        pio_wait(false, PioWaitOn::pin, 0),                        // start: wait 0 pin 0
        pio_delay(pio_set(PioSetTo::x, 7), 10),                    // set x, 7 [10]
        pio_in(PioIn::pins, 1),                                    // bitloop: in pins, 1
        pio_delay(pio_jmp(PioJmp::x_dec, 2), 6),                   // jmp x-- bitloop [6]
        pio_jmp(PioJmp::pin, 8),                                   // jmp pin good_stop
        pio_irq(4, false, false, PioIrqScope::relative),           // irq 4 rel
        pio_wait(true, PioWaitOn::pin, 0),                         // wait 1 pin 0
        pio_jmp(PioJmp::always, 0),                                // jmp start
        pio_push(false, true),                                     // good_stop: push
    };
    return p;
}();

/// 11.2.1's square wave: the pin high two cycles, low two - a period of
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

/// 11.6.8: a PWM counted by the machine - the period in the ISR, the
/// level pulled from the FIFO or kept when none arrives, the pin on the
/// side-set.
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
 * PioUartTx<n, sm, pin>: a transmitter on any pin at eight machine cycles
 * a bit. The FIFO is joined into eight entries of one byte each; the DMA
 * request is `Sm::dreq_tx` on `tx_address()`.
 *
 * `pin` is a SYSTEM GPIO number and init() converts it through the
 * block's GPIO window, refusing a pad the window does not cover.
 */
template <uint8_t n, uint8_t sm, uint8_t pin>
struct PioUartTx {
    PioUartTx() = delete;
    static_assert(pin < gpio_count, "brio PioUartTx: no such GPIO in this package");
    using Block = Pio<n>;
    using Sm = PioSm<n, sm>;

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud) {
        const auto div = pio_clock_div_for(clock_hz(clock), baud * 8u);
        const auto at_pin = Block::pin_index(pin);
        if (!div || !at_pin) {
            return false;
        }
        const auto at = Block::add(pio_uart_tx_program);
        if (!at) {
            return false;
        }
        offset_ = *at;
        Sm::pin_levels(*at_pin, 1, true);
        Sm::pin_directions(*at_pin, 1, true);
        Pin<pin>::function(Block::pin_function);
        PioSmConfig c{};
        c.clock = *div;
        c.out_shift_right = true;
        c.autopull = false;
        c.pull_threshold = 32;
        c.out_base = *at_pin;
        c.out_count = 1;
        c.sideset_base = *at_pin;
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
 * PioUartRx<n, sm, pin>: a receiver on any pin, the byte in the top eight
 * bits of each RX entry (`read()` takes it), a frame without its stop bit
 * counted in flag 4 + sm (`frame_error()`), the FIFO joined into eight;
 * the DMA request is `Sm::dreq_rx` on `rx_top_byte_address()` for a byte
 * engine.
 */
template <uint8_t n, uint8_t sm, uint8_t pin>
struct PioUartRx {
    PioUartRx() = delete;
    static_assert(pin < gpio_count, "brio PioUartRx: no such GPIO in this package");
    using Block = Pio<n>;
    using Sm = PioSm<n, sm>;
    static constexpr uint8_t frame_flag = static_cast<uint8_t>(4u + sm);

    template <typename Clock>
    static bool init(Clock clock, uint32_t baud, PinPull pull = PinPull::up) {
        const auto div = pio_clock_div_for(clock_hz(clock), baud * 8u);
        const auto at_pin = Block::pin_index(pin);
        if (!div || !at_pin) {
            return false;
        }
        const auto at = Block::add(pio_uart_rx_program);
        if (!at) {
            return false;
        }
        offset_ = *at;
        Sm::pin_directions(*at_pin, 1, false);
        Pin<pin>::function(Block::pin_function, {.pull = pull});
        PioSmConfig c{};
        c.clock = *div;
        c.in_base = *at_pin;
        c.jmp_pin = *at_pin;
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
    /// A frame arrived without its stop bit (or the line broke): the flag
    /// the program raises, cleared here.
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
    static_assert(pin < gpio_count, "brio PioSquareWave: no such GPIO in this package");
    using Block = Pio<n>;
    using Sm = PioSm<n, sm>;

    template <typename Clock>
    static bool init(Clock clock, uint32_t hz) {
        const auto div = pio_clock_div_for(clock_hz(clock), hz * 4u);
        const auto at_pin = Block::pin_index(pin);
        if (!div || !at_pin) {
            return false;
        }
        const auto at = Block::add(pio_square_wave_program);
        if (!at) {
            return false;
        }
        offset_ = *at;
        Sm::pin_directions(*at_pin, 1, true);
        Pin<pin>::function(Block::pin_function);
        PioSmConfig c{};
        c.clock = *div;
        c.set_base = *at_pin;
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
 * PioPwm<n, sm, pin, period>: 11.6.8's PWM as a util/pwm_channel.hpp
 * PwmChannel - `max` = period, the level pulled from the FIFO once per
 * pulse and kept when none arrives. A pulse takes (period + 1) x 3
 * machine cycles (the count loop is three instructions).
 */
template <uint8_t n, uint8_t sm, uint8_t pin, uint16_t period = 0xFFFF>
struct PioPwm {
    PioPwm() = delete;
    static_assert(pin < gpio_count, "brio PioPwm: no such GPIO in this package");
    static_assert(period > 0u, "a PWM period of zero has no duty to set");
    using Block = Pio<n>;
    using Sm = PioSm<n, sm>;
    static constexpr uint16_t max = period;
    static constexpr uint32_t cycles_per_pulse = (static_cast<uint32_t>(period) + 1u) * 3u;

    /// The machine at `divider`; the period loaded into the ISR through
    /// the FIFO and two instructions executed on the side (11.6.8).
    static bool init(PioClockDiv divider = {}) {
        const auto at_pin = Block::pin_index(pin);
        if (!at_pin) {
            return false;
        }
        const auto at = Block::add(pio_pwm_program);
        if (!at) {
            return false;
        }
        offset_ = *at;
        Sm::pin_directions(*at_pin, 1, true);
        Pin<pin>::function(Block::pin_function);
        PioSmConfig c{};
        c.clock = divider;
        c.sideset_base = *at_pin;
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

// 11.4's opcodes, and the examples' own words. The nine instructions of
// the RP2040 encode identically here, which is why the transmitter's
// assembled words below are the ones that chapter prints too.
static_assert(pio_jmp(PioJmp::always, 0) == 0x0000u && pio_jmp(PioJmp::x_dec, 2) == 0x0042u);
static_assert(pio_wait(false, PioWaitOn::pin, 0) == 0x2020u && pio_wait(true, PioWaitOn::gpio, 15) == 0x208Fu);
static_assert(pio_in(PioIn::pins, 1) == 0x4001u && pio_in(PioIn::null, 32) == 0x4060u);
static_assert(pio_out(PioOut::pins, 1) == 0x6001u && pio_out(PioOut::isr, 32) == 0x60C0u);
static_assert(pio_push() == 0x8020u && pio_pull() == 0x80A0u && pio_pull(false, false) == 0x8080u);
static_assert(pio_mov(PioMovTo::x, PioMovFrom::osr) == 0xA027u && pio_nop() == 0xA042u);
static_assert(pio_irq(4, false, false, PioIrqScope::relative) == 0xC014u && pio_irq(0, false, true) == 0xC020u &&
              pio_irq(1, true) == 0xC041u);
static_assert(pio_set(PioSetTo::x, 7) == 0xE027u && pio_set(PioSetTo::pindirs, 1) == 0xE081u && pio_set(PioSetTo::pins, 0) == 0xE000u);
// The delay and the side-set: `set pins, 1 [1]` is 0xE101; the UART
// transmitter's `pull side 1 [7]` under an optional one-pin side-set (an
// enable bit, a data bit, three delay bits) is 0x9FA0 and its `set x, 7
// side 0 [7]` 0xF727 - the vendor's assembled words.
static_assert(pio_delay(pio_set(PioSetTo::pins, 1), 1) == 0xE101u);
static_assert(pio_uart_tx_program.code[0] == 0x9FA0u && pio_uart_tx_program.code[1] == 0xF727u);
static_assert(pio_uart_tx_program.code[2] == 0x6001u && pio_uart_tx_program.code[3] == 0x0642u);
static_assert(pio_uart_rx_program.code[1] == 0xEA27u && pio_uart_rx_program.code[4] == 0x00C8u && pio_uart_rx_program.code[5] == 0xC014u);
static_assert(pio_relocate(pio_jmp(PioJmp::x_dec, 2), 10) == 0x004Cu && pio_relocate(pio_set(PioSetTo::x, 7), 10) == 0xE027u);

// AND THE FORMS THIS CHIP ADDED (11.1.1, 11.4). The IRQ index's two most
// significant bits are the scope, which is why the RP2040's `relative`
// keeps its 0x10: it is scope 2 of four.
static_assert(pio_irq(3, false, false, PioIrqScope::next) == 0xC01Bu);
static_assert(pio_irq(3, true, false, PioIrqScope::prev) == 0xC04Bu);
static_assert(pio_wait_irq(true, 3, PioIrqScope::prev) == 0x20CBu);
static_assert(pio_wait_irq(true, 4, PioIrqScope::relative) == 0x20D4u);
static_assert(pio_wait_jmppin(true) == 0x20E0u && pio_wait_jmppin(false, 3) == 0x2063u);
static_assert(pio_mov(PioMovTo::pindirs, PioMovFrom::null) == 0xA063u);
static_assert(pio_mov(PioMovTo::pindirs, PioMovFrom::null, PioMovOp::invert) == 0xA06Bu);
// PUT and GET share PUSH and PULL's opcode and are told apart by bit 4,
// which a PUSH or a PULL always leaves clear; bit 3 says the entry is the
// instruction's own and not scratch Y's.
static_assert(pio_put(0) == 0x8018u && pio_put(2) == 0x801Au && pio_put_y() == 0x8010u);
static_assert(pio_get(0) == 0x8098u && pio_get(1) == 0x8099u && pio_get_y() == 0x8090u);
static_assert(pio_status_irq(5) == 0x05u && pio_status_irq(5, PioStatusIrq::prev) == 0x0Du &&
              pio_status_irq(5, PioStatusIrq::next) == 0x15u);

// The register words a configuration makes, for the fields this chip
// added: IN_COUNT sits at the bottom of SHIFTCTRL (32 encoded as 0), and
// the two new join bits are its 15 and 14.
static_assert((pio_shiftctrl_of(PioSmConfig{}) & PIO_SM0_SHIFTCTRL_IN_COUNT_BITS) == 0u);
static_assert((pio_shiftctrl_of(PioSmConfig{.in_count = 5}) & PIO_SM0_SHIFTCTRL_IN_COUNT_BITS) == 5u);
static_assert((pio_shiftctrl_of(PioSmConfig{.fifo_join = PioFifoJoin::rx_put}) & PIO_SM0_SHIFTCTRL_FJOIN_RX_PUT_BITS) != 0u);
static_assert((pio_shiftctrl_of(PioSmConfig{.fifo_join = PioFifoJoin::rx_get}) & PIO_SM0_SHIFTCTRL_FJOIN_RX_GET_BITS) != 0u);
static_assert((pio_shiftctrl_of(PioSmConfig{.fifo_join = PioFifoJoin::rx_putget}) &
               (PIO_SM0_SHIFTCTRL_FJOIN_RX_PUT_BITS | PIO_SM0_SHIFTCTRL_FJOIN_RX_GET_BITS)) ==
              (PIO_SM0_SHIFTCTRL_FJOIN_RX_PUT_BITS | PIO_SM0_SHIFTCTRL_FJOIN_RX_GET_BITS));
static_assert((pio_execctrl_of(PioSmConfig{.status_select = PioStatus::irq_flag, .status_n = pio_status_irq(2)}) &
               PIO_SM0_EXECCTRL_STATUS_SEL_BITS) == (2u << PIO_SM0_EXECCTRL_STATUS_SEL_LSB));
static_assert(!pio_sm_config_valid(PioSmConfig{.in_count = 0}));
static_assert(!pio_sm_config_valid(PioSmConfig{.status_n = 9}));
static_assert(pio_sm_config_valid(PioSmConfig{.status_select = PioStatus::irq_flag, .status_n = pio_status_irq(7, PioStatusIrq::next)}));

static_assert(pio_clock_div_for(150'000'000, 150'000'000)->integer == 1u && pio_clock_div_for(150'000'000, 921'600)->integer == 162u);
static_assert(pio_clock_div_for(150'000'000, 1'000'000'000) == std::nullopt);
static_assert(pio_sm_hz(150'000'000, {1, 0}) == 150'000'000u && pio_sm_hz(150'000'000, {2, 128}) == 60'000'000u);
static_assert(pio_uart_tx_program.valid() && pio_uart_rx_program.valid() && pio_square_wave_program.valid() && pio_pwm_program.valid());
static_assert(PwmChannel<PioPwm<2, 0, 17, 999>>);

// The three blocks are a ring, and each one's requests, reset line, pin
// function and pair of interrupt lines are its own.
static_assert(Pio<0>::next_index == 1u && Pio<1>::next_index == 2u && Pio<2>::next_index == 0u);
static_assert(Pio<0>::prev_index == 2u && Pio<1>::prev_index == 0u && Pio<2>::prev_index == 1u);
static_assert(Pio<0>::pin_function == PinFunction::pio0 && Pio<2>::pin_function == PinFunction::pio2);
static_assert(PioSm<0, 2>::dreq_rx == Dreq::pio0_rx2 && PioSm<1, 3>::dreq_tx == Dreq::pio1_tx3);
static_assert(PioSm<2, 0>::dreq_tx == Dreq::pio2_tx0 && PioSm<2, 3>::dreq_rx == Dreq::pio2_rx3);
static_assert(Pio<2>::irq(0) == PIO2_IRQ_0_IRQn && Pio<2>::irq(1) == PIO2_IRQ_1_IRQn);
static_assert(PioInterrupt::flag(7) == 0x8000u && PioInterrupt::flag(0) == 0x0100u && PioInterrupt::all == 0xFFFFu);

} // namespace brio
