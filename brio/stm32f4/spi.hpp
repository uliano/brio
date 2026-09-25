/*
 * spi.hpp
 *
 * The SPI and the I2S of the STM32F4 (RM0090 ch. 28, RM0390 ch. 26,
 * RM0383 ch. 20 - one chapter, three manuals, the same registers): up to
 * six instances of one block that wears TWO FACES, chosen by one bit of
 * I2SCFGR, in the layers every brio bus driver has
 * (docs/design/spi-bus.md):
 *
 *  Spi<n>                the RESOURCE in its SPI face: the register
 *                        block, its bus, its gate and its reset, its one
 *                        vector, and the whole register description as
 *                        verbs - both roles, the four modes, 8- or
 *                        16-bit frames, the eight baud rates off the
 *                        instance's OWN APB clock, the three NSS
 *                        arrangements, the simplex and bidirectional
 *                        line modes, the hardware CRC with CRCNEXT, the
 *                        TI frame format, every flag with its clearing
 *                        sequence, the interrupt enables, the two DMA
 *                        requests, and one ISR body. It decides nothing.
 *  I2s<n> / I2sExt<n>    the RESOURCE in its I2S face: I2SMOD, the four
 *                        standards with PCM's two frame lengths, the
 *                        data and channel lengths, the master clock
 *                        output, and the I2SDIV/ODD arithmetic against
 *                        the audio PLL (stm32f4/clock.hpp's `Rcc`).
 *                        `I2sExt<n>` is the full-duplex extension block
 *                        the classes that have one give SPI2 and SPI3.
 *  SpiHost<n, pins, ...> the ENGINE util/spi_bus.hpp's SpiBus drives:
 *                        the Request is the other strata's VERBATIM, so
 *                        an application written over SpiBus on a Nucleo
 *                        or a Pico runs here unchanged.
 *  SpiClient<n, pins>    the other end of the wire, a thin polled
 *                        surface with an ISR body: a client is a
 *                        protocol and which one is the application's.
 *
 * THE F1 LINEAGE, LIKE THE USART OF THIS FAMILY. There is no FIFO and no
 * frame size beyond eight or sixteen bits: one transmit buffer, one
 * receive buffer, one shift register, so TXE means "the buffer moved
 * into the shifter" and RXNE "a frame has been shifted BOTH ways". The
 * host's pump therefore runs on RXNE alone, as every other stratum's
 * does, because a frame that came back is the one witness that the bus
 * is idle for the next. DFF and CRCEN "should be written only when SPE
 * is 0" (28.5.1), which is what makes a request that changes the frame
 * size pay a disable/enable pair.
 *
 * THE RATE IS THE INSTANCE'S OWN APB CLOCK OVER A POWER OF TWO, /2 to
 * /256 - and on this family the two APBs differ from HCLK at every rate
 * above their ceilings, so a host asks `apb_hz(clock, on_apb2)`
 * (stm32f4/clock.hpp) and never SYSCLK. SPI1, SPI4, SPI5 and SPI6 are
 * APB2's; SPI2 and SPI3 are APB1's.
 *
 * FOUR ERRATA OF THIS CHAPTER ARE LIVE ON EVERY PART WITH A MANUAL ON
 * THE DESK (ES0206 2.12, ES0287 2.11, ES0298 2.14), and three of them
 * are answered as code:
 *  - BSY MAY STAY HIGH WHEN THE SPI IS DISABLED. `disable()` waits for
 *    TXE and then for BSY in the transmitting configurations, as the
 *    workaround says, and in MASTER RECEIVE-ONLY (RXONLY, or
 *    bidirectional with the output off) it does not look at BSY at all -
 *    the errata's own instruction, and the manual's 28.3.7 already warns
 *    the flag is kept low there.
 *  - ANTICIPATED COMMUNICATION UPON A TRANSIT FROM SLAVE RECEIVER TO
 *    MASTER: the clock starts on the MSTR write even with SPE clear.
 *    `configure()` therefore pulses the RCC reset line before it writes
 *    a HOST configuration over a client receive-only one - the first of
 *    the two workarounds, and the one that needs no ordering trick.
 *  - A WRONG CRC WHEN THE POLYNOMIAL IS EVEN: `spi_config_valid()`
 *    refuses an even polynomial, so the CRC unit is never armed with a
 *    divisor that computes nonsense. (The reset value 0x0007 is odd.)
 *  - THE LAST RECEIVED BIT MAY BE WRONG WHEN THE SCK FEEDBACK IS SLOW.
 *    The internal loop back from the SCK PAD must return inside two APB
 *    periods, and the errata's table gives the ceiling per pad speed
 *    at a 30 pF load: 84 MHz of APB at `high` or `very_high`, 75 at
 *    `medium`, 25 at `low`. It is a PAD fact and not a rate fact - the
 *    SPI's own bit rate does not enter it - so every pad this driver
 *    hands over goes out at `very_high` and `sck_speed()` is the verb
 *    that moves it, `spi_errata_apb_ceiling_hz()` the table. A part run
 *    at 90 MHz of PCLK2 is ABOVE the ceiling at every pad speed, and
 *    what that costs is a bench measurement and not a promise.
 *
 * TWO OPTIONAL DMA ENGINE SLOTS, the shape every brio bus task has: name
 * a stm32f4/dma.hpp DmaTxEngine and/or DmaRxEngine and the data phase
 * moves without the CPU; name neither (the default) and every engine
 * branch disappears. A stream and a channel are not free here - the
 * request mapping gives an instance one, two or three (controller,
 * stream, channel) cells and no others - so a slot is checked against
 * the reserve's SPI slice of those tables at compile time, and refused
 * on a part class whose manual was not read.
 *
 * THE PADS ARE THE APPLICATION'S, as `UartPins` has them: SCK, MISO,
 * MOSI and NSS as `PinSel`s, each with the alternate function the
 * DATASHEET gives that signal on that pad - AF5 for SPI1, SPI2, SPI4,
 * SPI5 and SPI6, AF6 for SPI3 and for the I2S signals that share a pad
 * with it, with a handful of AF5/AF6 exceptions per pad that only the
 * datasheet's table 12 can settle. The device header carries no pin
 * table at all, so nothing here can check an AF: the bench is the check.
 *
 * NOT COVERED YET is at the end of docs/stm32f4/spi.md.
 */

#pragma once

#include <stdint.h>

#include <optional>
#include <type_traits>

#include "stm32f4xx.h"

#include "stm32f4/clock.hpp"
#include "stm32f4/delay.hpp"
#include "stm32f4/device_tables.hpp"
#include "stm32f4/dma_engine.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "kernel/borrowed.hpp"
#include "kernel/post.hpp"
#include "util/clock.hpp"
#include "util/spi_bus.hpp"

namespace brio {

// =============================================================================
// The vocabulary
// =============================================================================

/// The SR bits, spelled once for the handlers and the suites (28.5.3).
struct SpiFlag {
    SpiFlag() = delete;
    static constexpr uint32_t rxne = SPI_SR_RXNE;
    static constexpr uint32_t txe = SPI_SR_TXE;
    static constexpr uint32_t channel_side = SPI_SR_CHSIDE;   ///< I2S only
    static constexpr uint32_t underrun = SPI_SR_UDR;          ///< I2S only
    static constexpr uint32_t crc_error = SPI_SR_CRCERR;
    static constexpr uint32_t mode_fault = SPI_SR_MODF;
    static constexpr uint32_t overrun = SPI_SR_OVR;
    static constexpr uint32_t busy = SPI_SR_BSY;
    static constexpr uint32_t frame_error = SPI_SR_FRE;       ///< TI slave / I2S slave
    /// What ERRIE puts on the vector.
    static constexpr uint32_t errors = crc_error | mode_fault | overrun | underrun | frame_error;
};

/// CPOL and CPHA as the four Motorola modes (28.3.1, figure 248).
enum class SpiMode : uint8_t { mode0 = 0, mode1 = 1, mode2 = 2, mode3 = 3 };

constexpr bool spi_mode_cpol(SpiMode m) { return (static_cast<uint8_t>(m) & 2u) != 0u; }
constexpr bool spi_mode_cpha(SpiMode m) { return (static_cast<uint8_t>(m) & 1u) != 0u; }

/// BR[2:0] (28.5.1): the instance's APB clock over two to the code plus one.
enum class SpiClock : uint8_t {
    div2 = 0, div4 = 1, div8 = 2, div16 = 3, div32 = 4, div64 = 5, div128 = 6, div256 = 7,
};

constexpr uint32_t spi_sck_hz(uint32_t pclk, SpiClock c) {
    return pclk >> (static_cast<uint8_t>(c) + 1u);
}

/// The coarsest code whose SCK does not exceed `max_hz`; nullopt when even
/// PCLK/256 does - refused, never rounded up.
constexpr std::optional<SpiClock> spi_rate_for(uint32_t pclk, uint32_t max_hz) {
    if (pclk == 0u || max_hz == 0u) {
        return {};
    }
    for (uint8_t code = 0; code < 8u; ++code) {
        if ((pclk >> (code + 1u)) <= max_hz) {
            return static_cast<SpiClock>(code);
        }
    }
    return {};
}

/// DFF: the two frame sizes this block has.
enum class SpiDataSize : uint8_t { bits8 = 0, bits16 = 1 };

constexpr bool spi_frame_is_halfword(SpiDataSize s) { return s == SpiDataSize::bits16; }
constexpr uint8_t spi_data_bits(SpiDataSize s) { return s == SpiDataSize::bits16 ? 16u : 8u; }
constexpr uint16_t spi_frame_mask(SpiDataSize s) { return s == SpiDataSize::bits16 ? 0xFFFFu : 0x00FFu; }

enum class SpiRole : uint8_t { client = 0, host = 1 };

/// The chip-select arrangements (28.3.1): `software` is SSM/SSI, the NSS
/// pad free (what the engine's GPIO select needs); `hardware_output` is
/// SSOE, the pad driven low by the host while SPE is set and unusable in
/// a multi-host system; `hardware_input` is the pad as an input, whose low
/// level on a host raises MODF and demotes it to a client.
enum class SpiNss : uint8_t { software, hardware_output, hardware_input };

/// 28.3.4's line arrangements.
enum class SpiDirection : uint8_t {
    full_duplex,       ///< two data lines, both ways
    receive_only,      ///< RXONLY: two lines, the output one released
    half_duplex_out,   ///< BIDIMODE + BIDIOE: one line, transmitting
    half_duplex_in,    ///< BIDIMODE, BIDIOE clear: one line, receiving
};

/// CR2.FRF: Motorola (CPOL/CPHA and a level select) or TI (an NSS pulse
/// framing every word, and no CPOL, CPHA, LSBFIRST or software select).
enum class SpiFrameFormat : uint8_t { motorola = 0, ti = 1 };

/**
 * ES0206 2.12.4 / ES0287 2.11.4 / ES0298's twin, table 5: the highest APB
 * frequency at which a MASTER's last received bit is still captured, per
 * the SCK pad's OSPEEDR class at a 30 pF load. Above it the last bit of
 * each received frame may keep the previous pattern's value, and a CRC
 * over such data may report a false error. `for_i2s` gives the I2S
 * column, which is half as forgiving.
 */
constexpr uint32_t spi_errata_apb_ceiling_hz(PinSpeed sck_speed, bool for_i2s = false) {
    switch (sck_speed) {
        case PinSpeed::very_high:
        case PinSpeed::high:   return for_i2s ? 42'000'000u : 84'000'000u;
        case PinSpeed::medium: return for_i2s ? 35'000'000u : 75'000'000u;
        case PinSpeed::low:    return for_i2s ? 16'000'000u : 25'000'000u;
    }
    return 0u;
}

struct SpiConfig {
    SpiRole role = SpiRole::host;
    SpiMode mode = SpiMode::mode0;
    SpiClock clock = SpiClock::div16;
    SpiDataSize bits = SpiDataSize::bits8;
    bool lsb_first = false;
    SpiNss nss = SpiNss::software;
    SpiDirection direction = SpiDirection::full_duplex;
    SpiFrameFormat frame_format = SpiFrameFormat::motorola;
    bool crc = false;
    uint16_t crc_polynomial = 0x0007u;   ///< the reset value; MUST be odd
    bool dma_transmit = false;
    bool dma_receive = false;
};

/**
 * The refusals the chapter and the errata owe:
 *  - AN EVEN CRC POLYNOMIAL computes the wrong checksum on every part
 *    with an errata sheet on the desk, and zero is even too;
 *  - SSOE IS A HOST'S: a client has no NSS output to enable;
 *  - TI MODE USES NEITHER the software select nor LSBFIRST (28.5.1's
 *    notes on SSM, SSI and LSBFIRST), so a configuration that names one
 *    beside it is a statement the silicon will not honour.
 */
constexpr bool spi_config_valid(const SpiConfig& c) {
    if (c.crc && (c.crc_polynomial % 2u) == 0u) {
        return false;
    }
    if (c.role == SpiRole::client && c.nss == SpiNss::hardware_output) {
        return false;
    }
    if (c.frame_format == SpiFrameFormat::ti && (c.lsb_first || c.nss == SpiNss::software)) {
        return false;
    }
    return true;
}

/// What CR1 holds for a configuration, SPE and CRCNEXT clear.
constexpr uint32_t spi_cr1_of(const SpiConfig& c) {
    uint32_t v = 0;
    if (spi_mode_cpha(c.mode)) { v |= SPI_CR1_CPHA; }
    if (spi_mode_cpol(c.mode)) { v |= SPI_CR1_CPOL; }
    if (c.role == SpiRole::host) { v |= SPI_CR1_MSTR; }
    v |= static_cast<uint32_t>(c.clock) << SPI_CR1_BR_Pos;
    if (c.lsb_first) { v |= SPI_CR1_LSBFIRST; }
    if (c.nss == SpiNss::software) {
        v |= SPI_CR1_SSM;
        // A host under SSM must hold SSI HIGH or it faults itself
        // (28.3.10: SSI low on a master is a mode fault); a client under
        // SSM keeps it low, which is "selected".
        if (c.role == SpiRole::host) { v |= SPI_CR1_SSI; }
    }
    if (c.bits == SpiDataSize::bits16) { v |= SPI_CR1_DFF; }
    if (c.crc) { v |= SPI_CR1_CRCEN; }
    switch (c.direction) {
        case SpiDirection::full_duplex:     break;
        case SpiDirection::receive_only:    v |= SPI_CR1_RXONLY; break;
        case SpiDirection::half_duplex_out: v |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE; break;
        case SpiDirection::half_duplex_in:  v |= SPI_CR1_BIDIMODE; break;
    }
    return v;
}

/// What CR2 holds for a configuration, the interrupt enables kept apart.
constexpr uint32_t spi_cr2_of(const SpiConfig& c) {
    uint32_t v = 0;
    if (c.nss == SpiNss::hardware_output) { v |= SPI_CR2_SSOE; }
    if (c.frame_format == SpiFrameFormat::ti) { v |= SPI_CR2_FRF; }
    if (c.dma_transmit) { v |= SPI_CR2_TXDMAEN; }
    if (c.dma_receive) { v |= SPI_CR2_RXDMAEN; }
    return v;
}

/// Whether a configuration is a MASTER RECEIVE-ONLY one - the shape whose
/// BSY flag the manual (28.3.7) keeps low and the errata calls unreliable.
constexpr bool spi_master_receive_only(const SpiConfig& c) {
    return c.role == SpiRole::host &&
           (c.direction == SpiDirection::receive_only || c.direction == SpiDirection::half_duplex_in);
}

// =============================================================================
// Spi<n>: the resource, SPI face
// =============================================================================

/**
 * One SPI instance, register by register. Every verb stores what it names
 * and nothing else; a verb whose combination the chapter or an errata
 * declares wrong refuses (false, nothing written). The tasks below are
 * written on these verbs; a program that wants the chapter beyond a bus
 * transaction - a CRC-checked link, a TI-framed one, a receive-only
 * stream - reaches them here.
 */
template <uint8_t n>
struct Spi {
    static_assert(spi_present(n),
                  "brio Spi: this device has no SPI instance of that number (the device "
                  "header declares no SPIn_BASE for it)");

    Spi() = delete;

    static constexpr uint8_t number = n;
    static constexpr bool on_apb2 = spi_on_apb2(n);
    static constexpr IRQn_Type irq = spi_irq(n);
    /// Whether this instance wears the I2S face at all on this part class
    /// (stm32f4/device_tables.hpp: the manuals' memory maps, not the
    /// header, and a class whose manual was not read answers false).
    static constexpr bool has_i2s = spi_i2s_capable(n);

    static SPI_TypeDef& regs() { return *reinterpret_cast<SPI_TypeDef*>(spi_base(n)); }

    // ---- the gate and the reset ------------------------------------------------
    static void bus_clock(bool on) {
        if constexpr (on_apb2) {
            Rcc::apb2_clock(spi_clock_mask(n), on);
        } else {
            Rcc::apb1_clock(spi_clock_mask(n), on);
        }
    }
    static bool bus_clock() {
        if constexpr (on_apb2) {
            return Rcc::apb2_clock(spi_clock_mask(n));
        } else {
            return Rcc::apb1_clock(spi_clock_mask(n));
        }
    }
    /// Every register to its reset value through the RCC's reset line.
    static void reset() {
        if constexpr (on_apb2) {
            Rcc::apb2_reset(spi_clock_mask(n));
        } else {
            Rcc::apb1_reset(spi_clock_mask(n));
        }
    }

    // ---- the configuration -------------------------------------------------------
    /**
     * The whole configuration, with SPE clear on the way in (DFF and CRCEN
     * are enable-protected, 28.5.1). Refused by spi_config_valid()'s rules;
     * the interrupt enables in CR2 are kept as they are.
     *
     * ERRATA 2.12.2: writing MSTR while the block stands in a CLIENT
     * receive-only or bidirectional-receive configuration starts the
     * communication clock even with SPE clear. So a move from such a state
     * to a host one pulses the RCC reset first - which is the errata's own
     * first workaround and costs nothing on the ordinary path, where the
     * block is not a client at all.
     */
    static bool configure(const SpiConfig& c) {
        if (!spi_config_valid(c)) {
            return false;
        }
        SPI_TypeDef& r = regs();
        const uint32_t cr1 = r.CR1;
        const bool was_client_receiving =
            (cr1 & SPI_CR1_MSTR) == 0u &&
            ((cr1 & SPI_CR1_RXONLY) != 0u ||
             ((cr1 & SPI_CR1_BIDIMODE) != 0u && (cr1 & SPI_CR1_BIDIOE) == 0u));
        if (c.role == SpiRole::host && was_client_receiving) {
            const uint32_t keep = r.CR2 & (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
            reset();
            r.CR2 = keep;
        }
        r.CR1 = cr1 & ~SPI_CR1_SPE;
        if (c.crc) {
            r.CRCPR = c.crc_polynomial;
        }
        r.CR1 = spi_cr1_of(c);
        r.CR2 = (r.CR2 & (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE)) | spi_cr2_of(c);
        return true;
    }

    static void enable() { regs().CR1 |= SPI_CR1_SPE; }
    static bool enabled() { return (regs().CR1 & SPI_CR1_SPE) != 0u; }

    /**
     * 28.3.8's disable procedure, with ES0206 2.12.1 folded in: in a
     * transmitting configuration wait for TXE and then for BSY to fall
     * before dropping SPE; in MASTER RECEIVE-ONLY do not look at BSY at
     * all, because the flag is kept low there by design and unreliable by
     * errata. The manual's first step - wait for RXNE, to take the last
     * frame - is the CALLER'S: the engine above has already read it, and
     * waiting here would hang on a frame nobody is going to send. Each
     * wait is bounded; false says one ran out and SPE was dropped anyway.
     */
    static bool disable() {
        bool ok = true;
        if (enabled() && !master_receive_only()) {
            ok = wait_until([] { return tx_empty(); }) && wait_until([] { return !busy(); });
        }
        regs().CR1 &= ~SPI_CR1_SPE;
        return ok;
    }

    static bool master_receive_only() {
        const uint32_t cr1 = regs().CR1;
        if ((cr1 & SPI_CR1_MSTR) == 0u) {
            return false;
        }
        return (cr1 & SPI_CR1_RXONLY) != 0u ||
               ((cr1 & SPI_CR1_BIDIMODE) != 0u && (cr1 & SPI_CR1_BIDIOE) == 0u);
    }

    /// BIDIOE alone: the line's direction inside bidirectional mode, the
    /// one configuration bit the chapter lets a program flip while the
    /// block runs - what a three-wire device (a command out, an answer
    /// back on the same pad) needs between its two phases. Clearing it on
    /// a MASTER also starts a continuous receive, so the caller's next
    /// business is 28.3.8's stop.
    ///
    /// THE PAD IS THE CALLER'S TO TURN ROUND, and it must: this bit stops
    /// the SHIFTER from driving and not the pad's output stage, so a pad
    /// left as a push-pull alternate function keeps the line driven and
    /// the device's answer never arrives (measured - docs/stm32f4/spi.md).
    /// The receive window wants the pad in its alternate function still,
    /// but OPEN DRAIN with a pull-up; a plain input takes the peripheral's
    /// input away with the output.
    static void bidirectional_output(bool transmitting) {
        if (transmitting) {
            regs().CR1 |= SPI_CR1_BIDIOE;
        } else {
            regs().CR1 &= ~SPI_CR1_BIDIOE;
        }
    }
    static bool bidirectional() { return (regs().CR1 & SPI_CR1_BIDIMODE) != 0u; }

    // ---- data ---------------------------------------------------------------------
    /// 28.5 opens with the access rule: "the peripheral registers must be
    /// accessed by half-words (16 bits) or words (32 bits)". A frame goes
    /// out as ONE WORD store whatever DFF says - the buffer behind DR is
    /// one frame wide, and in 8-bit mode only DR[7:0] is used. (A byte
    /// store is what the DMA engines do, and the request mapping is what
    /// makes that legal for them: it is a stream's access width, not a
    /// core access to the register.)
    static void data(uint16_t v) { regs().DR = v; }
    /// Read one frame, which is also what clears RXNE.
    static uint16_t data(SpiDataSize bits) {
        return static_cast<uint16_t>(regs().DR) & spi_frame_mask(bits);
    }
    static uint16_t data() { return static_cast<uint16_t>(regs().DR); }
    static void data8(uint8_t v) { data(v); }
    static uint8_t data8() { return static_cast<uint8_t>(regs().DR); }
    static volatile uint32_t* data_address() { return &regs().DR; }

    static bool rx_ready() { return (regs().SR & SPI_SR_RXNE) != 0u; }
    static bool tx_empty() { return (regs().SR & SPI_SR_TXE) != 0u; }
    static bool busy() { return (regs().SR & SPI_SR_BSY) != 0u; }
    static uint32_t status() { return regs().SR; }
    static bool flag(uint32_t mask) { return (regs().SR & mask) != 0u; }

    /// Throw away whatever the receive buffer holds, and the OVR it may
    /// have raised: the DR read then the SR read IS the overrun's own
    /// clearing sequence (28.3.10).
    static void flush_rx() {
        (void)regs().DR;
        (void)regs().SR;
    }

    // ---- the errors and their sequences (28.3.10) ---------------------------------
    static bool overrun() { return (regs().SR & SPI_SR_OVR) != 0u; }
    /// A DR read followed by an SR read, in that order.
    static void clear_overrun() {
        (void)regs().DR;
        (void)regs().SR;
    }
    static bool mode_fault() { return (regs().SR & SPI_SR_MODF) != 0u; }
    /// An SR access while MODF stands, then a CR1 write. MODF has also
    /// cleared SPE and MSTR, and the silicon refuses to set either again
    /// until the sequence has run - so the caller reconfigures after it.
    static void clear_mode_fault() {
        (void)regs().SR;
        regs().CR1 = regs().CR1;
    }
    static bool crc_error() { return (regs().SR & SPI_SR_CRCERR) != 0u; }
    /// The one rc_w0 bit of this register: a zero written into it.
    static void clear_crc_error() { regs().SR = ~SPI_SR_CRCERR; }
    /// TI slave framing, and the I2S slave's desynchronization: one flag,
    /// cleared by reading SR.
    static bool frame_error() { return (regs().SR & SPI_SR_FRE) != 0u; }
    static void clear_frame_error() { (void)regs().SR; }

    // ---- the CRC unit (28.3.6) -----------------------------------------------------
    /// The polynomial. Refused when even (the errata) and while the unit
    /// is enabled - CRCEN's own note wants SPE clear for a CRC change, and
    /// this register is the divisor the running calculation uses.
    static bool crc_polynomial(uint16_t poly) {
        if ((poly % 2u) == 0u || enabled()) {
            return false;
        }
        regs().CRCPR = poly;
        return true;
    }
    static uint16_t crc_polynomial() { return static_cast<uint16_t>(regs().CRCPR); }
    /// CRCEN. Writing a one RESETS both CRC registers (28.3.6), which is
    /// also the chapter's way of clearing a stale calculation between two
    /// client selections. Refused while SPE stands.
    static bool crc_enable(bool on) {
        if (enabled()) {
            return false;
        }
        if (on) {
            regs().CR1 |= SPI_CR1_CRCEN;
        } else {
            regs().CR1 &= ~SPI_CR1_CRCEN;
        }
        return true;
    }
    static bool crc_enabled() { return (regs().CR1 & SPI_CR1_CRCEN) != 0u; }
    /// CRCNEXT: the frame after the one just written is the CRC. In full
    /// duplex or transmit-only it is written IMMEDIATELY after the last
    /// data goes into DR; in receive-only, after the second-to-last frame
    /// has arrived. Under DMA it is not used at all - the block sends and
    /// checks the pattern by itself (28.3.9).
    static void crc_next() { regs().CR1 |= SPI_CR1_CRCNEXT; }
    static bool crc_next_pending() { return (regs().CR1 & SPI_CR1_CRCNEXT) != 0u; }
    /// A read while BSY stands "could return an incorrect value" (28.5.6).
    static uint16_t rx_crc() { return static_cast<uint16_t>(regs().RXCRCR); }
    static uint16_t tx_crc() { return static_cast<uint16_t>(regs().TXCRCR); }

    // ---- the chip select ------------------------------------------------------------
    /// SSI, for a configuration under SSM: high is "not selected" on a
    /// host (and what keeps it a host - a low SSI is a mode fault), low
    /// selects a client.
    static void software_select(bool selected) {
        if (selected) {
            regs().CR1 &= ~SPI_CR1_SSI;
        } else {
            regs().CR1 |= SPI_CR1_SSI;
        }
    }
    static bool software_selected() { return (regs().CR1 & SPI_CR1_SSI) == 0u; }
    /// SSOE: the NSS pad driven by a host while SPE is set. "The cell
    /// cannot work in a multimaster environment" once it is on.
    static void nss_output(bool on) {
        if (on) {
            regs().CR2 |= SPI_CR2_SSOE;
        } else {
            regs().CR2 &= ~SPI_CR2_SSOE;
        }
    }
    static bool nss_output() { return (regs().CR2 & SPI_CR2_SSOE) != 0u; }

    // ---- DMA and interrupts -----------------------------------------------------------
    static void dma_requests(bool tx, bool rx) {
        uint32_t v = regs().CR2 & ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
        if (tx) { v |= SPI_CR2_TXDMAEN; }
        if (rx) { v |= SPI_CR2_RXDMAEN; }
        regs().CR2 = v;
    }
    static void dma_transmit(bool on) { bit(regs().CR2, SPI_CR2_TXDMAEN, on); }
    static void dma_receive(bool on) { bit(regs().CR2, SPI_CR2_RXDMAEN, on); }

    static void rxne_interrupt(bool on) { bit(regs().CR2, SPI_CR2_RXNEIE, on); }
    static bool rxne_interrupt() { return (regs().CR2 & SPI_CR2_RXNEIE) != 0u; }
    static void txe_interrupt(bool on) { bit(regs().CR2, SPI_CR2_TXEIE, on); }
    static void error_interrupt(bool on) { bit(regs().CR2, SPI_CR2_ERRIE, on); }

    /**
     * The instance's ONE interrupt body - call from its vector. Returns
     * the raised sources that are ENABLED (table 126): RXNE and TXE under
     * their own enables, and the five error flags under ERRIE. It
     * CONSUMES NOTHING - every one of those flags is cleared by a
     * sequence only the owner can decide to run (a DR read takes a frame,
     * a CR1 write re-states a configuration).
     */
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t sr = regs().SR;
        const uint32_t cr2 = regs().CR2;
        uint32_t up = 0;
        if ((cr2 & SPI_CR2_RXNEIE) != 0u && (sr & SPI_SR_RXNE) != 0u) { up |= SpiFlag::rxne; }
        if ((cr2 & SPI_CR2_TXEIE) != 0u && (sr & SPI_SR_TXE) != 0u) { up |= SpiFlag::txe; }
        if ((cr2 & SPI_CR2_ERRIE) != 0u) { up |= sr & SpiFlag::errors; }
        return up;
    }

private:
    static void bit(volatile uint32_t& r, uint32_t mask, bool on) {
        if (on) {
            r |= mask;
        } else {
            r &= ~mask;
        }
    }

    /// A bounded wait: true when the condition came, false when the budget
    /// ran out (a bus whose clock never runs must not hang the kernel; the
    /// budget is generous against the slowest frame at the slowest rate).
    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = 400'000u; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }
};

// =============================================================================
// I2s<n>: the resource, I2S face
// =============================================================================

/// I2SCFG[1:0] (28.5.8): what the block is on the audio bus.
enum class I2sMode : uint8_t {
    client_transmit = 0,
    client_receive = 1,
    host_transmit = 2,
    host_receive = 3,
};

constexpr bool i2s_mode_is_host(I2sMode m) {
    return m == I2sMode::host_transmit || m == I2sMode::host_receive;
}
constexpr bool i2s_mode_is_transmit(I2sMode m) {
    return m == I2sMode::host_transmit || m == I2sMode::client_transmit;
}

/// I2SSTD[1:0]: the four audio protocols (28.4.3).
enum class I2sStandard : uint8_t {
    philips = 0,        ///< the I2S Philips standard, WS one clock ahead
    msb_justified = 1,  ///< left justified
    lsb_justified = 2,  ///< right justified
    pcm = 3,            ///< PCM, whose frame length PCMSYNC picks
};

/// DATLEN[1:0]: the data a frame carries.
enum class I2sDataLength : uint8_t { bits16 = 0, bits24 = 1, bits32 = 2 };

/// CHLEN: the channel's width. 28.5.8: the bit "has a meaning only if
/// DATLEN = 00, otherwise the channel length is fixed to 32-bit by
/// hardware whatever the value filled in".
enum class I2sChannelLength : uint8_t { bits16 = 0, bits32 = 1 };

struct I2sConfig {
    I2sMode mode = I2sMode::host_transmit;
    I2sStandard standard = I2sStandard::philips;
    bool pcm_long_frame = false;     ///< PCMSYNC, and only under `pcm`
    I2sDataLength data = I2sDataLength::bits16;
    I2sChannelLength channel = I2sChannelLength::bits16;
    bool clock_idle_high = false;    ///< CKPOL, the steady state of CK
    bool master_clock_out = false;   ///< MCKOE: MCK on its own pad, 256 x FS
    uint8_t div = 2;                 ///< I2SDIV, 2..255 (0 and 1 forbidden)
    bool odd = false;                ///< ODD: the real divider is 2 x div + 1
    bool dma_transmit = false;
    bool dma_receive = false;
};

/// The channel width the silicon really uses: 32 bits whenever the data
/// is wider than 16, whatever CHLEN was written with (28.5.8).
constexpr bool i2s_channel_is_32(const I2sConfig& c) {
    return c.data != I2sDataLength::bits16 || c.channel == I2sChannelLength::bits32;
}

/// The whole divisor the clock generator applies, 28.4.4: the bit clock is
/// I2SxCLK over `2 x I2SDIV + ODD`, and one audio frame is two channels of
/// 16 or 32 bits - or, with MCK out, a fixed 256 x FS.
constexpr uint32_t i2s_frame_factor(bool channel_32, bool master_clock_out) {
    if (master_clock_out) {
        return 256u;   // 16*2*8 and 32*2*4 are both 256 - the MCK = 256 x FS rule
    }
    return channel_32 ? 64u : 32u;
}

/// The sampling frequency a divider pair produces. 0 for a forbidden
/// divider (I2SDIV of 0 or 1).
constexpr uint32_t i2s_fs_hz(uint32_t i2s_clk_hz, uint8_t div, bool odd, bool channel_32,
                             bool master_clock_out) {
    if (div < 2u) {
        return 0u;
    }
    const uint32_t whole = 2u * static_cast<uint32_t>(div) + (odd ? 1u : 0u);
    const uint32_t den = i2s_frame_factor(channel_32, master_clock_out) * whole;
    return den == 0u ? 0u : i2s_clk_hz / den;
}

struct I2sPrescaler {
    uint8_t div = 0;
    bool odd = false;
};

/// The divider pair whose FS is nearest `fs_hz`, or nullopt when the whole
/// divisor would fall outside 4..511 (I2SDIV 2..255 with ODD). The
/// rounding is to nearest, and `i2s_fs_hz` says what really comes out -
/// an audio rate is hardly ever exact and the chapter's own table 127
/// prints the error.
constexpr std::optional<I2sPrescaler> i2s_prescaler_for(uint32_t i2s_clk_hz, uint32_t fs_hz,
                                                        bool channel_32, bool master_clock_out) {
    if (i2s_clk_hz == 0u || fs_hz == 0u) {
        return {};
    }
    const uint32_t unit = i2s_frame_factor(channel_32, master_clock_out) * fs_hz;
    if (unit == 0u) {
        return {};
    }
    const uint32_t whole = (i2s_clk_hz + unit / 2u) / unit;
    if (whole < 4u || whole > 511u) {
        return {};
    }
    return I2sPrescaler{static_cast<uint8_t>(whole / 2u), (whole & 1u) != 0u};
}

/// The refusals the chapter owes: a forbidden divider, and PCM's long
/// frame named outside PCM.
constexpr bool i2s_config_valid(const I2sConfig& c) {
    if (i2s_mode_is_host(c.mode) && c.div < 2u) {
        return false;
    }
    if (c.pcm_long_frame && c.standard != I2sStandard::pcm) {
        return false;
    }
    return true;
}

constexpr uint32_t i2s_i2scfgr_of(const I2sConfig& c) {
    uint32_t v = SPI_I2SCFGR_I2SMOD;
    v |= static_cast<uint32_t>(c.mode) << SPI_I2SCFGR_I2SCFG_Pos;
    v |= static_cast<uint32_t>(c.standard) << SPI_I2SCFGR_I2SSTD_Pos;
    if (c.pcm_long_frame) { v |= SPI_I2SCFGR_PCMSYNC; }
    v |= static_cast<uint32_t>(c.data) << SPI_I2SCFGR_DATLEN_Pos;
    if (c.channel == I2sChannelLength::bits32) { v |= SPI_I2SCFGR_CHLEN; }
    if (c.clock_idle_high) { v |= SPI_I2SCFGR_CKPOL; }
    return v;
}

constexpr uint32_t i2s_i2spr_of(const I2sConfig& c) {
    uint32_t v = c.div;
    if (c.odd) { v |= SPI_I2SPR_ODD; }
    if (c.master_clock_out) { v |= SPI_I2SPR_MCKOE; }
    return v;
}

/**
 * I2s<n> - the same register block in its audio face, and `I2sExt<n>` the
 * FULL-DUPLEX EXTENSION block beside it (28.4.2): a second, always SLAVE,
 * cell that shares the instance's CK and WS and carries the other
 * direction on its own data pad. The part classes that have no extension
 * block do full duplex by pairing two whole instances instead (RM0390
 * 26.6.2), which is why the presence question is asked of the header.
 *
 * WHAT IS SHARED WITH THE SPI FACE AND WHAT IS NOT. The gate, the reset,
 * the vector, DR, SR and CR2's interrupt and DMA enables are one block's
 * and are reached here by the same names; CR1 is not used at all in I2S
 * mode, and I2SCFGR's I2SMOD is what decides which face answers. The CRC
 * unit does not exist here (28.4.10).
 *
 * THE CLOCK IS NOT THE APB'S. An I2S divides I2SxCLK - the audio PLL's R
 * output, an external clock on the I2S_CKIN pad, or on the parts with the
 * RCC_DCKCFGR pair the main PLL's R or the root itself - selected through
 * `Rcc::i2s_source()` and configured through `Rcc::plli2s_configure()`
 * (stm32f4/clock.hpp). The APB clock only has to be fast enough to serve
 * the data register.
 */
template <uint8_t n, bool extension = false>
struct I2s {
    static_assert(spi_present(n),
                  "brio I2s: this device has no SPI instance of that number");
    static_assert(spi_i2s_facts().known,
                  "brio I2s: which instances of this part wear the I2S face is a fact of "
                  "the reference manual and not of the device header, and no manual for "
                  "this part class has been read (stm32f4/device_tables.hpp's "
                  "spi_i2s_facts(): the F405 class, the F42x/F43x class, the F411 and the "
                  "F446 are the classes that are known)");
    static_assert(extension || spi_i2s_capable(n),
                  "brio I2s: this instance has no I2S face on this part class - the "
                  "manual's memory map does not name it SPIn/I2Sn");
    static_assert(!extension || i2s_ext_present(n),
                  "brio I2sExt: this part has no full-duplex extension block for that "
                  "instance (the header declares no I2Snext_BASE); on a part class without "
                  "them, full duplex is two whole instances paired");

    I2s() = delete;

    static constexpr uint8_t number = n;
    static constexpr bool is_extension = extension;
    static constexpr bool on_apb2 = spi_on_apb2(n);
    /// The extension block has no vector of its own: it shares the
    /// instance's (RM0090 table 61 lists SPIn and nothing else).
    static constexpr IRQn_Type irq = spi_irq(n);

    static SPI_TypeDef& regs() {
        return *reinterpret_cast<SPI_TypeDef*>(extension ? i2s_ext_base(n) : spi_base(n));
    }

    // The gate and the reset are the INSTANCE'S - the extension block has
    // neither of its own.
    static void bus_clock(bool on) { Spi<n>::bus_clock(on); }
    static bool bus_clock() { return Spi<n>::bus_clock(); }
    static void reset() { Spi<n>::reset(); }

    /**
     * Write I2SCFGR and I2SPR from a configuration, with I2SE clear on the
     * way in - "for correct operation, these bits should be configured
     * when the I2S is disabled" is written against every field of both
     * registers. Refused by i2s_config_valid()'s rules, and refused
     * outright when a MASTER mode is asked of the extension block, which
     * 28.4.2 says "operate always in slave mode".
     */
    static bool configure(const I2sConfig& c) {
        if (!i2s_config_valid(c)) {
            return false;
        }
        if constexpr (extension) {
            if (i2s_mode_is_host(c.mode)) {
                return false;
            }
        }
        SPI_TypeDef& r = regs();
        r.I2SCFGR &= ~SPI_I2SCFGR_I2SE;
        if constexpr (!extension) {
            // The extension block takes CK and WS from its instance and
            // has no generator: its I2SPR is not written.
            r.I2SPR = i2s_i2spr_of(c);
        }
        r.I2SCFGR = i2s_i2scfgr_of(c);
        r.CR2 = (r.CR2 & ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN)) |
                (c.dma_transmit ? SPI_CR2_TXDMAEN : 0u) | (c.dma_receive ? SPI_CR2_RXDMAEN : 0u);
        return true;
    }

    /// Back to the SPI face: I2SMOD cleared with the block disabled.
    static void select_spi_mode() {
        regs().I2SCFGR &= ~(SPI_I2SCFGR_I2SE | SPI_I2SCFGR_I2SMOD);
    }
    static bool i2s_mode_selected() { return (regs().I2SCFGR & SPI_I2SCFGR_I2SMOD) != 0u; }

    static void enable() { regs().I2SCFGR |= SPI_I2SCFGR_I2SE; }
    static bool enabled() { return (regs().I2SCFGR & SPI_I2SCFGR_I2SE) != 0u; }
    /// The transmitting configurations want the last frame out first
    /// (the SPI face's rule, and the same registers); a receiving one
    /// stops where it stands. Bounded; false says a wait ran out.
    static bool disable() {
        bool ok = true;
        if (enabled() && i2s_mode_is_transmit(mode())) {
            ok = wait_until([] { return tx_empty(); }) && wait_until([] { return !busy(); });
        }
        regs().I2SCFGR &= ~SPI_I2SCFGR_I2SE;
        return ok;
    }

    static I2sMode mode() {
        return static_cast<I2sMode>((regs().I2SCFGR & SPI_I2SCFGR_I2SCFG_Msk) >> SPI_I2SCFGR_I2SCFG_Pos);
    }

    // ---- the generator ------------------------------------------------------------
    /// I2SDIV and ODD alone, with the block disabled: what a program that
    /// changes the sampling rate and nothing else writes.
    static bool prescaler(uint8_t div, bool odd) {
        if (div < 2u || enabled()) {
            return false;
        }
        regs().I2SPR = (regs().I2SPR & SPI_I2SPR_MCKOE) | div | (odd ? SPI_I2SPR_ODD : 0u);
        return true;
    }
    static uint8_t prescaler_div() { return static_cast<uint8_t>(regs().I2SPR & SPI_I2SPR_I2SDIV); }
    static bool prescaler_odd() { return (regs().I2SPR & SPI_I2SPR_ODD) != 0u; }
    static bool master_clock_out() { return (regs().I2SPR & SPI_I2SPR_MCKOE) != 0u; }

    // ---- data and flags ------------------------------------------------------------
    static void data(uint16_t v) { regs().DR = v; }
    static uint16_t data() { return static_cast<uint16_t>(regs().DR); }
    static volatile uint32_t* data_address() { return &regs().DR; }

    static bool tx_empty() { return (regs().SR & SPI_SR_TXE) != 0u; }
    static bool rx_ready() { return (regs().SR & SPI_SR_RXNE) != 0u; }
    static bool busy() { return (regs().SR & SPI_SR_BSY) != 0u; }
    static uint32_t status() { return regs().SR; }
    /// CHSIDE, refreshed when TXE rises on a transmitter and when a frame
    /// lands on a receiver: false = left, true = right. Meaningless in PCM
    /// and after an underrun or an overrun (28.4.7).
    static bool right_channel() { return (regs().SR & SPI_SR_CHSIDE) != 0u; }

    /// UDR: a client transmitter clocked before software loaded DR.
    /// Cleared by a read of SR.
    static bool underrun() { return (regs().SR & SPI_SR_UDR) != 0u; }
    static void clear_underrun() { (void)regs().SR; }
    /// OVR: a DR read then an SR read, as on the SPI face.
    static bool overrun() { return (regs().SR & SPI_SR_OVR) != 0u; }
    static void clear_overrun() {
        (void)regs().DR;
        (void)regs().SR;
    }
    /// FRE: a client whose master moved WS where it was not expected.
    /// Cleared by a read of SR; recovering means disabling the block and
    /// re-enabling it on the right WS level (28.4.8).
    static bool frame_error() { return (regs().SR & SPI_SR_FRE) != 0u; }
    static void clear_frame_error() { (void)regs().SR; }

    static void dma_requests(bool tx, bool rx) { Spi<n>::dma_requests(tx, rx); }
    static void rxne_interrupt(bool on) { Spi<n>::rxne_interrupt(on); }
    static void txe_interrupt(bool on) { Spi<n>::txe_interrupt(on); }
    static void error_interrupt(bool on) { Spi<n>::error_interrupt(on); }

    /// The raised-and-enabled sources (table 128), consuming nothing.
    [[gnu::always_inline]] static uint32_t isr() {
        const uint32_t sr = regs().SR;
        const uint32_t cr2 = regs().CR2;
        uint32_t up = 0;
        if ((cr2 & SPI_CR2_RXNEIE) != 0u && (sr & SPI_SR_RXNE) != 0u) { up |= SpiFlag::rxne; }
        if ((cr2 & SPI_CR2_TXEIE) != 0u && (sr & SPI_SR_TXE) != 0u) { up |= SpiFlag::txe; }
        if ((cr2 & SPI_CR2_ERRIE) != 0u) {
            up |= sr & (SpiFlag::overrun | SpiFlag::underrun | SpiFlag::frame_error);
        }
        return up;
    }

private:
    template <typename Pred>
    static bool wait_until(Pred pred) {
        for (uint32_t spins = 400'000u; spins != 0u; --spins) {
            if (pred()) {
                return true;
            }
        }
        return false;
    }
};

/// The full-duplex extension block of instance `n` - a slave-only second
/// cell on the instance's own CK and WS.
template <uint8_t n>
using I2sExt = I2s<n, true>;

// =============================================================================
// The pads
// =============================================================================

/**
 * The four pads an SPI link can claim, each with the alternate function
 * the DATASHEET gives that signal on that pad (AF5 for SPI1, SPI2, SPI4,
 * SPI5 and SPI6, AF6 for SPI3, with per-pad exceptions the tables settle;
 * the I2S signals ride the same pads under the same numbers). MISO and
 * NSS may be left null: a write-only bus needs neither, and the engine's
 * chip select is a GPIO the Request carries.
 */
struct SpiPins {
    PinSel sck{};
    PinSel miso{};
    PinSel mosi{};
    PinSel nss{};
};

/// A link needs SCK, and no two signals may name the same pad.
constexpr bool spi_pins_valid(const SpiPins& p) {
    if (!p.sck.valid()) {
        return false;
    }
    const PinSel all[] = {p.sck, p.miso, p.mosi, p.nss};
    for (uint8_t i = 0; i < 4u; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < 4u; ++j) {
            if (all[i].valid() && all[j].valid() && all[i].port == all[j].port &&
                all[i].pin == all[j].pin) {
                return false;
            }
        }
    }
    return true;
}

/**
 * The four pads an I2S link can claim: the bit clock, the word select, the
 * data line and the optional master clock output. They are the SPI's pads
 * under other names - CK is SCK's pad, WS is NSS's, SD is MOSI's on the
 * instance and MISO's on the extension block - but a program reads the
 * audio names, so the type spells them.
 */
struct I2sPins {
    PinSel ck{};
    PinSel ws{};
    PinSel sd{};
    PinSel mck{};
};

constexpr bool i2s_pins_valid(const I2sPins& p) {
    if (!p.ck.valid() || !p.ws.valid() || !p.sd.valid()) {
        return false;
    }
    const PinSel all[] = {p.ck, p.ws, p.sd, p.mck};
    for (uint8_t i = 0; i < 4u; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < 4u; ++j) {
            if (all[i].valid() && all[j].valid() && all[i].port == all[j].port &&
                all[i].pin == all[j].pin) {
                return false;
            }
        }
    }
    return true;
}

/// Two engines on one bus must not name the same STREAM: a stream moves
/// data one way and has one FIFO.
template <typename Tx, typename Rx>
constexpr bool spi_engines_distinct() {
    if constexpr (Tx::present && Rx::present) {
        return Tx::controller != Rx::controller || Tx::stream != Rx::stream;
    } else {
        return true;
    }
}

/// Whether an engine sits on a cell of the request mapping that really
/// carries instance `n`'s transmit (or receive) request. True for an
/// absent engine, and FALSE on a part class whose request table was not
/// read - a refusal, never a guess.
template <uint8_t n, typename E, bool transmit>
constexpr bool spi_engine_placed() {
    if constexpr (!E::present) {
        return true;
    } else {
        return spi_dma_placement_valid(n, transmit, E::controller, E::stream, E::channel);
    }
}

// =============================================================================
// SpiHost: the bus engine
// =============================================================================

/// A DMA block the engines could not finish (a transfer error on either
/// stream, or a polled block that never completed): the engine's own
/// status code, in the range util/bus_master.hpp leaves to engines.
inline constexpr uint8_t spi_dma_fault = bus_engine_status;

/**
 * SpiHost<n, pins, TxEngine, RxEngine>
 *
 * The engine SpiBus (util/spi_bus.hpp = BusMaster) drives. Its Request is
 * the other strata's, field for field: the bus AO asserts the request's
 * own chip select around the transaction, a COMMAND phase goes out with
 * D/C low and a DATA phase with it high, the data phase has an optional
 * out buffer (null = 0xFF dummies) and an optional in buffer (null = the
 * frames read back are discarded), and every buffer is LENT until the
 * reply lands. The mode, rate and frame size travel per request, so a
 * shared bus serves devices that disagree on them; the bit order is a
 * property of the WIRE and a bus-level verb.
 *
 * THE PUMP RUNS ON RXNE. A frame written to DR is clocked out and the
 * frame that comes back raises RXNE when it has been shifted BOTH ways -
 * the one moment the bus is provably idle - and TXE, which rises a frame
 * early, is never the pump's edge.
 *
 * TWO COMPLETION STYLES, the request's choice: `polled` false runs the
 * data on this interrupt with the kernel free between frames (a
 * TransferDone follows), `polled` true spins inside start() with only
 * this SPI's own RXNE silenced and completes synchronously.
 *
 * THE ENGINE SLOTS carry the DATA PHASE, both or neither (the phase is
 * full duplex and its completion is the RECEIVE block's), on cells the
 * request mapping really gives this instance. A request in 16-bit frames
 * falls back to the pump: the engines carry bytes.
 *
 * FRAMES IN A BYTE BUFFER: one byte per 8-bit frame, two bytes low-first
 * per 16-bit frame - the other strata's rule.
 *
 * THE PADS GO OUT AT `very_high`, and that is ES0206 2.12.4 speaking, not
 * a taste: the SCK pad's own feedback delay is what decides whether the
 * last received bit is captured, and the errata's table gives the APB
 * ceiling per speed class. `sck_speed()` moves it for a program that
 * wants to weigh the effect. `mosi_speed()` moves the DATA pad, which
 * no erratum asks for and a bus of wires does: at PCLK2/8 on a
 * breadboard the device took nothing of an interrupt-pumped request
 * until SCK or MOSI was slowed to `medium` (docs/stm32f4/spi.md), and
 * the pad a program can afford to slow is the one whose edge is not a
 * correctness parameter of the silicon.
 */
template <uint8_t n, SpiPins pins, typename TxEngine = NoDmaEngine,
          typename RxEngine = NoDmaEngine>
class SpiHost {
    using S = Spi<n>;

    static_assert(sizeof(TxEngine) > 0 && sizeof(RxEngine) > 0,
                  "the engine slots must name a complete type: a DmaTxEngine / "
                  "DmaRxEngine from stm32f4/dma.hpp, or NoDmaEngine (the default)");
    static_assert(TxEngine::present == RxEngine::present,
                  "brio SpiHost: name both DMA engines or neither - the data phase is "
                  "full-duplex and its completion is the RECEIVE block's");
    static_assert(spi_engines_distinct<TxEngine, RxEngine>(),
                  "brio SpiHost: the transmit and receive engines must use DIFFERENT DMA "
                  "streams - a stream carries one direction and has one FIFO");
    static_assert(spi_engine_placed<n, TxEngine, true>(),
                  "brio SpiHost: the transmit engine's (controller, stream, channel) is not "
                  "a cell this instance's transmit request is wired to - RM0090 tables 43 "
                  "and 44 and their RM0390 / RM0383 twins, keyed per part class in "
                  "stm32f4/device_tables.hpp (a part class whose manual was not read has no "
                  "table, and an engine is refused there rather than guessed)");
    static_assert(spi_engine_placed<n, RxEngine, false>(),
                  "brio SpiHost: the receive engine's (controller, stream, channel) is not a "
                  "cell this instance's receive request is wired to - see the transmit "
                  "engine's message");
    static_assert(spi_pins_valid(pins),
                  "brio SpiHost: these SPI pads are not a link - SCK is required, every pad "
                  "named must be a real pin of a present port, and no two signals may name "
                  "the same pad");
    static_assert(pins.mosi.valid(),
                  "brio SpiHost: a host needs SCK and MOSI; MISO is optional only for a "
                  "write-only bus, and the pump reads whatever comes back either way");

    using SckPin = Pin<pins.sck.port, pins.sck.pin>;
    using MosiPin = Pin<pins.mosi.port, pins.mosi.pin>;
    using MisoPin = Pin<pins.miso.valid() ? pins.miso.port : pins.sck.port,
                        pins.miso.valid() ? pins.miso.pin : pins.sck.pin>;
    using NssPin = Pin<pins.nss.valid() ? pins.nss.port : pins.sck.port,
                       pins.nss.valid() ? pins.nss.pin : pins.sck.pin>;

public:
    SpiHost() = delete;

    using Resource = S;
    static constexpr SpiPins pin_sel = pins;
    static constexpr bool has_engines = TxEngine::present;

    struct Request {
        PinRef cs;   ///< asserted low around the transaction
        PinRef dc;   ///< display D/C line; null = no such pin
        /// Microseconds between the CS assertion and the first clock -
        /// what a device's datasheet calls CS setup. Spent spinning in
        /// start(), main context; 0 = none.
        uint8_t cs_setup_us = 0;
        /// Phase 1, sent with DC low; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> cmd;
        uint8_t cmd_len;   ///< in FRAMES (see the class comment)
        /// Phase 2 out, null = 0xFF dummies; LENT until the reply lands.
        Borrowed<const uint8_t, Lease::reply> tx;
        /// Phase 2 in, null = discard; LENT until the reply lands.
        Borrowed<uint8_t, Lease::reply> rx;
        uint16_t len;      ///< phase 2 length, in FRAMES
        ReplyTo<SpiDone> reply;

        /// Per-transaction bus configuration: a shared bus's devices each
        /// name their own, and the engine reprograms the peripheral only
        /// when something CHANGED.
        SpiClock clock = SpiClock::div16;
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;

        /// Completion style: false = per-frame ISR pump, true = POLLED
        /// inside start() (see the class comment).
        bool polled = false;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    // ---- lifecycle ------------------------------------------------------------------

    /**
     * Bring the instance up as a host: gate, reset, configuration, pads,
     * the NVIC line. `clock` is the app's Clock tag and the rate the BR
     * field divides is THE INSTANCE'S OWN APB CLOCK, never SYSCLK.
     * `max_sck_hz` is an optional CEILING for the whole bus, re-resolved
     * on rebase(); 0 = none. False when the ceiling cannot be produced at
     * this clock, or the boot configuration is refused.
     */
    template <typename Clock>
    static bool init(Clock clock, uint32_t max_sck_hz = 0) {
        static_assert(clock_follows<Clock, SpiHost>(),
                      "this SpiHost is initialized with a DynamicClock that does not list "
                      "it among its Users: its SCK ceiling and its cs_setup timing would go "
                      "stale on a clock change");
        Nvic::disable(S::irq);
        ceiling_hz_ = max_sck_hz;
        rebase_pclk(apb_hz(clock, S::on_apb2), clock_hz(clock));
        if (max_sck_hz != 0u && !ceiling_) {
            return false;   // not even PCLK/256 honours it
        }
        S::bus_clock(true);
        S::reset();
        applied_ = boot_config();
        if (!S::configure(applied_)) {
            return false;
        }
        status_ = spi_ok;
        if constexpr (has_engines) {
            TxEngine::arm(S::data_address());
            RxEngine::arm(S::data_address());
        }
        // The pads go to the peripheral only now, with the registers
        // already holding the idle polarity: a pad handed over first would
        // park undriven on the bus, and a client watching SCK cannot tell
        // a glitch from an edge. SPE is raised last.
        apply_sck_pad();
        apply_mosi_pad();
        if constexpr (pins.miso.valid()) {
            MisoPin::function(pins.miso.function, {.pull = PinPull::up});
        }
        // THE NSS PAD IS NOT CLAIMED: the boot configuration is
        // SpiNss::software and the pad is free under it - which is what
        // this engine's chip select is, an ordinary GPIO the Request
        // carries. claim_nss_pad() is for a program that moves the
        // resource to a hardware arrangement.
        S::enable();
        S::flush_rx();
        Nvic::enable(S::irq);
        return true;
    }

    /// The core clock changed (DynamicClock fan-out). A Request's `clock`
    /// is a DIVISION of the instance's APB clock and scales by itself;
    /// what is recomputed is the ceiling and the cs_setup timing. `hz` is
    /// SYSCLK, as every other task's rebase takes it - the APB divider in
    /// force is read back from the RCC. THE BUS MUST BE IDLE.
    static void rebase(uint32_t hz) {
        const uint32_t pclk = hz / (S::on_apb2 ? Rcc::apb2_divider() : Rcc::apb1_divider());
        rebase_pclk(pclk, hz);
    }

    /// The BR code that produces at most `hz` of SCK at the APB clock last
    /// seen - the chooser a device's datasheet limit is spoken to.
    static std::optional<SpiClock> clock_for(uint32_t hz) { return spi_rate_for(pclk_hz_, hz); }
    /// What a request at this code really runs at, ceiling included.
    static uint32_t sck_hz(SpiClock c) { return spi_sck_hz(pclk_hz_, clamp(c)); }
    static uint32_t max_sck_hz() { return ceiling_hz_; }
    static std::optional<SpiClock> ceiling_clock() { return ceiling_; }
    /// The rate the BR field divides: this instance's APB clock.
    static uint32_t reference_hz() { return pclk_hz_; }

    /// Put the peripheral at this mode, rate and frame size NOW, moving no
    /// data - for callers that frame the select window themselves (Request
    /// .cs null). A mode change is a CPOL FLIP ON THE WIRE and a selected
    /// client counts it as an edge: prime FIRST, then select.
    static void prime(SpiMode m, SpiClock c, SpiDataSize bits = SpiDataSize::bits8) {
        apply(m, clamp(c), bits);
    }

    /// The bus's bit order (LSBFIRST): a property of the WIRE, applied
    /// once, not a per-request field. The bus must be idle.
    static bool bit_order(bool lsb_first) {
        if (applied_.lsb_first == lsb_first) {
            return true;
        }
        applied_.lsb_first = lsb_first;
        (void)S::disable();
        const bool ok = S::configure(applied_);
        S::enable();
        return ok;
    }
    static bool lsb_first() { return applied_.lsb_first; }

    /// The SCK pad's slew class. `very_high` is what init() hands over and
    /// what ES0206 2.12.4's table asks for at any APB above 75 MHz; the
    /// verb exists so a program can weigh the errata rather than take it
    /// on trust.
    static void sck_speed(PinSpeed s) {
        sck_speed_ = s;
        apply_sck_pad();
    }
    static PinSpeed sck_speed() { return sck_speed_; }
    /// The MOSI pad's slew class, `very_high` from init(). No erratum
    /// names it; a bus of long wires does (the class comment), and it is
    /// the pad a program slows first because ES0206 2.12.4 has nothing
    /// to say about it.
    static void mosi_speed(PinSpeed s) {
        mosi_speed_ = s;
        apply_mosi_pad();
    }
    static PinSpeed mosi_speed() { return mosi_speed_; }
    /// The APB ceiling this pad speed buys, from the errata's table - and
    /// whether the bus is running under it right now.
    static uint32_t errata_apb_ceiling_hz() { return spi_errata_apb_ceiling_hz(sck_speed_); }
    static bool within_errata_ceiling() { return pclk_hz_ <= errata_apb_ceiling_hz(); }

    // ---- the transfer -----------------------------------------------------------------

    /// Begin a transaction (SpiBus calls it from main context). True when
    /// it completed SYNCHRONOUSLY (polled requests, the empty one); false
    /// when it runs on the ISR and a TransferDone follows - exactly
    /// util/bus_master.hpp's engine contract.
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        in_cmd_ = (r.cmd_len > 0u);
        status_ = spi_ok;
        if (total_len() == 0u) {
            return true;
        }
        apply(r.mode, clamp(r.clock), r.bits);
        if (in_cmd_) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();   // assert, active low
        if (r.cs_setup_us != 0u) {
            (void)delay_us(cs_rate_, r.cs_setup_us);
        }
        S::flush_rx();   // a stale frame would be captured as this one's

        if constexpr (has_engines) {
            if (dma_serves(r)) {
                if (!r.polled) {
                    if (in_cmd_) {
                        // The command phase runs on the pump; isr() hands
                        // over to the engines at its end.
                        S::rxne_interrupt(true);
                        S::data(frame_at(req_.cmd.get(), 0));
                        return false;
                    }
                    launch_dma();
                    return false;   // dma_isr() is the completion edge
                }
                S::rxne_interrupt(false);
                for (uint8_t i = 0; i < r.cmd_len; ++i) {
                    (void)xfer(frame_at(r.cmd.get(), i));
                }
                r.dc.set();
                if (r.len != 0u) {
                    launch_dma();
                    spin_dma();
                }
                r.cs.set();
                return true;
            }
        }

        if (!r.polled) {
            S::rxne_interrupt(true);
            S::data(first_frame());   // the ISR pumps the rest
            return false;
        }
        // Polled pump: silence this SPI's own RXNE (a bound handler would
        // steal the frames). Global interrupts STAY ENABLED.
        S::rxne_interrupt(false);
        for (uint8_t i = 0; i < r.cmd_len; ++i) {
            (void)xfer(frame_at(r.cmd.get(), i));
        }
        r.dc.set();
        for (uint16_t i = 0; i < r.len; ++i) {
            const uint16_t in = xfer(data_frame(i));
            if (r.rx.get() != nullptr) {
                store_frame(r.rx.get(), i, in);
            }
        }
        r.cs.set();
        return true;
    }

    /// The instance's interrupt body - call from its vector. Only RXNE is
    /// ever armed by this engine, and reading DR is both the capture and
    /// the acknowledgement. True when the transaction just completed (CS
    /// released): the edge the app's glue posts TransferDone on.
    [[gnu::always_inline]] static bool isr() {
        if ((S::isr() & SpiFlag::rxne) == 0u) {
            return false;
        }
        const uint16_t in = S::data(req_.bits);

        if constexpr (has_engines) {
            if (dma_serves(req_)) {
                // Only the COMMAND phase runs on this pump when the
                // engines serve the request; its echo is discarded.
                (void)in;
                ++pos_;
                if (pos_ >= req_.cmd_len) {
                    in_cmd_ = false;
                    S::rxne_interrupt(false);
                    req_.dc.set();
                    if (req_.len == 0u) {
                        req_.cs.set();
                        return true;
                    }
                    launch_dma();
                    return false;
                }
                S::data(frame_at(req_.cmd.get(), pos_));
                return false;
            }
        }

        if (!in_cmd_ && req_.rx.get() != nullptr) {
            store_frame(req_.rx.get(), pos_, in);
        }
        ++pos_;
        if (in_cmd_ && pos_ >= req_.cmd_len) {
            in_cmd_ = false;
            pos_ = 0;
            req_.dc.set();
        }
        if (!in_cmd_ && pos_ >= req_.len) {
            S::rxne_interrupt(false);
            req_.cs.set();
            return true;
        }
        S::data(next_frame());
        return false;
    }

    /// The DMA streams' interrupt body - call from BOTH streams' vectors
    /// (one vector per stream on this family, shared with nothing). A
    /// transfer error on either ends the transaction with spi_dma_fault.
    /// Compiles away on an engineless host. True when the transaction just
    /// completed.
    [[gnu::always_inline]] static bool dma_isr() {
        if constexpr (has_engines) {
            const uint8_t tx = TxEngine::service();
            if ((tx & TxEngine::flag_error) != 0u) {
                (void)TxEngine::abandon();
                return finish_dma(spi_dma_fault);
            }
            if ((tx & TxEngine::flag_complete) != 0u) {
                (void)TxEngine::complete();   // the transmit side never completes a transaction
            }
            const uint8_t rx = RxEngine::service();
            if (rx != 0u && dma_active_) {
                if ((rx & RxEngine::flag_error) != 0u) {
                    return finish_dma(spi_dma_fault);
                }
                if ((rx & RxEngine::flag_complete) != 0u) {
                    return finish_dma(spi_ok);
                }
            }
        }
        return false;
    }

    /// The engine's completion status, for the TransferDone payload.
    static uint8_t status() { return status_; }

    /// Put the ENGINE back where start() is legal (the verb a timed SpiBus
    /// calls on a transaction that never answered): the select released
    /// FIRST, the engines put away and re-claimed, the peripheral reset
    /// and reconfigured to the applied state. False when a bounded wait
    /// ran out.
    static bool recover() {
        req_.cs.set();
        if constexpr (has_engines) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address());
        }
        Nvic::disable(S::irq);
        in_cmd_ = false;
        dma_active_ = false;
        dma_done_ = false;
        S::rxne_interrupt(false);
        const bool ok = S::disable();
        S::reset();
        const bool cfg = S::configure(applied_);
        S::enable();
        S::flush_rx();
        Nvic::enable(S::irq);
        return ok && cfg;
    }

    static void release() {
        if constexpr (has_engines) {
            TxEngine::stop();
            RxEngine::stop();
        }
        Nvic::disable(S::irq);
        (void)S::disable();
        S::bus_clock(false);
        SckPin::release();
        MosiPin::release();
        if constexpr (pins.miso.valid()) {
            MisoPin::release();
        }
    }

    /// Hand the NSS pad to the peripheral - only for a program that has
    /// moved the resource to a hardware arrangement. The task's own
    /// transactions never use it.
    static void claim_nss_pad(bool on) {
        if constexpr (pins.nss.valid()) {
            if (on) {
                NssPin::function(pins.nss.function, {.speed = PinSpeed::very_high});
            } else {
                NssPin::release();
            }
        } else {
            (void)on;
        }
    }

private:
    static_assert([] {
        if constexpr (TxEngine::present) {
            return std::is_same_v<typename TxEngine::element, uint8_t> &&
                   std::is_same_v<typename RxEngine::element, uint8_t>;
        } else {
            return true;
        }
    }(), "brio SpiHost: the DMA engines must carry uint8_t elements - the Request's "
         "buffers are bytes and the engined path serves 8-bit frames");

    /// The engines serve a request in 8-bit frames; 16-bit ones are two
    /// bytes a datum, which byte engines cannot express.
    static bool dma_serves(const Request& r) { return !spi_frame_is_halfword(r.bits); }

    static void launch_dma() {
        dma_done_ = false;
        dma_active_ = true;
        // THE REQUESTS ARE ARMED AROUND THE ENGINES AND NOWHERE ELSE. The
        // handshake of 10.3.2 is level-driven: a request that stands while
        // its stream is disabled is served the moment the stream is
        // enabled, so RXDMAEN raised before the receive stream is running
        // would hand the block the echo of a command frame - or the stale
        // frame a previous transaction left in DR - as its first byte. So
        // RXDMAEN goes up only once the receive stream is enabled, TXDMAEN
        // only once the transmit one is, and finish_dma() drops both.
        // THE RECEIVE STREAM FIRST: its request rises only when a frame
        // has come back, and the transmit side is what starts the clock.
        if (req_.rx.get() != nullptr) {
            (void)RxEngine::start(req_.rx.get(), req_.len);
        } else {
            (void)RxEngine::start_discard(&rx_sink_, req_.len);
        }
        S::dma_requests(false, true);
        if (req_.tx.get() != nullptr) {
            (void)TxEngine::start(req_.tx.get(), req_.len);
        } else {
            (void)TxEngine::start_fixed(&tx_dummy_, req_.len);
        }
        S::dma_requests(true, true);
    }

    /// One exit for the data phase. True when the ISR-style caller should
    /// post completion.
    static bool finish_dma(uint8_t st) {
        S::dma_requests(false, false);
        if (st != spi_ok) {
            status_ = st;
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address());
        } else if (TxEngine::busy()) {
            // The receive block completing is proof the transmit one did:
            // every frame that came back was clocked out first.
            (void)TxEngine::complete();
        }
        dma_active_ = false;
        dma_done_ = true;
        if (!req_.polled) {
            req_.cs.set();
            return true;
        }
        return false;
    }

    /// The polled request's wait on the DMA completion - bounded, and
    /// scaled against the slowest frame this block can clock.
    static void spin_dma() {
        uint32_t spins = 200'000u + 6'000u * static_cast<uint32_t>(req_.len);
        while (!dma_done_ && spins-- != 0u) {
        }
        if (!dma_done_) {
            S::dma_requests(false, false);
            (void)TxEngine::abandon();
            RxEngine::stop();
            RxEngine::arm(S::data_address());
            dma_active_ = false;
            status_ = spi_dma_fault;
        }
    }

    static void rebase_pclk(uint32_t pclk, uint32_t sysclk) {
        pclk_hz_ = pclk;
        ceiling_ = ceiling_hz_ != 0u ? spi_rate_for(pclk, ceiling_hz_) : std::optional<SpiClock>{};
        cs_rate_ = delay_rate(sysclk);   // the busy-wait counts core cycles
    }

    static void apply_sck_pad() {
        SckPin::function(pins.sck.function, {.speed = sck_speed_});
    }
    static void apply_mosi_pad() {
        MosiPin::function(pins.mosi.function, {.speed = mosi_speed_});
    }

    /// A slower rate is a LARGER code, so the ceiling clamps from below.
    static SpiClock clamp(SpiClock c) {
        if (!ceiling_) {
            return c;
        }
        return static_cast<uint8_t>(c) < static_cast<uint8_t>(*ceiling_) ? *ceiling_ : c;
    }

    static SpiConfig boot_config() {
        SpiConfig c{};
        c.role = SpiRole::host;
        c.mode = SpiMode::mode0;
        c.bits = SpiDataSize::bits8;
        c.clock = ceiling_ ? *ceiling_ : SpiClock::div16;
        c.nss = SpiNss::software;
        // The DMA requests are NOT part of the applied configuration:
        // launch_dma() raises them around the engines (see there).
        return c;
    }

    /// Put the peripheral where this request wants it. DFF is
    /// enable-protected and BR/CPOL/CPHA "should not be changed when
    /// communication is ongoing", so any change costs a disable/enable
    /// pair - paid only when something moved.
    static void apply(SpiMode m, SpiClock c, SpiDataSize bits) {
        if (m == applied_.mode && c == applied_.clock && bits == applied_.bits) {
            return;
        }
        applied_.mode = m;
        applied_.clock = c;
        applied_.bits = bits;
        (void)S::disable();
        (void)S::configure(applied_);
        S::enable();
    }

    /// One polled frame: write, spin on RXNE, read back. Bounded.
    static uint16_t xfer(uint16_t out) {
        S::data(out);
        uint32_t spins = 400'000u;
        while (!S::rx_ready() && spins-- != 0u) {
        }
        return S::data(req_.bits);
    }

    static uint16_t total_len() { return static_cast<uint16_t>(req_.cmd_len) + req_.len; }

    static uint16_t frame_at(const uint8_t* p, uint16_t i) {
        if (p == nullptr) {
            return 0xFFFFu;
        }
        if (spi_frame_is_halfword(req_.bits)) {
            const uint16_t k = static_cast<uint16_t>(2u * i);
            return static_cast<uint16_t>(p[k] | (static_cast<uint16_t>(p[k + 1u]) << 8));
        }
        return p[i];
    }
    static void store_frame(uint8_t* p, uint16_t i, uint16_t v) {
        if (spi_frame_is_halfword(req_.bits)) {
            const uint16_t k = static_cast<uint16_t>(2u * i);
            p[k] = static_cast<uint8_t>(v);
            p[k + 1u] = static_cast<uint8_t>(v >> 8);
        } else {
            p[i] = static_cast<uint8_t>(v);
        }
    }
    static uint16_t first_frame() { return in_cmd_ ? frame_at(req_.cmd.get(), 0) : data_frame(0); }
    static uint16_t next_frame() { return in_cmd_ ? frame_at(req_.cmd.get(), pos_) : data_frame(pos_); }
    static uint16_t data_frame(uint16_t i) {
        return (req_.tx.get() != nullptr) ? frame_at(req_.tx.get(), i) : 0xFFFFu;
    }

    static inline Request req_{};
    static inline uint16_t pos_ = 0;
    static inline bool in_cmd_ = false;
    static inline uint8_t status_ = spi_ok;
    static inline volatile bool dma_done_ = false;
    static inline volatile bool dma_active_ = false;
    static inline uint8_t rx_sink_ = 0;
    static constexpr uint8_t tx_dummy_ = 0xFF;
    static inline SpiConfig applied_{};
    static inline uint32_t pclk_hz_ = 0;
    static inline uint32_t ceiling_hz_ = 0;
    static inline std::optional<SpiClock> ceiling_{};
    static inline DelayRate cs_rate_{};
    static inline PinSpeed sck_speed_ = PinSpeed::very_high;
    static inline PinSpeed mosi_speed_ = PinSpeed::very_high;
};

// =============================================================================
// SpiClient: the other end of the wire
// =============================================================================

/**
 * SpiClient<n, pins>
 *
 * SCK, MOSI and NSS are inputs, the host sets the pace, and the only
 * thing this side controls is WHAT it has ready to shift out when the
 * next clock arrives.
 *
 * ONE FRAME AHEAD. With no FIFO the transmit buffer holds exactly the
 * next answer: the first frame moves from the buffer into the shifter on
 * the first sampling edge and TXE rises then - the moment to write the
 * NEXT one. So load the first answer before the host selects
 * (`enable(first)`), then write frame k + 1 on TXE while frame k shifts.
 * A client that falls behind does not stall the bus: what the host reads
 * is whatever the shifter held, a wrong answer and never a missing one.
 *
 * THE NSS PAD IS THE TRANSACTION: read directly (`selected()`), since the
 * peripheral publishes no select status. `drive_output` makes this client
 * a DARK LISTENER - MISO is handed to the peripheral only for the window
 * it answers in, so a shared harness stays uncontested.
 *
 * ES0206 2.12.5: a client's BSY may stay high at the end of a transfer
 * when the core clock and the host's SCK synchronize badly, so nothing
 * here uses BSY to decide that a transaction ended - RXNE is the witness
 * on the receiving side and the NSS pad on both.
 */
template <uint8_t n, SpiPins pins>
class SpiClient {
    using S = Spi<n>;

    static_assert(spi_pins_valid(pins),
                  "brio SpiClient: these SPI pads are not a link - SCK is required and no "
                  "two signals may name the same pad");
    static_assert(pins.mosi.valid(), "brio SpiClient: a client listens on MOSI");

    using SckPin = Pin<pins.sck.port, pins.sck.pin>;
    using MosiPin = Pin<pins.mosi.port, pins.mosi.pin>;
    using MisoPin = Pin<pins.miso.valid() ? pins.miso.port : pins.sck.port,
                        pins.miso.valid() ? pins.miso.pin : pins.sck.pin>;
    using NssPin = Pin<pins.nss.valid() ? pins.nss.port : pins.sck.port,
                       pins.nss.valid() ? pins.nss.pin : pins.sck.pin>;

public:
    SpiClient() = delete;

    using Resource = S;
    static constexpr SpiPins pin_sel = pins;
    static constexpr bool has_nss_pad = pins.nss.valid();

    struct Config {
        SpiMode mode = SpiMode::mode0;
        SpiDataSize bits = SpiDataSize::bits8;
        bool lsb_first = false;
        /// software: the pad is free and select() drives SSI.
        /// hardware_input: the NSS pad IS the chip select (the default).
        SpiNss nss = SpiNss::hardware_input;
        SpiDirection direction = SpiDirection::full_duplex;
        SpiFrameFormat frame_format = SpiFrameFormat::motorola;
        bool crc = false;
        uint16_t crc_polynomial = 0x0007u;
        /// Whether the MISO pad is handed to the peripheral at all.
        bool drive_output = true;
    };

    /// How many answers must be queued ahead of the frame the host is
    /// about to clock: ONE here - no FIFO, one buffer.
    static constexpr uint8_t frames_ahead = 1;

    /// Bring the instance up as a client. The bus clock is required even
    /// though the shifter runs on the host's SCK. False when the
    /// configuration is refused.
    template <typename Clock>
    static bool init(Clock clock, const Config& cfg = {}) {
        (void)clock;
        Nvic::disable(S::irq);
        S::bus_clock(true);
        S::reset();
        if (!S::configure(config_of(cfg))) {
            return false;
        }
        // Inputs first, the output last.
        SckPin::function(pins.sck.function);
        MosiPin::function(pins.mosi.function);
        if constexpr (has_nss_pad) {
            if (cfg.nss != SpiNss::software) {
                NssPin::function(pins.nss.function, {.pull = PinPull::up});
            }
        }
        drive_output(cfg.drive_output);
        S::flush_rx();
        Nvic::enable(S::irq);
        return true;
    }

    /// Hand the answer line to the peripheral, or take it back (parked in
    /// analog, driving nothing).
    static void drive_output(bool on) {
        if constexpr (pins.miso.valid()) {
            if (on) {
                MisoPin::function(pins.miso.function, {.speed = PinSpeed::very_high});
            } else {
                MisoPin::release();
            }
        } else {
            (void)on;
        }
    }

    /// SPE up and the FIRST ANSWER in the buffer before the host's clock
    /// can arrive. Call it with NSS still high.
    static void enable(uint16_t first = 0xFFFFu) {
        S::enable();
        write(first);
    }
    static bool disable() { return S::disable(); }

    /// Load the next frame to shift out; on TXE, for the one-ahead rule.
    static void write(uint16_t v) { S::data(v); }
    static bool writable() { return S::tx_empty(); }

    /// One received frame, or nothing. Reading DR clears RXNE.
    static std::optional<uint16_t> poll() {
        if (!S::rx_ready()) {
            return {};
        }
        return S::data(bits_);
    }

    /// Is the host holding NSS low right now? A live pad read - and the
    /// pad must be an input of this port for it to mean anything. Always
    /// false without an NSS pad.
    static bool selected() {
        if constexpr (has_nss_pad) {
            return !NssPin::read();
        } else {
            return false;
        }
    }

    /// The software select, for a client under SSM.
    static void select(bool on) { S::software_select(on); }

    static bool overrun() { return S::overrun(); }
    static void clear_overrun() { S::clear_overrun(); }
    static bool crc_error() { return S::crc_error(); }
    static void clear_crc_error() { S::clear_crc_error(); }
    static bool frame_error() { return S::frame_error(); }
    static uint32_t status() { return S::status(); }

    /// The ISR body: the raised-and-enabled sources, for the app's glue.
    [[gnu::always_inline]] static uint32_t isr() { return S::isr(); }

    static void rxne_interrupt(bool on) { S::rxne_interrupt(on); }
    static void txe_interrupt(bool on) { S::txe_interrupt(on); }
    static void error_interrupt(bool on) { S::error_interrupt(on); }

    static SpiDataSize bits() { return bits_; }

    static void release() {
        Nvic::disable(S::irq);
        (void)S::disable();
        S::bus_clock(false);
        SckPin::release();
        MosiPin::release();
        if constexpr (pins.miso.valid()) {
            MisoPin::release();
        }
        if constexpr (has_nss_pad) {
            NssPin::release();
        }
    }

private:
    static SpiConfig config_of(const Config& c) {
        bits_ = c.bits;
        SpiConfig s{};
        s.role = SpiRole::client;
        s.mode = c.mode;
        s.bits = c.bits;
        s.lsb_first = c.lsb_first;
        s.nss = c.nss;
        s.direction = c.direction;
        s.frame_format = c.frame_format;
        s.crc = c.crc;
        s.crc_polynomial = c.crc_polynomial;
        return s;
    }

    static inline SpiDataSize bits_ = SpiDataSize::bits8;
};

// =============================================================================
// The chapter's own arithmetic, pinned at compile time
// =============================================================================

// 28.5.1's BR table at the two APB rates a 180 MHz part runs.
static_assert(spi_sck_hz(90'000'000UL, SpiClock::div2) == 45'000'000UL);
static_assert(spi_sck_hz(90'000'000UL, SpiClock::div256) == 351'562UL);
static_assert(spi_sck_hz(45'000'000UL, SpiClock::div16) == 2'812'500UL);
static_assert(spi_rate_for(90'000'000UL, 10'000'000UL) == SpiClock::div16);
static_assert(spi_rate_for(90'000'000UL, 45'000'000UL) == SpiClock::div2);
static_assert(spi_rate_for(90'000'000UL, 44'999'999UL) == SpiClock::div4);
// A ceiling below PCLK/256 is REFUSED, never rounded up.
static_assert(!spi_rate_for(90'000'000UL, 351'561UL).has_value());
static_assert(!spi_rate_for(0, 1'000'000UL).has_value());

static_assert(!spi_frame_is_halfword(SpiDataSize::bits8) && spi_frame_is_halfword(SpiDataSize::bits16));
static_assert(spi_frame_mask(SpiDataSize::bits8) == 0xFFu && spi_data_bits(SpiDataSize::bits16) == 16u);

// The four modes.
static_assert(!spi_mode_cpol(SpiMode::mode1) && spi_mode_cpha(SpiMode::mode1));
static_assert(spi_mode_cpol(SpiMode::mode2) && !spi_mode_cpha(SpiMode::mode2));
static_assert(spi_mode_cpol(SpiMode::mode3) && spi_mode_cpha(SpiMode::mode3));

// The refusals spi_config_valid() owes the chapter and the errata.
static_assert(spi_config_valid(SpiConfig{}));
static_assert(!spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0x1000u}));
static_assert(!spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0}));
static_assert(spi_config_valid(SpiConfig{.crc = true, .crc_polynomial = 0x1021u | 1u}));
static_assert(!spi_config_valid(SpiConfig{.role = SpiRole::client, .nss = SpiNss::hardware_output}));
static_assert(!spi_config_valid(SpiConfig{.nss = SpiNss::software,
                                          .frame_format = SpiFrameFormat::ti}));
static_assert(!spi_config_valid(SpiConfig{.lsb_first = true, .nss = SpiNss::hardware_output,
                                          .frame_format = SpiFrameFormat::ti}));

// The control words: a mode 3 host at /16 under software select.
static_assert(spi_cr1_of(SpiConfig{.mode = SpiMode::mode3, .clock = SpiClock::div16}) ==
              (SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_MSTR | (3u << SPI_CR1_BR_Pos) | SPI_CR1_SSM |
               SPI_CR1_SSI));
// A client under software management keeps SSI LOW: that is "selected".
static_assert((spi_cr1_of(SpiConfig{.role = SpiRole::client}) & (SPI_CR1_SSM | SPI_CR1_SSI)) == SPI_CR1_SSM);
static_assert(spi_cr1_of(SpiConfig{.direction = SpiDirection::half_duplex_out}) &
              (SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE));
static_assert((spi_cr1_of(SpiConfig{.direction = SpiDirection::receive_only}) & SPI_CR1_RXONLY) != 0u);
static_assert(spi_cr2_of(SpiConfig{.nss = SpiNss::hardware_output, .dma_receive = true}) ==
              (SPI_CR2_SSOE | SPI_CR2_RXDMAEN));
static_assert(spi_master_receive_only(SpiConfig{.direction = SpiDirection::receive_only}));
static_assert(!spi_master_receive_only(SpiConfig{.role = SpiRole::client,
                                                 .direction = SpiDirection::receive_only}));

// The errata's table (ES0206 2.12.4).
static_assert(spi_errata_apb_ceiling_hz(PinSpeed::very_high) == 84'000'000UL);
static_assert(spi_errata_apb_ceiling_hz(PinSpeed::low) == 25'000'000UL);
static_assert(spi_errata_apb_ceiling_hz(PinSpeed::very_high, true) == 42'000'000UL);

// 28.4.4's clock generator, against the chapter's own table 127. With the
// master clock out the divisor is 256 x (2 x I2SDIV + ODD) whatever the
// channel width, which is the MCK = 256 x FS rule.
static_assert(i2s_frame_factor(false, false) == 32u && i2s_frame_factor(true, false) == 64u);
static_assert(i2s_frame_factor(false, true) == 256u && i2s_frame_factor(true, true) == 256u);
static_assert(i2s_fs_hz(12'288'000UL, 12, false, false, false) == 16'000UL);
static_assert(i2s_fs_hz(1'536'000UL, 2, false, false, false) == 12'000UL);
static_assert(i2s_prescaler_for(12'288'000UL, 48'000UL, false, false)->div == 4u);
static_assert(!i2s_prescaler_for(12'288'000UL, 48'000UL, false, false)->odd);
// Table 127's 16-bit 48 kHz row: PLLI2S at 192/5 of a 1 MHz VCO input is
// 38.4 MHz, I2SDIV 12 with ODD - and the result is exact.
static_assert(i2s_fs_hz(38'400'000UL, 12, true, false, false) == 48'000UL);
// Out of range at both ends: a divisor below 4 and one above 511.
static_assert(!i2s_prescaler_for(1'000'000UL, 48'000UL, false, false).has_value());
static_assert(!i2s_prescaler_for(200'000'000UL, 8'000UL, false, false).has_value());
static_assert(i2s_config_valid(I2sConfig{}));
static_assert(!i2s_config_valid(I2sConfig{.div = 1}));
static_assert(!i2s_config_valid(I2sConfig{.pcm_long_frame = true}));
static_assert(i2s_config_valid(I2sConfig{.standard = I2sStandard::pcm, .pcm_long_frame = true}));
// A 24-bit datum forces a 32-bit channel whatever CHLEN says (28.5.8).
static_assert(i2s_channel_is_32(I2sConfig{.data = I2sDataLength::bits24}));
static_assert(!i2s_channel_is_32(I2sConfig{}));
static_assert((i2s_i2scfgr_of(I2sConfig{.mode = I2sMode::host_transmit}) &
               (SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_Msk)) ==
              (SPI_I2SCFGR_I2SMOD | (2u << SPI_I2SCFGR_I2SCFG_Pos)));
static_assert(i2s_i2spr_of(I2sConfig{.master_clock_out = true, .div = 3, .odd = true}) ==
              (SPI_I2SPR_MCKOE | SPI_I2SPR_ODD | 3u));

// The pads.
static_assert(!spi_pins_valid(SpiPins{}));
static_assert(!i2s_pins_valid(I2sPins{}));

} // namespace brio
