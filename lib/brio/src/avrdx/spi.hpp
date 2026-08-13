/*
 * spi.hpp
 *
 * SPI master transfer engine for the AVR Dx SPIn peripheral: the
 * target-side half of the SPI stack, driven by util/spi_ao.hpp (which
 * owns arbitration and replies). This engine owns the wire: chip
 * select, the D/C line of display-style devices, and the byte pump
 * under the SPI interrupt (no DMA on AVR Dx: one interrupt per byte is
 * the honest price).
 *
 * Transaction descriptor (Spi<n>::Request) - two phases in ONE chip
 * select window, covering every device on the bench:
 *
 *   phase 1 (optional): cmd[cmd_len] transmitted with DC LOW
 *   phase 2 (optional): len bytes with DC HIGH, FULL-DUPLEX -
 *                       transmit tx[] (or 0xFF dummies if tx == null),
 *                       capture into rx[] (or discard if rx == null)
 *
 *   - display (ILI9341 & co.): cmd + tx, DC toggling inside the CS
 *     window, rx null
 *   - rx-only ADC (MCP3550):    no cmd, tx null, rx set
 *   - loopback / generic xfer:  tx and rx both set (true full duplex)
 *   - SD card:                  sequences of plain xfers
 *
 * CS is ACTIVE LOW and asserted/released by the engine around the whole
 * transaction; dc may be a null PinRef for DC-less devices. Buffer
 * ownership travels with the request (client hands the spans off until
 * its SpiDone comes back). A request with zero total length is a client
 * bug: nothing is started and no completion will ever arrive.
 *
 * ISR wiring (app glue, as usual):
 *   ISR(SPI0_INT_vect) {
 *       if (SpiHw::isr()) { brio::post<SpiBus>(brio::TransferDone{brio::spi_ok}); }
 *   }
 */

#pragma once

#include <avr/io.h>
#include <stdint.h>

#include "avrdx/pin.hpp"
#include "kernel/post.hpp"
#include "util/spi_ao.hpp"

namespace brio {

/// SPI clock rate as a prescaler of CLK_PER (24 MHz here).
enum class SpiClock : uint8_t {
    div4 = SPI_PRESC_DIV4_gc,      // 6 MHz
    div16 = SPI_PRESC_DIV16_gc,    // 1.5 MHz
    div64 = SPI_PRESC_DIV64_gc,    // 375 kHz
    div128 = SPI_PRESC_DIV128_gc,  // 187.5 kHz
};

template <uint8_t spi_num = 0>
class Spi {
    static_assert(spi_num <= 1, "AVR Dx has SPI0/SPI1");

public:
    Spi() = delete;  // monostate

    struct Request {
        PinRef cs;             ///< asserted low around the transaction
        PinRef dc;             ///< display D/C line; null = no such pin
        const uint8_t* cmd;    ///< phase 1, sent with DC low
        uint8_t cmd_len;
        const uint8_t* tx;     ///< phase 2 out; null = 0xFF dummies
        uint8_t* rx;           ///< phase 2 in; null = discard
        uint16_t len;          ///< phase 2 length
        ReplyTo<SpiDone> reply;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    /**
     * Master, MSB first, default pins (SPI0: PA4 MOSI, PA5 MISO,
     * PA6 SCK; PA7 free - SSD disables the slave-select input). Call
     * after clock init, before sei(). CS/DC pins are configured by
     * their owners (the device clients), not here.
     */
    static void init(SpiClock clock = SpiClock::div16,
                     uint8_t mode = SPI_MODE_0_gc) {
        mode_ = mode;
        if constexpr (spi_num == 0) {
            PORTA.DIRSET = PIN4_bm | PIN6_bm;  // MOSI, SCK
            PORTA.DIRCLR = PIN5_bm;            // MISO
        } else {
            PORTC.DIRSET = PIN0_bm | PIN2_bm;  // MOSI, SCK
            PORTC.DIRCLR = PIN1_bm;            // MISO
        }
        regs().CTRLB = SPI_SSD_bm | mode_;
        regs().INTCTRL = SPI_IE_bm;
        regs().CTRLA = SPI_MASTER_bm | static_cast<uint8_t>(clock) |
                       SPI_ENABLE_bm;
    }

    /// Begin a transaction (called by SpiAo from main context). The
    /// completion arrives via the ISR returning true.
    static void start(const Request& r) {
        req_ = r;
        pos_ = 0;
        in_cmd_ = (r.cmd_len > 0);
        if (total_len() == 0) {
            return;  // client bug, documented: nothing will complete
        }
        if (in_cmd_) {
            r.dc.clear();
        } else {
            r.dc.set();
        }
        r.cs.clear();                      // assert, active low
        regs().DATA = first_byte();        // the ISR pumps the rest
    }

    /**
     * @brief SPI interrupt body - call from ISR(SPIn_INT_vect).
     * @return true when the transaction just completed (CS released):
     * the edge on which the glue posts TransferDone to the bus AO.
     */
    [[gnu::always_inline]] static bool isr() {
        (void)regs().INTFLAGS;             // IF clear sequence (with DATA)
        const uint8_t in = regs().DATA;

        if (!in_cmd_ && req_.rx != nullptr) {
            req_.rx[pos_] = in;
        }
        ++pos_;

        if (in_cmd_ && pos_ >= req_.cmd_len) {
            in_cmd_ = false;
            pos_ = 0;
            req_.dc.set();                 // command phase over
        }
        if (!in_cmd_ && pos_ >= req_.len) {
            req_.cs.set();                 // release: transaction done
            return true;
        }
        regs().DATA = next_byte();
        return false;
    }

private:
    static constexpr SPI_t& regs() {
        if constexpr (spi_num == 0) {
            return SPI0;
        } else {
            return SPI1;
        }
    }

    static uint16_t total_len() {
        return static_cast<uint16_t>(req_.cmd_len) + req_.len;
    }

    static uint8_t first_byte() { return in_cmd_ ? req_.cmd[0] : data_byte(0); }

    static uint8_t next_byte() {
        return in_cmd_ ? req_.cmd[pos_] : data_byte(pos_);
    }

    static uint8_t data_byte(uint16_t i) {
        return (req_.tx != nullptr) ? req_.tx[i] : 0xFF;
    }

    static inline Request req_{};
    static inline uint16_t pos_ = 0;
    static inline bool in_cmd_ = false;
    static inline uint8_t mode_ = SPI_MODE_0_gc;
};

} // namespace brio
