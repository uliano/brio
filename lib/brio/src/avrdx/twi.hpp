/*
 * twi.hpp
 *
 * I2C master transfer engine for the AVR Dx TWIn peripheral: the
 * target-side half of the I2C stack, driven by util/i2c_bus.hpp (the
 * BusMaster arbiter, which owns arbitration and replies). This engine
 * owns the wire: START/repeated START/STOP, the address phase, ACK
 * policy, and the byte pump under the TWI master interrupt (one
 * interrupt per byte, the same honest price the SPI engine pays).
 *
 * Transaction descriptor (Twi<n>::Request) - ONE bus tenure, from START
 * to STOP, in the three shapes I2C devices actually use:
 *
 *   write             tx[tx_len], rx_len == 0        S addr+W data... P
 *   read              tx_len == 0, rx[rx_len]        S addr+R data... P
 *   write-then-read   both                           S addr+W tx... Sr addr+R rx... P
 *   probe             both empty                     S addr+W P     -> ACK/NACK
 *
 * The write-then-read shape is the register-access idiom (send the
 * register index, repeated START, read the value) and it MUST be one
 * request: a repeated START is what keeps another client from slipping
 * in between - the SPI rule "the request is the complete script of one
 * bus tenure" holds verbatim. The probe is what an address scanner
 * sends: the reply's status says whether anybody ACKed.
 *
 * Status codes (util/i2c_bus.hpp): i2c_ok, i2c_nack_addr, i2c_nack_data,
 * i2c_arb_lost, i2c_bus_error. On any NACK the engine still issues the
 * STOP so the bus is released; on arbitration lost the bus belongs to
 * the other master and no STOP is sent; on bus error the peripheral is
 * forced back to the idle bus state.
 *
 * Buffer ownership travels with the request (client hands the spans
 * off until its I2cDone comes back). Bus speed travels per request, as
 * on SPI: a shared bus can carry a 100 kHz sensor and a 400 kHz DAC.
 *
 * NOT covered (noted, not built): a stuck-bus watchdog (a client that
 * holds SDA low forever leaves the transaction in flight - the kernel
 * keeps running, the bus AO stays busy; recovery by clocking SCL nine
 * times is the classic remedy, to be added when a real device makes it
 * necessary), 10-bit addressing, Fast-mode Plus (FMPEN), client mode.
 *
 * ISR wiring (app glue, as usual):
 *   ISR(TWI0_TWIM_vect) {
 *       if (TwiHw::isr()) { brio::post<I2c>(brio::TransferDone{TwiHw::status()}); }
 *   }
 */

#pragma once

#include <avr/io.h>
#include <stdint.h>

#include "kernel/post.hpp"
#include "util/i2c_bus.hpp"

namespace brio {

/// I2C bus speed class; MBAUD for each is derived from Clock::hz at
/// init() and kept in two bytes, selected per request.
enum class I2cSpeed : uint8_t {
    standard_100k,   ///< Standard-mode, 100 kHz, t_rise budget 1000 ns
    fast_400k,       ///< Fast-mode, 400 kHz, t_rise budget 300 ns
};

/// TWI pin routing (PORTMUX). Default: TWI0 SDA PA2 / SCL PA3,
/// TWI1 SDA PF2 / SCL PF3. alt2: TWI0 on PC2/PC3, TWI1 on PB2/PB3.
enum class TwiRoute : uint8_t { def = 0, alt2 = 2 };

template <uint8_t twi_num = 0, TwiRoute route = TwiRoute::def>
class Twi {
    static_assert(twi_num <= 1, "AVR Dx has TWI0/TWI1");

public:
    Twi() = delete;  // monostate

    struct Request {
        uint8_t addr;          ///< 7-bit client address (unshifted)
        const uint8_t* tx;     ///< bytes written after START (may be null if tx_len == 0)
        uint8_t tx_len;
        uint8_t* rx;           ///< bytes read after the (repeated) START+R
        uint8_t rx_len;
        ReplyTo<I2cDone> reply;
        I2cSpeed speed = I2cSpeed::standard_100k;
    };
    static_assert(std::is_trivially_copyable_v<Request>);

    /**
     * Master only, interrupt driven. Call after clock init, before
     * sei(). The pull-ups are external (the internal ones are far too
     * weak for I2C edges); the pins are left to the peripheral, which
     * drives them open-drain by itself.
     */
    template <typename Clock>
    static void init(Clock) {
        // Both MBAUD values fold at compile time from Clock::hz; the
        // per-request choice is a 1-of-2 lookup, no arithmetic on the fly.
        baud_[0] = baud_for<Clock::hz>(I2cSpeed::standard_100k);
        baud_[1] = baud_for<Clock::hz>(I2cSpeed::fast_400k);
        route_pins();
        regs().MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_ENABLE_bm;
        regs().MBAUD = baud_[0];
        regs().MSTATUS = TWI_BUSSTATE_IDLE_gc;   // the bus is ours to declare idle
    }

    /// Begin a transaction (called by the bus AO from main context).
    /// Always asynchronous: returns false and a TransferDone{status()}
    /// follows from the ISR glue - even the empty probe ends on the wire
    /// (its address phase IS the transaction).
    static bool start(const Request& r) {
        req_ = r;
        pos_ = 0;
        status_ = i2c_ok;
        regs().MBAUD = baud_[r.speed == I2cSpeed::fast_400k ? 1 : 0];
        if (r.tx_len == 0 && r.rx_len > 0) {
            phase_ = Phase::reading;
            regs().MADDR = static_cast<uint8_t>((r.addr << 1) | 1);
        } else {
            phase_ = Phase::writing;
            regs().MADDR = static_cast<uint8_t>(r.addr << 1);
        }
        return false;
    }

    /// Outcome of the last transaction (valid once isr() returned true).
    static uint8_t status() { return status_; }

    /**
     * @brief TWI master interrupt body - call from ISR(TWIn_TWIM_vect).
     * @return true when the transaction just completed (STOP issued or
     * bus lost): the edge on which the glue posts TransferDone.
     */
    [[gnu::always_inline]] static bool isr() {
        const uint8_t st = regs().MSTATUS;

        if (st & TWI_ARBLOST_bm) {          // another master won: not our bus
            regs().MSTATUS = TWI_ARBLOST_bm | TWI_WIF_bm;
            return finish(i2c_arb_lost);
        }
        if (st & TWI_BUSERR_bm) {           // protocol violation: force idle
            regs().MSTATUS = TWI_BUSERR_bm | TWI_WIF_bm | TWI_BUSSTATE_IDLE_gc;
            return finish(i2c_bus_error);
        }
        if (st & TWI_WIF_bm) {              // address or data byte went out
            if (st & TWI_RXACK_bm) {        // NACK: release the bus, report
                regs().MCTRLB = TWI_MCMD_STOP_gc;
                return finish((phase_ == Phase::writing && pos_ == 0)
                                  ? i2c_nack_addr : i2c_nack_data);
            }
            if (phase_ == Phase::writing && pos_ < req_.tx_len) {
                regs().MDATA = req_.tx[pos_++];
                return false;
            }
            if (req_.rx_len > 0) {          // repeated START, direction read
                phase_ = Phase::reading;
                pos_ = 0;
                regs().MADDR = static_cast<uint8_t>((req_.addr << 1) | 1);
                return false;
            }
            regs().MCTRLB = TWI_MCMD_STOP_gc;   // write / probe complete
            return finish(i2c_ok);
        }
        if (st & TWI_RIF_bm) {              // a data byte came in
            req_.rx[pos_++] = regs().MDATA;
            if (pos_ < req_.rx_len) {
                regs().MCTRLB = TWI_ACKACT_ACK_gc | TWI_MCMD_RECVTRANS_gc;
                return false;
            }
            regs().MCTRLB = TWI_ACKACT_NACK_gc | TWI_MCMD_STOP_gc;
            return finish(i2c_ok);
        }
        return false;                       // spurious: nothing to do
    }

private:
    enum class Phase : uint8_t { writing, reading };

    static constexpr TWI_t& regs() {
        if constexpr (twi_num == 0) {
            return TWI0;
        } else {
            return TWI1;
        }
    }

    static void route_pins() {
        constexpr uint8_t sel = static_cast<uint8_t>(route);
        if constexpr (twi_num == 0) {
            PORTMUX.TWIROUTEA = (PORTMUX.TWIROUTEA & ~PORTMUX_TWI0_gm) |
                                static_cast<uint8_t>(sel << PORTMUX_TWI0_gp);
        } else {
            PORTMUX.TWIROUTEA = (PORTMUX.TWIROUTEA & ~PORTMUX_TWI1_gm) |
                                static_cast<uint8_t>(sel << PORTMUX_TWI1_gp);
        }
    }

    /// f_SCL = f_CLK_PER / (10 + 2*BAUD + f_CLK_PER * t_RISE), solved for
    /// BAUD with the spec's worst-case rise time for the mode; real
    /// edges with stiff pull-ups are faster, which only slows SCL a
    /// touch below nominal - the safe side.
    template <uint32_t clk_hz>
    static constexpr uint8_t baud_for(I2cSpeed s) {
        const uint32_t f_scl = (s == I2cSpeed::fast_400k) ? 400000u : 100000u;
        const uint32_t t_rise_ns = (s == I2cSpeed::fast_400k) ? 300u : 1000u;
        const uint32_t rise_term = (clk_hz / 1000000u) * t_rise_ns / 2000u;
        const int32_t baud = static_cast<int32_t>(clk_hz / (2u * f_scl)) - 5 -
                             static_cast<int32_t>(rise_term);
        return static_cast<uint8_t>(baud < 1 ? 1 : (baud > 255 ? 255 : baud));
    }

    static inline uint8_t baud_[2] = {0, 0};   // MBAUD for standard / fast

    static bool finish(uint8_t code) {
        status_ = code;
        return true;
    }

    static inline Request req_{};
    static inline uint8_t pos_ = 0;
    static inline Phase phase_ = Phase::writing;
    static inline uint8_t status_ = i2c_ok;
};

} // namespace brio
