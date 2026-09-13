/*
 * usb.hpp
 *
 * The STM32F4's USB OTG controller IN DEVICE MODE (RM0383 ch. 22,
 * RM0390 ch. 31, RM0090 ch. 34 and 35), as the endpoint controller
 * util/usb/device.hpp's stack runs on: `UsbOtg<core>`, realizing the
 * `UsbController` contract at the packet, with `UsbFs` and `UsbHs` the
 * two cores this family may carry. brio's SECOND realization of that
 * contract, after the RP2040's own controller (docs/design/usb.md).
 *
 * WHAT THE BLOCK IS. A Synopsys DWC2 core with ST's glue: a full-speed
 * PHY on two pins with its DP pull-up, and - the whole of the interface
 * between the core and the CPU - a block of dedicated RAM organized as
 * FIFOs, reached through PUSH AND POP REGISTERS and never addressed
 * directly. That is the opposite economy from the RP2040's dual-port
 * RAM: there is no buffer to write a packet into, only a word-wide
 * window per FIFO, and the packet's LENGTH travels in a transfer-size
 * register beside it.
 *
 *   - ONE SHARED RECEIVE FIFO for every OUT endpoint. Each packet is
 *     stacked behind the last with a STATUS ENTRY on top of it; the
 *     application pops the status (GRXSTSP: which endpoint, how many
 *     bytes, what kind of entry) and then the bytes. A SETUP packet
 *     arrives as two entries - the eight bytes, then a "setup stage
 *     done" - and the endpoint's own SETUP interrupt follows the second.
 *   - ONE TRANSMIT FIFO PER IN ENDPOINT, each a slice of the same RAM
 *     the application maps by hand (a start address and a depth in
 *     32-bit words). This driver states the budget - 320 words on the FS
 *     core, 1024 on the HS one - and REFUSES a configuration that does
 *     not fit rather than overlapping two FIFOs, which the silicon would
 *     accept in silence.
 *   - A TRANSFER, not a packet, is what an endpoint register describes:
 *     a packet count and a byte count that the core walks down. This
 *     driver uses that on the OUT side and only there - see the next
 *     paragraph - and programs exactly one packet per transfer on every
 *     IN endpoint, which is what the contract above it promises.
 *
 * WHY THE OUT SIDE COUNTS PACKETS. A device that arms one OUT packet at
 * a time NAKs the host between packets, and a NAK behind a hub's
 * transaction translator costs a whole USB frame (measured on the
 * RP2040, docs/rp2040/usb.md, and the reason its bulk OUT endpoint is
 * double buffered). This core has no double buffer to offer, but it has
 * something better: an OUT transfer may be programmed for several
 * packets at once, and the core accepts them back to back into the
 * shared FIFO with no NAK in between. So `submit_out` COUNTS CREDITS -
 * the contract's one packet each - and the driver programs a transfer
 * of as many packets as the class has armed the moment the endpoint is
 * idle. `out_slots` says how many a class may hold armed, and the
 * receive FIFO is sized to hold exactly that many.
 *
 * ONE PACKET REPORTED PER PASS. The receive FIFO is one queue for every
 * endpoint, so a pass that pops two data packets for the same endpoint
 * could report only one of them (the contract's `out_data` is one packet
 * per endpoint). It therefore pops entries until the FIRST data packet,
 * hands that one over and LEAVES THE REST IN THE FIFO: RXFLVL is a level
 * - the FIFO is not empty - so the interrupt stands and the next pass
 * takes the next packet. Nothing is lost and nothing is copied twice.
 *
 * THE ADDRESS GOES IN BEFORE THE STATUS STAGE, and it is the one place
 * this core and chapter 9 disagree. The specification says the status of
 * SET_ADDRESS travels at the OLD address and the new one takes effect
 * after it, which is what util/usb/device.hpp implements and what a
 * controller whose address register bites at once needs; RM0383 22.17.5
 * puts the two steps the other way round, and MEASURED: with the address
 * written after the status stage the host is answered by nobody at it.
 * So this driver takes the address out of the SETUP packet as it passes
 * (`take_address_early`, the only chapter-9 knowledge in the file) and
 * the stack's own set_address, a status stage later, writes the same
 * value again.
 *
 * THE ERRATA that are live in device mode, and both are answered here:
 *  - ES0287 2.12.1 (ES0298 2.16.1, ES0206 2.14.1 and 2.15.1): a write
 *    sequence into a transmit FIFO that is INTERRUPTED by an access to
 *    an endpoint register corrupts the next data written. `submit_in`
 *    holds an InterruptGuard across the register writes and the push, so
 *    the sequence is uninterruptible whichever context called it.
 *  - ES0287 2.12.6 (ES0298 2.16.6, ES0206 2.14.6): DCFG's DAD and PFIVL
 *    read back wrong just after they are written. `address()` answers
 *    from the value this driver wrote and never reads the field.
 * The other four of that section are host-mode.
 *
 * THE CLOCK: the core wants 48 MHz +/- 0.25% on its own domain, which on
 * this family is the main PLL's Q output - so `init` static_asserts
 * `Clock::usb_hz == 48 MHz` and a program picks a SYSCLK whose ratio
 * yields it (96 or 168 MHz from a 25 or 8 MHz root; 180 MHz gives 45 and
 * is refused). The AHB must also run above 14.2 MHz (RM0383 22.3.3's
 * caution) and its rate chooses GUSBCFG's turnaround, table 132.
 *
 * WHAT IS NOT HERE: host mode and the OTG role switch (brio has no host
 * side, by decision - docs/design/usb.md), the ULPI transceiver of the
 * HS core (this driver drives that core's embedded FULL-SPEED PHY and
 * nothing else), the internal DMA (the HS core's; the FS core has none),
 * and isochronous endpoints, refused by `configure_endpoint` because the
 * even/odd frame the silicon wants per transfer has no place in a
 * contract drawn at the packet.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <span>

#include "stm32f4xx.h"

#include "stm32f4/device_tables.hpp"

#if defined(USB_OTG_FS_PERIPH_BASE) || defined(USB_OTG_HS_PERIPH_BASE)

#include "stm32f4/clock.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/pin.hpp"
#include "util/usb/device.hpp"

namespace brio {

/// The rate the core's own domain must run at (RM0383 22.3.2).
inline constexpr uint32_t otg_clock_hz = 48'000'000;

/// The AHB floor of RM0383 22.3.3's caution.
inline constexpr uint32_t otg_min_ahb_hz = 14'200'000;

/// GRXSTSP's PKTSTS codes in device mode (22.16.2).
struct OtgPacketStatus {
    static constexpr uint8_t global_out_nak = 0x1;
    static constexpr uint8_t out_data = 0x2;
    static constexpr uint8_t out_done = 0x3;
    static constexpr uint8_t setup_done = 0x4;
    static constexpr uint8_t setup_data = 0x6;
};

/**
 * One OTG core in device mode.
 *
 *   using Usb = brio::UsbFs;                       // UsbOtg<OtgCore::fs>
 *   Usb::init(clock);                              // false: a step did not answer
 *   Device::start();                               // the stack raises the pull-up
 *
 * Everything is static: a chip has one device stack, and the core is
 * the template argument rather than a number because the manuals name
 * these two instances and do not number them.
 */
template <OtgCore core>
struct UsbOtg {
    UsbOtg() = delete;

    static constexpr OtgFacts facts = otg_facts(core);
    static constexpr uint8_t endpoint_count = facts.endpoints;
    static constexpr uint16_t max_packet = usb_full_speed_packet;
    static constexpr IRQn_Type irq_line = otg_irq(core);

    /// How many OUT packets a class may hold armed at once. The core
    /// takes them as ONE transfer of that many packets and NAKs nobody
    /// in between; the receive FIFO below is sized for exactly this.
    static constexpr uint8_t out_slots = 8;

    // ---- the FIFO budget (22.13.1) ------------------------------------------------
    /// The core's whole dedicated RAM, in 32-bit words.
    static constexpr uint16_t fifo_words = facts.fifo_words;
    /// Endpoint zero's transmit FIFO: the register's own minimum, and one
    /// 64-byte packet exactly.
    static constexpr uint16_t ep0_tx_words = 16;
    /// The shared receive FIFO: the ten words 22.13.1 reserves for three
    /// back-to-back SETUP packets, one for the global OUT NAK pattern,
    /// one per OUT endpoint for its transfer-complete entry, and one
    /// packet's worth of data plus its status entry per armed slot.
    static constexpr uint16_t rx_words = static_cast<uint16_t>(
        10u + 1u + endpoint_count + out_slots * (max_packet / 4u + 1u));
    static_assert(rx_words >= 16u, "brio UsbOtg: GRXFSIZ's minimum is 16 words");
    static_assert(rx_words + ep0_tx_words <= fifo_words,
                  "brio UsbOtg: the receive FIFO and endpoint zero's transmit FIFO do not fit "
                  "in this core's dedicated RAM");

    /// The words one IN endpoint's transmit FIFO takes: its packet,
    /// rounded up to words, never below the register's minimum of 16.
    static constexpr uint16_t tx_words_for(uint16_t packet) {
        const uint16_t w = static_cast<uint16_t>((packet + 3u) / 4u);
        return w < 16u ? 16u : w;
    }

    // ---- the register blocks ------------------------------------------------------
    static USB_OTG_GlobalTypeDef& global() {
        return *reinterpret_cast<USB_OTG_GlobalTypeDef*>(static_cast<uintptr_t>(facts.base));
    }
    static USB_OTG_DeviceTypeDef& device() {
        return *reinterpret_cast<USB_OTG_DeviceTypeDef*>(
            static_cast<uintptr_t>(facts.base + USB_OTG_DEVICE_BASE));
    }
    static USB_OTG_INEndpointTypeDef& in_ep(uint8_t n) {
        return *reinterpret_cast<USB_OTG_INEndpointTypeDef*>(
            static_cast<uintptr_t>(facts.base + USB_OTG_IN_ENDPOINT_BASE + 0x20u * n));
    }
    static USB_OTG_OUTEndpointTypeDef& out_ep(uint8_t n) {
        return *reinterpret_cast<USB_OTG_OUTEndpointTypeDef*>(
            static_cast<uintptr_t>(facts.base + USB_OTG_OUT_ENDPOINT_BASE + 0x20u * n));
    }
    /// One FIFO's push/pop window. Writing FIFO n pushes into IN endpoint
    /// n's transmit FIFO; reading ANY of them pops the one shared receive
    /// FIFO, and this driver always reads window zero.
    static volatile uint32_t& fifo(uint8_t n) {
        return *reinterpret_cast<volatile uint32_t*>(
            static_cast<uintptr_t>(facts.base + USB_OTG_FIFO_BASE + 0x1000u * n));
    }
    static volatile uint32_t& pcgcctl() {
        return *reinterpret_cast<volatile uint32_t*>(
            static_cast<uintptr_t>(facts.base + USB_OTG_PCGCCTL_BASE));
    }

    /**
     * The core brought up as a full-speed device: its bus clock opened,
     * the two data pads handed to the PHY, the core soft-reset, the
     * transceiver powered and VBUS either sensed or given away, device
     * mode forced, the FIFO map written, the interrupts unmasked at the
     * block and at the NVIC. The pull-up stays DOWN - the stack raises
     * it with connect(true) when it is ready to answer.
     *
     * `sense_vbus` false (the default) frees the VBUS pad, which is what
     * a bus-powered device with no VBUS wire wants and what leaves that
     * pad to another peripheral; the core then takes VBUS as valid at
     * all times, so a cable pulled out looks like a bus that went quiet.
     *
     * False when the core did not answer: a soft reset that never
     * finished, a FIFO flush that stood.
     */
    template <typename Clock>
    static bool init(Clock, bool sense_vbus = false) {
        static_assert(facts.present,
                      "brio UsbOtg: this part has no such OTG core (the device header declares "
                      "no base address for it)");
        static_assert(Clock::usb_hz == otg_clock_hz,
                      "brio UsbOtg: the controller needs 48 MHz exactly on its own domain, and on "
                      "this family that is the main PLL's Q output - pick a SYSCLK whose ratio "
                      "yields it (Clock::usb_hz says what a rate gives; 180 MHz gives 45 MHz and "
                      "is not a USB clock)");
        static_assert(Clock::hz >= otg_min_ahb_hz,
                      "brio UsbOtg: RM0383 22.3.3 - the AHB must run above 14.2 MHz for this "
                      "controller to work");
        constexpr uint8_t trdt = otg_turnaround_for(Clock::hz);
        static_assert(trdt != 0u, "brio UsbOtg: no turnaround value for this AHB rate (table 132)");

        open_gate();
        claim_pads();

        global().GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;   // the line masked while the core is rebuilt

        // The HS core's PHY select must be written before the reset; the
        // FS core's is a read-only one.
        if constexpr (facts.phy_select_writable) {
            global().GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
        }
        if (!core_reset()) {
            return false;
        }

        // The transceiver, and what the core is to believe about VBUS.
        // GCCFG's low half is reserved with an UNDEFINED reset value
        // (22.16.2 states the reset value as 0x0000 XXXX), so it is read
        // and put back rather than written.
        global().GCCFG = (global().GCCFG & 0xFFFFu) | otg_gccfg_device(sense_vbus);
        if (!sense_vbus && otg_has_session_override()) {
            global().GOTGCTL |= otg_session_override_bits();
        }

        // Device mode, forced: the ID pin is then ignored and its pad
        // free. 22.16.2 asks for 25 ms before the change takes effect.
        global().GUSBCFG = (global().GUSBCFG & ~(USB_OTG_GUSBCFG_TRDT | USB_OTG_GUSBCFG_FHMOD |
                                                 USB_OTG_GUSBCFG_TOCAL)) |
                           USB_OTG_GUSBCFG_FDMOD |
                           (static_cast<uint32_t>(trdt) << USB_OTG_GUSBCFG_TRDT_Pos);
        spin_at_least(Clock::hz / 40u);   // 25 ms of core cycles

        return device_init();
    }

    /// The core back into reset and its bus clock closed; the pads are
    /// left where init() put them.
    static void release() {
        Nvic::disable(irq_line);
        connect(false);
        global().GINTMSK = 0;
        global().GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
        if constexpr (facts.gate_on_ahb1) {
            Rcc::ahb1_reset(facts.reset_mask);
            Rcc::ahb1_clock(facts.clock_mask, false);
        } else {
            Rcc::ahb2_reset(facts.reset_mask);
            Rcc::ahb2_clock(facts.clock_mask, false);
        }
    }

    // ---- the UsbController contract -----------------------------------------------

    /// The DP pull-up, through the soft-disconnect bit: set, the host
    /// sees no device at all (22.5.2, and table 133's 2.5 us minimum for
    /// the host to notice).
    static void connect(bool on) {
        if (on) {
            device().DCTL &= ~USB_OTG_DCTL_SDIS;
        } else {
            device().DCTL |= USB_OTG_DCTL_SDIS;
        }
    }

    /// The device address, effective at once. The stack calls this after
    /// the status stage of SET_ADDRESS, as chapter 9 wants; this core
    /// wants it BEFORE, so `take_address_early` below has already
    /// written the same value and this write changes nothing.
    static void set_address(uint8_t address) {
        device().DCFG = (device().DCFG & ~USB_OTG_DCFG_DAD) |
                        ((static_cast<uint32_t>(address) & 0x7Fu) << USB_OTG_DCFG_DAD_Pos);
        address_ = address;   // ES0287 2.12.6: the field does not read back
    }

    /// One direction of one endpoint claimed. An IN endpoint takes a
    /// slice of the FIFO RAM here, numbered as the endpoint is, and the
    /// claim is REFUSED when the budget is spent - the alternative would
    /// be two FIFOs at one address, which the silicon takes in silence.
    static bool configure_endpoint(uint8_t address, UsbEndpointType type, uint16_t max) {
        const uint8_t n = usb_ep_number(address);
        const bool in = usb_ep_is_in(address);
        if (n == 0u || n >= endpoint_count || max == 0u || max > max_packet ||
            type == UsbEndpointType::control || type == UsbEndpointType::isochronous) {
            return false;
        }
        const uint32_t kind = static_cast<uint32_t>(type) << USB_OTG_DIEPCTL_EPTYP_Pos;
        if (in) {
            const uint16_t words = tx_words_for(max);
            if (static_cast<uint32_t>(next_fifo_) + words > fifo_words) {
                return false;
            }
            global().DIEPTXF[n - 1u] = (static_cast<uint32_t>(words) << 16) | next_fifo_;
            next_fifo_ = static_cast<uint16_t>(next_fifo_ + words);
            (void)flush_tx(n);
            in_ep(n).DIEPCTL = static_cast<uint32_t>(max) | kind |
                               (static_cast<uint32_t>(n) << USB_OTG_DIEPCTL_TXFNUM_Pos) |
                               USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_SD0PID_SEVNFRM |
                               USB_OTG_DIEPCTL_SNAK;
            device().DAINTMSK |= 1UL << n;
            in_mps_[n] = max;
            in_pending_[n] = 0;
            in_active_[n] = true;
        } else {
            out_ep(n).DOEPCTL = static_cast<uint32_t>(max) | kind | USB_OTG_DOEPCTL_USBAEP |
                                USB_OTG_DOEPCTL_SD0PID_SEVNFRM | USB_OTG_DOEPCTL_SNAK;
            device().DAINTMSK |= 1UL << (16u + n);
            out_mps_[n] = max;
            out_credits_[n] = 0;
            out_armed_[n] = 0;
            out_length_[n] = 0;
            out_active_[n] = true;
        }
        return true;
    }

    /// Every endpoint but zero released, and the FIFO map back to what
    /// init() left: the receive FIFO and endpoint zero's, nothing else.
    static void deconfigure_endpoints() {
        for (uint8_t n = 1; n < endpoint_count; ++n) {
            disable_in(n);
            disable_out(n);
            device().DAINTMSK &= ~((1UL << n) | (1UL << (16u + n)));
            device().DIEPEMPMSK &= ~(1UL << n);
            in_active_[n] = false;
            out_active_[n] = false;
            in_pending_[n] = 0;
            out_credits_[n] = 0;
            out_armed_[n] = 0;
            out_length_[n] = 0;
        }
        next_fifo_ = static_cast<uint16_t>(rx_words + ep0_tx_words);
        (void)flush_tx(flush_all_fifos);
    }

    /// The halt of one direction. Endpoint zero's is cleared by the
    /// silicon when the next SETUP token arrives, as chapter 9 wants -
    /// so the setup reception is re-armed here, the stall having
    /// disabled the endpoint.
    static void stall(uint8_t address, bool on) {
        const uint8_t n = usb_ep_number(address);
        if (n >= endpoint_count) {
            return;
        }
        if (usb_ep_is_in(address)) {
            const uint32_t was = in_ep(n).DIEPCTL;
            if (on) {
                // 22.17.6: an enabled IN endpoint is disabled in the same
                // write that stalls it, the STALL bit taking priority.
                const bool running = n != 0u && (was & USB_OTG_DIEPCTL_EPENA) != 0u;
                in_ep(n).DIEPCTL =
                    was | USB_OTG_DIEPCTL_STALL | (running ? USB_OTG_DIEPCTL_EPDIS : 0u);
                stalls_ = stalls_ + 1u;
            } else {
                // Cleared with the data toggle back to DATA0, which is what
                // CLEAR_FEATURE(ENDPOINT_HALT) promises the host.
                in_ep(n).DIEPCTL = (was & ~USB_OTG_DIEPCTL_STALL) |
                                   (n != 0u ? USB_OTG_DIEPCTL_SD0PID_SEVNFRM : 0u);
            }
        } else {
            const uint32_t was = out_ep(n).DOEPCTL;
            if (on) {
                out_ep(n).DOEPCTL = was | USB_OTG_DOEPCTL_STALL;
                stalls_ = stalls_ + 1u;
            } else {
                out_ep(n).DOEPCTL = (was & ~USB_OTG_DOEPCTL_STALL) |
                                    (n != 0u ? USB_OTG_DOEPCTL_SD0PID_SEVNFRM : 0u);
            }
        }
        if (n == 0u && on) {
            arm_setup();
        }
    }

    static bool stalled(uint8_t address) {
        const uint8_t n = usb_ep_number(address);
        if (n >= endpoint_count) {
            return false;
        }
        return usb_ep_is_in(address)
                   ? (in_ep(n).DIEPCTL & USB_OTG_DIEPCTL_STALL) != 0u
                   : (out_ep(n).DOEPCTL & USB_OTG_DOEPCTL_STALL) != 0u;
    }

    /// ONE packet offered on an IN endpoint: the transfer programmed for
    /// exactly one packet, the endpoint enabled, and the bytes pushed
    /// into its own FIFO. When the FIFO has no room the bytes are held
    /// here and pushed by the empty-level interrupt instead - which is
    /// why the contract's "copied before this returns" holds either way.
    ///
    /// The whole sequence runs with interrupts masked: ES0287 2.12.1
    /// corrupts a FIFO write that an endpoint-register access interrupts.
    static bool submit_in(uint8_t number, std::span<const uint8_t> data) {
        if (number >= endpoint_count || data.size() > in_mps_[number] ||
            (number != 0u && !in_active_[number])) {
            return false;
        }
        InterruptGuard guard;
        const uint32_t size = static_cast<uint32_t>(data.size());
        in_ep(number).DIEPTSIZ = (1UL << USB_OTG_DIEPTSIZ_PKTCNT_Pos) | size;
        in_ep(number).DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
        if (size == 0u) {
            return true;   // a zero-length packet needs no FIFO write at all
        }
        const uint32_t words = (size + 3u) / 4u;
        if ((in_ep(number).DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) >= words) {
            push(number, data.data(), size);
        } else {
            memcpy(in_buf_[number], data.data(), size);
            in_pending_[number] = static_cast<uint16_t>(size);
            device().DIEPEMPMSK |= 1UL << number;
            fifo_waits_ = fifo_waits_ + 1u;
        }
        return true;
    }

    /// ONE packet's reception armed. The credit is what the transfer the
    /// core walks is built from: the endpoint is programmed for as many
    /// packets as stand armed the moment it is idle, so a run of them
    /// crosses with no NAK in between (the file header).
    static bool submit_out(uint8_t number, uint16_t max) {
        if (number >= endpoint_count || max > max_packet ||
            (number != 0u && !out_active_[number])) {
            return false;
        }
        if (number == 0u) {
            // Endpoint zero's transfer-size register has one packet-count
            // bit and a seven-bit byte count, so one packet is all it can
            // ever be - and STUPCNT is re-stated so a SETUP token is
            // still answered while a data stage is armed.
            out_ep(0).DOEPTSIZ = (3UL << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
                                 (1UL << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | max;
            out_ep(0).DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
            return true;
        }
        if (out_credits_[number] >= out_slots) {
            return false;
        }
        out_credits_[number] = static_cast<uint8_t>(out_credits_[number] + 1u);
        rearm_out(number);
        return true;
    }

    /// The last packet that came on the endpoint - copied out of the
    /// shared FIFO when it was popped, so it stands until the next one.
    static std::span<const uint8_t> out_data(uint8_t number) {
        if (number >= endpoint_count) {
            return {};
        }
        return {out_buf_[number], out_length_[number]};
    }

    static UsbSetup setup() {
        return UsbSetup{.request_type = static_cast<uint8_t>(setup_words_[0]),
                        .request = static_cast<uint8_t>(setup_words_[0] >> 8),
                        .value = static_cast<uint16_t>(setup_words_[0] >> 16),
                        .index = static_cast<uint16_t>(setup_words_[1]),
                        .length = static_cast<uint16_t>(setup_words_[1] >> 16)};
    }

    /// THE ISR BODY the controller's vector reaches through the stack:
    /// GINTSTS read once, the bus events acknowledged, the receive FIFO
    /// drained to its first data packet, and the endpoint interrupts
    /// scanned through DAINT.
    [[gnu::always_inline]] static UsbEvents take_events() {
        UsbEvents ev{};
        const uint32_t sts = global().GINTSTS & global().GINTMSK;
        if ((sts & USB_OTG_GINTSTS_USBRST) != 0u) {
            global().GINTSTS = USB_OTG_GINTSTS_USBRST;
            on_bus_reset();
            ev.reset = true;
        }
        if ((sts & USB_OTG_GINTSTS_ENUMDNE) != 0u) {
            global().GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
            on_enumeration_done();
        }
        if ((sts & USB_OTG_GINTSTS_ESUSP) != 0u) {
            global().GINTSTS = USB_OTG_GINTSTS_ESUSP;
            early_suspends_ = early_suspends_ + 1u;
        }
        if ((sts & USB_OTG_GINTSTS_USBSUSP) != 0u) {
            global().GINTSTS = USB_OTG_GINTSTS_USBSUSP;
            ev.suspend = true;
        }
        if ((sts & USB_OTG_GINTSTS_WKUINT) != 0u) {
            global().GINTSTS = USB_OTG_GINTSTS_WKUINT;
            ev.resume = true;
        }
        if ((sts & USB_OTG_GINTSTS_SRQINT) != 0u) {
            global().GINTSTS = USB_OTG_GINTSTS_SRQINT;
            sessions_ = sessions_ + 1u;
        }
        if ((sts & USB_OTG_GINTSTS_OTGINT) != 0u) {
            global().GOTGINT = global().GOTGINT;   // rc_w1, every flag at once
            otg_events_ = otg_events_ + 1u;
        }
        if ((sts & USB_OTG_GINTSTS_MMIS) != 0u) {
            global().GINTSTS = USB_OTG_GINTSTS_MMIS;
            mode_mismatches_ = mode_mismatches_ + 1u;
        }
        if ((global().GINTSTS & USB_OTG_GINTSTS_RXFLVL) != 0u) {
            drain_receive_fifo(ev);
        }
        if ((global().GINTSTS & (USB_OTG_GINTSTS_IEPINT | USB_OTG_GINTSTS_OEPINT)) != 0u) {
            serve_endpoints(ev);
        }
        return ev;
    }

    // ---- readbacks -----------------------------------------------------------------
    /// The address this driver last wrote - never the field, which does
    /// not read back (ES0287 2.12.6).
    static uint8_t address() { return address_; }
    static bool pulled_up() { return (device().DCTL & USB_OTG_DCTL_SDIS) == 0u; }
    static bool suspended() { return (device().DSTS & USB_OTG_DSTS_SUSPSTS) != 0u; }
    /// The core's erratic-error flag: what turns an early suspend into a
    /// condition only a soft disconnect recovers from (22.16.4).
    static bool erratic_error() { return (device().DSTS & USB_OTG_DSTS_EERR) != 0u; }
    /// DSTS.ENUMSPD, 3 for the full speed this core comes up at.
    static uint8_t enumerated_speed() {
        return static_cast<uint8_t>((device().DSTS & USB_OTG_DSTS_ENUMSPD) >>
                                    USB_OTG_DSTS_ENUMSPD_Pos);
    }
    /// The frame number of the last start-of-frame the host sent - one a
    /// millisecond while the bus is awake.
    static uint16_t frame() {
        return static_cast<uint16_t>((device().DSTS & USB_OTG_DSTS_FNSOF) >>
                                     USB_OTG_DSTS_FNSOF_Pos);
    }
    /// Whether the core believes a B-session is valid - which with the
    /// VBUS pad given away is a forced yes, not a measurement.
    static bool session_valid() {
        return otg_session_valid_bit() != 0u &&
               (global().GOTGCTL & otg_session_valid_bit()) != 0u;
    }
    /// GINTSTS.CMOD: 0 in device mode, and a mode mismatch is what an
    /// access to the other mode's registers would raise.
    static bool in_device_mode() { return (global().GINTSTS & USB_OTG_GINTSTS_CMOD) == 0u; }
    /// The core's product identifier, a plain readback that proves the
    /// block is clocked and out of reset.
    static uint32_t core_id() { return global().CID; }
    /// How much of the FIFO RAM is spoken for: the receive FIFO,
    /// endpoint zero's and every claim configure_endpoint has served.
    static uint16_t fifo_words_used() { return next_fifo_; }
    static uint32_t rx_entries() { return rx_entries_; }
    static uint32_t fifo_waits() { return fifo_waits_; }
    static uint32_t stalls() { return stalls_; }
    static uint32_t timeouts() { return timeouts_; }
    static uint32_t setup_overruns() { return setup_overruns_; }
    static uint32_t early_suspends() { return early_suspends_; }
    static uint32_t mode_mismatches() { return mode_mismatches_; }
    static uint32_t sessions() { return sessions_; }
    static uint32_t otg_events() { return otg_events_; }
    /// How many times the core has reported the end of a bus reset -
    /// once per enumeration the host drives.
    static uint32_t enumerations() { return enumerations_; }
    /// How many packets an OUT endpoint's current transfer still holds
    /// room for - the credit the host may spend before the next NAK.
    static uint8_t out_armed(uint8_t number) {
        return number < endpoint_count ? out_armed_[number] : 0u;
    }

    // ---- the low-power bits of 22.8 -------------------------------------------------
    /// The PHY clock stopped and HCLK gated inside the core: legal only
    /// while the bus is suspended or no session is valid, and undone by
    /// the resume the core detects on its own.
    static void gate_clocks(bool on) {
        pcgcctl() = on ? (USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK) : 0u;
    }
    static bool phy_suspended() { return (pcgcctl() & USB_OTG_PCGCCTL_PHYSUSP) != 0u; }
    /// Remote wake-up signalling, raised here and lowered by the caller
    /// 1 to 15 ms later (22.5.2); the host is not told the device may do
    /// this unless the configuration descriptor says so.
    static void remote_wakeup(bool on) {
        if (on) {
            device().DCTL |= USB_OTG_DCTL_RWUSIG;
        } else {
            device().DCTL &= ~USB_OTG_DCTL_RWUSIG;
        }
    }

private:
    /// GRSTCTL.TXFNUM's "every transmit FIFO" code (22.16.2).
    static constexpr uint8_t flush_all_fifos = 0x10;
    /// A bounded spin on a core flag: at the slowest legal AHB rate the
    /// longest of these (a FIFO flush, eight clocks) is microseconds, so
    /// this bound is a hardware fault and not a slow answer.
    static constexpr uint32_t flag_spins = 200'000u;
    /// Every write-one-to-clear flag of an IN and of an OUT endpoint,
    /// named one by one rather than as a span of bits: the reserved
    /// positions between them are to be left at their reset value.
    static constexpr uint32_t in_endpoint_flags =
        USB_OTG_DIEPINT_XFRC | USB_OTG_DIEPINT_EPDISD | USB_OTG_DIEPINT_AHBERR |
        USB_OTG_DIEPINT_TOC | USB_OTG_DIEPINT_ITTXFE | USB_OTG_DIEPINT_INEPNM |
        USB_OTG_DIEPINT_INEPNE | USB_OTG_DIEPINT_TXFIFOUDRN | USB_OTG_DIEPINT_BNA |
        USB_OTG_DIEPINT_BERR | USB_OTG_DIEPINT_NAK;
    static constexpr uint32_t out_endpoint_flags =
        USB_OTG_DOEPINT_XFRC | USB_OTG_DOEPINT_EPDISD | USB_OTG_DOEPINT_AHBERR |
        USB_OTG_DOEPINT_STUP | USB_OTG_DOEPINT_OTEPDIS | USB_OTG_DOEPINT_OTEPSPR |
        USB_OTG_DOEPINT_B2BSTUP | USB_OTG_DOEPINT_OUTPKTERR | USB_OTG_DOEPINT_NAK;

    static void open_gate() {
        if constexpr (facts.gate_on_ahb1) {
            Rcc::ahb1_clock(facts.clock_mask, true);
            // The embedded full-speed PHY is not the ULPI one: its clock
            // stays closed, and a gate left open there is current spent
            // on a transceiver nothing drives.
            Rcc::ahb1_clock(facts.ulpi_clock_mask, false);
        } else {
            Rcc::ahb2_clock(facts.clock_mask, true);
        }
    }

    static void claim_pads() {
        constexpr PinConfig pads{.pull = PinPull::none, .open_drain = false,
                                 .speed = PinSpeed::very_high};
        Pin<facts.dm_port, facts.dm_pin>::function(
            static_cast<PinFunction>(facts.pad_function), pads);
        Pin<facts.dp_port, facts.dp_pin>::function(
            static_cast<PinFunction>(facts.pad_function), pads);
    }

    static bool wait_ahb_idle() {
        for (uint32_t spins = 0; spins < flag_spins; ++spins) {
            if ((global().GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) != 0u) {
                return true;
            }
        }
        return false;
    }

    /// The core's own soft reset: every state machine and every FIFO
    /// pointer back to the reset state, the CSR bits the AHB domain owns
    /// cleared. The bit clears itself when it is done.
    static bool core_reset() {
        if (!wait_ahb_idle()) {
            return false;
        }
        global().GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
        for (uint32_t spins = 0; spins < flag_spins; ++spins) {
            if ((global().GRSTCTL & USB_OTG_GRSTCTL_CSRST) == 0u) {
                return wait_ahb_idle();
            }
        }
        return false;
    }

    static bool flush_tx(uint8_t fifo_number) {
        if (!wait_ahb_idle()) {
            return false;
        }
        global().GRSTCTL = (static_cast<uint32_t>(fifo_number) << USB_OTG_GRSTCTL_TXFNUM_Pos) |
                           USB_OTG_GRSTCTL_TXFFLSH;
        for (uint32_t spins = 0; spins < flag_spins; ++spins) {
            if ((global().GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == 0u) {
                return true;
            }
        }
        return false;
    }

    static bool flush_rx() {
        if (!wait_ahb_idle()) {
            return false;
        }
        global().GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
        for (uint32_t spins = 0; spins < flag_spins; ++spins) {
            if ((global().GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == 0u) {
                return true;
            }
        }
        return false;
    }

    /// At least `cycles` core cycles with no timebase of any kind: the
    /// body is eight NOPs, so one iteration is at least eight cycles and
    /// the count is a floor. Used once, for the 25 ms the chapter asks
    /// for after the mode is forced.
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

    /// Everything below the mode: the speed, the FIFO map, the endpoint
    /// registers at rest, the interrupts.
    static bool device_init() {
        // The PHY clock started, whatever a previous program's power
        // saving left in it (22.16.5), and the transmit FIFO map wiped:
        // every DIEPTXFx comes out of reset pointing at a slice of the
        // RAM this driver's own map does not use, and a stale entry is a
        // second FIFO at an address the receive one already owns.
        pcgcctl() = 0;
        for (uint8_t f = 0; f < 15u; ++f) {
            global().DIEPTXF[f] = 0;
        }
        device().DCFG = 3u;   // full speed, address 0, no non-zero-length status handshake
        device().DCTL |= USB_OTG_DCTL_SDIS;
        address_ = 0;

        if (!flush_tx(flush_all_fifos) || !flush_rx()) {
            return false;
        }
        for (uint8_t n = 0; n < endpoint_count; ++n) {
            in_ep(n).DIEPCTL = 0;
            in_ep(n).DIEPTSIZ = 0;
            in_ep(n).DIEPINT = in_endpoint_flags;
            out_ep(n).DOEPCTL = 0;
            out_ep(n).DOEPTSIZ = 0;
            out_ep(n).DOEPINT = out_endpoint_flags;
            in_active_[n] = false;
            out_active_[n] = false;
            in_pending_[n] = 0;
            out_credits_[n] = 0;
            out_armed_[n] = 0;
            out_length_[n] = 0;
        }
        in_mps_[0] = max_packet;
        out_mps_[0] = max_packet;
        device().DIEPMSK = 0;
        device().DOEPMSK = 0;
        device().DAINTMSK = 0;
        device().DIEPEMPMSK = 0;

        global().GRXFSIZ = rx_words;
        global().DIEPTXF0_HNPTXFSIZ = (static_cast<uint32_t>(ep0_tx_words) << 16) | rx_words;
        next_fifo_ = static_cast<uint16_t>(rx_words + ep0_tx_words);

        global().GINTSTS = 0xBFFF'FFFFu;   // every write-one flag, the reserved one left alone
        global().GINTMSK = USB_OTG_GINTMSK_USBRST | USB_OTG_GINTMSK_ENUMDNEM |
                           USB_OTG_GINTMSK_ESUSPM | USB_OTG_GINTMSK_USBSUSPM |
                           USB_OTG_GINTMSK_WUIM | USB_OTG_GINTMSK_RXFLVLM |
                           USB_OTG_GINTMSK_IEPINT | USB_OTG_GINTMSK_OEPINT |
                           USB_OTG_GINTMSK_SRQIM | USB_OTG_GINTMSK_OTGINT |
                           USB_OTG_GINTMSK_MMISM;
        // TXFELVL: the empty-level interrupt means COMPLETELY empty, so a
        // whole packet is known to fit when it fires.
        global().GAHBCFG = USB_OTG_GAHBCFG_TXFELVL | USB_OTG_GAHBCFG_GINT;
        Nvic::enable(irq_line);
        return true;
    }

    /// The reset the host drives, answered as 22.17.5's "endpoint
    /// initialization on USB reset" asks: every OUT endpoint NAKing, the
    /// two endpoint-zero interrupts unmasked, the transmit FIFOs flushed
    /// and the setup reception armed. The stack's own on_reset follows
    /// with deconfigure_endpoints() and address zero.
    static void on_bus_reset() {
        device().DCTL &= ~USB_OTG_DCTL_RWUSIG;
        (void)flush_tx(flush_all_fifos);
        for (uint8_t n = 0; n < endpoint_count; ++n) {
            in_ep(n).DIEPINT = in_endpoint_flags;
            out_ep(n).DOEPINT = out_endpoint_flags;
            if (n != 0u) {
                out_ep(n).DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
            }
            in_pending_[n] = 0;
            out_credits_[n] = 0;
            out_armed_[n] = 0;
            out_length_[n] = 0;
        }
        device().DIEPEMPMSK = 0;
        device().DAINTMSK = (1UL << 0) | (1UL << 16);
        device().DOEPMSK = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM |
                           USB_OTG_DOEPMSK_OTEPDM | USB_OTG_DOEPMSK_EPDM;
        device().DIEPMSK = USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_TOM | USB_OTG_DIEPMSK_EPDM;
        device().DCFG &= ~USB_OTG_DCFG_DAD;
        address_ = 0;
        arm_setup();
    }

    /// The end of the reset: the speed is known and endpoint zero takes
    /// its maximum packet size (22.17.5, "on enumeration completion").
    static void on_enumeration_done() {
        in_ep(0).DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;   // code 0 = 64 bytes
        in_mps_[0] = max_packet;
        out_mps_[0] = max_packet;
        arm_setup();
        enumerations_ = enumerations_ + 1u;
    }

    /**
     * THE ONE PLACE THIS DRIVER READS A SETUP PACKET, and the reason is
     * the silicon's. RM0383 22.17.5, "endpoint initialization on
     * SetAddress command", puts the two steps in this order: program
     * DCFG with the address, THEN send the status IN. Chapter 9 of the
     * specification puts them the other way round - the status travels
     * at the OLD address - and that is the order util/usb/device.hpp
     * implements, because it is what a controller whose address register
     * takes effect at once needs.
     *
     * MEASURED HERE: with the address written only after the status
     * stage, the host takes the status, sends its next request at the
     * new address, and is answered by nobody - seven bus resets and
     * "device descriptor read/8, error -32" from the kernel, every time.
     * With DCFG written the moment the SETUP packet is popped, the same
     * host enumerates the device in some six hundred milliseconds.
     *
     * So the driver takes the address out of the packet as it passes -
     * a standard OUT request to the device, code 5 - and writes it at
     * once. The stack's own set_address, one status stage later, writes
     * the same value into the same field, which changes nothing.
     */
    static void take_address_early() {
        // bmRequestType (host-to-device, standard, to the device) in the
        // low byte, bRequest in the next; wValue is the address.
        if ((setup_words_[0] & 0xFFFFu) != 0x0500u) {
            return;
        }
        const uint16_t wanted = static_cast<uint16_t>(setup_words_[0] >> 16);
        if (wanted <= 127u) {
            set_address(static_cast<uint8_t>(wanted));
        }
    }

    /// Endpoint zero armed for up to three back-to-back SETUP packets.
    /// The endpoint need not be ENABLED for this: with STUPCNT non-zero
    /// the core takes SETUP packets whatever the NAK and enable bits say
    /// (22.17.6).
    static void arm_setup() {
        out_ep(0).DOEPTSIZ = (3UL << USB_OTG_DOEPTSIZ_STUPCNT_Pos) |
                             (1UL << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | 24u;
    }

    /// The transfer an idle OUT endpoint is given: as many packets as
    /// stand armed. Nothing to do while one is running - the core is
    /// already taking packets, and the transfer-size register may not be
    /// touched while EPENA stands.
    static void rearm_out(uint8_t n) {
        if (n == 0u || out_credits_[n] == 0u ||
            (out_ep(n).DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0u) {
            return;
        }
        const uint32_t packets = out_credits_[n];
        out_ep(n).DOEPTSIZ = (packets << USB_OTG_DOEPTSIZ_PKTCNT_Pos) | (packets * out_mps_[n]);
        out_ep(n).DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
        out_armed_[n] = static_cast<uint8_t>(packets);
    }

    static void disable_in(uint8_t n) {
        volatile uint32_t& ctl = in_ep(n).DIEPCTL;
        if ((ctl & USB_OTG_DIEPCTL_EPENA) != 0u) {
            ctl |= USB_OTG_DIEPCTL_SNAK | USB_OTG_DIEPCTL_EPDIS;
            for (uint32_t spins = 0; spins < flag_spins; ++spins) {
                if ((ctl & USB_OTG_DIEPCTL_EPENA) == 0u) {
                    break;
                }
            }
        }
        in_ep(n).DIEPINT = in_endpoint_flags;
        ctl = 0;
    }

    static void disable_out(uint8_t n) {
        volatile uint32_t& ctl = out_ep(n).DOEPCTL;
        if ((ctl & USB_OTG_DOEPCTL_EPENA) != 0u) {
            ctl |= USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS;
            for (uint32_t spins = 0; spins < flag_spins; ++spins) {
                if ((ctl & USB_OTG_DOEPCTL_EPENA) == 0u) {
                    break;
                }
            }
        }
        out_ep(n).DOEPINT = out_endpoint_flags;
        ctl = 0;
    }

    /// One word into an IN endpoint's FIFO at a time, the tail padded.
    /// The source may be unaligned (a span into a descriptor), so every
    /// word is assembled by memcpy and not read as one.
    static void push(uint8_t n, const uint8_t* p, uint32_t len) {
        volatile uint32_t& window = fifo(n);
        for (uint32_t at = 0; at < len; at += 4u) {
            const uint32_t left = len - at;
            uint32_t word = 0;
            memcpy(&word, p + at, left < 4u ? left : 4u);
            window = word;
        }
    }

    /// `bytes` popped off the shared receive FIFO, the first `room` of
    /// them kept. The whole packet is always popped, whatever is kept:
    /// a word left behind would put every later entry out of step.
    static void pop(uint8_t* dst, uint32_t room, uint32_t bytes) {
        volatile uint32_t& window = fifo(0);
        for (uint32_t at = 0; at < bytes; at += 4u) {
            const uint32_t word = window;
            const uint32_t left = bytes - at;
            const uint32_t take = left < 4u ? left : 4u;
            if (dst != nullptr && at < room) {
                const uint32_t keep = (room - at) < take ? (room - at) : take;
                memcpy(dst + at, &word, keep);
            }
        }
    }

    /// The receive FIFO drained UP TO AND INCLUDING its first data
    /// packet, and no further: one packet per endpoint is what the
    /// contract can carry, and RXFLVL is a level that brings the rest
    /// back on the next pass.
    static void drain_receive_fifo(UsbEvents& ev) {
        // The bound is the FIFO's own depth in entries: a status word is
        // one word, so no run of entries can be longer, and reading an
        // empty receive FIFO is what the chapter calls undefined.
        for (uint16_t entries = 0; entries < rx_words; ++entries) {
            if ((global().GINTSTS & USB_OTG_GINTSTS_RXFLVL) == 0u) {
                return;
            }
            const uint32_t entry = global().GRXSTSP;
            rx_entries_ = rx_entries_ + 1u;
            const uint8_t n = static_cast<uint8_t>(entry & USB_OTG_GRXSTSP_EPNUM);
            const uint32_t bytes = (entry & USB_OTG_GRXSTSP_BCNT) >> USB_OTG_GRXSTSP_BCNT_Pos;
            const uint8_t kind =
                static_cast<uint8_t>((entry & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos);
            if (kind == OtgPacketStatus::setup_data) {
                // Always eight bytes; the last of a back-to-back run is
                // the one the stack answers, and the last one stored is
                // exactly that.
                pop(reinterpret_cast<uint8_t*>(setup_words_), sizeof setup_words_, bytes);
                take_address_early();
                continue;
            }
            if (kind == OtgPacketStatus::out_data) {
                if (n < endpoint_count) {
                    pop(out_buf_[n], max_packet, bytes);
                    out_length_[n] = static_cast<uint16_t>(bytes < max_packet ? bytes : max_packet);
                    if (out_credits_[n] != 0u) {
                        out_credits_[n] = static_cast<uint8_t>(out_credits_[n] - 1u);
                    }
                    if (out_armed_[n] != 0u) {
                        out_armed_[n] = static_cast<uint8_t>(out_armed_[n] - 1u);
                    }
                    ev.out_done |= static_cast<uint16_t>(1u << n);
                } else {
                    pop(nullptr, 0, bytes);
                }
                return;
            }
            // Global OUT NAK, the setup stage done, the transfer done:
            // no payload, and the endpoint interrupt that follows each is
            // where they are acted on.
            pop(nullptr, 0, bytes);
        }
    }

    static void serve_endpoints(UsbEvents& ev) {
        const uint32_t pending = device().DAINT & device().DAINTMSK;
        for (uint8_t n = 0; n < endpoint_count; ++n) {
            if ((pending & (1UL << n)) != 0u) {
                serve_in(n, ev);
            }
            if ((pending & (1UL << (16u + n))) != 0u) {
                serve_out(n, ev);
            }
        }
    }

    static void serve_in(uint8_t n, UsbEvents& ev) {
        const uint32_t flags = in_ep(n).DIEPINT;
        if ((flags & USB_OTG_DIEPINT_XFRC) != 0u) {
            in_ep(n).DIEPINT = USB_OTG_DIEPINT_XFRC;
            ev.in_done |= static_cast<uint16_t>(1u << n);
        }
        if ((flags & USB_OTG_DIEPINT_TOC) != 0u) {
            in_ep(n).DIEPINT = USB_OTG_DIEPINT_TOC;
            timeouts_ = timeouts_ + 1u;
        }
        if ((flags & USB_OTG_DIEPINT_ITTXFE) != 0u) {
            in_ep(n).DIEPINT = USB_OTG_DIEPINT_ITTXFE;
        }
        if ((flags & USB_OTG_DIEPINT_EPDISD) != 0u) {
            in_ep(n).DIEPINT = USB_OTG_DIEPINT_EPDISD;
        }
        // The empty-level interrupt is armed for one endpoint at a time,
        // and only by a submit_in that found no room; it is disarmed the
        // moment the held packet goes.
        if ((flags & USB_OTG_DIEPINT_TXFE) != 0u && (device().DIEPEMPMSK & (1UL << n)) != 0u) {
            const uint16_t left = in_pending_[n];
            if (left != 0u && (in_ep(n).DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) >= (left + 3u) / 4u) {
                push(n, in_buf_[n], left);
                in_pending_[n] = 0;
            }
            if (in_pending_[n] == 0u) {
                device().DIEPEMPMSK &= ~(1UL << n);
            }
        }
    }

    static void serve_out(uint8_t n, UsbEvents& ev) {
        const uint32_t flags = out_ep(n).DOEPINT;
        if ((flags & USB_OTG_DOEPINT_STUP) != 0u) {
            out_ep(n).DOEPINT = USB_OTG_DOEPINT_STUP;
            if (n == 0u) {
                // STUPCNT counts DOWN by one per SETUP packet and is not
                // restored by the core: a control transfer whose whole
                // answer is a status IN never touches DOEPTSIZ0, so
                // after three of those the endpoint would stop taking
                // SETUP packets. Topped up here, at every setup.
                arm_setup();
                ev.setup = true;
            }
        }
        if ((flags & USB_OTG_DOEPINT_B2BSTUP) != 0u) {
            out_ep(n).DOEPINT = USB_OTG_DOEPINT_B2BSTUP;
            setup_overruns_ = setup_overruns_ + 1u;
        }
        if ((flags & USB_OTG_DOEPINT_XFRC) != 0u) {
            out_ep(n).DOEPINT = USB_OTG_DOEPINT_XFRC;
            out_armed_[n] = 0;
            rearm_out(n);
        }
        if ((flags & USB_OTG_DOEPINT_OTEPDIS) != 0u) {
            out_ep(n).DOEPINT = USB_OTG_DOEPINT_OTEPDIS;
        }
        if ((flags & USB_OTG_DOEPINT_EPDISD) != 0u) {
            out_ep(n).DOEPINT = USB_OTG_DOEPINT_EPDISD;
        }
    }

    static inline uint16_t next_fifo_ = static_cast<uint16_t>(rx_words + ep0_tx_words);
    static inline uint32_t setup_words_[2]{};
    static inline uint8_t out_buf_[endpoint_count][max_packet]{};
    static inline uint8_t in_buf_[endpoint_count][max_packet]{};
    static inline uint16_t out_length_[endpoint_count]{};
    static inline uint16_t in_pending_[endpoint_count]{};
    static inline uint16_t in_mps_[endpoint_count]{};
    static inline uint16_t out_mps_[endpoint_count]{};
    static inline uint8_t out_credits_[endpoint_count]{};
    static inline volatile uint8_t out_armed_[endpoint_count]{};
    static inline bool in_active_[endpoint_count]{};
    static inline bool out_active_[endpoint_count]{};
    // Read by the program, written by the ISR body.
    static inline volatile uint8_t address_ = 0;
    static inline volatile uint32_t rx_entries_ = 0;
    static inline volatile uint32_t fifo_waits_ = 0;
    static inline volatile uint32_t stalls_ = 0;
    static inline volatile uint32_t timeouts_ = 0;
    static inline volatile uint32_t setup_overruns_ = 0;
    static inline volatile uint32_t early_suspends_ = 0;
    static inline volatile uint32_t mode_mismatches_ = 0;
    static inline volatile uint32_t sessions_ = 0;
    static inline volatile uint32_t otg_events_ = 0;
    static inline volatile uint32_t enumerations_ = 0;
};

#if defined(USB_OTG_FS_PERIPH_BASE)
/// The full-speed core, on PA11 and PA12.
using UsbFs = UsbOtg<OtgCore::fs>;
static_assert(UsbController<UsbFs>);
#endif

#if defined(USB_OTG_HS_PERIPH_BASE)
/// The high-speed core driven through its own EMBEDDED full-speed PHY,
/// on PB14 and PB15 - the same programmer's model at another base, and
/// the way out on a part where ES0206 2.2.14 asks for PA12 to be left
/// unconnected.
using UsbHs = UsbOtg<OtgCore::hs>;
static_assert(UsbController<UsbHs>);
#endif

} // namespace brio

#endif   // the part has an OTG core at all (the F410 has neither)
