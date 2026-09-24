/*
 * usb.hpp
 *
 * The CH32V203's USB DEVICE controller (RM ch. 21), realizing
 * util/usb's UsbController at the packet.
 *
 * WHAT THIS BLOCK IS. Register for register and bit for bit, it is the
 * USB device peripheral of the STM32F1 and F0 lines: eight endpoint
 * registers whose status fields are TOGGLES, a control and a status
 * register, a device address, and a packet memory the CPU and the
 * engine share, addressed through a table of descriptors the program
 * lays out itself. Nothing here is DWC2 (the STM32F4's core) and
 * nothing here is the RP2040's: this is the third USB device controller
 * shape in this repository, and the first of ITS shape - which is why
 * it lives in this stratum and not in one of its own. An IP stratum is
 * born at the SECOND family that carries the IP, never at the first.
 *
 * THE PACKET MEMORY IS 512 BYTES SEEN THROUGH A 32-BIT WINDOW. Each
 * 16-bit word of it occupies four bytes of the CPU's address space, so
 * the PMA byte offset `o` lives at 0x40006000 + 2*o and nothing wider
 * than a halfword may be stored. Every access here goes through pma().
 *
 * AND IT IS SHARED WITH THE CAN CONTROLLER. 21.2.1: when CAN is used,
 * its filter table takes the TOP 128 bytes and the USB keeps the low
 * 384. That is not a fact this file can check - CAN is another driver's
 * business - so it is a PARAMETER: `Usbd<>` budgets 384 bytes and
 * refuses the endpoint that would not fit, and a program with no CAN
 * can say `Usbd<512>` and get the rest. The budget is spent by
 * configure_endpoint() in order, and a bus reset returns it.
 *
 * THE STATUS FIELDS ARE WRITTEN BY XOR. STAT_RX, STAT_TX, DTOG_RX and
 * DTOG_TX are "write 1 to INVERT, write 0 to leave" (21.3.6), while
 * CTR_RX and CTR_TX in the same register are "write 0 to clear, write 1
 * to leave". So no field of an endpoint register can be written
 * directly: a store computes the XOR that lands on the value it wants
 * and puts ones in the two flags it must not disturb. That is what
 * write_epr(), set_stat_tx() and set_stat_rx() are for, and it is the
 * single sharpest edge of this peripheral.
 *
 * AND THE PROGRAM MUST NOT SLEEP WHILE IT USES THIS BLOCK. Measured on
 * the bench, and the reference manual says the opposite: 2.4's table
 * gives Sleep as "core clock off, no effect on other clocks" and its
 * prose as "the core stops running and all peripherals are still
 * running" - but with the core in WFI or WFE this controller cannot
 * reach its packet memory. An armed bulk endpoint receives NOTHING (the
 * host's bytes are lost and the packet-memory overflow counts up, one
 * per attempt), and an enumeration never gets past its first control
 * transfer: the SETUP is taken and answered, the host acknowledges the
 * data, and the status stage dies with an overflow. The same image with
 * a loop that only step()s enumerates, configures and carries bytes with
 * not one overflow. And the vendor's own example (the EVT's SimulateCDC,
 * built with WCH's compiler) fails the same way with one wfi added to
 * its loop and with its own __WFE(), enumeration and bulk OUT alike,
 * while a busy-wait of the same length works - with the core woken
 * every USB frame, so the rule is not about how long a sleep lasts.
 * Nor is it the USB's alone: in Sleep no bus master but the core gets
 * a cycle (a DMA stalls the same way), and no mitigation short of
 * staying awake works - the overflow interrupt wakes the core but the
 * packet is gone, a NAK'd endpoint still overflows; dividing HCLK
 * works down to 24 MHz, and 12 fails like the sleep - which is why
 * init() refuses a tree below `usbd_min_hclk_hz`.
 * SO THE RULE IS A MECHANISM AND NOT AN INSTRUCTION TO THE
 * PROGRAMMER: an attached controller counts itself as a bus master
 * (ch32vx03/bus_activity.hpp), idle() DOES NOT SLEEP while it is
 * attached, and a sleep site refuses to arm over it - so a program
 * with USB may drive the kernel any way it likes, the loop included,
 * and what decides is the silicon's own state. `connect()` is where
 * the count is held and released.
 *
 * AND AN OVERFLOW IS NOT SOMETHING A RECEIVER SURVIVES BY ITSELF. The
 * lost packet leaves its COUNT in the halfword that carries the
 * buffer's SIZE, and raises no completion - so an endpoint left alone
 * after one overflow reads as a buffer of zero blocks and loses every
 * packet after it, for ever. take_events() therefore writes that
 * encoding back for every armed receiver whenever the flag stands
 * (restore_rx_buffers): the counter still climbs for as long as the
 * cause lasts, and the port carries bytes again the moment it stops.
 *
 * WHAT IS NOT COVERED YET, each with its reason:
 *  - the DOUBLE BUFFER (EP_KIND on a bulk endpoint) and the
 *    isochronous endpoints: a CDC console needs neither, and the
 *    double buffer changes the meaning of DTOG on every access - it is
 *    written when a program needs the bandwidth and can measure it.
 *  - SUSPEND's low-power half: the handshake of 21.2.4 is kept as far
 *    as FSUSP, because that is what arms the wake-up (a module left
 *    running through a host's suspend never reports the resume -
 *    measured), but LPMODE and the regulator are not touched. What a
 *    suspended controller would let the program sleep through is a
 *    question for a meter, and the count above is deliberately held
 *    from the pull-up to the detach and not from the first packet to
 *    the suspend.
 *  - the USBFS host/device controller (ch. 23) on PB6/PB7, which is a
 *    DIFFERENT peripheral with its own registers, and the 1-wire mode
 *    of CNTR, which the manual gives to another family's lot numbers.
 *  - remote wake-up (CNTR.RESUME): no program asks for it yet.
 */

#pragma once

#include <stdint.h>

#include <span>

#include "ch32vx03/bus_activity.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "util/usb/device.hpp"

namespace brio {

// ---- the registers (RM 21.3) ----------------------------------------------
/// Sixteen-bit registers on 32-bit boundaries, the block's own layout.
struct UsbdRegs {
    struct Endpoint {
        volatile uint16_t R;      ///< USBD_EPRx
        uint16_t RESERVED;
    } EPR[8];                     ///< 0x00..0x1c
    uint32_t RESERVED0[8];        ///< 0x20..0x3c
    volatile uint16_t CNTR;   uint16_t RESERVED1;   ///< 0x40 control
    volatile uint16_t ISTR;   uint16_t RESERVED2;   ///< 0x44 interrupt status
    volatile uint16_t FNR;    uint16_t RESERVED3;   ///< 0x48 frame number
    volatile uint16_t DADDR;  uint16_t RESERVED4;   ///< 0x4c device address
    volatile uint16_t BTABLE; uint16_t RESERVED5;   ///< 0x50 the table's offset in the PMA
};

inline UsbdRegs* usbd() { return reinterpret_cast<UsbdRegs*>(pb1_base + 0x5c00); }

/// The shared packet memory. `offset` is a PMA BYTE offset; the window
/// is twice as wide (see the file header).
inline constexpr uint32_t usbd_pma_base = pb1_base + 0x6000;
inline volatile uint16_t& usbd_pma(uint16_t offset) {
    return *reinterpret_cast<volatile uint16_t*>(usbd_pma_base + 2u * static_cast<uint32_t>(offset));
}

/// USBD_CNTR (21.3.1)
inline constexpr uint16_t usbd_fres     = 1U << 0;    ///< force the module into reset (set out of reset)
inline constexpr uint16_t usbd_pdwn     = 1U << 1;    ///< the analogue off (set out of reset)
inline constexpr uint16_t usbd_lpmode   = 1U << 2;
inline constexpr uint16_t usbd_fsusp    = 1U << 3;
inline constexpr uint16_t usbd_resume   = 1U << 4;
inline constexpr uint16_t usbd_esofm    = 1U << 8;
inline constexpr uint16_t usbd_sofm     = 1U << 9;
inline constexpr uint16_t usbd_resetm   = 1U << 10;
inline constexpr uint16_t usbd_suspm    = 1U << 11;
inline constexpr uint16_t usbd_wkupm    = 1U << 12;
inline constexpr uint16_t usbd_errm     = 1U << 13;
inline constexpr uint16_t usbd_pmaovrm  = 1U << 14;
inline constexpr uint16_t usbd_ctrm     = 1U << 15;

/// USBD_ISTR (21.3.2). Every flag but CTR is read-and-write-zero: it is
/// cleared by storing a word with a ZERO in it and ones everywhere
/// else. CTR is not clearable here at all - it stands until the
/// endpoint's own CTR_RX/CTR_TX is cleared.
inline constexpr uint16_t istr_ep_mask = 0x000FU;     ///< which endpoint the CTR belongs to
inline constexpr uint16_t istr_dir     = 1U << 4;     ///< 1: the event is a reception (OUT or SETUP)
inline constexpr uint16_t istr_esof    = 1U << 8;
inline constexpr uint16_t istr_sof     = 1U << 9;
inline constexpr uint16_t istr_reset   = 1U << 10;
inline constexpr uint16_t istr_susp    = 1U << 11;
inline constexpr uint16_t istr_wkup    = 1U << 12;
inline constexpr uint16_t istr_err     = 1U << 13;
inline constexpr uint16_t istr_pmaovr  = 1U << 14;
inline constexpr uint16_t istr_ctr     = 1U << 15;

/// USBD_DADDR (21.3.4)
inline constexpr uint16_t usbd_daddr_mask = 0x007FU;
inline constexpr uint16_t usbd_ef         = 1U << 7;   ///< the endpoint machine enabled

/// USBD_EPRx (21.3.6)
inline constexpr uint16_t ep_ea_mask   = 0x000FU;
inline constexpr uint16_t ep_stat_tx   = 0x0030U;
inline constexpr uint16_t ep_dtog_tx   = 0x0040U;
inline constexpr uint16_t ep_ctr_tx    = 0x0080U;
inline constexpr uint16_t ep_kind      = 0x0100U;
inline constexpr uint16_t ep_type_mask = 0x0600U;
inline constexpr uint16_t ep_setup     = 0x0800U;
inline constexpr uint16_t ep_stat_rx   = 0x3000U;
inline constexpr uint16_t ep_dtog_rx   = 0x4000U;
inline constexpr uint16_t ep_ctr_rx    = 0x8000U;

/// The endpoint types as EPTYPE codes - the chapter's order, which is
/// NOT the descriptor's (bulk is 00 here and 10 on the wire).
inline constexpr uint16_t ep_type_bulk      = 0x0000U;
inline constexpr uint16_t ep_type_control   = 0x0200U;
inline constexpr uint16_t ep_type_iso       = 0x0400U;
inline constexpr uint16_t ep_type_interrupt = 0x0600U;

/// The four status codes, in the position of STAT_TX (shift for RX).
inline constexpr uint16_t ep_stat_disabled = 0x0U;
inline constexpr uint16_t ep_stat_stall    = 0x1U;
inline constexpr uint16_t ep_stat_nak      = 0x2U;
inline constexpr uint16_t ep_stat_valid    = 0x3U;

/// The bits a plain store must carry through untouched: the address,
/// the kind, the type, the read-only SETUP flag - and NOT the four
/// toggles, which a one would invert.
inline constexpr uint16_t ep_keep = ep_ea_mask | ep_kind | ep_type_mask | ep_setup;

/// THE BUS HAS A FLOOR, AND IT IS THE SLEEP FINDING BY ANOTHER ROUTE.
/// What this controller needs of the bus matrix is CYCLES: with the
/// core awake and HCLK divided down it enumerates and carries its four
/// kilobytes at 96, at 48 and at 24 MHz, and at 12 it fails exactly as
/// a sleeping core makes it fail - the packets lost and the overflow
/// counting up. 24 MHz is therefore the lowest rate this driver will be
/// run at, and init() refuses a slower tree where the rate is a
/// constant.
inline constexpr uint32_t usbd_min_hclk_hz = 24'000'000UL;

/// COUNTn_RX carries the buffer's SIZE as blocks, not as bytes: up to
/// 62 bytes in blocks of two, 64 and above in blocks of thirty-two
/// (21.3.10). The received count comes back in the low ten bits.
constexpr uint16_t usbd_count_rx_for(uint16_t bytes) {
    return bytes <= 62u ? static_cast<uint16_t>((bytes + 1u) / 2u << 10)
                        : static_cast<uint16_t>(0x8000U | (((bytes / 32u) - 1u) << 10));
}

/**
 * The device controller.
 *
 * `pma_bytes` is how much of the shared 512-byte memory this program
 * gives the USB (see the file header): 384 by default, which is what is
 * left when the CAN controller takes its filter table.
 */
template <uint16_t pma_bytes = 384>
struct Usbd {
    static_assert(pma_bytes >= 128u && pma_bytes <= 512u && (pma_bytes % 2u) == 0u,
                  "brio Usbd: the shared memory is 512 bytes, and the CAN filter table takes the "
                  "top 128 of them when CAN is used (RM 21.2.1)");
    static_assert(device::has_usbd, "brio Usbd: this part has no USB device controller");

    Usbd() = delete;

    /// The largest packet this driver stages out of the PMA - the
    /// full-speed maximum for control, bulk and interrupt endpoints.
    /// Isochronous endpoints may ask for more and are refused.
    static constexpr uint16_t max_packet = 64;
    /// The buffer description table is the first thing in the memory:
    /// eight entries of four halfwords.
    static constexpr uint16_t btable_offset = 0;
    static constexpr uint16_t buffer_floor = 64;

    static UsbdRegs& regs() { return *usbd(); }

    /**
     * Bring the block up: its clock, then the wake from power-down that
     * 21.2.2 spells out - clear PDWN, let the analogue start, clear
     * FRES, clear the status flags, arm the interrupts, enable the
     * endpoint machine at address zero. The pull-up is NOT raised here:
     * connect() is the stack's, and the host must not see a device
     * until the program is ready to answer it.
     *
     * The clock is checked at COMPILE time, and TWICE: this peripheral
     * wants 48 MHz exactly and the tree's USBPRE is what makes it
     * (clock.hpp), so a program whose rate cannot feed it does not
     * build; and the BUS has its own floor, measured, so neither does a
     * program that would run the core below usbd_min_hclk_hz while the
     * controller is attached to it.
     */
    template <typename C>
    static bool init(C clock) {
        static_assert(C::usb_hz == usb_required_hz,
                      "brio Usbd: this controller must be fed 48 MHz, and only a PLL rate of 48, "
                      "96 or 144 MHz divides to it (clock.hpp's USBPRE)");
        static_assert(C::hz >= usbd_min_hclk_hz,
                      "brio Usbd: this controller loses packets with HCLK below 24 MHz - "
                      "measured, and the same finding as the sleep: what it needs on the bus "
                      "matrix is cycles (96, 48 and 24 MHz carry data, 12 does not)");
        (void)clock;

        // THE PADS ARE STILL GPIO PADS, AND THIS IS THE STEP THAT IS
        // NOWHERE IN THE CHAPTER. D- and D+ are PA11 and PA12, and the
        // block reaches them THROUGH PORT A: with that port's clock
        // closed - which is how it comes out of reset - the pull-up
        // still works (it lives in EXTEN, not in the port), so the host
        // sees a device attach and starts enumerating, and then every
        // packet fails. Measured that way on this bench: the SETUP
        // counted but its first halfword never stored, sixteen
        // packet-memory overflows in one attempt, the host giving up
        // with a protocol error. WCH's own USB_Port_Set() opens the
        // port's clock and leaves the two pads FLOATING INPUTS before
        // it touches the pull-up, and so does this.
        //
        // The two pads are named through their PORT rather than as
        // Pin<'A', 11> and Pin<'A', 12>, because this is a template
        // whose body is checked where it is written: a Pin naming a pad
        // is formed there, and two packages of this family bond neither
        // of these (parts/ch32v203f8.hpp). What refuses the part is the
        // static_assert on device::has_usbd above, which is the same
        // fact - a part with this controller brings its pads out - and
        // the header still compiles everywhere, as every header of this
        // stratum must.
        Port<'A'>::configure(11, pin_nibble(PinMode::input, PinDrive::push_pull, PinSpeed::fast));
        Port<'A'>::configure(12, pin_nibble(PinMode::input, PinDrive::push_pull, PinSpeed::fast));
        Rcc::enable(Bus::pb1, rcc_pb1_usbd);

        regs().CNTR = usbd_fres;            // hold the module in reset, analogue on
        regs().CNTR = 0;                    // out of reset and out of power-down
        regs().ISTR = 0;                    // nothing pending from before

        next_buffer_ = buffer_floor;
        // The two counters belong to the ATTACHMENT and not to the
        // program: they say what this bring-up of the block has seen,
        // so a program that gives the block back and takes it again
        // reads its own attempt and not the last one's.
        errors_ = 0;
        overruns_ = 0;
        regs().BTABLE = btable_offset;
        if (!claim_control_endpoint()) {
            return false;
        }

        regs().CNTR = usbd_ctrm | usbd_resetm | usbd_suspm | usbd_wkupm | usbd_errm | usbd_pmaovrm;
        regs().DADDR = usbd_ef;

        Pfic::enable(Irq::usb_lp_can1_rx0);
        return true;
    }

    /// Give the block back: the interrupts off, the pull-up down, the
    /// module in reset and its clock closed.
    static void release() {
        connect(false);
        regs().CNTR = usbd_fres | usbd_pdwn;
        Pfic::disable(Irq::usb_lp_can1_rx0);
        Rcc::disable(Bus::pb1, rcc_pb1_usbd);
    }

    // ---- the UsbController contract ---------------------------------------

    /// The internal 1.5k pull-up on D+, which is what tells the host a
    /// full-speed device is there. It lives in EXTEN and not in this
    /// block (RM 33.2.1), so the store masks the lock-up flag out - that
    /// bit is write-one-to-clear and is not this driver's to take.
    ///
    /// AND IT IS WHERE THIS CONTROLLER COUNTS ITSELF AS A BUS MASTER.
    /// From the pull-up to the detach the block reaches its packet
    /// memory whenever the host speaks, and in a sleep of any depth it
    /// cannot (the file header); so an attached controller holds one
    /// count in ch32vx03/bus_activity.hpp, which is what keeps the
    /// kernel's idle path from sleeping and a sleep site from arming.
    static void connect(bool on) {
        const bool was = pulled_up();
        const uint32_t ctr = exten()->CTR & ~exten_lkuprst;
        exten()->CTR = on ? (ctr | exten_usbd_pullup) : (ctr & ~exten_usbd_pullup);
        if (on != was) {
            if (on) {
                BusActivity::entered();
            } else {
                BusActivity::left();
            }
        }
    }

    static void set_address(uint8_t address) {
        regs().DADDR = static_cast<uint16_t>(usbd_ef | (address & usbd_daddr_mask));
    }

    /**
     * Claim one direction of one endpoint: its type, its buffer in the
     * shared memory, and its status set to NAK - armed by submit_in or
     * submit_out and not before.
     *
     * False when the packet is larger than this driver stages, when the
     * endpoint number is past the eight this block has, or when the
     * memory budget is spent.
     */
    static bool configure_endpoint(uint8_t address, UsbEndpointType type, uint16_t max) {
        const uint8_t n = static_cast<uint8_t>(address & 0x0Fu);
        const bool in = (address & 0x80u) != 0u;
        if (n >= 8u || max == 0u || max > max_packet) {
            return false;
        }
        const uint16_t size = static_cast<uint16_t>((max + 1u) & ~1u);   // the PMA moves halfwords
        if (next_buffer_ + size > pma_bytes) {
            return false;
        }
        const uint16_t buffer = next_buffer_;
        next_buffer_ = static_cast<uint16_t>(next_buffer_ + size);

        write_epr(n, static_cast<uint16_t>(n | eptype_code(type)));
        if (in) {
            tx_addr_[n] = buffer;
            pma(entry(n) + 0u) = buffer;
            pma(entry(n) + 2u) = 0;
            set_stat_tx(n, ep_stat_nak);
        } else {
            rx_addr_[n] = buffer;
            rx_size_[n] = size;
            pma(entry(n) + 4u) = buffer;
            pma(entry(n) + 6u) = usbd_count_rx_for(size);
            set_stat_rx(n, ep_stat_nak);
        }
        return true;
    }

    /// Every endpoint but zero released, and the memory they held given
    /// back - the stack's answer to a bus reset and to a configuration
    /// the host takes away.
    static void deconfigure_endpoints() {
        for (uint8_t n = 1; n < 8u; ++n) {
            set_stat_tx(n, ep_stat_disabled);
            set_stat_rx(n, ep_stat_disabled);
            write_epr(n, 0);
            tx_addr_[n] = 0;
            rx_addr_[n] = 0;
            rx_size_[n] = 0;
            out_len_[n] = 0;
        }
        next_buffer_ = static_cast<uint16_t>(buffer_floor + 2u * max_packet);   // endpoint zero's two
    }

    static void stall(uint8_t address, bool on) {
        const uint8_t n = static_cast<uint8_t>(address & 0x0Fu);
        if (n >= 8u) {
            return;
        }
        const uint16_t stat = on ? ep_stat_stall : ep_stat_nak;
        if ((address & 0x80u) != 0u) {
            set_stat_tx(n, stat);
        } else {
            set_stat_rx(n, stat);
        }
    }

    static bool stalled(uint8_t address) {
        const uint8_t n = static_cast<uint8_t>(address & 0x0Fu);
        if (n >= 8u) {
            return false;
        }
        const uint16_t epr = regs().EPR[n].R;
        return (address & 0x80u) != 0u
                   ? ((epr & ep_stat_tx) >> 4) == ep_stat_stall
                   : ((epr & ep_stat_rx) >> 12) == ep_stat_stall;
    }

    /// One packet into the endpoint's buffer, and the endpoint armed to
    /// answer the next IN token with it. An empty span is the
    /// zero-length packet.
    static bool submit_in(uint8_t number, std::span<const uint8_t> data) {
        if (number >= 8u || data.size() > max_packet) {
            return false;
        }
        pma_write(tx_addr_[number], data);
        pma(entry(number) + 2u) = static_cast<uint16_t>(data.size());
        set_stat_tx(number, ep_stat_valid);
        return true;
    }

    /// Arm the reception of one packet. The buffer and its size were
    /// fixed by configure_endpoint; what this rewrites is the size
    /// encoding, because the hardware overwrites that halfword with the
    /// COUNT of what arrived.
    static bool submit_out(uint8_t number, uint16_t max) {
        if (number >= 8u || rx_size_[number] == 0u || max > rx_size_[number]) {
            return false;
        }
        pma(entry(number) + 6u) = usbd_count_rx_for(rx_size_[number]);
        set_stat_rx(number, ep_stat_valid);
        return true;
    }

    /// What the last OUT packet on that endpoint carried. The bytes were
    /// copied out of the shared memory when the packet arrived, so this
    /// span stays valid until the next one does.
    static std::span<const uint8_t> out_data(uint8_t number) {
        if (number >= 8u) {
            return {};
        }
        return {out_buf_[number], out_len_[number]};
    }

    static UsbSetup setup() { return setup_; }

    /**
     * What happened since the last call, acknowledged.
     *
     * The status register is drained in a loop because several events
     * can stand at once and CTR is not clearable in it at all - it
     * falls only when the endpoint's own flag does. The loop is bounded:
     * a block that keeps raising events would otherwise hold the
     * handler for ever.
     */
    static UsbEvents take_events() {
        UsbEvents ev;
        for (uint8_t turn = 0; turn < 32u; ++turn) {
            const uint16_t istr = regs().ISTR;

            if ((istr & istr_reset) != 0u) {
                clear_istr(istr_reset);
                on_bus_reset();
                ev.reset = true;
                continue;
            }
            if ((istr & istr_susp) != 0u) {
                // THE SUSPEND IS A HANDSHAKE AND NOT A NOTIFICATION
                // (21.2.4): the program answers the bus going idle by
                // putting the module in suspend itself, with FSUSP. That
                // is what shields the detector - the flag is the bus
                // STATE and the hardware raises it again while the bus
                // stays idle - and it is also what arms the wake-up:
                // measured, a module left running through a host's
                // suspend never reports the resume, because the
                // condition for WKUP is a wake-up signal reaching a
                // SUSPENDED module. LPMODE, the low-power half of the
                // same paragraph, is not touched here (the file header).
                force_suspend(true);
                clear_istr(istr_susp);
                ev.suspend = true;
                continue;
            }
            if ((istr & istr_wkup) != 0u) {
                // The way back, in the manual's order: the detector
                // restarted first, the flag cleared after.
                force_suspend(false);
                clear_istr(istr_wkup);
                ev.resume = true;
                continue;
            }
            if ((istr & istr_err) != 0u) {
                clear_istr(istr_err);
                bump(errors_);
                continue;
            }
            if ((istr & istr_pmaovr) != 0u) {
                clear_istr(istr_pmaovr);
                restore_rx_buffers();
                bump(overruns_);
                continue;
            }
            if ((istr & istr_sof) != 0u) {
                clear_istr(istr_sof);
                continue;
            }
            if ((istr & istr_esof) != 0u) {
                clear_istr(istr_esof);
                continue;
            }
            if ((istr & istr_ctr) != 0u) {
                service(static_cast<uint8_t>(istr & istr_ep_mask), ev);
                continue;
            }
            break;
        }
        return ev;
    }

    // ---- introspection -----------------------------------------------------
    static uint16_t frame() { return static_cast<uint16_t>(regs().FNR & 0x07FFu); }
    static uint8_t address() { return static_cast<uint8_t>(regs().DADDR & usbd_daddr_mask); }
    static bool pulled_up() { return (exten()->CTR & exten_usbd_pullup) != 0u; }
    static uint16_t buffer_used() { return next_buffer_; }
    static uint16_t buffer_free() { return static_cast<uint16_t>(pma_bytes - next_buffer_); }
    /// Both counters saturate and both count from init(): what THIS
    /// bring-up of the block has seen.
    static uint16_t errors() { return errors_; }
    static uint16_t overruns() { return overruns_; }

private:
    /// The buffer description entry of endpoint n, as a PMA offset.
    static constexpr uint16_t entry(uint8_t n) {
        return static_cast<uint16_t>(btable_offset + 8u * n);
    }

    static volatile uint16_t& pma(uint16_t offset) { return usbd_pma(offset); }

    static constexpr uint16_t eptype_code(UsbEndpointType type) {
        switch (type) {
            case UsbEndpointType::control:   return ep_type_control;
            case UsbEndpointType::isochronous: return ep_type_iso;
            case UsbEndpointType::interrupt: return ep_type_interrupt;
            default:                         return ep_type_bulk;
        }
    }

    /// A plain store into an endpoint register: the fields that are
    /// ordinary bits, with ZERO in every toggle (which leaves them) and
    /// ONE in the two flags (which leaves them too).
    static void write_epr(uint8_t n, uint16_t value) {
        regs().EPR[n].R = static_cast<uint16_t>(value | ep_ctr_rx | ep_ctr_tx);
    }

    /// The transmit status, written by XOR (see the file header).
    static void set_stat_tx(uint8_t n, uint16_t stat) {
        const uint16_t current = regs().EPR[n].R;
        const uint16_t want = static_cast<uint16_t>(stat << 4);
        regs().EPR[n].R = static_cast<uint16_t>(((current & (ep_keep | ep_stat_tx)) ^ want) |
                                                ep_ctr_rx | ep_ctr_tx);
    }

    /// The receive status, the same way.
    static void set_stat_rx(uint8_t n, uint16_t stat) {
        const uint16_t current = regs().EPR[n].R;
        const uint16_t want = static_cast<uint16_t>(stat << 12);
        regs().EPR[n].R = static_cast<uint16_t>(((current & (ep_keep | ep_stat_rx)) ^ want) |
                                                ep_ctr_rx | ep_ctr_tx);
    }

    /// Clear one of the two completion flags: a zero in the one that
    /// goes, a one in the other, zeros in every toggle.
    static void clear_ctr_rx(uint8_t n) {
        regs().EPR[n].R = static_cast<uint16_t>((regs().EPR[n].R & ep_keep) | ep_ctr_tx);
    }
    static void clear_ctr_tx(uint8_t n) {
        regs().EPR[n].R = static_cast<uint16_t>((regs().EPR[n].R & ep_keep) | ep_ctr_rx);
    }

    /// The status register's flags are read-and-write-zero: a store with
    /// a zero in one bit and ones everywhere else clears exactly it.
    static void clear_istr(uint16_t flag) {
        regs().ISTR = static_cast<uint16_t>(~flag);
    }

    /**
     * THE REPAIR AFTER A LOST PACKET, and the same halfword's trap for
     * the third time. An ordinary reception writes its length into the
     * low ten bits of COUNTn_RX and leaves the SIZE above them standing
     * (0x8440 for a 64-byte packet in a 64-byte buffer, measured). An
     * overflow does not: the size field comes back ZERO, the halfword
     * reading as the bare count (0x0040, measured over the debug port
     * on a wedged endpoint) - and because the packet never completed
     * there is no CTR_RX, so no layer above ever arms that endpoint
     * again. It stays VALID over a buffer that now reads as ZERO
     * BLOCKS, and every packet after it overflows too: measured, an
     * endpoint left alone after one overflow takes 64 bytes in 200 ms
     * where a healthy one takes ninety thousand, and the counter climbs
     * for as long as the host keeps sending.
     *
     * So the size encoding is written back for every endpoint that is
     * still armed. An endpoint the class holds at NAK is left alone
     * (submit_out writes the field when it arms it), and so is one with
     * a reception STANDING, whose length is that same halfword and is
     * the stack's to read.
     */
    static void restore_rx_buffers() {
        for (uint8_t n = 0; n < 8u; ++n) {
            if (rx_size_[n] == 0u) {
                continue;
            }
            const uint16_t epr = regs().EPR[n].R;
            if (((epr & ep_stat_rx) >> 12) != ep_stat_valid || (epr & ep_ctr_rx) != 0u) {
                continue;
            }
            pma(entry(n) + 6u) = usbd_count_rx_for(rx_size_[n]);
        }
    }

    /// The module's own suspend, in the control register the interrupt
    /// masks live in - so it is read, changed and written whole.
    static void force_suspend(bool on) {
        const uint16_t cntr = regs().CNTR;
        regs().CNTR = on ? static_cast<uint16_t>(cntr | usbd_fsusp)
                         : static_cast<uint16_t>(cntr & ~usbd_fsusp);
    }

    static void pma_write(uint16_t offset, std::span<const uint8_t> data) {
        uint16_t o = offset;
        size_t i = 0;
        for (; i + 1u < data.size(); i += 2u, o = static_cast<uint16_t>(o + 2u)) {
            pma(o) = static_cast<uint16_t>(data[i] | (static_cast<uint16_t>(data[i + 1u]) << 8));
        }
        if (i < data.size()) {
            // The odd tail: the high byte of the halfword is not sent,
            // because COUNTn_TX and not the buffer says how long the
            // packet is.
            pma(o) = data[i];
        }
    }

    static void pma_read(uint16_t offset, uint8_t* out, uint16_t count) {
        uint16_t o = offset;
        uint16_t i = 0;
        for (; i + 1u < count; i = static_cast<uint16_t>(i + 2u), o = static_cast<uint16_t>(o + 2u)) {
            const uint16_t w = pma(o);
            out[i] = static_cast<uint8_t>(w);
            out[i + 1u] = static_cast<uint8_t>(w >> 8);
        }
        if (i < count) {
            out[i] = static_cast<uint8_t>(pma(o));
        }
    }

    /// Endpoint zero, which init() claims and a bus reset re-claims: two
    /// buffers of the full-speed maximum, control type, receiving.
    static bool claim_control_endpoint() {
        if (buffer_floor + 2u * max_packet > pma_bytes) {
            return false;
        }
        tx_addr_[0] = buffer_floor;
        rx_addr_[0] = static_cast<uint16_t>(buffer_floor + max_packet);
        rx_size_[0] = max_packet;
        next_buffer_ = static_cast<uint16_t>(buffer_floor + 2u * max_packet);

        // A RECEPTION ALREADY IN THE BUFFER IS NOT DISTURBED. The
        // halfword that says how big the buffer is is the SAME one the
        // hardware writes the received count into, so laying the
        // endpoint out again over a packet that has arrived but not yet
        // been serviced destroys its length - and a SETUP of length zero
        // is a request the stack can only stall. That is not a corner:
        // the bus reset and the first SETUP after it are microseconds
        // apart and both can be standing when the handler finally looks
        // (measured on this bench, with the ISR held at a breakpoint).
        const bool reception_pending = (regs().EPR[0].R & ep_ctr_rx) != 0u;

        pma(entry(0) + 0u) = tx_addr_[0];
        pma(entry(0) + 2u) = 0;
        pma(entry(0) + 4u) = rx_addr_[0];
        if (!reception_pending) {
            pma(entry(0) + 6u) = usbd_count_rx_for(max_packet);
        }

        write_epr(0, ep_type_control);
        set_stat_tx(0, ep_stat_nak);
        if (!reception_pending) {
            set_stat_rx(0, ep_stat_valid);
        }
        return true;
    }

    /**
     * A bus reset, which this block takes literally: the endpoint
     * registers, the address and the table are back at their reset
     * values, so the control endpoint has to be laid out again before
     * the host's first SETUP - which follows within microseconds.
     */
    static void on_bus_reset() {
        // A RESET ENDS ANY SUSPEND, and a host may drive one straight
        // out of it (21.2.4 asks a suspended device to take the reset
        // as an ordinary one): the module is put back to work before
        // its endpoints are laid out again.
        force_suspend(false);
        for (uint8_t n = 1; n < 8u; ++n) {
            tx_addr_[n] = 0;
            rx_addr_[n] = 0;
            rx_size_[n] = 0;
            out_len_[n] = 0;
        }
        regs().BTABLE = btable_offset;
        (void)claim_control_endpoint();
        regs().DADDR = usbd_ef;
    }

    /// One endpoint's completion: the reception first, because a SETUP
    /// that arrives while a transmission completes must be seen.
    static void service(uint8_t n, UsbEvents& ev) {
        if (n >= 8u) {
            return;
        }
        const uint16_t epr = regs().EPR[n].R;

        if ((epr & ep_ctr_rx) != 0u) {
            const uint16_t count = static_cast<uint16_t>(pma(entry(n) + 6u) & 0x03FFu);
            const uint16_t len = count > max_packet ? max_packet : count;
            pma_read(rx_addr_[n], out_buf_[n], len);
            out_len_[n] = static_cast<uint8_t>(len);
            const bool is_setup = (epr & ep_setup) != 0u;
            clear_ctr_rx(n);
            if (is_setup && n == 0u) {
                setup_ = parse_setup(out_buf_[0], out_len_[0]);
                ev.setup = true;
            } else {
                ev.out_done = static_cast<uint16_t>(ev.out_done | (1u << n));
            }
            if (is_setup && n == 0u) {
                // A SETUP BEGINS A NEW TRANSFER, so the transmit side
                // goes back to NAK whatever it was: a control endpoint
                // that answered the previous request with a STALL would
                // otherwise stall the new one's data stage before the
                // stack ever saw it.
                set_stat_tx(0, ep_stat_nak);
            }
            if (n == 0u) {
                // THE CONTROL ENDPOINT RE-ARMS ITSELF, and it must: the
                // host may send the next SETUP at any moment, including
                // one that abandons the transfer in progress, and no
                // layer above knows that in time. The stack's
                // submit_out() arms an OUT DATA stage; this arms the
                // endpoint itself.
                //
                // The size field is rewritten with it, because the
                // hardware puts the RECEIVED COUNT in that same halfword
                // and wipes the block encoding doing so (measured on the
                // bench: an endpoint left as the hardware leaves it
                // acknowledges the next packet and stores NONE of it,
                // the count arriving in an otherwise untouched memory).
                pma(entry(0) + 6u) = usbd_count_rx_for(rx_size_[0]);
                set_stat_rx(0, ep_stat_valid);
            }
        }

        if ((epr & ep_ctr_tx) != 0u) {
            clear_ctr_tx(n);
            ev.in_done = static_cast<uint16_t>(ev.in_done | (1u << n));
        }
    }

    static UsbSetup parse_setup(const uint8_t* b, uint8_t len) {
        if (len < 8u) {
            return {};
        }
        return {b[0], b[1],
                static_cast<uint16_t>(b[2] | (static_cast<uint16_t>(b[3]) << 8)),
                static_cast<uint16_t>(b[4] | (static_cast<uint16_t>(b[5]) << 8)),
                static_cast<uint16_t>(b[6] | (static_cast<uint16_t>(b[7]) << 8))};
    }

    /// Saturating: a counter that wraps would report a healthy bus.
    static void bump(uint16_t& counter) {
        if (counter != 0xFFFFu) {
            ++counter;
        }
    }

    static inline uint16_t next_buffer_ = buffer_floor;
    static inline uint16_t tx_addr_[8]{};
    static inline uint16_t rx_addr_[8]{};
    static inline uint16_t rx_size_[8]{};
    static inline uint8_t out_len_[8]{};
    static inline uint8_t out_buf_[8][max_packet]{};
    static inline UsbSetup setup_{};
    static inline uint16_t errors_ = 0;
    static inline uint16_t overruns_ = 0;
};

/// The claim this driver makes - util/usb's controller contract, checked
/// at the part that carries the block. It is made through a template
/// because a bare `static_assert(UsbController<Usbd<>>)` would
/// INSTANTIATE the controller in every translation unit that includes
/// this header, and on a package that has no USBD (the CH32V203F8's
/// TSSOP20) the first thing it would meet is the refusal above - while a
/// header of this stratum has to compile on every part of the family.
template <bool present>
struct UsbdContract {
    static_assert(!present || UsbController<Usbd<>>,
                  "brio Usbd: this driver no longer realizes util/usb's UsbController");
};

static_assert(sizeof(UsbdContract<device::has_usbd>) > 0);

} // namespace brio
