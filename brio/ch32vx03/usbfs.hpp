/*
 * usbfs.hpp
 *
 * The USB full-speed HOST/DEVICE controller of RM ch. 23 - USBFS, which
 * WCH's datasheets also call OTG_FS - in DEVICE mode, realizing
 * util/usb's UsbController at the packet: the CH32V303's one full-speed
 * controller, and the CH32V203's second one beside the device controller
 * of usb.hpp.
 *
 * WHAT THIS BLOCK IS. WCH's own design and nothing of ST's: a register
 * file of BYTES at 0x50000000 on the HB bus, eight endpoint numbers with
 * a mode nibble each - packed two to a byte, in an order the chapter
 * spells out register by register (usbfs_mod_slot) - a DMA address, a
 * transmit length and TWO RESPONSE BYTES per number (one for IN, one for
 * OUT), and the endpoint buffers in the program's own SRAM, which the
 * block reaches by DMA. One flag byte, and beside it one status byte
 * naming the TOKEN and the ENDPOINT of the transfer that completed, and
 * ONE receive-length register for every endpoint. It shares nothing with
 * the device controller of ch. 21 - other registers, other buffers,
 * another vector - and a part may carry both on different pads (the
 * CH32V203C8 does); which parts carry this one, and on which pads, is
 * the part table's (`device::has_usbfs`, `device::usbfs_dm_port` and its
 * three siblings).
 *
 * THE BUFFERS ARE THE PROGRAM'S RAM, AND THIS DRIVER OWNS THEM. A static
 * pool, four-byte aligned because 23.2.2.6 asks every buffer address to
 * be, handed out in the order the stack claims endpoints (the RP2040's
 * method): endpoint zero's 64 bytes by init(), since that endpoint sends
 * and receives through ONE buffer (23.2.2); then 64 bytes a direction.
 * A number with BOTH directions takes 128 bytes in one piece, the OUT
 * half first and the IN half 64 above it - table 23-4's single-buffered
 * row with both enables set, and the vendor's own sequence loads an IN
 * packet at +64 exactly when the number receives too. The stack claims
 * one direction at a time, so the second claim of a number lands beside
 * the first when the first was the last thing handed out, and otherwise
 * both halves move to a fresh 128 bytes; that happens inside a
 * configuration, before anything is armed, and a claim that would move
 * an ARMED half is refused. `pool_bytes` is a template parameter because
 * the memory is the program's: `Usbfs<>` reserves 960 bytes (endpoint
 * zero and all seven numbers both ways, so no claim ever fails for want
 * of memory) and a CDC ACM port spends 256 of them.
 *
 * THE BUFFER REGISTER HOLDS AN OFFSET. R32_UEPn_DMA is written with the
 * buffer's whole address, and it reads back the address's LOW BITS alone
 * - 0x5E0 for a buffer at 0x2000_05E0, measured on the CH32V303VC, with
 * reset values of up to seventeen bits in the numbers nothing wrote -
 * which is also how WCH's copy path reads it (back, plus 0x20000000). So
 * a buffer must lie in the first 64 KB of SRAM, which every part's
 * linked SRAM does (the static_assert below), and this driver never
 * reads the register back: it keeps its own copy of every buffer's
 * place.
 *
 * ONE RX_LEN AND ONE INT_ST FOR EVERY ENDPOINT, AND THE BIT THAT MAKES
 * THAT SAFE. The length of the last reception and the token and endpoint
 * of the last transfer are single registers, overwritten by the next
 * transfer. RB_UC_INT_BUSY (23.2.1.1) is what keeps them standing: with
 * it set the block answers NAK to every transaction while UIF_TRANSFER
 * is up, so the two registers describe exactly one transfer until the
 * handler clears the flag - MEASURED: with the vector held off for 20 ms
 * under a host's pour the status byte changed once, for the first packet,
 * and never again, where with the bit clear it changed 248 to 297 times,
 * each change a packet taken over one nobody had read. init() sets it
 * and every other line of take_events() rests on it: the status and the
 * length are read first, the endpoint's response is changed, and the
 * flag is cleared LAST. `auto_pause(false)` exists for one reason - a
 * suite staging what the pause prevents.
 *
 * A SETUP IS NOT JUDGED BY ITS ENDPOINT FIELD. At a SETUP the status
 * byte's endpoint number is not the setup's: on the CH32V303VC it read 6
 * at the first SETUP after a bus reset - the field holds what the last IN
 * or OUT left, or its reset garbage - and a driver that took it for the
 * endpoint dropped every SETUP of an enumeration. WCH's own sequence
 * never looks at it for a SETUP. Every SETUP is endpoint zero's here, and
 * RX_LEN beside it is not the setup's length either (1022, measured): the
 * eight bytes are copied whatever it says.
 *
 * AND THE RESPONSE IS NOT CHANGED BY THE SILICON: after a completed OUT
 * the endpoint's register still answers ACK (measured, with the vector
 * held off), and only the pause above keeps the block from taking the
 * next packet into the same buffer - where ST's block drops its status to
 * NAK by itself. WCH's sequence sets NAK itself after every IN on a data
 * endpoint, and after a bulk OUT leaves ACK and re-points the buffer at
 * a ring of its own. So take_events() sets NAK on every endpoint it
 * reports a completion for, before it clears the flag, and a packet goes
 * out or comes in once per submit: the stack's own contract.
 *
 * THE DATA TOGGLE IS THE DRIVER'S, AND ITS ONLY COPY IS THE REGISTER.
 * T_TOG and R_TOG (23.2.2.8, 23.2.2.9) are the PID to send and the PID
 * to expect. The AUTO_TOG bits would flip them in hardware on endpoints
 * 1..7 and endpoint zero has none, so this driver flips them itself, on
 * every endpoint the same way and in the register, with no second copy
 * to fall out of step: DATA0 at a claim and after a cleared halt, DATA1
 * both ways on endpoint zero at every SETUP (chapter 9's rule, as the
 * RP2040's driver keeps it), and flipped at every IN the host took and at
 * every OUT whose PID matched. An OUT whose PID did NOT match (RB_UIS_
 * TOG_OK clear) is a packet the host is sending AGAIN because it never
 * saw the device's acknowledgement: the silicon acknowledges it
 * (measured: a staged mismatch cost the stream exactly one packet and the
 * host did not send it again), and this driver drops it and leaves the
 * endpoint armed - `toggle_errors()` counts them. The stack never sees a
 * PID.
 *
 * A SETUP IS TAKEN WHATEVER ENDPOINT ZERO'S RESPONSE SAYS - measured:
 * this driver leaves endpoint zero at NAK both ways between control
 * transfers and every SETUP of every enumeration landed. At a SETUP both
 * directions of endpoint zero go to NAK with DATA1 and the eight bytes
 * are COPIED OUT of the shared buffer at once, because the answer the
 * stack writes next lands in that same buffer.
 *
 * THE PULL-UP IS THIS BLOCK'S, AND IT IS ALSO THE DEVICE ENABLE.
 * MASK_UC_SYS_CTRL (table 23-2) is one field for both: 00 the device
 * function off and no pull-up, 1x the device function on with the
 * internal 1.5 kOhm on D+. connect() writes that field and nothing else,
 * init() leaves it at 00, so the host sees no device until the stack is
 * ready to answer - and the transceiver itself (RB_UD_PORT_EN) and the
 * pull-downs' release (RB_UD_PD_DIS, for host mode) are init()'s.
 *
 * AND A DETACHED BLOCK REPORTS NOTHING. With the pull-up down the bus is
 * SE0 - the host's two pull-downs and nobody's pull-up - and the
 * transceiver, still on, calls that a bus reset (measured: one within
 * 100 ms of every detach), which the stack would take for an attachment.
 * take_events() acknowledges and drops every flag while connect(false)
 * stands.
 *
 * AND connect() IS WHERE THIS CONTROLLER COUNTS ITSELF A BUS MASTER, for
 * the reason usb.hpp gives for the other one: its DMA reaches the pool
 * over the bus matrix whenever the host speaks, and in a sleep of any
 * depth on this family no master but the core gets a cycle
 * (ch32vx03/bus_activity.hpp). Measured on this block with the core in a
 * bare wfi between packets: the host's OUT packets arrived whole and
 * none was lost, but the IN packets the block read out of RAM carried
 * zeros where the program's bytes should have been - a third of the
 * bytes of a counting stream broken, with no error at either end - and
 * an enumeration got through SET_ADDRESS and stopped at the descriptors
 * the block had to read out of RAM. So an attached controller
 * holds one count from the pull-up to the detach, the platform's idle()
 * does not sleep over it and a sleep site refuses to arm over it.
 *
 * THE PADS NEED NO PORT CLOCK, and are left floating inputs anyway.
 * Measured: with the gate of the pads' port shut the host enumerated the
 * device, and WCH's own sequence touches no GPIO - where the device
 * controller of ch. 21 cannot work so (usb.hpp). init() still opens the
 * port and leaves the two pads floating inputs, their reset state, so no
 * GPIO configuration a program left behind stands between the
 * transceiver and the wire.
 *
 * ONE CLOCK RATE, AND A FLOOR UNDER THE BUS. The controller wants 48 MHz
 * and the tree's USBPRE makes it (clock.hpp): refused at compile time
 * under a static Clock that cannot, and answered with false by init()
 * under a dynamic one whose rate in force cannot. The datasheet's own
 * note under its clock tree asks the CPU clock to be 48, 96 or 144 MHz
 * whenever USB is used; measured on the CH32V303VC with HCLK divided
 * under a 96 MHz PLL, this block enumerated and carried a pour with no
 * loss at 96, 48, 24 and 12 MHz, enumerated and lost bytes of the pour -
 * with no overflow flagged - at 6, and did neither at 1.5 - so
 * `usbfs_min_hclk_hz` is 12 MHz, refused below at compile time.
 *
 * THE WAKE-UP LINE. Table 9-3 gives this controller's wake-up event to
 * EXTI line 20 on every class of this stratum, and on the CH32V30x_D8 it
 * names line 18 for "USBD/USBFSOTG" as well. MEASURED on the CH32V303VC:
 * a host's resume raises line 18, once, and line 20 never - so
 * `wakeup_line` and `wakeup_irq` are 18 and vector 58 on that class and
 * table 9-3's 20 (vector 60) on the CH32V203's. `arm_wakeup()` arms the
 * line and `wakeup_isr()` is its body. The stack uses neither: a
 * suspended controller keeps its bus-master count.
 *
 * THE ISR BODY IS THE STACK'S: `UsbDevice<Usbfs<>, ...>::isr()` is what
 * the app binds to `usbfs_handler`, and it drains take_events(). One
 * vector carries every event of this block in device mode.
 *
 * WHAT IS NOT COVERED YET, each with its reason:
 *  - the HOST half of the chapter (23.2.3) and OTG's SRP and HNP: brio
 *    has no host side and will not (docs/design/usb.md); the OTG
 *    registers are, besides, the CH32V305's and CH32V307's alone.
 *  - the DOUBLE BUFFER (RB_UEPn_BUF_MOD, table 23-4) and ISOCHRONOUS
 *    endpoints (the "no response" codes, endpoint 3's 1023 bytes): the
 *    contract admits one packet in flight per endpoint and direction, and
 *    no class here streams.
 *  - endpoints 8..15, which 23.2.2 maps onto the registers of 1..7: a
 *    number above 7 would share the resources of the one it maps onto.
 *  - LOW SPEED (RB_UC_LOW_SPEED, RB_UD_LOW_SPEED) and the 1-WIRE MODE
 *    (RB_U_1WIRE_MODE, the CH32V303's on some lots): this stratum is a
 *    full-speed device on two wires.
 *  - the NAK INTERRUPT (RB_UIE_DEV_NAK): it would fire on every NAK this
 *    device answers - every poll of an idle IN endpoint - and nothing
 *    above the packet wants that; it is left disabled.
 *  - the suspend's low-power half: what a suspended controller would let
 *    the core sleep through is a question for a meter, and the count is
 *    deliberately the attachment's and not the traffic's.
 *  - the two general-purpose bits (RB_UDA_GP_BIT, RB_UD_GP_BIT): scratch
 *    a program has RAM for. set_address() carries the first through.
 *
 * The numbers behind every measured statement above are in
 * docs/ch32vx03/usbfs.md.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <span>

#include "ch32vx03/bus_activity.hpp"
#include "ch32vx03/clock.hpp"
#include "ch32vx03/device.hpp"
#include "ch32vx03/exti.hpp"
#include "ch32vx03/pfic.hpp"
#include "ch32vx03/pin.hpp"
#include "util/usb/device.hpp"

namespace brio {

// ---- the registers (RM 23.2.1, 23.2.2) --------------------------------------
/// The device-mode view of the block: bytes and halfwords at their own
/// offsets, which is how the chapter's table 23-1 and 23-3 lay them out.
/// In host mode some of the same addresses mean other things (23.2.3);
/// this driver never selects host mode.
struct UsbfsRegs {
    volatile uint8_t CTRL;          ///< 0x00 R8_USB_CTRL
    volatile uint8_t UDEV_CTRL;     ///< 0x01 R8_UDEV_CTRL
    volatile uint8_t INT_EN;        ///< 0x02 R8_USB_INT_EN
    volatile uint8_t DEV_AD;        ///< 0x03 R8_USB_DEV_AD
    uint8_t RESERVED0;              ///< 0x04
    volatile uint8_t MIS_ST;        ///< 0x05 R8_USB_MIS_ST
    volatile uint8_t INT_FG;        ///< 0x06 R8_USB_INT_FG
    volatile uint8_t INT_ST;        ///< 0x07 R8_USB_INT_ST
    volatile uint16_t RX_LEN;       ///< 0x08 R16_USB_RX_LEN
    uint16_t RESERVED1;             ///< 0x0a
    /// 0x0c..0x0f: R8_UEP4_1_MOD, R8_UEP2_3_MOD, R8_UEP5_6_MOD and
    /// R8_UEP7_MOD, in that order (usbfs_mod_slot says which nibble is
    /// whose).
    volatile uint8_t UEP_MOD[4];
    volatile uint32_t UEP_DMA[8];   ///< 0x10..0x2c R32_UEPn_DMA
    struct Endpoint {
        volatile uint16_t T_LEN;    ///< R16_UEPn_T_LEN
        volatile uint8_t TX_CTRL;   ///< R8_UEPn_TX_CTRL: the answer to IN
        volatile uint8_t RX_CTRL;   ///< R8_UEPn_RX_CTRL: the answer to OUT
    } UEP[8];                       ///< 0x30..0x4c
    uint32_t RESERVED2;             ///< 0x50
    /// 0x54, 0x58: R32_USB_OTG_CR and R32_USB_OTG_SR, which 23.2.1.8 gives
    /// to the CH32V305 and CH32V307 alone - named so the layout is whole,
    /// and never touched here.
    uint32_t OTG_CR;
    uint32_t OTG_SR;
};

static_assert(offsetof(UsbfsRegs, RX_LEN) == 0x08 && offsetof(UsbfsRegs, UEP_MOD) == 0x0c &&
                  offsetof(UsbfsRegs, UEP_DMA) == 0x10 && offsetof(UsbfsRegs, UEP) == 0x30 &&
                  offsetof(UsbfsRegs, OTG_CR) == 0x54,
              "brio Usbfs: the register view must lie where tables 23-1 and 23-3 put it");

inline UsbfsRegs* usbfs() { return reinterpret_cast<UsbfsRegs*>(usbfs_base); }

/// R8_USB_CTRL (23.2.1.1). The system-control field and the host bit
/// together are table 23-2's combinations.
inline constexpr uint8_t usbfs_uc_dma_en         = 1U << 0;   ///< the DMA and its interrupt
inline constexpr uint8_t usbfs_uc_clr_all        = 1U << 1;   ///< the FIFO and the flags held clear
inline constexpr uint8_t usbfs_uc_reset_sie      = 1U << 2;   ///< the protocol engine held in reset
inline constexpr uint8_t usbfs_uc_int_busy       = 1U << 3;   ///< NAK everything while UIF_TRANSFER stands
inline constexpr uint8_t usbfs_uc_sys_ctrl_mask  = 0x3U << 4;
inline constexpr uint8_t usbfs_uc_dev_en_no_pu   = 1U << 4;   ///< 01: the device function, an external pull-up
inline constexpr uint8_t usbfs_uc_dev_pu_en      = 1U << 5;   ///< 1x: the device function and the internal 1.5k
inline constexpr uint8_t usbfs_uc_low_speed      = 1U << 6;
inline constexpr uint8_t usbfs_uc_host_mode      = 1U << 7;

/// R8_UDEV_CTRL (23.2.2.1)
inline constexpr uint8_t usbfs_ud_port_en        = 1U << 0;   ///< the transceiver
inline constexpr uint8_t usbfs_ud_gp_bit         = 1U << 1;
inline constexpr uint8_t usbfs_ud_low_speed      = 1U << 2;
inline constexpr uint8_t usbfs_ud_dm_pin         = 1U << 4;   ///< read-only: the level on D-
inline constexpr uint8_t usbfs_ud_dp_pin         = 1U << 5;   ///< read-only: the level on D+
inline constexpr uint8_t usbfs_ud_pd_dis         = 1U << 7;   ///< 1: the 15k pull-downs OFF (reset value)

/// R8_USB_INT_EN (23.2.1.2). Bit 7 is reserved in the manual and named
/// "device SOF" in the vendor's own header (the file's frame counter).
inline constexpr uint8_t usbfs_uie_bus_rst       = 1U << 0;
inline constexpr uint8_t usbfs_uie_transfer      = 1U << 1;
inline constexpr uint8_t usbfs_uie_suspend       = 1U << 2;
inline constexpr uint8_t usbfs_uie_hst_sof       = 1U << 3;   ///< host mode
inline constexpr uint8_t usbfs_uie_fifo_ov       = 1U << 4;
inline constexpr uint8_t usbfs_u_1wire_mode      = 1U << 5;
inline constexpr uint8_t usbfs_uie_dev_nak       = 1U << 6;
inline constexpr uint8_t usbfs_uie_dev_sof       = 1U << 7;

/// R8_USB_DEV_AD (23.2.1.3)
inline constexpr uint8_t usbfs_uda_addr_mask     = 0x7FU;
inline constexpr uint8_t usbfs_uda_gp_bit        = 1U << 7;

/// R8_USB_MIS_ST (23.2.1.4), device-mode bits.
inline constexpr uint8_t usbfs_ums_suspend       = 1U << 2;
inline constexpr uint8_t usbfs_ums_bus_reset     = 1U << 3;
inline constexpr uint8_t usbfs_ums_r_fifo_rdy    = 1U << 4;
inline constexpr uint8_t usbfs_ums_sie_free      = 1U << 5;

/// R8_USB_INT_FG (23.2.1.5): the five flags are write-one-to-clear, the
/// three above them read-only mirrors of the status.
inline constexpr uint8_t usbfs_uif_bus_rst       = 1U << 0;
inline constexpr uint8_t usbfs_uif_transfer      = 1U << 1;
inline constexpr uint8_t usbfs_uif_suspend       = 1U << 2;
inline constexpr uint8_t usbfs_uif_hst_sof       = 1U << 3;
inline constexpr uint8_t usbfs_uif_fifo_ov       = 1U << 4;
inline constexpr uint8_t usbfs_uif_all           = 0x1FU;
inline constexpr uint8_t usbfs_u_sie_free        = 1U << 5;
inline constexpr uint8_t usbfs_u_tog_ok          = 1U << 6;
inline constexpr uint8_t usbfs_u_is_nak          = 1U << 7;

/// R8_USB_INT_ST (23.2.1.6). Token 01 is "reserved" in the manual and the
/// start of a frame in the vendor's header.
inline constexpr uint8_t usbfs_uis_endp_mask     = 0x0FU;
inline constexpr uint8_t usbfs_uis_token_mask    = 0x30U;
inline constexpr uint8_t usbfs_uis_token_out     = 0x00U;
inline constexpr uint8_t usbfs_uis_token_sof     = 0x10U;
inline constexpr uint8_t usbfs_uis_token_in      = 0x20U;
inline constexpr uint8_t usbfs_uis_token_setup   = 0x30U;
inline constexpr uint8_t usbfs_uis_tog_ok        = 1U << 6;
inline constexpr uint8_t usbfs_uis_is_nak        = 1U << 7;

/// One endpoint's MODE NIBBLE (23.2.2.2..5), the receive and transmit
/// enables and the double-buffer bit this driver never sets.
inline constexpr uint8_t usbfs_mod_rx_en         = 0x8U;
inline constexpr uint8_t usbfs_mod_tx_en         = 0x4U;
inline constexpr uint8_t usbfs_mod_buf_mod       = 0x1U;

/// R8_UEPn_TX_CTRL and R8_UEPn_RX_CTRL (23.2.2.8, 23.2.2.9): the same
/// layout both ways - the response in the low two bits, the toggle above
/// it, the automatic toggle above that (reserved on endpoint zero).
inline constexpr uint8_t usbfs_uep_res_mask      = 0x03U;
inline constexpr uint8_t usbfs_uep_res_ack       = 0x00U;
inline constexpr uint8_t usbfs_uep_res_none      = 0x01U;   ///< isochronous: no handshake
inline constexpr uint8_t usbfs_uep_res_nak       = 0x02U;
inline constexpr uint8_t usbfs_uep_res_stall     = 0x03U;
inline constexpr uint8_t usbfs_uep_tog           = 1U << 2;  ///< 1: DATA1
inline constexpr uint8_t usbfs_uep_auto_tog      = 1U << 3;

/// Where endpoint n's mode nibble lives: which of the four mode bytes and
/// which half of it. 1 and 4 share the first byte (1 in the high half), 2
/// and 3 the second (3 high), 5 and 6 the third (6 high), 7 has the
/// fourth to itself. `reg` 0xFF for a number with no nibble (0, and 8 up).
struct UsbfsModSlot {
    uint8_t reg;
    uint8_t shift;
};

constexpr UsbfsModSlot usbfs_mod_slot(uint8_t n) {
    switch (n) {
        case 1: return {0, 4};
        case 4: return {0, 0};
        case 2: return {1, 0};
        case 3: return {1, 4};
        case 5: return {2, 0};
        case 6: return {2, 4};
        case 7: return {3, 0};
        default: return {0xFF, 0};
    }
}

/// Endpoint zero's one buffer, shared by both directions (23.2.2).
inline constexpr uint16_t usbfs_ep0_bytes = 64;
/// What one direction of endpoints 1..7 is given: the full-speed maximum,
/// which also satisfies 23.2.2.6's note that a receive buffer be at least
/// min(the largest packet + 2, 64) bytes long.
inline constexpr uint16_t usbfs_half_bytes = 64;
/// The pool that never refuses a claim: endpoint zero and all seven
/// numbers both ways.
inline constexpr uint16_t usbfs_pool_all = usbfs_ep0_bytes + 7u * 2u * usbfs_half_bytes;
/// What a buffer register can reach, read the way the vendor reads it: an
/// offset into SRAM.
inline constexpr uint32_t usbfs_dma_reach_bytes = 64UL * 1024UL;

/// THE BUS FLOOR (the file header): the lowest HCLK this block was
/// measured carrying both an enumeration and a whole pour at - 12 MHz on
/// the CH32V303VC, under a PLL whose 48 MHz USBPRE takes whole; at 6 MHz
/// it enumerated and lost bytes of a pour in three runs of seven, and at
/// 1.5 MHz it did neither.
inline constexpr uint32_t usbfs_min_hclk_hz = 12'000'000UL;

/**
 * The host/device controller in device mode.
 *
 * `pool_bytes` is how much RAM this program gives the endpoint buffers
 * (the file header): a whole number of 64-byte halves, endpoint zero's
 * first. 960 by default, which no set of claims can exhaust.
 */
template <uint16_t pool_bytes = usbfs_pool_all>
struct Usbfs {
    static_assert(device::has_usbfs,
                  "brio Usbfs: this part has no USB host/device controller (parts/<part>.hpp)");
    static_assert(pool_bytes >= usbfs_ep0_bytes,
                  "brio Usbfs: the pool must hold endpoint zero's 64 bytes, which init() claims");
    static_assert(pool_bytes < usbfs_ep0_bytes || pool_bytes % usbfs_half_bytes == 0u,
                  "brio Usbfs: the pool is handed out 64 bytes at a time, so it is a whole number of them");
    static_assert(device::sram_bytes <= usbfs_dma_reach_bytes,
                  "brio Usbfs: the vendor reads a buffer register back as a 16-bit offset into SRAM, so "
                  "the pool must lie in its first 64 KB - which a part linking more SRAM than that "
                  "could not promise without measuring the register first");
    static_assert(!device::has_usbfs || (pad_bonded(Pad{device::usbfs_dm_port, device::usbfs_dm_pin}) &&
                                         pad_bonded(Pad{device::usbfs_dp_port, device::usbfs_dp_pin})),
                  "brio Usbfs: a part with this controller brings out its two pads (parts/<part>.hpp)");

    Usbfs() = delete;

    static constexpr uint8_t endpoint_count = 8;
    static constexpr uint16_t max_packet = usb_full_speed_packet;
    static constexpr uint16_t pool_size = pool_bytes;
    /// The EXTI line this controller's wake-up event arrives on, and the
    /// vector that line interrupts on. Table 9-3 names line 20 on every
    /// class and line 18 as well on the CH32V30x_D8; MEASURED on the
    /// CH32V303VC, a host's resume raises line 18 (vector 58) and never
    /// line 20, so the class decides. The CH32V203's line is table 9-3's.
    static constexpr bool wakes_on_line18 = device::device_class == DeviceClass::v30x_d8;
    static constexpr uint8_t wakeup_line =
        wakes_on_line18 ? Exti::line_usbd_wakeup : Exti::line_usbfs_wakeup;
    static constexpr Irq wakeup_irq = wakes_on_line18 ? Irq::usb_wakeup : Irq::usbfs_wakeup;

    static UsbfsRegs& regs() { return *usbfs(); }

    /**
     * Bring the block up: the pads, the HB gate, the protocol engine
     * reset and released, every endpoint parked at NAK, endpoint zero
     * laid out over its buffer, the interrupts of a device (a bus reset,
     * a completed transfer, suspend and resume, a FIFO overflow), the
     * automatic pause, the DMA and the transceiver on - and the device
     * function and its pull-up still OFF: connect() is the stack's, and
     * the host must not see a device until the program is ready to
     * answer it. The counters start again here: they belong to this
     * bring-up of the block, not to the program.
     *
     * The clock is checked where it can be (the file header): at compile
     * time under a static Clock, at run time - false, nothing touched -
     * under a dynamic one.
     */
    template <typename C>
    static bool init(C clock) {
        (void)clock;
        if constexpr (C::is_static) {
            static_assert(C::usb_hz == usb_required_hz,
                          "brio Usbfs: this controller must be fed 48 MHz, and only a PLL rate of 48, "
                          "96 or 144 MHz divides to it (clock.hpp's USBPRE)");
            static_assert(C::hz >= usbfs_min_hclk_hz,
                          "brio Usbfs: HCLK below the bus floor this block was measured at "
                          "(usbfs_min_hclk_hz: 12 MHz carried an enumeration and a whole pour, "
                          "6 MHz lost bytes of the pour, 1.5 MHz carried neither)");
        } else {
            if (C::usb_hz() != usb_required_hz || C::hz() < usbfs_min_hclk_hz) {
                return false;
            }
        }

        connect(false);   // a bring-up over an attachment gives its count back first
        float_pad<device::usbfs_dm_port>(device::usbfs_dm_pin);
        float_pad<device::usbfs_dp_port>(device::usbfs_dp_pin);
        Rcc::enable(Bus::hb, rcc_hb_usbfs);

        // The engine held in reset with the FIFO and the flags cleared -
        // which is also the register's reset value - for at least ten
        // microseconds, the vendor's figure (the chapter gives none),
        // then released with the device function off.
        regs().CTRL = static_cast<uint8_t>(usbfs_uc_reset_sie | usbfs_uc_clr_all);
        settle(10);
        regs().CTRL = 0;
        regs().INT_FG = usbfs_uif_all;
        regs().UDEV_CTRL = usbfs_ud_pd_dis;
        regs().DEV_AD = 0;

        for (uint8_t n = 1; n < endpoint_count; ++n) {
            release_number(n);
        }
        for (uint8_t r = 0; r < 4u; ++r) {
            regs().UEP_MOD[r] = 0;
        }
        next_ = usbfs_ep0_bytes;
        setup_ = UsbSetup{};
        out_len_[0] = 0;
        errors_ = 0;
        overruns_ = 0;
        toggle_errors_ = 0;
        frames_ = 0;
        for (uint8_t n = 0; n < endpoint_count; ++n) {
            received_[n] = 0;
        }
        lay_out_control();

        regs().INT_EN = static_cast<uint8_t>(usbfs_uie_bus_rst | usbfs_uie_transfer | usbfs_uie_suspend |
                                             usbfs_uie_fifo_ov);
        regs().CTRL = static_cast<uint8_t>(usbfs_uc_int_busy | usbfs_uc_dma_en);
        regs().UDEV_CTRL = static_cast<uint8_t>(usbfs_ud_pd_dis | usbfs_ud_port_en);
        Pfic::enable(Irq::usbfs);
        return true;
    }

    /// Give the block back: the wake-up line and the vector off, the
    /// device function and its pull-up off (the bus-master count with
    /// them), the transceiver off, the engine held in reset - the
    /// register's reset value - and the HB gate shut.
    static void release() {
        (void)arm_wakeup(false);
        connect(false);
        Pfic::disable(Irq::usbfs);
        regs().INT_EN = 0;
        regs().UDEV_CTRL = usbfs_ud_pd_dis;
        regs().CTRL = static_cast<uint8_t>(usbfs_uc_reset_sie | usbfs_uc_clr_all);
        Rcc::disable(Bus::hb, rcc_hb_usbfs);
    }

    // ---- the UsbController contract ---------------------------------------

    /// The device function and the internal pull-up on D+, which is one
    /// field here (the file header) - and the bus-master count, held from
    /// the pull-up to the detach.
    static void connect(bool on) {
        const uint8_t ctrl = static_cast<uint8_t>(regs().CTRL & ~usbfs_uc_sys_ctrl_mask);
        regs().CTRL = on ? static_cast<uint8_t>(ctrl | usbfs_uc_dev_pu_en) : ctrl;
        if (on != attached_) {
            attached_ = on;
            if (on) {
                BusActivity::entered();
            } else {
                BusActivity::left();
            }
        }
    }

    /// The device address, the general-purpose bit beside it kept.
    static void set_address(uint8_t address) {
        regs().DEV_AD = static_cast<uint8_t>((regs().DEV_AD & usbfs_uda_gp_bit) |
                                             (address & usbfs_uda_addr_mask));
    }

    /**
     * Claim one direction of one endpoint: its half of the pool, its mode
     * nibble, its buffer register, and its response parked at NAK with
     * DATA0 - armed by submit_in or submit_out and not before.
     *
     * False for endpoint zero (init()'s), for a number past 7, for a
     * packet larger than 64 bytes, for a type other than bulk and
     * interrupt, when the pool is spent, and when the claim would move
     * the other half of its number while that half is armed.
     */
    static bool configure_endpoint(uint8_t address, UsbEndpointType type, uint16_t max) {
        const uint8_t n = usb_ep_number(address);
        const bool in = usb_ep_is_in(address);
        if (n == 0u || n >= endpoint_count || max == 0u || max > max_packet ||
            (type != UsbEndpointType::bulk && type != UsbEndpointType::interrupt)) {
            return false;
        }
        const uint8_t mine = in ? claim_in : claim_out;
        const uint8_t other = in ? claim_out : claim_in;
        if ((claimed_[n] & mine) != 0u) {
            // Claimed again (a class configuring twice): the same buffer,
            // back to DATA0 and NAK.
            (in ? max_in_ : max_out_)[n] = static_cast<uint8_t>(max);
            park(n, in);
            return true;
        }

        if ((claimed_[n] & other) == 0u) {
            if (next_ + usbfs_half_bytes > pool_bytes) {
                return false;
            }
            slot_[n] = next_;
            next_ = static_cast<uint16_t>(next_ + usbfs_half_bytes);
        } else {
            // The number's second direction: the two halves must be one
            // piece, the OUT half first. Beside the first when the first
            // was the last thing handed out - the IN half moving up by 64
            // when it was the one claimed first - and otherwise both to a
            // fresh 128 bytes.
            const bool beside = slot_[n] + usbfs_half_bytes == next_ &&
                                next_ + usbfs_half_bytes <= pool_bytes;
            const bool other_moves = !beside || !in;
            if (other_moves && armed(n, !in)) {
                return false;
            }
            if (beside) {
                next_ = static_cast<uint16_t>(next_ + usbfs_half_bytes);
            } else {
                if (next_ + 2u * usbfs_half_bytes > pool_bytes) {
                    return false;
                }
                slot_[n] = next_;
                next_ = static_cast<uint16_t>(next_ + 2u * usbfs_half_bytes);
            }
        }

        claimed_[n] = static_cast<uint8_t>(claimed_[n] | mine);
        (in ? max_in_ : max_out_)[n] = static_cast<uint8_t>(max);
        regs().UEP_DMA[n] = pool_address(slot_[n]);
        write_mode(n);
        park(n, in);
        return true;
    }

    /// The same claim with the number, the type and the packet size
    /// known at compile time: what the run-time form answers with false
    /// is a compile error here, and so is a pool that could not hold this
    /// one claim beside endpoint zero's buffer even with nothing else
    /// claimed.
    template <uint8_t address, UsbEndpointType type, uint16_t max>
    static bool configure_endpoint() {
        static_assert(usb_ep_number(address) >= 1u && usb_ep_number(address) < endpoint_count,
                      "brio Usbfs: endpoints 1..7 are claimed (zero is init()'s, and 23.2.2 maps 8..15 "
                      "onto the registers of 1..7, which this driver does not)");
        static_assert(max >= 1u && max <= max_packet,
                      "brio Usbfs: a full-speed packet of 1..64 bytes (the isochronous 1023 of endpoint 3 "
                      "is not offered)");
        static_assert(type == UsbEndpointType::bulk || type == UsbEndpointType::interrupt,
                      "brio Usbfs: bulk and interrupt endpoints only - endpoint zero is the one control "
                      "endpoint, and isochronous transfers are not offered");
        static_assert(pool_bytes >= usbfs_ep0_bytes + usbfs_half_bytes,
                      "brio Usbfs: the pool cannot hold endpoint zero's buffer and this claim's half");
        return configure_endpoint(address, type, max);
    }

    /// Every endpoint but zero released and its memory given back - the
    /// stack's answer to a bus reset and to a configuration the host takes
    /// away.
    static void deconfigure_endpoints() {
        for (uint8_t n = 1; n < endpoint_count; ++n) {
            release_number(n);
        }
        for (uint8_t r = 0; r < 4u; ++r) {
            regs().UEP_MOD[r] = 0;
        }
        next_ = usbfs_ep0_bytes;
    }

    /// The halt of one direction. Clearing it restarts the toggle at
    /// DATA0 on every endpoint but zero (chapter 9); zero's halt is
    /// cleared by the next SETUP.
    static void stall(uint8_t address, bool on) {
        const uint8_t n = usb_ep_number(address);
        if (n >= endpoint_count) {
            return;
        }
        volatile uint8_t& ctrl = usb_ep_is_in(address) ? regs().UEP[n].TX_CTRL : regs().UEP[n].RX_CTRL;
        if (on) {
            ctrl = static_cast<uint8_t>((ctrl & usbfs_uep_tog) | usbfs_uep_res_stall);
        } else if (n == 0u) {
            ctrl = static_cast<uint8_t>((ctrl & usbfs_uep_tog) | usbfs_uep_res_nak);
        } else {
            ctrl = usbfs_uep_res_nak;
        }
    }

    static bool stalled(uint8_t address) {
        const uint8_t n = usb_ep_number(address);
        if (n >= endpoint_count) {
            return false;
        }
        const uint8_t ctrl = usb_ep_is_in(address) ? regs().UEP[n].TX_CTRL : regs().UEP[n].RX_CTRL;
        return (ctrl & usbfs_uep_res_mask) == usbfs_uep_res_stall;
    }

    /// One packet copied into the endpoint's buffer, its length written
    /// and the endpoint armed to answer the next IN token with it, with
    /// the PID the register holds. An empty span is the zero-length
    /// packet.
    static bool submit_in(uint8_t number, std::span<const uint8_t> data) {
        if (number >= endpoint_count || data.size() > max_packet) {
            return false;
        }
        if (number != 0u && (!claimed(number, true) || data.size() > max_in_[number])) {
            return false;
        }
        volatile uint8_t* dst = &pool_[tx_offset(number)];
        for (size_t i = 0; i < data.size(); ++i) {
            dst[i] = data[i];
        }
        regs().UEP[number].T_LEN = static_cast<uint16_t>(data.size());
        respond(regs().UEP[number].TX_CTRL, usbfs_uep_res_ack);
        return true;
    }

    /// Arm the reception of one packet of at most `max` - which the
    /// buffer is already large enough for: the endpoint's response set to
    /// ACK, the expected PID left as the register holds it.
    static bool submit_out(uint8_t number, uint16_t max) {
        if (number >= endpoint_count) {
            return false;
        }
        const uint16_t room = number == 0u ? max_packet : max_out_[number];
        if ((number != 0u && !claimed(number, false)) || max > room) {
            return false;
        }
        respond(regs().UEP[number].RX_CTRL, usbfs_uep_res_ack);
        return true;
    }

    /// What the last OUT packet on that endpoint carried: the bytes in
    /// the buffer the block wrote them into, with the length the block
    /// reported for THAT transfer. Valid until the endpoint is armed
    /// again - and on endpoint zero, whose one buffer both directions
    /// share, until the next submit_in there too.
    static std::span<const uint8_t> out_data(uint8_t number) {
        if (number >= endpoint_count || (number != 0u && !claimed(number, false))) {
            return {};
        }
        return {const_cast<const uint8_t*>(&pool_[rx_offset(number)]), out_len_[number]};
    }

    static UsbSetup setup() { return setup_; }

    /**
     * What happened since the last call, acknowledged.
     *
     * A bus reset first, then a completed transfer, then suspend or
     * resume, then a FIFO overflow, in a bounded loop. A transfer is read
     * (the status, the length) and answered (the response to NAK, the
     * toggle flipped) BEFORE its flag is cleared, which is what the
     * automatic pause is for. AND THE LOOP STOPS AT THE FIRST TRANSFER ON
     * ENDPOINT ZERO: the control machine above takes the SETUP before the
     * completions, so a completion and a SETUP that followed it must not
     * reach it in one batch - the next pass of the vector brings the
     * rest, in the order the bus made them.
     */
    static UsbEvents take_events() {
        UsbEvents ev;
        if (!attached_) {
            // A DETACHED BLOCK REPORTS NOTHING. With the pull-up down the
            // bus is SE0 - the host's two pull-downs and nobody's pull-up
            // - and the transceiver, still on, calls that a bus reset
            // (measured: one within 100 ms of every detach), which the
            // stack would take for an attachment. Acknowledged, dropped.
            regs().INT_FG = usbfs_uif_all;
            return ev;
        }
        for (uint8_t turn = 0; turn < 16u; ++turn) {
            const uint8_t flags = regs().INT_FG;

            if ((flags & usbfs_uif_bus_rst) != 0u) {
                regs().INT_FG = usbfs_uif_bus_rst;
                on_bus_reset();
                ev.reset = true;
                continue;
            }
            if ((flags & usbfs_uif_transfer) != 0u) {
                const bool control = service(regs().INT_ST, ev);
                regs().INT_FG = usbfs_uif_transfer;
                if (control) {
                    break;
                }
                continue;
            }
            if ((flags & usbfs_uif_suspend) != 0u) {
                // ONE FLAG FOR TWO EVENTS (23.2.1.5): the bus went idle,
                // or it woke. The state register says which, read after
                // the vendor's ten microseconds.
                regs().INT_FG = usbfs_uif_suspend;
                settle(10);
                if ((regs().MIS_ST & usbfs_ums_suspend) != 0u) {
                    ev.suspend = true;
                } else {
                    ev.resume = true;
                }
                continue;
            }
            if ((flags & usbfs_uif_fifo_ov) != 0u) {
                regs().INT_FG = usbfs_uif_fifo_ov;
                bump(overruns_);
                continue;
            }
            break;
        }
        return ev;
    }

    // ---- the wake-up line ---------------------------------------------------

    /**
     * Arm (or give back) the EXTI line of this controller's wake-up event
     * (`wakeup_line`): a rising edge, the interrupt enabled, the vector
     * opened - so the core is woken, and the app binds that line's
     * vector to wakeup_isr(): `usb_wakeup_handler` on the CH32V303, whose
     * line is 18, and `usbfs_wakeup_handler` on the CH32V203. False where
     * the part has no such line.
     */
    static bool arm_wakeup(bool on) {
        if (!on) {
            Pfic::disable(wakeup_irq);
            return Exti::release(wakeup_line);
        }
        (void)Exti::clear(wakeup_line);
        (void)Exti::sense(wakeup_line, ExtiSense::rising);
        const bool ok = Exti::interrupt(wakeup_line, true);
        Pfic::enable(wakeup_irq);
        return ok;
    }

    static bool wakeup_armed() { return Exti::interrupt(wakeup_line); }

    /// THE WAKE-UP LINE'S ISR BODY: its flag read and cleared, and counted.
    /// True when this vector's line had fired.
    [[gnu::always_inline]] static bool wakeup_isr() {
        const uint32_t fired = Exti::isr(Exti::vector_lines(wakeup_irq));
        if (fired != 0u) {
            wakeups_ = wakeups_ + 1u;
        }
        return fired != 0u;
    }

    static uint32_t wakeups() { return wakeups_; }

    // ---- the frame counter ------------------------------------------------

    /// Count the host's start-of-frame packets. THE BIT IS THE VENDOR'S
    /// AND NOT THE MANUAL'S: INT_EN bit 7 is reserved in 23.2.1.2, and the
    /// vendor's own header names it the device SOF interrupt, reported as
    /// a transfer with token 01 - which 23.2.1.6 also calls reserved.
    /// Off at init(): a thousand interrupts a second, each pausing the
    /// bus until it is served. This block keeps no frame NUMBER of its
    /// own.
    static void count_frames(bool on) {
        regs().INT_EN = on ? static_cast<uint8_t>(regs().INT_EN | usbfs_uie_dev_sof)
                           : static_cast<uint8_t>(regs().INT_EN & ~usbfs_uie_dev_sof);
    }

    static uint32_t frames() { return frames_; }
    /// The count modulo 2048, the width of a frame number - what a
    /// program that prints a "frame" gets on this block.
    static uint16_t frame() { return static_cast<uint16_t>(frames_ & 0x07FFu); }

    // ---- readbacks and the instruments -------------------------------------
    static uint8_t address() { return static_cast<uint8_t>(regs().DEV_AD & usbfs_uda_addr_mask); }
    /// The field that is the device function and the pull-up at once.
    static bool pulled_up() {
        return Rcc::enabled(Bus::hb, rcc_hb_usbfs) && (regs().CTRL & usbfs_uc_dev_pu_en) != 0u;
    }
    /// The bus-master count this controller holds.
    static bool attached() { return attached_; }
    static bool suspended() { return (regs().MIS_ST & usbfs_ums_suspend) != 0u; }
    static bool bus_in_reset() { return (regs().MIS_ST & usbfs_ums_bus_reset) != 0u; }
    static bool sie_free() { return (regs().MIS_ST & usbfs_ums_sie_free) != 0u; }
    /// The levels the block sees on its two pads (RB_UD_DP_PIN, RB_UD_DM_PIN).
    static bool dp_high() { return (regs().UDEV_CTRL & usbfs_ud_dp_pin) != 0u; }
    static bool dm_high() { return (regs().UDEV_CTRL & usbfs_ud_dm_pin) != 0u; }
    /// R8_USB_INT_FG and R8_USB_INT_ST as they stand - for an instrument
    /// that looks at what the handler is about to see.
    static uint8_t flags() { return regs().INT_FG; }
    static uint8_t status() { return regs().INT_ST; }
    /// What RX_LEN said at the last SETUP - eight, if the register counts
    /// a SETUP at all; the driver copies eight bytes whatever it says.
    static uint16_t setup_length() { return setup_length_; }
    /// What R8_USB_INT_ST said at the last SETUP, whole.
    static uint8_t setup_status() { return setup_status_; }

    static uint16_t buffer_used() { return next_; }
    static uint16_t buffer_free() { return static_cast<uint16_t>(pool_bytes - next_); }

    /// Three counters from init(), all saturating - a counter that
    /// wrapped would report a healthy bus. errors(): an IN or OUT
    /// completion reported on a number nothing has claimed (a SETUP is
    /// not judged by its number - the file header says why).
    /// overruns(): the FIFO overflow flag (RB_UIF_FIFO_OV). toggle_errors():
    /// OUT packets dropped because their PID was not the one expected.
    static uint16_t errors() { return errors_; }
    static uint16_t overruns() { return overruns_; }
    static uint16_t toggle_errors() { return toggle_errors_; }

    /// The bytes accepted on endpoint n since init(), counted from the
    /// length the block reported for each packet - RX_LEN is one register
    /// for every endpoint, and this is what it said about THIS one.
    static uint32_t received(uint8_t n) { return n < endpoint_count ? received_[n] : 0u; }

    /// The PID each direction will send or expect next.
    static bool in_toggle(uint8_t n) {
        return n < endpoint_count && (regs().UEP[n].TX_CTRL & usbfs_uep_tog) != 0u;
    }
    static bool out_toggle(uint8_t n) {
        return n < endpoint_count && (regs().UEP[n].RX_CTRL & usbfs_uep_tog) != 0u;
    }

    /// Set the PID the next OUT packet on n must carry. A program never
    /// needs this - the driver keeps the toggle - and a suite uses it to
    /// stage the lost acknowledgement the toggle exists for: the next
    /// packet then arrives with the other PID and is dropped, and the one
    /// after it is taken. False for a number nothing has claimed.
    static bool set_out_toggle(uint8_t n, bool data1) {
        if (n >= endpoint_count || (n != 0u && !claimed(n, false))) {
            return false;
        }
        InterruptGuard guard;
        const uint8_t ctrl = regs().UEP[n].RX_CTRL;
        regs().UEP[n].RX_CTRL = static_cast<uint8_t>((ctrl & ~usbfs_uep_tog) | (data1 ? usbfs_uep_tog : 0u));
        return true;
    }

    /// RB_UC_INT_BUSY, the pause every other verb here relies on (the file
    /// header). Off only to stage what it prevents.
    static void auto_pause(bool on) {
        regs().CTRL = on ? static_cast<uint8_t>(regs().CTRL | usbfs_uc_int_busy)
                         : static_cast<uint8_t>(regs().CTRL & ~usbfs_uc_int_busy);
    }
    static bool auto_pause() { return (regs().CTRL & usbfs_uc_int_busy) != 0u; }

private:
    static constexpr uint8_t claim_out = 0x1;
    static constexpr uint8_t claim_in = 0x2;

    static bool claimed(uint8_t n, bool in) { return (claimed_[n] & (in ? claim_in : claim_out)) != 0u; }

    /// A direction holding a packet (IN) or waiting for one (OUT).
    static bool armed(uint8_t n, bool in) {
        const uint8_t ctrl = in ? regs().UEP[n].TX_CTRL : regs().UEP[n].RX_CTRL;
        return claimed(n, in) && (ctrl & usbfs_uep_res_mask) == usbfs_uep_res_ack;
    }

    /// Where each direction's bytes are in the pool: endpoint zero's one
    /// buffer at the bottom; a number's OUT half at its slot, its IN half
    /// 64 above it when the number receives too and at the slot otherwise.
    static uint16_t rx_offset(uint8_t n) { return n == 0u ? 0u : slot_[n]; }
    static uint16_t tx_offset(uint8_t n) {
        if (n == 0u) {
            return 0u;
        }
        return claimed(n, false) ? static_cast<uint16_t>(slot_[n] + usbfs_half_bytes) : slot_[n];
    }

    static uint32_t pool_address(uint16_t offset) {
        return static_cast<uint32_t>(reinterpret_cast<uintptr_t>(&pool_[offset]));
    }

    /// The response bits of one control byte, the toggle kept.
    static void respond(volatile uint8_t& ctrl, uint8_t response) {
        ctrl = static_cast<uint8_t>((ctrl & usbfs_uep_tog) | response);
    }

    /// One direction back to NAK with DATA0 - the state of a fresh claim.
    static void park(uint8_t n, bool in) {
        if (in) {
            regs().UEP[n].T_LEN = 0;
            regs().UEP[n].TX_CTRL = usbfs_uep_res_nak;
        } else {
            regs().UEP[n].RX_CTRL = usbfs_uep_res_nak;
        }
    }

    /// Number n's nibble from its claims, its neighbour's left as it is.
    static void write_mode(uint8_t n) {
        const UsbfsModSlot slot = usbfs_mod_slot(n);
        if (slot.reg == 0xFFu) {
            return;
        }
        const uint8_t nibble = static_cast<uint8_t>((claimed(n, false) ? usbfs_mod_rx_en : 0u) |
                                                    (claimed(n, true) ? usbfs_mod_tx_en : 0u));
        const uint8_t keep = static_cast<uint8_t>(~(0xFu << slot.shift));
        regs().UEP_MOD[slot.reg] =
            static_cast<uint8_t>((regs().UEP_MOD[slot.reg] & keep) | (nibble << slot.shift));
    }

    /// Number n answered NAK both ways and forgotten.
    static void release_number(uint8_t n) {
        regs().UEP[n].TX_CTRL = usbfs_uep_res_nak;
        regs().UEP[n].RX_CTRL = usbfs_uep_res_nak;
        regs().UEP[n].T_LEN = 0;
        claimed_[n] = 0;
        slot_[n] = 0;
        max_in_[n] = 0;
        max_out_[n] = 0;
        out_len_[n] = 0;
    }

    /// Endpoint zero over its buffer: nothing to send, ready for a SETUP.
    static void lay_out_control() {
        regs().UEP_DMA[0] = pool_address(0);
        regs().UEP[0].T_LEN = 0;
        regs().UEP[0].TX_CTRL = usbfs_uep_res_nak;
        regs().UEP[0].RX_CTRL = usbfs_uep_res_ack;
    }

    /// A pad left a floating input with its port's clock open - through
    /// the port the part table names, spelled so that a part with no such
    /// pad (a port of 0) forms no Port at all.
    template <char port>
    static void float_pad(uint8_t pin) {
        constexpr uint32_t floating = pin_nibble(PinMode::input, PinDrive::push_pull, PinSpeed::fast);
        if constexpr (port == 'A') {
            Port<'A'>::configure(pin, floating);
        } else if constexpr (port == 'B') {
            Port<'B'>::configure(pin, floating);
        }
    }

    /// At least `us` microseconds with no timer: a register read per turn
    /// and enough turns for the part's fastest HCLK, which is longer at
    /// every slower one. For the two waits this block's own sequence
    /// asks for, both before a timer can be assumed to run.
    static void settle(uint32_t us) {
        const uint32_t turns = us * (device::sysclk_max_hz / 1'000'000UL);
        for (uint32_t i = 0; i < turns; ++i) {
            (void)regs().MIS_ST;
        }
    }

    /**
     * A BUS RESET: the address back to zero, every number released, and
     * endpoint zero laid out again for the SETUP that follows within
     * microseconds. Only registers are written - a SETUP that has
     * already landed in the buffer is not touched, and a transfer flag
     * standing beside the reset is served on the loop's next turn.
     */
    static void on_bus_reset() {
        regs().DEV_AD = static_cast<uint8_t>(regs().DEV_AD & usbfs_uda_gp_bit);
        deconfigure_endpoints();
        lay_out_control();
    }

    /// One completed transfer, from the status byte. True when it was
    /// endpoint zero's (the loop stops there, see take_events()).
    static bool service(uint8_t status, UsbEvents& ev) {
        const uint8_t token = static_cast<uint8_t>(status & usbfs_uis_token_mask);
        const uint8_t n = static_cast<uint8_t>(status & usbfs_uis_endp_mask);
        switch (token) {
            case usbfs_uis_token_setup: {
                setup_status_ = status;
                // A SETUP BEGINS A NEW TRANSFER: both directions to NAK
                // with DATA1, a halt from the last request cleared with
                // them, and the packet copied out before the answer
                // lands in the same buffer.
                setup_length_ = rx_length();
                regs().UEP[0].TX_CTRL = static_cast<uint8_t>(usbfs_uep_tog | usbfs_uep_res_nak);
                regs().UEP[0].RX_CTRL = static_cast<uint8_t>(usbfs_uep_tog | usbfs_uep_res_nak);
                setup_ = parse_setup();
                ev.setup = true;
                return true;
            }
            case usbfs_uis_token_in: {
                if (n >= endpoint_count || (n != 0u && !claimed(n, true))) {
                    // A completion on a number nothing has claimed - one
                    // from before a bus reset released it, or a status
                    // byte that is not what the chapter says.
                    bump(errors_);
                    return n == 0u;
                }
                const uint8_t tx = regs().UEP[n].TX_CTRL;
                regs().UEP[n].TX_CTRL =
                    static_cast<uint8_t>(((tx ^ usbfs_uep_tog) & usbfs_uep_tog) | usbfs_uep_res_nak);
                ev.in_done = static_cast<uint16_t>(ev.in_done | (1u << n));
                return n == 0u;
            }
            case usbfs_uis_token_out: {
                if (n >= endpoint_count || (n != 0u && !claimed(n, false))) {
                    bump(errors_);
                    return n == 0u;
                }
                const uint16_t length = rx_length();
                if ((status & usbfs_uis_tog_ok) == 0u) {
                    // A REPEAT: acknowledged by the silicon, dropped here,
                    // the endpoint left armed with the PID it expected.
                    bump(toggle_errors_);
                    return n == 0u;
                }
                const uint8_t rx = regs().UEP[n].RX_CTRL;
                regs().UEP[n].RX_CTRL =
                    static_cast<uint8_t>(((rx ^ usbfs_uep_tog) & usbfs_uep_tog) | usbfs_uep_res_nak);
                out_len_[n] = static_cast<uint8_t>(length > max_packet ? max_packet : length);
                received_[n] = received_[n] + out_len_[n];
                ev.out_done = static_cast<uint16_t>(ev.out_done | (1u << n));
                return n == 0u;
            }
            default:
                // Token 01: the start of a frame (count_frames()).
                frames_ = frames_ + 1u;
                return false;
        }
    }

    static uint16_t rx_length() { return static_cast<uint16_t>(regs().RX_LEN & 0x03FFu); }

    static UsbSetup parse_setup() {
        const volatile uint8_t* b = &pool_[0];
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

    alignas(4) static inline volatile uint8_t pool_[pool_bytes]{};
    static inline uint16_t next_ = usbfs_ep0_bytes;
    static inline uint16_t slot_[endpoint_count]{};
    static inline uint8_t claimed_[endpoint_count]{};
    static inline uint8_t max_in_[endpoint_count]{};
    static inline uint8_t max_out_[endpoint_count]{};
    static inline uint8_t out_len_[endpoint_count]{};
    static inline uint32_t received_[endpoint_count]{};
    static inline UsbSetup setup_{};
    static inline uint16_t setup_length_ = 0;
    static inline uint8_t setup_status_ = 0;
    static inline bool attached_ = false;
    static inline uint16_t errors_ = 0;
    static inline uint16_t overruns_ = 0;
    static inline uint16_t toggle_errors_ = 0;
    static inline volatile uint32_t frames_ = 0;
    static inline volatile uint32_t wakeups_ = 0;
};

/// The claim this driver makes - util/usb's controller contract, checked
/// at a part that carries the block, through a template for usb.hpp's
/// reason: a bare static_assert would instantiate the controller on every
/// part, and on one without it the first thing met would be the refusal
/// above, while a header of this stratum compiles on every part.
template <bool present>
struct UsbfsContract {
    static_assert(!present || UsbController<Usbfs<>>,
                  "brio Usbfs: this driver no longer realizes util/usb's UsbController");
};

static_assert(sizeof(UsbfsContract<device::has_usbfs>) > 0);

} // namespace brio
