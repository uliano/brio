// BKP family smoke TU: the backup data registers, the tamper input
// that wipes them, and the three things this block can put on the
// TAMPER pad - every verb, on every part.
//
// TWO PER-PART FACTS run through this file. How many data registers
// there are is the DEVICE CLASS's (ten on the CH32V20x_D6, forty-two on
// the D8), and whether the TAMPER pad comes out of the package at all
// is the PACKAGE's: PC13 is bonded on three parts of the nine. Both are
// read from the part table, and neither is named as a number here.
#include <stddef.h>

#include "ch32v203/rtc.hpp"

using namespace brio;

// ---- the register map (RM 4.3's table 4-1) ----------------------------------
static_assert(offsetof(BkpRegs, DATAR) == 0x04);
static_assert(offsetof(BkpRegs, OCTLR) == 0x2C);
static_assert(offsetof(BkpRegs, TPCTLR) == 0x30);
static_assert(offsetof(BkpRegs, TPCSR) == 0x34);
static_assert(offsetof(BkpRegs, DATAR_HIGH) == 0x40);
static_assert(sizeof(BkpDataReg) == 4u);
static_assert(sizeof(BkpRegs) == 0xC0u);

// ---- the bits ---------------------------------------------------------------
static_assert(bkp_cal_mask == 0x7Fu);
static_assert(bkp_cco == (1u << 7) && bkp_asoe == (1u << 8) && bkp_asos == (1u << 9));
static_assert(bkp_tpe == 1u && bkp_tpal == 2u);
static_assert(bkp_cte == 1u && bkp_cti == 2u && bkp_tpie == 4u);
static_assert(bkp_tef == (1u << 8) && bkp_tif == (1u << 9));

// ---- how many registers, and what that is worth in bytes --------------------
static_assert(Bkp::count == device::bkp_data_registers);
static_assert(Bkp::count == (device::is_d8_class ? 42u : 10u));
static_assert(Bkp::count * 2u >= 20u);

// ---- the pad ----------------------------------------------------------------
static_assert(Bkp::tamper_pad_port == 'C' && Bkp::tamper_pad_pin == 13u);
static_assert(Bkp::has_tamper_pad == ((device::port_pins('C') & (1u << 13)) != 0u));
// The pad the tamper input uses is also the pad the calibration output
// and the pulse output use, which is why those three verbs exclude one
// another (4.2.3, 4.3.2's note 1).
static_assert(Bkp::has_tamper_pad == device::has_port('C'));

// ---- every verb -------------------------------------------------------------
void bkp_verbs() {
    Bkp::clock(true);
    (void)Bkp::clock();
    (void)Bkp::open();
    Bkp::reset_block();

    (void)Bkp::data(1u, 0xBEEFu);
    (void)Bkp::data(1u);
    (void)Bkp::data(0u);                  // refused: the chapter numbers from one
    (void)Bkp::data(Bkp::count + 1u);     // refused: past this class's count
    (void)Bkp::data<1>();
    Bkp::data<1>(0x1234u);
    (void)Bkp::data<Bkp::count>();

    (void)Bkp::calibration(0x7Fu);
    (void)Bkp::calibration(0x80u);        // refused: CAL is seven bits
    (void)Bkp::calibration();

    (void)Bkp::clock_output(true);
    (void)Bkp::clock_output();
    (void)Bkp::pulse_output(true, BkpPulse::second);
    (void)Bkp::pulse_output(true, BkpPulse::alarm);
    (void)Bkp::pulse_output(false);
    (void)Bkp::pulse_output();
    (void)Bkp::pulse_source();

    (void)Bkp::tamper(true, true);
    (void)Bkp::tamper(false);
    (void)Bkp::tamper_enabled();
    (void)Bkp::tamper_active_low();
    Bkp::tamper_interrupt(true);
    (void)Bkp::tamper_interrupt();
    (void)Bkp::tamper_event();
    (void)Bkp::tamper_interrupt_flag();
    Bkp::clear_event();
    Bkp::clear_interrupt();
    (void)Bkp::isr();
    (void)Bkp::regs().TPCSR;
}
