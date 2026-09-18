// OTP family smoke TU: every READ verb instantiated once, chapter 13's
// ECC and voting arithmetic exercised at compile time against codewords
// this file builds itself, and THE ABSENCE OF A WRITE PATH asserted -
// the hard rule of this bench, checked by the compiler on every sweep
// rather than trusted to a reader.
#include <array>
#include <type_traits>

#include "rp2350/otp.hpp"

using namespace brio;

// ---- the geometry (13.1, 13.5) --------------------------------------------

static_assert(otp_row_count == 4096);
static_assert(otp_page_rows == 64);
static_assert(otp_page_count == 64);
static_assert(otp_row_count == otp_page_rows * otp_page_count);
static_assert(otp_ecc_bytes == 8192);
static_assert(Otp::page_of(0) == 0);
static_assert(Otp::page_of(63) == 0);
static_assert(Otp::page_of(64) == 1);
static_assert(Otp::page_of(4095) == 63);

// The four windows: bit 14 picks raw over ECC, bit 15 guarded over not.
static_assert(otp_ecc_window == 0x40130000u);
static_assert(otp_raw_window == 0x40134000u);
static_assert(otp_ecc_guarded_window == 0x40138000u);
static_assert(otp_raw_guarded_window == 0x4013c000u);
static_assert((otp_raw_window ^ otp_ecc_window) == 0x4000u);
static_assert((otp_ecc_guarded_window ^ otp_ecc_window) == otp_guarded_offset);

// 13.3's ceiling on clk_ref while the array is accessed.
static_assert(otp_clk_ref_max_hz == 25'000'000u);

// ---- the locks (13.5.1): a thermometer that only climbs ------------------

static_assert(otp_lock_of(0) == OtpLock::read_write);
static_assert(otp_lock_of(1) == OtpLock::read_only);
static_assert(otp_lock_of(3) == OtpLock::inaccessible);
static_assert(static_cast<uint8_t>(OtpLock::read_write) <
              static_cast<uint8_t>(OtpLock::read_only));
static_assert(static_cast<uint8_t>(OtpLock::read_only) <
              static_cast<uint8_t>(OtpLock::inaccessible));

// ---- ECC and bit repair (13.6), judged on codewords built here ------------

/// A row as the array would hold it: the datum, its six parity bits, and
/// no bit repair. Built here so that the decoder is judged against the
/// standard's own definition and not against itself.
constexpr uint32_t codeword(uint16_t data) {
    return (static_cast<uint32_t>(otp_ecc_parity(data)) << 16) | data;
}

static_assert(otp_even_parity(0u) == 0u);
static_assert(otp_even_parity(1u) == 1u);
static_assert(otp_even_parity(3u) == 0u);
static_assert(otp_even_parity(0xFFFFFFFFu) == 0u);

// A clean row decodes clean, whatever the datum.
static_assert(otp_ecc_decode(codeword(0x0000u)).ecc == OtpEcc::ok);
static_assert(otp_ecc_decode(codeword(0xBEEFu)).ecc == OtpEcc::ok);
static_assert(otp_ecc_decode(codeword(0xBEEFu)).data == 0xBEEFu);
static_assert(!otp_ecc_decode(codeword(0xBEEFu)).inverted);

// One flipped DATA bit is corrected, and the datum comes back.
static_assert(otp_ecc_decode(codeword(0xBEEFu) ^ 0x0040u).ecc == OtpEcc::corrected);
static_assert(otp_ecc_decode(codeword(0xBEEFu) ^ 0x0040u).data == 0xBEEFu);
static_assert(otp_ecc_decode(codeword(0xBEEFu) ^ 0x0040u).corrected_bit == 6);

// One flipped PARITY bit is corrected too, and leaves the datum alone.
static_assert(otp_ecc_decode(codeword(0x1234u) ^ 0x00010000u).ecc == OtpEcc::corrected);
static_assert(otp_ecc_decode(codeword(0x1234u) ^ 0x00010000u).data == 0x1234u);

// Two flipped bits are DETECTED and not repaired - which is the whole
// point of the sixth parity bit.
static_assert(otp_ecc_decode(codeword(0x1234u) ^ 0x0003u).ecc == OtpEcc::uncorrectable);
static_assert(otp_ecc_decode(codeword(0x1234u) ^ 0x0101u).ecc == OtpEcc::uncorrectable);

// THE WHOLE CODE, SWEPT, at compile time: every one of the twenty-two
// single flips is located and repaired, and every one of the two hundred
// and thirty-one double flips is detected and refused. This is what
// makes the sixth check's definition (rp2350/otp.hpp's `otp_ecc_checks`)
// a checked fact rather than a reading of one ambiguous sentence.
constexpr bool every_single_flip_is_repaired(uint16_t data) {
    const uint32_t cw = codeword(data);
    for (uint8_t bit = 0; bit < 22; ++bit) {
        const OtpRow r = otp_ecc_decode(cw ^ (1u << bit));
        if (r.ecc != OtpEcc::corrected || r.data != data || r.corrected_bit != bit) {
            return false;
        }
    }
    return true;
}
constexpr bool every_double_flip_is_detected(uint16_t data) {
    const uint32_t cw = codeword(data);
    for (uint8_t a = 0; a < 22; ++a) {
        for (uint8_t b = static_cast<uint8_t>(a + 1); b < 22; ++b) {
            if (otp_ecc_decode(cw ^ (1u << a) ^ (1u << b)).ecc != OtpEcc::uncorrectable) {
                return false;
            }
        }
    }
    return true;
}
static_assert(otp_ecc_checks(codeword(0xBEEFu)) == 0u);
static_assert(otp_ecc_checks(codeword(0xBEEFu) ^ 0x0008u) != 0u);
static_assert((otp_ecc_checks(codeword(0xBEEFu) ^ 0x0008u) & 0x20u) != 0u);
static_assert(every_single_flip_is_repaired(0x0000u));
static_assert(every_single_flip_is_repaired(0xBEEFu));
static_assert(every_single_flip_is_repaired(0xFFFFu));
static_assert(every_double_flip_is_detected(0x1234u));
static_assert(every_double_flip_is_detected(0xA5A5u));

// Bit repair by polarity: a row stored inverted carries ones in 23:22,
// and reads back as the datum it was made from (13.6.1).
constexpr uint32_t repaired(uint16_t data) { return ~codeword(data) & 0x00FFFFFFu; }
static_assert((repaired(0x1234u) & 0x00C00000u) == 0x00C00000u);
static_assert(otp_ecc_decode(repaired(0x1234u)).inverted);
static_assert(otp_ecc_decode(repaired(0x1234u)).ecc == OtpEcc::ok);
static_assert(otp_ecc_decode(repaired(0x1234u)).data == 0x1234u);

// ---- the two votes (13.4, 13.10) -----------------------------------------

static_assert(otp_vote3(0xFFu, 0xFFu, 0x00u) == 0xFFu);
static_assert(otp_vote3(0xFFu, 0x00u, 0x00u) == 0x00u);
static_assert(otp_vote3(0x0Fu, 0xF0u, 0xFFu) == 0xFFu);

// Three of eight is deliberately biased towards SET: three copies carry
// the flag, two do not.
static_assert(otp_vote8({1u, 1u, 1u, 0u, 0u, 0u, 0u, 0u}) == 1u);
static_assert(otp_vote8({1u, 1u, 0u, 0u, 0u, 0u, 0u, 0u}) == 0u);
static_assert(otp_vote8({0xFFFFFFu, 0xFFFFFFu, 0xFFFFFFu, 0u, 0u, 0u, 0u, 0u}) == 0xFFFFFFu);
// And it never reads above the row's own width.
static_assert(otp_vote8({~0u, ~0u, ~0u, ~0u, ~0u, ~0u, ~0u, ~0u}) == 0x00FFFFFFu);

// ---- the named rows and flags (13.10) -------------------------------------

static_assert(OtpRowId::chipid0 == 0x000u);
static_assert(OtpRowId::randid0 == 0x004u);
static_assert(OtpRowId::crit0 == 0x038u);
static_assert(OtpRowId::crit1 == 0x040u);
static_assert(OtpRowId::boot_flags0 == 0x048u);
// CRIT0 and CRIT1 are eight rows apart, which is the redundancy they
// carry; the boot flags are three.
static_assert(OtpRowId::crit1 - OtpRowId::crit0 == 8);
static_assert(OtpRowId::boot_flags1 - OtpRowId::boot_flags0 == 3);
// Both critical rows are in page 0 and page 1, as 13.4 says.
static_assert(Otp::page_of(OtpRowId::crit0) == 0);
static_assert(Otp::page_of(OtpRowId::crit1) == 1);

static_assert(OtpCrit0::arm_disable != OtpCrit0::riscv_disable);
static_assert(OtpCrit1::secure_boot_enable != OtpCrit1::debug_disable);
static_assert(OtpCritical::arm_disable == OTP_CRITICAL_ARM_DISABLE_BITS);

// ---- THE WRITE PATH DOES NOT EXIST ----------------------------------------
//
// Not "is not used": is not there. These are the verbs a programming
// driver would have, and every one of them is checked to be absent on
// every sweep of the family check, on both architectures and both
// packages. The reason is in the file header of rp2350/otp.hpp: one bit
// of this array set by mistake can remove a debug port or an
// architecture from the only board this stratum has.

template <typename T>
concept has_program = requires { T::program(uint16_t{0}, uint16_t{0}); };
template <typename T>
concept has_write = requires { T::write(uint16_t{0}, uint16_t{0}); };
template <typename T>
concept has_soft_lock = requires { T::soft_lock(uint8_t{0}, OtpLock::read_only); };
template <typename T>
concept has_lock = requires { T::lock(uint8_t{0}, OtpLock::read_only); };
template <typename T>
concept has_sbpi = requires { T::sbpi_instruction(uint32_t{0}); };

static_assert(!has_program<Otp>, "brio Otp is read-only: no programming verb may exist");
static_assert(!has_write<Otp>, "brio Otp is read-only: no write verb may exist");
static_assert(!has_soft_lock<Otp>, "brio Otp is read-only: SW_LOCKn is read and never written");
static_assert(!has_lock<Otp>, "brio Otp is read-only: SW_LOCKn is read and never written");
static_assert(!has_sbpi<Otp>, "brio Otp is read-only: the SBPI programming bridge is not driven");

// ---- the read verbs -------------------------------------------------------

void otp_window_verbs() {
    (void)Otp::regs().CRITICAL;
    (void)Otp::ecc(OtpRowId::num_gpios);
    (void)Otp::raw(OtpRowId::crit1);
    (void)Otp::ecc_guarded(OtpRowId::num_gpios);
    (void)Otp::raw_guarded(OtpRowId::crit1);

    const auto row = Otp::read(OtpRowId::chipid0);
    if (row) {
        (void)row->raw;
        (void)row->data;
        (void)(row->ecc == OtpEcc::ok);
        (void)row->corrected_bit;
        (void)row->inverted;
    }
    (void)Otp::read(otp_row_count);
    (void)Otp::read_run(OtpRowId::chipid0, 4);
}

void otp_lock_verbs() {
    const OtpPageLock lock = Otp::page_lock(0);
    (void)(lock.secure == OtpLock::read_only);
    (void)(lock.non_secure == OtpLock::read_write);
    (void)Otp::readable(63);
    (void)Otp::readable(otp_page_count);
}

void otp_fact_verbs() {
    (void)Otp::chip_id();
    (void)Otp::random_id();
    (void)Otp::rosc_calib_khz();
    (void)Otp::lposc_calib_hz();
    (void)Otp::num_gpios();
    (void)Otp::info_crc();
    (void)Otp::flash_devinfo();
    (void)Otp::crit0();
    (void)Otp::crit1();
    (void)Otp::boot_flags0();
    (void)Otp::boot_flags1();
    (void)Otp::usb_boot_flags();
}

void otp_register_verbs() {
    (void)Otp::critical();
    (void)Otp::key_valid();
    (void)Otp::debugen();
    (void)Otp::debugen_lock();
    (void)Otp::archsel();
    (void)Otp::bootdis();
    (void)Otp::data_window_enabled();
    (void)Otp::rma_flag();
    (void)Otp::debug_status();
    (void)Otp::interrupt_raw();
    (void)Otp::interrupt_status();
    Otp::clear_interrupts(0u);
    (void)(Otp::irq == OTP_IRQ_IRQn);

    const OtpArchSel arch = Otp::archsel_status();
    (void)(arch.core0 == OtpArchitecture::arm);
    (void)(arch.core1 == OtpArchitecture::riscv);
    (void)arch.raw;
}
