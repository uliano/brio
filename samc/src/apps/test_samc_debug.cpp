// test_samc_debug - the reference bench suite for the four small
// debug-class chapters of the SAM C21: samc/pac.hpp (ch. 11),
// samc/dsu.hpp (ch. 13), samc/divas.hpp (ch. 14) and samc/mtb.hpp
// (10.3).
//
// ONE SUITE FOR FOUR CHAPTERS, because each of them is two or three
// letters' worth and because three of the four meet each other here:
// the DSU comes out of reset PAC-protected, DIVAS's only error report is
// a bit in the PAC's own AHB flag register, and the MTB's identifier is
// a number only the PAC's register map states. Letters are grouped by
// chapter and named in the menu.
//
// NOTHING TO WIRE. Not one of these blocks has a pad: 11.4.1, 13.5.1 (a
// probe's three debug signals, which no suite can drive), 14.5.1 and the
// MTB's nothing at all.
//
// WHAT MAKES THE VERDICTS CHECKABLE, chapter by letter:
//
//   - The PAC is checked against ITSELF: every protection claim is made
//     twice, once with protection off (the control) and once with it on,
//     and the flag banks are cleared between. What that measures is not
//     "does protection work" but WHICH PERIPHERALS REPORT A VIOLATION -
//     the contrast the stratum has already met twice with opposite
//     answers (TSENS's erratum 1.19.1 drops a protected write in
//     silence; the CCL's 1.7.4 raises a flag with no protection at all).
//   - The DSU's CRC32 is checked against a SOFTWARE CRC-32 computed here
//     over the same bytes. util/crc.hpp is CRC-16 and is not touched:
//     the reference is a table-free bitwise CRC-32 local to this file,
//     which is the honest shape for a one-off reference.
//   - DIVAS is checked against the COMPILER: every quotient and
//     remainder is compared with what gcc's own software division
//     produces for the same operands, and the timing letter measures
//     both.
//   - The MTB is checked against the LINKER: a trace window runs a known
//     chain of noinline functions and the decoded packets must contain
//     their addresses.
//
// build: boards = c21j
// build: monitor_speed = 115200

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/divas.hpp"
#include "samc/dsu.hpp"
#include "samc/evsys.hpp"
#include "samc/mtb.hpp"
#include "samc/nvm.hpp"
#include "samc/nvic.hpp"
#include "samc/pac.hpp"
#include "samc/pin.hpp"
#include "samc/reset.hpp"
#include "samc/sercom.hpp"
#include "samc/ticker.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

using SysClock = brio::Clock<brio::ClockSource::internal, 48'000'000>;
constexpr SysClock clock;

// ---------------------------------------------------------------------------
// The token letter c lives in (the test_samc_platform machinery)
//
// INLINE and in .noinit for the two reasons that suite gives: the
// section must survive the crt, and gcc gives an inline variable with a
// section attribute a COMDAT group where a plain one gets none. Table
// 18-1 lists no SRAM row for any reset source, so every read is guarded
// by the magic word.
// ---------------------------------------------------------------------------
inline constexpr uint16_t token_magic = 0xDB01;

struct Token {
    uint16_t magic;
    uint8_t leg;        ///< which reset we are waiting for (0 = none)
    uint16_t pass;
    uint16_t fail;
};
[[gnu::section(".noinit")]] inline Token token;

namespace {

using namespace brio;
using brio::crlf;
using brio::print;

constexpr UartPads console_pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'B', 30, PinFunction::d},
    .rx_pin = {'B', 31, PinFunction::d},
};
using Serial = Uart<5, console_pads>;
constexpr Serial serial;

using Led = Pin<'B', 23>;

TestBench<Serial> bench;

// ---------------------------------------------------------------------------
// The boot snapshot
//
// PAC protection state and flags are GLOBAL and STICKY, so "what does a
// reset leave behind" can only be answered by a reading taken before any
// letter has run. main() takes it first thing.
// ---------------------------------------------------------------------------
uint32_t boot_pac_status[3] = {0, 0, 0};
uint32_t boot_pac_flags[3] = {0, 0, 0};
uint32_t boot_pac_ahb = 0;
uint8_t boot_dsu_statusa = 0;
uint8_t boot_dsu_statusb = 0;
ResetCause boot_cause = ResetCause::unknown;

// ---------------------------------------------------------------------------
// The cycle stopwatch (the test_samc_dma / test_samc_ccl technique)
// ---------------------------------------------------------------------------
uint32_t cycles_now() {
    const uint32_t reload = SysTick->LOAD;
    for (;;) {
        const uint32_t t0 = Ticker::ticks();
        const uint32_t val = SysTick->VAL;
        const uint32_t t1 = Ticker::ticks();
        if (t0 == t1) {
            return t0 * (reload + 1u) + (reload - val);
        }
    }
}

void console_drain() {
    for (uint32_t i = 0; i < 8'000'000UL && !Serial::tx_idle(); ++i) {
    }
    const uint32_t t0 = cycles_now();
    while (cycles_now() - t0 < SysClock::hz / 500u) {
    }
}

/// Eight zero-padded lowercase hex digits - the format
/// tools/bench_boards.py records the die serial in, which a bare hex()
/// (no padding) cannot produce.
const char* hex8(uint32_t v, char* buf) {
    static const char digits[] = "0123456789abcdef";
    for (uint8_t i = 0; i < 8u; ++i) {
        buf[7u - i] = digits[(v >> (4u * i)) & 0xFu];
    }
    buf[8] = '\0';
    return buf;
}

const char* cause_name(ResetCause c) {
    switch (c) {
        case ResetCause::power_on: return "POR";
        case ResetCause::brown_out_core: return "BOD12";
        case ResetCause::brown_out_vdd: return "BOD33";
        case ResetCause::external: return "EXT";
        case ResetCause::watchdog: return "WDT";
        case ResetCause::system_request: return "SYST";
        default: return "unknown";
    }
}

// =============================================================================
// PAC letter a - the block, its keys and its balance rule
// =============================================================================
//
// The peripheral protected here is TC1: nothing in this suite or in this
// image uses it, its APB clock is off, and a protection state left on it
// by accident costs nothing.
constexpr uint16_t victim = 77;   // ID_TC1

void ta_pac_block() {
    print(serial, "  boot PAC: STATUSA=", hex(boot_pac_status[0]),
          " STATUSB=", hex(boot_pac_status[1]),
          " STATUSC=", hex(boot_pac_status[2]),
          " INTFLAGAHB=", hex(boot_pac_ahb), crlf);
    print(serial, "  boot PAC flags: A=", hex(boot_pac_flags[0]),
          " B=", hex(boot_pac_flags[1]), " C=", hex(boot_pac_flags[2]), crlf);

    bench.verdict("the identifier arithmetic is 11.7.1's: PERID = 32 x "
                  "bridge + index",
                  Pac::bridge_of(87) == 2u && Pac::bit_of(87) == (1UL << 23) &&
                      Pac::perid_of(2, 23) == 87u);
    bench.verdict("this device's PAC has three peripheral bridges",
                  Pac::bridge_count == 3u);
    bench.verdict("and an identifier on the C21N's fourth is refused",
                  !Pac::id_valid(96) && !Pac::protect(96));

    // "PAC WRITE PROTECTION IS OFF OUT OF RESET" is a sentence every
    // driver in this stratum has in its comments (11.5.2.2 says the PAC
    // is enabled, not that anything is protected). Here it is measured -
    // and it is measured to have EXACTLY ONE EXCEPTION.
    bench.verdict("out of reset nothing on bridge A is protected",
                  boot_pac_status[0] == 0u);
    bench.verdict("nor any peripheral bridge C's register description draws",
                  (boot_pac_status[2] & PAC_STATUSC_Msk) == 0u);
    bench.verdict("but bridge B comes up with the DSU ALREADY PROTECTED, "
                  "and only the DSU (STATUSB reset value 0x2, table 12-3's "
                  "one Y in the 'Prot at Reset' column)",
                  boot_pac_status[1] == Pac::bit_of(Dsu::pac_id));
    bench.verdict("and no access error is standing at boot",
                  boot_pac_flags[0] == 0u && boot_pac_flags[1] == 0u &&
                      boot_pac_flags[2] == 0u && boot_pac_ahb == 0u);

    // A BIT NEITHER DOCUMENT HAS. STATUSC comes up with bit 25 set, and
    // bit 25 is outside 11.7.12's drawing (which stops at CCL, bit 23)
    // AND outside the device header's own PAC_STATUSC_Msk of 0x00FFFFFF.
    // PERID 64 + 25 = 89, past the header's ID_PERIPH_MAX of 87. So this
    // device protects something at reset that no table names.
    const uint32_t undrawn_c = boot_pac_status[2] & ~PAC_STATUSC_Msk;
    print(serial, "  STATUSC bits outside its own register mask: ",
          hex(undrawn_c), "  (PAC_STATUSC_Msk = ", hex(PAC_STATUSC_Msk), ")",
          crlf);
    bench.verdict("bridge C comes up protecting something at a bit BOTH "
                  "documents leave blank - bit 25, PERID 89, past the "
                  "header's own ID_PERIPH_MAX of 87",
                  undrawn_c == (1UL << 25));

    // Whether the mystery bit behaves like a protection bit at all: can
    // it be cleared, does clearing it flag, and does it come back?
    Pac::clear_all_flags();
    (void)Pac::unprotect(89);
    const bool undrawn_cleared = (Pac::status(2) & (1UL << 25)) == 0u;
    const bool undrawn_flagged = Pac::flagged(Pac::pac_id);
    if (undrawn_cleared) {
        Pac::clear_all_flags();
        (void)Pac::protect(89);
    }
    const bool undrawn_restored = (Pac::status(2) & (1UL << 25)) != 0u;
    Pac::clear_all_flags();
    print(serial, "  a CLEAR aimed at PERID 89: bit ",
          undrawn_cleared ? "CLEARED" : "unmoved", ", INTFLAGA.PAC ",
          undrawn_flagged ? "set" : "clear", ", restored ", undrawn_restored,
          crlf);
    bench.verdict("and it answers WRCTRL like any other protection bit, so it "
                  "is a peripheral and not a stuck bit",
                  undrawn_cleared && undrawn_restored);

    // The round trip.
    Pac::clear_all_flags();
    const bool was_clear = !Pac::is_protected(victim);
    bench.verdict("the victim starts unprotected", was_clear);
    (void)Pac::protect(victim);
    bench.verdict("a SET makes STATUS say protected", Pac::is_protected(victim));
    bench.verdict("and raises no error of its own", !Pac::any_error());
    (void)Pac::unprotect(victim);
    bench.verdict("a CLEAR puts it back", !Pac::is_protected(victim));
    bench.verdict("still with no error", !Pac::any_error());

    // THE BALANCE RULE (11.5.2.6): a double set or a double clear is an
    // error in itself, reported in INTFLAGA.PAC - the PAC's own bit,
    // whichever peripheral the request named. This is the rule that
    // makes a naive scoped guard wrong, which is why this driver does
    // not offer one.
    Pac::clear_all_flags();
    (void)Pac::unprotect(victim);
    bench.verdict("clearing protection that is already clear raises "
                  "INTFLAGA.PAC (11.5.2.6's balance rule)",
                  Pac::flagged(Pac::pac_id));
    Pac::clear_all_flags();
    (void)Pac::protect(victim);
    (void)Pac::protect(victim);
    bench.verdict("and so does setting it twice", Pac::flagged(Pac::pac_id));
    bench.verdict("while the peripheral itself is not blamed",
                  !Pac::flagged(victim));
    Pac::clear_all_flags();
    (void)Pac::unprotect(victim);

    // KEY = OFF: the register's own no-op, and it must not count as a
    // request at all.
    Pac::clear_all_flags();
    (void)Pac::write_control(victim, PacKey::off);
    bench.verdict("KEY = OFF does nothing and is not an error",
                  !Pac::is_protected(victim) && !Pac::any_error());

    // ONLY A WORD-WISE WRITE COUNTS (11.5.2.6), and any other access is
    // itself flagged. A byte store into WRCTRL's low byte is the test;
    // the driver never emits one, so this is a RAW probe.
    Pac::clear_all_flags();
    *reinterpret_cast<volatile uint8_t*>(&PAC_REGS->PAC_WRCTRL) =
        static_cast<uint8_t>(victim);
    *(reinterpret_cast<volatile uint8_t*>(&PAC_REGS->PAC_WRCTRL) + 2) =
        static_cast<uint8_t>(PAC_WRCTRL_KEY_SET_Val);
    const bool byte_wise_did_nothing = !Pac::is_protected(victim);
    const bool byte_wise_flagged = Pac::flagged(Pac::pac_id);
    print(serial, "  a byte-wise WRCTRL write: protection ",
          byte_wise_did_nothing ? "unchanged" : "CHANGED", ", INTFLAGA.PAC ",
          byte_wise_flagged ? "set" : "clear", crlf);
    bench.verdict("a byte-wise write to WRCTRL changes no protection",
                  byte_wise_did_nothing);
    bench.verdict("and is itself reported in INTFLAGA.PAC", byte_wise_flagged);
    Pac::clear_all_flags();
    (void)Pac::unprotect(victim);   // in case the byte write DID land
    Pac::clear_all_flags();

    // 11.4.8: WRCTRL and the flag banks are NOT PAC-write-protectable,
    // so a fully protected system can still unprotect itself and still
    // clear its own errors. Proven by protecting the PAC itself.
    (void)Pac::protect(Pac::pac_id);
    bench.verdict("the PAC can protect itself", Pac::is_protected(Pac::pac_id));
    (void)Pac::protect(victim);
    const bool still_works = Pac::is_protected(victim);
    (void)Pac::unprotect(victim);
    (void)Pac::unprotect(Pac::pac_id);
    bench.verdict("and WRCTRL still works while it is protected (11.4.8 "
                  "excepts it by name)",
                  still_works && !Pac::is_protected(victim));
    Pac::clear_all_flags();
    bench.verdict("the flag banks are clearable too", !Pac::any_error());

    // The event output, written and read back. Nothing here listens to
    // it - the ACCERR generator is vocabulary evsys.hpp can route the
    // day something wants it.
    Pac::event_output(true);
    const bool ev_on = Pac::event_output();
    Pac::event_output(false);
    bench.verdict("EVCTRL.ERREO is writable and reads back",
                  ev_on && !Pac::event_output());
    bench.verdict("and the generator code is the header's 86",
                  Pac::ev_gen_error == 86u);

    // The interrupt enable, likewise armed and disarmed without anything
    // binding the shared vector.
    Pac::arm(true);
    const bool armed = Pac::armed() != 0u;
    Pac::arm(false);
    bench.verdict("INTENSET/INTENCLR arm and disarm the one ERR source",
                  armed && Pac::armed() == 0u);
}

// =============================================================================
// PAC letter b - the contrast map: who reports a protected write?
// =============================================================================
//
// THE MEASUREMENT THAT EARNS THIS CHAPTER. Sixteen peripherals across
// all three bridges, each with a register its own chapter marks "PAC
// Write-Protection". For each one: a self-write (the value just read,
// written back) with protection OFF as the control, then the same write
// with protection ON. What is recorded is whether the peripheral's bit
// in INTFLAGn came up.
//
// A self-write is used deliberately: it cannot disturb anything whether
// it lands or not, so the flag is the only variable. Five of the sixteen
// then get a SECOND probe with a changed value, to separate "the write
// was dropped" from "the write was flagged" - two different claims that
// erratum 1.19.1 and erratum 1.7.4 show can come apart in either
// direction.

struct Probe {
    const char* name;
    uint16_t perid;
    volatile void* reg;
    uint8_t width;
};

uint32_t probe_read(const Probe& p) {
    switch (p.width) {
        case 1: return *static_cast<volatile uint8_t*>(p.reg);
        case 2: return *static_cast<volatile uint16_t*>(p.reg);
        default: return *static_cast<volatile uint32_t*>(p.reg);
    }
}

void probe_write(const Probe& p, uint32_t v) {
    switch (p.width) {
        case 1: *static_cast<volatile uint8_t*>(p.reg) = static_cast<uint8_t>(v); break;
        case 2: *static_cast<volatile uint16_t*>(p.reg) = static_cast<uint16_t>(v); break;
        default: *static_cast<volatile uint32_t*>(p.reg) = v; break;
    }
}

/// The map. Every register named here is one its own chapter marks "PAC
/// Write-Protection", and every one is readable so a self-write is
/// possible. The console's own SERCOM5 is deliberately ABSENT: writing
/// its CTRLA under protection would be a test that eats its own output.
const Probe probes[] = {
    // bridge A
    {"MCLK.CPUDIV", 2, &MCLK_REGS->MCLK_CPUDIV, 1},
    {"RTC.CTRLA", 9, &RTC_REGS->MODE0.RTC_CTRLA, 2},
    {"EIC.CTRLA", 10, &EIC_REGS->EIC_CTRLA, 1},
    {"FREQM.CFGA", 11, &FREQM_REGS->FREQM_CFGA, 2},
    {"TSENS.CTRLA", 12, &TSENS_REGS->TSENS_CTRLA, 1},
    // bridge B
    {"PORT.CTRL", 32, &PORT_REGS->GROUP[0].PORT_CTRL, 4},
    {"DSU.ADDR", 33, &DSU_REGS->DSU_ADDR, 4},
    {"NVMCTRL.CTRLB", 34, &NVMCTRL_REGS->NVMCTRL_CTRLB, 4},
    {"MTB.FLOW", 36, &MTB_REGS->MTB_FLOW, 4},
    // bridge C
    {"EVSYS.CHANNEL11", 64, &EVSYS_REGS->EVSYS_CHANNEL[11], 4},
    {"SERCOM0.CTRLA", 65, &SERCOM0_REGS->USART_INT.SERCOM_CTRLA, 4},
    {"TCC0.CTRLA", 73, &TCC0_REGS->TCC_CTRLA, 4},
    {"TC0.CTRLA", 76, &TC0_REGS->COUNT8.TC_CTRLA, 4},
    {"AC.CTRLA", 84, &AC_REGS->AC_CTRLA, 1},
    {"DAC.CTRLA", 85, &DAC_REGS->DAC_CTRLA, 1},
    {"CCL.CTRL", 87, &CCL_REGS->CCL_CTRL, 1},
};
constexpr uint8_t probe_count = sizeof(probes) / sizeof(probes[0]);

/// Which of the sixteen also get the "did the write LAND?" probe: one
/// per bridge at least, and only registers whose bits are harmless to
/// flip with the block disabled.
bool probe_takes_a_value_test(uint16_t perid) {
    return perid == 11u ||   // FREQM.CFGA - REFNUM, block disabled
           perid == 32u ||   // PORT.CTRL - continuous sampling, nothing claimed
           perid == 33u ||   // DSU.ADDR - a scratch address register
           perid == 36u ||   // MTB.FLOW - watermark, tracing off
           perid == 64u;     // EVSYS.CHANNEL11 - an unused channel
}

/// Everything the map pokes needs its APB clock: a write to an unclocked
/// peripheral is a different experiment from a write to a protected one.
void map_clocks(bool on) {
    Mclk::apb_a(MCLK_APBAMASK_TSENS_Msk, on);
    Mclk::apb_c(MCLK_APBCMASK_EVSYS_Msk | MCLK_APBCMASK_SERCOM0_Msk |
                    MCLK_APBCMASK_TCC0_Msk | MCLK_APBCMASK_TC0_Msk |
                    MCLK_APBCMASK_AC_Msk | MCLK_APBCMASK_DAC_Msk |
                    MCLK_APBCMASK_CCL_Msk,
                on);
}

void tb_pac_map() {
    map_clocks(true);

    uint8_t flagged_count = 0;
    uint8_t silent_count = 0;
    uint8_t dropped_count = 0;
    uint8_t landed_count = 0;
    bool control_clean = true;

    print(serial, "  peripheral        id  br  control  protected-write  "
                  "value", crlf);

    for (uint8_t i = 0; i < probe_count; ++i) {
        const Probe& p = probes[i];

        // The peripheral must start unprotected for the experiment to
        // mean anything. The DSU does not (it comes up protected), and
        // a locked one would not either.
        if (Pac::is_protected(p.perid)) {
            (void)Pac::unprotect(p.perid);
        }
        Pac::clear_all_flags();

        // THE CONTROL: the identical write with protection off.
        const uint32_t original = probe_read(p);
        probe_write(p, original);
        const bool control_flag = Pac::flagged(p.perid);
        if (control_flag) {
            control_clean = false;
        }

        // THE EXPERIMENT.
        Pac::clear_all_flags();
        (void)Pac::protect(p.perid);
        Pac::clear_all_flags();
        probe_write(p, original);
        const bool prot_flag = Pac::flagged(p.perid);

        // And, for the five that can take it, whether the write LANDED -
        // WITH ITS OWN CONTROL, because "the write was dropped" and "that
        // bit never sticks anyway" look identical from the read side.
        const char* value_verdict = "-";
        if (probe_takes_a_value_test(p.perid)) {
            const uint32_t changed = original ^ 0x10UL;
            (void)Pac::unprotect(p.perid);
            Pac::clear_all_flags();
            probe_write(p, changed);
            const bool sticks_unprotected = probe_read(p) != original;
            probe_write(p, original);
            (void)Pac::protect(p.perid);
            Pac::clear_all_flags();
            probe_write(p, changed);
            const uint32_t after = probe_read(p);
            if (!sticks_unprotected) {
                value_verdict = "no-control";
            } else if (after == original) {
                value_verdict = "DROPPED";
                ++dropped_count;
            } else {
                value_verdict = "LANDED";
                ++landed_count;
            }
        }

        (void)Pac::unprotect(p.perid);
        Pac::clear_all_flags();
        probe_write(p, original);   // put it back whatever happened
        Pac::clear_all_flags();

        if (prot_flag) {
            ++flagged_count;
        } else {
            ++silent_count;
        }

        print(serial, "  ", p.name, "   ", p.perid, "  ",
              Pac::bridge_of(p.perid), "   ", control_flag ? "FLAG" : "ok",
              "     ", prot_flag ? "FLAGGED" : "silent ", "        ",
              value_verdict, crlf);
    }

    print(serial, "  totals: ", flagged_count, " of ", probe_count,
          " peripherals report a protected write, ", silent_count,
          " are silent; of the ", static_cast<uint8_t>(dropped_count + landed_count),
          " value probes ", dropped_count, " dropped and ", landed_count,
          " landed", crlf);

    bench.verdict("the control is clean: an unprotected write raises no "
                  "flag anywhere in the map",
                  control_clean);
    bench.verdict("a protected write is REPORTED by at least one peripheral",
                  flagged_count > 0u);
    bench.verdict("and a protected write is DROPPED wherever the value "
                  "could be checked",
                  landed_count == 0u && dropped_count > 0u);

    // ERRATUM 1.23.1, live at revision F: "Writes to the MCLK Control A
    // register (MCLK.CTRLA) do not generate a PAC protection error even
    // if this register has been write-protected using the PAC."
    //
    // AND MCLK.CTRLA DOES NOT EXIST ON THIS DEVICE. Chapter 17's
    // register map starts at INTENCLR (offset 0x01) and the device
    // header's mclk_registers_t opens with one reserved byte at 0x00. So
    // the erratum names a register this family does not implement, and
    // the question it really poses is a different one: does an access to
    // an UNIMPLEMENTED register raise the "illegal access" error
    // 11.5.2.4 promises? Measured with a control, unprotected and
    // protected, because the two are different claims.
    Pac::clear_all_flags();
    const uint8_t mclk_unprot =
        *reinterpret_cast<volatile uint8_t*>(MCLK_BASE_ADDRESS);
    const bool mclk_illegal_unprotected = Pac::flagged(2);
    Pac::clear_all_flags();
    (void)Pac::protect(2);
    Pac::clear_all_flags();
    const uint8_t mclk_prot =
        *reinterpret_cast<volatile uint8_t*>(MCLK_BASE_ADDRESS);
    const bool mclk_illegal_protected = Pac::flagged(2);
    (void)Pac::unprotect(2);
    Pac::clear_all_flags();
    print(serial, "  MCLK offset 0x00 (the Reserved byte where erratum 1.23.1 "
                  "puts CTRLA) reads ", hex(mclk_unprot), "/", hex(mclk_prot),
          "; illegal-access flag unprotected ",
          mclk_illegal_unprotected ? "SET" : "clear", ", protected ",
          mclk_illegal_protected ? "SET" : "clear", crlf);
    bench.verdict("an access to an UNIMPLEMENTED register raises the illegal-"
                  "access error 11.5.2.4 promises, protection or no protection",
                  mclk_illegal_unprotected && mclk_illegal_protected);

    // ERRATUM 1.13.2, live at revision F: PORT accesses beyond the last
    // implemented register group do NOT raise a protection error. The
    // PORT has two groups of 0x80 bytes; +0x100 is past both, and the
    // MCLK probe above is the control that says such an access DOES
    // normally flag.
    Pac::clear_all_flags();
    const uint32_t past_the_groups =
        *reinterpret_cast<volatile uint32_t*>(PORT_BASE_ADDRESS + 0x100UL);
    const bool port_illegal_unprotected = Pac::flagged(32);
    Pac::clear_all_flags();
    (void)Pac::protect(32);
    Pac::clear_all_flags();
    const uint32_t past_again =
        *reinterpret_cast<volatile uint32_t*>(PORT_BASE_ADDRESS + 0x100UL);
    const bool port_illegal_protected = Pac::flagged(32);
    (void)Pac::unprotect(32);
    Pac::clear_all_flags();
    print(serial, "  PORT + 0x100 (past both groups) reads ",
          hex(past_the_groups), "/", hex(past_again), "; flag unprotected ",
          port_illegal_unprotected ? "SET" : "clear", ", protected ",
          port_illegal_protected ? "SET" : "clear", crlf);
    bench.verdict("erratum 1.13.2 confirmed WITH A CONTROL: the same illegal "
                  "access that flags on the MCLK raises nothing on the PORT",
                  !port_illegal_unprotected && !port_illegal_protected);

    // ERRATUM 1.13.3, live at revision F: an IOBUS write is said to go
    // past PAC write protection. The PORT answers at 0x41000000 on the
    // APB and at 0x60000000 on the CPU's single-cycle local bus (the
    // device header defines both: PORT_REGS and PORT_IOBUS_REGS), and
    // the claim is that the same register behaves differently through
    // the two.
    //
    // THE WINDOW HAS TO BE CHARACTERIZED FIRST, because it turns out not
    // to be a plain mirror: four registers read through both buses.
    Pac::clear_all_flags();
    const uint32_t port_ctrl_original = PORT_REGS->GROUP[0].PORT_CTRL;
    PORT_REGS->GROUP[0].PORT_CTRL = 0xFFFFFFFFUL;
    const uint32_t ctrl_apb = PORT_REGS->GROUP[0].PORT_CTRL;
    const uint32_t ctrl_io = PORT_IOBUS_REGS->GROUP[0].PORT_CTRL;
    PORT_REGS->GROUP[0].PORT_CTRL = port_ctrl_original;

    print(serial, "  the same PORT registers through both buses:", crlf);
    print(serial, "    offset 0x00 DIR(B)  APB ", hex(PORT_REGS->GROUP[1].PORT_DIR),
          "  IOBUS ", hex(PORT_IOBUS_REGS->GROUP[1].PORT_DIR), crlf);
    print(serial, "    offset 0x10 OUT(B)  APB ", hex(PORT_REGS->GROUP[1].PORT_OUT),
          "  IOBUS ", hex(PORT_IOBUS_REGS->GROUP[1].PORT_OUT), crlf);
    print(serial, "    offset 0x20 IN(B)   APB ", hex(PORT_REGS->GROUP[1].PORT_IN),
          "  IOBUS ", hex(PORT_IOBUS_REGS->GROUP[1].PORT_IN), crlf);
    print(serial, "    offset 0x24 CTRL(A) APB ", hex(ctrl_apb),
          "  IOBUS ", hex(ctrl_io), " (both after an APB write of all ones)",
          crlf);

    const bool dir_mirrors =
        PORT_IOBUS_REGS->GROUP[1].PORT_DIR == PORT_REGS->GROUP[1].PORT_DIR &&
        PORT_REGS->GROUP[1].PORT_DIR != 0u;
    bench.verdict("the PORT's IOBUS window is live: DIR reads the same "
                  "through both buses",
                  dir_mirrors);

    // THE CONTROL FOR THE ERRATUM, on a register the window really
    // carries: DIRTGL of the LED's own pin, toggled through the IOBUS
    // and watched through the APB.
    const uint32_t dir_before = PORT_REGS->GROUP[1].PORT_DIR;
    PORT_IOBUS_REGS->GROUP[1].PORT_DIRTGL = 1UL << 23;
    const uint32_t dir_after_io = PORT_REGS->GROUP[1].PORT_DIR;
    const bool iobus_write_works = dir_after_io != dir_before;
    PORT_REGS->GROUP[1].PORT_DIR = dir_before;
    PORT_REGS->GROUP[1].PORT_DIRSET = dir_before;
    Pac::clear_all_flags();
    print(serial, "  control: an IOBUS write to DIRTGL with no protection ",
          iobus_write_works ? "reaches the PORT" : "DOES NOT REACH IT", crlf);
    bench.verdict("an IOBUS WRITE reaches the PORT when nothing is protected",
                  iobus_write_works);

    // THE EXPERIMENT.
    (void)Pac::protect(32);
    Pac::clear_all_flags();
    const uint32_t dir_guarded = PORT_REGS->GROUP[1].PORT_DIR;
    PORT_IOBUS_REGS->GROUP[1].PORT_DIRTGL = 1UL << 23;
    const uint32_t dir_after_guarded = PORT_REGS->GROUP[1].PORT_DIR;
    const bool iobus_flag = Pac::flagged(32);
    const bool iobus_landed = dir_after_guarded != dir_guarded;
    // ... and the same write through the APB, which must be refused.
    PORT_REGS->GROUP[1].PORT_DIRTGL = 1UL << 23;
    const bool apb_landed = PORT_REGS->GROUP[1].PORT_DIR != dir_after_guarded;
    const bool apb_flag = Pac::flagged(32);
    (void)Pac::unprotect(32);
    Pac::clear_all_flags();
    PORT_REGS->GROUP[1].PORT_DIR = dir_before;
    PORT_REGS->GROUP[1].PORT_DIRSET = dir_before;
    Led::output();
    Pac::clear_all_flags();

    print(serial, "  with PORT protected, the same DIRTGL write: IOBUS ",
          iobus_landed ? "LANDED" : "dropped", " (flag ",
          iobus_flag ? "set" : "clear", "), APB ",
          apb_landed ? "LANDED" : "dropped", " (flag ",
          apb_flag ? "set" : "clear", ")", crlf);
    bench.verdict("erratum 1.13.3 CONFIRMED: the APB write is refused and "
                  "flagged while the IOBUS write to the same register goes "
                  "straight through - PAC write protection has a back door "
                  "on this bus",
                  iobus_landed && !apb_landed && apb_flag);

    // ERRATUM 1.19.1 (TSENS) AND ERRATUM 1.7.4 (CCL), THE TWO POLES OF
    // THIS CHAPTER, measured side by side against the map above.
    //
    // 1.19.1: a write to a PAC-protected TSENS.CTRLB is not functional -
    // and test_samc_tsens letter p found it dropped in COMPLETE SILENCE.
    // 43.5.8 lists CTRLB among the registers PAC protection does not
    // cover, so by the chapter that write should land and flag nothing.
    // TSENS.CTRLA is in the map above and FLAGS, so this is the same
    // peripheral answering two ways.
    Mclk::apb_a(MCLK_APBAMASK_TSENS_Msk, true);
    Pac::clear_all_flags();
    (void)Pac::protect(12);
    Pac::clear_all_flags();
    TSENS_REGS->TSENS_CTRLB = TSENS_CTRLB_START_Msk;
    const bool tsens_ctrlb_flag = Pac::flagged(12);
    (void)Pac::unprotect(12);
    Pac::clear_all_flags();
    Mclk::apb_a(MCLK_APBAMASK_TSENS_Msk, false);
    print(serial, "  TSENS.CTRLB written under protection: flag ",
          tsens_ctrlb_flag ? "SET" : "clear",
          "  (CTRLA, in the map above, flags)", crlf);
    bench.verdict("erratum 1.19.1's silence is a fact about the REGISTER and "
                  "not about the peripheral: TSENS.CTRLB raises no flag where "
                  "TSENS.CTRLA does",
                  !tsens_ctrlb_flag);

    // 1.7.4: writing CCL.CTRL.SWRST triggers a PAC protection error -
    // WITH NO PROTECTION SET AT ALL. The opposite pole: a flag without a
    // violation, where TSENS gives a violation without a flag.
    Mclk::apb_c(MCLK_APBCMASK_CCL_Msk, true);
    Pac::clear_all_flags();
    const bool ccl_protected_before = Pac::is_protected(87);
    CCL_REGS->CCL_CTRL = CCL_CTRL_SWRST_Msk;
    const bool ccl_swrst_flag = Pac::flagged(87);
    Pac::clear_all_flags();
    Mclk::apb_c(MCLK_APBCMASK_CCL_Msk, false);
    print(serial, "  CCL.CTRL.SWRST written with the CCL ",
          ccl_protected_before ? "PROTECTED" : "unprotected", ": flag ",
          ccl_swrst_flag ? "SET" : "clear", crlf);
    bench.verdict("erratum 1.7.4 confirmed here too: a CCL software reset "
                  "raises the CCL's PAC flag with no protection anywhere - "
                  "so an ABSENT flag is not evidence and a PRESENT one is not "
                  "proof of a violation",
                  !ccl_protected_before && ccl_swrst_flag);
    Pac::clear_all_flags();

    map_clocks(false);
}

// =============================================================================
// PAC letter c - the lock, across two real resets (outside z)
// =============================================================================
//
// 11.5.2.5 says a locked protection "will only be cleared by a hardware
// reset" and 11.5.2.2 that "only a hardware reset will reset the PAC
// module". NEITHER SENTENCE SAYS WHICH RESETS COUNT AS HARDWARE - and
// table 18-1, which answers that question for every other block on this
// device, has no PAC row. So the answer is measured: a lock, then a
// SYSTEM reset (SYSRESETREQ), then a WATCHDOG reset.
//
// The victim is TCC2 (id 75). Nothing in this image uses it, and if the
// lock turns out to survive resets then it survives until the board is
// power-cycled - which the letter detects on a later run and says so
// instead of failing.
constexpr uint16_t lock_victim = 75;   // ID_TCC2

void bank(uint8_t leg) {
    token.magic = token_magic;
    token.leg = leg;
    token.pass = bench.passed();
    token.fail = bench.failed();
}

[[noreturn]] void await_reset(const char* what) {
    print(serial, "  ", what, crlf);
    console_drain();
    for (;;) {
    }
}

[[noreturn]] void leg_system_reset() {
    bank(1);
    print(serial, "  leg 1: a SYSTEM reset (SYSRESETREQ) with the lock "
                  "standing ...", crlf);
    console_drain();
    Reset::software();
    await_reset("(waiting for it to land)");
}

[[noreturn]] void leg_watchdog_reset() {
    bank(2);
    print(serial, "  leg 2: a WATCHDOG reset with the lock re-applied ...",
          crlf);
    console_drain();
    Watchdog::force_reset();
    await_reset("(waiting for it to land - CLEAR is synchronized)");
}

void tc_lock() {
    Pac::clear_all_flags();

    if (Pac::is_protected(lock_victim)) {
        // Not reachable once the answer below is known - a lock does not
        // survive a reset here - but kept as a guard, because a standing
        // lock would make every verdict after it meaningless.
        print(serial, "  TCC2 is ALREADY protected at boot; the legs cannot "
                      "run. Power-cycle the board.", crlf);
        bench.verdict("the victim starts unlocked", false);
        return;
    }

    (void)Pac::lock(lock_victim);
    bench.verdict("a LOCK sets protection like a SET does",
                  Pac::is_protected(lock_victim));
    bench.verdict("and raises no error of its own", !Pac::any_error());

    Pac::clear_all_flags();
    (void)Pac::unprotect(lock_victim);
    bench.verdict("a CLEAR of a locked peripheral leaves it protected",
                  Pac::is_protected(lock_victim));
    bench.verdict("and is reported in INTFLAGA.PAC", Pac::flagged(Pac::pac_id));

    Pac::clear_all_flags();
    (void)Pac::lock(lock_victim);
    bench.verdict("so is a second LOCK of a locked peripheral",
                  Pac::flagged(Pac::pac_id));
    Pac::clear_all_flags();

    leg_system_reset();
}

/// Everything after a reset letter c caused.
void tc_resume() {
    bench.resume_tally(token.pass, token.fail);
    const uint8_t leg = token.leg;
    token.leg = 0;

    print(serial, crlf, "c (continued after reset ", leg, " of 2)", crlf);
    print(serial, "  boot: RCAUSE = ", cause_name(boot_cause), ", PAC STATUSC = ",
          hex(boot_pac_status[2]), crlf);

    const bool still_locked =
        (boot_pac_status[2] & Pac::bit_of(lock_victim)) != 0u;

    // A lock that is gone must be GONE, not merely invisible: the
    // protection has to be settable and clearable again with no error.
    Pac::clear_all_flags();
    (void)Pac::protect(lock_victim);
    (void)Pac::unprotect(lock_victim);
    const bool fully_released =
        !still_locked && !Pac::is_protected(lock_victim) && !Pac::any_error();
    Pac::clear_all_flags();

    if (leg == 1) {
        bench.verdict("the reset really was a SYSTEM one",
                      boot_cause == ResetCause::system_request);
        print(serial, "  after SYSRESETREQ the lock is ",
              still_locked ? "STILL STANDING" : "GONE", crlf);
        // THE ANSWER, AND IT IS THE OPPOSITE OF THE CAUTIOUS READING.
        // 11.5.2.5's "will only be cleared by a hardware reset" and
        // 11.5.2.2's "only a hardware reset will reset the PAC module"
        // invite the reading that the CPU's own reset request is not
        // enough. Measured, it is.
        bench.verdict("a SYSTEM reset CLEARS a PAC lock - the CPU's own "
                      "SYSRESETREQ counts as the 'hardware reset' 11.5.2.2 "
                      "requires",
                      !still_locked);
        bench.verdict("and it is fully released: protection can be set and "
                      "cleared again with no error",
                      fully_released);
        (void)Pac::lock(lock_victim);
        Pac::clear_all_flags();
        leg_watchdog_reset();
    }

    if (leg == 2) {
        bench.verdict("the reset really was a WATCHDOG one",
                      boot_cause == ResetCause::watchdog);
        print(serial, "  after a watchdog reset the lock is ",
              still_locked ? "STILL STANDING" : "GONE", crlf);
        bench.verdict("a WATCHDOG reset clears it too", !still_locked);
        bench.verdict("and it is fully released here as well", fully_released);
        print(serial, "  so a PAC lock lasts until the NEXT RESET OF ANY "
                      "KIND, not until power-on - and this letter is "
                      "re-runnable because of it", crlf);
    }

    bench.end_letter();
}

// =============================================================================
// DSU letter d - board identity
// =============================================================================
//
// THE OPERATIONAL DELIVERABLE. tools/bench_boards.py records board C's
// factory 128-bit die serial and its DSU DID, and until this letter
// existed the comment said so in as many words: "recorded, NOT yet
// checked". These four words are that record; the verdict is the check.
constexpr uint32_t manifest_serial[4] = {
    0xF9E78960UL, 0x51574841UL, 0x59202020UL, 0xFF160321UL,
};
constexpr uint32_t manifest_did = 0x11010500UL;

void td_identity() {
    (void)Dsu::init();

    const DsuDeviceId id = Dsu::device_id();
    print(serial, "  DSU DID = ", hex(id.raw), crlf);
    print(serial, "    PROCESSOR ", id.processor, " (1 = Cortex-M0+)",
          "  FAMILY ", id.family, " (2 = 5V Industrial)",
          "  SERIES ", id.series, " (1 = M0+ with CAN = C21)", crlf);
    print(serial, "    DIE ", id.die, "  REVISION ", id.revision, " = rev ",
          id.revision_letter(), "  DEVSEL ", hex(id.devsel), crlf);

    bench.verdict("the processor field says Cortex-M0+", id.processor == 1u);
    bench.verdict("the family field says 5V Industrial - the SAM C",
                  id.family == 2u);
    bench.verdict("the series field says the CAN-bearing series - a C21 and "
                  "not a C20",
                  id.series == 1u);
    bench.verdict("the revision letter is a letter of the errata matrix",
                  id.revision_letter() >= 'A' && id.revision_letter() <= 'Z');
    bench.verdict("and the whole DID is the one docs/samc/vendor/README.md "
                  "recorded over SWD at bring-up",
                  id.raw == manifest_did);

    // The four serial words, in the manifest's own format.
    const DeviceSerial serial_words = DeviceSerial::read();
    char b0[9], b1[9], b2[9], b3[9];
    print(serial, "  die serial = ", hex8(serial_words.word[0], b0), "-",
          hex8(serial_words.word[1], b1), "-", hex8(serial_words.word[2], b2),
          "-", hex8(serial_words.word[3], b3), crlf);

    bool serial_matches = true;
    for (uint8_t i = 0; i < 4u; ++i) {
        if (serial_words.word[i] != manifest_serial[i]) {
            serial_matches = false;
        }
    }
    bench.verdict("the factory die serial matches the one board C carries in "
                  "tools/bench_boards.py - THE BOARD IS THE BOARD",
                  serial_matches);
    bench.verdict("and it is not a blank or an erased word",
                  serial_words.word[0] != 0u &&
                      serial_words.word[0] != 0xFFFFFFFFUL);

    // The CoreSight identification path, which is what an external probe
    // uses to conclude the same thing.
    const uint16_t partnum = Dsu::rom_partnum();
    print(serial, "  CoreSight ROM: PARTNUM ", hex(partnum), " PID0..3 ",
          hex(Dsu::rom_pid(0)), " ", hex(Dsu::rom_pid(1)), " ",
          hex(Dsu::rom_pid(2)), " ", hex(Dsu::rom_pid(3)), crlf);
    bench.verdict("PARTNUM is 0xCD0, table 13-2's 'a DSU is present'",
                  partnum == 0xCD0u);

    Dsu::release();
}

// =============================================================================
// DSU letter e - the CRC32 engine against a software CRC-32
// =============================================================================

/// The reference: table-free, bit at a time, reflected polynomial
/// 0xEDB88320, initial value and final complement as the standard CRC-32
/// defines them. util/crc.hpp is CRC-16 and stays untouched - a one-off
/// reference belongs in the suite that needs it.
uint32_t sw_crc32_update(uint32_t crc, uint8_t byte) {
    crc ^= byte;
    for (uint8_t i = 0; i < 8u; ++i) {
        crc = (crc >> 1) ^ (0xEDB88320UL & (0u - (crc & 1u)));
    }
    return crc;
}

uint32_t sw_crc32(const void* addr, uint32_t words, uint32_t seed = 0xFFFFFFFFUL) {
    const volatile uint8_t* p = static_cast<const volatile uint8_t*>(addr);
    uint32_t crc = seed;
    for (uint32_t i = 0; i < words * 4u; ++i) {
        crc = sw_crc32_update(crc, p[i]);
    }
    return crc;
}

alignas(4) volatile uint8_t crc_sample[1024];

void te_crc32() {
    (void)Dsu::init();

    // A range in FLASH, where the bytes are whatever the image put
    // there. Three sizes, because the length register's units are the
    // one thing chapter 13 states ambiguously.
    const uint32_t sizes[3] = {16, 256, 1024};
    bool all_match = true;
    for (uint8_t i = 0; i < 3u; ++i) {
        const uint32_t words = sizes[i];
        const auto hw = Dsu::crc32(0, words);
        const uint32_t sw = ~sw_crc32(reinterpret_cast<const void*>(0), words);
        print(serial, "  flash 0x0 x ", words, " words: DSU ",
              hex(hw ? *hw : 0u), " software ", hex(sw),
              hw && *hw == sw ? "  MATCH" : "  MISMATCH", crlf);
        if (!hw || *hw != sw) {
            all_match = false;
        }
    }
    bench.verdict("the engine's CRC32 is the standard reflected-0xEDB88320 "
                  "CRC-32 of the same bytes, once complemented, at three "
                  "different lengths",
                  all_match);

    // THE LENGTH UNITS, SETTLED BY DATA. 13.14.5 calls the field "Length
    // in words" and puts it at bits 31:2, which means the register value
    // is four times the word count. If the driver's `words x 4` were
    // wrong by that factor, a 256-word request would checksum 64 words
    // or 1024 - and the software reference over exactly 256 words would
    // not match. It does, above; here is the control that makes it
    // decisive.
    const auto quarter = Dsu::crc32(0, 64);
    const uint32_t sw_quarter = ~sw_crc32(reinterpret_cast<const void*>(0), 64);
    const auto full = Dsu::crc32(0, 256);
    bench.verdict("a quarter-length request checksums a quarter of the bytes, "
                  "so LENGTH really holds the word count in bits 31:2",
                  quarter && *quarter == sw_quarter && full && *full != *quarter);

    // AND LENGTH IS A COUNTER, NOT A LATCHED CONFIGURATION - something
    // 13.14.5 does not say. Written before a command it reads back the
    // byte count; after the command it reads zero, because the engine
    // consumed it.
    Dsu::set_length_words(256);
    const uint32_t len_before = Dsu::length_raw();
    (void)Dsu::crc32(0, 256);
    const uint32_t len_after = Dsu::length_raw();
    print(serial, "  LENGTH for 256 words reads ", hex(len_before),
          " before the command and ", hex(len_after), " after it", crlf);
    bench.verdict("LENGTH is written as a BYTE count (words x 4, the field "
                  "sitting at bits 31:2)",
                  len_before == 256UL * 4UL);
    bench.verdict("and it is a working COUNTER: the engine consumes it down "
                  "to zero, which 13.14.5 never says",
                  len_after == 0u);

    // CHAINING: two halves seeded from one another must equal the whole.
    // This is what the non-complemented `crc32_raw` exists for.
    const auto first = Dsu::crc32_raw(0, 128);
    const auto second = first ? Dsu::crc32_raw(512, 128, *first) : std::nullopt;
    const auto whole = Dsu::crc32_raw(0, 256);
    bench.verdict("seeding the second half with the first half's raw result "
                  "gives the whole range's checksum",
                  first && second && whole && *second == *whole);

    // SRAM, with contents this suite chose - so the answer is
    // predictable from the pattern and not only from the flash image.
    for (uint32_t i = 0; i < sizeof(crc_sample); ++i) {
        crc_sample[i] = static_cast<uint8_t>(i * 7u + 13u);
    }
    const auto ram_hw =
        Dsu::crc32(reinterpret_cast<uint32_t>(&crc_sample[0]), 256);
    const uint32_t ram_sw = ~sw_crc32(const_cast<const uint8_t*>(&crc_sample[0]), 256);
    print(serial, "  1 KB of SRAM: DSU ", hex(ram_hw ? *ram_hw : 0u),
          " software ", hex(ram_sw), crlf);
    bench.verdict("the engine reaches SRAM as well as flash, with the same "
                  "answer as software",
                  ram_hw && *ram_hw == ram_sw);

    // A CHANGED BYTE MUST CHANGE THE ANSWER - the whole point of a
    // checksum, and the control that says the engine really read the
    // memory rather than a cached answer.
    crc_sample[100] = static_cast<uint8_t>(crc_sample[100] ^ 0xFFu);
    const auto ram_hw2 =
        Dsu::crc32(reinterpret_cast<uint32_t>(&crc_sample[0]), 256);
    bench.verdict("flipping one byte changes it", ram_hw2 && *ram_hw2 != *ram_hw);

    // A BUS ERROR. 13.12.3.2 requires STATUSA.BERR to be checked after
    // every run, and the driver refuses to hand back a checksum taken
    // over one. 0x50000000 is in the "Undefined" region of the memory
    // map - no client answers there.
    const auto bad = Dsu::crc32(0x50000000UL, 16, 0xFFFFFFFFUL, 2'000'000UL);
    print(serial, "  a CRC32 over an unmapped address returns ",
          bad ? "A VALUE" : "nothing", "; PAC INTFLAGAHB = ",
          hex(Pac::ahb_flags()), crlf);
    bench.verdict("a checksum over an address nothing answers is refused, not "
                  "returned",
                  !bad.has_value());
    Pac::clear_all_flags();
    Dsu::clear_status();

    // WHAT IT COSTS. The number that decides whether the engine is worth
    // using at all: 1024 words through hardware against the same 1024
    // words through the bitwise software reference above.
    const uint32_t t0 = cycles_now();
    (void)Dsu::crc32(0, 1024);
    const uint32_t t1 = cycles_now();
    const uint32_t sw_t0 = cycles_now();
    (void)sw_crc32(reinterpret_cast<const void*>(0), 1024);
    const uint32_t sw_t1 = cycles_now();
    const uint32_t hw_cycles = t1 - t0;
    const uint32_t sw_cycles = sw_t1 - sw_t0;
    print(serial, "  4096 bytes: engine ", hw_cycles, " cycles (",
          hw_cycles / 1024u, " per word), bitwise software ", sw_cycles,
          " cycles (", sw_cycles / 1024u, " per word) = ",
          hw_cycles == 0u ? 0u : sw_cycles / hw_cycles, "x", crlf);
    bench.verdict("and the engine is faster than the bitwise software "
                  "reference by more than an order of magnitude",
                  hw_cycles != 0u && sw_cycles > hw_cycles * 10u);

    Dsu::release();
}

// =============================================================================
// DSU letter f - status, the CoreSight ROM, and MBIST
// =============================================================================

/// The buffer MBIST is allowed to destroy. Nothing else ever reads it,
/// and it is written afresh before every run.
alignas(4) volatile uint32_t mbist_buffer[64];

void tf_dsu_services() {
    (void)Dsu::init();

    // THE PROTECTION ROUND TRIP, which is this driver's own init().
    Dsu::release();
    const bool protected_after_release = Pac::is_protected(Dsu::pac_id);
    Pac::clear_all_flags();
    const uint32_t addr_before = Dsu::address_raw();
    DSU_REGS->DSU_ADDR = addr_before ^ 0x10UL;
    const bool write_dropped = Dsu::address_raw() == addr_before;
    const bool write_flagged = Pac::flagged(Dsu::pac_id);
    Pac::clear_all_flags();
    const bool init_ok = Dsu::init();
    bench.verdict("release() puts the DSU back under the protection reset "
                  "left it in",
                  protected_after_release);
    bench.verdict("a write to a protected DSU is dropped", write_dropped);
    bench.verdict("and IS reported - the DSU is one of the peripherals that "
                  "flags",
                  write_flagged);
    bench.verdict("init() takes the protection off again", init_ok);

    // STATUS B, the debugger's half. Printed and mostly NOT judged: what
    // a probe did before this program started is not something the
    // program can arrange.
    print(serial, "  STATUSA = ", hex(Dsu::status()), " (boot ",
          hex(boot_dsu_statusa), ")   STATUSB = ", hex(Dsu::status_b()),
          " (boot ", hex(boot_dsu_statusb), ")", crlf);
    print(serial, "    PROT=", Dsu::device_protected(),
          " DBGPRES=", Dsu::debugger_present(),
          " HPE=", Dsu::hot_plugging_enabled(),
          " CRSTEXT=", Dsu::cpu_reset_extended(), crlf);
    bench.verdict("the device is NOT locked by the NVM security bit - which "
                  "is why every DSU service is available here",
                  !Dsu::device_protected());
    bench.verdict("nothing is holding the CPU in the extended reset phase - "
                  "code that can read this bit is by definition running",
                  !Dsu::cpu_reset_extended());
    print(serial, "  (DBGPRES is never cleared once set, so it reports "
                  "whether a probe was ever seen since power-on, not now - "
                  "printed, not judged)", crlf);

    // THE TWO REGISTERS CHAPTER 13 DOES NOT DESCRIBE.
    print(serial, "  STATUSC (offset 0x03, 'Reserved' in 13.13, a STATE field "
                  "in the device header) = ", hex(Dsu::status_c_raw()), crlf);
    print(serial, "  DCFG0 = ", hex(Dsu::dcfg_raw(0)), " DCFG1 = ",
          hex(Dsu::dcfg_raw(1)), " (offset 0xF0, absent from ch. 13)", crlf);

    // THE DEBUG COMMUNICATION CHANNELS, exercised from the one side that
    // is present: the dirty bit sets on write and clears on read
    // (13.12.4).
    Dsu::dcc(0, 0xA5A5A5A5UL);
    const bool dirty_after_write = Dsu::dcc_dirty(0);
    const uint32_t read_back = Dsu::dcc(0);
    const bool dirty_after_read = Dsu::dcc_dirty(0);
    print(serial, "  DCC0 written 0xA5A5A5A5, reads ", hex(read_back),
          "; DCCD0 after write ", dirty_after_write, ", after read ",
          dirty_after_read, crlf);
    bench.verdict("DCC0 holds what is written to it", read_back == 0xA5A5A5A5UL);
    bench.verdict("and its dirty bit sets on the write and clears on the read",
                  dirty_after_write && !dirty_after_read);

    // THE CORESIGHT ROM. The component identification is a fixed
    // four-byte preamble on every CoreSight component; reading it back
    // proves the ROM is where 13.11.1 says and is the table a probe
    // walks.
    print(serial, "  ROM: ENTRY0 ", hex(Dsu::rom_entry(0)), " ENTRY1 ",
          hex(Dsu::rom_entry(1)), " END ", hex(Dsu::rom_end()), " MEMTYPE ",
          hex(Dsu::rom_memtype()), crlf);
    print(serial, "  CID0..3 = ", hex(Dsu::rom_cid(0)), " ", hex(Dsu::rom_cid(1)),
          " ", hex(Dsu::rom_cid(2)), " ", hex(Dsu::rom_cid(3)), crlf);
    bench.verdict("the CoreSight component id preamble is the architected "
                  "0x0D / 0x05 / 0xB1",
                  (Dsu::rom_cid(0) & 0xFFu) == 0x0Du &&
                      (Dsu::rom_cid(2) & 0xFFu) == 0x05u &&
                      (Dsu::rom_cid(3) & 0xFFu) == 0xB1u);
    bench.verdict("CID1's component class says ROM TABLE (class 1)",
                  ((Dsu::rom_cid(1) >> 4) & 0xFu) == 0x1u);
    // THE ROM TABLE'S ENTRIES ARE ADDRESSES RELATIVE TO THE TABLE
    // ITSELF, which sits at the DSU's base + 0x1000. Resolved, they name
    // the two CoreSight components of this device - and the second of
    // them is the MTB, which is how chapter 13 and section 10.3 turn out
    // to be about the same debug system.
    const uint32_t rom_base = DSU_BASE_ADDRESS + 0x1000UL;
    const uint32_t target0 = rom_base + Dsu::rom_entry_offset(0);
    const uint32_t target1 = rom_base + Dsu::rom_entry_offset(1);
    print(serial, "  ENTRY0 -> ", hex(target0), " (present bit ",
          Dsu::rom_entry_present(0), "), ENTRY1 -> ", hex(target1),
          " (present bit ", Dsu::rom_entry_present(1), ")", crlf);
    bench.verdict("ENTRY0 resolves to 0xE00FF000, the Cortex-M0+'s own "
                  "CoreSight ROM table",
                  target0 == 0xE00FF000UL);
    bench.verdict("and ENTRY1 to 0x41008000 - THE MTB, so the DSU's ROM is "
                  "what tells a probe that section 10.3's trace buffer is "
                  "there",
                  target1 == MTB_BASE_ADDRESS);
    bench.verdict("both entries are marked present, and they point at real "
                  "components - so of 13.14.10's two contradictory sentences "
                  "the one that survives is EPRES = 1 means PRESENT",
                  Dsu::rom_entry_present(0) && Dsu::rom_entry_present(1));
    print(serial, "  (13.14.10's two sentences about EPRES contradict each "
                  "other word for word - both begin 'if the device is not "
                  "protected' - so the bit is reported and not interpreted)",
          crlf);

    // MBIST, over a buffer this file owns and nothing else reads.
    for (uint32_t i = 0; i < 64u; ++i) {
        mbist_buffer[i] = 0xDEADBEEFUL;
    }
    const uint32_t mb_t0 = cycles_now();
    const DsuMbistResult r =
        Dsu::mbist(reinterpret_cast<uint32_t>(&mbist_buffer[0]), 64);
    const uint32_t mb_t1 = cycles_now();
    print(serial, "  MBIST over 64 words: done=", r.done, " failed=", r.failed,
          " berr=", r.bus_error, " in ", mb_t1 - mb_t0, " cycles (",
          (mb_t1 - mb_t0) / 64u, " per word)", crlf);
    bench.verdict("the March LR self-test completes", r.done);
    bench.verdict("and this SRAM passes it", !r.failed && !r.bus_error);

    // AND IT REALLY DID WRITE THE MEMORY - the claim that makes MBIST
    // dangerous, proven rather than asserted.
    bool destroyed = false;
    for (uint32_t i = 0; i < 64u; ++i) {
        if (mbist_buffer[i] != 0xDEADBEEFUL) {
            destroyed = true;
        }
    }
    print(serial, "  the buffer after the test: word 0 = ",
          hex(mbist_buffer[0]), " (was 0xDEADBEEF)", crlf);
    bench.verdict("MBIST DESTROYS what it tests - the pattern written before "
                  "it is gone",
                  destroyed);

    // Pause-on-error mode over the same good buffer must also pass, and
    // must leave the same DONE.
    const DsuMbistResult rp =
        Dsu::mbist(reinterpret_cast<uint32_t>(&mbist_buffer[0]), 64, true);
    bench.verdict("pause-on-error mode (AMOD 1) passes the same memory",
                  rp.done && !rp.failed);

    // A range no client answers must come back as a bus error rather
    // than as a pass.
    const DsuMbistResult rb = Dsu::mbist(0x50000000UL, 16, false, 2'000'000UL);
    print(serial, "  MBIST over an unmapped address: done=", rb.done,
          " failed=", rb.failed, " berr=", rb.bus_error, crlf);
    bench.verdict("MBIST over an address nothing answers does not report a "
                  "pass",
                  !(rb.done && !rb.failed && !rb.bus_error));
    Pac::clear_all_flags();

    Dsu::release();
}

// =============================================================================
// DIVAS letter g - the arithmetic
// =============================================================================

volatile uint32_t v_a = 0;
volatile uint32_t v_b = 0;

void tg_divas_math() {
    Divas::bus_clock(true);
    bench.verdict("the AHB clock is on out of reset (table 12-3, index 12)",
                  Divas::bus_clock());

    Divas::configure(false);
    bench.verdict("CTRLA reads back what was written",
                  !Divas::signed_division() && !Divas::leading_zero_disabled());
    Divas::configure(false, true);
    bench.verdict("and DLZ with it", Divas::leading_zero_disabled());
    Divas::configure(false, false);

    // UNSIGNED, against the compiler's own answers for the same
    // operands. The operands go through volatiles so gcc cannot fold the
    // reference away.
    const uint32_t pairs[][2] = {
        {1000u, 7u},          {0xFFFFFFFFUL, 1u},   {0xFFFFFFFFUL, 0xFFFFFFFFUL},
        {0xFFFFFFFFUL, 3u},   {1u, 0xFFFFFFFFUL},   {0u, 5u},
        {48'000'000UL, 115200u}, {65535u, 256u},    {0x80000000UL, 7u},
    };
    bool unsigned_ok = true;
    for (uint8_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); ++i) {
        v_a = pairs[i][0];
        v_b = pairs[i][1];
        const DivasResult r = Divas::divide_unsigned(v_a, v_b);
        const uint32_t q = v_a / v_b;
        const uint32_t m = v_a % v_b;
        if (r.result != q || r.remainder != m) {
            unsigned_ok = false;
            print(serial, "  MISMATCH ", v_a, " / ", v_b, ": DIVAS ", r.result,
                  " r ", r.remainder, ", software ", q, " r ", m, crlf);
        }
    }
    bench.verdict("nine unsigned divisions agree with gcc's software division "
                  "in quotient AND remainder",
                  unsigned_ok);

    // THE IOBUS PATH must give identical answers - it is the same engine
    // at a second address, and that address comes from the data sheet's
    // memory map because no device header in this pack defines it.
    bool iobus_ok = true;
    for (uint8_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); ++i) {
        v_a = pairs[i][0];
        v_b = pairs[i][1];
        const DivasResult r = Divas::divide_unsigned<DivasBus::iobus>(v_a, v_b);
        if (r.result != v_a / v_b || r.remainder != v_a % v_b) {
            iobus_ok = false;
        }
    }
    bench.verdict("and the IOBUS alias at 0x60000200 - which only the data "
                  "sheet's memory map names - is the same engine, answer for "
                  "answer",
                  iobus_ok);

    // SIGNED. 14.6.2.4's rules: the remainder takes the DIVIDEND's sign,
    // the quotient is negative when the signs differ. C++ has said the
    // same since C++11, so the compiler is again the reference.
    Divas::configure(true);
    const int32_t spairs[][2] = {
        {-1000, 7}, {1000, -7}, {-1000, -7}, {-1, 2}, {7, -1},
        {-2147483647 - 1, 2},
    };
    bool signed_ok = true;
    for (uint8_t i = 0; i < sizeof(spairs) / sizeof(spairs[0]); ++i) {
        v_a = static_cast<uint32_t>(spairs[i][0]);
        v_b = static_cast<uint32_t>(spairs[i][1]);
        const DivasSignedResult r = Divas::divide_signed(
            static_cast<int32_t>(v_a), static_cast<int32_t>(v_b));
        const int32_t q = static_cast<int32_t>(v_a) / static_cast<int32_t>(v_b);
        const int32_t m = static_cast<int32_t>(v_a) % static_cast<int32_t>(v_b);
        if (r.result != q || r.remainder != m) {
            signed_ok = false;
            print(serial, "  MISMATCH ", static_cast<int32_t>(v_a), " / ",
                  static_cast<int32_t>(v_b), ": DIVAS ", r.result, " r ",
                  r.remainder, ", software ", q, " r ", m, crlf);
        }
    }
    bench.verdict("six signed divisions match the language's own truncating "
                  "semantics, remainders included",
                  signed_ok);

    // THE OVERFLOW THE CHAPTER PROMISES WITH NO INDICATION (14.6.2.4).
    Divas::clear_divide_by_zero();
    const DivasSignedResult ovf =
        Divas::divide_signed(-2147483647 - 1, -1);
    print(serial, "  0x80000000 / -1 = ", ovf.result, " r ", ovf.remainder,
          ", DBZ ", Divas::divide_by_zero(), crlf);
    bench.verdict("the most negative number over minus one returns the most "
                  "negative number, with no status bit anywhere - exactly as "
                  "14.6.2.4 warns",
                  ovf.result == (-2147483647 - 1) && !Divas::divide_by_zero());

    Divas::configure(false);

    // DIVIDE BY ZERO: not a fault, and the status LATCHES.
    Divas::clear_divide_by_zero();
    bench.verdict("DBZ starts clear", !Divas::divide_by_zero());
    const DivasResult dz = Divas::divide_unsigned(12345u, 0u);
    print(serial, "  12345 / 0 = ", dz.result, " r ", dz.remainder, ", DBZ ",
          Divas::divide_by_zero(), crlf);
    bench.verdict("a divide by zero gives quotient 0 and remainder = dividend "
                  "(14.6.2.5), and does not trap",
                  dz.result == 0u && dz.remainder == 12345u);
    bench.verdict("and sets STATUS.DBZ", Divas::divide_by_zero());
    (void)Divas::divide_unsigned(100u, 4u);
    bench.verdict("which LATCHES across a later good division", Divas::divide_by_zero());
    Divas::clear_divide_by_zero();
    bench.verdict("until it is written back", !Divas::divide_by_zero());

    // SQUARE ROOT: exact squares, non-squares, and the remainder
    // identity REMAINDER = n - RESULT^2.
    const uint32_t roots[] = {0u, 1u, 4u, 1'000'000UL, 999'999UL, 0xFFFFFFFFUL,
                              123456789UL};
    bool sqrt_ok = true;
    for (uint8_t i = 0; i < sizeof(roots) / sizeof(roots[0]); ++i) {
        const DivasResult r = Divas::square_root(roots[i]);
        const uint64_t sq = static_cast<uint64_t>(r.result) * r.result;
        const uint64_t next =
            static_cast<uint64_t>(r.result + 1u) * (r.result + 1u);
        if (sq > roots[i] || next <= roots[i] ||
            r.remainder != roots[i] - static_cast<uint32_t>(sq)) {
            sqrt_ok = false;
            print(serial, "  sqrt MISMATCH ", roots[i], " -> ", r.result, " r ",
                  r.remainder, crlf);
        }
    }
    bench.verdict("seven square roots are the exact integer root, with "
                  "REMAINDER = n - root^2 (14.6.2.3)",
                  sqrt_ok);
    const DivasResult big = Divas::square_root(0xFFFFFFFFUL);
    print(serial, "  sqrt(0xFFFFFFFF) = ", big.result, " r ", big.remainder,
          crlf);
    bench.verdict("including the widest input, whose root is 65535",
                  big.result == 65535u);

    // The square root is unsigned whatever CTRLA says - 14.6.2.7 gives
    // it no signed mode, and the driver neither reads nor writes the bit.
    Divas::configure(true);
    const DivasResult sq_signed = Divas::square_root(1'000'000UL);
    Divas::configure(false);
    bench.verdict("and it ignores CTRLA.SIGNED, which has no meaning for it",
                  sq_signed.result == 1000u);

    // WRITING AN OPERAND WHILE BUSY. 14.5.8 says it "will result in an
    // error" without saying where; the only candidate on this device is
    // PAC.INTFLAGAHB.DIVAS. Two divisions issued back to back through
    // the IOBUS - the bus that does not stall - is the fastest a CPU can
    // try to overlap them.
    Pac::clear_ahb_flags();
    for (uint16_t i = 0; i < 200u; ++i) {
        divas_registers_t& io = Divas::io_regs();
        io.DIVAS_DIVIDEND = 0xFFFFFFFFUL;
        io.DIVAS_DIVISOR = 3u;
        io.DIVAS_DIVIDEND = 0xFFFFFFFFUL;   // deliberately while busy
        io.DIVAS_DIVISOR = 7u;
    }
    const uint32_t ahb_flags = Pac::ahb_flags();
    (void)Divas::wait_idle();
    print(serial, "  200 deliberately overlapped operand writes leave "
                  "PAC.INTFLAGAHB = ", hex(ahb_flags), " (the DIVAS bit is ",
          hex(PacAhbFlag::divas), ")", crlf);
    bench.verdict("14.5.8's unnamed 'error' for an operand write while busy "
                  "IS the PAC's AHB-client flag for DIVAS - the only place on "
                  "this device it could have been",
                  (ahb_flags & PacAhbFlag::divas) != 0u);
    bench.verdict("and nothing else in INTFLAGAHB is disturbed",
                  (ahb_flags & ~PacAhbFlag::divas) == 0u);
    Pac::clear_ahb_flags();

    // And the engine must still be right afterwards.
    v_a = 1'000'000UL;
    v_b = 7u;
    const DivasResult after = Divas::divide_unsigned(v_a, v_b);
    bench.verdict("and the engine still divides correctly after the abuse",
                  after.result == v_a / v_b && after.remainder == v_a % v_b);
}

// =============================================================================
// DIVAS letter h - what it costs, against gcc's software division
// =============================================================================
//
// THE NUMBER THAT DECIDES WHETHER ADOPTING THIS BLOCK AS THE TOOLCHAIN'S
// DIVISION IS EVER WORTH IT. Every measurement is a DIFFERENCE OF TWO
// LOOPS - the loop with the operation and an otherwise identical loop
// with a cheap operation in its place - because a single division is a
// handful of cycles and the stopwatch's own read costs more than that
// (the lesson test_samc_dac wrote down).
//
// The SysTick interrupt runs inside the measured windows, at one entry
// per millisecond; at 48 MHz that is well under a tenth of a per cent
// and is not corrected for.

constexpr uint32_t bench_n = 4000;

[[gnu::noinline]] uint32_t loop_baseline(uint32_t n) {
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; ++i) {
        acc += v_a + v_b;
    }
    return acc;
}

[[gnu::noinline]] uint32_t loop_software_div(uint32_t n) {
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; ++i) {
        acc += v_a / v_b;
    }
    return acc;
}

[[gnu::noinline]] uint32_t loop_software_divmod(uint32_t n) {
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; ++i) {
        acc += (v_a / v_b) + (v_a % v_b);
    }
    return acc;
}

[[gnu::noinline]] uint32_t loop_divas_ahb(uint32_t n) {
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const DivasResult r = Divas::divide_unsigned(v_a, v_b);
        acc += r.result + r.remainder;
    }
    return acc;
}

[[gnu::noinline]] uint32_t loop_divas_ahb_quotient(uint32_t n) {
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; ++i) {
        DIVAS_REGS->DIVAS_DIVIDEND = v_a;
        DIVAS_REGS->DIVAS_DIVISOR = v_b;
        acc += DIVAS_REGS->DIVAS_RESULT;
    }
    return acc;
}

[[gnu::noinline]] uint32_t loop_divas_iobus(uint32_t n) {
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; ++i) {
        const DivasResult r = Divas::divide_unsigned<DivasBus::iobus>(v_a, v_b);
        acc += r.result + r.remainder;
    }
    return acc;
}

[[gnu::noinline]] uint32_t loop_divas_sqrt(uint32_t n) {
    uint32_t acc = 0;
    for (uint32_t i = 0; i < n; ++i) {
        acc += Divas::square_root(v_a).result;
    }
    return acc;
}

volatile uint32_t timing_sink = 0;

uint32_t time_loop(uint32_t (*fn)(uint32_t)) {
    const uint32_t t0 = cycles_now();
    timing_sink = timing_sink + fn(bench_n);
    const uint32_t t1 = cycles_now();
    return t1 - t0;
}

/// Per-operation cost in HUNDREDTHS of a cycle, so a fraction survives
/// the integer print.
uint32_t centicycles(uint32_t total, uint32_t baseline) {
    if (total <= baseline) {
        return 0;
    }
    return ((total - baseline) * 100u) / bench_n;
}

void print_row(const char* name, uint32_t total, uint32_t baseline) {
    const uint32_t cc = centicycles(total, baseline);
    print(serial, "    ", name, ": ", cc / 100u, ".",
          (cc % 100u) / 10u, (cc % 10u), " cycles", crlf);
}

void th_divas_cost() {
    Divas::bus_clock(true);
    Divas::configure(false, false);

    struct Case {
        const char* label;
        uint32_t a;
        uint32_t b;
    };
    const Case cases[] = {
        {"small  (0xFF / 7)", 0xFFUL, 7u},
        {"medium (0xFFFF / 7)", 0xFFFFUL, 7u},
        {"large  (0xFFFFFFFF / 7)", 0xFFFFFFFFUL, 7u},
    };

    uint32_t sw_large = 0, hw_large = 0;

    for (uint8_t i = 0; i < 3u; ++i) {
        v_a = cases[i].a;
        v_b = cases[i].b;
        const uint32_t base = time_loop(loop_baseline);
        const uint32_t sw = time_loop(loop_software_div);
        const uint32_t swm = time_loop(loop_software_divmod);
        const uint32_t hw = time_loop(loop_divas_ahb);
        const uint32_t hwq = time_loop(loop_divas_ahb_quotient);
        const uint32_t io = time_loop(loop_divas_iobus);

        print(serial, "  ", cases[i].label, "  (", bench_n,
              " iterations, baseline ", base, " cycles)", crlf);
        print_row("gcc a/b                     ", sw, base);
        print_row("gcc a/b and a%b             ", swm, base);
        print_row("DIVAS quotient only, AHB    ", hwq, base);
        print_row("DIVAS quotient+remainder,AHB", hw, base);
        print_row("DIVAS quotient+remainder,IO ", io, base);

        if (i == 2u) {
            sw_large = sw - base;
            hw_large = hwq - base;
        }
    }

    bench.verdict("the accelerator beats gcc's software division on the "
                  "widest operands",
                  hw_large != 0u && hw_large < sw_large);

    // LEADING-ZERO OPTIMIZATION. With DLZ clear the engine skips the
    // dividend's leading zeros, so a small dividend is cheaper; with DLZ
    // set every 32-bit division takes sixteen cycles whatever the
    // operands (14.6.2.6). Measured both ways on both extremes.
    print(serial, "  leading-zero optimization (CTRLA.DLZ):", crlf);
    for (uint8_t dlz = 0; dlz < 2u; ++dlz) {
        Divas::configure(false, dlz != 0u);
        v_a = 0xFFUL;
        v_b = 7u;
        const uint32_t base_small = time_loop(loop_baseline);
        const uint32_t small = time_loop(loop_divas_ahb_quotient);
        v_a = 0xFFFFFFFFUL;
        const uint32_t base_large = time_loop(loop_baseline);
        const uint32_t large = time_loop(loop_divas_ahb_quotient);
        // BOTH VERDICTS ARE BANDS ON THE ABSOLUTE DIFFERENCE, and the
        // second one has to be: with DLZ set the two costs are the SAME
        // number (2080 centicycles measured), so an ordering comparison
        // between them is a coin flip - the stopwatch's own noise, a
        // SysTick entry landing in one window and not the other, decides
        // it. The first version wrote `large >= small` and failed one run
        // in three, which is the latent-suite-bug shape this project has
        // been bitten by before.
        const uint32_t cs = centicycles(small, base_small);
        const uint32_t cl = centicycles(large, base_large);
        const uint32_t diff = cl > cs ? cl - cs : cs - cl;
        print(serial, "    DLZ=", dlz, "  0xFF/7: ", cs / 100u, ".",
              (cs % 100u) / 10u, "   0xFFFFFFFF/7: ", cl / 100u, ".",
              (cl % 100u) / 10u, "  cycles, apart by ", diff / 100u, ".",
              (diff % 100u) / 10u, crlf);
        if (dlz == 0u) {
            // Measured 8.3 against 20.8 - twelve cycles apart, so a
            // five-cycle floor is nowhere near either the noise or the
            // claim.
            bench.verdict("with the optimization on, a small dividend costs "
                          "measurably less than a full-width one",
                          cl > cs && diff > 500u);
        } else {
            // Measured 20.8 against 20.8. Two cycles is twenty times the
            // observed spread and still far below the twelve the
            // optimization is worth.
            bench.verdict("with it off, the two cost the same to within two "
                          "cycles - the deterministic timing 14.6.2.6 offers",
                          diff < 200u);
        }
    }
    Divas::configure(false, false);

    // The square root, which has no software counterpart in this build
    // at all - so its cost is reported as an absolute.
    v_a = 0xFFFFFFFFUL;
    const uint32_t base_sq = time_loop(loop_baseline);
    const uint32_t sq = time_loop(loop_divas_sqrt);
    print_row("square root of 0xFFFFFFFF   ", sq, base_sq);
    bench.verdict("the square root costs something measurable and finite",
                  sq > base_sq);

    print(serial, "  (adopting the block as the toolchain's division is a "
                  "whole-image decision this driver deliberately does not "
                  "take - docs/samc/divas.md names it open, with these "
                  "numbers as its input)", crlf);
}

// =============================================================================
// MTB letter i - a self-hosted trace, decoded
// =============================================================================

/// The trace buffer: 1024 bytes = 128 packets, aligned to its own size
/// because the write pointer wraps within a naturally aligned region.
alignas(1024) uint32_t trace_buffer[256];

volatile uint32_t trace_sink = 0;

// A chain of functions the linker gives distinct addresses and gcc
// cannot fold together (each body is different) or tail-call away (each
// ends with a store of its own).
[[gnu::noinline]] void trace_leaf_a() { trace_sink = trace_sink + 11u; }
[[gnu::noinline]] void trace_leaf_b() { trace_sink = trace_sink * 3u + 1u; }
[[gnu::noinline]] void trace_leaf_c() { trace_sink = trace_sink ^ 0x5A5Au; }

[[gnu::noinline]] void trace_chain() {
    trace_leaf_a();
    trace_leaf_b();
    trace_leaf_c();
    trace_sink = trace_sink + 1u;   // stops the last call being a tail call
}

uint32_t address_of(void (*fn)()) {
    return reinterpret_cast<uint32_t>(fn) & ~1UL;
}

/// Whether any packet in the window landed on `target`.
bool trace_contains_destination(uint32_t packets, uint32_t target) {
    for (uint32_t i = 0; i < packets; ++i) {
        if (Mtb::packet(trace_buffer, i).destination() == target) {
            return true;
        }
    }
    return false;
}

void ti_mtb_trace() {
    print(serial, "  MTB BASE = ", hex(Mtb::sram_base()), ", buffer at ",
          hex(reinterpret_cast<uint32_t>(trace_buffer)), " x ",
          static_cast<uint32_t>(sizeof(trace_buffer)), " bytes = ",
          Mtb::packets_for(sizeof(trace_buffer)), " packets", crlf);

    bench.verdict("BASE reports the SRAM the MTB writes to",
                  Mtb::sram_base() == 0x20000000UL);
    bench.verdict("the buffer is a legal one - a power of two, aligned to "
                  "its own size",
                  Mtb::buffer_valid(trace_buffer, sizeof(trace_buffer)));
    bench.verdict("and a misaligned one is refused, because it would not "
                  "fail - it would trace somewhere else",
                  !Mtb::buffer_valid(
                      reinterpret_cast<const void*>(
                          reinterpret_cast<uint32_t>(trace_buffer) + 8u),
                      sizeof(trace_buffer)));

    bench.verdict("configure() takes it",
                  Mtb::configure(trace_buffer, sizeof(trace_buffer)));
    bench.verdict("MASTER.MASK reads back log2(bytes) - 4",
                  Mtb::mask() == Mtb::mask_for(sizeof(trace_buffer)));
    bench.verdict("and POSITION holds the buffer's OFFSET FROM BASE, not its "
                  "address",
                  Mtb::write_offset() ==
                      reinterpret_cast<uint32_t>(trace_buffer) - Mtb::sram_base());
    bench.verdict("with nothing traced yet", !Mtb::enabled() && !Mtb::wrapped());

    // THE WINDOW. Nothing is printed inside it: a console line is
    // milliseconds of branches and would bury the chain.
    for (uint32_t i = 0; i < 256u; ++i) {
        trace_buffer[i] = 0;
    }
    (void)Mtb::configure(trace_buffer, sizeof(trace_buffer));
    Mtb::enable(true);
    trace_chain();
    Mtb::enable(false);

    const uint32_t packets = Mtb::packets_written(trace_buffer, sizeof(trace_buffer));
    print(serial, "  the window produced ", packets, " packets, WRAP ",
          Mtb::wrapped(), crlf);
    bench.verdict("a trace window with no debugger attached produces packets",
                  packets > 0u);
    bench.verdict("and did not wrap a 128-packet buffer", !Mtb::wrapped());

    const uint32_t a = address_of(trace_leaf_a);
    const uint32_t b = address_of(trace_leaf_b);
    const uint32_t c = address_of(trace_leaf_c);
    print(serial, "  the linker put the chain at ", hex(a), " ", hex(b), " ",
          hex(c), crlf);
    for (uint32_t i = 0; i < packets && i < 12u; ++i) {
        const MtbPacket p = Mtb::packet(trace_buffer, i);
        print(serial, "    packet ", i, ": ", hex(p.source()), " -> ",
              hex(p.destination()), "   flags ", p.source_flag() ? 1u : 0u,
              "/", p.destination_flag() ? 1u : 0u, crlf);
    }

    bench.verdict("the trace contains the call to the first leaf",
                  trace_contains_destination(packets, a));
    bench.verdict("and to the second", trace_contains_destination(packets, b));
    bench.verdict("and to the third", trace_contains_destination(packets, c));
    bench.verdict("- so a Cortex-M0+ can read its own hardware backtrace with "
                  "nothing plugged into it",
                  trace_contains_destination(packets, a) &&
                      trace_contains_destination(packets, b) &&
                      trace_contains_destination(packets, c));

    // WHAT THE TWO BIT-0 FLAGS DO. Their names belong to the CoreSight
    // MTB-M0+ TRM, which is not a document of record here, so what is
    // reported is the count.
    uint32_t src_flags = 0, dst_flags = 0;
    for (uint32_t i = 0; i < packets; ++i) {
        const MtbPacket p = Mtb::packet(trace_buffer, i);
        if (p.source_flag()) {
            ++src_flags;
        }
        if (p.destination_flag()) {
            ++dst_flags;
        }
    }
    print(serial, "  bit 0 set on ", src_flags, " source words and ", dst_flags,
          " destination words of ", packets, crlf);
    const bool first_only =
        dst_flags == 1u && Mtb::packet(trace_buffer, 0).destination_flag();
    bench.verdict("bit 0 of the DESTINATION word marks the START OF TRACE: it "
                  "is set on exactly one packet of the window, the first",
                  first_only);
    bench.verdict("and bit 0 of the SOURCE word is set on none of them, so it "
                  "means something this window never produced",
                  src_flags == 0u);

    // A DISABLED MTB TRACES NOTHING - the control that makes the window
    // a window.
    const uint32_t frozen = Mtb::write_offset();
    trace_chain();
    trace_chain();
    bench.verdict("with MASTER.EN clear the pointer does not move",
                  Mtb::write_offset() == frozen);

    // WRAP: a long run must fill the buffer and set the wrap bit, and
    // the pointer must land back inside the buffer.
    (void)Mtb::configure(trace_buffer, sizeof(trace_buffer));
    Mtb::enable(true);
    for (uint32_t i = 0; i < 200u; ++i) {
        trace_chain();
    }
    Mtb::enable(false);
    const uint32_t start_offset =
        reinterpret_cast<uint32_t>(trace_buffer) - Mtb::sram_base();
    print(serial, "  after 200 chains: WRAP ", Mtb::wrapped(), ", pointer at +",
          hex(Mtb::write_offset() - start_offset), " of ",
          static_cast<uint32_t>(sizeof(trace_buffer)), crlf);
    bench.verdict("a buffer that fills WRAPS rather than stopping (10.3.2)",
                  Mtb::wrapped());
    bench.verdict("and the pointer stays inside the buffer",
                  Mtb::write_offset() >= start_offset &&
                      Mtb::write_offset() < start_offset + sizeof(trace_buffer));

    // AUTOSTOP: the watermark stops the trace instead, and clears
    // MASTER.EN.
    MtbConfig stop_cfg{};
    stop_cfg.auto_stop = true;
    stop_cfg.watermark_packets = 8;
    (void)Mtb::configure(trace_buffer, sizeof(trace_buffer), stop_cfg);
    Mtb::enable(true);
    for (uint32_t i = 0; i < 200u; ++i) {
        trace_chain();
    }
    const bool stopped = !Mtb::enabled();
    const uint32_t stop_packets =
        (Mtb::write_offset() - start_offset) / Mtb::packet_bytes;
    Mtb::enable(false);
    print(serial, "  AUTOSTOP at 8 packets: MASTER.EN ",
          stopped ? "CLEARED by the hardware" : "still set", ", pointer at ",
          stop_packets, " packets, WRAP ", Mtb::wrapped(), crlf);
    bench.verdict("FLOW.AUTOSTOP stops the trace at the watermark and clears "
                  "MASTER.EN itself",
                  stopped);
    bench.verdict("with the buffer holding exactly the watermark's packets "
                  "and never wrapping",
                  stop_packets == 8u && !Mtb::wrapped());

    Mtb::release();
    bench.verdict("release() hands the block back", Mtb::master() == 0u);
}

// =============================================================================
// MTB letter j - the two event users, and which numbers are connected
// =============================================================================
//
// Table 12-3 numbers the MTB's trace-start and trace-stop event users 44
// and 45; the device header's EVENT_ID_USER_MTB_START / _STOP say 45 and
// 46 and leave 44 unassigned. ONE CONNECTION SEPARATES THEM, so the
// bench decides: each candidate user is wired to a channel in turn and a
// software event is fired at it.
constexpr uint8_t ev_channel = 11;

/// Fire one software event at `user` with the MTB armed for a start, and
/// say whether tracing began.
bool start_arrives_at(uint8_t user, EventPath path, EventEdge edge) {
    MtbConfig cfg{};
    cfg.start_on_event = true;
    (void)Mtb::configure(trace_buffer, sizeof(trace_buffer), cfg);

    const EventChannelConfig ch{
        .generator = 0, .path = path, .edge = edge, .on_demand = true};
    Evsys::disconnect(user);
    if (!Evsys::connect(user, ev_channel, ch)) {
        return false;
    }
    for (uint32_t i = 0; i < 200u; ++i) {
    }
    Evsys::trigger(ev_channel);
    for (uint32_t i = 0; i < 2000u; ++i) {
    }
    const bool started = Mtb::enabled();
    Mtb::enable(false);
    Evsys::disconnect(user);
    Evsys::release_channel(ev_channel);
    return started;
}

void tj_mtb_events() {
    Evsys::bus_clock(true);
    (void)Gclk<0>::enabled();
    // The channel needs a generic clock for the clocked paths; generator
    // 0 is the CPU's own and is always up.
    (void)GclkChannel::connect(Evsys::gclk_id(ev_channel), 0);

    print(serial, "  the driver's constants are the device header's: START ",
          Mtb::ev_user_start, ", STOP ", Mtb::ev_user_stop,
          "; table 12-3 says 44 and 45", crlf);

    uint8_t working_user = 0xFF;
    for (uint8_t user = 44; user <= 46u; ++user) {
        const bool async = start_arrives_at(user, EventPath::asynchronous,
                                            EventEdge::none);
        const bool resync = start_arrives_at(user, EventPath::resynchronized,
                                             EventEdge::rising);
        print(serial, "    user ", user, ": asynchronous ",
              async ? "STARTS THE TRACE" : "nothing", ", resynchronized+rising ",
              resync ? "STARTS THE TRACE" : "nothing", crlf);
        if ((async || resync) && working_user == 0xFFu) {
            working_user = user;
        }
    }

    bench.verdict("exactly one event user starts the trace",
                  working_user != 0xFFu);
    if (working_user != 0xFFu) {
        print(serial, "  the connected trace-START user is ", working_user,
              crlf);
        bench.verdict("and it is the number the DEVICE HEADER gives, not the "
                      "one table 12-3 prints",
                      working_user == Mtb::ev_user_start);
    }

    // And the stop input, from the other end: tracing running, a stop
    // event must clear MASTER.EN.
    if (working_user != 0xFFu) {
        const uint8_t stop_user = static_cast<uint8_t>(working_user + 1u);
        MtbConfig cfg{};
        cfg.stop_on_event = true;
        (void)Mtb::configure(trace_buffer, sizeof(trace_buffer), cfg);
        const EventChannelConfig ch{.generator = 0,
                                    .path = EventPath::asynchronous,
                                    .edge = EventEdge::none,
                                    .on_demand = true};
        (void)Evsys::connect(stop_user, ev_channel, ch);
        Mtb::enable(true);
        const bool running = Mtb::enabled();
        Evsys::trigger(ev_channel);
        for (uint32_t i = 0; i < 2000u; ++i) {
        }
        const bool halted = !Mtb::enabled();
        Evsys::disconnect(stop_user);
        Evsys::release_channel(ev_channel);
        Mtb::release();
        print(serial, "  user ", stop_user, " (trace STOP): tracing was ",
              running ? "running" : "NOT running", ", after the event ",
              halted ? "STOPPED" : "still running", crlf);
        bench.verdict("the neighbouring user stops a running trace",
                      running && halted);
    }

    GclkChannel::disconnect(Evsys::gclk_id(ev_channel));
    Mtb::release();
}

// =============================================================================
// MTB letter k - AUTOHALT and HALTREQ with no debugger (outside z)
// =============================================================================
//
// OUTSIDE z BECAUSE IT RISKS THE BOARD. MASTER.HALTREQ and FLOW.AUTOHALT
// ask the core to HALT. On ARMv6-M a halt request is honoured only when
// DHCSR.C_DEBUGEN is set, and tools/bench.py CLEARS that bit at the end
// of every SAM flash - so the expectation is that both are inert here.
// If the expectation is wrong the board stops dead and needs a reflash,
// which is exactly why this is asked for by name.
void tk_mtb_halt() {
    // THE CORE CANNOT ASK. On ARMv6-M the Debug Halting Control and
    // Status register is debugger-access-only: samc/platform_sam.hpp
    // already records that this is why BKPT cannot be made conditional
    // here. So there is no reading of C_DEBUGEN to take before the
    // experiment - the only evidence is whether the board answers
    // afterwards, which is the whole reason this letter is asked for by
    // name.
    print(serial, "  the core cannot read DHCSR (debugger-access-only on "
                  "ARMv6-M), so the only evidence is survival", crlf);
    console_drain();

    // AUTOHALT at a watermark of two packets: the trace runs into it
    // immediately.
    MtbConfig cfg{};
    cfg.auto_halt = true;
    cfg.watermark_packets = 2;
    (void)Mtb::configure(trace_buffer, sizeof(trace_buffer), cfg);
    Mtb::enable(true);
    trace_chain();
    trace_chain();
    const bool alive_after_autohalt = true;
    const uint32_t flow_after = Mtb::flow();
    Mtb::enable(false);
    print(serial, "  AUTOHALT reached its watermark and the CPU is still "
                  "running; FLOW = ", hex(flow_after), crlf);
    bench.verdict("FLOW.AUTOHALT does not stop a core with no debugger "
                  "attached",
                  alive_after_autohalt);

    // HALTREQ, asked for directly.
    console_drain();
    const uint32_t m = Mtb::master();
    MTB_REGS->MTB_MASTER = m | MTB_MASTER_HALTREQ_Msk;
    for (uint32_t i = 0; i < 100'000UL; ++i) {
    }
    MTB_REGS->MTB_MASTER = m;
    print(serial, "  MASTER.HALTREQ was set and cleared and the CPU is still "
                  "running", crlf);
    bench.verdict("MASTER.HALTREQ is inert too - a halt request needs "
                  "DHCSR.C_DEBUGEN, which bench.py clears after every flash",
                  true);

    Mtb::release();
}

// =============================================================================
// The menu
// =============================================================================
void banner() {
    print(serial, crlf,
          "test_samc_debug - SAMC21J18A PAC (11) / DSU (13) / DIVAS (14) / "
          "MTB (10.3), clk=", SysClock::hz, " Hz", crlf);
    bench.menu();
}

} // namespace

extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
extern "C" void SERCOM5_Handler() { (void)Serial::isr(); }

// The PAC's own ERR line is IRQ 0, shared with PM/MCLK/OSCCTRL/
// OSC32KCTRL/SUPC. It is deliberately left unbound: every violation this
// suite provokes is read from the flag banks by polling, which is the
// only way to attribute one to the write that caused it.

int main() {
    // THE BOOT SNAPSHOT, FIRST. PAC protection and its flags are global
    // and sticky, so "what does a reset leave behind" can only be
    // answered before any letter has touched them - and before the
    // console is even up, since bringing up SERCOM5 writes a peripheral.
    for (uint8_t b = 0; b < 3u; ++b) {
        boot_pac_status[b] = brio::Pac::status(b);
        boot_pac_flags[b] = brio::Pac::flags(b);
    }
    boot_pac_ahb = brio::Pac::ahb_flags();
    boot_dsu_statusa = DSU_REGS->DSU_STATUSA;
    boot_dsu_statusb = DSU_REGS->DSU_STATUSB;
    boot_cause = brio::Reset::cause();

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    Led::output();
    brio::enable_interrupts();

    bench.letter('a', "PAC: the block, the keys and the balance rule",
                 ta_pac_block);
    bench.letter('b', "PAC: who reports a protected write, across three "
                      "bridges", tb_pac_map);
    bench.letter('c', "PAC: the lock, across two real resets", tc_lock, false);
    bench.letter('d', "DSU: board identity - DID, revision, die serial",
                 td_identity);
    bench.letter('e', "DSU: the CRC32 engine against a software CRC-32",
                 te_crc32);
    bench.letter('f', "DSU: status, the CoreSight ROM and MBIST",
                 tf_dsu_services);
    bench.letter('g', "DIVAS: the arithmetic against gcc's own division",
                 tg_divas_math);
    bench.letter('h', "DIVAS: what it costs", th_divas_cost);
    bench.letter('i', "MTB: a self-hosted trace, decoded", ti_mtb_trace);
    bench.letter('j', "MTB: the two event users and their real numbers",
                 tj_mtb_events);
    bench.letter('k', "MTB: AUTOHALT and HALTREQ with no debugger",
                 tk_mtb_halt, false);

    const bool resuming = token.magic == token_magic && token.leg != 0u;

    if (serial_ok) {
        print(serial, crlf, "boot: clk=", clock_ok ? "OSC48M" : "FAILED",
              " tick=", tick_ok ? "SysTick" : "FAILED", " reset=",
              cause_name(boot_cause), crlf);
        if (resuming) {
            tc_resume();
        } else {
            banner();
        }
    }
    bench.prompt();

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        print(serial, static_cast<char>(c), crlf);
        Led::toggle();
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            print(serial, "unknown letter (? for the menu)", crlf);
        }
        bench.prompt();
    }
}
