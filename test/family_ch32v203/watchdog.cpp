// Watchdog family smoke TU: both blocks, every verb, and the two
// arithmetics - the independent watchdog's against the LSI rate the
// caller states, the window one's against PCLK1.
//
// Neither block is a per-part fact: every part of this series carries
// both (datasheet table 2-1's "2 (WWDG + IWDG)" row spans the whole
// table), so this TU compiles for all nine. What IS per part is the
// LSI's rated spread, which is why the time-out helpers take the rate
// as an argument and this file asks device:: for the corners rather
// than naming a number.
#include "ch32v203/pfic.hpp"
#include "ch32v203/watchdog.hpp"

using namespace brio;

// ---- the independent watchdog's arithmetic (RM 7.2.1, 7.3.2) ---------------
static_assert(iwdg_prescaler_divider(IwdgPrescaler::div4) == 4);
static_assert(iwdg_prescaler_divider(IwdgPrescaler::div64) == 64);
static_assert(iwdg_prescaler_divider(IwdgPrescaler::div256) == 256);

// The period is (reload + 1) prescaled ticks: at 40 kHz and /32 a
// reload of 1249 is one second, and the reset value (0x0FFF at /4) is
// what the silicon starts from.
static_assert(iwdg_timeout_ms(IwdgPrescaler::div32, 1249, 40000) == 1000);
static_assert(iwdg_timeout_us(IwdgPrescaler::div4, 0, 40000) == 100);
static_assert(iwdg_timeout_ms(IwdgPrescaler::div4, 0x0FFF, 40000) == 409);
static_assert(iwdg_timeout_ms(IwdgPrescaler::div256, 0x0FFF, 40000) == 26214);
static_assert(iwdg_timeout_us(IwdgPrescaler::div4, 0, 0) == 0);

// Never early: a reload is the SMALLEST whose time-out reaches what was
// asked, and a request past the twelve-bit field answers 0xFFFF.
static_assert(iwdg_reload_for(IwdgPrescaler::div32, 1000000, 40000) == 1249);
static_assert(iwdg_timeout_us(IwdgPrescaler::div32,
                              iwdg_reload_for(IwdgPrescaler::div32, 999999, 40000),
                              40000) >= 999999);
static_assert(iwdg_reload_for(IwdgPrescaler::div4, 100000000, 40000) == 0xFFFF);
static_assert(iwdg_reload_for(IwdgPrescaler::div4, 0, 40000) == 0);

// THE SPREAD IS THE PART'S, and it is what makes a nominal time-out a
// nominal one: the same setting is this many times longer at the slow
// corner than at the fast one.
static_assert(device::lsi_min_hz < device::lsi_typ_hz &&
              device::lsi_typ_hz < device::lsi_max_hz);
static_assert(iwdg_timeout_ms(IwdgPrescaler::div32, 1249, device::lsi_min_hz) >
              iwdg_timeout_ms(IwdgPrescaler::div32, 1249, device::lsi_max_hz));

static_assert(iwdg_config_valid({.prescaler = IwdgPrescaler::div256, .reload = 0x0FFF}));
static_assert(!iwdg_config_valid({.reload = 0x1000}));
static_assert(!iwdg_config_valid({.prescaler = static_cast<IwdgPrescaler>(7)}));

// ---- the window watchdog's arithmetic (RM 8.2.1, 8.3.2) --------------------
static_assert(wwdg_cycles_per_tick(WwdgPrescaler::div1) == 4096);
static_assert(wwdg_cycles_per_tick(WwdgPrescaler::div8) == 32768);
static_assert(wwdg_tick_hz(36000000, WwdgPrescaler::div1) == 8789);
static_assert(wwdg_tick_hz(0, WwdgPrescaler::div1) == 0);

// From a refresh at 0x7F the counter has 64 ticks to fall to 0x3F.
static_assert(wwdg_timeout_us(72000000, WwdgPrescaler::div1, 0x7F) ==
              64ULL * 4096ULL * 1000000ULL / 72000000ULL);
static_assert(wwdg_timeout_us(72000000, WwdgPrescaler::div8, 0x40) ==
              1ULL * 32768ULL * 1000000ULL / 72000000ULL);
static_assert(wwdg_timeout_us(0, WwdgPrescaler::div1, 0x7F) == 0);

// And the window is the wait BEFORE a refresh is legal: from 0x7F down
// to the window value.
static_assert(wwdg_window_wait_us(72000000, WwdgPrescaler::div1, 0x7F, 0x5F) ==
              32ULL * 4096ULL * 1000000ULL / 72000000ULL);
static_assert(wwdg_window_wait_us(72000000, WwdgPrescaler::div1, 0x7F, 0x7F) == 0);

static_assert(wwdg_config_valid({.window = 0x40}));
static_assert(!wwdg_config_valid({.window = wwdg_floor}));
static_assert(!wwdg_config_valid({.window = 0x80}));
static_assert(Wwdg::irq() == Irq::wwdg);

void exercise_iwdg() {
    (void)Iwdg::configure({.prescaler = IwdgPrescaler::div64, .reload = 2000});
    (void)Iwdg::configure<IwdgConfig{.prescaler = IwdgPrescaler::div16, .reload = 100}>();
    Iwdg::unlock();
    Iwdg::refresh();
    (void)Iwdg::status();
    (void)Iwdg::busy();
    (void)Iwdg::busy(iwdg_pvu);
    (void)Iwdg::wait_idle();
    (void)Iwdg::wait_idle(iwdg_rvu, 10);
    (void)Iwdg::prescaler();
    (void)Iwdg::reload();
    (void)Iwdg::running();
    (void)Iwdg::timeout_us(device::lsi_typ_hz);
    (void)Iwdg::regs().STATR;
    // start() and arm() are the point of no return, and force_reset()
    // never comes back: named here, called by a program that means it.
    (void)static_cast<void (*)()>(&Iwdg::start);
    (void)static_cast<bool (*)(const IwdgConfig&)>(&Iwdg::arm);
}

void exercise_wwdg() {
    Wwdg::init();
    Wwdg::bus_clock(true);
    (void)Wwdg::bus_clock();
    Wwdg::reset();
    (void)Wwdg::configure({.prescaler = WwdgPrescaler::div4,
                           .window = 0x5F,
                           .early_wakeup = true});
    (void)Wwdg::configure<WwdgConfig{.window = 0x60}>();
    Wwdg::start(0x7F);
    Wwdg::refresh();
    Wwdg::refresh(0x70);
    (void)Wwdg::ctlr();
    (void)Wwdg::cfgr();
    (void)Wwdg::enabled();
    (void)Wwdg::counter();
    (void)Wwdg::prescaler();
    (void)Wwdg::window();
    (void)Wwdg::early_wakeup_enabled();
    (void)Wwdg::in_window();
    (void)Wwdg::flag();
    Wwdg::clear_flag();
    (void)Wwdg::isr();
    (void)Wwdg::timeout_us(36000000);
    (void)Wwdg::timeout_us(36000000, 0x60);
}

/// The one vector either block has, as an application binds it: a
/// handler with a single tick to refresh before the reset.
extern "C" BRIO_CH32_INTERRUPT void wwdg_handler() {
    if (Wwdg::isr()) {
        Wwdg::refresh();
    }
}
