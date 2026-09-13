// FMC family smoke TU: the external memory controller - the block, the
// SDRAM banks, the NOR/PSRAM sub-banks and the two tasks over them - on
// every header of the pack. The chapter is the one place where the
// question "which peripheral does this part have" has THREE answers
// (the FMC, the FSMC, or neither), so the presence facts are asserted
// class by class here and the register-facing types are only reached
// where the header declares the block.
#include <type_traits>

#include "stm32f4/clock.hpp"
#include "stm32f4/fmc.hpp"

using namespace brio;

// ---- the reserve's presence facts ------------------------------------------------

// The two blocks are never both there, and neither is ever "both
// absent and present".
static_assert(!(fmc_present() && fsmc_present()));
static_assert(fmc_present() == (fmc_bank1_base() != 0u));
static_assert(fmc_present() == (fmc_clock_mask() != 0u));
static_assert(fmc_present() == (fmc_reset_mask() != 0u));
static_assert(fmc_static_banks() == (fmc_present() ? 4u : 0u));

#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx)
// The FMC in its widest shape: SDRAM, both NAND banks and a PC Card.
static_assert(fmc_present() && !fsmc_present());
static_assert(fmc_bank1_base() == FMC_Bank1_R_BASE);
static_assert(fmc_bank1e_base() == FMC_Bank1E_R_BASE);
static_assert(fmc_sdram_base() == FMC_Bank5_6_R_BASE);
static_assert(fmc_sdram_banks() == 2u);
static_assert(fmc_nand_banks() == 2u);
static_assert(fmc_nand_bank_present(2) && fmc_nand_bank_present(3));
static_assert(fmc_nand_base(2) == FMC_Bank2_3_R_BASE);
static_assert(fmc_nand_base(3) == FMC_Bank2_3_R_BASE + 0x20u);
static_assert(fmc_pccard_present() && fmc_pccard_base() == FMC_Bank4_R_BASE);
static_assert(fmc_clock_mask() == RCC_AHB3ENR_FMCEN);
static_assert(fmc_reset_mask() == RCC_AHB3RSTR_FMCRST);
static_assert(fmc_irq() == FMC_IRQn);
// RM0090 37.5.6 has WRAPMOD; RM0390 11.6.6 does not.
static_assert(fmc_wrapped_burst_mask() == FMC_BCR1_WRAPMOD);
// The windows of figure 457.
static_assert(fmc_static_window(1) == 0x60000000UL && fmc_static_window(4) == 0x6C000000UL);
static_assert(fmc_nand_window(2) == 0x70000000UL && fmc_nand_window(3) == 0x80000000UL);
static_assert(fmc_pccard_window() == 0x90000000UL);
static_assert(fmc_sdram_window(1) == 0xC0000000UL && fmc_sdram_window(2) == 0xD0000000UL);
static_assert(fmc_static_window(0) == 0u && fmc_static_window(5) == 0u);
static_assert(fmc_sdram_window(0) == 0u && fmc_sdram_window(3) == 0u);
#elif defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
// The FMC with the narrower static half: one NAND bank under a struct of
// its own, no PC Card, no WRAPMOD.
static_assert(fmc_present() && !fsmc_present());
static_assert(fmc_sdram_banks() == 2u && fmc_sdram_base() == FMC_Bank5_6_R_BASE);
static_assert(fmc_nand_banks() == 1u);
static_assert(!fmc_nand_bank_present(2) && fmc_nand_bank_present(3));
static_assert(fmc_nand_base(3) == FMC_Bank3_R_BASE);
static_assert(!fmc_pccard_present() && fmc_pccard_base() == 0u);
static_assert(fmc_nand_window(2) == 0u && fmc_nand_window(3) == 0x80000000UL);
static_assert(fmc_pccard_window() == 0u);
static_assert(fmc_wrapped_burst_mask() == 0u);
static_assert(fmc_irq() == FMC_IRQn);
#elif defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || \
    defined(STM32F417xx) || defined(STM32F412Rx) || defined(STM32F412Vx) ||   \
    defined(STM32F412Zx) || defined(STM32F413xx) || defined(STM32F423xx)
// The FSMC: another peripheral. Nothing of fmc.hpp exists here, and the
// reserve says which block the part really has rather than "none".
static_assert(!fmc_present() && fsmc_present());
static_assert(fmc_bank1_base() == 0u && fmc_sdram_base() == 0u);
static_assert(fmc_static_banks() == 0u && fmc_sdram_banks() == 0u);
static_assert(fmc_nand_banks() == 0u && !fmc_pccard_present());
static_assert(fmc_clock_mask() == 0u && fmc_reset_mask() == 0u);
static_assert(fmc_irq() == NonMaskableInt_IRQn);
// Every window is 0, so an address from the reserve is never a
// plausible-looking lie on a part that cannot answer at it.
static_assert(fmc_static_window(1) == 0u && fmc_sdram_window(1) == 0u);
static_assert(fmc_nand_window(2) == 0u && fmc_pccard_window() == 0u);
#else
// The F401, the F410, the F411 and the 48-pin F412: no external memory
// controller of either kind.
static_assert(!fmc_present() && !fsmc_present());
static_assert(fmc_static_banks() == 0u && fmc_sdram_banks() == 0u);
static_assert(fmc_clock_mask() == 0u);
static_assert(fmc_irq() == NonMaskableInt_IRQn);
#endif

// ---- the arithmetic, everywhere ---------------------------------------------------
//
// These are the chapter's formulas and the device's geometry: pure
// constexpr, so they hold on a part with no controller too.

static_assert(sdram_column_bits(SdramColumns::eight) == 8u);
static_assert(sdram_column_bits(SdramColumns::eleven) == 11u);
static_assert(sdram_row_bits(SdramRows::eleven) == 11u);
static_assert(sdram_row_bits(SdramRows::thirteen) == 13u);
static_assert(sdram_width_bytes(SdramWidth::bits8) == 1u);
static_assert(sdram_width_bytes(SdramWidth::bits16) == 2u);
static_assert(sdram_width_bytes(SdramWidth::bits32) == 4u);

// 4096 rows x 256 columns x 4 internal banks x 16 bits = 8 MB, the
// 64-Mbit device shape; and the chapter's own biggest, 256 MB.
constexpr SdramConfig eight_megabytes{.columns = SdramColumns::eight,
                                      .rows = SdramRows::twelve,
                                      .width = SdramWidth::bits16,
                                      .internal_banks = SdramInternalBanks::four,
                                      .cas = SdramCas::three};
static_assert(sdram_capacity_bytes(eight_megabytes) == 8UL * 1024UL * 1024UL);
constexpr SdramConfig biggest{.columns = SdramColumns::eleven,
                              .rows = SdramRows::thirteen,
                              .width = SdramWidth::bits32,
                              .internal_banks = SdramInternalBanks::four};
static_assert(sdram_capacity_bytes(biggest) == 256UL * 1024UL * 1024UL);

// The memory clock: HCLK over 2 or 3, and nothing at all when SDCLK is
// off.
static_assert(sdram_clock_hz(180'000'000u, SdramClock::hclk_div2) == 90'000'000u);
static_assert(sdram_clock_hz(180'000'000u, SdramClock::hclk_div3) == 60'000'000u);
static_assert(sdram_clock_hz(180'000'000u, SdramClock::off) == 0u);

// 37.7.5's worked example: 64 ms over 8196 rows at 60 MHz gives 448
// after the 20-cycle margin. (The chapter writes 8196 where it means
// 8192; the formula is what is being checked, so its own number is
// used.)
static_assert(sdram_refresh_count(60'000'000u, 64'000u, 8196u) == 448u);
// The same period over 4096 rows at 90 MHz - the geometry above.
static_assert(sdram_refresh_count(90'000'000u, 64'000u, 4096u) == 1386u);
static_assert(sdram_refresh_count(0u, 64'000u, 4096u) == 0u);
static_assert(sdram_refresh_count(90'000'000u, 64'000u, 0u) == 0u);

static_assert(!sdram_refresh_count_valid(0u));
static_assert(!sdram_refresh_count_valid(40u));
static_assert(sdram_refresh_count_valid(41u));
static_assert(sdram_refresh_count_valid(0x1FFFu));
static_assert(!sdram_refresh_count_valid(0x2000u));

// The device's own mode register: burst length 1, sequential, CAS 3,
// single-location writes - the word 37.7.3 step 7 asks for.
static_assert(sdram_mode_register(SdramCas::three) == 0x0230u);
static_assert(sdram_mode_register(SdramCas::two) == 0x0220u);
static_assert(sdram_mode_register(SdramCas::three, SdramBurstLength::four,
                                  SdramBurstType::interleaved,
                                  SdramWriteBurst::programmed) == 0x003Au);

// ---- the refusals -----------------------------------------------------------------

static_assert(sdram_config_valid(eight_megabytes));
static_assert(!sdram_config_valid({.rows = static_cast<SdramRows>(3)}));
static_assert(!sdram_config_valid({.width = static_cast<SdramWidth>(3)}));
static_assert(!sdram_config_valid({.cas = static_cast<SdramCas>(0)}));
static_assert(!sdram_config_valid({.clock = static_cast<SdramClock>(1)}));
static_assert(!sdram_config_valid({.read_pipe = static_cast<SdramPipe>(3)}));

constexpr SdramTiming ninety_mhz{.load_mode_to_active = 2,
                                 .exit_self_refresh = 7,
                                 .self_refresh = 4,
                                 .row_cycle = 6,
                                 .write_recovery = 2,
                                 .row_precharge = 2,
                                 .row_to_column = 2};
static_assert(sdram_timing_valid(ninety_mhz));
static_assert(!sdram_timing_valid({.load_mode_to_active = 0}));
static_assert(!sdram_timing_valid({.exit_self_refresh = 17}));
// TWR must cover what is left of TRAS after TRCD ...
static_assert(!sdram_timing_valid({.exit_self_refresh = 7,
                                   .self_refresh = 6,
                                   .row_cycle = 6,
                                   .write_recovery = 2,
                                   .row_precharge = 2,
                                   .row_to_column = 2}));
// ... and of TRC after TRCD and TRP. TRC = 7 with TRCD = TRP = TWR = 2
// is the value ST's own board software uses and the chapter forbids.
static_assert(!sdram_timing_valid({.exit_self_refresh = 7,
                                   .self_refresh = 4,
                                   .row_cycle = 7,
                                   .write_recovery = 2,
                                   .row_precharge = 2,
                                   .row_to_column = 2}));
static_assert(sdram_timing_valid({.exit_self_refresh = 7,
                                  .self_refresh = 4,
                                  .row_cycle = 7,
                                  .write_recovery = 3,
                                  .row_precharge = 2,
                                  .row_to_column = 2}));

// TWR + TRP + TRC + TRCD + 4 is the one COUNT the chapter forbids.
static_assert(sdram_forbidden_count(ninety_mhz) == 16u);
static_assert(sdram_init_valid({.refresh_count = 1386}, ninety_mhz));
static_assert(!sdram_init_valid({.refresh_cycles = 0}, ninety_mhz));
static_assert(!sdram_init_valid({.refresh_cycles = 16}, ninety_mhz));
static_assert(!sdram_init_valid({.mode_register = 0x2000}, ninety_mhz));
static_assert(!sdram_init_valid({.refresh_count = 40}, ninety_mhz));
// Legal as a count, forbidden by that one equality - and only because
// these are the timings in force.
constexpr SdramTiming forbids_41{.load_mode_to_active = 2,
                                 .exit_self_refresh = 7,
                                 .self_refresh = 4,
                                 .row_cycle = 16,
                                 .write_recovery = 5,
                                 .row_precharge = 8,
                                 .row_to_column = 8};
static_assert(sdram_timing_valid(forbids_41) && sdram_forbidden_count(forbids_41) == 41u);
static_assert(!sdram_init_valid({.refresh_count = 41}, forbids_41));
static_assert(sdram_init_valid({.refresh_count = 42}, forbids_41));

constexpr StaticConfig plain_sram{};
static_assert(static_config_valid(plain_sram, 1) && static_config_valid(plain_sram, 4));
static_assert(!static_config_valid(plain_sram, 0) && !static_config_valid(plain_sram, 5));
static_assert(!static_config_valid({.memory = static_cast<StaticMemory>(3)}, 1));
static_assert(!static_config_valid({.width = static_cast<StaticWidth>(3)}, 1));
static_assert(!static_config_valid({.page_size = static_cast<CramPageSize>(5)}, 1));
// MUXEN is NOR's and PSRAM's alone.
static_assert(!static_config_valid({.memory = StaticMemory::sram, .multiplexed = true}, 1));
static_assert(static_config_valid({.memory = StaticMemory::nor, .multiplexed = true}, 1));
// CPSIZE is Cellular RAM's alone.
static_assert(!static_config_valid({.memory = StaticMemory::nor,
                                    .page_size = CramPageSize::bytes256}, 1));
static_assert(static_config_valid({.memory = StaticMemory::psram,
                                   .page_size = CramPageSize::bytes256}, 1));
// CCLKEN is bank 1's field, and bank 1 must be synchronous to make the
// clock it enables.
static_assert(static_config_valid({.burst_read = true, .continuous_clock = true}, 1));
static_assert(!static_config_valid({.burst_read = true, .continuous_clock = true}, 2));
static_assert(!static_config_valid({.burst_read = false, .continuous_clock = true}, 1));
// WRAPMOD where the part has no such field.
static_assert(static_config_valid({.wrapped_burst = true}, 1) ==
              (fmc_wrapped_burst_mask() != 0u));

constexpr StaticTiming slowest{};
static_assert(static_timing_valid(slowest, plain_sram));
static_assert(!static_timing_valid({.address_hold = 0}, plain_sram));
static_assert(!static_timing_valid({.data_phase = 0}, plain_sram));
static_assert(!static_timing_valid({.clock_divide = 1}, plain_sram));
static_assert(!static_timing_valid({.clock_divide = 17}, plain_sram));
static_assert(!static_timing_valid({.data_latency = 1}, plain_sram));
static_assert(!static_timing_valid({.data_latency = 18}, plain_sram));
// ADDSET may be zero in the plain modes and not in a multiplexed one.
static_assert(static_timing_valid({.address_setup = 0}, plain_sram));
static_assert(!static_timing_valid({.address_setup = 0},
                                   {.memory = StaticMemory::nor, .multiplexed = true}));
static_assert(!static_timing_valid({.address_setup = 0, .access = StaticAccess::mode_d},
                                   {.extended_mode = true}));

// ---- the register-facing half -------------------------------------------------------

#if defined(FMC_Bank1_R_BASE)

// The clock this file needs is the CORE clock, for the power-up delay
// alone; the reset rate compiles on every part class, ladder or no
// ladder.
using Boot = Clock<ClockSource::hsi, 16'000'000>;

static_assert(Fmc::static_banks == 4u);
static_assert(Fmc::sdram_banks == fmc_sdram_banks());
static_assert(Fmc::nand_banks == fmc_nand_banks());
static_assert(Fmc::has_pc_card == fmc_pccard_present());
static_assert(Fmc::irq_line == FMC_IRQn);
static_assert(Fmc::static_window(2) == 0x64000000UL);
static_assert(Fmc::sdram_window(2) == 0xD0000000UL);

void block() {
    Fmc::clock(true);
    (void)Fmc::clock();
    Fmc::enable_interrupt();
    Fmc::disable_interrupt();
    Fmc::spin_at_least(64);
    Fmc::reset();
}

// The four static sub-banks: every verb of the resource and of the two
// tasks over it.
static_assert(FmcNorPsram<1>::base == 0x60000000UL);
static_assert(FmcNorPsram<4>::base == 0x6C000000UL);
static_assert(FmcNorPsram<1>::control_index == 0u && FmcNorPsram<1>::read_timing_index == 1u);
static_assert(FmcNorPsram<4>::control_index == 6u && FmcNorPsram<4>::read_timing_index == 7u);
static_assert(FmcNorPsram<3>::write_timing_index == 4u);
static_assert(FmcSram<2>::base == FmcNorPsram<2>::base);
static_assert(FmcNor<3>::base == FmcNorPsram<3>::base);

template <uint8_t bank>
void static_bank() {
    using R = FmcNorPsram<bank>;
    constexpr StaticConfig c{.memory = StaticMemory::psram,
                             .width = StaticWidth::bits16,
                             .burst_read = true,
                             .burst_write = true,
                             .wait_timing = WaitTiming::during,
                             .wait_polarity = WaitPolarity::active_low,
                             .async_wait = true,
                             .extended_mode = true,
                             .page_size = CramPageSize::bytes512};
    constexpr StaticTiming t{.address_setup = 3,
                             .address_hold = 2,
                             .data_phase = 6,
                             .bus_turnaround = 1,
                             .clock_divide = 4,
                             .data_latency = 3,
                             .access = StaticAccess::mode_b};
    (void)R::configure(c);
    (void)R::timing(t, c);
    (void)R::write_timing(t, c);
    R::enable();
    (void)R::enabled();
    (void)R::control_word();
    (void)R::timing_word();
    (void)R::write_timing_word();
    R::disable();
    (void)R::template at<uint16_t>(0x10);
    (void)&R::regs();
    (void)&R::write_regs();

    (void)FmcSram<bank>::init(StaticWidth::bits8, t);
    (void)FmcSram<bank>::init(StaticWidth::bits32, t, t);
    (void)FmcSram<bank>::template at<uint32_t>(4);
    FmcSram<bank>::release();
    (void)FmcNor<bank>::init(StaticWidth::bits16, t);
    (void)FmcNor<bank>::init_multiplexed(StaticWidth::bits16, t);
    (void)FmcNor<bank>::init_burst(StaticWidth::bits16, t);
    (void)FmcNor<bank>::template at<uint16_t>(2);
    FmcNor<bank>::release();
}

void static_banks() {
    static_bank<1>();
    static_bank<2>();
    static_bank<3>();
    static_bank<4>();
}

#if defined(FMC_Bank5_6_R_BASE)

static_assert(FmcSdram<1>::base == 0xC0000000UL);
static_assert(FmcSdram<2>::base == 0xD0000000UL);
static_assert(FmcSdram<1>::window_bytes == 0x10000000UL);
static_assert(std::is_same_v<decltype(FmcSdram<2>::at<uint16_t>(0)), volatile uint16_t*>);

template <uint8_t bank>
void sdram_bank() {
    using S = FmcSdram<bank>;
    (void)S::configure(eight_megabytes);
    (void)S::timing(ninety_mhz);
    (void)S::control_word();
    (void)S::timing_word();
    S::write_protect(true);
    (void)S::write_protect();
    (void)S::busy();
    (void)S::wait_ready();
    (void)S::wait_ready(16);
    (void)S::command(SdramCommand::clock_enable);
    (void)S::command(SdramCommand::auto_refresh, 8);
    (void)S::command(SdramCommand::load_mode, 1, sdram_mode_register(SdramCas::three));
    (void)S::command(SdramCommand::precharge_all, 1, 0, true);
    (void)S::state();
    (void)S::self_refresh();
    (void)S::power_down(true);
    (void)S::normal();
    (void)S::refresh_rate(1386u);
    (void)S::refresh_rate(1386u, ninety_mhz);
    (void)S::refresh_rate();
    (void)S::memory_clock(SdramClock::hclk_div2);
    (void)S::memory_clock();
    S::refresh_interrupt(true);
    (void)S::refresh_interrupt();
    (void)S::refresh_error();
    S::clear_refresh_error();
    (void)S::refresh_isr();
    (void)S::initialize(Boot{}, eight_megabytes, ninety_mhz,
                        SdramInit{.power_up_us = 100,
                                  .refresh_cycles = 8,
                                  .mode_register = sdram_mode_register(SdramCas::three),
                                  .refresh_count = 1386});
    (void)S::template at<uint8_t>(0);
    (void)S::template at<uint32_t>(4);
    (void)&S::regs();
}

void sdram_banks() {
    sdram_bank<1>();
    sdram_bank<2>();
}

#endif   // FMC_Bank5_6_R_BASE
#endif   // FMC_Bank1_R_BASE
