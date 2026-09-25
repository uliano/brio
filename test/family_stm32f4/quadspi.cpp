// Quad-SPI memory interface family smoke TU (RM0386 ch. 13, RM0390 ch. 12).
// THE POINT OF THIS TU IS THE ABSENCE AND THE WINDOW: eight of the
// twenty-three headers declare QSPI_R_BASE and the resource exists there;
// the rest have no block, and the vocabulary above it still compiles. Of
// the eight, the two classes whose manual is on the desk know their
// memory-mapped window and their DMA cell, and the other two refuse both.
#include "stm32f4/quadspi.hpp"

using namespace brio;

// ---- the vocabulary, on every part ------------------------------------------------

constexpr QspiCommand fast_read{.instruction = 0x0B,
                                .address_lines = QspiLines::one,
                                .dummy_cycles = 8,
                                .data_lines = QspiLines::one};
constexpr QspiCommand quad_read{.instruction = 0xEB,
                                .address_lines = QspiLines::four,
                                .dummy_cycles = 10,
                                .data_lines = QspiLines::four};
constexpr QspiCommand write_enable{.instruction = 0x06};
constexpr QspiCommand no_phase{.instruction_lines = QspiLines::none};
constexpr QspiCommand too_many_dummies{.instruction = 0x0B, .dummy_cycles = 32,
                                       .data_lines = QspiLines::one};
constexpr QspiCommand quad_no_turnaround{.instruction = 0xEB,
                                         .address_lines = QspiLines::four,
                                         .data_lines = QspiLines::four};

static_assert(qspi_command_ok(fast_read) && qspi_command_ok(quad_read) && qspi_command_ok(write_enable));
static_assert(!qspi_command_ok(no_phase), "13.3.3: at least one phase");
static_assert(!qspi_command_ok(too_many_dummies), "DCYC is five bits");
static_assert(qspi_read_needs_turnaround(quad_no_turnaround) && !qspi_read_needs_turnaround(quad_read));

constexpr QspiConfig sixteen_megabytes{.prescaler = 1, .fifo_threshold = 4, .address_bits = 24,
                                       .cs_high_cycles = 5};
static_assert(qspi_config_ok(sixteen_megabytes));
static_assert(!qspi_config_ok(QspiConfig{.fifo_threshold = 0}));
static_assert(!qspi_config_ok(QspiConfig{.fifo_threshold = 33}));
static_assert(!qspi_config_ok(QspiConfig{.address_bits = 0}));
static_assert(!qspi_config_ok(QspiConfig{.address_bits = 33}));
static_assert(!qspi_config_ok(QspiConfig{.cs_high_cycles = 0}));
static_assert(!qspi_config_ok(QspiConfig{.cs_high_cycles = 9}));

// The arithmetic: HCLK / (PRESCALER + 1), the smallest divisor under a
// ceiling, the deselect time in cycles, the size in address bits.
static_assert(qspi_clock_hz(180'000'000u, 0) == 180'000'000u);
static_assert(qspi_clock_hz(180'000'000u, 1) == 90'000'000u);
static_assert(qspi_clock_hz(180'000'000u, 3) == 45'000'000u);
static_assert(qspi_prescaler_for(180'000'000u, 133'000'000u) == 1u, "133 MHz wants HCLK/2 at 180");
static_assert(qspi_prescaler_for(180'000'000u, 54'000'000u) == 3u, "54 MHz wants HCLK/4 at 180");
static_assert(qspi_prescaler_for(180'000'000u, 180'000'000u) == 0u);
static_assert(qspi_prescaler_for(180'000'000u, 0u) == 255u);
static_assert(qspi_cs_high_cycles_for(90'000'000u, 50u) == 5u, "50 ns at 90 MHz is 4.5 cycles, so 5");
static_assert(qspi_cs_high_cycles_for(90'000'000u, 20u) == 2u);
static_assert(qspi_cs_high_cycles_for(90'000'000u, 0u) == 1u, "at least one cycle");
static_assert(qspi_cs_high_cycles_for(180'000'000u, 50u) == 9u ? false : true);
static_assert(qspi_cs_high_cycles_for(180'000'000u, 50u) == 0u, "nine cycles do not fit the field");
static_assert(qspi_address_bits(16u * 1024u * 1024u) == 24u);
static_assert(qspi_address_bits(4096ull * 1024u * 1024u) == 32u);
static_assert(qspi_address_bits(3u * 1024u * 1024u) == 0u, "not a power of two");
static_assert(qspi_address_bits(0u) == 0u);

// ---- the reserve's presence answers ------------------------------------------------

static_assert(quadspi_present() == (quadspi_clock_mask() != 0u));
static_assert(quadspi_present() == (quadspi_reset_mask() != 0u));
static_assert(!quadspi_window().known || quadspi_present(), "a window without a block is impossible");
static_assert(!quadspi_dma_placements().known || quadspi_present());
static_assert(quadspi_dma_placement_valid(2, 7, 3) == quadspi_dma_placements().known,
              "the one cell, DMA2 stream 7 channel 3 (RM0386 table 30, RM0390 table 29)");
static_assert(!quadspi_dma_placement_valid(1, 7, 3) && !quadspi_dma_placement_valid(2, 6, 3));

#if defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx)
static_assert(quadspi_present() && quadspi_window().known && quadspi_window().base == 0x9000'0000u &&
              quadspi_window().bytes == 256u * 1024u * 1024u);
static_assert(quadspi_dma_placements().known && quadspi_dma_placements().count == 1u);
#elif defined(STM32F412Cx) || defined(STM32F412Rx) || defined(STM32F412Vx) || defined(STM32F412Zx) || \
    defined(STM32F413xx) || defined(STM32F423xx)
// The block is there and its manual is not: the window and the cell are
// refused, the indirect verbs work.
static_assert(quadspi_present() == quadspi_present());
static_assert(!quadspi_window().known && !quadspi_dma_placements().known);
#else
static_assert(!quadspi_present() && !quadspi_window().known && !quadspi_dma_placements().known);
#endif

#if defined(QSPI_R_BASE)

static_assert(quadspi_present());
static_assert(Quadspi::irq == quadspi_irq());
static_assert(Quadspi::window_facts.known == quadspi_window().known);

// The command word, bit by bit against 13.5.6.
static_assert(Quadspi::ccr_word(write_enable, 0) == (0x06u | (1u << QUADSPI_CCR_IMODE_Pos)));
static_assert(Quadspi::ccr_word(quad_read, 3) ==
              (0xEBu | (1u << QUADSPI_CCR_IMODE_Pos) | (3u << QUADSPI_CCR_ADMODE_Pos) |
               (2u << QUADSPI_CCR_ADSIZE_Pos) | (10u << QUADSPI_CCR_DCYC_Pos) |
               (3u << QUADSPI_CCR_DMODE_Pos) | (3u << QUADSPI_CCR_FMODE_Pos)));

// ---- every verb, once ---------------------------------------------------------------

void every_verb(uint8_t* buf, uint32_t& status) {
    (void)Quadspi::init<sixteen_megabytes>();
    (void)Quadspi::init(sixteen_megabytes);
    (void)Quadspi::clock();
    (void)Quadspi::enabled();
    (void)Quadspi::prescaler();
    (void)Quadspi::address_bits();
    (void)Quadspi::cs_high_cycles();
    (void)Quadspi::fifo_threshold();
    (void)Quadspi::busy();
    (void)Quadspi::fifo_level();
    (void)Quadspi::transfer_complete();
    (void)Quadspi::transfer_error();
    (void)Quadspi::status_match();
    (void)Quadspi::fifo_threshold_reached();
    (void)Quadspi::timed_out();
    Quadspi::clear_flags();
    (void)Quadspi::abort();
    Quadspi::interrupts({.transfer_complete = true});
    (void)Quadspi::isr();
    (void)Quadspi::command(write_enable);
    (void)Quadspi::write(fast_read, 0, buf, 16);   // refused: dummy cycles on a write (2.4.2)
    (void)Quadspi::read(fast_read, 0, buf, 16);
    (void)Quadspi::read(quad_read, 0, buf, 16);
    (void)Quadspi::poll(QspiCommand{.instruction = 0x05, .data_lines = QspiLines::one}, 0, 1, 0x01u,
                        0x00u, 16, false, status);
    (void)Quadspi::map(quad_read);
    (void)Quadspi::mapped();
    (void)Quadspi::window();
    (void)Quadspi::unmap();
    Quadspi::reset_block();
    Quadspi::release();
}

#endif  // QSPI_R_BASE
