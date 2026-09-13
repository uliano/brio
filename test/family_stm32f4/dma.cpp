// DMA family smoke TU (RM0090 ch. 10, RM0390 ch. 9, RM0383 ch. 9). The
// controller itself is the uniform half of this chapter - two blocks,
// eight streams each, eight channels a stream, a four-word FIFO, one
// vector per stream, on every part of the pack - so what this TU really
// checks is the two halves that are NOT uniform: what the header says
// (the bases, the gates, the sixteen vectors) and what only a reference
// manual can say (which cell of the request mapping carries a serial
// instance, and that a part class whose manual was not read has no cells
// at all).
//
// A family fixture is the one place that may ask the header a question
// TWICE, by two independent symbols, and assert the two answers agree.
#include <stdint.h>

#include "stm32f4/dma.hpp"
#include "stm32f4/usart.hpp"

using namespace brio;

// ---- the reserve's own answers ----------------------------------------------

static_assert(dma_streams == 8, "eight streams per controller");
static_assert(dma_channels == 8, "CHSEL is three bits");
static_assert(dma_fifo_words == 4, "each stream's FIFO is four words");

static_assert(dma_present(1) && dma_present(2), "both controllers, on every part");
static_assert(!dma_present(0) && !dma_present(3));
static_assert(dma_base(1) != dma_base(2));

static_assert(dma_stream_present(1, 0) && dma_stream_present(2, 7));
static_assert(!dma_stream_present(1, 8) && !dma_stream_present(3, 0));

// The stream bases are 0x18 apart from stream 0 of their controller, and
// stream 0 sits 0x10 past the block - the offsets 10.5.5 gives. Derived
// here from the header's OWN macros, which is what makes the driver's
// per-stream addressing a checked fact.
static_assert(dma_stream_base(1, 0) == dma_base(1) + 0x10u);
static_assert(dma_stream_base(1, 7) == dma_base(1) + 0x10u + 7u * 0x18u);
static_assert(dma_stream_base(2, 3) == dma_base(2) + 0x10u + 3u * 0x18u);

// The gate and the reset line share a bit position, as every AHB1
// peripheral of this family does.
static_assert(dma_clock_mask(1) == dma_reset_mask(1));
static_assert(dma_clock_mask(2) == dma_reset_mask(2));
static_assert(dma_clock_mask(1) != dma_clock_mask(2));
static_assert(dma_clock_mask(3) == 0u);

// ONE VECTOR PER STREAM, sixteen of them, all distinct - and nothing for
// a controller or a stream that does not exist.
static_assert(dma_stream_irq(1, 0) != dma_stream_irq(1, 1));
static_assert(dma_stream_irq(1, 7) != dma_stream_irq(2, 7));
static_assert(dma_stream_irq(3, 0) == NonMaskableInt_IRQn);
static_assert(dma_stream_irq(1, 8) == NonMaskableInt_IRQn);

// Memory-to-memory is DMA2's alone (figures 33/34, note 1) - a wiring
// fact no register carries.
static_assert(!dma_memory_to_memory_capable(1));
static_assert(dma_memory_to_memory_capable(2));

// ---- the vocabulary ----------------------------------------------------------

static_assert(dma_width_bytes(DmaWidth::byte) == 1);
static_assert(dma_width_bytes(DmaWidth::half) == 2);
static_assert(dma_width_bytes(DmaWidth::word) == 4);
static_assert(dma_width_of<uint8_t>() == DmaWidth::byte);
static_assert(dma_width_of<uint16_t>() == DmaWidth::half);
static_assert(dma_width_of<uint32_t>() == DmaWidth::word);

static_assert(dma_burst_beats(DmaBurst::single) == 1);
static_assert(dma_burst_beats(DmaBurst::incr4) == 4);
static_assert(dma_burst_beats(DmaBurst::incr8) == 8);
static_assert(dma_burst_beats(DmaBurst::incr16) == 16);

static_assert(dma_fifo_threshold_bytes(DmaFifoThreshold::quarter) == 4);
static_assert(dma_fifo_threshold_bytes(DmaFifoThreshold::half) == 8);
static_assert(dma_fifo_threshold_bytes(DmaFifoThreshold::three_quarters) == 12);
static_assert(dma_fifo_threshold_bytes(DmaFifoThreshold::full) == 16);

// The five flags of a stream's six-bit group, bit 1 reserved (10.5.1).
static_assert(DmaFlag::fifo_error == 0x01u);
static_assert(DmaFlag::direct_mode_error == 0x04u);
static_assert(DmaFlag::transfer_error == 0x08u);
static_assert(DmaFlag::half == 0x10u);
static_assert(DmaFlag::complete == 0x20u);
static_assert(DmaFlag::all == 0x3Du);
static_assert(DmaFlag::errors == 0x0Du);

// The group shifts are the register's, and the driver's shift table is
// checked against the header's own bit positions.
static_assert(Dma<1>::flag_shift(0) == DMA_LISR_FEIF0_Pos);
static_assert(Dma<1>::flag_shift(1) == DMA_LISR_FEIF1_Pos);
static_assert(Dma<1>::flag_shift(2) == DMA_LISR_FEIF2_Pos);
static_assert(Dma<1>::flag_shift(3) == DMA_LISR_FEIF3_Pos);
static_assert(Dma<1>::flag_shift(4) == DMA_HISR_FEIF4_Pos);
static_assert(Dma<1>::flag_shift(7) == DMA_HISR_FEIF7_Pos);
static_assert(DmaFlag::complete << Dma<1>::flag_shift(3) == DMA_LISR_TCIF3);
static_assert(DmaFlag::transfer_error << Dma<1>::flag_shift(6) == DMA_HISR_TEIF6);
static_assert(DmaFlag::half << Dma<1>::flag_shift(5) == DMA_HISR_HTIF5);
static_assert(DmaFlag::direct_mode_error << Dma<1>::flag_shift(2) == DMA_LISR_DMEIF2);

// ---- TABLE 49, as the predicate ----------------------------------------------
//
// Every cell of the FIFO threshold table, the forbidden ones included.
// The driver carries the arithmetic, not the table; these are the table.

// Byte memory accesses.
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::quarter, DmaBurst::incr4, DmaWidth::byte,
                                   DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::quarter, DmaBurst::incr8, DmaWidth::byte,
                                    DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::quarter, DmaBurst::incr16, DmaWidth::byte,
                                    DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::half, DmaBurst::incr4, DmaWidth::byte,
                                   DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::half, DmaBurst::incr8, DmaWidth::byte,
                                   DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::half, DmaBurst::incr16, DmaWidth::byte,
                                    DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::three_quarters, DmaBurst::incr4,
                                   DmaWidth::byte, DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::three_quarters, DmaBurst::incr8,
                                    DmaWidth::byte, DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::incr4, DmaWidth::byte,
                                   DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::incr8, DmaWidth::byte,
                                   DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::incr16, DmaWidth::byte,
                                   DmaBurst::single, DmaWidth::byte));

// Half-word memory accesses.
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::quarter, DmaBurst::incr4, DmaWidth::half,
                                    DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::half, DmaBurst::incr4, DmaWidth::half,
                                   DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::half, DmaBurst::incr8, DmaWidth::half,
                                    DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::three_quarters, DmaBurst::incr4,
                                    DmaWidth::half, DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::incr4, DmaWidth::half,
                                   DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::incr8, DmaWidth::half,
                                   DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::incr16, DmaWidth::half,
                                    DmaBurst::single, DmaWidth::byte));

// Word memory accesses: only the full threshold with INCR4 survives.
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::quarter, DmaBurst::incr4, DmaWidth::word,
                                    DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::half, DmaBurst::incr4, DmaWidth::word,
                                    DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::three_quarters, DmaBurst::incr4,
                                    DmaWidth::word, DmaBurst::single, DmaWidth::byte));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::incr4, DmaWidth::word,
                                   DmaBurst::single, DmaWidth::byte));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::incr8, DmaWidth::word,
                                    DmaBurst::single, DmaWidth::byte));

// The note under the table: a peripheral burst that fills the whole FIFO
// leaves too little room at a 3/4 threshold.
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::three_quarters, DmaBurst::single,
                                    DmaWidth::byte, DmaBurst::incr4, DmaWidth::word));
static_assert(dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::single, DmaWidth::byte,
                                   DmaBurst::incr4, DmaWidth::word));
static_assert(!dma_fifo_burst_valid(DmaFifoThreshold::full, DmaBurst::single, DmaWidth::byte,
                                    DmaBurst::incr8, DmaWidth::word));

// ---- table 50's combinations -------------------------------------------------

constexpr DmaStreamConfig mem2mem{
    .direction = DmaDirection::memory_to_memory,
    .peripheral_increment = true,
    .memory_increment = true,
    .use_fifo = true,
    .fifo_threshold = DmaFifoThreshold::full,
};
static_assert(dma_stream_config_valid(mem2mem, 2), "DMA2 reaches the bus matrix");
static_assert(!dma_stream_config_valid(mem2mem, 1), "DMA1's peripheral port does not");

constexpr DmaStreamConfig mem2mem_direct{
    .direction = DmaDirection::memory_to_memory,
    .peripheral_increment = true,
    .use_fifo = false,
};
static_assert(!dma_stream_config_valid(mem2mem_direct, 2),
              "10.3.6: direct mode must not be used for memory-to-memory");

constexpr DmaStreamConfig mem2mem_circular{
    .direction = DmaDirection::memory_to_memory,
    .circular = true,
    .peripheral_increment = true,
    .use_fifo = true,
    .fifo_threshold = DmaFifoThreshold::full,
};
static_assert(!dma_stream_config_valid(mem2mem_circular, 2), "table 50: circular is forbidden");

constexpr DmaStreamConfig flow_circular{
    .circular = true,
    .peripheral_flow_control = true,
};
static_assert(!dma_stream_config_valid(flow_circular, 1),
              "10.3.15: circular is forbidden under the peripheral flow controller");

constexpr DmaStreamConfig direct_mixed_widths{
    .peripheral_width = DmaWidth::byte,
    .memory_width = DmaWidth::word,
    .use_fifo = false,
};
static_assert(!dma_stream_config_valid(direct_mixed_widths, 1),
              "direct mode: one width, PSIZE's");

constexpr DmaStreamConfig direct_burst{
    .memory_burst = DmaBurst::incr4,
    .use_fifo = false,
};
static_assert(!dma_stream_config_valid(direct_burst, 1), "direct mode cannot burst");

constexpr DmaStreamConfig channel_nine{.channel = 8};
static_assert(!dma_stream_config_valid(channel_nine, 1));

constexpr DmaStreamConfig plain_rx{
    .channel = 4,
    .direction = DmaDirection::peripheral_to_memory,
    .memory_increment = true,
};
static_assert(dma_stream_config_valid(plain_rx, 1) && dma_stream_config_valid(plain_rx, 2));

// ---- a whole transfer --------------------------------------------------------

alignas(4) uint8_t bytes_a[64];
alignas(4) uint8_t bytes_b[64];
uint16_t words[8];

static_assert(dma_address_aligned(0x20000004u, DmaWidth::word));
static_assert(!dma_address_aligned(0x20000002u, DmaWidth::word));
static_assert(dma_address_aligned(0x20000002u, DmaWidth::half));
static_assert(!dma_address_aligned(0x20000001u, DmaWidth::half));
static_assert(dma_address_aligned(0x20000001u, DmaWidth::byte));

static_assert(dma_flow_control_count == 0xFFFFu);

// ---- the block and the stream, every verb ------------------------------------

using Block = Dma<2>;
using Stream = DmaStream<2, 0>;
using Slave = DmaStream<1, 3>;

static_assert(Block::instance == 2);
static_assert(Block::streams == 8);
static_assert(Block::memory_to_memory);
static_assert(!Dma<1>::memory_to_memory);
static_assert(Stream::controller == 2 && Stream::index == 0);
static_assert(Stream::memory_to_memory_capable && !Slave::memory_to_memory_capable);
static_assert(Stream::irq() == dma_stream_irq(2, 0));

void block_verbs() {
    Block::init();
    Block::bus_clock(true);
    (void)Block::bus_clock();
    Block::reset();
    (void)Block::regs().LISR;
    (void)Block::low_flags();
    (void)Block::high_flags();
    (void)Block::stream_flags(5);
    Block::clear(5, DmaFlag::all);
    (void)Block::irq(5);
}

void stream_verbs() {
    DmaTransfer t{};
    t.peripheral = bytes_a;
    t.memory = bytes_b;
    t.memory1 = nullptr;
    t.count = 64;
    t.config = mem2mem;

    (void)Stream::regs().NDTR;
    (void)Stream::enabled();
    (void)Stream::enable();
    (void)Stream::abort();
    (void)Stream::abort(10);
    (void)Stream::configure(t.config);
    (void)Stream::control();
    (void)Stream::fifo_control();
    (void)Stream::channel();
    (void)Stream::circular();
    (void)Stream::double_buffer();
    (void)Stream::flow_controlled();
    (void)Stream::direct_mode();
    (void)Stream::direction();
    (void)Stream::fifo_status();
    (void)Stream::set_count(64);
    (void)Stream::count();
    (void)Stream::set_peripheral(bytes_a);
    (void)Stream::set_memory(bytes_b);
    (void)Stream::set_memory1(bytes_a);
    (void)Stream::current_target_is_m1();
    (void)Stream::set_idle_buffer(bytes_a);
    (void)Stream::prepare(t);
    (void)Stream::load(t);
    (void)Stream::trigger();
    (void)Stream::flags();
    (void)Stream::flag(DmaFlag::complete);
    Stream::clear(DmaFlag::all);
    Stream::arm(DmaFlag::all, true);
    (void)Stream::armed();
    (void)Stream::isr();
    (void)Stream::progress(64);
    Stream::stop();
}

/// The double buffer and the flow controller, which no other letter of
/// this fixture reaches: both are whole-transfer shapes and both have
/// their own validity.
void double_buffer_and_flow() {
    DmaTransfer t{};
    t.peripheral = words;
    t.memory = bytes_a;
    t.memory1 = bytes_b;
    t.count = 8;
    t.config.channel = 4;
    t.config.direction = DmaDirection::peripheral_to_memory;
    t.config.double_buffer = true;
    t.config.memory_increment = true;
    t.config.peripheral_width = DmaWidth::half;
    t.config.memory_width = DmaWidth::half;
    (void)Slave::prepare(t);

    DmaTransfer f{};
    f.peripheral = words;
    f.memory = bytes_a;
    f.count = 0;   // forced to 0xFFFF by the silicon when the peripheral leads
    f.config.channel = 4;
    f.config.peripheral_flow_control = true;
    f.config.memory_increment = true;
    (void)Slave::prepare(f);
    (void)Slave::progress(dma_flow_control_count);
}

// ---- the engines, and the request map they are checked against ---------------

using Tx = DmaTxEngine<2, 7, 4>;
using Rx = DmaRxEngine<2, 2, 4>;
using Wide = DmaTxEngine<2, 6, 5, uint16_t>;

static_assert(Tx::present && Rx::present && Wide::present);
static_assert(Tx::controller == 2 && Tx::stream == 7 && Tx::channel == 4);
static_assert(Tx::width == DmaWidth::byte && Wide::width == DmaWidth::half);
static_assert(Tx::flag_complete == DmaFlag::complete);
static_assert(Tx::flag_error == DmaFlag::transfer_error);
static_assert(Rx::flag_fifo_error == DmaFlag::fifo_error);

// Two engines of one transport must not share a stream - a stream has one
// direction and one FIFO. Nothing in the serial request map puts a
// transmit and a receive request on the same stream, so this is a rule
// the fixture states rather than a case the map can produce.
static_assert(uart_engines_distinct<Tx, Rx>());
static_assert(!uart_engines_distinct<DmaTxEngine<2, 7, 4>, DmaRxEngine<2, 7, 4>>());
static_assert(uart_engines_distinct<NoDmaEngine, NoDmaEngine>());
static_assert(uart_engines_distinct<Tx, NoDmaEngine>());

void engine_verbs() {
    static uint8_t buffer[32];
    static uint8_t cell = 0;
    Tx::arm(bytes_a, DmaPriority::high);
    (void)Tx::service();
    (void)Tx::start(buffer, 32);
    (void)Tx::start_fixed(&cell, 4);
    (void)Tx::complete();
    (void)Tx::busy();
    (void)Tx::in_flight();
    (void)Tx::progress();
    (void)Tx::abandon();
    (void)Tx::faults();
    Tx::clear_faults();
    Tx::stop();

    Rx::arm(bytes_a);
    (void)Rx::service();
    (void)Rx::idle();
    (void)Rx::start(buffer, 32);
    (void)Rx::start_discard(&cell, 4);
    (void)Rx::take();
    (void)Rx::full();
    (void)Rx::capacity();
    (void)Rx::taken();
    (void)Rx::abandon();
    (void)Rx::faults();
    Rx::clear_faults();
    Rx::stop();
}

// ---- the request map ---------------------------------------------------------
//
// THE MAP IS A REFERENCE MANUAL'S AND NOT A HEADER'S, so what a fixture
// can check is the shape and the refusals: a cell of a part class whose
// manual was read is accepted, every other cell of that instance is not,
// and a class without a table accepts nothing at all.

constexpr DmaPlacements usart1_rx = usart_dma_placements(1, false);
constexpr DmaPlacements usart1_tx = usart_dma_placements(1, true);

static_assert(usart1_rx.known == usart1_tx.known, "a class has a table or it has not");

// Where the table IS known, USART1 is on DMA2 channel 4 in all three
// manuals read - two receive cells, one transmit.
static_assert(!usart1_rx.known || usart1_rx.count == 2);
static_assert(!usart1_tx.known || usart1_tx.count == 1);
static_assert(!usart1_rx.known || usart_dma_placement_valid(1, false, 2, 2, 4));
static_assert(!usart1_rx.known || usart_dma_placement_valid(1, false, 2, 5, 4));
static_assert(!usart1_tx.known || usart_dma_placement_valid(1, true, 2, 7, 4));
// The transmit cell is not a receive cell and the other controller is not
// this one: the check is a cell, never a channel number on its own.
static_assert(!usart1_rx.known || !usart_dma_placement_valid(1, false, 2, 7, 4));
static_assert(!usart1_tx.known || !usart_dma_placement_valid(1, true, 1, 7, 4));
static_assert(!usart1_tx.known || !usart_dma_placement_valid(1, true, 2, 7, 5));

// AN INSTANCE THE PART HAS NOT GOT HAS NO CELLS, whatever its class's
// table says about the instance's number - the two derivations, presence
// and placement, are checked against each other here.
static_assert(usart_present(3) || usart_dma_placements(3, true).count == 0);
static_assert(usart_present(8) || usart_dma_placements(8, false).count == 0);
static_assert(usart_dma_placements(0, true).count == 0);
static_assert(usart_dma_placements(11, true).count == 0);

// A class with no table refuses every cell, the right one included.
static_assert(usart1_tx.known || !usart_dma_placement_valid(1, true, 2, 7, 4));

// ---- an engined transport ----------------------------------------------------
//
// A TEMPLATE, so the branch is not instantiated where the class has no
// request table: the Uart's static_assert would refuse it there, which is
// exactly what neg/uart_engine_on_an_unread_class.cpp checks. The engines
// ride template parameters of their own, because a construct that does
// NOT depend on the parameter is checked at the definition and a
// discarded statement would not save it.
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7},
                                .rx = {'A', 10, PinFunction::af7}};

template <bool mapped, typename TxE = Tx, typename RxE = Rx>
void engined_console() {
    if constexpr (mapped) {
        using Serial = Uart<1, console_pins, 64, 256, TxE, RxE>;
        static_assert(Serial::has_tx_engine && Serial::has_rx_engine);
        (void)Serial::init(Clock<ClockSource::hsi, 16'000'000>{}, 115200);
        (void)Serial::dma_isr();
        (void)Serial::harvest();
        (void)Serial::write_byte('x');
        (void)Serial::dma_faults();
        Serial::clear_errors();
        Serial::release();
    }
}

// The engineless transport says so, and its harvest and fault count are
// free constants - the branch that must disappear from every console.
using Plain = Uart<1, console_pins>;
static_assert(!Plain::has_tx_engine && !Plain::has_rx_engine);

void engined() {
    engined_console<usart_dma_placements(1, true).known>();
    (void)Plain::harvest();
    (void)Plain::dma_faults();
}
