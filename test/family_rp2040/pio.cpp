// PIO family smoke TU: the assembler, a program, the block, a machine,
// the four tasks.
#include "rp2040/clock.hpp"
#include "rp2040/dma.hpp"
#include "rp2040/pio.hpp"

using namespace brio;

static_assert(pio_instruction_count == 32u && pio_sm_count == 4u && pio_fifo_depth == 4u);
static_assert(pio_is_jmp(pio_jmp(PioJmp::pin, 3)) && !pio_is_jmp(pio_set(PioSetTo::y, 1)));
static_assert(PioSideSet{.count = 5}.delay_bits() == 0u && PioSideSet{.count = 2, .optional = true}.data_bits() == 1u);
static_assert(!PioSideSet{.count = 6}.valid());
static_assert(PioClockDiv{}.x256() == 256u && PioClockDiv{0, 0}.x256() == 65536u * 256u && !PioClockDiv{0, 5}.valid());
static_assert(pio_sm_config_valid({}) && !pio_sm_config_valid({.pull_threshold = 33}) && !pio_sm_config_valid({.set_count = 6}));
static_assert((pio_pinctrl_of({.set_base = 17, .set_count = 1}) & PIO_SM0_PINCTRL_SET_BASE_BITS) == (17u << PIO_SM0_PINCTRL_SET_BASE_LSB));
static_assert(PioInterrupt::rx_not_empty(3) == 0x8u && PioInterrupt::tx_not_full(0) == 0x10u && PioInterrupt::flag(2) == 0x400u);
static_assert(Pio<1>::irq(1) == PIO1_IRQ_1_IRQn && Pio<0>::dreq_tx(1) == Dreq::pio0_tx1);

using SysClock = Clock<ClockSource::pll, 125'000'000>;
using Tx = PioUartTx<0, 0, 13>;
using Rx = PioUartRx<0, 1, 15>;
using Wave = PioSquareWave<1, 0, 17>;
using Pwm1 = PioPwm<1, 1, 12, 999>;
using TxEngine = DmaTxEngine<9, uint8_t>;

constexpr PioProgram<2> tiny = [] {
    PioProgram<2> p{};
    p.code = {pio_set(PioSetTo::x, 5), pio_jmp(PioJmp::always, 0)};
    return p;
}();
static_assert(tiny.valid());

uint8_t bytes[16];

void pio_verbs() {
    constexpr SysClock clock;
    using B = Pio<0>;
    using S = PioSm<0, 2>;
    (void)B::reset();
    B::hold();
    (void)B::load(tiny, 4);
    (void)B::place(tiny);
    (void)B::add(tiny);
    B::unload(4, 2);
    (void)B::used_slots();
    B::enable(0x4);
    B::disable(0x4);
    (void)B::enabled();
    B::restart(0x4);
    B::restart_clocks(0xF);
    (void)B::fifo_status();
    (void)B::fifo_debug();
    B::clear_fifo_debug(0xFFFFFFFFu);
    (void)B::fifo_levels();
    (void)B::memory_size();
    (void)B::machine_count();
    (void)B::fifo_depth();
    B::sync_bypass(0);
    (void)B::flags();
    (void)B::flag(4);
    B::clear_flags(0xFF);
    B::raise_flags(0x01);
    B::interrupts(0, PioInterrupt::rx_not_empty(2), true);
    B::interrupts_only(1, 0);
    (void)B::raw_pending();
    (void)B::pending(0);
    B::force(0, PioInterrupt::flag(0), false);
    (void)B::isr(0);

    (void)S::configure({.clock = {2, 0}, .set_base = 12, .set_count = 1});
    (void)S::init(tiny, 4, {});
    S::enable(true);
    (void)S::enabled();
    S::restart();
    S::clock({3, 128});
    (void)S::exec(pio_nop());
    (void)S::exec_wait(pio_set(PioSetTo::x, 1));
    (void)S::exec_stalled();
    (void)S::address();
    (void)S::instruction();
    S::pin_directions(12, 2, true);
    S::pin_levels(12, 2, false);
    (void)S::tx_full();
    (void)S::tx_empty();
    (void)S::rx_full();
    (void)S::rx_empty();
    (void)S::tx_level();
    (void)S::rx_level();
    S::push(1);
    (void)S::pop();
    (void)S::push_wait(2);
    (void)S::pop_wait();
    S::drain_rx();
    S::drain_tx();
    (void)S::tx_stalled();
    (void)S::tx_overflowed();
    (void)S::rx_underflowed();
    (void)S::rx_stalled();
    S::clear_fifo_debug();
    (void)S::tx_address();
    (void)S::rx_address();
    (void)S::rx_top_byte_address();

    (void)Tx::init(clock, 115200);
    (void)Tx::writable();
    Tx::write('a');
    (void)Tx::write_wait('b');
    (void)Tx::idle();
    TxEngine::arm(Tx::tx_address(), Tx::Sm::dreq_tx);
    (void)TxEngine::start(bytes, 16);
    Tx::release();
    (void)Rx::init(clock, 115200);
    (void)Rx::readable();
    (void)Rx::read();
    (void)Rx::read_wait();
    (void)Rx::frame_error();
    (void)Rx::rx_top_byte_address();
    Rx::release();
    (void)Wave::init(clock, 1'000'000);
    Wave::release();
    (void)Pwm1::init({1, 0});
    Pwm1::duty(500);
    Pwm1::release();
}
