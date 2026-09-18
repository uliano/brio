// PIO family smoke TU: the assembler's nine instructions and the forms
// this chip added, every verb of the block and of a state machine, the
// GPIO window, the neighbour masks, the four register faces of a FIFO,
// and the four tasks - instantiated on all THREE blocks and on more than
// one machine, so that a fact that is true of PIO0 alone does not pass
// for a fact of the chapter.
#include "rp2350/pio.hpp"

#include "rp2350/clock.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::pll, 150'000'000>;

// The block count and the ring it makes.
static_assert(pio_block_count == 3u);
static_assert(pio_version == 1u);
static_assert(Pio<0>::next_index == 1u && Pio<2>::next_index == 0u);
static_assert(Pio<0>::prev_index == 2u && Pio<1>::prev_index == 0u);
static_assert(Pio<0>::reset_bit == ResetBlock::pio0 && Pio<2>::reset_bit == ResetBlock::pio2);
static_assert(Pio<1>::pin_function == PinFunction::pio1);
static_assert(Pio<1>::irq(0) == PIO1_IRQ_0_IRQn && Pio<1>::irq(1) == PIO1_IRQ_1_IRQn);
static_assert(Pio<0>::gpio_window == 32u);

// A machine's own constants.
static_assert(PioSm<2, 1>::bit == 0x02u && PioSm<0, 3>::bit == 0x08u);
static_assert(PioSm<2, 2>::dreq_tx == Dreq::pio2_tx2 && PioSm<2, 2>::dreq_rx == Dreq::pio2_rx2);
static_assert(PioSm<1, 1>::interrupt_rx_not_empty == 0x0002u && PioSm<1, 1>::interrupt_tx_not_full == 0x0020u);

// A program of this file's own, using the instruction forms the chapter
// added: a WAIT on the branch pin, a MOV to PINDIRS, a cross-block IRQ,
// and the RX FIFO written as a register.
constexpr PioProgram<8> new_forms_program = [] {
    constexpr PioSideSet s{.count = 1};
    PioProgram<8> p{};
    p.side = s;
    p.code = {
        pio_side_delay(pio_mov(PioMovTo::pindirs, PioMovFrom::null, PioMovOp::invert), s, 1),
        pio_wait_jmppin(true),
        pio_wait_jmppin(false, 2),
        pio_in(PioIn::pins, 8),
        pio_put(1),
        pio_put_y(),
        pio_irq(5, false, false, PioIrqScope::next),
        pio_jmp(PioJmp::always, 1),
    };
    p.wrap_bottom = 1;
    p.wrap_top = 7;
    return p;
}();
constexpr PioProgram<3> get_program = [] {
    PioProgram<3> p{};
    p.code = {pio_get(3), pio_get_y(), pio_mov(PioMovTo::x, PioMovFrom::osr)};
    return p;
}();
static_assert(new_forms_program.valid() && get_program.valid());

// The assembler, every encoder once (the values themselves are pinned in
// the driver against the vendor's assembled words).
constexpr PioInstr assembled[] = {
    pio_jmp(PioJmp::x_ne_y, 4),
    pio_wait(true, PioWaitOn::gpio, 9),
    pio_wait(false, PioWaitOn::pin, 0),
    pio_wait_irq(false, 2),
    pio_wait_irq(true, 2, PioIrqScope::relative),
    pio_wait_irq(true, 2, PioIrqScope::prev),
    pio_wait_irq(true, 2, PioIrqScope::next),
    pio_wait_jmppin(true, 1),
    pio_in(PioIn::isr, 4),
    pio_in(PioIn::osr, 32),
    pio_out(PioOut::pindirs, 5),
    pio_out(PioOut::exec, 16),
    pio_out(PioOut::pc, 5),
    pio_push(true, false),
    pio_pull(true, false),
    pio_put(3),
    pio_put_y(),
    pio_get(2),
    pio_get_y(),
    pio_mov(PioMovTo::exec, PioMovFrom::isr),
    pio_mov(PioMovTo::pc, PioMovFrom::status),
    pio_mov(PioMovTo::pindirs, PioMovFrom::null, PioMovOp::invert),
    pio_mov(PioMovTo::y, PioMovFrom::x, PioMovOp::reverse),
    pio_irq(6, true, false, PioIrqScope::here),
    pio_irq(6, false, true, PioIrqScope::next),
    pio_set(PioSetTo::y, 31),
    pio_set(PioSetTo::pindirs, 0),
    pio_nop(),
    pio_delay(pio_nop(), 31),
    pio_relocate(pio_jmp(PioJmp::y_dec, 1), 7),
};
static_assert(assembled[0] != assembled[1]);
static_assert(pio_is_jmp(assembled[0]) && !pio_is_jmp(assembled[1]));

// The side-set's shape, and the divider's arithmetic.
static_assert(PioSideSet{.count = 3, .optional = true}.data_bits() == 2u);
static_assert(PioSideSet{.count = 3, .optional = true}.delay_bits() == 2u);
static_assert(!PioSideSet{.count = 6}.valid());
static_assert(PioClockDiv{}.valid() && PioClockDiv{0, 0}.valid() && !PioClockDiv{0, 1}.valid());
static_assert(PioClockDiv{0, 0}.x256() == 65536u * 256u);
static_assert(pio_clock_div_for(150'000'000, 150'000'000) == PioClockDiv{1, 0});
static_assert(pio_clock_div_for(150'000'000, 0) == std::nullopt);

// The status source, both faces.
static_assert(pio_status_irq(0) == 0u);
static_assert(pio_status_irq(7, PioStatusIrq::next) == 0x17u);

// The interrupt sources: all eight flags reach a line here.
static_assert(PioInterrupt::rx_not_empty(3) == 0x0008u);
static_assert(PioInterrupt::tx_not_full(3) == 0x0080u);
static_assert(PioInterrupt::flag(4) == 0x1000u);
static_assert(PioInterrupt::all_flags == 0xFF00u);

// The neighbour masks.
static_assert(!PioNeighbours{}.any());
static_assert(PioNeighbours{.next = 0x3}.any() && PioNeighbours{.prev = 0x1}.any());

template <uint8_t n>
void block_verbs() {
    using B = Pio<n>;
    (void)B::reset();
    B::hold();
    (void)B::reset();

    // The window.
    (void)B::gpio_base();
    (void)B::gpio_base(16u);
    (void)B::pin_index(20u);
    (void)B::sees(20u);
    (void)B::gpio_base(0u);

    // The memory.
    const auto at = B::add(new_forms_program);
    (void)B::place(get_program);
    (void)B::load(get_program, 24u);
    (void)B::used_slots();
    if (at) {
        B::unload(*at, new_forms_program.length);
    }
    B::unload(24u, get_program.length);

    // The machines together, and the neighbours with them.
    B::enable(0x3u);
    (void)B::enabled();
    B::restart(0x1u);
    B::restart_clocks(0xFu);
    B::disable(0xFu);
    B::enable_across(0x1u, {.next = 0x1, .prev = 0x2});
    B::restart_clocks_across(0x1u, {.next = 0xF});
    B::disable_across(0x1u, {.next = 0x1, .prev = 0x2});

    // What the silicon says it is, and what it is driving.
    (void)B::version();
    (void)B::memory_size();
    (void)B::machine_count();
    (void)B::fifo_depth();
    (void)B::pad_out();
    (void)B::pad_oe();
    B::sync_bypass(0u);
    (void)B::sync_bypass();

    // The status words.
    (void)B::fifo_status();
    (void)B::fifo_debug();
    B::clear_fifo_debug(0xFu);
    (void)B::fifo_levels();

    // The eight flags.
    (void)B::flags();
    (void)B::flag(7u);
    B::clear_flags(0xFFu);
    B::raise_flags(0x80u);

    // The two lines, all sixteen sources.
    B::interrupts(0u, PioInterrupt::all, true);
    B::interrupts(1u, PioInterrupt::flag(7), false);
    B::interrupts_only(0u, PioInterrupt::rx_not_empty(0));
    (void)B::raw_pending();
    (void)B::pending(1u);
    B::force(0u, PioInterrupt::flag(0), true);
    B::force(0u, PioInterrupt::flag(0), false);
    const uint32_t line_registers = B::inte(0u) | B::intf(1u) | B::ints(0u);
    (void)line_registers;
    (void)B::isr(0u);
    (void)B::isr(1u);
}

template <uint8_t n, uint8_t sm>
void machine_verbs() {
    using S = PioSm<n, sm>;

    const uint32_t config_registers = S::clkdiv() | S::execctrl() | S::shiftctrl() | S::pinctrl();
    (void)config_registers;
    S::txf() = 0u;
    const uint32_t taken = S::rxf();
    (void)taken;
    (void)S::tx_address();
    (void)S::rx_address();
    (void)S::rx_top_byte_address();
    const uint32_t cells = S::putget(0u) | S::putget(3u);
    S::putget(1u) = cells;

    PioSmConfig c{};
    c.clock = PioClockDiv{3, 128};
    c.jmp_pin = 5;
    c.out_sticky = true;
    c.inline_out_enable = true;
    c.out_enable_bit = 7;
    c.status_select = PioStatus::irq_flag;
    c.status_n = pio_status_irq(2, PioStatusIrq::prev);
    c.fifo_join = PioFifoJoin::rx_put;
    c.pull_threshold = 8;
    c.push_threshold = 8;
    c.out_shift_right = false;
    c.in_shift_right = false;
    c.autopull = true;
    c.autopush = true;
    c.in_count = 4;
    c.out_base = 2;
    c.out_count = 8;
    c.set_base = 3;
    c.set_count = 2;
    c.sideset_base = 4;
    c.in_base = 5;
    (void)pio_sm_config_valid(c);
    (void)pio_execctrl_of(c);
    (void)pio_shiftctrl_of(c);
    (void)pio_pinctrl_of(c);
    (void)S::configure(c);

    for (PioFifoJoin j : {PioFifoJoin::none, PioFifoJoin::tx, PioFifoJoin::rx, PioFifoJoin::rx_put, PioFifoJoin::rx_get,
                          PioFifoJoin::rx_putget}) {
        PioSmConfig k{};
        k.fifo_join = j;
        (void)S::configure(k);
    }

    (void)S::init(new_forms_program, 0u, c);
    (void)S::init(get_program, 20u, PioSmConfig{});
    S::enable(true);
    (void)S::enabled();
    S::restart();
    S::clock(PioClockDiv{1, 0});
    S::enable(false);

    (void)S::exec(pio_nop());
    (void)S::exec_wait(pio_set(PioSetTo::x, 1), 100u);
    (void)S::exec_stalled();
    (void)S::address();
    (void)S::instruction();
    S::pin_directions(0u, 8u, true);
    S::pin_levels(0u, 3u, false);

    (void)S::tx_full();
    (void)S::tx_empty();
    (void)S::rx_full();
    (void)S::rx_empty();
    (void)S::tx_level();
    (void)S::rx_level();
    S::push(0x1234u);
    (void)S::pop();
    (void)S::push_wait(1u, 10u);
    (void)S::pop_wait(10u);
    S::drain_rx();
    S::drain_tx();
    (void)S::tx_stalled();
    (void)S::tx_overflowed();
    (void)S::rx_underflowed();
    (void)S::rx_stalled();
    S::clear_fifo_debug();
}

// The four tasks, on three blocks and three machines, over pads every
// package of this chip bonds.
using Tx = PioUartTx<0, 0, 13>;
using Rx = PioUartRx<0, 1, 15>;
using Wave = PioSquareWave<1, 2, 17>;
using Pwm = PioPwm<2, 3, 9, 999>;
static_assert(PwmChannel<Pwm>);
static_assert(Pwm::max == 999u && Pwm::cycles_per_pulse == 3000u);
static_assert(Rx::frame_flag == 5u);

void task_verbs() {
    constexpr SysClock clock;

    (void)Tx::init(clock, 115'200u);
    (void)Tx::writable();
    Tx::write(0x41u);
    (void)Tx::write_wait(0x42u);
    (void)Tx::idle();
    (void)Tx::tx_address();
    Tx::release();

    (void)Rx::init(clock, 115'200u);
    (void)Rx::init(clock, 115'200u, PinPull::none);
    (void)Rx::readable();
    (void)Rx::read();
    (void)Rx::read_wait(10u);
    (void)Rx::frame_error();
    (void)Rx::rx_top_byte_address();
    Rx::release();

    (void)Wave::init(clock, 1'000'000u);
    Wave::release();

    (void)Pwm::init();
    (void)Pwm::init(PioClockDiv{4, 0});
    Pwm::duty(500u);
    Pwm::duty(5000u);
    Pwm::release();
}

void f() {
    block_verbs<0>();
    block_verbs<1>();
    block_verbs<2>();
    machine_verbs<0, 0>();
    machine_verbs<1, 2>();
    machine_verbs<2, 3>();
    task_verbs();
}
