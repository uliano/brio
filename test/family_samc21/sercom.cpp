// SERCOM (USART mode) family smoke TU: the Sercom<n> resource, the Uart
// task above it and the baud arithmetic between them.
//
// The instance COUNT is what differs across this family - four on the E
// package, six on the G and J - so this TU exercises SERCOM0, which
// every variant has, and neg/sercom_no_instance_5.cpp states the other
// side of that fact for the E. The pads are PA04/PA05 (SERCOM0 PAD[0]
// and PAD[1] through PMUX function D), bonded on all three variants; the
// console's own PB30/PB31 pair exists only on the 64-pin J and is
// therefore an app-level fact, not one this fixture can compile.
#include "samc21/clock.hpp"
#include "samc21/sercom.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 48'000'000>;

static_assert(sercom_count >= 4, "every variant in the pack has at least SERCOM0..3");

// ---- the pad rules, at compile time ----------------------------------------
// CTRLA.TXPO puts TxD on PAD[0] or PAD[2] and nowhere else, and the two
// directions share a pad only as 31.6.3.8's loop-back - one pin named twice.
static_assert(uart_tx_pad_exists(SercomPad::pad0));
static_assert(uart_tx_pad_exists(SercomPad::pad2));
static_assert(!uart_tx_pad_exists(SercomPad::pad1));
static_assert(!uart_tx_pad_exists(SercomPad::pad3));
static_assert(uart_txpo(SercomPad::pad0) == 0);
static_assert(uart_txpo(SercomPad::pad2) == 1);   // the header calls this code PAD1
static_assert(uart_rxpo(SercomPad::pad3) == 3);

constexpr UartPads pads{
    .tx = SercomPad::pad0,
    .rx = SercomPad::pad1,
    .tx_pin = {'A', 4, PinFunction::d},   // PA04 = SERCOM0/PAD[0], function D
    .rx_pin = {'A', 5, PinFunction::d},   // PA05 = SERCOM0/PAD[1], function D
};
static_assert(uart_pads_valid(pads));
static_assert(!uart_pads_valid({.tx = SercomPad::pad1, .rx = SercomPad::pad0}));
static_assert(uart_pads_valid({.tx = SercomPad::pad0,
                               .rx = SercomPad::pad0,
                               .tx_pin = {'A', 4, PinFunction::d},
                               .rx_pin = {'A', 4, PinFunction::d}}));
static_assert(!uart_pads_valid({.tx = SercomPad::pad0,
                                .rx = SercomPad::pad0,
                                .tx_pin = {'A', 4, PinFunction::d},
                                .rx_pin = {'A', 5, PinFunction::d}}));
static_assert(!uart_pads_valid({.tx = SercomPad::pad0,
                                .rx = SercomPad::pad1,
                                .tx_pin = {'C', 0, PinFunction::d}}));

// The device header must agree that PA04/PA05 reach SERCOM0's first two
// pads through function D - the one half of the pad-to-pin claim that IS
// checkable without a per-package pad table.
static_assert(MUX_PA04D_SERCOM0_PAD0 == static_cast<uint8_t>(PinFunction::d));
static_assert(MUX_PA05D_SERCOM0_PAD1 == static_cast<uint8_t>(PinFunction::d));

// ---- the baud arithmetic ----------------------------------------------------
// Table 30-2, 16x arithmetic: BAUD = 65536 x (1 - 16 x f_baud / f_ref).
static_assert(sercom_baud_reg(48'000'000, 115200).value() == 63019);
static_assert(sercom_baud_reg(48'000'000, 9600).value() == 65326);
static_assert(sercom_baud_reg(48'000'000, 3'000'000).value() == 0);   // f_ref/16: BAUD 0 is LEGAL
static_assert(sercom_baud_reg(48'000'000, 3'000'001).has_value() == false);  // above f_ref/16
static_assert(sercom_baud_reg(0, 115200).has_value() == false);
static_assert(sercom_baud_reg(48'000'000, 0).has_value() == false);
// Round trip: what the generator really produces, within its own step.
static_assert(sercom_actual_baud(48'000'000, 63019) == 115219);
static_assert(sercom_actual_baud(48'000'000, 0) == 3'000'000);
static_assert(sercom_min_ref_hz(115200) == 1'843'200);

using Serial = Uart<0, pads>;
static_assert(ByteTransport<Serial> && BulkSink<Serial> && SpanSource<Serial>);
static_assert(ClockUser<Serial>);
static_assert(Serial::generator == 0);
static_assert(Serial::min_hz_for(115200) == 1'843'200);
static_assert(Serial::can_baud(48'000'000, 115200));
static_assert(!Serial::can_baud(1'000'000, 115200));
void serial_set_baud() { (void)Serial::set_baud(48'000'000u, 9'600u); }

// ---- the register words -----------------------------------------------------
constexpr SercomUartConfig console_cfg{
    .pads = pads,
    .format = {.bits = UartBits::eight, .parity = UartParity::none},
    .baud = 63019,
};
static_assert((sercom_uart_ctrla(console_cfg) & SERCOM_USART_INT_CTRLA_MODE_Msk) ==
              SERCOM_USART_INT_CTRLA_MODE_USART_INT_CLK);
static_assert((sercom_uart_ctrla(console_cfg) & SERCOM_USART_INT_CTRLA_ENABLE_Msk) == 0);
// A DEFAULT frame must be LSB-first: DORD's reset value is MSB-first
// and a defaulted-false lsb_first shipped every byte bit-reversed
// (bench-caught on the first live banner).
static_assert((sercom_uart_ctrla({.pads = pads, .baud = 63019}) &
               SERCOM_USART_INT_CTRLA_DORD_Msk) != 0);
static_assert((sercom_uart_ctrlb(console_cfg) &
               (SERCOM_USART_INT_CTRLB_TXEN_Msk | SERCOM_USART_INT_CTRLB_RXEN_Msk)) ==
              (SERCOM_USART_INT_CTRLB_TXEN_Msk | SERCOM_USART_INT_CTRLB_RXEN_Msk));

void resource_verbs() {
    using Sc = Sercom<0>;
    static_assert(Sc::index == 0);
    static_assert(Sc::gclk_core_id() == SERCOM0_GCLK_ID_CORE);
    static_assert(Sc::apb_mask() == MCLK_APBCMASK_SERCOM0_Msk);
    static_assert(Sc::irq() == SERCOM0_IRQn);

    Sc::bus_clock(true);
    (void)Sc::core_clock(0);
    (void)Sc::reset();
    (void)Sc::configure(console_cfg);
    (void)Sc::enable(true);
    (void)Sc::enabled();
    (void)Sc::sync_busy(SERCOM_USART_INT_SYNCBUSY_CTRLB_Msk);
    (void)Sc::wait_sync(SERCOM_USART_INT_SYNCBUSY_ENABLE_Msk);

    (void)Sc::baud_reg();
    Sc::baud_reg(63019);

    (void)Sc::pending();
    (void)Sc::flags();
    (void)Sc::armed();
    Sc::clear_flags(SercomFlag::txc);
    Sc::enable_interrupt(SercomFlag::rxc | SercomFlag::dre, true);
    Sc::enable_dre_interrupt(false);
    Sc::enable_rxc_interrupt(true);
    Sc::enable_txc_interrupt(false);
    (void)Sc::dre_flag();
    (void)Sc::rxc_flag();
    (void)Sc::txc_flag();

    (void)Sc::status();
    Sc::clear_status(SercomStatus::receive_errors);
    (void)Sc::data();
    Sc::data(0x55);
    Sc::flush_rx();
    Sc::release();

    // The highest instance this device has, whichever that is.
    using Last = Sercom<sercom_count - 1>;
    (void)Last::irq();
    (void)Last::gclk_core_id();
}

void task_verbs() {
    constexpr SysClock clock;
    (void)Serial::init(clock, 115200);
    (void)Serial::init(clock, 9600, {.bits = UartBits::seven, .parity = UartParity::even,
                                     .two_stop = true});
    Serial::rebase(24'000'000);
    (void)Serial::actual_baud(48'000'000);

    (void)Serial::isr();

    uint8_t byte = 0;
    (void)Serial::write_byte('x');
    (void)Serial::read_byte(byte);
    const uint8_t text[] = {'h', 'i'};
    (void)Serial::write_bulk(text);
    (void)Serial::read_span();
    Serial::consume(1);

    (void)Serial::rx_pending();
    (void)Serial::tx_idle();
    (void)Serial::rx_overruns();
    (void)Serial::frame_errors();
    (void)Serial::parity_errors();
    (void)Serial::hw_overruns();
    Serial::clear_errors();
    Serial::release();

    // A second instantiation on the same instance but the other legal
    // TxD pad, with rings of a size that takes the guarded path on the
    // AVR: this core reads a word atomically, so the lock-free path holds
    // at any size here.
    constexpr UartPads wide_pads{
        .tx = SercomPad::pad2,
        .rx = SercomPad::pad3,
        .tx_pin = {'A', 6, PinFunction::d},
        .rx_pin = {'A', 7, PinFunction::d},
    };
    using Wide = Uart<0, wide_pads, 1024, 1024>;
    static_assert(Ring<uint8_t, 1024, SamPlatform>::lock_free);
    (void)Wide::init(clock, 460800);

    // The loop-back (31.6.3.8): the receiver on its own transmitter's pad,
    // one pin named for both directions.
    constexpr UartPads loop_pads{
        .tx = SercomPad::pad0,
        .rx = SercomPad::pad0,
        .tx_pin = {'A', 4, PinFunction::d},
        .rx_pin = {'A', 4, PinFunction::d},
    };
    using Loop = Uart<0, loop_pads>;
    (void)Loop::init(clock, 3'000'000);
    (void)Loop::tx_idle();
    Loop::release();
}

// ---- the optional DMA engine -------------------------------------------------
// The engine lives in samc21/dmac.hpp, not in sercom.hpp: including the
// DMAC from the SERCOM would give every program with a serial port the
// descriptor sections. So an application that wants it includes both
// headers and names it, and the Uart takes it in its ONE slot - the
// transmit one - which DEFAULTS to NoDmaEngine, so every use above
// compiles unchanged. A receive slot does not exist
// (neg/uart_receive_engine_slot.cpp).
#include "samc21/dmac.hpp"

using TxEngined = Uart<0, pads, 64, 256, DmaTxEngine>;

static_assert(!NoDmaEngine::present);
static_assert(!Serial::has_tx_engine && TxEngined::has_tx_engine);
static_assert(DmaTxEngine::present && DmaTxEngine::channel == 0);
// The concepts hold whether or not the engine is named.
static_assert(ByteTransport<TxEngined> && BulkSink<TxEngined> && SkippingSource<TxEngined> &&
              ClockUser<TxEngined>);
// The refused claim's breadcrumb names the refused owner.
static_assert(dma_claim_refused_context(6) == 0xD6);

void engined_uart_verbs() {
    constexpr SysClock clock;
    (void)TxEngined::init(clock, 9600);
    (void)TxEngined::isr();          // the receiver's RXC, still an interrupt
    TxEngined::dma_isr();            // the DMAC's one vector
    (void)TxEngined::write_byte('x');
    (void)TxEngined::write_bulk(std::span<const uint8_t>());
    (void)TxEngined::dma_faults();
    (void)TxEngined::tx_idle();
    TxEngined::release();
    // The claim, as a verb: one owner at a time.
    (void)DmaTxEngine::claim(1);
    (void)DmaTxEngine::owner();
    DmaTxEngine::release(1);
    (void)Serial::dma_faults();      // engineless: 0, and free
}

// ---- the ring's bulk API, which the TX engine drains through ----------------
// Added for the engines and used by them: the contiguous run each side
// owns, so a whole block goes out in one transfer instead of one byte
// per interrupt. design/ring.md carries the contract.
void ring_span_verbs() {
    static Ring<uint8_t, 256, SamPlatform> r;
    const auto w = r.write_span();
    if (!w.empty()) {
        w[0] = 0x5A;
        r.publish(1);
    }
    const auto rd = r.read_span();
    r.consume(static_cast<uint8_t>(rd.size()));
}
