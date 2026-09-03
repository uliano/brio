// IRTIM family smoke TU (RM0444 ch. 27): three bits of SYSCFG_CFGR1 and
// one pad. TIM17_CH1 is always the carrier and the envelope is chosen;
// which USART code 10 means is a per-part fact of the reserve.
#include "stm32g0/irtim.hpp"
#include "stm32g0/tim.hpp"

using namespace brio;

static_assert(irtim_envelope_valid(IrtimEnvelope::tim16));
static_assert(irtim_envelope_valid(IrtimEnvelope::second_usart));
static_assert(!irtim_envelope_valid(static_cast<IrtimEnvelope>(3)));   // Reserved
#if defined(STM32G071xx) || defined(STM32G0B1xx)
static_assert(Irtim::second_usart_index == 4);
#else
static_assert(Irtim::second_usart_index == 2);
#endif
static_assert(usart_present(Irtim::second_usart_index));

// The two IR_OUT pads of DS13560's tables: PB9 AF0 and PA13 AF1. PA13 is
// SWDIO on every Nucleo and this project never claims it - the compile
// check below is the whole of its use.
constexpr PinSel ir_pb9{'B', 9, PinFunction::af0};
constexpr PinSel ir_pa13{'A', 13, PinFunction::af1};
using IrPad = IrtimPad<ir_pb9>;
using IrPadSwd = IrtimPad<ir_pa13>;

// The carrier and the envelope are TIM17 and TIM16 channel 1, and
// NEITHER NEEDS A PAD: figure 278's connections are internal.
using Carrier = TimPwm<Tim<17>, 0>;
using Envelope = TimPwm<Tim<16>, 0>;

void irtim_verbs() {
    Irtim::init();
    (void)Irtim::bus_clock();
    (void)Irtim::envelope(IrtimEnvelope::tim16);
    (void)Irtim::envelope(IrtimEnvelope::usart1);
    (void)Irtim::envelope(IrtimEnvelope::second_usart);
    (void)Irtim::envelope();
    Irtim::polarity(true);
    (void)Irtim::polarity();
    Irtim::pb9_high_sink(true);
    (void)Irtim::pb9_high_sink();
    IrPad::claim();
    IrPad::claim(PinSpeed::high, true);
    IrPad::release();
    (void)sizeof(IrPadSwd);

    Tim<17>::init();
    (void)Carrier::setup(0);
    Carrier::duty(Carrier::max / 2);
    Tim<16>::init();
    (void)Envelope::setup(0);
    Envelope::duty(Envelope::max / 2);
    Irtim::release();
}
