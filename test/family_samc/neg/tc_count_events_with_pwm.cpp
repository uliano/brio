// mcu: samc21e18a samc21g18a samc21j18a
// 35.6.2.5.3, on the count-on-event action: "If this operation mode is
// selected, PWM generation is not supported." The rule lives BETWEEN the
// two configuration structs - waveform in one, event action in the other
// - which is exactly why tc_event_config_valid() takes both.

#include "samc/tc.hpp"

using namespace brio;

constexpr TcConfig wave_cfg{.waveform = TcWaveform::normal_pwm};
constexpr TcEventConfig event_cfg{.action = TcEventAction::count};
static_assert(tc_event_config_valid(wave_cfg, event_cfg),
              "this assertion is meant to FAIL: a counter counting events "
              "generates no PWM");
