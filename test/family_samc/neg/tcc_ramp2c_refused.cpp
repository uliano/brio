// mcu: samc21e18a samc21g18a samc21j18a
// 36.8.17: WAVE.RAMP = 0x3 (RAMP2C) "is only available in variant L
// devices". The device header defines the encoding for the whole silicon
// family; this part is not one of those devices, so the driver refuses
// it - which also puts erratum 1.21.11 (the OVF DMA trigger in DMAOS
// mode, broken for RAMP2C and RAMP2CS only) out of reach by
// construction.

#include "samc/tcc.hpp"

using namespace brio;

static_assert(tcc_wave_valid(0, TccWaveConfig{.ramp = TccRamp::ramp2_critical}),
              "this assertion is meant to FAIL: RAMP2C is a variant-L mode");
