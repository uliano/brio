// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// Table 135 gives encoder mode to LPTIM1 alone - and the shared
// LPTIM_TypeDef would let a driver write CFGR.ENC on either.
#include "stm32g0/lptim.hpp"
brio::LptimEncoder<brio::Lptim<2>> impossible;
