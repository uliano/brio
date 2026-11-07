// mcu: ch32v006f8 ch32v006d8
// The OPA's second output pad is PA5, which the CH32V006's QFN32 and
// QSOP24 bond and its 20- and 12-pin packages do not (DS V2.0 table
// 2-1-1): a config naming it must be REFUSED there - pinned as a
// constant, since configure() answers false at run time.
#include "ch32v00x/opa.hpp"

static_assert(brio::opa_config_valid(brio::OpaConfig{.output = brio::OpaOutput::pa5}));
