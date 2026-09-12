// mcu: ch32v006k8 ch32v003f4
// The transport's rings carry bytes: a nine-bit frame in its options
// must be REFUSED (the resource's write_word()/read_word() speak it).
#include "ch32v00x/platform.hpp"
#include "ch32v00x/usart.hpp"

constexpr brio::UartOptions nine{.format = {.bits = brio::UartBits::nine}};
using Wide = brio::Uart<1, brio::Ch32v00xPlatform<>, 64, 64, brio::NoDmaEngine, brio::NoDmaEngine, 0, nine>;
void f() { (void)Wide::init(brio::Clock<brio::ClockSource::pll, 48'000'000>{}, 9600); }
