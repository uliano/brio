// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6
// The fourth serial port is on two parts of the nine (datasheet table
// 2-1), and its remap field is not reachable where the port is not.
#include "ch32v203/afio.hpp"

void f() { brio::Afio::remap<brio::Remap::uart4, 1>(); }
