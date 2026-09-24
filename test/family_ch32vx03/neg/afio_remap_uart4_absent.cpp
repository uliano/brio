// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v303cb ch32v303rb
// The fourth serial port is on two CH32V203 of the nine (datasheet table
// 2-1) and on the two 256 KB CH32V303, and its remap field is not
// reachable where the port is not.
#include "ch32vx03/afio.hpp"

void f() { brio::Afio::remap<brio::Remap::uart4, 1>(); }
