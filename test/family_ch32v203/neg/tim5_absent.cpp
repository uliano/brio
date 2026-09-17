// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8
// The 32-bit TIM5 belongs to the 128 KB part alone (datasheet table
// 2-1's "General-purpose (32-bit)" row), so eight of the nine parts
// refuse it - and the part's own table is what says so.
#include "ch32v203/tim.hpp"

using Wide = brio::Tim<5>;
void f() { Wide::init(); }
