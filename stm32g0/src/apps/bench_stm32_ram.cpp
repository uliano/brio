// bench_stm32_ram - bench_stm32.cpp with the drivers' hot paths in SRAM:
// the same app, the same letters and the same lines, every driver it
// binds that takes one given util/placement.hpp's InRam - the console's
// and letter u's ports (isr() and dma_isr()) and the SPI hosts (the
// pump's entry and the polled transaction) - each body one flattened
// instance in `.fast_text.brio`, which the stratum's linker scripts put
// first in .data and the crt's copy moves to SRAM before main. The
// vectors stay where bench_stm32 has them, in the flash, so every body
// is a call through the linker's long-branch veneer. Every letter, read
// beside bench_stm32's, is what the placement buys a program on this
// family (docs/stm32g0/platform.md, "Code in SRAM").
//
// NOT ON THE G031K8: the vectors bind every driver whatever group an
// image carries, so every instance is in every image - some 8 KB of SRAM,
// the part's whole RAM before a ring or a buffer. What each instance
// costs is docs/stm32g0/platform.md's census.
// build: boards = g0b1re,g071rb
// build: monitor_speed = 115200

#define BENCH_IMAGE "bench_stm32_ram"
#define BENCH_HOT_IN_RAM 1
#include "bench_stm32.cpp"
