// bench_samc_ram - bench_samc.cpp placed in SRAM: the same app, the same
// letters and the same lines, with every vector it binds and letter r's
// stamp loop in `.fast_text` (samc21/ld/samc21j18a.ld puts it first in
// .data, and the crt's copy of .data moves it to SRAM before main), and
// the SPI host given util/placement.hpp's InRam - its polled transaction,
// which no vector reaches. Every letter, read beside bench_samc's, is the
// measurement of what executing out of the flash costs this family, and
// the program-side spelling of both placements (docs/samc21/platform.md,
// "A handler in SRAM").
//
// build: boards = c21j
// build: groups = rmpteu,rmpti
// build: monitor_speed = 115200

#define BENCH_IMAGE "bench_samc_ram"
#define BENCH_VECTORS_IN_SRAM 1
#define BENCH_SPI_IN_RAM 1
#include "bench_samc.cpp"
