// bench_samc_ram - bench_samc.cpp placed in SRAM: the same app, the same
// letters and the same lines, with SERCOM5_Handler, SysTick_Handler and
// letter r's stamp loop in `.ram_text` (samc21/ld/samc21j18a.ld puts it
// first in .data, and the crt's copy of .data moves it to SRAM before
// main). Its letters p and t, read beside bench_samc's, are the
// measurement of what executing the vectors out of flash costs this
// family.
//
// The placement is this family's documented option of the ISR binding
// pattern (docs/samc21/platform.md, "A handler in SRAM"). TEMPORARY: this
// twin stays until the console's per-byte path is inline into its handler
// and letters p and t are measured again in both images - the entry in
// that document's "Not covered yet" - and is deleted then.
//
// TWO IMAGES (the groups line below; design/overview.md, "A suite's
// image fits the family's smallest chip"): the handlers' copy in SRAM
// takes some 4 KB of the 32 beside bench_samc's own buffers, and the
// whole app would leave its stack a handful of bytes; letter d's buffers
// go in an image of their own.
//
// build: boards = c21j
// build: groups = rmptuei,d
// build: monitor_speed = 115200

#define BENCH_RAM_TEXT 1
#include "bench_samc.cpp"
