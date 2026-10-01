// bench_samc_ram - bench_samc.cpp placed in SRAM: the same app, the same
// letters and the same lines, with SERCOM5_Handler, SysTick_Handler and
// letter r's stamp loop in `.ram_text` (samc21/ld/samc21j18a.ld puts it
// first in .data, and the crt's copy of .data moves it to SRAM before
// main). Its letters p and t, read beside bench_samc's, are the
// measurement of what executing the vectors out of flash costs this
// family.
//
// TEMPORARY: it lives for that one decision - whether the stratum's
// ISR-binding pattern gains the placement as a documented option - and is
// deleted with it, whichever way it goes.
//
// build: boards = c21j
// build: monitor_speed = 115200

#define BENCH_RAM_TEXT 1
#include "bench_samc.cpp"
