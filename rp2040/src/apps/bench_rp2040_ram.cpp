// bench_rp2040_ram - bench_rp2040.cpp placed in SRAM: the same app, the
// same letters and the same lines, with BOTH placements this family offers
// - every driver of letters d, e, u and i given util/placement.hpp's InRam
// (each hot path one instance in `.fast_text.brio`, which reaches what no
// vector does: the SPI and I2C hosts' start()), and the four vectors those
// letters bind in `.fast_text`, flattened - both collected into .data by
// the linker script and copied to SRAM by the crt before main(). Every
// letter, read beside bench_rp2040's, warm and after letter c cold, is the
// measurement of what the placement buys on this chip ("A handler in
// SRAM" in the family's platform.md); the drivers alone or the vectors
// alone are a define away (the file header of bench_rp2040.cpp).
//
// build: boards = pico,picow,weact2040
// build: monitor_speed = 115200

#define BENCH_IMAGE "bench_rp2040_ram"
#define BENCH_HOT_IN_RAM 1
#define BENCH_VECTORS_IN_SRAM 1
#include "bench_rp2040.cpp"
