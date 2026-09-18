// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb
// A CRC OVER A LENGTH THAT IS NOT WHOLE WORDS. RM 5.2: the unit
// "calculates the whole 32-bit data word, rather than byte per byte",
// so a byte run must be a multiple of four - padded by the caller, the
// padding being part of the checksum's definition, or computed in
// software by util/crc.hpp.
#include "ch32v203/crc.hpp"

static constexpr uint8_t six[6] = {1, 2, 3, 4, 5, 6};

void f() { (void)brio::Crc::compute_bytes(six); }
