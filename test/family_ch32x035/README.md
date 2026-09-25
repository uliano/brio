# Family compile check (CH32X035)

Smoke translation units proving the `ch32x035/` stratum - and every
`kernel/` and `util/` header above it - compiles for the series with the
project's own flags (WCH's gcc 15.2, the QingKe V4C's
`-march=rv32imac_xw -mabi=ilp32`): instantiation only, no hardware, no
`main()`. `brio check ch32x035` compiles each `*.cpp` here for each part in
its list, both ways the project builds an image (with the core's hardware
prologue and without it), and each `neg/*.cpp` must FAIL for the parts its
`// mcu:` header line names.

SEVEN PARTS, ONE DIE, ONE MEMORY SIZE. There is no vendor header in this
build: the part is a definition the script passes as the project does
(`-DCH32X035F8` and the like), and `brio/ch32x035/device.hpp` includes the
part's own table under it. Every part has 62 KB of code flash and 20 KB of
SRAM, so what the sweep proves is the PACKAGE: which pads it bonds, which
pads it shorts together on one pin, and therefore which USART columns a
transport may use. A positive TU compiles for all seven parts unless its
`// mcu:` line names fewer, and where the parts differ it reads the
difference from `device::` rather than from the part definition - inside a
template where a branch names a pad some package has not got, so that the
branch is never formed there.

WHAT THE SWEEP CATCHES that one part cannot. The bench part, the
CH32X035F8U6, is the one package whose PC16 and PC17 are NOT shorted to
PC11 and PC10, so a USB-pad column that works there is refused everywhere
else; the two 20-pin CH32X035 offer no USART1 at all; the two 28-pin parts
put PB1 and PB5 on one pin and the QSOP28 two more pairs; PA8 - USART4's
RTS in its default column - is a pin of the LQFP packages alone. Each of
those is a `device::` constant here and a refusal there, and a negative TU
names only the parts where its refusal holds.

`device.cpp` is the map's own check: every register struct at the offsets
the reference manual gives, and the part table's facts consistent with
each other - among them the model table's GPIO count, which the bonded
pads reproduce exactly once a shorted pair is counted as one pin.
`util_all.cpp` includes EVERY kernel, util and gfx header and instantiates
each service over this target's platform, which is the compiler's verdict
on the whole framework for this core.
