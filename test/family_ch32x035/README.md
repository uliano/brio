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
`dma.cpp` holds the DMA controller's map against table 9-2 and the
request table against figure 9-1 on every part, and the serial transport
on its two engines wherever the part offers the instance; the negatives
refuse an engine on another instance's channel, a ninth channel, a beat
wider than the binding's, and a request whose peripheral the part has
not got. `i2c.cpp` holds the I2C's map against table 15-1 (no rise-time
register past CKCFGR), I2C1's six columns against what each package bonds
alone, the host on the part's default column in both engine arrangements
under the arbiter and the client polled and on its vectors; the
negatives refuse an engine on USART2's channels, one engine of two, a
wide beat, a second instance, PA10/PA11 where they are not bonded, the
USB pads where they share their pins, and pads named out of their
column's order. `tim.cpp` holds the two advanced-control timers' map
against table 12-3, their requests against figure 9-1 and both timers'
columns against what each package bonds, every verb of the subset on both
instances, a pad formed only where the package drives it alone; the
negatives refuse TIM3, a DynamicClock, a pad the package does not bond
and a signal the column has not got. `util_all.cpp` includes EVERY kernel, util and gfx header and instantiates
each service over this target's platform, which is the compiler's verdict
on the whole framework for this core.
