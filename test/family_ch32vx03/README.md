# Family compile check (CH32V203)

Smoke translation units proving the `ch32vx03/` stratum - and every
`kernel/` and `util/` header above it - compiles for the family with the
project's own flags (WCH's gcc 15.2, each part with its own ISA and
ABI): instantiation only, no hardware, no `main()`. `brio check
ch32vx03` compiles each `*.cpp` here for each part in its list, and each
`neg/*.cpp` must FAIL for the parts its `// mcu:` header line names.

THIRTEEN PARTS, TWO ISAS. There is no vendor header in this build: the
part is a definition the script passes as the project does
(`-DCH32V203C8` and the like), and `brio/ch32vx03/device.hpp` includes
the part's own table under it. The nine CH32V203 are the QingKe V4B
(`-march=rv32imac_xw -mabi=ilp32`) and the four CH32V303 the V4F
(`-march=rv32imafc_xw -mabi=ilp32f`, the same core with a
single-precision floating-point unit), so the architecture string is a
part fact here as the part table states it - but what changes most is
the table, and that is the whole point of the sweep. A positive TU
compiles for all thirteen parts unless its `// mcu:` line names fewer,
and where the parts differ it reads the difference from `device::`
rather than from the part definition: which USART instance this part
offers, whether its package has oscillator pads, how many analog
channels it bonds, how many DMA1 channels there are.

WHAT THE SWEEP CATCHES that one part cannot. Several facts of this
family are not "the first n of them": the smallest package offers ONE
usart and it is USART2 (it bonds neither of USART1's pin pairs), two
packages bring out no OSC_IN/OSC_OUT at all, two others do not bring out
PA8 and so have the clock output's multiplexer without its pad, one
CH32V203 is the CH32V20x_D8 DEVICE CLASS, whose vector table is seven
entries longer, whose UART4 sits at a different index and whose PLL
divides its 32 MHz oscillator by four or eight where every other part
divides by one or two, and the four CH32V303 are a third class with a
table 104 words long, a DMA1 of seven channels beside a second
controller, and port E on the LQFP100 alone. Each of those is a
`device::` constant here and a driver branch there, and a negative TU
that refuses a block names only the parts where the block is absent.

The other half of the check is the compiler's verdict: that WCH's gcc
15.2 accepts every C++23 construct brio's kernel and services are
written with - on the ilp32 and ilp32f ABIs this time, where the
CH32V00x fixture proves the same headers on ilp32e. That is what
`util_all.cpp` exists for: it includes EVERY kernel, util and gfx header
and instantiates each service over this target's platform.
