# Family compile check (CH32V203)

Smoke translation units proving the `ch32v203/` stratum - and every
`kernel/` and `util/` header above it - compiles for the family with the
project's own flags (`-march=rv32imac_xw -mabi=ilp32`, WCH's gcc 15.2):
instantiation only, no hardware, no `main()`. `brio check ch32v203`
compiles each `*.cpp` here for each part in its list, and each
`neg/*.cpp` must FAIL for the parts its `// mcu:` header line names.

NINE PARTS, ONE ISA. There is no vendor header in this build: the part
is a definition the script passes as the project does (`-DCH32V203C8`
and the like), and `brio/ch32v203/device.hpp` includes the part's own
table under it. Every part of this family is the same QingKe V4B core,
so unlike the CH32V00x fixture the architecture string never changes -
what changes is the table, and that is the whole point of the sweep. A
positive TU compiles for all nine parts unless its `// mcu:` line names
fewer, and where the parts differ it reads the difference from
`device::` rather than from the part definition: which USART instance
this part offers, whether its package has oscillator pads, how many
analog channels it bonds.

WHAT THE SWEEP CATCHES that one part cannot. Three facts of this family
are not "the first n of them": the smallest package offers ONE usart and
it is USART2 (it bonds neither of USART1's pin pairs), two packages
bring out no OSC_IN/OSC_OUT at all, and one part of the nine is the
other DEVICE CLASS, whose vector table is seven entries longer and whose
UART4 sits at a different index. Each of those is a `device::` constant
here and a driver branch there.

The other half of the check is the compiler's verdict: that WCH's gcc
15.2 accepts every C++23 construct brio's kernel and services are
written with - on the ilp32 ABI this time, where the CH32V00x fixture
proves the same headers on ilp32e. That is what `util_all.cpp` exists
for: it includes EVERY kernel, util and gfx header and instantiates each
service over this target's platform.
