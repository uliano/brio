# Family compile check (CH32V00x)

Smoke translation units proving the `ch32v00x/` stratum - and every
`kernel/` and `util/` header above it - compiles for the family with
the project's own flags (`-march=rv32ec_zmmul_xw -mabi=ilp32e`, WCH's
gcc 15.2): instantiation only, no hardware, no `main()`. `brio check
ch32v00x` compiles each `*.cpp` here for each part in its list, and
each `neg/*.cpp` must FAIL for the parts its `// mcu:` header line
names.

TWO PARTS. There is no vendor header in this build: the part is a
definition the script passes as the project does (`-DCH32V006` with
`-march=rv32ec_zmmul_xw`, `-DCH32V003` with `-march=rv32ec_xw`), and
`brio/ch32v00x/device.hpp` includes the part's own table under it. A
positive TU compiles for both parts unless its `// mcu:` line names
fewer - none does today - and pins a part's values under that part's
definition, with `if constexpr` on the `device::` facts where the two
differ. What the sweep
also proves is the other half of the check: that WCH's gcc 15.2
accepts every C++23 construct brio's kernel and services are written
with (the other three targets compile with gcc 16.2), which is what
`util_all.cpp` exists for - it includes EVERY kernel and util header
and instantiates each service over this target's platform.
