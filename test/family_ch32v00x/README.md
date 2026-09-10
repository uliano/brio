# Family compile check (CH32V00x)

Smoke translation units proving the `ch32v00x/` stratum - and every
`kernel/` and `util/` header above it - compiles for the family with
the project's own flags (`-march=rv32ec_zmmul -mabi=ilp32e`, WCH's
gcc 15.2): instantiation only, no hardware, no `main()`. `brio check
ch32v00x` compiles each `*.cpp` here for each part in its list, and
each `neg/*.cpp` must FAIL for the parts its `// mcu:` header line
names.

ONE PART TODAY. There is no vendor header in this build and the
stratum's map (`brio/ch32v00x/device.hpp`) states the CH32V006K8
alone, so the part list has one entry and the part changes nothing in
the compile; the loop exists so that the second part - and the tiering
it will bring - slots into a script that already runs. What the sweep
proves meanwhile is the other half of the check: that WCH's gcc 15.2
accepts every C++23 construct brio's kernel and services are written
with (the other three targets compile with gcc 16.2), which is what
`util_all.cpp` exists for - it includes EVERY kernel and util header
and instantiates each service over this target's platform.
