# Family compile check (CH32V00x)

Smoke translation units proving the `ch32v00x/` stratum - and every
`kernel/` and `util/` header above it - compiles for the family with
the project's own flags (`-march=rv32ec_zmmul_xw -mabi=ilp32e`, WCH's
gcc 15.2): instantiation only, no hardware, no `main()`. `brio check
ch32v00x` compiles each `*.cpp` here for each part in its list, and
each `neg/*.cpp` must FAIL for the parts its `// mcu:` header line
names.

TWO CHIPS, SEVEN PACKAGES. There is no vendor header in this build: the
part is two definitions the script passes as the project does - the chip
(`-DCH32V006` with `-march=rv32ec_zmmul_xw`, `-DCH32V003` with
`-march=rv32ec_xw`) and the package, the part number with its package
letter (`-DCH32V006F8`) - and `brio/ch32v00x/device.hpp` includes the
chip's own table, which says what the package bonds. The sweep covers
every package the two datasheets list with the chip's memories: the
CH32V006's K8 (QFN32), E8 (QSOP24), F8 (QFN20 and TSSOP20) and D8
(QFN12), the CH32V003's F4 (TSSOP20 and QFN20), A4 (SOP16) and J4
(SOP8). A positive TU compiles for every one unless its `// mcu:` line
names fewer: the driver fixtures name the pads of the bench packages, so
they list the packages that bond those pads, and `packages.cpp` compiles
every driver that takes pads on every package over the first column,
pad or channel that package carries - `pin.cpp` pins each package's
table against the datasheet's pin table. A TU pins a part's values under
that part's definition, with `if constexpr` on the `device::` facts
where the two differ. The negatives prove a package refuses a pad it
does not bond (`pin_absent_pad`, `pin_port_b_absent`, `pin_sop16_absent`,
and a column of each driver: `uart_column_absent`, `spi_column_absent`,
`i2c_column_absent`, `tim_pad_absent`, `exti_absent_pad`,
`opa_output_absent`) and refuses to drive a pad that shares its package
pin (`pin_twinned_output`, `pin_twinned_function`). What the sweep
also proves is the other half of the check: that WCH's gcc 15.2
accepts every C++23 construct brio's kernel and services are written
with (the other three targets compile with gcc 16.2), which is what
`util_all.cpp` exists for - it includes EVERY kernel and util header
and instantiates each service over this target's platform.

THE PLACEMENT (util/placement.hpp): `platform.cpp`, `spi.cpp`,
`dma.cpp` and `i2c.cpp` each place a plain function of the program -
a vector - in `.fast_text` (`platform.cpp` in `.ram_text`) in the same
unit that emits the drivers' `InRam` instances in `.fast_text.brio` or
the platform's own SRAM body in `.ram_text.brio`: the two names are
what lets GCC take a plain and a grouped function in one unit, and a
regression to one name fails to compile here. The negatives refuse an
unknown tag (`hot_placement_unknown`, `spi_host_placement_unknown`),
`InRam` on a platform that does not place `.fast_text`
(`uart_placement_platform_cannot`) and on a port with nothing to place
(`uart_placement_without_engine`).
