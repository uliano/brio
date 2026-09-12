# RP2040 documents of record

The PDFs are local symlinks (git-ignored, `*.pdf` in this directory)
to the user's copies; the files are Raspberry Pi's own from
https://datasheets.raspberrypi.com/. Cite by SECTION, never by page:
pages move between builds, section numbers do not.

| Document | Build | What brio takes from it |
|----------|-------|-------------------------|
| RP2040 Datasheet | build-version 3184e62-clean | THE document of record: the chip whole - 2.1 bus fabric and the atomic register aliases (2.1.2), 2.3 the processor subsystem (SIO, the two NVICs, table 2.3.2's lines), 2.7/2.8 boot sequence and bootrom, 2.14 subsystem resets, 2.15 clocks, 2.16 XOSC, 2.17 ROSC, 2.18 PLL, 2.19 GPIO (table 279, the function select), 2.20 sysinfo, 4.2 UART - and Appendix B, the errata, in the same book |
| Raspberry Pi Pico Datasheet | the 2024 build (Pico and Pico H) | the reference board: schematic (appendix B), the pinout, what sits on GP23/24/25/29, the 12 MHz crystal, the flash |
| Hardware design with RP2040 | the 2024 build | the minimal design example every board here derives from: the crystal circuit, the flash, the decoupling, the 3-pin debug port |
| Getting started with Raspberry Pi Pico | the 2024 build | the Debug Probe wiring and OpenOCD usage, as Raspberry Pi describes them |
| Raspberry Pi Debug Probe product brief | - | the probe's two ports and its cables |

The device description in code is the pico-sdk's, vendored at tag
2.3.1 in `third_party/pico-sdk/` (its README says what and why): the
CMSIS device header generated from the chip's SVD, the register
bit-field headers generated from the same SVD, and the SVD itself in
`rp2040/svd/RP2040.svd` for the Peripheral Viewer. The second-stage
bootloaders in `rp2040/src/glue/boot2_*.S` are the SDK's sources
assembled once and checked in as bytes (their provenance in their
own header). The bootrom source (github.com/raspberrypi/pico-bootrom-rp2040)
is the reference for the boot protocol and the launch of core 1; it
is read, not vendored.

## The bench chip

The RP2040 on the WeAct board: SYSINFO.CHIP_ID 0x20002927 -
manufacturer 0x927 (Raspberry Pi), part 0x0002, revision 2, the B2
silicon - read over SWD and printed by the bring-up firmware. Both
debug ports answer the multidrop select (DPIDR 0x0bc12477, instance
ids 0 and 1), each core a Cortex-M0+ r0p1 with four breakpoints and
two watchpoints. The board's flash identifies as a Zetta ZD25Q16
(JEDEC 0x1560ba, 2048 KiB in 512 sectors of 4 KB).

## The errata pass (Appendix B, sixteen items)

| Erratum | Block | Where it lands in brio |
|---------|-------|------------------------|
| RP2040-E1 | watchdog counts down twice per tick | the watchdog chapter's, when written: LOAD takes twice the value |
| RP2040-E2, E3, E4, E5, E15, E16 | USB device and host | no USB driver; nothing to do until one exists (E5 is fixed in B2) |
| RP2040-E6 | GPIO digital input left enabled on the ADC pads | fixed in the B2 bootrom; the ADC chapter's, when written (`PinConfig::input_enable` is the knob) |
| RP2040-E7 | the XOSC and ROSC COUNT registers are unreliable | `rp2040/clock.hpp` never uses them; every wait there is a bounded spin |
| RP2040-E8 | XIP DMA streaming abort race | no XIP streaming; the flash chapter's, when written |
| RP2040-E9, E14 | the bootrom's UF2 loader | not brio's: images go in over SWD |
| RP2040-E10 | ROSC STATUS.BADWRITE unreliable | the ROSC is never configured by brio |
| RP2040-E11 | ADC DNL peaks at four codes | the ADC chapter's, when written |
| RP2040-E12, E13 | DMA READ_ADDR/WRITE_ADDR while running, the ABORT race | the DMA chapter's, when written: progress by TRANS_COUNT, the abort sequence as code |
