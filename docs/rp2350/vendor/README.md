# RP2350 documents of record

The PDFs are local symlinks (git-ignored, `*.pdf` in this directory) to
the user's copies; the files are Raspberry Pi's own from
https://datasheets.raspberrypi.com/. Cite by SECTION, never by page:
pages move between builds, section numbers do not.

| Document | Build | What brio takes from it |
|----------|-------|-------------------------|
| RP2350 Datasheet | build d126e9e | THE document of record: 2.1 the bus fabric and the atomic register aliases (2.1.3), 3.1 SIO (3.1.2 the GPIO path, 3.1.8 the RISC-V platform timer), 3.2 the interrupt numbering both architectures share, 3.8 the Hazard3 core (3.8.4 traps and interrupts, 3.8.5 `wfi`, 3.8.6.1 the Xh3irq controller), 3.9 the architecture select, 3.5.8 the rescue reset, 5.1/5.9 the bootrom and the IMAGE_DEF, 7.5 subsystem resets, 8.1 clocks, 8.2 XOSC, 8.3 ROSC, 8.5 the tick generators, 8.6 the PLL, 9 GPIO, 12.15 the system control registers (SYSINFO, TBMAN) - and Appendix C (the steppings) and Appendix E (the errata) in the same book |
| Hardware design with RP2350 | the 2025 build | the minimal design every board here derives from: the crystal circuit, the flash, the decoupling, the debug port |
| Raspberry Pi Debug Probe product brief | - | the probe's two ports and its cables |

The device description in code is the pico-sdk's, vendored at tag 2.3.1
in `third_party/pico-sdk/rp2350/` (the directory's README says what and
why): the CMSIS device header generated from the chip's SVD, the
register bit-field headers generated from the same SVD, and the SVD
itself in `rp2350/svd/RP2350.svd` for the Peripheral Viewer. There is no
second-stage bootloader to vendor on this chip. The SDK's runtime is
read as a reference and cited, never taken.

## The bench chip

The RP2350 on the WeAct board: SYSINFO.CHIP_ID reads **0x20004927** -
part 0x0004, manufacturer 0x493 (the eleven bits this chip reports where
the RP2040 reports 0x927 with the JEP-106 stop bit included), REVISION
0x2, which appendix C names **stepping A2**. SYSINFO.PACKAGE_SEL reads
0, the QFN-80: 48 GPIO and eight ADC inputs. The flash beside it is a
Winbond W25Q128, 16 MB in 4096 sectors, which the OpenOCD fork
identifies by itself.

## The errata pass

Appendix E carries RP2350-E1 to E28, and every one that says "Affects
RP2350 A2" is live on this bench chip. What the stratum answers so far:

- **E9** (increased leakage on a Bank 0 pad with its input enabled,
  fixed in A3): stated in `pin.hpp` and in this target's front page - an
  idle level is history, not a measurement.
- **E4** (a debugger's system-bus access stalls while core 1 sits in
  Hazard3's clock-gated sleep): answered by never writing MSLEEP, so the
  RISC-V idle is a plain `wfi`.
- **E6** (the PMP's RWX fields are transposed on this core) and **E7**
  (U-mode does not treat mstatus.MIE as set): recorded and not reached -
  brio runs everything in Machine mode with no PMP region.

The rest belong to chapters that have no driver here yet, and each one
is answered in the chapter it bites.
