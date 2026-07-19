#!/usr/bin/env python3
# ============================================================================
#  gen_lst.py - post-build: emit a source-interleaved disassembly (.lst) with
#  avr-objdump and move the linker map into the build dir.
#
#  Inherited from uliano/AVR-Multislope (generate_lst.py). Loaded via
#  `extra_scripts = post:tools/gen_lst.py` in platformio.ini.
# ============================================================================
Import("env")  # type: ignore
import os
import shutil


def generate_listing(source, target, env):  # pylint: disable=unused-argument
    """Generate firmware.lst (disassembly + source) and collect firmware.map."""
    firmware_elf = str(target[0])
    build_dir = os.path.dirname(firmware_elf)
    firmware_lst = os.path.join(build_dir, "firmware.lst")
    firmware_map = os.path.join(build_dir, "firmware.map")

    objdump = env.get("OBJDUMP", "avr-objdump")

    # Disassembly with source (-S), section headers (-h), no zero-skip (-z).
    print("Generating disassembly listing: %s" % firmware_lst)
    env.Execute("%s -h -S -z %s > %s" % (objdump, firmware_elf, firmware_lst))

    # The linker writes firmware.map into the project root (CWD at link time);
    # move it next to the ELF so all artifacts live together.
    map_in_root = "firmware.map"
    if os.path.exists(map_in_root):
        print("Moving linker map file: %s -> %s" % (map_in_root, firmware_map))
        shutil.move(map_in_root, firmware_map)

    print("Build artifacts generated:")
    print("  - ELF: %s" % firmware_elf)
    print("  - MAP: %s" % firmware_map)
    print("  - LST: %s" % firmware_lst)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", generate_listing)  # type: ignore
