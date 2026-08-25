# ============================================================================
#  pio_flags.py - bare-metal AVR build flags + IntelliSense include paths.
#
#  Mirrors blackpill-experiments/tools/pio_flags.py (per-language flag
#  separation) merged with AVR-Multislope/add_toolchain_paths.py (feed the AVR
#  system headers + the __AVR_<mcu>__ define to IntelliSense). Loaded for every
#  env via `extra_scripts = pre:tools/pio_flags.py` in platformio.ini.
#
#  MUST be a `pre:` script so env.Append() runs BEFORE PlatformIO clones the
#  project source env (projenv); a plain/post script would not reach src files.
#
#  Note: -mmcu and F_CPU are injected by the atmelmegaavr builder from the
#  board JSON (boards/AVR128DB28.json), so they are not repeated here.
# ============================================================================
Import("env")
import os

# --- Guard: this script serves the EMBEDDED envs only. -----------------------
# [env:native] (host unit tests) inherits extra_scripts from the base [env];
# AVR flags (-fno-exceptions & co.) and avr-libc include paths would poison
# the host build (glibc vs avr-libc header clashes, doctest without
# exceptions). The native env declares its own flags in platformio.ini.
if env.get("PIOPLATFORM") == "native":
    print("pio_flags: native env, skipping AVR flags")
else:
    # --- Compiler / linker flags, with correct per-language separation. ------
    #   CCFLAGS  -> C *and* C++ (and link)     CFLAGS    -> C only
    #   CXXFLAGS -> C++ only                   LINKFLAGS -> link only
    #
    # Optimization is BUILD-TYPE aware: release gets -Os here; debug builds
    # ([env:<app>-debug], build_type = debug) get their whole optimization/
    # debug profile from debug_build_flags in platformio.ini (-Og -g3 -ggdb3
    # -fno-inline) and MUST NOT also receive -Os, so debug concessions never
    # leak into release firmware and vice versa.
    is_debug = env.GetBuildType() == "debug"

    common = [
        "-Wall", "-Wextra",
        "-ffunction-sections", "-fdata-sections",
    ]
    if not is_debug:
        common += ["-Os", "-g"]        # -g: source-level .lst, not in the .hex

    link = [
        "-Wl,--gc-sections",
        "-Wl,-Map,firmware.map",       # moved into the build dir by gen_lst.py
    ]
    if not is_debug:
        link += ["-Os", "-g"]

    # --- FLMAP, and locking it. ---------------------------------------------
    # gcc 16 puts .rodata in a 32 KB Flash SECTION reached through the
    # data-space window, and emits a write of NVMCTRL.CTRLB.FLMAP in .init to
    # select it. The linker script (avrxmega4_flmap.xn) computes that value as
    # __flmap_value_with_lock and ORs in NVMCTRL.FLMAPLOCK when the symbol
    # __flmap_lock is non-zero - it is weakly defined as 0, so by default the
    # window stays movable for the whole life of the program.
    #
    # brio locks it. The window is a MODE, and a mode that no code in the
    # framework uses is a mode that can only be changed by accident: every
    # flash verb in avrdx/nvm.hpp goes through ELPM/SPM with a 24-bit address
    # and never through the window (which is also what makes the DA's
    # FLMAP-vs-protection erratum inapplicable by construction). Locking is
    # one-way until reset, so it also removes the one way a runaway write
    # could move .rodata out from under the program.
    #
    # Per-env escape hatch, for an app that must exercise the field itself:
    #     // pio: custom_flmap_lock = 0
    # in the app's header (gen_apps.py copies "// pio:" lines into its envs).
    flmap_lock = str(env.GetProjectOption("custom_flmap_lock", "1")).strip()
    if flmap_lock not in ("0", "1"):
        print("pio_flags: custom_flmap_lock must be 0 or 1, got %r - using 1"
              % flmap_lock)
        flmap_lock = "1"
    link += ["-Wl,--defsym,__flmap_lock=" + flmap_lock]
    print("pio_flags: __flmap_lock = " + flmap_lock)

    env.Append(
        CCFLAGS=common,
        CFLAGS=["-std=gnu11"],
        CXXFLAGS=[
            # gnu++23 is the project standard (gcc 16 implements it fully);
            # bump to gnu++26 the day a C++26 feature is actually needed.
            "-std=gnu++23",
            "-fno-exceptions", "-fno-rtti",
            "-fno-threadsafe-statics", "-fno-use-cxa-atexit",
        ],
        LINKFLAGS=link,
        # Shared code lives in lib/brio: the LDF adds its include path and
        # links it automatically for every env whose app includes its headers.
    )

    # --- Feed the AVR toolchain headers to IntelliSense ----------------------
    # The compiler finds these itself via -mmcu; this block is purely so the
    # editor (clangd / cpptools) can resolve <avr/io.h> and friends.
    toolchain_dir = env.PioPlatform().get_package_dir("toolchain-atmelavr")
    if toolchain_dir:
        extra_includes = []
        avr_include = os.path.join(toolchain_dir, "avr", "include")
        if os.path.isdir(avr_include):
            extra_includes.append(avr_include)
        gcc_lib_dir = os.path.join(toolchain_dir, "lib", "gcc", "avr")
        if os.path.isdir(gcc_lib_dir):
            for ver in sorted(os.listdir(gcc_lib_dir)):
                inc = os.path.join(gcc_lib_dir, ver, "include")
                if os.path.isdir(inc):
                    extra_includes.append(inc)
                    break
        if extra_includes:
            env.Append(CPPPATH=extra_includes)
            print("pio_flags: added toolchain include paths:")
            for p in extra_includes:
                print("  ", p)

    # The whole -mmcu macro set, made explicit for IntelliSense. The editor's
    # compiler query runs without -mmcu, so every macro device-specs injects
    # is missing there: not just __AVR_<MCU>__/__AVR_DEVICE_NAME__ (the
    # device header name in avr-libc >= 16.2's computed <avr/io.h>) but also
    # __AVR_XMEGA__ (gates <avr/xmega.h>: _PROTECTED_WRITE/CCP), the sleep
    # macros' prerequisites (<avr/sleep.h> #errors without them, and that
    # one hard error cascades until whole brio headers "lose" their symbols),
    # __AVR_SFR_OFFSET__ (0x20 without -mmcu: every SFR address wrong) and
    # the per-device __AVR_HAVE_* set, which DIFFERS across the family.
    # Hand-listing them proved incomplete once already, so ask the compiler
    # itself: dump the predefined macros with and without -mmcu and feed the
    # __AVR* delta. The real build gets the same -D values -mmcu already
    # implies (identical by construction: no redefinition warnings).
    mcu = env.BoardConfig().get("build.mcu", "")
    if mcu and toolchain_dir:
        import subprocess

        gcc = os.path.join(toolchain_dir, "bin", "avr-gcc")

        def predefines(args):
            out = subprocess.run(
                [gcc] + args + ["-x", "c++", "-E", "-dM", "-"],
                input="", capture_output=True, text=True, check=True).stdout
            macros = {}
            for line in out.splitlines():
                parts = line.split(None, 2)
                if len(parts) >= 2 and parts[0] == "#define":
                    macros[parts[1]] = parts[2] if len(parts) > 2 else None
            return macros

        base = predefines([])
        with_mcu = predefines(["-mmcu=" + mcu])
        mcu_defines = [
            name if value is None else (name, value)
            for name, value in sorted(with_mcu.items())
            if name.startswith("__AVR") and base.get(name) != value
        ]
        env.Append(CPPDEFINES=mcu_defines)
        print("pio_flags: added MCU defines:", mcu_defines)
