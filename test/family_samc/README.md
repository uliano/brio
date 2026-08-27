# Family compile check (SAM C21)

Smoke translation units proving the `samc/` drivers compile for every
device header of the family the pack ships - instantiation only, no
hardware, no `main()`. `tools/check_samc.sh` compiles each `*.cpp` here
for the E/G/J 18A variants (seconds), and each `neg/*.cpp` must FAIL for
the variants its `// mcu:` header line names.

The AVR counterpart (`test/family/`, `tools/check_family.sh`) exists
because the bench chip masks half of that family: a missing port,
instance, register or enum value only shows up on the package that lacks
it. The SAM C21 is far more uniform - PORT (two groups everywhere),
GCLK, MCLK, OSCCTRL and the core are identical across E/G/J; what does
differ is the pin BONDING and the SERCOM count (four on the E, six on
the G and J) - so this fixture is a smaller net today. It grows with
each driver campaign, exactly as the AVR one did.

Neither `test/CMakeLists.txt` nor `samc21/CMakeLists.txt` sees this
directory: the host project only globs `test_*/main.cpp`, and the samc21
project only globs its own `src/apps/*.cpp`. These files are built by
the script alone.
