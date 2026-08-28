# ============================================================================
#  test/CMakeLists.txt - the host test project: brio's kernel is pure logic
#  behind the Platform concept (brio/platform.hpp), so it compiles
#  and runs on the machine itself with the self-built host g++ (/sw/gcc,
#  same GCC version as avr-gcc: identical language features on both sides).
#
#  A SEPARATE CMake project from the repo root's (own CMakeCache, own
#  compiler) because one CMake configure has exactly one compiler - this
#  cannot share a configure with the AVR cross build. See test/CMakePresets.json
#  for the "host" preset; VS Code's CMake Tools addresses both projects from
#  one workspace via the root .vscode/settings.json's cmake.sourceDirectory
#  array (a project picker in the status bar, no multi-root workspace
#  needed).
#
#  One executable + one ctest entry per test/test_*/main.cpp (22 suites
#  today), auto-discovered the same way the AVR side discovers apps.
#  Flags match the removed [env:native] exactly: the UBSan list is an
#  EXPLICIT check set rather than -fsanitize=undefined, because the
#  umbrella's null/nonnull checks make gcc refuse to fold
#  `&function != nullptr` to a constant and test_reply's legitimate
#  static_assert on a constexpr ReplyTo stops compiling (see design/
#  overview.md's integer-width ruling and CLAUDE.md's C++ style section).
# ============================================================================

cmake_minimum_required(VERSION 3.21)
project(brio_host_tests CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()
include(CTest)

set(BRIO_UBSAN_CHECKS
    signed-integer-overflow shift integer-divide-by-zero float-divide-by-zero
    float-cast-overflow bounds-strict vla-bound alignment bool enum return
    unreachable builtin object-size pointer-overflow
)
string(REPLACE ";" "," BRIO_UBSAN_CHECKS_CSV "${BRIO_UBSAN_CHECKS}")

file(GLOB SUITE_MAINS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test_*/main.cpp")

foreach(_main IN LISTS SUITE_MAINS)
    get_filename_component(_dir "${_main}" DIRECTORY)
    get_filename_component(_suite "${_dir}" NAME)

    add_executable(${_suite} ${_main})
    target_include_directories(${_suite} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../brio
        ${CMAKE_CURRENT_SOURCE_DIR}/../third_party/doctest
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_compile_options(${_suite} PRIVATE
        -std=gnu++23 -g -Wall -Wextra -Werror
        -fsanitize=${BRIO_UBSAN_CHECKS_CSV}
        -fno-sanitize-recover=all
    )
    target_link_options(${_suite} PRIVATE
        -fsanitize=${BRIO_UBSAN_CHECKS_CSV}
    )

    add_test(NAME ${_suite} COMMAND ${_suite})
endforeach()
