# ============================================================================
#  host/CMakeLists.txt - the host build: programs that RUN, as opposed to
#  the suites in test/ which run and exit.
#
#  The host is a target like any other, so it gets a build project like
#  any other, and the three axes finally agree: brio/host/ (the stratum),
#  docs/host/ (the documents), host/ (this). What it is NOT is a second
#  home for the tests - test/ is the TEST project (host suites and the
#  cross-compiled family fixtures), and every executable there becomes a
#  ctest entry by construction, which an interactive program cannot be.
#
#  One main() per src/apps/<app>.cpp, auto-discovered at configure time
#  from its own "// build:" header comment, the same grammar the cross
#  projects use. Nothing here is registered with ctest.
#
#  The flags match test/'s exactly - same compiler, same standard, the
#  same explicit UBSan check list - so that a program and the suites
#  cannot disagree about the language they are written in.
# ============================================================================

cmake_minimum_required(VERSION 3.21)
project(brio_host_apps CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The same list test/CMakeLists.txt uses, and for the same reason: the
# umbrella's null/nonnull checks stop gcc folding `&function != nullptr`
# to a constant.
set(BRIO_UBSAN_CHECKS
    signed-integer-overflow shift integer-divide-by-zero float-divide-by-zero
    float-cast-overflow bounds-strict vla-bound alignment bool enum return
    unreachable builtin object-size pointer-overflow
)
string(REPLACE ";" "," BRIO_UBSAN_CHECKS_CSV "${BRIO_UBSAN_CHECKS}")

file(GLOB APP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/apps/*.cpp")

foreach(_app_path IN LISTS APP_SOURCES)
    get_filename_component(_app "${_app_path}" NAME_WE)

    add_executable(${_app} ${_app_path})
    target_include_directories(${_app} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../brio
    )
    target_compile_options(${_app} PRIVATE
        -std=gnu++23 -g -Wall -Wextra -Werror
        -fsanitize=${BRIO_UBSAN_CHECKS_CSV}
        -fno-sanitize-recover=all
    )
    target_link_options(${_app} PRIVATE
        -fsanitize=${BRIO_UBSAN_CHECKS_CSV}
    )
    # A shared segment wants the real-time library where it is separate
    # (glibc); on macOS it is part of libSystem and there is no librt.
    if(NOT APPLE)
        target_link_libraries(${_app} PRIVATE rt)
    endif()
endforeach()

list(LENGTH APP_SOURCES _app_count)
message(STATUS "brio host: ${_app_count} app(s) in src/apps/")
