# ============================================================================
#  test/external_app/CMakeLists.txt - the external guard's fixture: a
#  project written the way an application kept OUTSIDE brio's tree is
#  written, which builds one of brio's own apps through cmake/brio.cmake so
#  that `brio check` can compare the image with the in-tree one.
#
#      cmake -S test/external_app -B <dir> -G Ninja -DBRIO_MCU=<part> \
#            [-DBRIO_BOARD=<board type>] [the family's own variables]
#      cmake --build <dir>
#
#  The part comes from the command line (BRIO_MCU), and so does the board
#  type (BRIO_BOARD) when the in-tree build puts a board file on the app's
#  include path. The app is the family's maintained teaching example,
#  blink, where the family has one, else its console - every family has
#  one of the two - built from its in-tree source under its own name, so
#  its image is <dir>/<app>.hex on the AVR and <dir>/<app>.bin elsewhere.
#  cli/check.py configures this project once per release preset and fails
#  the check on any image that differs from the in-tree image of the same
#  app on the same preset.
# ============================================================================

cmake_minimum_required(VERSION 3.25)
include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/brio.cmake)
project(brio_external_app)

# brio.cmake has named the family (BRIO_FAMILY) and brio's root (BRIO_ROOT).
set(_app_dir "${BRIO_ROOT}/${BRIO_FAMILY}/src/apps")
if(EXISTS "${_app_dir}/blink.cpp")
    set(_app blink)
else()
    set(_app console)
endif()

brio_add_app(${_app} "${_app_dir}/${_app}.cpp")
