// boards/v006k8/board.hpp - the CH32V006K8U6 module, board type `v006k8`
// (docs/boards/ch32v006k8.md): what is fitted on it, as constants a
// program reads at compile time. The ch32v00x build puts this directory
// on the include path of every app it configures for `v006k8`, and an app
// writes `#include "board.hpp"`; what a board file is and what goes in it
// is docs/boards/README.md's, "The board file".
//
// Constants and pin aliases only: nothing here forms code, and nothing
// under brio/ includes it.

#pragma once

#include <stdint.h>

#include "ch32v00x/device.hpp"
#include "ch32v00x/pin.hpp"

#include <string_view>

namespace board {

/// The board type this file describes: the key the build and the bench
/// tool know the assembly by.
inline constexpr std::string_view type = "v006k8";

/// Whether this is the board of type `t`: decided at compile time at every
/// optimization level (consteval), so a choice keyed on the board costs
/// no code even where the optimizer folds nothing.
consteval bool is(std::string_view t) { return t == type; }

static_assert(brio::device::part == brio::Ch32Part::v006 && brio::device::package == brio::Ch32Package::pins32,
              "boards/v006k8: the board carries a CH32V006 in its QFN32 - the build names another part");

/// The LED every app drives: the module's LED has no pad of its own and
/// is jumpered to PC0.
using Led = brio::Pin<'C', 0>;
/// Whether a high pad lights it: the jumpered LED hangs from 3.3 V (the
/// console's reading of the module, kept as it stands).
inline constexpr bool led_active_high = false;

/// A 24 MHz crystal sits on PA1/PA2.
inline constexpr bool has_crystal = true;

/// Nothing fitted on PC4: a letter may use it as a pad with no wire.
inline constexpr bool pc4_free = true;

/// The pad a letter reads through its own pull-up: PC4, which nothing
/// loads.
using PulledPad = brio::Pin<'C', 4>;

/// Nothing fitted on PD4: a letter may drive it, read its level or its
/// pull.
inline constexpr bool pd4_free = true;

/// THE PIN SUITE'S JUMPER, its driving and its sensing pad: PD2 to PD4.
using JumperDrive = brio::Pin<'D', 2>;
using JumperSense = brio::Pin<'D', 4>;

}  // namespace board
