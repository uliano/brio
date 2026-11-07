// boards/v003f4/board.hpp - WCH's CH32V003F4P6 evaluation board, board
// type `v003f4` (docs/boards/ch32v003f4.md): what is fitted on it, as
// constants a program reads at compile time. The ch32v00x build puts this
// directory on the include path of every app it configures for `v003f4`,
// and an app writes `#include "board.hpp"`; what a board file is and what
// goes in it is docs/boards/README.md's, "The board file".
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
inline constexpr std::string_view type = "v003f4";

/// Whether this is the board of type `t`: decided at compile time at every
/// optimization level (consteval), so a choice keyed on the board costs
/// no code even where the optimizer folds nothing.
consteval bool is(std::string_view t) { return t == type; }

static_assert(brio::device::part == brio::Ch32Part::v003 && brio::device::package == brio::Ch32Package::pins20,
              "boards/v003f4: the board carries a CH32V003 in its TSSOP20 - the build names another part");

/// The LED every app drives: the board's LEDs sit on the P4 row and
/// reach a pad by a jumper wire; the apps drive PC0.
using Led = brio::Pin<'C', 0>;
/// Whether a high pad lights it: a jumpered LED hangs from 3.3 V (the
/// console's reading of the board, kept as it stands).
inline constexpr bool led_active_high = false;

/// A 24 MHz crystal Y1 sits on PA1/PA2.
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
