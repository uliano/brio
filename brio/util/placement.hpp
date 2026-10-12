/*
 * placement.hpp
 *
 * WHERE A DRIVER'S HOT PATH RUNS FROM, chosen by the program per driver:
 * a tag type, `InFlash` (the default everywhere) or `InRam`, and the one
 * trampoline that turns the tag into a placement, `Hot<Placement>`.
 *
 * WHY A CHOICE. A core that fetches from a flash behind wait states runs
 * a loop or a handler slower than from SRAM (the CH32X035's two wait
 * states at 48 MHz: an SPI frame's handler a third shorter, a USART's
 * block restart under half - docs/ch32x035/platform.md, "A handler in
 * SRAM"), and SRAM is what the program's data needs too. So no driver
 * places anything by itself: a program spends RAM on exactly the paths it
 * names, and a driver it leaves alone costs no byte of SRAM - only the
 * rendezvous its critical loops would have met and the handler timing it
 * would have had.
 *
 * THE MECHANISM. A driver writes its hot body ONCE, always inline, and
 * calls it through `Hot<P>::run(lambda)`; both specializations flatten it
 * (every call in the body inlined, so the path has no call left to
 * another place):
 *  - `Hot<InFlash>` - INLINE where it is called, in the ordinary text:
 *    the path is what it would be without the trampoline, an ISR body
 *    inside the vector that binds it (design/overview.md's rule for ISR
 *    bodies: one call site, so inlined, and only the registers it uses
 *    saved);
 *  - `Hot<InRam>` - ONE OUT-OF-LINE INSTANCE a lambda, in
 *    `.fast_text.brio`, which a family that can place code in SRAM
 *    collects into its `.data` image (its linker scripts), so the crt's
 *    copy of `.data` moves it there before `main()`.
 * Only the instances an image calls exist in it: the RAM is spent by the
 * drivers that opt in, nothing else, and an image that opts into nothing
 * has no instance at all. There is no primary template, so a third tag
 * names an incomplete type and fails where it is written.
 *
 * WHY THE FLASH ONE IS INLINE. Out of line it is a call from the vector
 * and a return, and in a flash behind wait states each taken branch is a
 * fetch the core waits for: measured on the CH32X035 behind the SPI
 * pump's vector, 300.5 cycles a frame with the flash instance out of line
 * against 268.5 inline (docs/ch32x035/spi.md).
 *
 * WHY `.fast_text.brio` AND `noclone`. The instance is a template's, so
 * GCC emits it in a COMDAT group, and refuses one section name for a
 * grouped function and a program's plain one in the same translation unit
 * ("section type conflict", measured with WCH's gcc 15.2): the brio name
 * is for header functions, and a program's own plain function names
 * `.fast_text`. A local clone of a grouped function - IPA-SRA's, which
 * drops the closure parameter an empty lambda leaves unused - leaves the
 * group and meets the same refusal among the grouped ones, so the RAM
 * instance is never cloned.
 *
 * THE BODY IS TAKEN BY REFERENCE. An instance that is never cloned keeps
 * its parameter as written, and a closure wider than two registers passed
 * BY VALUE is an aggregate the caller builds and then copies for the call
 * - measured on the CH32X035: a polled loop's five captures cost a
 * memcpy() of twenty bytes and a frame of 272 on every transaction. Taken
 * by reference, the caller builds the closure once and passes its
 * address, and the instance reads the captures once at its entry.
 *
 * TWO NAMES, TWO MEANINGS. `.fast_text` is code that MAY run from SRAM,
 * for speed: this header's. `.ram_text` stays code that MUST, for
 * correctness - the RP2040's flash engine while the flash is
 * disconnected, the CH32V00x's platform pieces - and is a family's own
 * business, never a program's choice.
 *
 * WHO MAY PLACE. A family whose linker scripts collect `.fast_text` says
 * so on its platform type, `static constexpr bool places_fast_text =
 * true`; every other platform says nothing and is refused `InRam`
 * (`HotPlacement` below) - at compile time, never an orphan section the
 * linker would put wherever its defaults say. A driver asks the question
 * of its own family's platform where the tag is named.
 *
 * WHAT PLACEMENT LEAVES ALONE. The vector the program binds stays where
 * the program puts it: a driver's placed body is a call from it. A
 * program that also wants the vector itself in SRAM names `.fast_text` on
 * it, under the rule above - and a body inline in that vector (InFlash's)
 * then runs from SRAM with it, no call at all.
 */

#pragma once

#include <concepts>

namespace brio {

/// The hot path runs from the code flash, where the rest of the image is:
/// the default of every driver that offers a placement.
struct InFlash {};
/// The hot path runs from SRAM: `.fast_text.brio`, copied with `.data`.
struct InRam {};

/// The trampoline - a specialization per placement and no primary
/// template (the file header).
template <class Placement>
struct Hot;

template <>
struct Hot<InFlash> {
    /// `f()` where it is called, everything it calls inlined into it;
    /// returns what `f()` returns.
    template <class F>
    [[gnu::always_inline, gnu::flatten]] static auto run(F&& f) {
        return f();
    }
};

template <>
struct Hot<InRam> {
    /// `f()` in an out-of-line function in `.fast_text.brio`, everything it
    /// calls inlined into it, never cloned (the file header); returns what
    /// `f()` returns.
    template <class F>
    [[gnu::noinline, gnu::noclone, gnu::flatten, gnu::section(".fast_text.brio")]] static auto run(F&& f) {
        return f();
    }
};

/// Does this platform's family place `.fast_text` in SRAM? Its platform
/// type says so, and saying nothing is no.
template <class Platform>
inline constexpr bool platform_places_fast_text = requires { requires Platform::places_fast_text; };

/// A placement a driver of this platform's family may be given: `InFlash`
/// anywhere, `InRam` where the family places `.fast_text`, nothing else.
template <class Placement, class Platform>
concept HotPlacement = std::same_as<Placement, InFlash> ||
                       (std::same_as<Placement, InRam> && platform_places_fast_text<Platform>);

} // namespace brio
