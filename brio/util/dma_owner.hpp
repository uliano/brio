/*
 * dma_owner.hpp
 *
 * One owner per DMA slot, and one slot per owner, checked when the program
 * is compiled. A slot is what one engine runs on: a DMA channel, or on the
 * STM32F4 a stream. Its owner is the peripheral request it serves -
 * USART2's transmit, SPI1's receive, TIM4's update - and, for a channel no
 * peripheral paces (a memory-to-memory copy, a channel programmed by hand),
 * a tag its user names. A second owner on a slot the image has already
 * given is a compile error that names the slot and both owners; a second
 * slot for an owner that already has one is a compile error that names the
 * owner and both slots. The rule and its reasons are docs/design/dma.md,
 * "The ownership rule"; this header is its check.
 *
 * Who uses it. A driver author: the type that binds a slot to its request
 * (an engine, or the transport that takes one) claims the pair in its class
 * body, and a program that programs a channel by hand claims it with a tag
 * of its own. A program otherwise meets this header only in the error:
 *
 *     error: ambiguating new declaration of 'constexpr brio::DmaOwner<
 *         brio::DmaPeriph::i2c, 1, brio::DmaSignal::rx>
 *         brio::dma_owner_of(DmaSlotId<1, 7>)'
 *     note: old declaration 'constexpr brio::DmaOwner<brio::DmaPeriph::usart,
 *         2, brio::DmaSignal::tx> brio::dma_owner_of(DmaSlotId<1, 7>)'
 *
 * read as "DMA1's channel 7, already USART2's transmit, asked again for
 * I2C1's receive" (the line breaks are this comment's; GCC 15 and 16 print
 * it so, the line it calls "required from here" being where the second owner
 * was instantiated). The other direction reads the same way through the
 * other function: 'constexpr brio::DmaSlotId<1, 5> brio::dma_slot_of(
 * DmaOwner<brio::DmaPeriph::usart, 2, brio::DmaSignal::tx>)' against the
 * old declaration's DmaSlotId<1, 4> is "USART2's transmit, already on
 * channel 4, asked again for channel 5". Nothing here runs: a claim is a declaration the
 * compiler checks and never emits, so it adds no byte to the image and no
 * cycle (only the debug information describes its types), and nothing waits,
 * masks or allocates.
 *
 * The smallest use, an engine that binds DMA1's channel `ch` to the request
 * its owner names:
 *
 *     template <uint8_t ch, typename Elem, typename Owner>
 *     struct Engine {
 *         static_assert(brio::dma_claim<brio::DmaSlotId<1, ch>, Owner>);
 *         ...
 *     };
 *
 * THE KEY AND THE OWNER. The key says which slot: `DmaSlotId<controller,
 * slot>`, the controller and the slot numbered as the family's manual
 * numbers them (DMA1 is 1), the controller 0 where the manual numbers none
 * (the SAM C21's DMAC, the RP families' DMA). On the STM32F4 the slot is the
 * stream, never the channel cell of the stream's CHSEL: two cells of one
 * stream cannot run at once (RM0090 10.3.3), so two cells are one slot. The owner says which request: `DmaOwner<periph,
 * instance, signal>`, one peripheral's one request, so the key leaves the
 * element type out and the owner leaves the engine and the transport out -
 * a host that uses its channel at two element widths, or one peripheral
 * driven by two transport types over the same request, is one owner and
 * passes. Any empty class is an owner too: the tag a program names for a
 * channel it programs by hand, or for a copy - one tag a slot.
 *
 * BOTH WAYS. The claim binds the pair in both directions: the slot to one
 * owner, and the owner to one slot. The second half is the silicon's own
 * rule wherever a request can reach more than one channel: the RP families
 * route any request to any channel and forbid two channels on one
 * ("multiple channels should not be connected to the same DREQ", RP2040
 * datasheet 2.5.3.2; "must not", RP2350 12.6.4.2), the STM32G0's DMAMUX
 * forbids one request id on two channels at once (RM0444 11.4.4, the
 * caution), and an STM32F4 request offered two streams would be served by
 * both if both were bound to it. Where the map wires each request to one
 * channel (the WCH families) the second half can never fire and costs
 * nothing. The owner being the request, the RPs' one channel per DREQ is
 * this half and needs no key of its own.
 *
 * WHERE THE CLAIM IS MADE, AND WHEN IT COUNTS. `dma_claim<Slot, Owner>` is a
 * constant whose initializer instantiates `DmaClaim<Slot, Owner>`, and the
 * claim is that instantiation: its two friend functions, `dma_owner_of(Slot)`
 * returning `Owner` and `dma_slot_of(Owner)` returning `Slot`, are declared
 * in namespace brio. One slot with one owner is one specialization, however
 * many types name it and however often, so it declares each function once;
 * one slot with two owners declares `dma_owner_of(Slot)` twice with two
 * return types, and one owner on two slots does the same to
 * `dma_slot_of(Owner)` - two declarations of one function that differ in
 * their return type alone, which C++ forbids and a compiler must diagnose
 * within one translation unit. Nothing reads any state back, and nothing is
 * ever called: the functions are found by no lookup (they are friends of a
 * class no call's argument is associated with) and are never emitted.
 *
 * A claim written in a class template's body (a `static_assert`, as above)
 * counts when that class is instantiated - when the program calls one of its
 * functions or otherwise needs it complete -, so an owner a program names
 * in an alias and never uses claims nothing. It is decided by the compiler's
 * front end, before any optimization: the same in a release and a debug
 * build, with inlining or without.
 *
 * THE LIMIT: ONE TRANSLATION UNIT. The compiler sees one source file and the
 * headers it includes at a time, and so does the check. Every brio image's
 * application is one source file (its other units, the startup code and the
 * runtime, own no channel), which is why the check is enough here. A program
 * split across files would keep the check inside each file and lose it
 * between them: two files giving one slot to two owners each compile clean,
 * the two friend declarations differ only across files, where C++ requires no
 * diagnostic, and the linker sees nothing, the functions never being emitted.
 *
 * The vocabulary below names the peripheral kinds whose requests brio's
 * drivers raise; a family's manual numbers the instance (USART2 is 2, the
 * RP2040's UART0 is 0, a SAM C21 SERCOM by its own number, a DAC channel by
 * its channel, a PIO state machine as its block times four plus its number).
 * A kind a driver adds is a line here.
 */

#pragma once

#include <stdint.h>

namespace brio {

/// Which slot: a controller and one of its channels (a stream on the
/// STM32F4), numbered as the family's manual numbers them, the controller
/// 0 where the manual numbers none. A key, never stored: it has no members.
template <uint8_t controller, uint8_t slot>
struct DmaSlotId {};

/// The peripheral kinds whose requests an owner names. One kind is one
/// numbering of instances in the manual: `usart` covers the USARTs and the
/// UARTs that share the ST and WCH numbering (USART1, UART4), `uart` the
/// RP families' PL011s, `lpuart` the STM32G0's low-power ones, `spi` the
/// PL022s too and an SPI's I2S face (its requests are the SPI's), `sercom`
/// a SAM C21 SERCOM in whichever mode, `dmamux` the STM32G0's request
/// generators.
enum class DmaPeriph : uint8_t {
    usart,
    uart,
    lpuart,
    spi,
    i2c,
    fmpi2c,
    adc,
    dac,
    tim,
    quadspi,
    sercom,
    pio,
    pwm,
    sha256,
    dmamux,
};

/// Which of a peripheral's requests: a data register's two directions (a
/// converter's result is `rx`, a DAC's sample `tx`), or one of a timer's
/// events (a PWM slice's wrap and a request generator's output are
/// `update`).
enum class DmaSignal : uint8_t {
    tx,
    rx,
    update,
    compare1,
    compare2,
    compare3,
    compare4,
    trigger,
    com,
};

/// The owner a slot is given: one request of one peripheral instance.
template <DmaPeriph periph, uint8_t instance, DmaSignal signal>
struct DmaOwner {};

/// The claim itself: one specialization per slot and owner, its friends the
/// slot's one declaration and the owner's one declaration in the image's
/// translation unit (the file header).
template <typename Slot, typename Owner>
struct DmaClaim {
    friend constexpr Owner dma_owner_of(Slot) { return Owner{}; }
    friend constexpr Slot dma_slot_of(Owner) { return Slot{}; }
};

/// True, and the claim made, wherever it is evaluated: write it in the
/// body of the type that binds the slot, `static_assert(dma_claim<Slot,
/// Owner>)`. A second owner on the same slot, or a second slot for the same
/// owner, does not compile.
template <typename Slot, typename Owner>
inline constexpr bool dma_claim = sizeof(DmaClaim<Slot, Owner>) != 0u;

} // namespace brio
