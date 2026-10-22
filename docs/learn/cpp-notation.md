# The C++ in blink

The notation a C programmer meets in
[the first program](first-program.md), explained through the lines that
use it. This is not a C++ course: each entry says what the construct
does in that program, what it is not, and where a C comparison helps or
misleads. Read it from the links in the tutorial, or straight through
once the program is familiar.

## Namespaces: `brio::`

Every name of the framework lives in one namespace, `brio`.
`brio::Pin` is "the `Pin` in brio", the way a prefix would spell it in
C (`brio_Pin`), except that the compiler enforces it and a `using
namespace` could drop the prefix. The example writes the prefix every
time, which keeps it clear what is the framework's and what is the
application's.

The application's own types are wrapped in an anonymous namespace,
`namespace { ... }`: the C++ way of saying `static` for a whole group
of definitions, so they are local to this file.

## `using`: a name for a type

```cpp
using P = brio::Stm32g0Platform<>;
using Led = brio::Pin<'A', 5>;
```

`using X = T;` gives the type `T` a second name, like `typedef T X;` in
C. It introduces no variable and runs no code: `Led` is a type, and the
hardware is touched only when a function of that type is called
(`Led::output()`). Choosing the platform and the pin by alias puts the
target-specific choices in one place and lets the rest of the file name
them by role.

## Templates and angle brackets: choices fixed at compile time

```cpp
brio::Pin<'A', 5>
brio::EventQueue<Event, 4, P>
brio::Fsm<Blinker, Toggle, SetPeriod>
brio::ticks_from_ms<P>(500)
```

A template is a type or function with blanks in it. The compiler fills
the blanks with the choices written between the angle brackets and
builds one concrete type or function from them. `Pin<'A', 5>` and
`Pin<'C', 6>` are two different types with the same functions, each
knowing its own port and bit as constants. A template is not a macro:
the arguments are typed, checked and part of the type's identity.

The distinction to keep in mind is between a choice fixed when the
program is compiled and a value that changes while it runs. The pin's
letter and number are fixed. The pin's level is a run-time value that
`toggle()` changes. The queue's depth, 4, is fixed; how many events are
waiting is a run-time value.

`Stm32g0Platform<>` with empty brackets takes every argument at its
default (here, the SysTick timebase).

## Structs with functions, inheritance

```cpp
struct Blinker : brio::Fsm<Blinker, Toggle, SetPeriod> { ... };
```

A C++ `struct` can contain functions and can be built on another type.
`Blinker : brio::Fsm<...>` says that `Blinker` has everything
`Fsm<...>` defines (the `Event` type, `dispatch`, `start`, `handled`,
`unhandled`, `transition`) and adds its own members. In this program
inheritance is used as a way to pull in a ready-made set of members,
not as a way to override behaviour: there is no virtual function
anywhere in brio's kernel.

## The derived type as template argument

```cpp
struct Blinker : brio::Fsm<Blinker, Toggle, SetPeriod>
```

`Blinker` passes its own name as the first argument of the base
template. The current-state pointer is a static member of the base
type, so `Fsm<Blinker, ...>` has one state pointer and
`Fsm<Supervisor, ...>` another; without the name, two objects with the
same event list would share a single state. This is the concrete
purpose here. The idiom is often called CRTP (curiously recurring
template pattern); brio's `Fsm` does not use it to call back into the
derived class, and the explanation above is all the tutorial needs.

## `static inline` members: one per type, no .cpp file

```cpp
static inline brio::EventQueue<Event, 4, P> queue;
static inline uint8_t index = 0;
```

A `static` data member belongs to the type, not to any instance; there
is exactly one `Blinker::queue` in the program. brio's active objects
have no instances at all (`Fsm` deletes the constructor), so "the type"
is the object and every member is static. `inline` on a static member
(C++17) lets its storage be defined right here, in a header or in the
class body, instead of in a separate source file; the linker keeps one
copy. The state is static in the C sense of "allocated for the
program's life, in .bss or .data", and it changes while the program
runs (`index` does). What `static inline` does not promise is anything
about the order in which complicated initializers run across files;
brio's kernel statics are zero-initialized or constant-initialized,
which is why the question does not arise in `blink`.

`static` on a member function (`static void init()`) means it is called
on the type, `Blinker::init()`, with no object and no `this`.

## `constexpr`: computed by the compiler

```cpp
static constexpr uint16_t periods[] = { ... };
constexpr SysClock clock;
brio::ticks_from_ms<P>(500)
```

`constexpr` marks a value, or a function, as computable at compile
time. `periods` is a table the compiler fills in; `ticks_from_ms` is a
function the compiler runs when its argument is a constant, so the
500 ms becomes a literal 500 in the binary. The same function also
works at run time with a variable argument. `constexpr SysClock clock;`
is an empty object whose type carries the clock's configuration; it is
passed to `Ticker::init` so that the ticker can read the rate from the
type.

## `std::variant`: one value, one of several types

`Fsm<Blinker, Toggle, SetPeriod>` defines `Event` as
`std::variant<Entry, Exit, Toggle, SetPeriod>`. A variant holds exactly
one value of one of the listed types and remembers which. A C
programmer would write a tagged union: a `union` of the alternatives
plus an enum saying which member is live. The variant is that, with the
tag maintained by the language: it is set when a value is stored, and
code that reads the wrong alternative does not compile or fails loudly
instead of reading garbage. The size is the largest alternative plus
the tag.

The distinction worth keeping: an `Event` variable holds one
alternative at a time (the event that arrived), while the `Event` type
lists all the alternatives the object accepts.

## Lambdas: a function written where it is used

```cpp
[](Toggle) { Led::toggle(); return handled(); }
[](SetPeriod p) { heartbeat.arm_every(p.ticks); return handled(); }
[](auto) { return unhandled(); }
```

A lambda is a function defined inline, at the place where it is passed
to another function. The square brackets hold the captures: variables
of the surrounding function the lambda may use. All the lambdas in
`blink` have empty brackets, `[]`: they capture nothing and use only
static members and their own parameter, which is what lets them behave
like plain functions. The parentheses hold the parameter: `(Toggle)`
takes a `Toggle` and ignores it, `(SetPeriod p)` takes one and names it
`p`. The body returns a `Status`; every lambda given to one `match` must
return the same type.

`(auto)` is a parameter of any type: this lambda is a template in
disguise, instantiated for each alternative the other lambdas did not
name.

## `match`: choose the lambda by the event type

```cpp
return brio::match(e,
    [](brio::Entry) { ... },
    [](Toggle) { ... },
    [](SetPeriod p) { ... },
    [](auto) { return unhandled(); });
```

`match` (in `brio/kernel/fsm.hpp`) takes the variant and a list of
lambdas, and calls the one whose parameter type is the alternative the
variant holds right now. The compiler turns it into a switch on the
variant's tag. Two rules follow from how it is built:

- Every alternative must be covered, or the program does not compile.
  The `[](auto)` catch-all is how a handler says "everything else".
  Without it, forgetting one alternative is a compile error, which is
  the point.
- All the lambdas return the same type. In a state handler that type is
  `Status`, built by `handled()`, `unhandled()` or `transition(&next)`.

It is a four-line function over the standard `std::visit`; the reason
it exists is that the handler reads as a table of cases, the way a
`switch` over the tag would in C.

## `static_cast`: a conversion spelled out

```cpp
static_cast<uint16_t>(brio::ticks_from_ms<P>(500))
```

`static_cast<T>(x)` is the C++ spelling of `(T)x` for ordinary
conversions, used instead of the C cast so that every narrowing is
visible and searchable. Here a 32-bit tick count is stored in the
16-bit field the event carries.

## `extern "C"` and the vector name

```cpp
extern "C" void SysTick_Handler() { brio::Ticker::tick(); }
```

C++ encodes a function's parameter types into its linker-visible name.
`extern "C"` turns that off, so this function is named exactly
`SysTick_Handler` and the project's startup file, which lists the
interrupt vectors by that name, can find it. It is the same interrupt
vector binding you would write in C, in the one place an application
names a vector.

## What is deliberately absent

No `new`, no `delete`, no heap, no exceptions, no virtual functions, no
run-time type information: the STM32G0 project compiles with
exceptions and RTTI disabled and links no C library
([docs/design/runtime.md](../design/runtime.md)). The C++ in a brio
application is the part of the language that resolves at compile time:
types, templates, `constexpr`, and lambdas that compile to plain
functions.
