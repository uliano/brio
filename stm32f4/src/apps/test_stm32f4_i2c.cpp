// test_stm32f4_i2c - the reference bench suite for the STM32F4's I2C: the
// resource over the whole of the chapter, the I2cHost engine under
// util/i2c_bus.hpp's arbiter, the DMA engines and the recovery verbs -
// stm32f4/i2c.hpp.
//
// A test_<target>_<subject> suite is a menu of single-letter tests over
// the console, judged by brio's "ALL: N pass, M fail" grammar
// (util/testbench.hpp owns that grammar). It is a REFERENCE test: it is
// meant to keep passing through every later restructuring of the code
// under it.
//
// THE INSTRUMENT IS A DEVICE ON THE BUS, which is why this suite builds
// for three boards alone: a bus driver measured against nothing is a
// register read-back. Each Discovery board carries its touch-screen
// controller on a bus of its own, and the Nucleo-F446RE's bench wires
// I2C1 to the same chip's FMPI2C1, which this suite serves as a register
// file. The letters know a PEER -
// an address, an identity register with the bytes its datasheet gives it,
// one register they may write and read back with nothing moving on the
// board, and the way the device is put back where its reset leaves it -
// and nothing else of the device:
//
//   STM32F429I-DISC1
//   I2C3   SCL PA8, SDA PC9 (AF4), the pull-ups the board's own
//   the touch controller   an STMPE811 at the 7-bit address 0x41, whose
//                          CHIP_ID at register 0x00 reads 0x0811 as two
//                          bytes MSB first and whose ID_VER at 0x02 reads
//                          a revision byte; SYS_CTRL2 at 0x04 is four
//                          clock gates, written and read back here, and
//                          SYS_CTRL1's soft reset puts every one of its
//                          registers back
//
//   32F469IDISCOVERY
//   I2C1   SCL PB8, SDA PB9 (AF4), the board's 1.5 k pull-ups (MB1189
//          R140/R142); the panel and its touch controller share the
//          reset line PH7 (UM1932 4.14), which this suite pulses before
//          the scan and again to leave the device where its reset puts it
//   the touch controller   the FocalTech capacitive controller of the
//                          MB1166 display board, found by the scan at
//                          the 7-bit address 0x38, tearing its INT line
//                          on PJ5: its register map is FocalTech's
//                          "Application Note for FT6x06 CTPM" (v1.0,
//                          bound into the FT6236/FT6336/FT6436 series
//                          datasheet v0.3), its pins and timings the
//                          FT6x06 datasheet v0.1 and that series
//                          datasheet - FOCALTECH_ID at 0xA8 (0x11 by the
//                          note), CIPHER at 0xA3, FIRMID at 0xA6, LIB_VER
//                          at 0xA1/0xA2, RELEASE_CODE_ID at 0xAF; TH_GROUP
//                          at 0x80 is the touch threshold, written and
//                          read back here; the reset is the shared line's
//                          pulse, 300 ms before the first report
//
//   Nucleo-F446RE
//   I2C1   SCL PB8, SDA PB9 (AF4), wired to PC6 and PC7 with a 2.2 kOhm
//          pull-up on each line (the bench's self-link)
//   the peer               the chip's own FMPI2C1 on PC6/PC7 at 0x2C,
//                          SERVED FROM ITS OWN TWO VECTORS: a register
//                          file of this suite's own map (an identity at
//                          0x00, a version at 0x02, a reset command at
//                          0x03, a scratch register at 0x04, a fixed
//                          pattern beyond), its pointer set by the first
//                          written byte of a tenure - and able to REFUSE
//                          a chosen written byte, which no device on the
//                          other two boards does
//
// and the parts of the chapter that need no device are measured anyway:
// the timing arithmetic against the registers in force, the address scan
// against every address nobody answers, and the SCL frequency counted on
// the clock pad's OWN INPUT BUFFER while a DMA-carried read runs (an I2C
// pad is an open-drain alternate function, so its IDR is the wire).
//
// The touch controller is left in ITS RESET STATE (the device's own
// reset - the STMPE811's soft reset, the other board's reset line - is
// the last thing the device letters do to it).
//
// What is exercised, letter by letter:
//   a  the instances: presence, bus, the two vectors, the gate closed at
//      reset and the reset values behind it
//   b  the timing arithmetic and what it puts in FREQ, CCR and TRISE
//   c  what the driver refuses, and that a refusal writes nothing
//   d  the address scan: who answers, and that everybody else NACKs
//   e  the device's identity through a write-then-read tenure
//   f  the three receive choreographies - one, two and N bytes
//   g  the speeds and both fast-mode duties, with SCL counted on the pad
//   h  a register written and read back, then the device's own reset
//   i  THE KERNEL: I2cBus over I2cHost, the rejection, the sleep votes
//   j  the engine named: `dma` refused on the engineless host, the
//      tenures through the two DMA engines at their shortest - a write
//      and a read of one byte - and `interrupt` on the engined host
//      touching no stream
//   k  the recovery verbs: SWRST, recover() and unstick()
//   l  (the 32F469IDISCOVERY, by name only) A FINGER ON THE GLASS: the
//      touch controller in interrupt trigger mode, its INT counted on
//      EXTI line 5 and its touch data polled for eight seconds while a
//      human touches the panel - coordinates, event flags, touch ids
//   m  (the Nucleo-F446RE) THE REFUSED LAST BYTE: the peer refusing the
//      last of one to four written bytes of a write-then-read - the NACK
//      arriving with the repeated START requested - through the pump and
//      the engines, at 100 and 400 kHz, the core at 180, 48 and 16 MHz
//      (a DynamicClock over the three): the status, the controller 2 ms
//      on (START, STOP, MSL), the next tenure on the same host; each count
//      also acknowledged whole, the control
//   o  (the Nucleo-F446RE) THE RECEIVE STREAM UNDER A REAL LOAD: paced streams at
//      very_high on DMA1, four copies on DMA2 and the core, reads of 1, 2 and 8 at 100
//      and 400 kHz, the event vector's entries and the wire against the completion
//   p  (the Nucleo-F446RE) THE STREAM'S COMPLETION HELD OFF: SCL counted behind the last
//      byte while the vector waits, what stands then, the next read; the stray beat's
//      stage on this block
//   q  (the Nucleo-F446RE) start()'S TWO BOUNDS against the wall: the STOP's drain and
//      the wait for BUSY, staged with the far pads, with the ticker, without it and
//      with the DWT's counter stopped
//   r  (the Nucleo-F446RE, by name only) A LOAD THAT TAKES ALL OF APB1: nothing answering
//      ahead of a starved stream, the read and the next one exact when the load comes off;
//      runs of two and eight starved from the start, from the third byte and across a
//      read's last two, the event vector's entries, the bytes at the target and its pointer
//   s  (the Nucleo-F446RE) A CLIENT HOLDS THE CLOCK PAST THE DRAIN: a request right behind a
//      stretched acknowledge, through an arbiter with a per-bus timeout and one with none: the
//      answer, its time, what it moved, and the next requests with no init() and no recover()
//
// build: boards = f429zi,f469ni,f446re
// build: monitor_speed = 115200

#include <stdint.h>

#include <utility>
#include <variant>

#include "stm32f4/clock.hpp"
#include "stm32f4/dma.hpp"
#include "stm32f4/dwt.hpp"
#include "stm32f4/exti.hpp"
#if defined(STM32F446xx)
#include "stm32f4/fmpi2c.hpp"
#endif
#include "stm32f4/i2c.hpp"
#include "stm32f4/nvic.hpp"
#include "stm32f4/ticker.hpp"
#include "stm32f4/tim.hpp"
#include "stm32f4/usart.hpp"
#include "kernel/event_queue.hpp"
#include "kernel/post.hpp"
#include "kernel/tenuto.hpp"
#include "kernel/time.hpp"
#include "kernel/time_event.hpp"
#include "util/i2c_bus.hpp"
#include "util/power.hpp"
#include "util/print.hpp"
#include "util/testbench.hpp"

#if defined(STM32F446xx)
#include "../../../avrxt/src/apps/twi_link_client.hpp"
#endif

#if defined(STM32F446xx)
/// The Nucleo-64 fits no crystal: its 8 MHz is the ST-LINK's clock output,
/// taken in bypass.
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000,
                             brio::HseMode::bypass>;
#else
using SysClock = brio::Clock<brio::ClockSource::pll_hse, 180'000'000, 8'000'000>;
#endif
constexpr SysClock clock;

namespace {

using namespace brio;

using P = Stm32f4Platform<>;

// ---- the board: the console, the bus, the peer -------------------------------------------

/// What the letters know of the device beyond the bus: the address the
/// board's schematic names, where its identity starts and what the
/// datasheet says it reads (MSB first), a second register printed beside
/// it, and one register that may be written and read back with nothing
/// moving on the board. `id_len == 0` says the datasheet is NOT on the
/// desk: the letters that need the identity or the scratch register then
/// decline by name, and the bus is still measured on whatever the device
/// answers at `id_reg`.
struct PeerFacts {
    const char* name;
    uint8_t addr;
    uint8_t id_reg;
    uint8_t id_len;
    uint8_t id_bytes[2];
    const char* id_name;
    uint8_t ver_reg;
    const char* ver_name;
    uint8_t scratch_reg;
    const char* scratch_name;
    uint8_t scratch_patterns[3];
};

#if defined(STM32F469xx)
constexpr UartPins console_pins{.tx = {'B', 10, PinFunction::af7}, .rx = {'B', 11, PinFunction::af7}};
constexpr uint8_t console_instance = 3;
constexpr uint8_t bus_instance = 1;
constexpr I2cPins bus_pins{.scl = {'B', 8, PinFunction::af4}, .sda = {'B', 9, PinFunction::af4}};
using SclPad = Pin<'B', 8>;
using SdaPad = Pin<'B', 9>;
/// I2C1's transmit request is DMA1 stream 6 on channel 1 and its receive
/// DMA1 stream 0 on channel 1 (RM0386 table 29) - the reserve checks these
/// two cells at compile time.
using TxEngine = DmaTxEngine<1, 6, 1>;
using RxEngine = DmaRxEngine<1, 0, 1>;
/// The line the panel and the touch controller are reset by (UM1932 4.14).
using ResetLine = Pin<'H', 7>;
/// FocalTech's application note for the FT6x06 CTPM: FOCALTECH_ID at
/// 0xA8 (0x11), FIRMID at 0xA6, TH_GROUP at 0x80 - the threshold for
/// touch detection, a plain read/write register that moves no pin.
constexpr PeerFacts peer{.name = "the MB1166's touch controller",
                         .addr = 0,   // found by the scan
                         .id_reg = 0xA8,
                         .id_len = 1,
                         .id_bytes = {0x11, 0},
                         .id_name = "FOCALTECH_ID (0xA8)",
                         .ver_reg = 0xA6,
                         .ver_name = "FIRMID (0xA6)",
                         .scratch_reg = 0x80,
                         .scratch_name = "TH_GROUP (0x80)",
                         .scratch_patterns = {0x14, 0x40, 0x22}};
/// The rest of the note's map the finger letter reads: the touch data
/// block (TD_STATUS then the first point's XH/XL/YH/YL), the interrupt
/// mode, the versions.
constexpr uint8_t reg_td_status = 0x02;
constexpr uint8_t reg_g_mode = 0xA4;
constexpr uint8_t reg_lib_ver_h = 0xA1;
constexpr uint8_t reg_cipher = 0xA3;
constexpr uint8_t reg_release_code = 0xAF;
constexpr uint8_t reg_ctrl = 0x86;
constexpr uint8_t reg_period_active = 0x88;
/// LCD_INT on PJ5 (UM1932 4.14): the controller's INT line, EXTI line 5.
using TouchInt = Pin<'J', 5>;
using TouchIrq = ExtInt<TouchInt>;
volatile uint32_t touch_int_edges = 0;
#elif defined(STM32F446xx)
constexpr UartPins console_pins{.tx = {'A', 2, PinFunction::af7}, .rx = {'A', 3, PinFunction::af7}};
constexpr uint8_t console_instance = 2;
constexpr uint8_t bus_instance = 1;
constexpr I2cPins bus_pins{.scl = {'B', 8, PinFunction::af4}, .sda = {'B', 9, PinFunction::af4}};
using SclPad = Pin<'B', 8>;
using SdaPad = Pin<'B', 9>;
/// I2C1's transmit request is DMA1 stream 6 on channel 1 and its receive
/// DMA1 stream 0 on channel 1 (RM0390 table 28) - the reserve checks these
/// two cells at compile time.
using TxEngine = DmaTxEngine<1, 6, 1>;
using RxEngine = DmaRxEngine<1, 0, 1>;
/// THE PEER IS THE CHIP'S OWN FMPI2C1, on PC6 (SCL) and PC7 (SDA) at AF4
/// (DS10693 table 11), the bench's two wires joining them to PB8 and PB9:
/// a register file this suite serves from that block's own two vectors
/// (the target below) - an identity at 0x00, a version at 0x02, a reset
/// command at 0x03, a scratch register at 0x04, and a fixed pattern
/// everywhere else, the pointer set by the first written byte of a tenure
/// and advanced by every byte moved, as a register-mapped device's is.
constexpr FmpI2cPins target_pins{.scl = {'C', 6, PinFunction::af4},
                                 .sda = {'C', 7, PinFunction::af4}};
using Target = FmpI2cClient<1, target_pins>;
constexpr uint8_t reg_reset = 0x03;
constexpr uint8_t reset_code = 0x02;
constexpr PeerFacts peer{.name = "the chip's own FMPI2C1",
                         .addr = 0x2C,
                         .id_reg = 0x00,
                         .id_len = 2,
                         .id_bytes = {0x04, 0x46},
                         .id_name = "ID (0x00, two bytes)",
                         .ver_reg = 0x02,
                         .ver_name = "VER (0x02)",
                         .scratch_reg = 0x04,
                         .scratch_name = "SCRATCH (0x04)",
                         .scratch_patterns = {0x5A, 0xA5, 0x3C}};
#else
constexpr UartPins console_pins{.tx = {'A', 9, PinFunction::af7}, .rx = {'A', 10, PinFunction::af7}};
constexpr uint8_t console_instance = 1;
constexpr uint8_t bus_instance = 3;
constexpr I2cPins bus_pins{.scl = {'A', 8, PinFunction::af4}, .sda = {'C', 9, PinFunction::af4}};
using SclPad = Pin<'A', 8>;
using SdaPad = Pin<'C', 9>;
/// I2C3's transmit request is DMA1 stream 4 on channel 3 and its receive
/// DMA1 stream 2 on channel 3 (RM0090 table 43) - the reserve checks these
/// two cells at compile time.
using TxEngine = DmaTxEngine<1, 4, 3>;
using RxEngine = DmaRxEngine<1, 2, 3>;
/// The STMPE811: CHIP_ID at 0x00 (0x0811, two bytes MSB first), ID_VER at
/// 0x02, SYS_CTRL2 at 0x04 - four clock gates and nothing that moves a
/// pin - and SYS_CTRL1's soft reset.
constexpr uint8_t reg_sys_ctrl1 = 0x03;
constexpr uint8_t stmpe811_soft_reset = 0x02;
constexpr PeerFacts peer{.name = "the STMPE811",
                         .addr = 0x41,
                         .id_reg = 0x00,
                         .id_len = 2,
                         .id_bytes = {0x08, 0x11},
                         .id_name = "CHIP_ID (0x00, two bytes MSB first)",
                         .ver_reg = 0x02,
                         .ver_name = "ID_VER (0x02)",
                         .scratch_reg = 0x04,
                         .scratch_name = "SYS_CTRL2 (0x04)",
                         .scratch_patterns = {0x00, 0x0F, 0x01}};
#endif

using Serial = Uart<console_instance, console_pins>;
constexpr Serial serial;

// TWO SETS on the Nucleo-F446RE: `z` the board's own letters, runnable
// with the desk partner on its dark console, and `t` the letters whose
// instrument is the partner's `twi_peer` on the same wire (they FAIL
// without it). The scan of letter d would meet the peer's command
// address, so the far board is dark for `z`.
TwoKeyTestBench<Serial, 't'> bench;

using S = I2c<bus_instance>;
using Host = I2cHost<bus_instance, bus_pins>;
using DmaHost = I2cHost<bus_instance, bus_pins, TxEngine, RxEngine>;

/// An address nobody on any of the three boards' buses answers - neither
/// touch controller (0x41, 0x38), nor the F446's self-link peer (0x2C),
/// nor the partner's twi_peer (dark for `z`) - and one every host may
/// name (0x08..0x77): the arbiter letter's probe of nobody.
constexpr uint8_t nobody_addr = 0x5D;

/// The registers the letters read and write, by the peer's names.
constexpr uint8_t reg_chip_id = peer.id_reg;
constexpr uint8_t reg_id_ver = peer.ver_reg;
constexpr uint8_t reg_sys_ctrl2 = peer.scratch_reg;
constexpr bool peer_known = peer.id_len != 0u;

/// What the boot-time scan found: which address answered, and whether the
/// part behind it is the one this board's schematic names.
uint8_t peer_addr = 0;
bool peer_identified = false;
uint8_t peer_sys_ctrl2_boot = 0;

volatile bool bus_ao_live = false;
volatile bool dma_host_live = false;
volatile bool xfer_done = false;
#if defined(STM32F446xx)
// Letter s's glue: the arbiter with no timeout live, the far SCL pad pulled low from the
// completion's entry, the completion's time, the tick at which a client's hold ends.
volatile bool drain_plain_live = false;
volatile bool drain_grab_scl = false;
volatile uint32_t drain_done_at = 0;
volatile uint32_t drain_let_go_tick = 0;
enum class DrainLetGo : uint8_t { none, pad, target };
volatile DrainLetGo drain_let_go = DrainLetGo::none;
#endif
/// Entries of the two DMA streams' vectors, counted whoever owns them.
volatile uint32_t tx_dma_entries = 0;
volatile uint32_t rx_dma_entries = 0;

// ---- the rulers ------------------------------------------------------------------------

/// SysTick's reload period in core cycles - what a VAL delta wraps by.
/// VAL is the one ruler fine enough for a bit period, and reading it has
/// no side effect (cortexm/delay.hpp says why).
uint32_t systick_period() { return SysTick->LOAD + 1u; }

bool same(const uint8_t* a, const uint8_t* b, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) {
            return false;
        }
    }
    return true;
}

// ---- the bus, in the state the letters expect ----------------------------------------------

/// Bring I2C3 back up as the host every letter starts from.
bool host_ready(I2cDuty duty = I2cDuty::ratio_2) {
    bus_ao_live = false;
    dma_host_live = false;
    return Host::init(clock, I2cHostConfig{.duty = duty});
}

/// What this suite reports for a tenure that never answered - the code an
/// arbiter with a timeout would replace with i2c_timeout. It must never be
/// mistaken for the i2c_ok start() left standing.
constexpr uint8_t parked = 0xFF;

/// One tenure against `addr`, driven by hand (the arbiter skipped, as
/// i2c-bus.md allows a device that owns a bus alone). Returns the status
/// the engine ended with, or `parked`; the wait is bounded.
/// THE ENGINE: `dma` on the engined host, which is what DmaHost is in this
/// suite for, and `interrupt` on the plain one, unless the caller names it.
template <typename Bus>
constexpr I2cEngine engine_of() {
    return Bus::has_dma ? I2cEngine::dma : I2cEngine::interrupt;
}

template <typename Bus>
uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
               I2cSpeed speed = I2cSpeed::standard_100k,
               I2cEngine engine = engine_of<Bus>()) {
    typename Bus::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    r.engine = engine;
    xfer_done = false;
    if (Bus::start(r)) {
        return Bus::status();   // the synchronous refusal
    }
    for (uint32_t spins = 8'000'000u; spins != 0u; --spins) {
        if (xfer_done) {
            return Bus::status();
        }
    }
    return parked;
}

/// A register read as one write-then-read tenure: the index out, a
/// repeated START, the value back.
template <typename Bus = Host>
uint8_t read_regs(uint8_t reg, uint8_t* into, uint8_t n,
                  I2cSpeed speed = I2cSpeed::standard_100k) {
    const uint8_t cmd[1] = {reg};
    return tenure<Bus>(peer_addr, cmd, 1, into, n, speed);
}

uint8_t read_reg(uint8_t reg, I2cSpeed speed = I2cSpeed::standard_100k) {
    uint8_t v = 0;
    (void)read_regs(reg, &v, 1, speed);
    return v;
}

uint8_t write_reg(uint8_t reg, uint8_t value) {
    const uint8_t cmd[2] = {reg, value};
    return tenure<Host>(peer_addr, cmd, 2, nullptr, 0);
}

/// The empty Request: START, the address, STOP - the ACK is the answer.
uint8_t probe(uint8_t addr) { return tenure<Host>(addr, nullptr, 0, nullptr, 0); }

/// Whether `bytes` are the identity the datasheet gives the peer.
bool is_peer_id(const uint8_t* bytes) { return peer_known && same(bytes, peer.id_bytes, peer.id_len); }

#if defined(STM32F446xx)
/// THE TARGET, served from FMPI2C1's own two vectors: the register file
/// the board block describes. `refuse_at` makes it NACK one written byte
/// of a tenure, counted from one (the index is the first): 23.7.2's NACK
/// bit governs "the current received byte", so it is set as the byte
/// BEFORE the refused one is taken - at the address match for the first.
struct Regs {
    static constexpr uint8_t size = 64;
    static inline uint8_t file[size];
    static inline volatile uint8_t ptr = 0;
    static inline bool indexed = false;   ///< the next written byte is the index
    static inline volatile uint8_t took = 0;
    static inline volatile uint8_t refuse_at = 0;

    static void reset() {
        for (uint8_t i = 0; i < size; ++i) {
            file[i] = static_cast<uint8_t>(0x80u + i);
        }
        file[0] = peer.id_bytes[0];
        file[1] = peer.id_bytes[1];
        file[peer.ver_reg] = 0x01;
        file[reg_reset] = 0x00;
        file[peer.scratch_reg] = 0x00;
    }

    /// Up at `clock`, the five events and the errors interrupting.
    template <typename C>
    static bool up(C clk) {
        reset();
        const bool ok = Target::init(clk, FmpI2cAddressConfig{.own = peer.addr},
                                     FmpI2cSpeed::fast_400k);
        Target::interrupt(FmpI2cInterrupt::addr | FmpI2cInterrupt::rx | FmpI2cInterrupt::tx |
                              FmpI2cInterrupt::nack | FmpI2cInterrupt::stop |
                              FmpI2cInterrupt::error,
                          true);
        return ok;
    }

    [[gnu::always_inline]] static void event() {
        switch (Target::service()) {
            case FmpI2cClientEvent::addressed:
                if (!Target::host_reads_last()) {
                    indexed = true;
                    took = 0;
                    if (refuse_at == 1u) {
                        Target::Resource::nack_next(true);
                    }
                }
                break;
            case FmpI2cClientEvent::byte_received: {
                const uint8_t v = Target::take();
                took = static_cast<uint8_t>(took + 1u);
                if (refuse_at != 0u && took + 1u == refuse_at) {
                    Target::Resource::nack_next(true);
                }
                if (indexed) {
                    ptr = static_cast<uint8_t>(v % size);
                    indexed = false;
                } else {
                    if (ptr == reg_reset && v == reset_code) {
                        reset();
                    } else if (ptr == peer.scratch_reg) {
                        file[ptr] = v;
                    }
                    ptr = static_cast<uint8_t>((ptr + 1u) % size);
                }
                break;
            }
            case FmpI2cClientEvent::byte_wanted:
                Target::give(file[ptr]);
                ptr = static_cast<uint8_t>((ptr + 1u) % size);
                break;
            case FmpI2cClientEvent::nacked:
                // The controller's closing NACK: the byte TXIS asked for
                // ahead of it is dropped (23.7.7's TXE flush).
                Target::flush();
                break;
            default:
                break;
        }
    }
    [[gnu::always_inline]] static void error() { (void)Target::error_service(); }
};
#endif

#if defined(STM32F469xx) || defined(STM32F446xx)
void spin_ms(uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
    }
}
#endif

/// The device put back where its reset leaves it, and the time it takes
/// to answer again: the STMPE811's soft reset over the bus, the other
/// board's reset LINE pulsed low - the panel behind it resets too, which
/// is nothing this suite drew on it.
uint8_t peer_reset() {
#if defined(STM32F469xx)
    ResetLine::output();
    ResetLine::clear();
    spin_ms(20);
    ResetLine::set();
    spin_ms(300);
    return i2c_ok;
#elif defined(STM32F446xx)
    const uint8_t cmd[2] = {reg_reset, reset_code};
    return tenure<Host>(peer_addr, cmd, 2, nullptr, 0);
#else
    const uint8_t cmd[2] = {reg_sys_ctrl1, stmpe811_soft_reset};
    const uint8_t st = tenure<Host>(peer_addr, cmd, 2, nullptr, 0);
    for (uint32_t spins = 400'000u; spins != 0u; --spins) {
    }
    return st;
#endif
}

// ---- what the registers held before this program touched them --------------------------------

struct BootState {
    bool gate = false;
    uint32_t cr1 = 0, cr2 = 0, oar1 = 0, oar2 = 0, sr1 = 0, sr2 = 0, ccr = 0, trise = 0;
    bool scl = false, sda = false;   ///< the two lines, read as plain inputs
    uint32_t sr2_ready = 0;          ///< SR2 once the pads are the peripheral's
};
BootState boot;

// =============================================================================
// a - the instances and the block
// =============================================================================

void ta_block() {
    print(serial, "  I2C", bus_instance, " gate at reset: ", boot.gate ? "OPEN" : "closed", "; CR1 ", hex(boot.cr1),
          " CR2 ", hex(boot.cr2), " OAR1 ", hex(boot.oar1), " OAR2 ", hex(boot.oar2), " SR1 ",
          hex(boot.sr1), " SR2 ", hex(boot.sr2), " CCR ", hex(boot.ccr), " TRISE ",
          hex(boot.trise), crlf);
    bench.verdict("every peripheral clock is off at reset, this one included", !boot.gate);
    // Table 125: every register zero but TRISE, which resets to 0x0002.
    // SR2's BUSY is not a stored bit at all - "set by hardware on detection
    // of SDA or SCL low", and "still updated when the interface is
    // disabled" (27.6.7) - so it is read as the WIRE and judged below.
    bench.verdict("the reset values are the chapter's: every register zero but TRISE, which is "
                  "0x0002 (SR2's BUSY excluded: it is a live reading of the wire and not a "
                  "stored bit)",
                  boot.cr1 == 0u && boot.cr2 == 0u && boot.oar1 == 0u && boot.oar2 == 0u &&
                      boot.sr1 == 0u && (boot.sr2 & ~I2C_SR2_BUSY) == 0u && boot.ccr == 0u &&
                      boot.trise == 2u);
    print(serial, "  the wire at boot, read as plain inputs: SCL ", boot.scl ? "high" : "LOW",
          ", SDA ", boot.sda ? "high" : "LOW", "; SR2.BUSY with the pads NOT yet handed over: ",
          (boot.sr2 & I2C_SR2_BUSY) != 0u ? "SET" : "clear", "; after init(): ",
          (boot.sr2_ready & I2C_SR2_BUSY) != 0u ? "SET" : "clear", crlf);
    // The trap this chapter sets for a bring-up, measured: BUSY watches the
    // PERIPHERAL'S line inputs, and those read low while the pad is not in
    // its alternate function - so a block configured before its pads comes
    // up BUSY over an idle wire and its first START never leaves, BUSY
    // being cleared only by a STOP that will never come. init() therefore
    // hands the pads over BEFORE the software reset that takes BUSY with
    // them.
    bench.verdict("with the pads not yet in their alternate function the peripheral reads its own "
                  "inputs LOW and BUSY stands, over a wire both pads read HIGH",
                  boot.scl && boot.sda && (boot.sr2 & I2C_SR2_BUSY) != 0u);
    bench.verdict("... and once init() has handed the pads over and pulsed SWRST behind them, "
                  "BUSY reads the wire and is clear",
                  (boot.sr2_ready & I2C_SR2_BUSY) == 0u);

    print(serial, "  instances on this part:");
    uint8_t count = 0;
    for (uint8_t i = 1; i <= 3u; ++i) {
        if (i2c_present(i)) {
            ++count;
            print(serial, " I2C", i, "(", i2c_on_apb2(i) ? "APB2" : "APB1", ",ev",
                  static_cast<int32_t>(i2c_event_irq(i)), ",er",
                  static_cast<int32_t>(i2c_error_irq(i)), ")");
        }
    }
    print(serial, crlf);
    bench.verdict("this part carries three I2C instances, all on APB1", count == 3u &&
                                                                            !i2c_on_apb2(1) &&
                                                                            !i2c_on_apb2(3));
    bench.verdict("each has TWO vectors of its own, the events on one and the errors on the other",
                  S::event_irq == i2c_event_irq(bus_instance) &&
                      S::error_irq == i2c_error_irq(bus_instance) &&
                      i2c_event_irq(1) != i2c_error_irq(1));
    print(serial, "  the noise filter register: ", S::has_filter ? "present" : "absent",
          "; an FMPI2C1 on this part: ", fmpi2c_present() ? "yes" : "no", crlf);
#if defined(STM32F446xx)
    bench.verdict("this part class has I2C_FLTR - the analog filter's off switch and the digital "
                  "filter - and an FMPI2C1 beside the three, another block and another chapter "
                  "(RM0390 ch. 23)",
                  S::has_filter && fmpi2c_present());
#else
    bench.verdict("this part class has I2C_FLTR - the analog filter's off switch and the digital "
                  "filter - and no FMPI2C1, which is another block and another chapter",
                  S::has_filter && !fmpi2c_present());
#endif

    print(serial, "  APB1 = ", Host::reference_hz() / 1000u, " kHz, PCLK2 = ",
          SysClock::pclk2_hz / 1000u, " kHz, HCLK = ", SysClock::hz / 1000u, " kHz", crlf);
    bench.verdict("the rate FREQ states and CCR divides is ITS OWN bus clock, PCLK1, and not "
                  "SYSCLK",
                  Host::reference_hz() == SysClock::pclk1_hz &&
                      Host::reference_hz() != SysClock::hz);
}

// =============================================================================
// b - the timing arithmetic
// =============================================================================

void tb_timing() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    const uint32_t pclk = Host::reference_hz();
    bool all_stated = true;
    // The duty shape is fast mode's alone, so the standard row is printed
    // once and the fast one under each of its two shapes.
    for (uint8_t duty = 0; duty < 2u; ++duty) {
        if (!host_ready(static_cast<I2cDuty>(duty))) {
            all_stated = false;
            continue;
        }
        for (uint8_t i = 0; i < i2c_speed_count; ++i) {
            const I2cSpeed sp = static_cast<I2cSpeed>(i);
            if (sp == I2cSpeed::standard_100k && duty != 0u) {
                continue;
            }
            const I2cTiming t = Host::timing_of(sp);
            print(serial, "  ",
                  sp == I2cSpeed::standard_100k ? "Sm 100k       "
                                                : (duty == 0u ? "Fm 400k duty 2" : "Fm 400k 16/9  "),
                  ": FREQ ", t.freq_mhz, " CCR ", hex(t.ccr), " TRISE ", t.trise, " -> ",
                  Host::scl_hz(sp), " Hz", crlf);
            if (t.freq_mhz != pclk / 1'000'000u) {
                all_stated = false;
            }
        }
    }
    bench.verdict("FREQ carries the APB1 clock in whole megahertz on every row of the table",
                  all_stated);
    (void)host_ready();

    // What the registers really hold once a speed is applied: the driver
    // writes the row and nothing else.
    const I2cTiming want = Host::timing_of(I2cSpeed::standard_100k);
    const I2cTiming got = S::timing();
    print(serial, "  applied: FREQ ", got.freq_mhz, " CCR ", hex(got.ccr), " TRISE ", got.trise,
          "; the table says FREQ ", want.freq_mhz, " CCR ", hex(want.ccr), " TRISE ", want.trise,
          crlf);
    bench.verdict("the three timing registers in force are exactly the row the arithmetic solved",
                  got.freq_mhz == want.freq_mhz && got.ccr == want.ccr && got.trise == want.trise);
    bench.verdict("... and at 45 MHz of APB1 standard mode is EXACT: CCR 225 gives 100000 Hz",
                  want.ccr == 225u && Host::scl_hz(I2cSpeed::standard_100k) == 100'000UL);
    bench.verdict("TRISE is the specification's 1000 ns in whole APB periods plus one - 46 here",
                  want.trise == 46u);

    // Fast mode at this clock is NOT exact, and the rounding is downwards
    // in frequency: 45 MHz / 1.2 MHz is 37.5, so CCR is 38 and the bus
    // runs slower than asked, never faster.
    const uint32_t fast = Host::scl_hz(I2cSpeed::fast_400k);
    print(serial, "  fast mode at 45 MHz: CCR ", Host::timing_of(I2cSpeed::fast_400k).ccr & 0xFFFu,
          " -> ", fast, " Hz (400000 would want a multiple of 1.2 MHz on APB1)", crlf);
    bench.verdict("CCR is rounded UP, so a bus never runs faster than the speed asked for",
                  fast < 400'000UL && fast > 390'000UL);

    // The window FREQ imposes, and the refusals outside it.
    bench.verdict("below 2 MHz of APB1 no speed is legal (27.6.2)",
                  !i2c_timing_for(1'999'999UL, I2cSpeed::standard_100k).has_value());
    bench.verdict("... and fast mode wants 4 MHz, where standard mode is already legal",
                  !i2c_timing_for(2'000'000UL, I2cSpeed::fast_400k).has_value() &&
                      i2c_timing_for(2'000'000UL, I2cSpeed::standard_100k).has_value());
    bench.verdict("above 50 MHz the FREQ field has no encoding, and the row is refused",
                  !i2c_timing_for(50'000'001UL, I2cSpeed::standard_100k).has_value());
    bench.verdict("a rise time TRISE's six bits cannot hold is refused rather than truncated",
                  !i2c_timing_for(45'000'000UL, I2cSpeed::standard_100k, I2cDuty::ratio_2, 1500)
                       .has_value());

    print(serial, "  table 122's ceiling on the digital filter at this clock: ",
          Host::digital_filter_max(I2cSpeed::standard_100k), " in Sm, ",
          Host::digital_filter_max(I2cSpeed::fast_400k), " in Fm; ES0206 2.10.4 puts a repeated "
          "START at ", Host::scl_hz(I2cSpeed::standard_100k), " Hz ",
          Host::repeated_start_setup_at_risk(I2cSpeed::standard_100k) ? "IN" : "outside",
          " the window where its setup time can be violated", crlf);
    bench.verdict("a 100 kHz standard-mode bus sits inside that errata window, and the driver "
                  "says so instead of promising the timing",
                  Host::repeated_start_setup_at_risk(I2cSpeed::standard_100k) &&
                      !Host::repeated_start_setup_at_risk(I2cSpeed::fast_400k));
}

// =============================================================================
// c - the refusals
// =============================================================================

void tc_refusals() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    // An own address outside its field: refused whole, nothing written.
    (void)S::addresses(I2cAddressConfig{.own = 0x41});
    const uint32_t before = S::regs().OAR1;
    bench.verdict("an own address past seven bits is refused",
                  !S::addresses(I2cAddressConfig{.own = 0x80}));
    bench.verdict("... and a refused address configuration writes nothing",
                  S::regs().OAR1 == before);
    bench.verdict("a ten-bit own address WITH a second one is refused - the dual arrangement is "
                  "seven-bit only (27.6.4)",
                  !S::addresses(I2cAddressConfig{.own = 0x123, .ten_bit = true, .second = 0x10}));
    bench.verdict("the same ten-bit address alone is accepted",
                  S::addresses(I2cAddressConfig{.own = 0x123, .ten_bit = true}));
    // 27.6.3: bit 14 of OAR1 "should always be kept at 1 by software".
    print(serial, "  OAR1 after a ten-bit address: ", hex(S::regs().OAR1), crlf);
    bench.verdict("bit 14 of OAR1 is kept at one by every write, as 27.6.3 asks",
                  (S::regs().OAR1 & (1u << 14)) != 0u);

    // The filter is enable-protected and its length is four bits.
    S::enable();
    bench.verdict("the noise filters are refused while the peripheral is enabled",
                  !S::filter(I2cFilter{.analog = false, .digital = 1}));
    S::disable();
    bench.verdict("with PE down they are accepted", S::filter(I2cFilter{.analog = false,
                                                                       .digital = 4}));
    const I2cFilter back = S::filter();
    print(serial, "  FLTR reads analog ", back.analog ? "on" : "OFF", ", digital ", back.digital,
          crlf);
    bench.verdict("... and read back as they were written", !back.analog && back.digital == 4u);
    bench.verdict("a digital filter past the four bits of DNF is refused",
                  !S::filter(I2cFilter{.digital = 16}));

    // The engine's one synchronous refusal: a speed the clock cannot make.
    (void)host_ready();
    bench.verdict("both speeds are reachable at 45 MHz of APB1",
                  Host::speed_ok(I2cSpeed::standard_100k) && Host::speed_ok(I2cSpeed::fast_400k));
    print(serial, "  (a speed the clock in force cannot produce is answered i2c_rejected inside "
                  "start(), no byte moved - and at this clock there is no such speed to show)",
          crlf);

    // The engine's other synchronous refusal: an address no Request may
    // name (util/i2c_bus.hpp) - the general call, the codes below 0x08, the
    // four 10-bit prefixes this block would send as a header (ADD10) and
    // the four above them. Each is answered true and i2c_rejected by
    // start() with nothing written: CR1 and CR2 as they were (no START, no
    // event vector raised) and the controller never on the wire (MSL).
    (void)host_ready();
    const uint32_t cr1 = S::regs().CR1;
    const uint32_t cr2 = S::regs().CR2;
    const uint8_t refused[] = {0x00, 0x07, 0x78, 0x7A, 0x7B, 0x7C, 0x7F};
    uint8_t refusals = 0;
    for (const uint8_t a : refused) {
        Host::Request r{};
        r.addr = a;
        const bool sync = Host::start(r);
        if (sync && Host::status() == i2c_rejected && S::regs().CR1 == cr1 && S::regs().CR2 == cr2 &&
            !S::host()) {
            ++refusals;
        }
    }
    print(serial, "  addresses outside 0x08..0x77 refused inside start(), nothing written: ", refusals,
          " of ", static_cast<uint8_t>(sizeof refused), crlf);
    bench.verdict("an address outside 0x08..0x77 is answered i2c_rejected inside start(), CR1 and "
                  "CR2 untouched and the controller never on the wire",
                  refusals == sizeof refused);
}

// =============================================================================
// d - the address scan
// =============================================================================

void td_scan() {
    if (!host_ready()) {
        bench.verdict("the host came up", false);
        return;
    }
    uint8_t answered = 0;
    uint8_t nacked = 0;
    uint8_t other = 0;
    uint8_t first = 0;
    print(serial, "  answering addresses:");
    for (uint8_t a = 0x08; a <= 0x77u; ++a) {
        const uint8_t st = probe(a);
        if (st == i2c_ok) {
            ++answered;
            if (first == 0u) {
                first = a;
            }
            print(serial, " ", hex(a));
        } else if (st == i2c_nack_addr) {
            ++nacked;
        } else {
            ++other;
        }
    }
    print(serial, answered == 0u ? " (none)" : "", crlf);
    print(serial, "  ", answered, " answered, ", nacked, " NACKed, ", other,
          " ended some other way; ", peer.name, " is ", crlf, "  ");
    if (peer.addr != 0u) {
        print(serial, "at ", hex(peer.addr), " by the board's schematic", crlf);
    } else {
        print(serial, "at the first address that answered, ", hex(peer_addr), crlf);
    }
    bench.verdict("the probe - a Request with both spans empty - reaches the wire: at least one "
                  "device on this board answers its address",
                  answered != 0u);
    bench.verdict("every address nobody answers comes back i2c_nack_addr, and nothing comes back "
                  "any other way",
                  other == 0u && nacked == (0x77u - 0x08u + 1u) - answered);
    bench.verdict("the peer is one of them", peer_addr != 0u);
    print(serial, "  a second scan repeats it: ", probe(peer_addr) == i2c_ok ? "yes" : "NO",
          "; an address next to it still NACKs: ",
          probe(static_cast<uint8_t>(peer_addr + 1u)) == i2c_nack_addr ? "yes" : "NO", crlf);
    bench.verdict("the scan is repeatable, and a NACK on the address leaves the bus fit for the "
                  "next tenure",
                  probe(peer_addr) == i2c_ok);
}

// =============================================================================
// e - the identity
// =============================================================================

void te_identity() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    uint8_t id[2] = {};
    const uint8_t st = read_regs(reg_chip_id, id, 2);
    const uint8_t ver = read_reg(reg_id_ver);
    print(serial, "  address ", hex(peer_addr), ": ", peer.id_name, " reads ", hex(id[0]), " ",
          hex(id[1]), ", ", peer.ver_name, " reads ", hex(ver), "; status ", st, crlf);
    bench.verdict("a write-then-read tenure - the index out, a repeated START, the value back - "
                  "answers i2c_ok",
                  st == i2c_ok);
    if constexpr (peer_known) {
        bench.verdict("the device on this bus is the one the board's schematic names, and it says "
                      "so in its own identity register",
                      is_peer_id(id));
    } else {
        print(serial, "  the device's datasheet is not on the desk: its identity is printed and "
                      "not judged (declined by name)", crlf);
    }

    // The same read eight times running: a bus that is right is right
    // every time.
    bool stable = true;
    for (uint8_t i = 0; i < 8u; ++i) {
        uint8_t again[2] = {};
        (void)read_regs(reg_chip_id, again, 2);
        if (!same(id, again, 2)) {
            stable = false;
        }
    }
    bench.verdict("eight write-then-read tenures in a row give the same two bytes", stable);

    // A register that is not the identity reads as something else, which
    // is what says the bytes above are the device's and not the wire's -
    // a judgement that needs the datasheet to name a register that
    // differs, so without it the line is printed and not judged.
    if constexpr (peer_known) {
        const uint8_t ctrl = read_reg(reg_sys_ctrl2);
        print(serial, "  ", peer.scratch_name, " reads ", hex(ctrl),
              " - a different register, a different byte", crlf);
        bench.verdict("a different register answers differently: the bytes come from the device "
                      "and not from a pull-up",
                      ctrl != id[0] || ctrl != id[1]);
    } else {
        print(serial, "  which register would answer differently is the datasheet's to say: "
                      "not judged here (declined by name)", crlf);
    }
}

// =============================================================================
// f - the three receive choreographies
// =============================================================================

void tf_receive_shapes() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    // 27.3.3 gives the controller receiver THREE different procedures by
    // count, and this is where they are proved to agree: the same
    // registers read one byte at a time, two at a time and eight at a
    // time must come back identical.
    uint8_t one[8] = {};
    for (uint8_t i = 0; i < 8u; ++i) {
        one[i] = read_reg(static_cast<uint8_t>(reg_chip_id + i));
    }
    uint8_t two[8] = {};
    for (uint8_t i = 0; i < 8u; i += 2u) {
        (void)read_regs(static_cast<uint8_t>(reg_chip_id + i), &two[i], 2);
    }
    uint8_t eight[8] = {};
    const uint8_t st = read_regs(reg_chip_id, eight, 8);

    print(serial, "  one at a time:", crlf, "   ");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", hex(one[i]));
    }
    print(serial, crlf, "  two at a time (POS and the single BTF):", crlf, "   ");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", hex(two[i]));
    }
    print(serial, crlf, "  eight in one tenure (the BTF pair at the tail):", crlf, "   ");
    for (uint8_t i = 0; i < 8u; ++i) {
        print(serial, " ", hex(eight[i]));
    }
    print(serial, crlf);

    // With the datasheet on the desk the bytes are the identity; without
    // it the same one-byte read done again is what a right procedure owes.
    uint8_t one_again = 0;
    (void)read_regs(reg_chip_id, &one_again, 1);
    bench.verdict("the ONE-byte procedure - ACK cleared before ADDR, STOP right after - reads the "
                  "device",
                  peer_known ? is_peer_id(one) : one_again == one[0]);
    bench.verdict("the TWO-byte procedure - POS with ACK, ACK off after ADDR, both bytes off one "
                  "BTF - agrees with it byte for byte",
                  same(one, two, 8));
    bench.verdict("the N-byte procedure - RxNE until three remain, then two BTFs - agrees with "
                  "both",
                  st == i2c_ok && same(one, eight, 8));

    // The device auto-increments its own register pointer, so a long read
    // is one tenure and not eight: the count is what changes, and the
    // three answers above are what says the choreography is right.
    uint8_t run[16] = {};
    const uint8_t long_st = read_regs(reg_chip_id, run, 16);
    bench.verdict("a sixteen-byte read is one tenure and ends i2c_ok",
                  long_st == i2c_ok && same(run, one, 8));
}

// =============================================================================
// g - the speeds, with SCL counted on the pad
// =============================================================================

/// Count SCL's rising edges while a tenure runs, and keep the SHORTEST
/// period between two of them - which is the bit period, every software
/// sequence of the chapter only ever lengthening one. The pad is an
/// open-drain alternate function, so its input buffer reads the wire.
struct SclReading {
    uint32_t edges = 0;
    uint32_t shortest = 0;   ///< the shortest gap between two edges, in core cycles
    uint32_t average = 0;    ///< the mean gap over the whole tenure, in core cycles
};

SclReading count_scl() {
    SclReading r{};
    const uint32_t period = systick_period();
    uint32_t last_val = SysTick->VAL;
    uint32_t shortest = 0xFFFFFFFFu;
    uint32_t total = 0;
    bool prev = SclPad::read();
    for (uint32_t spins = 40'000'000u; spins != 0u && !xfer_done; --spins) {
        const uint32_t v = SysTick->VAL;
        const bool now = SclPad::read();
        if (now && !prev) {
            const uint32_t d = (last_val >= v) ? (last_val - v) : (last_val + period - v);
            if (r.edges != 0u) {
                total += d;
                if (d < shortest) {
                    shortest = d;
                }
            }
            last_val = v;
            ++r.edges;
        }
        prev = now;
    }
    r.shortest = shortest == 0xFFFFFFFFu ? 0u : shortest;
    r.average = r.edges > 1u ? total / (r.edges - 1u) : 0u;
    return r;
}

/// A DMA-carried read of `n` bytes, with SCL watched from the loop: the
/// engines keep the CPU out of the byte pump, so what the pad shows is the
/// wire and not the software.
SclReading timed_read(uint8_t n, I2cSpeed speed, uint8_t* into) {
    const uint8_t cmd[1] = {reg_chip_id};
    DmaHost::Request r{};
    r.addr = peer_addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(into);
    r.rx_len = n;
    r.speed = speed;
    r.engine = I2cEngine::dma;
    xfer_done = false;
    if (DmaHost::start(r)) {
        return SclReading{};
    }
    return count_scl();
}

void tg_speeds() {
    uint8_t got[32] = {};
    uint8_t reference[32] = {};
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    (void)read_regs(reg_chip_id, reference, 32);

    Dma<1>::init();
    struct Row {
        I2cSpeed speed;
        I2cDuty duty;
        const char* name;
    };
    const Row rows[3] = {{I2cSpeed::standard_100k, I2cDuty::ratio_2, "Sm 100k       "},
                         {I2cSpeed::fast_400k, I2cDuty::ratio_2, "Fm 400k duty 2"},
                         {I2cSpeed::fast_400k, I2cDuty::ratio_16_9, "Fm 400k 16/9  "}};
    uint8_t exact = 0;
    bool all_ok = true;
    uint32_t wire_hz[3] = {};   ///< each row's mean rate on the pad
    for (uint8_t i = 0; i < 3u; ++i) {
        bus_ao_live = false;
        if (!DmaHost::init(clock, I2cHostConfig{.duty = rows[i].duty})) {
            all_ok = false;
            continue;
        }
        dma_host_live = true;
        for (uint8_t k = 0; k < 32u; ++k) {
            got[k] = 0;
        }
        const SclReading r = timed_read(32, rows[i].speed, got);
        const uint8_t st = DmaHost::status();
        dma_host_live = false;
        const uint32_t stated = DmaHost::scl_hz(rows[i].speed);
        const uint32_t high = r.shortest == 0u ? 0u : SysClock::hz / r.shortest;
        const uint32_t low = r.average == 0u ? 0u : SysClock::hz / r.average;
        // The ORDER is judged on the mean: it reads the same run after run
        // (456 and 501 core cycles in the two fast rows), where the
        // shortest gap moves with the polling loop's phase by more than the
        // two fast rows differ (413 to 489 in the 16/9 row, measured).
        wire_hz[i] = low;
        // The address with the write bit, the index, the address again
        // with the read bit and the 32 bytes: nine clocks a byte,
        // thirty-five bytes.
        const uint32_t expected_edges = 9u * (3u + 32u);
        if (st == i2c_ok && same(reference, got, 32)) {
            ++exact;
        }
        print(serial, "  ", rows[i].name, ": CCR ", DmaHost::timing_of(rows[i].speed).ccr & 0xFFFu,
              " states ", stated, " Hz; the pad gave ", r.edges, " rising edges of ",
              expected_edges, ", the shortest gap ", r.shortest, " core cycles (", high,
              " Hz) and the mean ", r.average, " (", low, " Hz); status ", st, crlf);
        if (st != i2c_ok) {
            all_ok = false;
        }
    }
    DmaHost::release();
    (void)host_ready();

    bench.verdict("the device answers at both speeds and under both fast-mode duty shapes, and "
                  "the bytes are the same at each",
                  exact == 3u);
    bench.verdict("every one of the three tenures ended i2c_ok", all_ok);
    bench.verdict("the three rows are three different rates ON THE WIRE, in the order the CCR "
                  "arithmetic puts them: fast mode at DUTY 2 above fast mode at 16/9, and both "
                  "well above standard mode",
                  wire_hz[1] > wire_hz[2] && wire_hz[2] > wire_hz[0]);
    print(serial, "  (the count is nine clocks for each of the tenure's thirty-five bytes, plus "
                  "the STOP's own rise where the loop is still watching when it comes. The two "
                  "rates BRACKET the bit rate: the mean gap counts the two address phases, whose "
                  "software sequences stretch SCL low, so it reads slow; the shortest gap is one "
                  "bit period sampled by a polling loop that timestamps a little before the edge "
                  "it sees, so it reads fast. What the CCR arithmetic states sits between them; "
                  "only the ORDER of the means is judged - one board, one set of pull-ups)",
          crlf);
}

// =============================================================================
// h - a register written and read back, then the device's own reset
// =============================================================================

void th_write_back() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    if constexpr (!peer_known) {
        print(serial, "  the device's datasheet is not on the desk, so no register of its is "
                      "written: the letter declines by name", crlf);
        return;
    } else {
        // The scratch register is one the datasheet says moves no pin: the
        // STMPE811's SYS_CTRL2 is four clock gates.
        const uint8_t start = read_reg(reg_sys_ctrl2);
        bool wrote = true;
        for (uint8_t i = 0; i < 3u; ++i) {
            const uint8_t st = write_reg(reg_sys_ctrl2, peer.scratch_patterns[i]);
            const uint8_t back = read_reg(reg_sys_ctrl2);
            print(serial, "  ", peer.scratch_name, " <- ", hex(peer.scratch_patterns[i]),
                  " (status ", st, "), reads ", hex(back), crlf);
            if (st != i2c_ok || back != peer.scratch_patterns[i]) {
                wrote = false;
            }
        }
        bench.verdict("a register written over this bus reads back exactly - the write path and "
                      "the read path are the same tenure with a repeated START between them",
                      wrote);
#if defined(STM32F429xx)
        // And one pattern the DEVICE masks: with its ADC gate open this
        // part holds the temperature-sensor gate clear whatever is
        // written. A device rule and not a bus one, so it is printed and
        // not judged - what the bus owes is that the byte written is the
        // byte that arrived, which the three above say.
        const uint8_t masked_st = write_reg(reg_sys_ctrl2, 0x0Cu);
        print(serial, "  SYS_CTRL2 <- 0xC (status ", masked_st, "), reads ",
              hex(read_reg(reg_sys_ctrl2)),
              " - the device masks a bit of its own, and that is the device's business", crlf);
#endif

        // A two-byte write in one tenure: the index and the value are one
        // span, so this is already what every write above did; what it
        // proves here is that the tenure ends on BTF and not on TxE.
        const uint8_t st = write_reg(reg_sys_ctrl2, start);
        print(serial, "  ", peer.scratch_name, " back to the value found at boot, ", hex(start),
              " (status ", st, ")", crlf);
        bench.verdict("the boot value is restored",
                      st == i2c_ok && read_reg(reg_sys_ctrl2) == start);

        // And the device's own reset, which is how this suite leaves it:
        // every register of its back where its power-on put them.
        const uint8_t reset_st = peer_reset();
        uint8_t id[2] = {};
        (void)read_regs(reg_chip_id, id, 2);
        const uint8_t after = read_reg(reg_sys_ctrl2);
        print(serial, "  after the device's reset: ", peer.id_name, " ", hex(id[0]), " ",
              hex(id[1]), ", ", peer.scratch_name, " ", hex(after),
              " (its reset value; boot found ", hex(peer_sys_ctrl2_boot), ")", crlf);
        bench.verdict("the device's own reset goes out and the device comes back on the bus, its "
                      "registers where its reset leaves them",
                      reset_st == i2c_ok && is_peer_id(id));
    }
}

// =============================================================================
// i - the kernel
// =============================================================================

namespace kl {

/// The per-bus timeout is 20 ms: two orders above any tenure letter i
/// queues (a register read is about 0.4 ms at 100 kHz), and the limit
/// letter n's peer stretches past.
constexpr uint32_t bus_timeout_ticks = ticks_from_ms<P>(20);
using I2cArb = I2cBus<Host, P, 4, BusPassThrough, bus_timeout_ticks>;

uint8_t rx_a[2];
const uint8_t index_chip_id[1] = {reg_chip_id};

class Probe {
public:
    using Event = std::variant<I2cDone, SleepVote>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t replies[8];
    static inline uint8_t n = 0;
    static inline uint8_t rejected = 0;
    static inline uint8_t votes = 0;
    static inline bool last_vote = false;

    static void init() { clear_tally(); }
    static void clear_tally() {
        n = 0;
        rejected = 0;
        votes = 0;
        last_vote = false;
    }

    static void dispatch(const Event& e) {
        brio::match(
            e,
            [](const I2cDone& d) {
                if (n < 8u) {
                    replies[n] = d.status;
                }
                ++n;
                if (d.status == i2c_rejected) {
                    ++rejected;
                }
            },
            [](const SleepVote& v) {
                ++votes;
                last_vote = v.ok;
            });
    }
};

using BusKernel = Tenuto<P, Probe, I2cArb>;

void pump() {
    TimeEvents<P>::process();
    while (BusKernel::step()) {
        TimeEvents<P>::process();
    }
}

void pump_until(uint8_t want, uint32_t ms) {
    const uint32_t t0 = Ticker::ticks();
    while (Ticker::ticks() - t0 < ms) {
        pump();
        if (Probe::n >= want) {
            break;
        }
    }
    pump();
}

Host::Request request(uint8_t addr, uint8_t* rx, uint8_t rx_len) {
    Host::Request r{};
    r.addr = addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(index_chip_id));
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.reply = reply_to<Probe, I2cDone>();
    return r;
}

}  // namespace kl

void ti_kernel() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    kl::BusKernel::init_all();
    bus_ao_live = true;

    for (uint8_t i = 0; i < 4u; ++i) {
        post<kl::I2cArb>(kl::request(peer_addr, (i == 3u) ? kl::rx_a : nullptr, (i == 3u) ? 2 : 1));
    }
    kl::pump_until(4, 500);
    print(serial, "  four queued tenures: replies ", kl::Probe::n, " [", kl::Probe::replies[0], " ",
          kl::Probe::replies[1], " ", kl::Probe::replies[2], " ", kl::Probe::replies[3],
          "], the last one read ", hex(kl::rx_a[0]), " ", hex(kl::rx_a[1]), crlf);
    bench.verdict("four tenures through I2cBus, four replies - util/i2c_bus.hpp and "
                  "util/bus_master.hpp unchanged on this architecture",
                  kl::Probe::n == 4u);
    bench.verdict("... every one i2c_ok",
                  kl::Probe::replies[0] == i2c_ok && kl::Probe::replies[1] == i2c_ok &&
                      kl::Probe::replies[2] == i2c_ok && kl::Probe::replies[3] == i2c_ok);
    bench.verdict("and what the arbiter handed back is the device's own identity",
                  (peer_known ? is_peer_id(kl::rx_a) : true));

    // The address NACK travels through the arbiter untouched: a probe of
    // an address nobody answers is the scanner's own result.
    kl::Probe::clear_tally();
    Host::Request nobody{};
    nobody.addr = nobody_addr;
    nobody.reply = reply_to<kl::Probe, I2cDone>();
    post<kl::I2cArb>(nobody);
    kl::pump_until(1, 200);
    print(serial, "  a probe of an address nobody answers came back ", kl::Probe::replies[0], " (",
          i2c_nack_addr, " is i2c_nack_addr)", crlf);
    bench.verdict("the wire's own outcome reaches the requester untouched: i2c_nack_addr",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_nack_addr);

    kl::Probe::clear_tally();
    for (uint8_t i = 0; i < 6u; ++i) {
        post<kl::I2cArb>(kl::request(peer_addr, nullptr, 1));
    }
    kl::pump_until(6, 500);
    print(serial, "  six posted into a four-deep queue: replies ", kl::Probe::n, ", rejected ",
          kl::Probe::rejected, crlf);
    bench.verdict("the arbiter rejects what it cannot queue, immediately", kl::Probe::rejected != 0u);
    bench.verdict("... and every request is still answered exactly once", kl::Probe::n == 6u);

    kl::Probe::clear_tally();
    post<kl::I2cArb>(PrepareSleep{
        .depth = SleepDepth::standby,
        .reply = reply_to<kl::Probe, SleepVote>(),
    });
    kl::pump();
    bench.verdict("an IDLE bus votes for the sleep", kl::Probe::votes == 1u && kl::Probe::last_vote);

    bus_ao_live = false;
    (void)host_ready();
}

// =============================================================================
// j - the DMA engines
// =============================================================================

/// The two engines' streams, read and written by letter j between
/// tenures (each stopped then: its block completed or never started).
using TxStreamRegs = DmaStream<TxEngine::controller, TxEngine::stream>;
using RxStreamRegs = DmaStream<RxEngine::controller, RxEngine::stream>;

/// What one tenure left on the engines. WHICH MECHANISM CARRIED A PHASE is
/// read from two things: the stream's memory address, zeroed before the
/// tenure with the stream stopped and written by nothing but a block start
/// (stm32f4/dma.hpp's launch()), so SxM0AR naming this tenure's buffer
/// afterwards means its engine was started for it; and the stream's
/// vector, entered exactly once by the block's completion (its transfer
/// complete, the engines' one enabled flag). Both are identities, no
/// tolerance.
struct EngineTrace {
    uint8_t status;
    bool tx_carried;
    bool rx_carried;
    uint32_t tx_entries;
    uint32_t rx_entries;
};

template <typename Bus>
EngineTrace traced(const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len,
                   I2cEngine engine) {
    // SxM0AR takes a write only with EN down (RM0390 9.5.8); between two
    // tenures both streams have stopped.
    TxStreamRegs::regs().M0AR = 0;
    RxStreamRegs::regs().M0AR = 0;
    tx_dma_entries = 0;
    rx_dma_entries = 0;
    const uint8_t st = tenure<Bus>(peer_addr, tx, tx_len, rx, rx_len, I2cSpeed::standard_100k,
                                   engine);
    return {st,
            tx != nullptr && TxStreamRegs::regs().M0AR == reinterpret_cast<uint32_t>(tx),
            rx != nullptr && RxStreamRegs::regs().M0AR == reinterpret_cast<uint32_t>(rx),
            tx_dma_entries, rx_dma_entries};
}

void tj_engines() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    uint8_t reference[8] = {};
    (void)read_regs(reg_chip_id, reference, 8);
    // Read out of the device what the engined write below will disturb,
    // while the plain host still owns the vectors.
    const uint8_t start = read_reg(reg_sys_ctrl2);

    // `dma` on the host with no engines bound: refused inside start(), CR1
    // and CR2 as they were (no START, no event vector raised) and the
    // controller never on the wire (MSL) - then the same read on `interrupt`
    // exact on the same host.
    {
        const uint8_t cmd[1] = {reg_chip_id};
        uint8_t got[8] = {};
        // The last read's STOP off the wire first - a bit period after its
        // completion, which comes when the STOP is requested - so CR1 and
        // MSL are read at rest: two ticks, the way letter m waits.
        const uint32_t t0 = Ticker::ticks();
        while (Ticker::ticks() - t0 < 2u) {
        }
        const uint32_t cr1 = S::regs().CR1;
        const uint32_t cr2 = S::regs().CR2;
        Host::Request q{};
        q.addr = peer_addr;
        q.tx = lend<Lease::reply>(static_cast<const uint8_t*>(cmd));
        q.tx_len = 1;
        q.rx = lend<Lease::reply>(static_cast<uint8_t*>(got));
        q.rx_len = 8;
        q.engine = I2cEngine::dma;
        const bool sync = Host::start(q);
        const uint8_t st = Host::status();
        const bool refused = sync && st == i2c_rejected && S::regs().CR1 == cr1 &&
                             S::regs().CR2 == cr2 && !S::host();
        q.engine = static_cast<I2cEngine>(3);
        const bool past = Host::start(q) && Host::status() == i2c_rejected;
        const uint8_t next = read_regs<Host>(reg_chip_id, got, 8);
        print(serial, "  dma on the engineless host: start() ", sync, ", status ", st,
              refused ? ", nothing written, MSL clear" : ", SOMETHING MOVED",
              "; the next read on interrupt: status ", next, crlf);
        bench.verdict("dma on a host with no engines bound is answered i2c_rejected inside "
                      "start(), CR1 and CR2 untouched and the controller never on the wire",
                      refused);
        bench.verdict("... a value past the enumeration the same way", past);
        bench.verdict("... and the next read on that host is exact",
                      next == i2c_ok && same(reference, got, 8));
    }

    Dma<1>::init();
    bus_ao_live = false;
    if (!DmaHost::init(clock)) {
        bench.verdict("the engined host came up", false);
        return;
    }
    dma_host_live = true;

    // A write phase of one (the index) on the transmit engine, a read phase
    // of eight on the receive engine under CR2.LAST.
    const uint8_t index[1] = {reg_chip_id};
    uint8_t got[8] = {};
    EngineTrace t = traced<DmaHost>(index, 1, got, 8, I2cEngine::dma);
    print(serial, "  dma, the index and eight: ", hex(got[0]), " ", hex(got[1]), " ",
          hex(got[2]), " ", hex(got[3]), " ", hex(got[4]), " ", hex(got[5]), " ", hex(got[6]),
          " ", hex(got[7]), ", status ", t.status, "; streams started tx ", t.tx_carried, " rx ",
          t.rx_carried, ", vector entries tx ", t.tx_entries, " rx ", t.rx_entries, crlf);
    bench.verdict("a write-then-read under dma ends i2c_ok, both phases on the engines - the "
                  "one-byte index too - each stream completing once",
                  t.status == i2c_ok && t.tx_carried && t.rx_carried && t.tx_entries == 1u &&
                      t.rx_entries == 1u);
    bench.verdict("... and the block is byte-exact against the same read on the pump",
                  same(reference, got, 8));

    // THE SHORTEST READ: one byte, on the receive engine under 24.3.8's
    // own sequence (ACK cleared at the SB, the STOP after ADDR's clear).
    uint8_t one = 0;
    t = traced<DmaHost>(index, 1, &one, 1, I2cEngine::dma);
    print(serial, "  dma, the index and one: ", hex(one), ", status ", t.status,
          "; streams started tx ", t.tx_carried, " rx ", t.rx_carried, ", vector entries tx ",
          t.tx_entries, " rx ", t.rx_entries, crlf);
    bench.verdict("a one-byte read under dma rides the receive engine, its stream completing "
                  "once, and it is the same byte",
                  t.status == i2c_ok && t.rx_carried && t.rx_entries == 1u && one == reference[0]);
    // THE SHORTEST WRITE: the index alone, a write of one byte, on the
    // transmit engine; the device's pointer is all it moves.
    t = traced<DmaHost>(index, 1, nullptr, 0, I2cEngine::dma);
    print(serial, "  dma, a write of one: status ", t.status, "; stream started ", t.tx_carried,
          ", vector entries tx ", t.tx_entries, " rx ", t.rx_entries, crlf);
    bench.verdict("a one-byte write under dma rides the transmit engine, its stream completing "
                  "once",
                  t.status == i2c_ok && t.tx_carried && t.tx_entries == 1u && t.rx_entries == 0u);
    // `interrupt` on the engined host: the same read of eight on the
    // vectors, no stream started, no stream vector entered.
    for (uint8_t i = 0; i < 8u; ++i) {
        got[i] = 0;
    }
    t = traced<DmaHost>(index, 1, got, 8, I2cEngine::interrupt);
    print(serial, "  interrupt on the engined host, the index and eight: status ", t.status,
          "; streams started tx ", t.tx_carried, " rx ", t.rx_carried, ", vector entries tx ",
          t.tx_entries, " rx ", t.rx_entries, crlf);
    bench.verdict("interrupt on the engined host touches no stream, and the read is exact",
                  t.status == i2c_ok && !t.tx_carried && !t.rx_carried && t.tx_entries == 0u &&
                      t.rx_entries == 0u && same(reference, got, 8));

    // A write of two bytes through the transmit engine, read back on the
    // pump: the engined write path really moves the bytes. Without the
    // device's datasheet nothing of its is written, and the write phase
    // is proved on the index byte every read above sent through the same
    // engine.
    if constexpr (peer_known) {
        const uint8_t cmd[2] = {reg_sys_ctrl2, peer.scratch_patterns[0]};
        const uint8_t wr = tenure<DmaHost>(peer_addr, cmd, 2, nullptr, 0);
        dma_host_live = false;
        DmaHost::release();
        (void)host_ready();
        const uint8_t back = read_reg(reg_sys_ctrl2);
        print(serial, "  a two-byte write through the transmit engine: status ", wr,
              ", the register reads ", hex(back), " (it held ", hex(start), " before)", crlf);
        bench.verdict("the engined write phase reaches the device",
                      wr == i2c_ok && back == peer.scratch_patterns[0]);
        (void)write_reg(reg_sys_ctrl2, start);
        (void)peer_reset();
    } else {
        dma_host_live = false;
        DmaHost::release();
        (void)host_ready();
        (void)start;
        print(serial, "  no register of the device is written (its datasheet is not on the desk): "
                      "the engined write phase is the index byte of the reads above (declined by "
                      "name)", crlf);
    }
}

// =============================================================================
// k - the recovery verbs
// =============================================================================

void tk_recovery() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    // SWRST puts the state machines and the flags back and keeps nothing:
    // 27.6.1's own verb, and ES0206 2.10.3's workaround for a START that
    // will not come out. The registers behind it are the caller's to
    // rewrite, which is what recover() does.
    (void)S::addresses(I2cAddressConfig{.own = 0x33});
    const uint32_t before_ccr = S::regs().CCR;
    const uint32_t before_oar = S::regs().OAR1;
    S::software_reset();
    const uint32_t after_ccr = S::regs().CCR;
    const uint32_t after_oar = S::regs().OAR1;
    const uint32_t after_cr1 = S::regs().CR1;
    print(serial, "  SWRST pulsed: CCR ", hex(before_ccr), " -> ", hex(after_ccr), ", OAR1 ",
          hex(before_oar), " -> ", hex(after_oar), ", CR1 ", hex(after_cr1), crlf);
    bench.verdict("the software reset puts the whole register file back, the configuration "
                  "included - so a driver that uses it must rewrite the timing",
                  after_ccr == 0u && after_oar == 0u && (after_cr1 & I2C_CR1_PE) == 0u);

    const bool recovered = Host::recover();
    const I2cTiming t = S::timing();
    print(serial, "  after recover(): FREQ ", t.freq_mhz, " CCR ", hex(t.ccr), " TRISE ", t.trise,
          ", PE ", S::enabled() ? "up" : "DOWN", crlf);
    bench.verdict("recover() is the SWRST with the timing written again and PE back up",
                  recovered && S::enabled() && t.ccr == Host::timing_of(I2cSpeed::standard_100k).ccr);
    uint8_t id[2] = {};
    const uint8_t st = read_regs(reg_chip_id, id, 2);
    bench.verdict("... and the bus works after it",
                  st == i2c_ok && (peer_known ? id[0] == peer.id_bytes[0] : true));

    // The unstick: nine clocks and a STOP by hand, only when SDA is
    // actually held low. A healthy wire is left alone and says 0.
    const uint8_t pulses = Host::unstick();
    print(serial, "  unstick() on a healthy wire: ", pulses,
          " pulses (0 = SDA was free and nothing was driven)", crlf);
    bench.verdict("unstick() finds this bus free and drives nothing", pulses == 0u);
    const uint8_t after = read_regs(reg_chip_id, id, 2);
    bench.verdict("... and the pads come back to the peripheral",
                  after == i2c_ok && (peer_known ? is_peer_id(id) : true));

    // The errata's own counter: how many BUS ERRORS were seen and ignored
    // while this side was the controller. RM0090 27.3.4 says a controller's
    // transfer is unaffected by one and ES0206 2.10.1 says one can be
    // raised where there was none, so the driver counts instead of aborting.
    print(serial, "  bus errors seen and ignored since boot: ", Host::spurious_bus_errors(),
          " (ES0206 2.10.1: in controller mode a BERR is spurious or harmless, and the tenure "
          "runs on)", crlf);
    bench.verdict("the tenures above all completed, whatever BERR did - the count is printed and "
                  "not judged, one board being one specimen",
                  true);
}

// =============================================================================
// the menu
// =============================================================================

#if defined(STM32F446xx)
// =============================================================================
// m - the refused last byte (the Nucleo-F446RE's self-link)
// =============================================================================

/// THE CORES letter m runs at: the boot rate, the PLL at 48 MHz and the
/// HSI alone at 16 MHz - a DynamicClock over the three, whose switch parks
/// on the HSI between PLL rates (clock.hpp). The slower the core, the later
/// the error vector's answer to a NACK lands against what the controller
/// does on its own after it.
using Core48 = Clock<ClockSource::pll_hse, 48'000'000, 8'000'000, HseMode::bypass>;
using Core16 = Clock<ClockSource::hsi, 16'000'000>;
// THE LIVE ENGINE LAST IN THE FAN-OUT: both hosts re-apply their own
// cached speed to the one CCR and TRISE they share, so the last one
// rebased decides what the registers hold - Host, which letter n's rebase
// leg times.
using Cores = DynamicClock<Rates<SysClock, Core48, Core16>, Ticker, Serial, DmaHost, Host>;
constexpr Cores cores;

/// What a run of refusals came to, over one core, one host, one speed.
struct RefusalTally {
    uint8_t refused;     ///< refusals tried
    uint8_t nack_data;   ///< answered i2c_nack_data
    uint8_t missed;      ///< i2c_ok: the TARGET was late with its NACK, the byte taken
    uint8_t let_go;      ///< START and STOP down and MSL clear 2 ms on
    uint8_t next_ok;     ///< the probe that followed on the same host: i2c_ok
    uint8_t acked_ok;    ///< the controls - every byte acknowledged - exact
    uint32_t first_cr1;  ///< CR1 and SR2 of the first refusal not let go
    uint32_t first_sr2;
    uint8_t first_st;    ///< its status, and the next tenure's, in two nibbles
};

/// One tenure, waited out on the tick for at most 20 ms.
template <typename Bus>
uint8_t run_ms(const uint8_t* tx, uint8_t tx_len, uint8_t* rx, uint8_t rx_len, I2cSpeed speed) {
    typename Bus::Request r{};
    r.addr = peer_addr;
    r.tx = lend<Lease::reply>(tx);
    r.tx_len = tx_len;
    r.rx = lend<Lease::reply>(rx);
    r.rx_len = rx_len;
    r.speed = speed;
    r.engine = engine_of<Bus>();
    xfer_done = false;
    if (Bus::start(r)) {
        return Bus::status();
    }
    const uint32_t t0 = Ticker::ticks();
    while (!xfer_done && Ticker::ticks() - t0 < 20u) {
    }
    return xfer_done ? Bus::status() : parked;
}

/// One to four bytes written - the scratch register's index, then data -
/// and two read, at one speed through one host: each count once
/// acknowledged whole (the control) and four times with its LAST written
/// byte refused, the NACK arriving with the repeated START requested.
template <typename Bus>
RefusalTally refusal_point(I2cSpeed speed) {
    RefusalTally t{};
    const uint8_t tx[4] = {peer.scratch_reg, 0x11, 0x22, 0x33};
    uint8_t rx[2] = {};
    for (uint8_t n = 1; n <= 4u; ++n) {
        Regs::refuse_at = 0;
        rx[0] = 0xEE;
        rx[1] = 0xEE;
        const uint8_t ack_st = run_ms<Bus>(tx, n, rx, 2, speed);
        const uint8_t at = static_cast<uint8_t>(peer.scratch_reg + n - 1u);
        if (ack_st == i2c_ok && Regs::took == n && rx[0] == Regs::file[at] &&
            rx[1] == Regs::file[at + 1u]) {
            ++t.acked_ok;
        }
        for (uint8_t trial = 0; trial < 4u; ++trial) {
            Regs::refuse_at = n;
            const uint8_t st = run_ms<Bus>(tx, n, rx, 2, speed);
            const uint32_t t0 = Ticker::ticks();
            while (Ticker::ticks() - t0 < 2u) {
            }
            const uint32_t cr1 = S::regs().CR1;
            const uint32_t sr2 = S::regs().SR2;
            Regs::refuse_at = 0;
            const uint8_t next = run_ms<Bus>(nullptr, 0, nullptr, 0, speed);
            const bool gone = (cr1 & (I2C_CR1_START | I2C_CR1_STOP)) == 0u &&
                              (sr2 & I2C_SR2_MSL) == 0u;
            ++t.refused;
            t.nack_data = static_cast<uint8_t>(t.nack_data + (st == i2c_nack_data ? 1u : 0u));
            t.missed = static_cast<uint8_t>(t.missed + (st == i2c_ok ? 1u : 0u));
            t.let_go = static_cast<uint8_t>(t.let_go + (gone ? 1u : 0u));
            t.next_ok = static_cast<uint8_t>(t.next_ok + (next == i2c_ok ? 1u : 0u));
            if ((!gone || next != i2c_ok) && t.first_cr1 == 0u) {
                t.first_cr1 = cr1;
                t.first_sr2 = sr2;
                t.first_st = static_cast<uint8_t>(st << 4 | (next & 0x0Fu));
            }
            if (!gone || next != i2c_ok) {
                (void)Bus::recover();
                (void)Bus::unstick();
            }
        }
    }
    return t;
}

/// One host at one speed, its counts printed; true when every refusal was
/// answered i2c_nack_data (or, the target late, taken whole) with the bus
/// let go and the next tenure running, and every control was exact.
template <typename Bus>
bool refusal_report(const char* core, I2cSpeed speed, bool engines) {
    const RefusalTally t = refusal_point<Bus>(speed);
    print(serial, "  ", core, speed == I2cSpeed::fast_400k ? " 400 kHz " : " 100 kHz ",
          engines ? "engines" : "pump   ", ": acked exact ", t.acked_ok, "/4; refused ",
          t.refused, ": nack_data ", t.nack_data, " (target late ", t.missed, "), let go ",
          t.let_go, ", next ok ", t.next_ok);
    if (t.first_cr1 != 0u) {
        print(serial, "; first stuck: status|next ", hex(t.first_st), " CR1 ", hex(t.first_cr1),
              " SR2 ", hex(t.first_sr2));
    }
    print(serial, crlf);
    return t.acked_ok == 4u && t.nack_data != 0u && t.nack_data + t.missed == t.refused &&
           t.let_go == t.refused && t.next_ok == t.refused;
}

/// Every core of the letter: the switch rebases the console, the tick and
/// the two hosts; the target is brought up again at the new rate.
template <uint8_t i>
bool refusal_core(const char* core) {
    const bool up = Cores::template set_index<i>();
    (void)Regs::up(cores);
    print(serial, "  the core at ", core, " (", up ? "up" : "FAILED", "), APB1 ",
          Cores::pclk1_hz() / 1'000'000u, " MHz", crlf);
    bool clean = up;
    bus_ao_live = false;
    dma_host_live = false;
    (void)Host::init(cores);
    clean = refusal_report<Host>(core, I2cSpeed::standard_100k, false) && clean;
    clean = refusal_report<Host>(core, I2cSpeed::fast_400k, false) && clean;
    Dma<1>::init();
    (void)DmaHost::init(cores);
    dma_host_live = true;
    clean = refusal_report<DmaHost>(core, I2cSpeed::standard_100k, true) && clean;
    clean = refusal_report<DmaHost>(core, I2cSpeed::fast_400k, true) && clean;
    dma_host_live = false;
    DmaHost::release();
    return clean;
}

void tm_refusal() {
    if (peer_addr == 0u) {
        bench.verdict("the target answered at boot", false);
        return;
    }
    const bool fast = refusal_core<0>("180 MHz");
    const bool mid = refusal_core<1>(" 48 MHz");
    const bool slow = refusal_core<2>(" 16 MHz");
    (void)Cores::template set_index<0>();
    (void)Regs::up(clock);
    (void)host_ready();
    bench.verdict("the LAST WRITTEN byte of a write-then-read refused - one to four written, "
                  "both speeds, pump and engines, the core at 180 MHz: i2c_nack_data, the "
                  "controller lets the bus go, and the next tenure runs",
                  fast);
    bench.verdict("... and the same with the core at 48 MHz", mid);
    bench.verdict("... and at 16 MHz, the HSI alone", slow);
    bench.verdict("the target is back at the boot rate and answers its identity",
                  read_reg(reg_chip_id) == peer.id_bytes[0]);
}
#endif

#if defined(STM32F469xx)
// =============================================================================
// l - a finger on the glass (by name only)
// =============================================================================

void tl_finger() {
    if (!host_ready() || peer_addr == 0u) {
        bench.verdict("the host came up and a device answered", false);
        return;
    }
    // The versions, for the record.
    uint8_t lib[2] = {0, 0};
    (void)read_regs(reg_lib_ver_h, lib, 2);
    print(serial, "  LIB_VER ", hex(lib[0]), " ", hex(lib[1]), ", CIPHER ", hex(read_reg(reg_cipher)), ", FIRMID ",
          hex(read_reg(peer.ver_reg)), ", RELEASE_CODE_ID ", hex(read_reg(reg_release_code)), ", CTRL ",
          hex(read_reg(reg_ctrl)), ", PERIODACTIVE ", hex(read_reg(reg_period_active)), ", G_MODE ",
          hex(read_reg(reg_g_mode)), crlf);

    // Interrupt trigger mode: a pulse on INT per report while a finger is
    // down (the note's 1.2), counted on EXTI line 5.
    const uint8_t g_before = read_reg(reg_g_mode);
    (void)write_reg(reg_g_mode, 0x01);
    const uint8_t g_trigger = read_reg(reg_g_mode);
    bench.verdict("G_MODE takes the interrupt trigger mode and reads it back", g_trigger == 0x01u);
    (void)TouchIrq::claim(PinPull::up);
    (void)TouchIrq::configure(ExtiSense::falling);
    (void)TouchIrq::clear();
    touch_int_edges = 0;
    (void)TouchIrq::arm(true);
    Nvic::enable(TouchIrq::irq());
    const bool int_idle_high = TouchInt::read();

    // Up to twenty-five seconds for a finger to arrive, then eight
    // seconds of it, the touch data polled every 10 ms.
    print(serial, "  TOUCH THE GLASS - waiting up to 25 s for a finger, then eight seconds of it", crlf);
    uint32_t samples = 0, touched = 0, twos = 0, errors = 0;
    uint16_t x_min = 0xFFFF, x_max = 0, y_min = 0xFFFF, y_max = 0;
    uint8_t flags_seen = 0, ids_seen = 0;
    bool first_printed = false;
    uint32_t start = Ticker::ticks();
    uint32_t window = 25000u;
    bool arrived = false;
    while (Ticker::ticks() - start < window) {
        if (!arrived && touched != 0u) {
            arrived = true;
            start = Ticker::ticks();
            window = 8000u;
            print(serial, "  a finger: eight seconds from now", crlf);
        }
        uint8_t td[5] = {0, 0, 0, 0, 0};
        if (read_regs(reg_td_status, td, 5) != i2c_ok) {
            ++errors;
        } else {
            ++samples;
            const uint8_t points = td[0] & 0x0Fu;
            if (points == 1u || points == 2u) {
                ++touched;
                if (points == 2u) {
                    ++twos;
                }
                const uint8_t flag = static_cast<uint8_t>(td[1] >> 6);
                const uint16_t x = static_cast<uint16_t>(((td[1] & 0x0Fu) << 8) | td[2]);
                const uint16_t y = static_cast<uint16_t>(((td[3] & 0x0Fu) << 8) | td[4]);
                const uint8_t id = static_cast<uint8_t>(td[3] >> 4);
                flags_seen = static_cast<uint8_t>(flags_seen | (1u << flag));
                ids_seen = static_cast<uint8_t>(ids_seen | (id < 8u ? (1u << id) : 0x80u));
                x_min = x < x_min ? x : x_min;
                x_max = x > x_max ? x : x_max;
                y_min = y < y_min ? y : y_min;
                y_max = y > y_max ? y : y_max;
                if (!first_printed) {
                    print(serial, "  first report: TD_STATUS ", hex(td[0]), ", event flag ", flag, ", id ", id, ", x ", x,
                          ", y ", y, crlf);
                    first_printed = true;
                }
            }
        }
        const uint32_t t = Ticker::ticks();
        while (Ticker::ticks() - t < 10u) {
        }
    }
    Nvic::disable(TouchIrq::irq());
    (void)TouchIrq::arm(false);
    const uint32_t edges = touch_int_edges;
    (void)write_reg(reg_g_mode, g_before);
    print(serial, "  ", samples, " samples, ", touched, " with a finger (", twos, " with two), ", errors, " bus errors; x ",
          x_min, "..", x_max, ", y ", y_min, "..", y_max, "; event flags seen ", hex(flags_seen), " (bit 0 press, 1 lift, 2 contact, 3 none), ids ",
          hex(ids_seen), "; INT idle ", int_idle_high ? "high" : "LOW", ", ", edges, " falling edges", crlf);
    bench.verdict("a finger was reported: TD_STATUS counted one or two points", touched != 0u);
    bench.verdict("the coordinates stayed inside a 800x800 frame (12 bits each, the panel 800 by 480)",
                  touched != 0u && x_max < 800u && y_max < 800u);
    bench.verdict("the contact event (10b) was among the flags seen", (flags_seen & 0x04u) != 0u);
    bench.verdict("INT idles high with no finger and pulsed on the reports in trigger mode",
                  int_idle_high && edges != 0u);
    bench.verdict("no bus error over the polling", errors == 0u);
    bench.verdict("G_MODE put back to what it was", read_reg(reg_g_mode) == g_before);
}
#endif

#if defined(STM32F446xx)
// =============================================================================
// n - THE DESK PARTNER: the STM32G0's twi_peer on the same wire (set `t`)
// =============================================================================
//
// The partner's I2C1 (PB8/PB9 on its side) taps the same two nodes as this
// board's I2C1 and FMPI2C1. Its command client answers 0x6B alone, which
// no letter of `z` sends but letter d's scan, so the partner is dark for
// `z` and runs `twi_peer` for this set. What the peer can do that the
// self-link's register file cannot: stretch the clock on command, hold SDA
// low from its own GPIO until a number of SCL edges, and go deaf.

/// Where the partner's client is told to answer: NOT twi_link's dut_addr
/// (0x2C), which is this board's own FMPI2C1 target in every letter here -
/// two targets on one address would collide on every read.
constexpr uint8_t partner_addr = twilink::shared_addr;

struct LinkPort {
    static uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len, uint8_t* rx,
                          uint8_t rx_len) {
        return ::tenure<Host>(addr, tx, tx_len, rx, rx_len);
    }
    static void ready() { (void)host_ready(); }
    static void wait_ms(uint32_t ms) { spin_ms(ms); }
};
using Link = TwiLinkClient<LinkPort>;
using twilink::Op;

/// The opening line of every letter of the set: the peer answers, or the
/// letter fails by name.
bool need_twi_peer() {
    (void)host_ready();
    if (Link::command(Op::ping)) {
        return true;
    }
    bench.verdict("the desk partner answers on the command address (the STM32G0 running "
                  "twi_peer)",
                  false);
    return false;
}

/// One 8-byte write to the peer's serving address, timed on the tick.
uint32_t timed_write(uint8_t& st) {
    static const uint8_t w8[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    const uint32_t t0 = Ticker::ticks();
    st = tenure<Host>(partner_addr, w8, 8, nullptr, 0);
    return Ticker::ticks() - t0;
}

void tn_partner() {
    if (!need_twi_peer()) {
        return;
    }
    twilink::Ident id{};
    const bool got = Link::ident(id);
    char label[9] = {};
    for (uint8_t i = 0; i < 8; ++i) {
        label[i] = id.label[i];
    }
    print(serial, "  peer: label '", label, "' sanity ", hex(id.sanity), " fw ", hex(id.version),
          crlf);
    bench.verdict("the partner is twi_peer (the sanity byte), an STM32G0 (fw 0x03xx)",
                  got && id.sanity == twilink::ident_sanity &&
                      twilink::peer_kind(id.version) == twilink::peer_stm32g0);
    uint8_t good = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        if (Link::command(Op::ping)) {
            ++good;
        }
    }
    bench.verdict("ten command round trips, each two tenures of this engine", good == 10u);

    // ---- clock stretching as flow control ----
    twilink::Params base{};
    base.count = 128;   // never reached: the deadline ends the serve
    base.ms = 300;
    base.addr = partner_addr;
    if (!Link::act(Op::serve, base)) {
        bench.verdict("the peer accepted the unstretched serve", false);
        return;
    }
    uint8_t st0 = 0;
    const uint32_t base_ms = timed_write(st0);
    spin_ms(350);
    twilink::Params st{};
    st.count = 64;
    st.ms = 150;
    st.addr = partner_addr;
    st.hold_us = 2000;
    if (!Link::act(Op::serve, st)) {
        bench.verdict("the peer accepted the stretched serve", false);
        return;
    }
    uint8_t st1 = 0;
    const uint32_t held_ms = timed_write(st1);
    spin_ms(200);
    twilink::Report r{};
    const bool rep = Link::report(r);
    print(serial, "  an 8-byte write: ", base_ms, " ms unstretched (status ", st0, "), ", held_ms,
          " ms with the peer holding SCL 2 ms before each byte (status ", st1, "); the peer took ",
          r.aux0, crlf);
    bench.verdict("unstretched, the 8-byte write completes i2c_ok inside a tick or two "
                  "(81 bit periods at 100 kHz: 0.8 ms)",
                  st0 == i2c_ok && base_ms <= 2u);
    // THE BAND: the peer spends its hold at the address match and before
    // every data byte, eight holds inside the controller's window (the
    // ninth falls after its last byte) - 16 ms - plus the 0.8 ms of bits
    // and the tick's own +-1: 16..19 ms.
    bench.verdict("a target stretching SCL 2 ms a byte stretches this controller's tenure to "
                  "the 16 ms the holds add, and it completes i2c_ok: the engine simply waits",
                  st1 == i2c_ok && held_ms >= 16u && held_ms <= 19u && rep && r.aux0 == 8u);

    // ---- unstick() against a client that really holds SDA ----
    twilink::Params h{};
    h.ms = 300;
    h.aux8 = 4;   // let go at the fourth falling edge of SCL
    if (!Link::act(Op::hold_sda, h)) {
        bench.verdict("the peer accepted the SDA hold", false);
        return;
    }
    spin_ms(5);
    const bool held = !SdaPad::read();
    const uint8_t pulses = Host::unstick();
    spin_ms(350);
    twilink::Report hr{};
    const bool hrep = Link::report(hr);
    print(serial, "  SDA held by the peer: ", held ? "low" : "NOT low", "; unstick() answered ",
          pulses, " pulse(s); the peer counted ", hr.aux0, " falling edge(s) and saw SDA ",
          hr.aux1 != 0u ? "high" : "low", " when it let go", crlf);
    bench.verdict("a client holding SDA until four SCL edges is freed by unstick() at the "
                  "fourth pulse, and unstick() says so",
                  held && pulses == 4u && hrep && hr.aux0 == 4u);

    twilink::Params h0{};
    h0.ms = 100;
    h0.aux8 = 0;   // only the deadline lets go
    if (!Link::act(Op::hold_sda, h0)) {
        bench.verdict("the peer accepted the SDA hold to its deadline", false);
        return;
    }
    spin_ms(5);
    const uint8_t stuck = Host::unstick();
    spin_ms(150);
    const uint8_t after = Host::unstick();
    print(serial, "  SDA held to the peer's deadline: unstick() ", stuck,
          "; once the hold is over: ", after, crlf);
    bench.verdict("nine pulses and a STOP cannot free a pad nothing releases: 0xFF",
                  stuck == 0xFFu);
    bench.verdict("... and with the wire free again unstick() drives nothing", after == 0u);
    bench.verdict("the command channel is back after the holds", need_twi_peer());

    // ---- the arbiter's per-bus timeout over this engine ----
    // The peer stretches 40 ms at the address: past the arbiter's 20 ms,
    // and a tenure that never completes is what the timeout is for.
    twilink::Params w{};
    w.count = 64;
    w.ms = 150;
    w.addr = partner_addr;
    w.hold_us = 40'000;
    if (!Link::act(Op::serve, w)) {
        bench.verdict("the peer accepted the wedging serve", false);
        return;
    }
    kl::BusKernel::init_all();
    bus_ao_live = true;
    kl::Probe::clear_tally();
    static uint8_t two[2] = {0x5A, 0xA5};
    Host::Request rq{};
    rq.addr = partner_addr;
    rq.tx = lend<Lease::reply>(static_cast<const uint8_t*>(two));
    rq.tx_len = 2;
    rq.reply = reply_to<kl::Probe, I2cDone>();
    const uint32_t t0 = Ticker::ticks();
    post<kl::I2cArb>(rq);
    uint32_t replied_at = 0;
    while (Ticker::ticks() - t0 < 200u) {
        kl::pump();
        if (kl::Probe::n != 0u && replied_at == 0u) {
            replied_at = Ticker::ticks() - t0;
        }
    }
    bus_ao_live = false;
    spin_ms(150);   // the peer's serve over
    const uint8_t freed = Host::unstick();
    print(serial, "  a tenure stretched 40 ms at its address: reply ", kl::Probe::replies[0],
          " after ", replied_at, " ms (", i2c_timeout, " is i2c_timeout); unstick() afterwards ",
          freed, crlf);
    // THE BAND: the timeout is a time event of 20 ticks armed at the start
    // and matured by the pump, which polls every tick: 20..21 ticks after
    // the post, and the stopwatch's own tick either side - 19..22.
    bench.verdict("the arbiter answers the wedged tenure with i2c_timeout, in its place",
                  kl::Probe::n == 1u && kl::Probe::replies[0] == i2c_timeout);
    bench.verdict("... at its 20 ms limit (19..22 ms on the tick)",
                  replied_at >= 19u && replied_at <= 22u);
    bench.verdict("and the same host carries the command channel again after recover()",
                  need_twi_peer());

    // ---- rebase() across the core's rate ----
    // The host is a user of letter m's DynamicClock, so every switch runs
    // its rebase(): FREQ, CCR and TRISE re-derived for the new APB1. A
    // wrong derivation still moves bytes - at a wrong rate - so each rung
    // TIMES its write against the rate asked.
    uint8_t rungs_ok = 0;
    for (uint8_t i = 0; i < 3u; ++i) {
        const bool up = i == 0u   ? Cores::template set_index<0>()
                        : i == 1u ? Cores::template set_index<1>()
                                  : Cores::template set_index<2>();
        twilink::Params sv{};
        sv.count = 64;
        sv.ms = 250;
        sv.addr = partner_addr;
        sv.seed = static_cast<uint8_t>(0x30u + i);
        const bool cmd = up && Link::act(Op::serve, sv);
        static const uint8_t w8[8] = {1, 2, 3, 4, 5, 6, 7, 8};
        uint8_t rb[8] = {};
        const uint32_t c0 = Ticker::cycles();
        const uint8_t ws = tenure<Host>(partner_addr, w8, 8, nullptr, 0);
        const uint32_t cyc = Ticker::cycles() - c0;
        const uint8_t rs = tenure<Host>(partner_addr, nullptr, 0, rb, 8);
        uint8_t bad = 0;
        for (uint8_t k = 0; k < 8u; ++k) {
            if (rb[k] != twilink::pattern_value(twilink::pattern_counting, sv.seed, k)) {
                ++bad;
            }
        }
        spin_ms(300);
        const uint32_t mhz = Cores::hz() / 1'000'000u;
        // 81 SCL periods in the write: the address and eight bytes.
        const uint32_t per_ns = mhz != 0u ? static_cast<uint32_t>(
                                                static_cast<uint64_t>(cyc) * 1000u / mhz / 81u)
                                          : 0u;
        const uint32_t pm = per_ns != 0u ? 10'000'000u / per_ns : 0u;   // thousandths of 100 kHz
        print(serial, "  core ", mhz, " MHz, APB1 ", Host::reference_hz() / 1000u,
              " kHz: serve ", cmd, ", write ", ws, ", read ", rs, ", ", bad,
              " wrong, the write's SCL at ", pm, " thousandths of 100 kHz", crlf);
        // Never above the rate by more than 2 %, and within a quarter below
        // it: a short tenure carries the request's own cost, and the
        // partner's polled target stretches each byte a little.
        if (cmd && ws == i2c_ok && rs == i2c_ok && bad == 0u && pm <= 1020u && pm >= 750u) {
            ++rungs_ok;
        }
    }
    (void)Cores::template set_index<0>();
    (void)Regs::up(clock);
    bench.verdict("at 180, 48 and 16 MHz of core the host's rebase() keeps the bus at the rate "
                  "asked (within 2 % above, a quarter below) and byte-exact against the partner",
                  rungs_ok == 3u);
    bench.verdict("... and the channel is back at the boot rate", need_twi_peer());
}
#endif

#if defined(STM32F446xx)
// =============================================================================
// o - THE RECEIVE STREAM UNDER A REAL LOAD (the Nucleo-F446RE's self-link)
// =============================================================================
//
// What ends a read under dma on this host is the stream's own completion: the event
// vector does nothing in that phase, and for a run of two or more the STOP is requested
// in the completion's entry (RM0390 24.3.8). So the load asks a question other than the
// FMPI2C's: not whether the host answers ahead of its stream, which it cannot, but what
// a late stream does to the wire and to the time of the answer. DMA1 has no
// memory-to-memory (RM0390 9.3.6): its load is peripheral-paced streams at very_high,
// above the host's `high`, a word from a timer's counter on APB1 to one word of SRAM, the
// count circular over 65535 beats so SxNDTR says how many moved. Four copies on DMA2 add
// the memory's other master and the core's turns of keep_busy() a third.

alignas(4) uint32_t paced_cell[6];

/// One stream, on its timer's update (`cc` 0) or compare `cc`, every `period` of the
/// timer's 90 MHz clock, two core cycles a count.
template <uint8_t stream, uint8_t channel, uint8_t tim, uint8_t slot, uint8_t cc>
struct Paced {
    using T = Tim<tim>;
    using St = DmaStream<1, stream>;
    static void on(uint32_t period) {
        if (!T::bus_clock()) {
            T::init();
        }
        (void)T::configure(TimConfig{.prescaler = 0, .period = period - 1u});
        DmaTransfer t{};
        t.peripheral = &T::regs().CNT;
        t.memory = &paced_cell[slot];
        t.count = 65535;
        t.config.channel = channel;
        t.config.direction = DmaDirection::peripheral_to_memory;
        t.config.circular = true;
        t.config.memory_increment = false;
        t.config.peripheral_width = DmaWidth::word;
        t.config.memory_width = DmaWidth::word;
        t.config.priority = DmaPriority::very_high;
        (void)St::load(t);
        if constexpr (cc == 0u) {
            T::interrupts(T::update_dma, true);
        } else {
            (&T::regs().CCR1)[cc - 1u] = period / 2u;
            T::regs().DIER = T::regs().DIER | (TIM_DIER_CC1DE << (cc - 1u));
        }
        T::enable(true);
    }
    static void stop() { St::stop(); }
    /// After every stream on this timer has stopped: a gate closed over a raised request
    /// holds it (docs/stm32f4/dma.md, "A released requester"), and a stream at very_high
    /// left on that cell takes every arbitration there is.
    static void gate() {
        if (T::bus_clock()) {
            T::release();
        }
    }
    static uint16_t count() { return St::count(); }
};

// RM0390 table 28: every stream of DMA1 but this host's 6 and 0.
using Paced1 = Paced<1, 3, 2, 0, 0>;   // TIM2_UP
using Paced2 = Paced<2, 5, 3, 1, 0>;   // TIM3_UP
using Paced3 = Paced<3, 2, 4, 2, 2>;   // TIM4_CH2
using Paced4 = Paced<4, 1, 7, 3, 0>;   // TIM7_UP
using Paced5 = Paced<5, 3, 2, 4, 1>;   // TIM2_CH1, the second stream on TIM2
using Paced6 = Paced<7, 2, 4, 5, 3>;   // TIM4_CH3, the second stream on TIM4

void paced_on(uint8_t streams, uint32_t period) {
    if (streams > 0u) { Paced1::on(period); }
    if (streams > 1u) { Paced2::on(period); }
    if (streams > 2u) { Paced3::on(period); }
    if (streams > 3u) { Paced4::on(period); }
    if (streams > 4u) { Paced5::on(period); }
    if (streams > 5u) { Paced6::on(period); }
}

void paced_off(uint8_t streams) {
    if (streams > 5u) { Paced6::stop(); }
    if (streams > 4u) { Paced5::stop(); }
    if (streams > 3u) { Paced4::stop(); }
    if (streams > 2u) { Paced3::stop(); }
    if (streams > 1u) { Paced2::stop(); }
    if (streams > 0u) { Paced1::stop(); }
    Paced1::gate();
    Paced2::gate();
    Paced3::gate();
    Paced4::gate();
    Paced5::gate();
    Paced6::gate();
}

/// The beats the load moved in 18000 core cycles, 100 us at 180 MHz, all its streams
/// together. APB1 at 45 MHz takes an access in two of its clocks, 2250 in that span.
uint32_t paced_beats(uint8_t streams) {
    uint16_t a[6] = {Paced1::count(), Paced2::count(), Paced3::count(),
                     Paced4::count(), Paced5::count(), Paced6::count()};
    const uint32_t t0 = CycleCounter::now();
    while (CycleCounter::now() - t0 < 18'000u) {
    }
    const uint16_t b[6] = {Paced1::count(), Paced2::count(), Paced3::count(),
                           Paced4::count(), Paced5::count(), Paced6::count()};
    uint32_t sum = 0;
    for (uint8_t i = 0; i < streams; ++i) {
        sum += static_cast<uint16_t>(a[i] - b[i]);
    }
    return sum;
}

alignas(16) uint32_t copy_src[4][256];
alignas(16) uint32_t copy_dst[4][256];
using CopyA = DmaCopyEngine<2, 0>;
using CopyB = DmaCopyEngine<2, 4>;
using CopyC = DmaCopyEngine<2, 5>;
using CopyD = DmaCopyEngine<2, 7>;

template <typename E>
void keep_busy(uint8_t i) {
    if (!E::busy()) {
        (void)E::copy(copy_dst[i], copy_src[i], 256u);
    }
}

void copies_on() {
    Dma<2>::init();
    CopyA::arm(DmaPriority::very_high);
    CopyB::arm(DmaPriority::very_high);
    CopyC::arm(DmaPriority::very_high);
    CopyD::arm(DmaPriority::very_high);
}

void copies_turn() {
    keep_busy<CopyA>(0);
    keep_busy<CopyB>(1);
    keep_busy<CopyC>(2);
    keep_busy<CopyD>(3);
}

void copies_off() {
    (void)CopyA::abandon();
    (void)CopyB::abandon();
    (void)CopyC::abandon();
    (void)CopyD::abandon();
}

/// Letter o: the entries of the event vector and when the stream's completion found
/// the wire, read by the suite's own vectors around the host's bodies.
struct ReadWatch {
    volatile bool on = false;
    volatile uint32_t ev_entries = 0;     ///< the event vector, whatever it found
    volatile uint32_t btf_entries = 0;    ///< BTF standing under the read's stream
    volatile uint32_t tc_after_wire = 0;  ///< a read of one's wire already over at the completion
    volatile uint32_t ev_cycles = 0;
};
ReadWatch read_watch;

struct ReadRow {
    uint32_t reads = 0;
    uint32_t exact = 0;
    uint32_t late = 0;      ///< BTF up at an entry under the stream: a byte waited for it
    uint32_t after = 0;     ///< the wire over ahead of the completion: a read of one's
    uint32_t entries = 0;   ///< the event vector's entries, all told
    uint32_t cycles = 0;    ///< and their cycles
    uint32_t slow = 0;
    uint32_t lost = 0;
    uint32_t moved = 0;
};

void read_row_print(const char* name, const ReadRow* by) {
    print(serial, "    ", name, " reads of 1, 2, 8: ", by[0].reads, " each; exact ", by[0].exact, ", ",
          by[1].exact, ", ", by[2].exact, "; BTF under the stream ", by[0].late, ", ", by[1].late, ", ",
          by[2].late, "; wire over at the completion ", by[0].after, ", ", by[1].after, ", ", by[2].after,
          "; event entries ", by[0].entries, ", ", by[1].entries, ", ", by[2].entries, " (",
          by[2].cycles, " cycles at 8); stretched ", by[0].slow + by[1].slow + by[2].slow,
          "; unanswered ", by[0].lost + by[1].lost + by[2].lost, "; moved after ",
          by[0].moved + by[1].moved + by[2].moved, crlf);
}

/// One read under `dma`, the target's pointer set from here (the target is this chip's).
/// The wait's first bound is twice the wire and three ticks, its last fifty times that: a
/// stream starved past it is answered here as an arbiter's timeout would be, recover().
template <typename Spin>
void watched_read(uint8_t n, I2cSpeed speed, Spin spin, ReadRow& row) {
    static uint8_t in[32];
    const uint8_t from = 0x10;
    Regs::ptr = from;
    for (uint8_t i = 0; i < n; ++i) {
        in[i] = static_cast<uint8_t>(~Regs::file[from + i]);
    }
    DmaHost::Request r{};
    r.addr = peer.addr;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(in));
    r.rx_len = n;
    r.speed = speed;
    r.engine = I2cEngine::dma;
    read_watch.ev_entries = 0;
    read_watch.btf_entries = 0;
    read_watch.tc_after_wire = 0;
    read_watch.ev_cycles = 0;
    read_watch.on = true;
    xfer_done = false;
    (void)DmaHost::start(r);
    const uint32_t bound = 2u * 9u * (n + 2u) * 1000u / i2c_speed_hz(speed) + 3u;
    const uint32_t t0 = Ticker::ticks();
    bool slow = false;
    while (!xfer_done && Ticker::ticks() - t0 < 50u * bound) {
        spin();
        slow = slow || Ticker::ticks() - t0 >= bound;
    }
    read_watch.on = false;
    bool exact = xfer_done && DmaHost::status() == i2c_ok;
    for (uint8_t i = 0; i < n && exact; ++i) {
        exact = in[i] == Regs::file[from + i];
    }
    if (!xfer_done) {
        (void)DmaHost::recover();
        row.lost = row.lost + 1u;
    }
    row.reads = row.reads + 1u;
    row.exact = row.exact + (exact ? 1u : 0u);
    row.late = row.late + (read_watch.btf_entries != 0u ? 1u : 0u);
    row.after = row.after + read_watch.tc_after_wire;
    row.entries = row.entries + read_watch.ev_entries;
    row.cycles = row.cycles + read_watch.ev_cycles;
    row.slow = row.slow + (slow ? 1u : 0u);
    const uint16_t c0 = RxStreamRegs::count();
    for (uint32_t i = 100u; i != 0u; --i) {
        __NOP();
    }
    row.moved = row.moved + ((RxStreamRegs::enabled() || RxStreamRegs::count() != c0) ? 1u : 0u);
}

template <typename Spin>
void read_row(const char* name, I2cSpeed speed, uint16_t reads, Spin spin, ReadRow& total) {
    ReadRow by[3] = {};
    const uint8_t lengths[3] = {1, 2, 8};
    for (uint16_t k = 0; k < reads; ++k) {
        for (uint8_t i = 0; i < 3u; ++i) {
            watched_read(lengths[i], speed, spin, by[i]);
        }
        if (by[0].lost + by[1].lost + by[2].lost >= 3u) {
            break;
        }
    }
    read_row_print(name, by);
    for (uint8_t i = 0; i < 3u; ++i) {
        total.reads += by[i].reads;
        total.exact += by[i].exact;
        total.late += by[i].late;
        total.after += by[i].after;
        total.entries += by[i].entries;
        total.slow += by[i].slow;
        total.lost += by[i].lost;
        total.moved += by[i].moved;
    }
}

struct LoadCell {
    uint8_t streams;
    uint32_t period;
    bool copies;
};

void to_load() {
    (void)CycleCounter::init();
    Dma<1>::init();
    bus_ao_live = false;
    const bool up = DmaHost::init(clock);
    dma_host_live = true;
    auto idle = [] {};
    ReadRow total{};
    for (const LoadCell& c : {LoadCell{0, 16, false}, LoadCell{6, 32, false}, LoadCell{4, 17, false},
                              LoadCell{3, 13, false}, LoadCell{0, 16, true}, LoadCell{2, 16, true},
                              LoadCell{2, 8, true}, LoadCell{6, 48, true}, LoadCell{3, 16, true}}) {
        if (c.copies) {
            copies_on();
        }
        paced_on(c.streams, c.period);
        print(serial, "  ", c.streams, " stream(s) at very_high every ", c.period, " of 90 MHz",
              c.copies ? " and four copies" : "", ": ", paced_beats(c.streams), " beats in 100 us", crlf);
        if (c.copies) {
            read_row("400 kHz", I2cSpeed::fast_400k, 150, copies_turn, total);
            read_row("100 kHz", I2cSpeed::standard_100k, 50, copies_turn, total);
        } else {
            read_row("400 kHz", I2cSpeed::fast_400k, 150, idle, total);
            read_row("100 kHz", I2cSpeed::standard_100k, 50, idle, total);
        }
        paced_off(c.streams);
        if (c.copies) {
            copies_off();
        }
    }
    print(serial, "  below all of APB1: ", total.reads, " reads, ", total.exact, " exact, the wire over "
          "ahead of a read of one's completion ", total.after, ", BTF up at an event entry ", total.late,
          ", stretched past the bound ", total.slow, ", unanswered ", total.lost, ", moved after the reply ",
          total.moved, crlf);
    bench.verdict("UNDER EVERY LOAD BELOW ALL OF APB1: every read that was answered is exact and i2c_ok, "
                  "the stream still after the reply",
                  up && total.reads != 0u && total.exact + total.lost == total.reads && total.moved == 0u);
    bench.verdict("... and a load makes a read of one's stream finish behind its wire", total.after != 0u);

    dma_host_live = false;
    DmaHost::release();
    (void)host_ready();
}

// =============================================================================
// r - A LOAD THAT TAKES ALL OF APB1 (by name only)
// =============================================================================
//
// 2250 beats in 100 us, the bus's own figure. Not in `z`: such a load can leave the core's
// own access to the bus waiting until a reset, and a suite that stands still judges
// nothing after it. A run that never reaches its verdicts is that, and is counted.

/// A run of `n` on the engines under all of APB1, the stream not served from the start or,
/// `late`, from the third byte on: what stands behind the wire's first bytes, what the
/// event vector did, and the run when the load comes off. A write's bytes are counted at
/// the target and a read's compared; the target comes up afresh for each, its pointer
/// the witness of how many bytes the wire asked of it.
struct StarvedRow {
    uint16_t rows = 0;
    uint16_t early = 0;      ///< answered with the load up
    uint16_t stretched = 0;  ///< BTF standing and the host still the controller
    uint32_t entries = 0;    ///< the event vector's, with the load up
    uint16_t right = 0;      ///< answered i2c_ok, every byte, once the load came off
    uint16_t given_max = 0;  ///< a read: the most bytes the target's pointer moved
    uint16_t given_min = 0xFFFFu;
    uint16_t refused = 0;    ///< answered i2c_nack_data, the target having refused the last byte
    uint16_t took_ok = 0;    ///< of those and the rest, answered i2c_ok
};

void starved_run(uint8_t n, bool write, uint16_t late_us, StarvedRow& row, bool load = true,
                 uint8_t refuse_at = 0, uint8_t read_too = 0) {
    const bool late = late_us != 0u;
    static uint8_t cells[9];
    const uint8_t from = 0x10;
    (void)Regs::up(clock);
    DmaHost::Request r{};
    r.addr = peer.addr;
    r.speed = late ? I2cSpeed::standard_100k : I2cSpeed::fast_400k;
    r.engine = I2cEngine::dma;
    if (write) {
        for (uint8_t i = 0; i < n; ++i) {
            cells[i] = from;   // the index, then bytes the read-only cells ignore
        }
        r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(cells));
        r.tx_len = n;
        if (read_too != 0u) {
            static uint8_t behind[8];
            r.rx = lend<Lease::reply>(static_cast<uint8_t*>(behind));
            r.rx_len = read_too;
        }
    } else {
        Regs::ptr = from;
        for (uint8_t i = 0; i < n; ++i) {
            cells[i] = static_cast<uint8_t>(~Regs::file[from + i]);
        }
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(cells));
        r.rx_len = n;
    }
    Regs::took = 0;
    Regs::refuse_at = refuse_at;
    if (load && !late) {
        paced_on(3, 5);
    }
    read_watch.ev_entries = 0;
    read_watch.on = true;
    xfer_done = false;
    (void)DmaHost::start(r);
    if (load && late) {
        // At 100 kHz the address and k bytes are 90 x (k + 1) us, and the load comes up.
        (void)delay_us(clock, late_us);
        paced_on(3, 5);
    }
    const uint32_t t0 = Ticker::ticks();
    // A run of eight's own clocks are 225 us at 400 kHz and 900 at 100 kHz.
    while (Ticker::ticks() - t0 < 3u) {
    }
    row.entries = row.entries + read_watch.ev_entries;
    row.early = static_cast<uint16_t>(row.early + ((load && xfer_done) ? 1u : 0u));
    // Not on a refused row: a status read here is half of a clear the stream's store would
    // then finish, and BTF left standing by that store is what those rows ask about.
    if (refuse_at == 0u) {
        row.stretched = static_cast<uint16_t>(row.stretched + ((S::host() && S::flag(I2cFlag::byte_finished)) ? 1u : 0u));
    }
    if (load) {
        paced_off(3);
    }
    while (!xfer_done && Ticker::ticks() - t0 < 20u) {
    }
    read_watch.on = false;
    spin_ms(1);
    Regs::refuse_at = 0;
    row.refused = static_cast<uint16_t>(row.refused + ((xfer_done && DmaHost::status() == i2c_nack_data) ? 1u : 0u));
    row.took_ok = static_cast<uint16_t>(row.took_ok + ((xfer_done && DmaHost::status() == i2c_ok) ? 1u : 0u));
    bool right = xfer_done && DmaHost::status() == i2c_ok;
    if (write) {
        right = right && Regs::took == n;
    } else {
        for (uint8_t i = 0; i < n && right; ++i) {
            right = cells[i] == Regs::file[from + i];
        }
        const uint16_t given = static_cast<uint16_t>(Regs::ptr - from);
        row.given_max = given > row.given_max ? given : row.given_max;
        row.given_min = given < row.given_min ? given : row.given_min;
    }
    if (!xfer_done) {
        (void)DmaHost::recover();
    }
    row.rows = static_cast<uint16_t>(row.rows + 1u);
    row.right = static_cast<uint16_t>(row.right + (right ? 1u : 0u));
}

void starved_print(const char* what, uint8_t n, const StarvedRow& row) {
    print(serial, "  ", what, " of ", n, ", ", row.rows, " rows: answered with the load up ", row.early,
          ", BTF standing ", row.stretched, ", event entries a row ", row.entries / (row.rows != 0u ? row.rows : 1u),
          "; exact once the load came off ", row.right);
    if (row.given_max != 0u) {
        print(serial, "; the target's pointer moved ", row.given_min, " to ", row.given_max);
    }
    print(serial, crlf);
}

void tr_all_of_apb1() {
    (void)CycleCounter::init();
    Dma<1>::init();
    bus_ao_live = false;
    const bool up = DmaHost::init(clock);
    dma_host_live = true;
    // The target afresh: a read behind a read leaves the byte it loaded for the next one.
    (void)Regs::up(clock);
    ReadRow total{};
    copies_on();
    paced_on(3, 12);
    print(serial, "  3 stream(s) at very_high every 12 of 90 MHz and four copies: ", paced_beats(3),
          " beats in 100 us", crlf);
    read_row("400 kHz", I2cSpeed::fast_400k, 150, copies_turn, total);
    read_row("100 kHz", I2cSpeed::standard_100k, 50, copies_turn, total);
    paced_off(3);
    copies_off();
    bench.verdict("THREE STREAMS AND THE COPIES AT THE BUS'S FIGURE: every read answered exact and "
                  "i2c_ok, the stream still after the reply",
                  up && total.reads != 0u && total.exact + total.lost == total.reads && total.moved == 0u);

    // A read of one has its STOP programmed at ADDR's clear, so the wire ends with the
    // byte in DR and the stream not served; the answer waits for the load to come off.
    uint16_t rows = 0;
    uint16_t wire_first = 0;
    uint16_t answered_early = 0;
    uint16_t right = 0;
    uint16_t next_right = 0;
    for (uint16_t k = 0; k < 100u; ++k) {
        static uint8_t one;
        static uint8_t next[8];
        Regs::ptr = 0x10;
        one = static_cast<uint8_t>(~Regs::file[0x10]);
        paced_on(3, 5);
        DmaHost::Request r{};
        r.addr = peer.addr;
        r.rx = lend<Lease::reply>(static_cast<uint8_t*>(&one));
        r.rx_len = 1;
        r.speed = I2cSpeed::fast_400k;
        r.engine = I2cEngine::dma;
        xfer_done = false;
        (void)DmaHost::start(r);
        const uint32_t t0 = Ticker::ticks();
        // The tenure's nineteen clocks, 48 us at 400 kHz: the wire's end is two ticks away.
        while (Ticker::ticks() - t0 < 2u) {
        }
        wire_first = static_cast<uint16_t>(wire_first + (!S::host() && S::flag(I2cFlag::rx_not_empty) ? 1u : 0u));
        answered_early = static_cast<uint16_t>(answered_early + (xfer_done ? 1u : 0u));
        paced_off(3);
        while (!xfer_done && Ticker::ticks() - t0 < 10u) {
        }
        rows = static_cast<uint16_t>(rows + 1u);
        right = static_cast<uint16_t>(right + (xfer_done && DmaHost::status() == i2c_ok &&
                                                       one == Regs::file[0x10] ? 1u : 0u));
        Regs::ptr = 0x20;
        bool same_next = tenure<DmaHost>(peer.addr, nullptr, 0, next, 8, I2cSpeed::fast_400k, I2cEngine::dma) == i2c_ok;
        for (uint8_t i = 0; i < 8u && same_next; ++i) {
            same_next = next[i] == Regs::file[0x20 + i];
        }
        next_right = static_cast<uint16_t>(next_right + (same_next ? 1u : 0u));
    }
    print(serial, "  all of APB1, a read of one, ", rows, " rows: the wire over with the byte in DR in ",
          wire_first, ", an answer ahead of the stream in ", answered_early, "; exact once the load came off ",
          right, ", the read of eight behind it ", next_right, crlf);
    // 100 rows a run. wire_first says the load reached the case.
    bench.verdict("A STREAM STARVED BY ALL OF APB1: the wire ends, nothing answers ahead of the stream, the "
                  "read and the next one exact when the load comes off",
                  rows == 100u && wire_first == 100u && answered_early == 0u && right == 100u &&
                      next_right == 100u);
    StarvedRow plain_rd2{}, plain_rd8{}, rd2{}, rd8{}, wr2{}, wr8{}, late_rd8{}, late_wr8{}, last_rd8{};
    StarvedRow plain_no8{}, late_no8{}, start_no8{}, plain_no_rd{}, late_no_rd{}, last_no8{}, last_no_rd{}, last_wr8{};
    // Three from the end: the address and six bytes are 630 us, the seventh in the shifter from
    // 540 and the eighth wanted in DR then. Load up at 600: the eighth's store waits, the seventh
    // ends, BTF rises and SCL holds until the load comes off, the store going in over BTF.
    constexpr uint16_t last_store_us = 600;
    // The address and two bytes are 270 us at 100 kHz; the address and six, 630, and the
    // seventh ends at 720: 680 catches the seventh in the shifter with the sixth served.
    constexpr uint16_t third_us = 300;
    constexpr uint16_t seventh_us = 680;
    for (uint8_t k = 0; k < 50u; ++k) {
        starved_run(2, false, 0, plain_rd2, false);
        starved_run(8, false, 0, plain_rd8, false);
        starved_run(2, false, 0, rd2);
        starved_run(8, false, 0, rd8);
        starved_run(2, true, 0, wr2);
        starved_run(8, true, 0, wr8);
        starved_run(8, false, third_us, late_rd8);
        starved_run(8, true, third_us, late_wr8);
        starved_run(8, false, seventh_us, last_rd8);
        // The last written byte refused, on a stream a load held: BTF standing behind the
        // late bytes must not end the write ahead of that byte's acknowledge.
        starved_run(8, true, 0, plain_no8, false, 8);
        starved_run(8, true, third_us, late_no8, true, 8);
        starved_run(8, true, 0, start_no8, true, 8);
        starved_run(8, true, 0, plain_no_rd, false, 8, 2);
        starved_run(8, true, third_us, late_no_rd, true, 8, 2);
        starved_run(8, true, last_store_us, last_wr8);
        starved_run(8, true, last_store_us, last_no8, true, 8);
        starved_run(8, true, last_store_us, last_no_rd, true, 8, 2);
    }
    for (const auto& [what, row] : {std::pair<const char*, const StarvedRow&>{"no load", plain_no8},
                                    {"all of APB1 from the third byte", late_no8},
                                    {"all of APB1 from the start", start_no8},
                                    {"no load, a read of two behind it", plain_no_rd},
                                    {"all of APB1 from the third byte, a read of two behind it", late_no_rd},
                                    {"the last store held", last_no8},
                                    {"the last store held, a read of two behind it", last_no_rd}}) {
        print(serial, "  the last of eight written refused, ", what, ", ", row.rows, " rows: i2c_nack_data ", row.refused,
              ", i2c_ok ", row.took_ok, crlf);
    }
    starved_print("the last store held, a write", 8, last_wr8);
    bench.verdict("... the last store's stage reached the case: BTF standing with the eighth byte's store held, every byte there when it went in",
                  last_wr8.stretched == last_wr8.rows && last_wr8.right == last_wr8.rows);
    bench.verdict("A REFUSED LAST BYTE ON A STREAM IS REPORTED, late or on time, a read behind it or none: i2c_nack_data every row "
                  "and never i2c_ok",
                  plain_no8.refused == plain_no8.rows && late_no8.refused == late_no8.rows &&
                      start_no8.refused == start_no8.rows && plain_no_rd.refused == plain_no_rd.rows &&
                      late_no_rd.refused == late_no_rd.rows && last_no8.refused == last_no8.rows &&
                      last_no_rd.refused == last_no_rd.rows);
    starved_print("no load, a read", 2, plain_rd2);
    starved_print("no load, a read", 8, plain_rd8);
    starved_print("all of APB1, a read", 2, rd2);
    starved_print("all of APB1, a read", 8, rd8);
    starved_print("all of APB1, a write", 2, wr2);
    starved_print("all of APB1, a write", 8, wr8);
    starved_print("all of APB1 from the third byte, a read", 8, late_rd8);
    starved_print("all of APB1 from the third byte, a write", 8, late_wr8);
    starved_print("all of APB1 from the seventh byte, a read", 8, last_rd8);
    bench.verdict("A RUN OF TWO OR EIGHT STARVED: nothing answers ahead of the stream, the event vector takes SB's and ADDR's entries "
                  "alone, and every byte is there when the load comes off",
                  rd2.right == rd2.rows && rd8.right == rd8.rows && wr2.right == wr2.rows && wr8.right == wr8.rows &&
                      late_rd8.right == late_rd8.rows && late_wr8.right == late_wr8.rows && last_rd8.right == last_rd8.rows &&
                      rd8.early == 0u && wr2.early == 0u && wr8.early == 0u && late_rd8.early == 0u && late_wr8.early == 0u &&
                      last_rd8.early == 0u);
    // The witness is the target's own pointer, so the bound is an identity: a read of n with
    // no load moves it n + 1 (the byte the target loads behind the NACK, stated by the two
    // no-load rows), and any byte more is a byte clocked off the target.
    bench.verdict("A READ OF TWO UNDER ALL OF APB1 CLOCKS NO THIRD BYTE: no stream to wait for, answered with the load up, "
                  "the target's pointer where the same read with no load leaves it, every row",
                  rd2.early == rd2.rows && plain_rd2.given_min == 3u && plain_rd2.given_max == 3u &&
                      rd2.given_min == 3u && rd2.given_max == 3u);
    bench.verdict("A READ OF EIGHT STARVED ANYWHERE BUT ACROSS ITS LAST TWO BYTES clocks no byte more: the pointer at nine, as with no load",
                  plain_rd8.given_min == 9u && plain_rd8.given_max == 9u && rd8.given_min == 9u && rd8.given_max == 9u &&
                      late_rd8.given_min == 9u && late_rd8.given_max == 9u);
    bench.verdict("A READ OF EIGHT STARVED ACROSS ITS LAST TWO BYTES CLOCKS ONE MORE, the reply exact: the pointer at ten "
                  "(the page's limit, stated, not a wish)",
                  last_rd8.given_max == 10u);
    dma_host_live = false;
    DmaHost::release();
    (void)host_ready();
}

// =============================================================================
// p - THE STREAM'S COMPLETION HELD OFF
// =============================================================================
//
// A read of three or more under dma: the STOP is requested in the stream's completion
// entry (RM0390 24.3.8), and the chapter does not say what the wire does between the
// last byte in memory and that entry. The stage: the stream's vector line off, a poll
// that dates the count's zero, a hold of `hold` core cycles counting SCL's rising edges,
// the line back on, and the wire followed for forty bits at 100 kHz. Then what stands
// in the peripheral, and the next read. No clock of a further byte is nine rising
// edges; the one allowed is the STOP's own.

struct HoldRow {
    uint32_t rows = 0;
    uint32_t exact = 0;
    uint32_t extra_edges = 0;   ///< rising edges of SCL after the stream's last byte, most
    uint32_t rxne = 0;          ///< RxNE up once the wire is over
    uint32_t next_exact = 0;    ///< the read of eight behind it
};

void held_read(uint8_t n, I2cSpeed speed, uint32_t hold, HoldRow& row) {
    static uint8_t in[32];
    Regs::ptr = 0x10;
    for (uint8_t i = 0; i < n; ++i) {
        in[i] = static_cast<uint8_t>(~Regs::file[0x10 + i]);
    }
    DmaHost::Request r{};
    r.addr = peer.addr;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(in));
    r.rx_len = n;
    r.speed = speed;
    r.engine = I2cEngine::dma;
    xfer_done = false;
    NVIC_DisableIRQ(RxStreamRegs::irq());
    (void)DmaHost::start(r);
    const uint32_t t0 = Ticker::ticks();
    while (RxStreamRegs::count() == 0u && Ticker::ticks() - t0 < 50u) {
    }
    while (RxStreamRegs::count() != 0u && Ticker::ticks() - t0 < 50u) {
    }
    uint32_t edges = 0;
    bool last = SclPad::read();
    const uint32_t c0 = CycleCounter::now();
    while (CycleCounter::now() - c0 < hold) {
        const bool now = SclPad::read();
        edges += (now && !last) ? 1u : 0u;
        last = now;
    }
    NVIC_EnableIRQ(RxStreamRegs::irq());
    const uint32_t c1 = CycleCounter::now();
    while (CycleCounter::now() - c1 < 40u * 1800u) {   // forty bits at 100 kHz, to the wire's end
        const bool now = SclPad::read();
        edges += (now && !last) ? 1u : 0u;
        last = now;
    }
    bool exact = xfer_done && DmaHost::status() == i2c_ok;
    for (uint8_t i = 0; i < n && exact; ++i) {
        exact = in[i] == Regs::file[0x10 + i];
    }
    const bool rxne = S::flag(I2cFlag::rx_not_empty);
    row.rows = row.rows + 1u;
    row.exact = row.exact + (exact ? 1u : 0u);
    row.extra_edges = edges > row.extra_edges ? edges : row.extra_edges;
    row.rxne = row.rxne + (rxne ? 1u : 0u);
    static uint8_t next[8];
    Regs::ptr = 0x20;
    for (uint8_t i = 0; i < 8u; ++i) {
        next[i] = static_cast<uint8_t>(~Regs::file[0x20 + i]);
    }
    bool same_next = tenure<DmaHost>(peer.addr, nullptr, 0, next, 8, speed, I2cEngine::dma) == i2c_ok;
    for (uint8_t i = 0; i < 8u && same_next; ++i) {
        same_next = next[i] == Regs::file[0x20 + i];
    }
    row.next_exact = row.next_exact + (same_next ? 1u : 0u);
}

/// THE STRAY BEAT'S STAGE ON THIS BLOCK, by hand: a read of one, DMAEN taken down behind
/// the ADDR entry so RxNE stands with no request, then DMAEN up and EN's clear in the next
/// store, so the clear lands ahead of the stream's beat. DR is then read with DMAEN up
/// (`enable_first` false) or down. True when the next block's count moved by itself.
bool stray_row(bool enable_first, bool& left) {
    static uint8_t in;
    static uint8_t scratch[2];
    DmaHost::Request r{};
    r.addr = peer.addr;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(&in));
    r.rx_len = 1;
    r.speed = I2cSpeed::fast_400k;
    r.engine = I2cEngine::dma;
    xfer_done = false;
    Nvic::disable(RxStreamRegs::irq());
    (void)DmaHost::start(r);
    const uint32_t t0 = Ticker::ticks();
    while ((S::regs().CR2 & I2C_CR2_DMAEN) == 0u && Ticker::ticks() - t0 < 5u) {
    }
    S::dma(false, false);
    while (!S::flag(I2cFlag::rx_not_empty) && Ticker::ticks() - t0 < 5u) {
    }
    {
        InterruptGuard guard;
        const uint32_t request = S::regs().CR2 | I2C_CR2_DMAEN;
        const uint32_t cleared = RxStreamRegs::regs().CR & ~DMA_SxCR_EN;
        S::regs().CR2 = request;
        RxStreamRegs::regs().CR = cleared;
        for (uint32_t i = 200'000u; i != 0u && RxStreamRegs::enabled(); --i) {   // abort()'s bound
        }
        left = S::flag(I2cFlag::rx_not_empty);
        if (enable_first) {
            S::dma(false, false);
        }
        (void)S::data();
    }
    S::dma(false, false);
    RxEngine::stop();
    (void)RxEngine::start(std::span<uint8_t>(scratch, 2u));
    for (uint32_t i = 100u; i != 0u; --i) {
        __NOP();
    }
    const bool moved = RxStreamRegs::count() != 2u;
    RxEngine::stop();
    RxEngine::service();
    Nvic::enable(RxStreamRegs::irq());
    spin_ms(1);
    (void)DmaHost::recover();
    return moved;
}

void tp_held() {
    (void)CycleCounter::init();
    Dma<1>::init();
    bus_ao_live = false;
    const bool up = DmaHost::init(clock);
    dma_host_live = true;
    HoldRow all{};
    for (const I2cSpeed v : {I2cSpeed::fast_400k, I2cSpeed::standard_100k}) {
        const uint32_t bit = v == I2cSpeed::fast_400k ? 456u : 1800u;
        for (const uint8_t n : {uint8_t{3}, uint8_t{8}}) {
            // In quarters of a bit, up to forty bits: past the next byte's nine clocks
            // and past a STOP's bit, so any byte the peripheral would clock is seen.
            for (const uint32_t quarters : {uint32_t{0}, uint32_t{2}, uint32_t{4}, uint32_t{8},
                                            uint32_t{36}, uint32_t{160}}) {
                HoldRow row{};
                for (uint8_t k = 0; k < 10u; ++k) {
                    held_read(n, v, quarters * bit / 4u, row);
                }
                print(serial, "  ", v == I2cSpeed::fast_400k ? "400" : "100", " kHz, read of ", n,
                      ", held ", quarters, "/4 of a bit: ", row.exact, " of ", row.rows,
                      " exact, SCL's rising edges behind the last byte at most ", row.extra_edges,
                      ", RxNE up ", row.rxne, ", the next read of 8 exact ", row.next_exact, crlf);
                all.rows += row.rows;
                all.exact += row.exact;
                all.rxne += row.rxne;
                all.next_exact += row.next_exact;
                all.extra_edges = row.extra_edges > all.extra_edges ? row.extra_edges : all.extra_edges;
            }
        }
    }
    bench.verdict("A COMPLETION HELD OFF stretches the wire and clocks no byte: the one rising edge "
                  "behind the last byte is the STOP's, RxNE down at the end, the read and the next "
                  "one exact",
                  up && all.rows == 240u && all.exact == all.rows && all.extra_edges <= 1u &&
                      all.rxne == 0u && all.next_exact == all.rows);

    // ---- the silicon's stray beat, and that no path of the host's reads DR over a stream ----
    uint16_t left_up = 0;
    uint16_t strays_up = 0;
    uint16_t left_down = 0;
    uint16_t strays_down = 0;
    for (uint8_t k = 0; k < 100u; ++k) {
        bool left = false;
        const bool moved = stray_row(false, left);
        left_up = static_cast<uint16_t>(left_up + (left ? 1u : 0u));
        strays_up = static_cast<uint16_t>(strays_up + (left && moved ? 1u : 0u));
        const bool moved_down = stray_row(true, left);
        left_down = static_cast<uint16_t>(left_down + (left ? 1u : 0u));
        strays_down = static_cast<uint16_t>(strays_down + (left && moved_down ? 1u : 0u));
    }
    print(serial, "  a clear ahead of the request, staged by hand: DR read with DMAEN up, the byte left in ",
          left_up, " of 100, a stray beat at the next start in ", strays_up,
          "; DMAEN down first, left in ", left_down, ", a stray in ", strays_down, crlf);
    bench.verdict("the stray beat's stage: DMAEN down ahead of DR's read, none at the next start",
                  strays_down == 0u);
    const uint8_t after = tenure<DmaHost>(peer.addr, nullptr, 0, nullptr, 0, I2cSpeed::fast_400k,
                                          I2cEngine::dma);
    bench.verdict("... and the host carries the next tenure", after == i2c_ok);
    dma_host_live = false;
    DmaHost::release();
    (void)host_ready();
}

// =============================================================================
// q - start()'s TWO BOUNDS, in microseconds
// =============================================================================
//
// start() waits for the last tenure's STOP to leave the wire and then for BUSY. Each
// wait is staged with the far pads of the self-link, PC6 and PC7, driven low from here.
// The wall of start() is read on the counter the case leaves running.

using FarScl = Pin<'C', 6>;
using FarSda = Pin<'C', 7>;

/// Letter s's client lets go: the far pad back to the target, in the state Target::init()
/// left it, or the target's event line unmasked. From the letter or from the tick's handler.
void drain_client_lets_go() {
    if (drain_let_go == DrainLetGo::pad) {
        FarScl::function(target_pins.scl.function, {.open_drain = true, .speed = PinSpeed::very_high});
    } else if (drain_let_go == DrainLetGo::target) {
        Nvic::enable(Target::Resource::event_irq);
    }
}

enum class BoundCase : uint8_t { ticker, no_ticker, no_counter };

constexpr const char* bound_name(BoundCase c) {
    return c == BoundCase::ticker      ? "the ticker running"
           : c == BoundCase::no_ticker ? "SysTick stopped"
                                       : "the DWT's counter stopped";
}

/// One start(): the case's counter stopped around it, its wall in core cycles.
uint32_t timed_start(BoundCase c, const Host::Request& r, bool& answer) {
    if (c == BoundCase::no_ticker) {
        SysTick->CTRL = SysTick->CTRL & ~SysTick_CTRL_ENABLE_Msk;
    }
    if (c == BoundCase::no_counter) {
        DWT->CTRL = DWT->CTRL & ~DWT_CTRL_CYCCNTENA_Msk;
        const uint32_t t0 = Ticker::cycles();
        answer = Host::start(r);
        const uint32_t wall = Ticker::cycles() - t0;
        DWT->CTRL = DWT->CTRL | DWT_CTRL_CYCCNTENA_Msk;
        return wall;
    }
    const uint32_t t0 = CycleCounter::now();
    answer = Host::start(r);
    const uint32_t wall = CycleCounter::now() - t0;
    if (c == BoundCase::no_ticker) {
        SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_ENABLE_Msk;
    }
    return wall;
}

/// The far pads back to the target, and the bus as a letter expects it.
void far_pads_back() {
    (void)Regs::up(clock);
    (void)host_ready();
}

struct BoundRow {
    uint32_t wall_min = 0xFFFF'FFFFu;
    uint32_t wall_max = 0;
    uint8_t staged = 0;   ///< the row reached the wait it is about
    uint8_t rows = 0;
};

void bound_note(BoundRow& row, uint32_t wall, bool staged) {
    row.rows = static_cast<uint8_t>(row.rows + 1u);
    row.staged = static_cast<uint8_t>(row.staged + (staged ? 1u : 0u));
    row.wall_min = wall < row.wall_min ? wall : row.wall_min;
    row.wall_max = wall > row.wall_max ? wall : row.wall_max;
}

/// THE STOP'S DRAIN: a probe's STOP is requested at its completion, SCL is pulled low
/// from the far pad behind it, so STOP stands; start() waits and answers i2c_timeout,
/// with no START and the event vector still down.
void stop_bound(BoundCase c, BoundRow& row) {
    (void)host_ready();
    const uint8_t st = tenure<Host>(peer.addr, nullptr, 0, nullptr, 0);
    FarScl::output(false, {.open_drain = true, .speed = PinSpeed::very_high});
    const bool standing = st == i2c_ok && S::stopping();
    Host::Request r{};
    r.addr = peer.addr;
    bool answer = true;
    const uint32_t wall = timed_start(c, r, answer);
    const bool answered_here = answer && Host::status() == i2c_timeout && !S::starting() && S::stopping() &&
                               (S::regs().CR2 & I2C_CR2_ITEVTEN) == 0u;
    FarScl::release();
    far_pads_back();
    bound_note(row, wall, standing && answered_here);
}

/// THE BUS'S: SDA pulled low from the far pad with nothing standing in CR1, BUSY up;
/// start() waits, then sets START, which waits behind BUSY until the far pad lets go.
void busy_bound(BoundCase c, BoundRow& row) {
    (void)host_ready();
    FarSda::output(false, {.open_drain = true, .speed = PinSpeed::very_high});
    (void)delay_us(clock, 20);
    const bool busy = S::busy() && !S::stopping();
    Host::Request r{};
    r.addr = peer.addr;
    xfer_done = false;
    bool answer = true;
    const uint32_t wall = timed_start(c, r, answer);
    const bool behind = !answer && S::starting();
    FarSda::release();
    far_pads_back();
    bound_note(row, wall, busy && behind);
}

void tq_bounds() {
    (void)CycleCounter::init();
    const uint32_t cycles_per_us = clock_hz(clock) / 1'000'000u;
    while (!Serial::tx_idle()) {
    }
    for (const BoundCase c : {BoundCase::ticker, BoundCase::no_ticker, BoundCase::no_counter}) {
        BoundRow stop_row{};
        BoundRow busy_row{};
        for (uint8_t k = 0; k < 10u; ++k) {
            stop_bound(c, stop_row);
            busy_bound(c, busy_row);
        }
        print(serial, "  ", bound_name(c), ": the 5000 us drain ", stop_row.wall_min / cycles_per_us, " to ",
              stop_row.wall_max / cycles_per_us, " us, reached ", stop_row.staged, " of ", stop_row.rows,
              "; the 100 us bus wait ", busy_row.wall_min / cycles_per_us, " to ",
              busy_row.wall_max / cycles_per_us, " us, reached ", busy_row.staged, " of ", busy_row.rows,
              crlf);
        // Above the time asked: start()'s own 181 cycles at most on a quiet bus, a poll's turn
        // of 30, a tick's handler of about 100, six at most in 5 ms: under 1000 cycles. The
        // console has drained (Serial::tx_idle() below), so no other vector lands.
        constexpr uint32_t over = 1000u;
        const bool timed = c != BoundCase::no_counter;
        const uint32_t drain_low = 5000u * cycles_per_us;
        const uint32_t busy_low = 100u * cycles_per_us;
        // The counter stopped, the budget alone: the time asked over poll_min_cycles (8) in
        // polls, each at most 40 cycles (six instructions, a load across the APB1 bridge):
        // five times the time asked, the bound on how long a stopped counter stretches it.
        const uint32_t drain_high = timed ? drain_low + over : 5u * drain_low;
        const uint32_t busy_high = timed ? busy_low + over : 5u * busy_low;
        bench.verdict("each bound served at least, and within the figures its comment derives",
                      stop_row.staged == 10u && busy_row.staged == 10u && stop_row.wall_min >= drain_low &&
                          stop_row.wall_max <= drain_high && busy_row.wall_min >= busy_low &&
                          busy_row.wall_max <= busy_high);
        while (!Serial::tx_idle()) {
        }
    }
    const uint8_t after = tenure<Host>(peer.addr, nullptr, 0, nullptr, 0);
    bench.verdict("the host carries the next tenure", after == i2c_ok);
}

// =============================================================================
// s - A CLIENT THAT HOLDS THE CLOCK PAST THE DRAIN, THROUGH THE ARBITER
// =============================================================================
//
// A request right behind one whose acknowledge a client is stretching, on an arbiter
// with a per-bus timeout and on one with timeout_ticks == 0. What holds SCL after a
// written byte's acknowledge is the far SCL pad, pulled low from the completion's own
// entry and given back to the target at the hold's end: this block's own client does
// that (BTF stands with the shifter full, RM0390 24.3.2). FMPI2C1 at its address does
// not: the row that masks its event line, ADDR holding SCL (23.4.8), shows the tenure's
// own completion waiting for the hold, so no STOP stands and this path is not reached.
// The scratch register is the witness of what a request answered i2c_timeout moved.

namespace drain {

/// A request's reply, with when it was dispatched.
class Probe {
public:
    using Event = std::variant<I2cDone>;
    static inline EventQueue<Event, 12, P> queue;
    static inline uint8_t status[8];
    static inline uint32_t at[8];
    static inline uint8_t n = 0;

    static void init() { clear(); }
    static void clear() { n = 0; }
    static void dispatch(const Event& e) {
        brio::match(e, [](const I2cDone& d) {
            if (n < 8u) {
                status[n] = d.status;
                at[n] = CycleCounter::now();
            }
            n = static_cast<uint8_t>(n + 1u);
        });
    }
};

using TimedArb = kl::I2cArb;
using PlainArb = I2cBus<Host, P, 4>;

uint8_t scratch_in[1];
const uint8_t scratch_index[1] = {reg_sys_ctrl2};
uint8_t first_out[2];
uint8_t next_out[2];
uint8_t behind_out[3][2];

template <typename Arb>
void arm_glue(bool on) {
    if constexpr (std::is_same_v<Arb, TimedArb>) {
        bus_ao_live = on;
    } else {
        drain_plain_live = on;
    }
}

template <typename Kernel>
void pump() {
    TimeEvents<P>::process();
    while (Kernel::step()) {
        TimeEvents<P>::process();
    }
}

/// Pump for `us`, or until `want` replies have come if it is not zero.
template <typename Kernel>
void pump_for(uint32_t us, uint8_t want = 0) {
    const uint32_t t0 = CycleCounter::now();
    const uint32_t span = us * (clock_hz(clock) / 1'000'000u);
    while (CycleCounter::now() - t0 < span) {
        pump<Kernel>();
        if (want != 0u && Probe::n >= want) {
            break;
        }
    }
    pump<Kernel>();
}

Host::Request write_request(uint8_t* out, uint8_t value) {
    out[0] = reg_sys_ctrl2;
    out[1] = value;
    Host::Request r{};
    r.addr = peer.addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(out));
    r.tx_len = 2;
    r.reply = reply_to<Probe, I2cDone>();
    return r;
}

Host::Request read_request() {
    Host::Request r{};
    r.addr = peer.addr;
    r.tx = lend<Lease::reply>(static_cast<const uint8_t*>(scratch_index));
    r.tx_len = 1;
    r.rx = lend<Lease::reply>(static_cast<uint8_t*>(scratch_in));
    r.rx_len = 1;
    r.reply = reply_to<Probe, I2cDone>();
    return r;
}

struct Stage {
    uint32_t hold_ms = 8;
    uint8_t behind = 1;            ///< requests posted right behind the one that gets stretched
    bool let_go_in_tick = false;   ///< the hold ends in SysTick's handler, inside the wait
    bool target_hold = false;      ///< FMPI2C1's own hold at its address, the far pad untouched
};

/// One row's sums over its passes.
struct Row {
    uint8_t passes = 0;
    uint8_t first_ok = 0;         ///< the stretched request answered i2c_ok
    uint8_t behind_timeout = 0;   ///< every request behind it answered i2c_timeout
    uint8_t behind_ok = 0;        ///< every request behind it answered i2c_ok
    uint8_t unanswered = 0;       ///< a request was still owed its answer when the row gave up
    uint8_t stood = 0;            ///< STOP stood at the hold's end
    uint8_t witness_ok = 0;       ///< the scratch register read as it should
    uint8_t next_ok = 0;          ///< a write and its read-back after the hold, exact
    uint32_t answer_min = 0xFFFF'FFFFu;   ///< the last answer, behind the first's completion
    uint32_t answer_max = 0;
    uint32_t first_min = 0xFFFF'FFFFu;    ///< the first's completion, behind its post
    uint32_t first_max = 0;
};

template <typename Arb, typename Kernel>
void one_pass(const Stage& g, Row& row) {
    while (!Serial::tx_idle()) {
    }
    (void)Regs::up(clock);
    (void)host_ready();
    const uint8_t seed_cmd[2] = {reg_sys_ctrl2, 0x5A};
    const bool seeded = tenure<Host>(peer.addr, seed_cmd, 2, nullptr, 0) == i2c_ok;
    spin_ms(1);

    Kernel::init_all();
    arm_glue<Arb>(true);
    Probe::clear();
    drain_done_at = 0;
    Host::Request first{};
    if (g.target_hold) {
        // A probe, ADDR left standing at the target.
        Nvic::disable(Target::Resource::event_irq);
        first.addr = peer.addr;
        first.reply = reply_to<Probe, I2cDone>();
        drain_let_go = DrainLetGo::target;
    } else {
        drain_grab_scl = true;
        first = write_request(first_out, 0x5A);
        drain_let_go = DrainLetGo::pad;
    }
    const uint32_t t0 = CycleCounter::now();
    if (g.let_go_in_tick) {
        drain_let_go_tick = Ticker::ticks() + g.hold_ms + 1u;
    }
    post<Arb>(first);
    for (uint8_t i = 0; i < g.behind; ++i) {
        post<Arb>(write_request(behind_out[i], static_cast<uint8_t>(0xA0u + i)));
    }
    const uint8_t want = static_cast<uint8_t>(1u + g.behind);
    // The hold's length, counted from the post: a two-byte write is 300 us of wire at
    // 100 kHz. A dispatch that waits runs past it.
    const uint32_t hold_cycles = g.hold_ms * (clock_hz(clock) / 1000u);
    while (CycleCounter::now() - t0 < hold_cycles) {
        pump<Kernel>();
    }
    const bool stood = S::stopping();
    drain_grab_scl = false;
    if (!g.let_go_in_tick) {
        drain_client_lets_go();
    }
    // What the hold left: the STOP on its way, then anything still owed. 30 ms a request
    // is past the 20 ms of the timed arbiter and the 5 ms start() spends ahead of arming it.
    pump_for<Kernel>(30'000u * want, want);
    const uint8_t answered = Probe::n;
    bool all_timeout = answered == want;
    bool all_ok = answered == want;
    for (uint8_t i = 1; i < want && i < answered; ++i) {
        all_timeout = all_timeout && Probe::status[i] == i2c_timeout;
        all_ok = all_ok && Probe::status[i] == i2c_ok;
    }
    row.passes = static_cast<uint8_t>(row.passes + 1u);
    row.first_ok = static_cast<uint8_t>(row.first_ok + ((answered != 0u && Probe::status[0] == i2c_ok) ? 1u : 0u));
    row.behind_timeout = static_cast<uint8_t>(row.behind_timeout + (all_timeout ? 1u : 0u));
    row.behind_ok = static_cast<uint8_t>(row.behind_ok + (all_ok ? 1u : 0u));
    row.unanswered = static_cast<uint8_t>(row.unanswered + (answered < want ? 1u : 0u));
    row.stood = static_cast<uint8_t>(row.stood + (stood ? 1u : 0u));
    if (drain_done_at != 0u) {
        const uint32_t first_took = drain_done_at - t0;
        row.first_min = first_took < row.first_min ? first_took : row.first_min;
        row.first_max = first_took > row.first_max ? first_took : row.first_max;
        if (answered == want) {
            const uint32_t took = Probe::at[want - 1u] - drain_done_at;
            row.answer_min = took < row.answer_min ? took : row.answer_min;
            row.answer_max = took > row.answer_max ? took : row.answer_max;
        }
    }

    // The next requests, with no init() and no recover() from here: the witness read, a
    // write and its read-back. A request answered i2c_timeout moved nothing; one answered
    // i2c_ok left its byte.
    Probe::clear();
    scratch_in[0] = 0;
    post<Arb>(read_request());
    pump_for<Kernel>(30'000u, 1);
    const uint8_t witness_want = (g.let_go_in_tick || g.target_hold) ? static_cast<uint8_t>(0xA0u + g.behind - 1u) : 0x5Au;
    row.witness_ok = static_cast<uint8_t>(
        row.witness_ok + ((seeded && Probe::n == 1u && Probe::status[0] == i2c_ok && scratch_in[0] == witness_want) ? 1u : 0u));
    Probe::clear();
    post<Arb>(write_request(next_out, 0x3C));
    post<Arb>(read_request());
    pump_for<Kernel>(30'000u, 2);
    const bool next = Probe::n == 2u && Probe::status[0] == i2c_ok && Probe::status[1] == i2c_ok && scratch_in[0] == 0x3Cu;
    row.next_ok = static_cast<uint8_t>(row.next_ok + (next ? 1u : 0u));
    arm_glue<Arb>(false);
    drain_grab_scl = false;
    drain_let_go_tick = 0;
    drain_let_go = DrainLetGo::none;
    Nvic::enable(Target::Resource::event_irq);
}

template <typename Arb, typename Kernel>
Row row_of(const char* arb, const char* what, const Stage& g) {
    constexpr uint8_t passes = 10;
    Row row{};
    for (uint8_t k = 0; k < passes; ++k) {
        one_pass<Arb, Kernel>(g, row);
    }
    const uint32_t cycles_per_us = clock_hz(clock) / 1'000'000u;
    print(serial, "  ", arb, ", ", what, ", ", g.behind, " behind, ", g.hold_ms, " ms: STOP stood ", row.stood, "/", row.passes,
          ", the first's completion ", row.first_max == 0u ? 0u : row.first_min / cycles_per_us, " to ", row.first_max / cycles_per_us,
          " us, i2c_ok ", row.first_ok, "; behind it i2c_timeout ", row.behind_timeout, ", i2c_ok ", row.behind_ok, ", unanswered ",
          row.unanswered, "; the last answer ", row.answer_max == 0u ? 0u : row.answer_min / cycles_per_us, " to ",
          row.answer_max / cycles_per_us, " us behind that completion (", row.answer_max == 0u ? 0u : row.answer_min, " to ",
          row.answer_max, " cycles); witness ", row.witness_ok, ", next ", row.next_ok, crlf);
    return row;
}

/// The band of the last answer behind the completion it follows, `waits` waits of 5000 us.
/// Below: start() serves the time asked at least (letter q). Above: letter q's 1000 cycles
/// a wait (start() itself, a poll's turn, the ticks that land), then, once, the way from the
/// completion's entry through a turn of the pump, the two dispatches of the arbiter and the
/// replies': 1237 cycles over the time asked for one wait, 2039 for three, in the row's
/// figures, which is under 400 cycles a wait and 1000 of way, and 3000 is three times that.
constexpr bool answer_in_band(const Row& row, uint8_t waits, uint32_t cycles_per_us) {
    const uint32_t low = waits * 5000u * cycles_per_us;
    const uint32_t high = low + waits * 1000u + 3000u;
    return row.answer_min >= low && row.answer_max <= high;
}

template <typename Arb, typename Kernel>
void stages(const char* arb) {
    const uint32_t cycles_per_us = clock_hz(clock) / 1'000'000u;
    const Row held = row_of<Arb, Kernel>(arb, "SCL held behind a written byte's acknowledge", Stage{});
    bench.verdict("A CLIENT HOLDING SCL PAST THE BOUND: the request behind it answered i2c_timeout by start() itself at the "
                  "bound, every pass",
                  held.first_ok == held.passes && held.behind_timeout == held.passes && held.stood == held.passes &&
                      answer_in_band(held, 1, cycles_per_us));
    bench.verdict("... and the host sound when the client lets go, no init() and no recover(): the witness untouched, a write "
                  "and its read-back exact",
                  held.witness_ok == held.passes && held.next_ok == held.passes);

    const Row three = row_of<Arb, Kernel>(arb, "SCL held behind a written byte's acknowledge", Stage{.hold_ms = 30, .behind = 3});
    bench.verdict("THREE REQUESTS BEHIND A LONGER HOLD: each answered i2c_timeout in its turn, in one dispatch of three bounds, "
                  "and the host sound when the client lets go",
                  three.first_ok == three.passes && three.behind_timeout == three.passes && three.stood == three.passes &&
                      answer_in_band(three, 3, cycles_per_us) && three.witness_ok == three.passes && three.next_ok == three.passes);

    const Row brief = row_of<Arb, Kernel>(arb, "SCL held and let go inside the bound", Stage{.hold_ms = 2, .let_go_in_tick = true});
    bench.verdict("THE CONTROL, A HOLD THAT ENDS INSIDE THE BOUND: the request behind it runs, i2c_ok, its byte in the register",
                  brief.first_ok == brief.passes && brief.behind_ok == brief.passes && brief.witness_ok == brief.passes &&
                      brief.next_ok == brief.passes);

    const Row own = row_of<Arb, Kernel>(arb, "FMPI2C1's own hold at its address", Stage{.target_hold = true});
    // 8000 us is when the row lets go; a bit period for the acknowledge's clock, the target's
    // vector and the host's: 7 us measured, 500 for a tick and a turn of the pump over that.
    bench.verdict("A HOLD AHEAD OF THE ACKNOWLEDGE'S END IS NOT THIS PATH: the tenure's own completion waits for it (8 ms from "
                  "the post, 8000 to 8500 us), no STOP stands, the request behind it runs",
                  own.stood == 0u && own.first_ok == own.passes && own.behind_ok == own.passes && own.witness_ok == own.passes &&
                      own.next_ok == own.passes && own.first_min >= 8000u * cycles_per_us && own.first_max <= 8500u * cycles_per_us);
}

}  // namespace drain

void ts_drain() {
    (void)CycleCounter::init();
    drain::stages<drain::TimedArb, Tenuto<P, drain::Probe, drain::TimedArb>>("timed 20 ms");
    drain::stages<drain::PlainArb, Tenuto<P, drain::Probe, drain::PlainArb>>("timeout_ticks 0");
    (void)Regs::up(clock);
    (void)host_ready();
    bench.verdict("the host carries the next tenure", probe(peer.addr) == i2c_ok);
}
#endif

void banner() {
    print(serial, crlf, "test_stm32f4_i2c - I2C", bus_instance, " with ", peer.name, crlf);
    bench.menu();
}

/// The boot scan: who is on this bus, and is it the part the schematic
/// names. Every device letter reads `peer_addr` from here.
void find_peer() {
    if (!host_ready()) {
        return;
    }
    for (uint8_t a = 0x08; a <= 0x77u && peer_addr == 0u; ++a) {
        if (probe(a) == i2c_ok) {
            peer_addr = a;
        }
    }
    if (peer_addr == 0u) {
        return;
    }
    uint8_t id[2] = {};
    (void)read_regs(reg_chip_id, id, 2);
    peer_identified = (peer.addr == 0u || peer_addr == peer.addr) && is_peer_id(id);
    peer_sys_ctrl2_boot = peer_known ? read_reg(reg_sys_ctrl2) : 0u;
}

/// What the board wants before the first tenure: the other Discovery's
/// touch controller sits behind the panel's reset line, left floating by
/// the boot, so it is pulsed once here - the pull-ups then read an idle
/// bus with a device on it.
void board_prepare() {
#if defined(STM32F469xx)
    (void)peer_reset();
#elif defined(STM32F446xx)
    (void)Regs::up(clock);
#endif
}

}  // namespace

// ---- the vectors ---------------------------------------------------------------------------

#if defined(STM32F469xx)
extern "C" void USART3_IRQHandler() { (void)Serial::isr(); }
extern "C" void EXTI9_5_IRQHandler() {
    const uint32_t fired = brio::Exti::isr(TouchIrq::mask);
    if (TouchIrq::served(fired)) {
        touch_int_edges = touch_int_edges + 1u;
    }
}
#define BRIO_I2C_EV_HANDLER I2C1_EV_IRQHandler
#define BRIO_I2C_ER_HANDLER I2C1_ER_IRQHandler
#define BRIO_I2C_TX_DMA_HANDLER DMA1_Stream6_IRQHandler
#define BRIO_I2C_RX_DMA_HANDLER DMA1_Stream0_IRQHandler
#elif defined(STM32F446xx)
extern "C" void USART2_IRQHandler() { (void)Serial::isr(); }
// The peer: FMPI2C1's two vectors serve the register file.
extern "C" void FMPI2C1_EV_IRQHandler() { Regs::event(); }
extern "C" void FMPI2C1_ER_IRQHandler() { Regs::error(); }
#define BRIO_I2C_EV_HANDLER I2C1_EV_IRQHandler
#define BRIO_I2C_ER_HANDLER I2C1_ER_IRQHandler
#define BRIO_I2C_TX_DMA_HANDLER DMA1_Stream6_IRQHandler
#define BRIO_I2C_RX_DMA_HANDLER DMA1_Stream0_IRQHandler
#else
extern "C" void USART1_IRQHandler() { (void)Serial::isr(); }
#define BRIO_I2C_EV_HANDLER I2C3_EV_IRQHandler
#define BRIO_I2C_ER_HANDLER I2C3_ER_IRQHandler
#define BRIO_I2C_TX_DMA_HANDLER DMA1_Stream4_IRQHandler
#define BRIO_I2C_RX_DMA_HANDLER DMA1_Stream2_IRQHandler
#endif

extern "C" void BRIO_I2C_EV_HANDLER() {
    if (dma_host_live) {
#if defined(STM32F446xx)
        if (read_watch.on) {
            const uint32_t c0 = brio::CycleCounter::now();
            const bool btf = S::flag(brio::I2cFlag::byte_finished);
            const bool done = DmaHost::isr();
            xfer_done = xfer_done || done;
            read_watch.ev_entries = read_watch.ev_entries + 1u;
            read_watch.btf_entries = read_watch.btf_entries + (btf ? 1u : 0u);
            read_watch.ev_cycles = read_watch.ev_cycles + (brio::CycleCounter::now() - c0);
            return;
        }
#endif
        if (DmaHost::isr()) {
            xfer_done = true;
        }
        return;
    }
    if (Host::isr()) {
#if defined(STM32F446xx)
        if (drain_grab_scl) {
            FarScl::output(false, {.open_drain = true, .speed = brio::PinSpeed::very_high});
            drain_grab_scl = false;
        }
        if (drain_done_at == 0u) {
            drain_done_at = brio::CycleCounter::now();
        }
        if (drain_plain_live) {
            brio::post<drain::PlainArb>(brio::TransferDone{Host::status()});
        }
#endif
        xfer_done = true;
        if (bus_ao_live) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        }
    }
}

extern "C" void BRIO_I2C_ER_HANDLER() {
    if (dma_host_live) {
        if (DmaHost::error_isr()) {
            xfer_done = true;
        }
        return;
    }
    if (Host::error_isr()) {
#if defined(STM32F446xx)
        if (drain_plain_live) {
            brio::post<drain::PlainArb>(brio::TransferDone{Host::status()});
        }
#endif
        xfer_done = true;
        if (bus_ao_live) {
            brio::post<kl::I2cArb>(brio::TransferDone{Host::status()});
        }
    }
}

extern "C" void BRIO_I2C_TX_DMA_HANDLER() {
    tx_dma_entries = tx_dma_entries + 1u;
    if (dma_host_live && DmaHost::dma_isr()) {
        xfer_done = true;
    }
}
extern "C" void BRIO_I2C_RX_DMA_HANDLER() {
    rx_dma_entries = rx_dma_entries + 1u;
#if defined(STM32F446xx)
    if (read_watch.on && !S::host()) {
        read_watch.tc_after_wire = read_watch.tc_after_wire + 1u;
    }
#endif
    if (dma_host_live && DmaHost::dma_isr()) {
        xfer_done = true;
    }
}

extern "C" void SysTick_Handler() {
    brio::Ticker::tick();
#if defined(STM32F446xx)
    if (drain_let_go_tick != 0u && brio::Ticker::ticks() == drain_let_go_tick) {
        drain_let_go_tick = 0;
        drain_client_lets_go();
    }
#endif
}

int main() {
    // What the silicon held before a line of this program ran: the gate is
    // read out of the RCC (reading it does not open it), and the registers
    // behind it need the clock, so it is opened for the reading.
    boot.gate = S::bus_clock();
    S::bus_clock(true);
    boot.cr1 = S::regs().CR1;
    boot.cr2 = S::regs().CR2;
    boot.oar1 = S::regs().OAR1;
    boot.oar2 = S::regs().OAR2;
    boot.sr1 = S::regs().SR1;
    boot.sr2 = S::regs().SR2;
    boot.ccr = S::regs().CCR;
    boot.trise = S::regs().TRISE;

    const bool clock_ok = SysClock::init();
    const bool serial_ok = Serial::init(clock, 115200);
    const bool tick_ok = brio::Ticker::init(clock);
    brio::enable_interrupts();

    // The two lines as plain inputs, before the peripheral is given them:
    // what the board leaves on the wire at reset.
    SclPad::input();
    SdaPad::input();
    boot.scl = SclPad::read();
    boot.sda = SdaPad::read();

    board_prepare();
    const bool host_ok = host_ready();
    boot.sr2_ready = S::regs().SR2;
    find_peer();

    bench.letter('a', "the instances and the block", ta_block);
    bench.letter('b', "the timing arithmetic", tb_timing);
    bench.letter('c', "what the driver refuses", tc_refusals);
    bench.letter('d', "the address scan", td_scan);
    bench.letter('e', "the device's identity", te_identity);
    bench.letter('f', "the three receive choreographies", tf_receive_shapes);
    bench.letter('g', "the speeds, with SCL counted on the pad", tg_speeds);
    bench.letter('h', "a register written and read back", th_write_back);
    bench.letter('i', "THE KERNEL: I2cBus over I2cHost", ti_kernel);
    bench.letter('j', "the same tenures through the DMA engines", tj_engines);
    bench.letter('k', "the recovery verbs", tk_recovery);
#if defined(STM32F469xx)
    bench.letter('l', "a finger on the glass: the touch controller reporting", tl_finger, false);
#endif
#if defined(STM32F446xx)
    bench.letter('m', "THE REFUSED LAST BYTE, the core at 180, 48 and 16 MHz", tm_refusal);
    bench.letter('o', "THE RECEIVE STREAM UNDER A REAL LOAD", to_load);
    bench.letter('p', "THE STREAM'S COMPLETION HELD OFF", tp_held);
    bench.letter('q', "start()'s TWO BOUNDS, in microseconds", tq_bounds);
    bench.letter('r', "A LOAD THAT TAKES ALL OF APB1, by name only", tr_all_of_apb1, false);
    bench.letter('s', "A CLIENT HOLDS THE CLOCK PAST THE DRAIN, a request behind it, both arbiters", ts_drain);
    bench.second_letter('n', "THE DESK PARTNER: stretching, a held SDA, the per-bus timeout",
                        tn_partner);
#endif

    if (serial_ok) {
        brio::print(serial, brio::crlf, "boot: clk=", clock_ok ? "PLL" : "FAILED",
                    " tick=", tick_ok ? "SysTick" : "FAILED", " i2c", bus_instance, "=",
                    host_ok ? "host" : "FAILED", " peer=", brio::hex(peer_addr),
                    peer_identified ? " (identified)" : " (unknown)", brio::crlf);
        banner();
        bench.prompt();
    }

    for (;;) {
        uint8_t c = 0;
        if (!Serial::read_byte(c)) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            continue;
        }
        brio::print(serial, static_cast<char>(c), brio::crlf);
        if (c == '?') {
            banner();
        } else if (!bench.handle(static_cast<char>(c))) {
            brio::print(serial, "unknown letter (? for the menu)", brio::crlf);
        }
        bench.prompt();
    }
}
