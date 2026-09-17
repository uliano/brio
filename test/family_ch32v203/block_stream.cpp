// The two BLOCK engines of ch32v203/dma.hpp against the two concepts of
// util/block_stream.hpp, and the relay over one of them.
//
// WHICH CHANNEL EACH ENGINE SITS ON is the request table's answer and
// not a number written here: the ping-pong source takes the ADC's
// channel (the regular group's request, table 11-5's first row) and the
// loop player a timer's update, so the TU says what a program would say
// and a part that could not raise the request would fail on the line
// that asked.
//
// WHAT THE TWO SHAPES ARE. The player rides the controller's CIRCULAR
// mode - CNTR reloads itself, the lap interrupt only counts - and the
// source does NOT, because "skip rather than tear" cannot be decided
// after the edge on a channel that never stops (design/block-stream.md,
// and the same decision on the SAM C21 and the STM32G0). So the source
// stops at every block and the handler re-arms the other buffer.
#include "ch32v203/adc.hpp"
#include "ch32v203/dma.hpp"
#include "ch32v203/platform.hpp"
#include "kernel/tenuto.hpp"
#include "util/block_stream.hpp"

using namespace brio;

using P = Ch32v203Platform<>;

using Source = DmaPingPongEngine<DmaRequestOf<DmaRequest::adc1>::channel, uint16_t>;
using Player = DmaLoopEngine<DmaRequestOf<DmaRequest::tim1_up>::channel, uint16_t>;

// ---- the concepts, which is what this file is for --------------------------
static_assert(BlockSource<Source>);
static_assert(BlockPlayer<Player>);
static_assert(Source::channel == 1u);
static_assert(Player::channel == 5u);
static_assert(Source::width == DmaWidth::half && Player::width == DmaWidth::half);
static_assert(Source::present && Player::present);

// A block of bytes and a block of words are the same engine with
// another beat: the element type IS the beat (design/block-stream.md).
static_assert(BlockSource<DmaPingPongEngine<2, uint8_t>>);
static_assert(BlockSource<DmaPingPongEngine<3, uint32_t>>);
static_assert(BlockPlayer<DmaLoopEngine<4, uint8_t>>);

/// One subscriber for the relay's loans.
struct Listener : Fsm<Listener, BlockReady<uint16_t>> {
    static inline EventQueue<Event, 4, P> queue;
    static inline uint32_t seen = 0;

    static void init() { start(&only); }
    static void dispatch(const Event& e) { Fsm::dispatch(e); }

    static Status only(const Event& e) {
        return match(e,
            [](Entry) { return handled(); },
            [](Exit) { return handled(); },
            [](BlockReady<uint16_t> b) { seen += b.length; return handled(); });
    }
};

using Relay = BlockRelay<P, Subscribers<Listener>, Source>;
static_assert(ActiveObject<Relay>);
static_assert(ActiveObject<Listener>);
/// Borrowers before lenders: the kernel refuses the other order.
using System = Tenuto<P, Listener, Relay>;

// ---- the buffers are the caller's -------------------------------------------
alignas(4) volatile uint16_t half_a[32];
alignas(4) volatile uint16_t half_b[32];
constexpr uint16_t table[8] = {0, 1, 2, 3, 4, 5, 6, 7};
uint32_t peripheral_register;

void source_verbs() {
    Source::arm(&peripheral_register);
    Source::arm(&peripheral_register, DmaPriority::very_high);
    (void)Source::start(half_a, half_b, 32);
    (void)Source::start(nullptr, half_b, 32);     // refused
    (void)Source::start(half_a, half_a, 32);      // refused: one buffer twice
    (void)Source::start(half_a, half_b, 0);       // refused
    (void)Source::service();
    (void)Source::complete();
    Source::fail();
    (void)Source::ready();
    (void)Source::ready_length();
    (void)Source::release();
    (void)Source::laps();
    (void)Source::overruns();
    (void)Source::stalled();
    (void)Source::running();
    (void)Source::length();
    (void)Source::pending();
    (void)Source::progress().remaining;
    (void)Source::abandon();
    (void)Source::faults();
    Source::clear_faults();
    Source::stop();
}

/// The converter's own verb for handing its regular group to an engine:
/// the engine armed on the data register and CTLR2.DMA set, with the
/// channel checked against the request table at compile time.
void adc_claims_the_stream() {
    Adc<1>::claim_stream<Source>();
    (void)Source::start(half_a, half_b, 32);
}

void player_verbs() {
    Player::arm(&peripheral_register);
    (void)Player::start(table, 8);
    (void)Player::start(nullptr, 8);   // refused
    (void)Player::start(table, 0);     // refused
    (void)Player::service();
    Player::lap();
    Player::fail();
    (void)Player::laps();
    (void)Player::faults();
    Player::clear_faults();
    (void)Player::running();
    (void)Player::length();
    (void)Player::progress().done;
    (void)Player::kick();
    Player::stop();
}

/// The relay, and the glue an application binds: a completion posts
/// BlockDone and the relay lends the block on for one dispatch.
void relay_verbs() {
    Listener::init();
    Relay::init();
    (void)Source::complete();
    post<Relay>(BlockDone{});
    Relay::dispatch(Relay::Event{BlockDone{}});
    Listener::dispatch(Listener::Event{BlockReady<uint16_t>{
        0, Borrowed<const volatile uint16_t, Lease::dispatch>{half_a}, 32}});
}
