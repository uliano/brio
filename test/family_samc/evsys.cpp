// Family smoke TU for samc/evsys.hpp: every verb must COMPILE on the E,
// G and J 18A headers (tools/check_samc.sh sweeps all three).
//
// EVSYS is one instance with the same twelve channels on every member of
// the family. What this fixture pins is the LEGALITY the driver adds -
// the path/edge relationship that runs both ways, and the erratum-driven
// refusal - plus the two numeric conventions a reader would otherwise
// have to trust: the per-channel GCLK ids and the user register's
// off-by-one.

#include <stdint.h>

#include "samc/clock.hpp"
#include "samc/evsys.hpp"

using namespace brio;

static_assert(Evsys::channel_count == 12);
static_assert(Evsys::user_count == 47);

// The channel GCLK ids are contiguous from the header's own first one.
static_assert(Evsys::gclk_id(0) == EVSYS_GCLK_ID_0);
static_assert(Evsys::gclk_id(11) == EVSYS_GCLK_ID_0 + 11);

static_assert(Evsys::valid_channel(11) && !Evsys::valid_channel(12));
static_assert(Evsys::valid_user(46) && !Evsys::valid_user(47));

// ---- the path decides whether an edge is legal, BOTH ways -------------------
//
// 29.6.2.6 on the asynchronous path: "the edge detection is not required
// and must be disabled by software". 29.6.2.7 on the other two: "edge
// detection must be enabled". A configuration that gets it backwards is
// refused rather than written.
static_assert(Evsys::config_valid(
    EventChannelConfig{.path = EventPath::asynchronous, .edge = EventEdge::none}));
static_assert(!Evsys::config_valid(
    EventChannelConfig{.path = EventPath::asynchronous, .edge = EventEdge::rising}));
static_assert(Evsys::config_valid(
    EventChannelConfig{.path = EventPath::synchronous, .edge = EventEdge::rising}));
static_assert(Evsys::config_valid(
    EventChannelConfig{.path = EventPath::resynchronized, .edge = EventEdge::both}));
static_assert(!Evsys::config_valid(
    EventChannelConfig{.path = EventPath::resynchronized, .edge = EventEdge::none}));

// ---- erratum 1.12.1, as a refusal ------------------------------------------
//
// A SYNCHRONOUS channel whose generic clock never stops raises spurious
// overrun interrupts on every silicon revision. ONDEMAND = 1 is the
// documented workaround and the only synchronous configuration this
// driver will write. The resynchronized path is not affected.
static_assert(!Evsys::config_valid(EventChannelConfig{
    .path = EventPath::synchronous, .edge = EventEdge::rising, .on_demand = false}));
static_assert(Evsys::config_valid(EventChannelConfig{
    .path = EventPath::resynchronized, .edge = EventEdge::rising, .on_demand = false}));

// EVGEN is seven bits, and 95 generators fit inside it.
static_assert(Evsys::config_valid(EventChannelConfig{.generator = 0x7F}));
static_assert(!Evsys::config_valid(EventChannelConfig{.generator = 0x80}));

// The flag masks split the same way CHSTATUS does: OVR low, EVD high.
static_assert(Evsys::overrun_flag(0) == EVSYS_INTFLAG_OVR0_Msk);
static_assert(Evsys::detected_flag(0) == EVSYS_INTFLAG_EVD0_Msk);
static_assert(Evsys::overrun_flag(11) != Evsys::detected_flag(11));

void verbs() {
    constexpr EventChannelConfig async_cfg{};
    constexpr EventChannelConfig sync_cfg{
        .generator = 42, .path = EventPath::synchronous, .edge = EventEdge::rising};

    Evsys::bus_clock(true);
    Evsys::reset();

    (void)Evsys::configure(0, async_cfg);
    (void)Evsys::configure(1, sync_cfg);
    (void)Evsys::channel_reg(0);
    Evsys::release_channel(1);

    (void)Evsys::connect(5, 0, async_cfg);
    (void)Evsys::attach(6, 0);
    (void)Evsys::user_channel(5);
    Evsys::disconnect(6);

    (void)Evsys::channel_status();
    (void)Evsys::busy(0);
    (void)Evsys::users_ready(0);
    (void)Evsys::flags();
    (void)Evsys::armed();
    Evsys::arm(Evsys::detected_flag(0));
    Evsys::disarm(Evsys::overrun_flag(0));
    Evsys::clear_flags(Evsys::detected_flag(0));
    (void)Evsys::overrun(0);
    (void)Evsys::detected(0);
    (void)Evsys::isr();
    (void)Evsys::irq();

    Evsys::trigger(0);
}
