// mcu: samc21e18a samc21g18a samc21j18a
// A timed BusMaster declares an engine dead and must be able to put it
// back where start() is legal: Bus::recover() is the contract verb,
// static_asserted at the spelling. An engine without it can carry an
// UNTIMED bus (the default, byte-identical images) but naming a
// timeout_ticks over it must be refused - not discovered when the
// first wedged transfer calls a verb that is not there.
#include "kernel/post.hpp"
#include "samc/platform_sam.hpp"
#include "util/bus_master.hpp"
using namespace brio;

struct NoRecoverEngine {
    struct Request {
        uint8_t id;
        ReplyTo<BusDone> reply;
    };
    static bool start(const Request&) { return true; }
    // no recover()
};

void f() {
    BusMaster<NoRecoverEngine, SamPlatform, 4, BusPassThrough, 100>::init();
}
