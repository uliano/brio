// mcu: samc21e18a samc21g18a samc21j18a
// The arbiter answers a synchronous start() with the engine's own
// status() - a polled DMA fault, a speed refused before the wire - and
// the app's ISR glue puts the same verb in every TransferDone. An engine
// without it cannot report a synchronous failure at all, so BusMaster
// refuses it at the spelling rather than answering bus_ok in its place.
#include "kernel/post.hpp"
#include "samc21/platform.hpp"
#include "util/bus_master.hpp"
using namespace brio;

struct MuteEngine {
    struct Request {
        uint8_t id;
        ReplyTo<BusDone> reply;
    };
    static bool start(const Request&) { return true; }
    // no status()
};

void f() {
    BusMaster<MuteEngine, SamPlatform>::init();
}
