// mcu: samc21e18a samc21g18a samc21j18a
// The typed verbs are checked against max_payload at the call site: a
// struct larger than the journal declared can never be stored, so asking
// for it back is a compile error and not a run-time nothing.
// Must not compile.

#include <stdint.h>

#include "samc/nvm_flash.hpp"
#include "util/nv_journal.hpp"

using namespace brio;

struct Oversize {
    uint8_t bytes[40];
};

NvJournal<RwweeJournalZone, 6, 32, 2> journal;

void bad() { (void)journal.load<Oversize>(0); }
