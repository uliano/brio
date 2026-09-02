// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// THE GEOMETRY INVARIANT, and it is the one this campaign's partition was
// sized against: after a collection a half must hold max_ids
// maximum-size entries, PLUS the entry being written, PLUS the panic
// reserve - (max_ids + 2) x max_entry_cells <= half_cells. One 2048-byte
// page is 256 eight-byte cells and a 32-byte value costs 6 of them, so
// 40 ids fit and 200 cannot. A journal that cannot guarantee its reserve
// would make JournalPanic a lie, so it is refused at compile time.
#include "stm32g0/nvm_flash.hpp"
#include "util/nv_journal.hpp"
using namespace brio;
NvJournal<MainFlashJournalZone, 200, 32, 1> journal;
void f() { (void)journal.mount(); }
