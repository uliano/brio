// mcu: stm32g0b1xx stm32g071xx stm32g031xx
// A journal half must be at least ONE erase unit (util/nv_journal.hpp):
// erasing one half must not take the other down with it, and on this
// target an erase unit is a whole 2048-byte page. A half_pages of zero
// would put both halves in the same page.
#include "stm32g0/nvm_flash.hpp"
#include "util/nv_journal.hpp"
using namespace brio;
NvJournal<MainFlashJournalZone, 6, 32, 0> journal;
void f() { (void)journal.mount(); }
