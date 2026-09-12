// Must FAIL: the journal's half is whole sectors of whole cells - a
// payload that does not fit the cells of a half is refused by the
// geometry assertion, here 32 ids of 200 bytes in a half of 32 cells.
#include "rp2040/nvm_flash.hpp"
#include "util/nv_journal.hpp"

brio::NvJournal<brio::QspiFlashJournalZone, 32, 200, 2> journal;
