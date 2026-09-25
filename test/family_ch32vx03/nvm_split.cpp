// The option bytes' MEMORY SPLIT, decoded and never written: RM table
// 32-4's two tables for USER[7:5], which part reads which, where
// FLASH_OBR carries the field, and what ESIG_FLACAP is compared with -
// on every part, the eleven with no split among them, where the decode
// answers nothing and the window is the part table's whatever the byte
// holds.
#include "ch32vx03/nvm.hpp"

using namespace brio;

// ---- the two tables, row by row (table 32-4) --------------------------------
constexpr FlashSplitTable big = FlashSplitTable::code_256k_ram_64k;
constexpr FlashSplitTable small = FlashSplitTable::code_128k_ram_64k;

static_assert(flash_split_of(big, 0b000) == FlashSplit{192, 128});
static_assert(flash_split_of(big, 0b001) == FlashSplit{192, 128});
static_assert(flash_split_of(big, 0b010) == FlashSplit{224, 96});
static_assert(flash_split_of(big, 0b011) == FlashSplit{224, 96});
static_assert(flash_split_of(big, 0b100) == FlashSplit{256, 64});
static_assert(flash_split_of(big, 0b101) == FlashSplit{256, 64});
static_assert(flash_split_of(big, 0b110) == FlashSplit{128, 192});
static_assert(flash_split_of(big, 0b111) == FlashSplit{288, 32});

static_assert(flash_split_of(small, 0b000) == FlashSplit{128, 64});
static_assert(flash_split_of(small, 0b001) == FlashSplit{128, 64});
static_assert(flash_split_of(small, 0b010) == FlashSplit{144, 48});
static_assert(flash_split_of(small, 0b011) == FlashSplit{144, 48});
static_assert(flash_split_of(small, 0b100) == FlashSplit{160, 32});
static_assert(flash_split_of(small, 0b111) == FlashSplit{160, 32});

static_assert(!flash_split_of(FlashSplitTable::none, 0b100).has_value());
// Only the field's three bits are read.
static_assert(flash_split_of(big, 0xFCu) == flash_split_of(big, 0b100));

// Every combination adds up to the same memory: the split moves a line,
// it does not make memory.
static_assert([] {
    for (uint8_t c = 0; c < 8u; ++c) {
        const auto s = flash_split_of(big, c);
        if (!s || s->code_kbytes + s->ram_kbytes != 320u) {
            return false;
        }
    }
    for (uint8_t c = 0; c < 8u; ++c) {
        const auto s = flash_split_of(small, c);
        if (!s || s->code_kbytes + s->ram_kbytes != 192u) {
            return false;
        }
    }
    return true;
}());

static_assert(flash_split_max_kbytes(big) == 288u);
static_assert(flash_split_max_kbytes(small) == 160u);
static_assert(flash_split_max_kbytes(FlashSplitTable::none) == 0u);

// ---- which part reads which table -------------------------------------------
constexpr bool rc_or_vc = device::flash_split_table == big;
constexpr bool rb_203 = device::device_class == DeviceClass::v20x_d8;
static_assert(rc_or_vc == (device::device_class == DeviceClass::v30x_d8 &&
                           device::flash_bytes == 256u * 1024u));
static_assert(rb_203 == (device::flash_split_table == small));

// What ESIG_FLACAP is compared with: the largest window, or the window.
static_assert(flash_window_max_bytes ==
              (rc_or_vc ? 288u * 1024u : rb_203 ? 160u * 1024u : device::flash_bytes));
static_assert(flash_window_max_bytes >= device::flash_bytes);

// ---- where FLASH_OBR carries the field --------------------------------------
// USER whole at [9:2]: a USER byte of 0x3F loads as 0x000000FC (read on a
// CH32V203C8), and the split is [9:7].
static_assert(flash_obr_user_shift == 2u);
static_assert(flash_obr_split_shift == 7u);
static_assert(flash_obr_split_mask == 0x380u);
static_assert(((0x9Fu << flash_obr_user_shift) & flash_obr_split_mask) >> flash_obr_split_shift ==
              (0x9Fu >> flash_user_split_shift));

// ---- the loaded and the stored decode agree, and say the part's window ------
// The factory's split on each table - 10x on the CH32V303RC/VC, 00x on
// the CH32V203RB - is the one the part table states.
constexpr uint8_t factory = rc_or_vc ? 0b100 : 0b000;
constexpr FlashOptions loaded{false, false, true, false, false, 0xFFFFFFFFu, factory};
static_assert(loaded.split_matches_part());
static_assert(loaded.split().has_value() == (rc_or_vc || rb_203));
constexpr FlashOptions moved{false, false, true, false, false, 0xFFFFFFFFu, 0b111};
static_assert(moved.split_matches_part() == !(rc_or_vc || rb_203));

constexpr FlashOptionArea stored{0xA5u, static_cast<uint8_t>((factory << 5) | 0x1Fu), 0u, 0u,
                                 {0xFFu, 0xFFu, 0xFFu, 0xFFu}};
static_assert(stored.split_code() == factory);
static_assert(stored.split() == loaded.split());

void split_verbs() {
    (void)Flash::split();
    (void)FlashOptions::read().split();
    (void)FlashOptions::read().split_matches_part();
    (void)FlashOptionArea::read().split();
    (void)FlashOptionArea::read().split_code();
    (void)(flash_size_kbytes() == flash_window_max_bytes / 1024u);
}
