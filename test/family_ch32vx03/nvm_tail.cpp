// mcu: ch32v203f6 ch32v203f8 ch32v203g6 ch32v203g8 ch32v203k6 ch32v203k8 ch32v203c6 ch32v203c8 ch32v203rb ch32v303rc ch32v303vc
// The flash's NON-ZERO-WAIT TAIL: the three numbers of the array, the
// reach of both methods - the standard one and the fast one alike, as
// the CH32V303VC measured - and the tail's own read with its rate
// contract, every verb in both spellings, on every part that has a
// tail: all nine
// CH32V203 (the datasheet's note 1 to table 2-1 gives each of them
// 224K - R0WAIT above its window) and the CH32V303RC and VC (480K -
// R0WAIT). The CH32V303CB and RB have none and are the negatives'.
#include "ch32vx03/nvm.hpp"

using namespace brio;

// ---- the three numbers (the part table) ------------------------------------
static_assert(Flash::has_tail);
static_assert(Flash::window_bytes == device::flash_bytes);
static_assert(Flash::tail_bytes == device::flash_tail_bytes);
static_assert(Flash::array_bytes == device::flash_array_bytes);
static_assert(Flash::tail_base == Flash::window_bytes);
static_assert(Flash::tail_end == Flash::window_bytes + Flash::tail_bytes);
// On every part with a tail, the window and the tail ARE the array.
static_assert(Flash::tail_end == Flash::array_bytes);
static_assert(Flash::array_bytes ==
              (device::device_class == DeviceClass::v30x_d8 ? 480u : 224u) * 1024u);

// The tail starts on a sector, and a sector is the standard erase grain,
// so its first and last sectors are both addresses erase_sector takes.
static_assert(Flash::tail_base % Flash::sector_size == 0u);
static_assert(Flash::tail_end % Flash::sector_size == 0u);
static_assert(Flash::tail_base % Flash::block_size == 0u);
static_assert(Flash::tail_bytes % Flash::block_size == 0u);
constexpr uint32_t first_sector = Flash::tail_base;
constexpr uint32_t last_sector = Flash::tail_end - Flash::sector_size;

// ---- the two predicates at their edges --------------------------------------
static_assert(Flash::in_window(0u, Flash::window_bytes));
static_assert(!Flash::in_window(0u, Flash::window_bytes + 1u));
static_assert(!Flash::in_window(Flash::tail_base, 1u));
static_assert(Flash::in_tail(Flash::tail_base, 1u));
static_assert(Flash::in_tail(last_sector, Flash::sector_size));
static_assert(!Flash::in_tail(last_sector, Flash::sector_size + 1u));
static_assert(!Flash::in_tail(Flash::tail_base - 1u, 1u));
static_assert(!Flash::in_tail(Flash::tail_end, 1u));
// A run that straddles the line belongs to neither side, and so to no
// write verb.
static_assert(!Flash::in_window(Flash::tail_base - 2u, 4u));
static_assert(!Flash::in_tail(Flash::tail_base - 2u, 4u));
static_assert(!Flash::in_array(Flash::tail_base - 2u, 4u));
static_assert(Flash::in_array(0u, Flash::window_bytes));
static_assert(Flash::in_array(last_sector, Flash::sector_size));
static_assert(!Flash::in_array(Flash::tail_end, 1u));

using Safe = Clock<ClockSource::pll, 96'000'000>;
using Full = Clock<ClockSource::pll, 144'000'000>;
using Paced = DynamicClock<Rates<Safe, Full>>;

static uint8_t page_buffer[Flash::page_size];
static uint8_t read_buffer[64];

void tail_verbs() {
    // The standard method, at a tail address: taken.
    (void)Flash::erase_sector(Safe{}, first_sector);
    (void)Flash::erase_sector(Safe{}, last_sector);
    (void)Flash::program_half_word(Safe{}, first_sector, 0x1234u);
    (void)Flash::program_half_word(Safe{}, Flash::tail_end - 2u, 0x5678u);

    // The same with the address a constant.
    (void)Flash::erase_sector<first_sector>(Safe{});
    (void)Flash::erase_sector<last_sector>(Safe{});
    (void)Flash::program_half_word<first_sector + 2u>(Safe{}, 0x9ABCu);

    // The fast method at a tail address: taken too, in both spellings -
    // the page program, the page erase and the 32 KB erase, whose tail
    // is seven whole blocks.
    (void)Flash::erase_page(Safe{}, first_sector);
    (void)Flash::erase_block32(Safe{}, first_sector);
    (void)Flash::program_page(Safe{}, first_sector,
                              std::span<const uint8_t>(page_buffer, Flash::page_size));
    (void)Flash::erase_page<last_sector>(Safe{});
    (void)Flash::erase_block32<Flash::tail_end - Flash::block_size>(Safe{});
    (void)Flash::program_page<last_sector>(Safe{}, std::span<const uint8_t>(page_buffer,
                                                                              Flash::page_size));

    // The tail read, both spellings, under a static clock and a dynamic
    // one - where 144 MHz is refused with a code.
    (void)Flash::read_tail(Safe{}, first_sector, std::span<uint8_t>(read_buffer, sizeof read_buffer));
    (void)Flash::read_tail<first_sector>(Safe{}, std::span<uint8_t>(read_buffer, sizeof read_buffer));
    (void)Flash::read_tail(Paced{}, last_sector, std::span<uint8_t>(read_buffer, sizeof read_buffer));
    (void)Flash::erase_sector(Paced{}, last_sector);
    (void)Flash::program_half_word(Paced{}, last_sector, 0xDEF0u);

    // The window's spellings of every write verb.
    (void)Flash::erase_page<0u>(Safe{});
    (void)Flash::erase_block32<0u>(Safe{});
    (void)Flash::program_page<0u>(Safe{}, std::span<const uint8_t>(page_buffer, Flash::page_size));
    (void)Flash::erase_sector<0u>(Safe{});
    (void)Flash::program_half_word<0u>(Safe{}, 0x1111u);
}

// The protection bits reach the tail too: every sector from 31 up is the
// last bit of WPR (32.6's WRPR3 note), and the tail's last sector is
// among them on every part - 55 on the CH32V203, 119 on the CH32V303.
constexpr FlashOptions top_locked{false, false, false, false, false, 0x7FFFFFFFu, 0u};
static_assert(top_locked.address_protected(last_sector));
static_assert(!top_locked.address_protected(0u));
