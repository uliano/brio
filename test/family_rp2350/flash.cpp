// Flash family smoke TU: the QMI over both memory windows (timing,
// both transfer formats, both command pairs, the four address
// translation panes, direct mode with its FIFOs and its two chip
// selects), the XIP cache with every control bit and all five
// maintenance operations, and the engine over the bootrom's flash
// functions - every verb instantiated once, and the pure arithmetic of
// the chapter checked at compile time against the register encodings
// the device header states.
#include "rp2350/flash.hpp"

using namespace brio;

// ---- the address map (4.4.1): four aliases of one 26-bit space -------------
static_assert(xip_nocache_base - xip_base == xip_alias_span);
static_assert(xip_maintenance_base - xip_nocache_base == xip_alias_span);
static_assert(xip_untranslated_base - xip_maintenance_base == xip_alias_span);
static_assert(xip_space_span == 4u * xip_alias_span);
static_assert(xip_window_span == 16u * 1024u * 1024u);
static_assert(xip_pane_span * 4u == xip_window_span);

// ---- DIRECT_TX: all control bits zero IS a plain eight-bit byte -----------
static_assert(qmi_direct_word(QmiDirectFrame{.data = 0x9F}) == 0x9Fu);
static_assert((qmi_direct_word(QmiDirectFrame{.data = 0x9F, .no_push = true}) &
               QMI_DIRECT_TX_NOPUSH_BITS) != 0u);
static_assert((qmi_direct_word(QmiDirectFrame{.width = QmiWidth::quad, .output_enable = true}) &
               (QMI_DIRECT_TX_OE_BITS | QMI_DIRECT_TX_IWIDTH_BITS)) ==
              (QMI_DIRECT_TX_OE_BITS | (2u << QMI_DIRECT_TX_IWIDTH_LSB)));
static_assert((qmi_direct_word(QmiDirectFrame{.data = 0xBEEF, .sixteen_bit = true}) &
               QMI_DIRECT_TX_DWIDTH_BITS) != 0u);

// ---- M*_TIMING and M*_RFMT round-trip through their words ----------------
constexpr QmiTiming boot_timing{.clkdiv = 12};
static_assert(qmi_timing_from(qmi_timing_word(boot_timing)).clkdiv == 12u);
static_assert(qmi_timing_word(QmiTiming{.clkdiv = 4, .cooldown = 1}) == QMI_M0_TIMING_RESET);
static_assert(qmi_timing_from(QMI_M0_TIMING_RESET).page_break == QmiPageBreak::none);
static_assert(qmi_timing_from(QMI_M0_TIMING_RESET).cooldown == 1u);

// A 03h serial read with a serial address and no dummy cycles IS the
// register's own reset value (prefix length 8, everything else zero).
static_assert(qmi_format_word(QmiTransferFormat{}) == QMI_M0_RFMT_RESET);
constexpr QmiTransferFormat quad_io{
    .prefix_width = QmiWidth::serial,
    .addr_width = QmiWidth::quad,
    .suffix_width = QmiWidth::quad,
    .dummy_width = QmiWidth::quad,
    .data_width = QmiWidth::quad,
    .prefix_len = QmiPrefixLen::eight,
    .suffix_len = QmiSuffixLen::eight,
    .dummy_len = 4,
};
static_assert(qmi_format_from(qmi_format_word(quad_io)).data_width == QmiWidth::quad);
static_assert(qmi_format_from(qmi_format_word(quad_io)).suffix_len == QmiSuffixLen::eight);
static_assert(qmi_format_from(qmi_format_word(quad_io)).dummy_len == 4u);
static_assert(!qmi_format_from(qmi_format_word(quad_io)).dtr);

static_assert(qmi_command_word(QmiCommand{.prefix = 0xEB, .suffix = 0xA0}) == 0xA0EBu);
static_assert(qmi_command_from(0xA0EBu).prefix == 0xEBu);
static_assert(qmi_command_from(0xA0EBu).suffix == 0xA0u);

// ---- the divisor's arithmetic, with zero meaning 256 (12.14.3) -----------
static_assert(qmi_sck_hz(150'000'000u, 4) == 37'500'000u);
static_assert(qmi_sck_hz(150'000'000u, 12) == 12'500'000u);
static_assert(qmi_sck_hz(150'000'000u, 0) == 585'937u);

// ---- ATRANS: the identity map, and what translation does (12.14.4) -------
static_assert(qmi_atrans_word(qmi_identity_pane(0)) == QMI_ATRANS0_RESET);
static_assert(qmi_atrans_word(qmi_identity_pane(3)) == QMI_ATRANS3_RESET);
static_assert(qmi_atrans_from(QMI_ATRANS2_RESET).base == 2u * xip_pane_span);
static_assert(qmi_atrans_from(QMI_ATRANS2_RESET).size == xip_pane_span);

constexpr std::array<QmiTranslation, 4> identity{qmi_identity_pane(0), qmi_identity_pane(1),
                                                 qmi_identity_pane(2), qmi_identity_pane(3)};
static_assert(qmi_translate(0u, identity) == 0u);
static_assert(qmi_translate(0x00ff'0000u, identity) == 0x00ff'0000u);
static_assert(!qmi_translate(xip_window_span, identity).has_value());

// The chapter's own worked example (figure 141): pane 0 based at 1 MB
// and four megabytes wide, the other three closed. An offset of 1 MB
// into the image reaches 2 MB of storage, and anything in pane 1 has
// nowhere to go.
constexpr std::array<QmiTranslation, 4> rolled{
    QmiTranslation{.base = 1u * 1024u * 1024u, .size = xip_pane_span},
    QmiTranslation{}, QmiTranslation{}, QmiTranslation{}};
static_assert(qmi_translate(0u, rolled) == 1u * 1024u * 1024u);
static_assert(qmi_translate(1024u * 1024u, rolled) == 2u * 1024u * 1024u);
static_assert(!qmi_translate(xip_pane_span, rolled).has_value());

// A base near the top wraps at the window's 16 MB, which is what
// 12.14.4.1 calls a rolling window.
constexpr std::array<QmiTranslation, 4> wrapping{
    QmiTranslation{.base = xip_window_span - qmi_translation_unit, .size = xip_pane_span},
    QmiTranslation{}, QmiTranslation{}, QmiTranslation{}};
static_assert(qmi_translate(0u, wrapping) == xip_window_span - qmi_translation_unit);
static_assert(qmi_translate(qmi_translation_unit, wrapping) == 0u);

// ---- the XIP cache's geometry and its maintenance addresses --------------
static_assert(Xip::cache_bytes == Xip::lines * Xip::line_bytes);
static_assert(Xip::lines == static_cast<uint32_t>(Xip::sets) * Xip::ways);
static_assert(Xip::maintenance_address(0u, XipMaintenance::invalidate_by_set_way) ==
              xip_maintenance_base);
static_assert(Xip::maintenance_address(0u, XipMaintenance::pin_by_set_way) == xip_maintenance_base + 7u);
// Erratum RP2350-E11's sweep: the datasheet's own address, which leaves
// every cleaned line tagged outside the QMI's half of the space.
static_assert(Xip::maintenance_address(xip_alias_span - Xip::cache_bytes,
                                       XipMaintenance::clean_by_set_way) == 0x1bff'c001u);

// ---- the flash chip: geometry and the allow-list -------------------------
static_assert(Flash::size_bytes == 16u * 1024u * 1024u);
static_assert(Flash::sector_size == 16u * Flash::page_size);
static_assert(Flash::block_size == 16u * Flash::sector_size);
static_assert(Flash::window == xip_base);

static_assert(flash_command_reads_only(FlashCommand::read_jedec_id));
static_assert(flash_command_reads_only(FlashCommand::read_unique_id));
static_assert(flash_command_reads_only(FlashCommand::read_status1));
static_assert(flash_command_reads_only(FlashCommand::read_status2));
static_assert(flash_command_reads_only(FlashCommand::read_status3));
static_assert(flash_command_reads_only(FlashCommand::read_sfdp));
static_assert(flash_command_reads_only(FlashCommand::read_data));
static_assert(flash_command_reads_only(FlashCommand::fast_read));
static_assert(flash_command_reads_only(FlashCommand::read_device_id));
// Every opcode that would CHANGE the chip, refused - the write enable,
// both status register writes, the erases, the page program, the reset
// pair and the four-byte address mode.
static_assert(!flash_command_reads_only(0x06));   // write enable
static_assert(!flash_command_reads_only(0x50));   // volatile status write enable
static_assert(!flash_command_reads_only(0x01));   // write status register 1
static_assert(!flash_command_reads_only(0x31));   // write status register 2
static_assert(!flash_command_reads_only(0x11));   // write status register 3
static_assert(!flash_command_reads_only(0x02));   // page program
static_assert(!flash_command_reads_only(0x20));   // sector erase
static_assert(!flash_command_reads_only(0xD8));   // block erase
static_assert(!flash_command_reads_only(0xC7));   // chip erase
static_assert(!flash_command_reads_only(0x66));   // enable reset
static_assert(!flash_command_reads_only(0x99));   // reset device
static_assert(!flash_command_reads_only(0xB7));   // four-byte address mode

// The capacity byte 9Fh answers, decoded.
static_assert(FlashJedecId{.manufacturer = 0xEF, .type = 0x40, .capacity = 0x18}.bytes() ==
              16u * 1024u * 1024u);
static_assert(FlashJedecId{.manufacturer = 0, .type = 0, .capacity = 0xFF}.bytes() == 0u);

// FLASH_DEVINFO's size codes (5.4.8.5): zero is "none" and every other
// code is eight kilobytes shifted.
static_assert(FlashDevInfo::size_bytes(0) == 0u);
static_assert(FlashDevInfo::size_bytes(1) == 8u * 1024u);
static_assert(FlashDevInfo::size_bytes(0xc) == 16u * 1024u * 1024u);
static_assert(FlashDevInfo{.raw = 0x0C00}.cs0_bytes() == 16u * 1024u * 1024u);
static_assert(FlashDevInfo{.raw = 0x0C00}.cs1_bytes() == 0u);
static_assert(!FlashDevInfo{.raw = 0x0C00}.d8h_erase_supported());
static_assert(FlashDevInfo{.raw = 0x0C80}.d8h_erase_supported());
static_assert(FlashDevInfo{.raw = 0x0C1F}.cs1_gpio() == 31u);

// ---- every verb, instantiated once ---------------------------------------

template <uint8_t w>
void qmi_window_verbs() {
    (void)QmiWindow<w>::window_base;
    (void)QmiWindow<w>::index;

    const auto t = QmiWindow<w>::timing();
    QmiWindow<w>::set_timing(t);
    QmiWindow<w>::set_clkdiv(12);

    const auto rf = QmiWindow<w>::read_format();
    QmiWindow<w>::set_read_format(rf);
    const auto wf = QmiWindow<w>::write_format();
    QmiWindow<w>::set_write_format(wf);

    const auto rc = QmiWindow<w>::read_command();
    QmiWindow<w>::set_read_command(rc);
    const auto wc = QmiWindow<w>::write_command();
    QmiWindow<w>::set_write_command(wc);

    (void)QmiWindow<w>::template atrans_offset<0>();
    const auto p0 = QmiWindow<w>::template translation<0>();
    QmiWindow<w>::template set_translation<0>(p0);
    QmiWindow<w>::template set_translation<1>(qmi_identity_pane(1));
    QmiWindow<w>::template set_translation<2>(qmi_identity_pane(2));
    QmiWindow<w>::template set_translation<3>(qmi_identity_pane(3));
    (void)QmiWindow<w>::translations();
    (void)QmiWindow<w>::translation_is_identity();
    (void)QmiWindow<w>::translate(0x1234u);

    QmiWindow<w>::assert_select(true);
    (void)QmiWindow<w>::select_asserted();
    QmiWindow<w>::assert_select(false);
    QmiWindow<w>::auto_select(true);
    QmiWindow<w>::auto_select(false);

    (void)Xip::template window_writable<w>();
    Xip::template set_window_writable<w>(false);
}

void qmi_verbs() {
    (void)Qmi::base;
    (void)&Qmi::csr();
    Qmi::enable_direct(true);
    (void)Qmi::direct_enabled();
    (void)Qmi::busy();
    (void)Qmi::tx_full();
    (void)Qmi::tx_empty();
    (void)Qmi::rx_full();
    (void)Qmi::rx_empty();
    (void)Qmi::tx_level();
    (void)Qmi::rx_level();
    Qmi::set_direct_clkdiv(6);
    (void)Qmi::direct_clkdiv();
    Qmi::set_direct_rxdelay(1);
    (void)Qmi::direct_rxdelay();
    Qmi::push(0x9Fu);
    Qmi::push(QmiDirectFrame{.data = 0x00, .no_push = true});
    (void)Qmi::pop();
    (void)&Qmi::tx_fifo();
    (void)&Qmi::rx_fifo();
    (void)&Qmi::fast_tx_fifo();
    (void)&Qmi::fast_rx_fifo();
    Qmi::enable_direct(false);
    Qmi::reset_address_translation();

    qmi_window_verbs<0>();
    qmi_window_verbs<1>();
}

void xip_verbs() {
    Xip::enable(true);
    (void)Xip::enabled_secure();
    (void)Xip::enabled_nonsecure();
    Xip::enable_secure(true);
    Xip::enable_nonsecure(true);
    Xip::power_down(false);
    (void)Xip::powered_down();
    (void)Xip::split_ways();
    Xip::set_split_ways(false);
    (void)Xip::maintenance_nonsecure();
    Xip::allow_nonsecure_maintenance(false);
    Xip::refuse_uncached(false, false);
    Xip::refuse_untranslated(false, false);
    (void)Xip::uncached_refused_secure();
    (void)Xip::untranslated_refused_secure();

    Xip::reset_counters();
    (void)Xip::hits();
    (void)Xip::accesses();

    Xip::invalidate_all();
    Xip::clean_all();
    Xip::invalidate_address(0x1000u);
    Xip::clean_address(0x1000u);
    Xip::invalidate_range(0x1000u, 256u);
    Xip::pin_address(xip_alias_span - 8u);

    Xip::stream_start(xip_base + 0x1000u, 16u);
    (void)Xip::stream_remaining();
    (void)Xip::stream_empty();
    (void)Xip::stream_full();
    (void)Xip::stream_pop();
    (void)&Xip::stream_fifo();
    (void)&Xip::fast_stream_fifo();
    Xip::stream_stop();
}

uint8_t page[Flash::page_size];
uint8_t answer[16];

void flash_verbs() {
    (void)Flash::init();
    (void)Flash::ready();
    (void)Flash::xip_restore_kind();
    (void)Flash::xip_setup_named_by_rom();
    (void)Flash::commit_writes();
    (void)Flash::setup_word(0);
    (void)Flash::device_info();
    (void)Flash::runtime_to_storage(xip_base);

    (void)Flash::in_window(page);
    (void)Flash::address(0u);
    (void)Flash::uncached_address(0u);
    (void)Flash::untranslated_address(0u);

    Flash::read(0x00ff'0000u, page);
    (void)Flash::erase(0x00ff'0000u, Flash::sector_size);
    (void)Flash::erase(0x00ff'0000u, Flash::sector_size, FlashEraseGrain::sector_or_block);
    (void)Flash::erase_sector(0x00ff'0000u);
    (void)Flash::program(0x00ff'0000u, page);

    const uint8_t head[4] = {0, 0, 0, 0};
    (void)Flash::command(FlashCommand::read_jedec_id, {}, answer);
    (void)Flash::command<FlashCommand::read_unique_id>(head, answer);
    (void)Flash::jedec_id();
    (void)Flash::unique_id();
    (void)Flash::status_register<1>();
    (void)Flash::status_register<2>();
    (void)Flash::status_register<3>();
    (void)Flash::read_sfdp(0u, answer);
}

void flash_family() {
    qmi_verbs();
    xip_verbs();
    flash_verbs();
}
