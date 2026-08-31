// SERCOM (SPI mode) family smoke TU: the Spi<n> resource, the SpiHost
// and SpiClient tasks over it, the DOPO triple table and the synchronous
// baud arithmetic.
//
// SERCOM0 is what every variant of the pack has, and its four pads are
// bonded to PA04..PA07 through PMUX function D on the E, the G and the J
// alike - so a four-wire SPI is expressible in this fixture without any
// per-package pad table. The board's own SERCOM1 on PA16..PA19 is an
// app-level fact and is not compiled here.
#include "samc/clock.hpp"
#include "samc/spi.hpp"

using namespace brio;

using SysClock = Clock<ClockSource::internal, 48'000'000>;

// ---- the DOPO triple, exhaustively -----------------------------------------
// 32.8.1's table has four rows and 32.8.1's table is all there is: a
// (DO, SCK, SS) triple that is not one of them is a wiring this
// peripheral cannot serve, whatever PORT says.
static_assert(spi_dopo_for(SercomPad::pad0, SercomPad::pad1, SercomPad::pad2).value() == 0);
static_assert(spi_dopo_for(SercomPad::pad2, SercomPad::pad3, SercomPad::pad1).value() == 1);
static_assert(spi_dopo_for(SercomPad::pad3, SercomPad::pad1, SercomPad::pad2).value() == 2);
static_assert(spi_dopo_for(SercomPad::pad0, SercomPad::pad3, SercomPad::pad1).value() == 3);
// The header names its codes after a PAD that is not the one DO lands on
// - the TXPO trap of sercom.hpp, again. Held here so a device-pack
// rename cannot silently move a signal.
static_assert(SERCOM_SPIM_CTRLA_DOPO_PAD1_Val == 1);   // "PAD1" = DO on PAD[2]
static_assert(SERCOM_SPIM_CTRLA_DOPO_PAD2_Val == 2);   // "PAD2" = DO on PAD[3]
// Twenty of the 24 orderings of three distinct pads are not rows.
static_assert(!spi_dopo_for(SercomPad::pad1, SercomPad::pad0, SercomPad::pad2));
static_assert(!spi_dopo_for(SercomPad::pad0, SercomPad::pad2, SercomPad::pad1));
static_assert(!spi_dopo_for(SercomPad::pad3, SercomPad::pad2, SercomPad::pad1));
// And a triple that reuses a pad is not one either.
static_assert(!spi_dopo_for(SercomPad::pad0, SercomPad::pad0, SercomPad::pad2));

// DIPO is the plain pad number.
static_assert(spi_dipo(SercomPad::pad0) == 0);
static_assert(spi_dipo(SercomPad::pad3) == 3);

// ---- the pads this fixture wires -------------------------------------------
// The HOST row: DO(MOSI) PAD[0], SCK PAD[1], SS PAD[2], DI(MISO) PAD[3].
constexpr SpiPads host_pads{
    .data_out = SercomPad::pad0,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad3,
    .data_out_pin = {'A', 4, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 7, PinFunction::d},
};
// The CLIENT row on THE SAME FOUR WIRES: the directions swap, so DO
// (which is MISO now) has to be the pad the host reads - PAD[3] - and
// only row 0x2 puts it there. This is the fact that makes a
// bidirectional harness two DOPO codes and not one.
constexpr SpiPads client_pads{
    .data_out = SercomPad::pad3,
    .sck = SercomPad::pad1,
    .ss = SercomPad::pad2,
    .data_in = SercomPad::pad0,
    .data_out_pin = {'A', 7, PinFunction::d},
    .sck_pin = {'A', 5, PinFunction::d},
    .ss_pin = {'A', 6, PinFunction::d},
    .data_in_pin = {'A', 4, PinFunction::d},
};
static_assert(spi_pads_valid(host_pads));
static_assert(spi_pads_valid(client_pads));
static_assert(spi_dopo_for(host_pads.data_out, host_pads.sck, host_pads.ss).value() == 0);
static_assert(spi_dopo_for(client_pads.data_out, client_pads.sck, client_pads.ss).value() == 2);

// The device header must agree that PA04..PA07 reach SERCOM0's four pads
// through function D - the one half of the pad-to-pin claim that IS
// checkable without a per-package pad table.
static_assert(MUX_PA04D_SERCOM0_PAD0 == static_cast<uint8_t>(PinFunction::d));
static_assert(MUX_PA05D_SERCOM0_PAD1 == static_cast<uint8_t>(PinFunction::d));
static_assert(MUX_PA06D_SERCOM0_PAD2 == static_cast<uint8_t>(PinFunction::d));
static_assert(MUX_PA07D_SERCOM0_PAD3 == static_cast<uint8_t>(PinFunction::d));

// A pad layout with an unreal pin is refused whichever pad it is.
static_assert(!spi_pads_valid({.data_out_pin = {'C', 0, PinFunction::d},
                               .sck_pin = {'A', 5, PinFunction::d},
                               .data_in_pin = {'A', 7, PinFunction::d}}));
static_assert(!spi_pads_valid({.data_out_pin = {'A', 4, PinFunction::d},
                               .sck_pin = {'A', 5, PinFunction::d},
                               .data_in_pin = {'A', 40, PinFunction::d}}));
// ... except the DI pin, when the DI pad is not claimed at all (32.5.1:
// with the receiver off the data input pin is free for other purposes).
static_assert(spi_pads_valid({.data_out_pin = {'A', 4, PinFunction::d},
                              .sck_pin = {'A', 5, PinFunction::d},
                              .data_in_pin = {'C', 0, PinFunction::d},
                              .has_data_in = false}));

// ---- the mode vocabulary ----------------------------------------------------
// brio's SpiMode is ONE spelling across the architectures: bit 1 is CPOL,
// bit 0 is CPHA, whichever register the two bits live in.
static_assert(!spi_cpol(SpiMode::mode0) && !spi_cpha(SpiMode::mode0));
static_assert(!spi_cpol(SpiMode::mode1) && spi_cpha(SpiMode::mode1));
static_assert(spi_cpol(SpiMode::mode2) && !spi_cpha(SpiMode::mode2));
static_assert(spi_cpol(SpiMode::mode3) && spi_cpha(SpiMode::mode3));

// ---- the synchronous baud arithmetic ----------------------------------------
// Table 30-2, synchronous row: f_SCK = f_ref / (2 x (BAUD + 1)), BAUD
// eight bits (32.8.3) - so 128 rates from f_ref/2 down to f_ref/512.
static_assert(spi_baud_reg(48'000'000, 12'000'000).value() == 1);
static_assert(spi_sck_hz(48'000'000, 1) == 12'000'000);
static_assert(spi_baud_reg(48'000'000, 200'000).value() == 119);
static_assert(spi_sck_hz(48'000'000, 119) == 200'000);
static_assert(spi_max_sck_hz(48'000'000) == 24'000'000);
static_assert(spi_min_sck_hz(48'000'000) == 93'750);
// BAUD 0 is LEGAL (it is the fastest rate), so it cannot double as a
// refusal - which is what the optional is for.
static_assert(spi_baud_reg(48'000'000, 24'000'000).value() == 0);
static_assert(!spi_baud_reg(48'000'000, 24'000'001));
static_assert(!spi_baud_reg(48'000'000, 93'749));
static_assert(!spi_baud_reg(0, 1'000'000));
static_assert(!spi_baud_reg(48'000'000, 0));
// The rounding rule: a requested SCK is a CEILING, so an inexact
// divisor lands BELOW it and never above.
static_assert(spi_sck_hz(48'000'000, spi_baud_reg(48'000'000, 700'000).value()) <= 700'000);
static_assert(spi_sck_hz(48'000'000, spi_baud_reg(48'000'000, 333'333).value()) <= 333'333);

// ---- the configuration's refusals -------------------------------------------
constexpr SpiConfig host_cfg{.pads = host_pads, .role = SpiRole::host};
constexpr SpiConfig client_cfg{.pads = client_pads, .role = SpiRole::client};
static_assert(spi_config_valid(host_cfg));
static_assert(spi_config_valid(client_cfg));

// Whose knob is whose (32.8.1, 32.8.2).
static_assert(!spi_config_valid({.pads = client_pads, .role = SpiRole::client,
                                 .hardware_ss = true}));
static_assert(!spi_config_valid({.pads = host_pads, .role = SpiRole::host,
                                 .preload = true}));
static_assert(!spi_config_valid({.pads = host_pads, .role = SpiRole::host,
                                 .ss_low_detect = true}));
static_assert(!spi_config_valid({.pads = host_pads, .role = SpiRole::host,
                                 .form = SpiForm::with_address}));
// 32.6.3.1's own sentence: address matching and preloading both want the
// client's first character.
static_assert(!spi_config_valid({.pads = client_pads, .role = SpiRole::client,
                                 .form = SpiForm::with_address, .preload = true}));
static_assert(spi_config_valid({.pads = client_pads, .role = SpiRole::client,
                                .form = SpiForm::with_address}));
static_assert(spi_config_valid({.pads = client_pads, .role = SpiRole::client,
                                .preload = true}));
// Reserved codes, reachable only by a cast and refused all the same.
static_assert(!spi_config_valid({.pads = host_pads, .form = static_cast<SpiForm>(1)}));
static_assert(!spi_config_valid({.pads = host_pads, .form = static_cast<SpiForm>(0xF)}));
static_assert(!spi_config_valid({.pads = host_pads, .bits = static_cast<SpiCharSize>(2)}));
static_assert(!spi_config_valid({.pads = client_pads, .role = SpiRole::client,
                                 .form = SpiForm::with_address,
                                 .address_mode = static_cast<SpiAddressMode>(3)}));
// A client must be able to receive and must have a real SS pin.
static_assert(!spi_config_valid({.pads = {.data_out = SercomPad::pad3,
                                          .sck = SercomPad::pad1,
                                          .ss = SercomPad::pad2,
                                          .data_in = SercomPad::pad0,
                                          .data_out_pin = {'A', 7, PinFunction::d},
                                          .sck_pin = {'A', 5, PinFunction::d},
                                          .ss_pin = {'A', 6, PinFunction::d},
                                          .has_data_in = false},
                                 .role = SpiRole::client}));
static_assert(!spi_config_valid({.pads = {.data_out = SercomPad::pad3,
                                          .sck = SercomPad::pad1,
                                          .ss = SercomPad::pad2,
                                          .data_in = SercomPad::pad0,
                                          .data_out_pin = {'A', 7, PinFunction::d},
                                          .sck_pin = {'A', 5, PinFunction::d},
                                          .ss_pin = {'C', 0, PinFunction::d},
                                          .data_in_pin = {'A', 4, PinFunction::d}},
                                 .role = SpiRole::client}));
// A HOST with software chip select never claims the SS pad, so its
// ss_pin is not read at all - which is the shape every device client in
// this framework really uses.
static_assert(spi_config_valid({.pads = {.data_out_pin = {'A', 4, PinFunction::d},
                                         .sck_pin = {'A', 5, PinFunction::d},
                                         .ss_pin = {'C', 9, PinFunction::d},
                                         .data_in_pin = {'A', 7, PinFunction::d}},
                                .role = SpiRole::host}));

// The DRIVER's own two refusals about the input pad, and the one case it
// deliberately does NOT refuse.
static_assert(!spi_config_valid({.pads = {.data_in = SercomPad::pad1,   // == SCK
                                          .data_out_pin = {'A', 4, PinFunction::d},
                                          .sck_pin = {'A', 5, PinFunction::d},
                                          .data_in_pin = {'A', 5, PinFunction::d}}}));
static_assert(!spi_config_valid({.pads = {.data_in = SercomPad::pad2,   // == a claimed SS
                                          .data_out_pin = {'A', 4, PinFunction::d},
                                          .sck_pin = {'A', 5, PinFunction::d},
                                          .ss_pin = {'A', 6, PinFunction::d},
                                          .data_in_pin = {'A', 6, PinFunction::d}},
                                 .hardware_ss = true}));
// LOOP-BACK: DI on the DO pad is 32.6.3.4's own arrangement ("the
// loop-back is through the pad"), so it is allowed on purpose.
static_assert(spi_config_valid({.pads = {.data_in = SercomPad::pad0,
                                         .data_out_pin = {'A', 4, PinFunction::d},
                                         .sck_pin = {'A', 5, PinFunction::d},
                                         .data_in_pin = {'A', 4, PinFunction::d}}}));

// ---- the register words ------------------------------------------------------
static_assert((spi_ctrla(host_cfg) & SERCOM_SPIM_CTRLA_MODE_Msk) ==
              SERCOM_SPIM_CTRLA_MODE(SERCOM_SPIM_CTRLA_MODE_SPI_MASTER_Val));
static_assert((spi_ctrla(client_cfg) & SERCOM_SPIM_CTRLA_MODE_Msk) ==
              SERCOM_SPIM_CTRLA_MODE(SERCOM_SPIM_CTRLA_MODE_SPI_SLAVE_Val));
// ENABLE is never part of the configuration word: CTRLA is
// enable-protected and is written with the peripheral stopped.
static_assert((spi_ctrla(host_cfg) & SERCOM_SPIM_CTRLA_ENABLE_Msk) == 0);
static_assert((spi_ctrla(host_cfg) & SERCOM_SPIM_CTRLA_DOPO_Msk) ==
              SERCOM_SPIM_CTRLA_DOPO(0));
static_assert((spi_ctrla(client_cfg) & SERCOM_SPIM_CTRLA_DOPO_Msk) ==
              SERCOM_SPIM_CTRLA_DOPO(2));
static_assert((spi_ctrla(host_cfg) & SERCOM_SPIM_CTRLA_DIPO_Msk) ==
              SERCOM_SPIM_CTRLA_DIPO(3));
// Unlike the USART's DORD, SPI's reset value IS the convention: MSB
// first, so the default configuration writes the bit clear.
static_assert((spi_ctrla(host_cfg) & SERCOM_SPIM_CTRLA_DORD_Msk) == 0);
static_assert((spi_ctrla({.pads = host_pads, .lsb_first = true}) &
               SERCOM_SPIM_CTRLA_DORD_Msk) != 0);
// The four transfer modes as the two CTRLA bits.
static_assert((spi_ctrla({.pads = host_pads, .mode = SpiMode::mode0}) &
               (SERCOM_SPIM_CTRLA_CPOL_Msk | SERCOM_SPIM_CTRLA_CPHA_Msk)) == 0);
static_assert((spi_ctrla({.pads = host_pads, .mode = SpiMode::mode1}) &
               (SERCOM_SPIM_CTRLA_CPOL_Msk | SERCOM_SPIM_CTRLA_CPHA_Msk)) ==
              SERCOM_SPIM_CTRLA_CPHA_Msk);
static_assert((spi_ctrla({.pads = host_pads, .mode = SpiMode::mode2}) &
               (SERCOM_SPIM_CTRLA_CPOL_Msk | SERCOM_SPIM_CTRLA_CPHA_Msk)) ==
              SERCOM_SPIM_CTRLA_CPOL_Msk);
static_assert((spi_ctrla({.pads = host_pads, .mode = SpiMode::mode3}) &
               (SERCOM_SPIM_CTRLA_CPOL_Msk | SERCOM_SPIM_CTRLA_CPHA_Msk)) ==
              (SERCOM_SPIM_CTRLA_CPOL_Msk | SERCOM_SPIM_CTRLA_CPHA_Msk));

static_assert((spi_ctrlb(host_cfg) & SERCOM_SPIM_CTRLB_RXEN_Msk) != 0);
static_assert((spi_ctrlb({.pads = host_pads, .receiver = false}) &
               SERCOM_SPIM_CTRLB_RXEN_Msk) == 0);
static_assert((spi_ctrlb({.pads = host_pads, .hardware_ss = true}) &
               SERCOM_SPIM_CTRLB_MSSEN_Msk) != 0);
static_assert((spi_ctrlb({.pads = client_pads, .role = SpiRole::client, .preload = true}) &
               SERCOM_SPIM_CTRLB_PLOADEN_Msk) != 0);
static_assert((spi_ctrlb({.pads = host_pads, .bits = SpiCharSize::nine}) &
               SERCOM_SPIM_CTRLB_CHSIZE_Msk) ==
              SERCOM_SPIM_CTRLB_CHSIZE(SERCOM_SPIM_CTRLB_CHSIZE_9_BIT_Val));
static_assert(spi_addr({.address = 0x42, .address_mask = 0xF0}) ==
              (SERCOM_SPIM_ADDR_ADDR(0x42) | SERCOM_SPIM_ADDR_ADDRMASK(0xF0)));

// ---- the resource's verbs -----------------------------------------------------
void resource_verbs() {
    using Sp = Spi<0>;
    static_assert(Sp::index == 0);
    // The per-INSTANCE facts are Sercom<n>'s and must be the same ones,
    // not a second table: this is the assertion that keeps a second
    // personality from becoming a second source of truth.
    static_assert(Sp::gclk_core_id() == Sercom<0>::gclk_core_id());
    static_assert(Sp::apb_mask() == Sercom<0>::apb_mask());
    static_assert(Sp::irq() == Sercom<0>::irq());
    static_assert(Sp::dma_rx_trigger() == Sercom<0>::dma_rx_trigger());
    static_assert(Sp::dma_tx_trigger() == Sercom<0>::dma_tx_trigger());

    Sp::bus_clock(true);
    (void)Sp::core_clock(0);
    (void)Sp::reset();
    (void)Sp::configure(host_cfg);
    (void)Sp::configure<host_cfg>();      // the compile-time twin
    (void)Sp::configure<client_cfg>();
    (void)Sp::enable(true);
    (void)Sp::enabled();
    (void)Sp::sync_busy(SERCOM_SPIM_SYNCBUSY_CTRLB_Msk);
    (void)Sp::wait_sync(SERCOM_SPIM_SYNCBUSY_ENABLE_Msk);

    (void)Sp::baud_reg();
    Sp::baud_reg(119);
    (void)Sp::ctrla();
    (void)Sp::ctrlb();
    (void)Sp::addr_reg();
    (void)Sp::dbgctrl();
    (void)Sp::role();
    (void)Sp::mode();
    (void)Sp::receiver();
    (void)Sp::receiver(false);
    (void)Sp::data_address();

    (void)Sp::pending();
    (void)Sp::flags();
    (void)Sp::armed();
    Sp::clear_flags(SpiFlag::txc | SpiFlag::ssl);
    Sp::enable_interrupt(SpiFlag::rxc, true);
    Sp::enable_dre_interrupt(false);
    Sp::enable_rxc_interrupt(true);
    Sp::enable_txc_interrupt(false);
    Sp::enable_ssl_interrupt(false);
    (void)Sp::dre_flag();
    (void)Sp::rxc_flag();
    (void)Sp::txc_flag();
    (void)Sp::ssl_flag();
    (void)Sp::error_flag();

    (void)Sp::status();
    (void)Sp::overflow_flag();
    Sp::clear_status(SpiStatus::receive_errors);
    (void)Sp::data();
    Sp::data(0x55);
    Sp::flush_rx();
    Sp::release();

    // The highest instance this device has, whichever that is.
    using Last = Spi<sercom_count - 1>;
    (void)Last::irq();
}

// ---- the tasks -----------------------------------------------------------------
using Host = SpiHost<0, host_pads>;
using Peer = SpiClient<0, client_pads>;

// The Request is the bus arbiter's token and must be copyable into its
// pending FIFO (util/bus_master.hpp).
static_assert(std::is_trivially_copyable_v<Host::Request>);
static_assert(Host::core_generator == 0);

void host_verbs() {
    constexpr SysClock clock;
    (void)Host::init(clock);
    (void)Host::init(clock, 4'000'000);   // with a bus-wide SCK ceiling
    Host::rebase(24'000'000);
    (void)Host::baud_for(1'000'000);
    (void)Host::sck_hz(23);
    (void)Host::max_sck_hz();
    (void)Host::ceiling_baud();
    (void)Host::reference_hz();

    static uint8_t cmd[2] = {0x9F, 0x00};
    static uint8_t out[4] = {1, 2, 3, 4};
    static uint8_t in[4] = {};
    const Host::Request r{
        .cs = Pin<'B', 0>::ref(),
        .dc = {},
        .cmd = lend<Lease::reply>(static_cast<const uint8_t*>(cmd)),
        .cmd_len = 2,
        .tx = lend<Lease::reply>(static_cast<const uint8_t*>(out)),
        .rx = lend<Lease::reply>(in),
        .len = 4,
        .reply = {},
        .baud = 23,
        .mode = SpiMode::mode3,
        .polled = false,
    };
    (void)Host::start(r);
    (void)Host::isr();
    // A degenerate request completes inside start() - the arbiter's
    // synchronous-completion path.
    (void)Host::start(Host::Request{});
    Host::release();
}

void client_verbs() {
    constexpr SysClock clock;
    (void)Peer::init(clock);
    (void)Peer::init(clock, {.mode = SpiMode::mode1,
                             .lsb_first = true,
                             .preload = true,
                             .ss_low_detect = true});
    (void)Peer::init(clock, {.preload = false,
                             .address_match = true,
                             .address_mode = SpiAddressMode::range,
                             .address = 0x7F,
                             .address_mask = 0x40,
                             .drive_output = false});
    Peer::drive_output(true);
    (void)Peer::selected();
    Peer::write(0xA5);
    (void)Peer::writable();
    (void)Peer::poll();
    (void)Peer::overflow();
    Peer::clear_overflow();
    (void)Peer::transaction_done();
    Peer::clear_transaction_done();
    (void)Peer::select_edge();
    Peer::clear_select_edge();
    (void)Peer::flags();
    (void)Peer::isr();
    Peer::enable_rxc_interrupt(true);
    Peer::enable_txc_interrupt(false);
    Peer::enable_ssl_interrupt(false);
    Peer::release();
}

// ---- what this peripheral does NOT have ----------------------------------------
// 32.5.6 and 32.6.4.3 are both "Not applicable": the SPI publishes no
// EVSYS generator and consumes no user, which under the EVSYS ruling
// (samc/evsys.hpp - the fabric is that driver's, the vocabulary is each
// peripheral's) means there is no vocabulary to publish. There is
// nothing to assert about an absence, so this is a comment and not an
// assertion - the guard against inventing one is that spi.hpp names no
// event code at all. What the SERCOM DOES have, the two DMAC triggers,
// comes from Sercom<n> and is checked above.

