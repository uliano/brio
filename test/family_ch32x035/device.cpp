// Device map family smoke TU: every register struct of
// brio/ch32x035/device.hpp at the offsets the reference manual gives (RM
// 3.4, 7.5, 8.3, 14.10, 19.2, 20.3), the part table's facts consistent
// with each other on every part, and the read-only decoders of the flash
// option bytes and the electronic signature.
#include <stddef.h>

#include "ch32x035/device.hpp"

using namespace brio;

// ---- the offsets ------------------------------------------------------------
static_assert(offsetof(RccRegs, CTLR) == 0x00);
static_assert(offsetof(RccRegs, CFGR0) == 0x04);
static_assert(offsetof(RccRegs, APB2PRSTR) == 0x0c);
static_assert(offsetof(RccRegs, APB1PRSTR) == 0x10);
static_assert(offsetof(RccRegs, AHBPCENR) == 0x14);
static_assert(offsetof(RccRegs, APB2PCENR) == 0x18);
static_assert(offsetof(RccRegs, APB1PCENR) == 0x1c);
static_assert(offsetof(RccRegs, RSTSCKR) == 0x24);
static_assert(offsetof(RccRegs, AHBRSTR) == 0x28);

static_assert(offsetof(GpioRegs, CFGLR) == 0x00);
static_assert(offsetof(GpioRegs, CFGHR) == 0x04);
static_assert(offsetof(GpioRegs, INDR) == 0x08);
static_assert(offsetof(GpioRegs, OUTDR) == 0x0c);
static_assert(offsetof(GpioRegs, BSHR) == 0x10);
static_assert(offsetof(GpioRegs, BCR) == 0x14);
static_assert(offsetof(GpioRegs, LCKR) == 0x18);
static_assert(offsetof(GpioRegs, CFGXR) == 0x1c);
static_assert(offsetof(GpioRegs, BSXR) == 0x20);

static_assert(offsetof(AfioRegs, PCFR1) == 0x04);
static_assert(offsetof(AfioRegs, EXTICR) == 0x08);
static_assert(offsetof(AfioRegs, CTLR) == 0x18);

static_assert(offsetof(ExtiRegs, INTENR) == 0x00);
static_assert(offsetof(ExtiRegs, INTFR) == 0x14);

static_assert(offsetof(UsartRegs, STATR) == 0x00);
static_assert(offsetof(UsartRegs, DATAR) == 0x04);
static_assert(offsetof(UsartRegs, BRR) == 0x08);
static_assert(offsetof(UsartRegs, CTLR1) == 0x0c);
static_assert(offsetof(UsartRegs, CTLR2) == 0x10);
static_assert(offsetof(UsartRegs, CTLR3) == 0x14);
static_assert(offsetof(UsartRegs, GPR) == 0x18);

static_assert(offsetof(PwrRegs, CSR) == 0x04);

static_assert(offsetof(FlashRegs, ACTLR) == 0x00);
static_assert(offsetof(FlashRegs, STATR) == 0x0c);
static_assert(offsetof(FlashRegs, CTLR) == 0x10);
static_assert(offsetof(FlashRegs, OBR) == 0x1c);
static_assert(offsetof(FlashRegs, WPR) == 0x20);
static_assert(offsetof(FlashRegs, BOOT_MODEKEYR) == 0x28);

static_assert(offsetof(StkRegs, CTLR) == 0x00);
static_assert(offsetof(StkRegs, CNTL) == 0x08);
static_assert(offsetof(StkRegs, CMPLR) == 0x10);
static_assert(offsetof(StkRegs, CMPHR) == 0x14);

static_assert(offsetof(PficRegs, ISR) == 0x000);
static_assert(offsetof(PficRegs, IPR) == 0x020);
static_assert(offsetof(PficRegs, ITHRESDR) == 0x040);
static_assert(offsetof(PficRegs, CFGR) == 0x048);
static_assert(offsetof(PficRegs, GISR) == 0x04c);
static_assert(offsetof(PficRegs, VTFIDR) == 0x050);
static_assert(offsetof(PficRegs, VTFADDRR) == 0x060);
static_assert(offsetof(PficRegs, IENR) == 0x100);
static_assert(offsetof(PficRegs, IRER) == 0x180);
static_assert(offsetof(PficRegs, IPSR) == 0x200);
static_assert(offsetof(PficRegs, IPRR) == 0x280);
static_assert(offsetof(PficRegs, IACTR) == 0x300);
static_assert(offsetof(PficRegs, IPRIOR) == 0x400);

// ---- the addresses (RM 1.2) -------------------------------------------------
static_assert(gpio_base_for('A') == 0x40010800UL);
static_assert(gpio_base_for('C') == 0x40011000UL);
static_assert(gpio_base_for('D') == 0);
static_assert(usart_base_for(1) == 0x40013800UL);
static_assert(usart_base_for(2) == 0x40004400UL);
static_assert(usart_base_for(4) == 0x40004c00UL);
static_assert(usart_base_for(5) == 0);

// ---- the part's facts, consistent -----------------------------------------------
static_assert(device::flash_bytes == 62u * 1024u && device::sram_bytes == 20u * 1024u);
static_assert(device::flash_page_bytes == 256u);
constexpr unsigned bits(uint32_t v) { return static_cast<unsigned>(__builtin_popcount(v)); }
// The bonded pads are pads of the die; the twinned ones are bonded.
static_assert((device::port_pins('A') & ~gpio_die_pads('A')) == 0u);
static_assert((device::port_pins('B') & ~gpio_die_pads('B')) == 0u);
static_assert((device::port_pins('C') & ~gpio_die_pads('C')) == 0u);
static_assert((device::port_twinned('A') & ~device::port_pins('A')) == 0u);
static_assert((device::port_twinned('B') & ~device::port_pins('B')) == 0u);
static_assert((device::port_twinned('C') & ~device::port_pins('C')) == 0u);
// The model table's GPIO count is the PINS: a twinned pair is one.
static_assert(bits(device::port_pins('A')) + bits(device::port_pins('B')) + bits(device::port_pins('C')) -
                  (bits(device::port_twinned('A')) + bits(device::port_twinned('B')) +
                   bits(device::port_twinned('C'))) / 2u ==
              device::gpio_count);
// Every part bonds the debug port's two pads and USART2's default pair.
static_assert((device::port_pins('C') & (3UL << 18)) == (3UL << 18));
static_assert((device::port_pins('A') & 0x0CUL) == 0x0CUL);
static_assert(device::has_usart(2));
static_assert(device::usart_count >= bits(device::usart_instances));
static_assert(vector_count == 55);
static_assert(static_cast<uint8_t>(Irq::tim3) + 1u == vector_count);

// ---- the read-only decoders ---------------------------------------------------------
static_assert(flash_latency_for(8'000'000) == 0);
static_assert(flash_latency_for(12'000'000) == 0);
static_assert(flash_latency_for(16'000'000) == 1);
static_assert(flash_latency_for(24'000'000) == 1);
static_assert(flash_latency_for(48'000'000) == 2);
constexpr FlashOptionBits ob = flash_option_bits(0x03FF0306UL);
static_assert(!ob.option_error && ob.read_protected && ob.iwdg_software && !ob.stop_reset &&
              !ob.standby_reset && ob.reset_mode == 0u && ob.data0 == 0xC0u && ob.data1 == 0xFFu);

void signature_reads() {
    (void)esig_flash_kbytes();
    (void)esig_uid_word(0);
    (void)esig_uid_word(2);
    (void)chip_id_word();
    (void)flash_options();
    (void)flash_write_protection();
    (void)device::part_name;
}
