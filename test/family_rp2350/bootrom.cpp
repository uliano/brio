// Bootrom family smoke TU: the fixed words of 5.4.1 pinned, the
// two-character codes of 5.4.7 computed the way the datasheet computes
// them, the architecture-dependent lookup checked to pick the RIGHT
// helper and flag on THIS build, every wrapper instantiated - and the
// OTP programming entry point asserted absent.
#include <array>

#include "rp2350/bootrom.hpp"

using namespace brio;

// ---- the fixed words (5.4.1, tables 453 and 454) --------------------------

static_assert(BootromAddress::magic == 0x10u);
static_assert(BootromAddress::version == 0x13u);
static_assert(BootromAddress::table_arm == 0x14u);
static_assert(BootromAddress::lookup_value_arm == 0x16u);
static_assert(BootromAddress::lookup_entry_arm == 0x18u);
static_assert(BootromAddress::table_riscv == 0x7df6u);
static_assert(BootromAddress::lookup_value_riscv == 0x7df8u);
static_assert(BootromAddress::lookup_entry_riscv == 0x7dfau);
static_assert(BootromAddress::entry_riscv == 0x7dfcu);
// Each set is three consecutive halfword cells.
static_assert(BootromAddress::lookup_value_arm - BootromAddress::table_arm == 2);
static_assert(BootromAddress::lookup_entry_arm - BootromAddress::lookup_value_arm == 2);
static_assert(BootromAddress::lookup_value_riscv - BootromAddress::table_riscv == 2);
static_assert(BootromAddress::entry_riscv - BootromAddress::lookup_entry_riscv == 2);
// 'M', 'u', 0x02 read as one little-endian 24-bit value.
static_assert(bootrom_magic == 0x02754Du);

// ---- the codes (5.4.7) ----------------------------------------------------

static_assert(bootrom_code('G', 'S') == 0x5347u);
static_assert(BootromCode::get_sys_info == bootrom_code('G', 'S'));
static_assert(BootromCode::get_partition_table_info == bootrom_code('G', 'P'));
static_assert(BootromCode::git_revision == bootrom_code('G', 'R'));
static_assert(BootromCode::partition_table_ptr == bootrom_code('P', 'T'));

// ---- the flags, and which one THIS build uses ----------------------------

static_assert(BootromFlag::func_riscv == 0x0001u);
static_assert(BootromFlag::func_arm_secure == 0x0004u);
static_assert(BootromFlag::func_arm_non_secure == 0x0010u);
static_assert(BootromFlag::data == 0x0040u);

// The one place the architecture shows: a Secure Arm lookup on one half,
// a RISC-V lookup on the other - decided by a CONSTANT and not by the
// preprocessor, and checked here on whichever compiler is running.
static_assert(core_kind == CoreKind::hazard3
                  ? Bootrom::function_flag == BootromFlag::func_riscv
                  : Bootrom::function_flag == BootromFlag::func_arm_secure);
// brio never asks for a Non-secure entry point: those must be enabled
// one by one by Secure code before they exist at all (5.4.2).
static_assert(Bootrom::function_flag != BootromFlag::func_arm_non_secure);

// ---- the return codes (5.4.3) --------------------------------------------

static_assert(BootromError::ok == 0);
static_assert(BootromError::not_permitted == -4);
static_assert(BootromError::precondition_not_met == -14);
static_assert(BootromError::not_found == -17);
static_assert(BootromError::lock_required == -19);

// ---- the two functions' flag words ---------------------------------------

static_assert(SysInfoFlag::chip_info == 0x0001u);
static_assert(SysInfoFlag::critical == 0x0002u);
static_assert(SysInfoFlag::cpu_info == 0x0004u);
static_assert(SysInfoFlag::flash_dev_info == 0x0008u);
static_assert(SysInfoFlag::boot_random == 0x0010u);
static_assert(SysInfoFlag::boot_info == 0x0040u);
// NONCE is not served by this silicon, so `all` leaves it out.
static_assert((SysInfoFlag::all & SysInfoFlag::nonce) == 0u);
// The words `all` can bring back: 1 + 3 + 1 + 1 + 1 + 4 + 4 = 15, which
// is what the decoder's buffer is sized for.
static_assert(PartitionInfoFlag::pt_info == 0x0001u);
static_assert(PartitionInfoFlag::single_partition == 0x8000u);

// ---- THE PROGRAMMING ENTRY POINT IS NOT WRAPPED --------------------------
//
// `otp_access` (5.4.8.21) is the ROM's OTP read AND WRITE call: its
// third argument carries an IS_WRITE bit. This project's rule is that no
// write path to OTP exists in the tree, so the call is not here at all
// and the compiler says so on every sweep.

template <typename T>
concept has_otp_access =
    requires(uint8_t* p) { T::otp_access(p, uint32_t{0}, uint32_t{0}); };
template <typename T>
concept has_reboot = requires { T::reboot(uint32_t{0}, uint32_t{0}, uint32_t{0}, uint32_t{0}); };

static_assert(!has_otp_access<Bootrom>,
              "brio Bootrom wraps no OTP programming call: OTP is read-only here");
static_assert(!has_reboot<Bootrom>,
              "brio Bootrom wraps no reboot: this stratum reboots through the watchdog");

// ---- the verbs ------------------------------------------------------------

void bootrom_lookup_verbs() {
    (void)Bootrom::present();
    (void)Bootrom::version();
    (void)Bootrom::table();
    (void)Bootrom::lookup_value();
    (void)Bootrom::lookup_entry();
    (void)Bootrom::lookup_function(BootromCode::get_sys_info);
    (void)Bootrom::lookup_data(BootromCode::git_revision);
    (void)Bootrom::git_revision();
}

void bootrom_call_verbs() {
    std::array<uint32_t, 16> out{};
    (void)Bootrom::get_sys_info(out.data(), static_cast<uint32_t>(out.size()),
                                SysInfoFlag::chip_info);
    (void)Bootrom::get_partition_table_info(out.data(), static_cast<uint32_t>(out.size()),
                                            PartitionInfoFlag::pt_info);

    const auto info = Bootrom::sys_info();
    if (info) {
        (void)info->served;
        (void)info->has(SysInfoFlag::chip_info);
        (void)info->package_sel;
        (void)info->device_id;
        (void)info->critical;
        (void)info->cpu;
        (void)info->flash_dev_info;
        (void)info->boot_random[3];
        (void)info->boot_info[3];
    }
    (void)Bootrom::sys_info(SysInfoFlag::cpu_info);
}
