/*
 * sim_usb.hpp
 *
 * A UsbController (util/usb/device.hpp) made of RAM: the host tests
 * play the HOST against it, one packet at a time, and read back what
 * the device answered. It models what a controller shows the stack -
 * one packet offered per endpoint and direction, the setup packet,
 * the reset, the address, the stall bits, the completion events - and
 * nothing of the wire.
 *
 * THE TEST'S VERBS (the host's side): `host_setup(packet)` delivers a
 * setup packet (the controller marks the event); `host_in(number)`
 * takes the packet the device offered on an IN endpoint, if any
 * (empty optional = NAK: nothing was offered); `host_out(number,
 * bytes)` delivers a packet on an OUT endpoint if one was armed
 * (false = NAK); `host_reset()`. After each, the test runs the
 * stack's isr() as the vector would. A control transfer is then a
 * script of these: setup, a few host_in, a host_out of zero bytes as
 * the status stage.
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <array>
#include <optional>
#include <span>
#include <vector>

#include "util/usb/device.hpp"

namespace brio {

struct SimUsb {
    SimUsb() = delete;

    static constexpr uint8_t endpoint_count = 16;

    struct Endpoint {
        bool configured = false;
        UsbEndpointType type = UsbEndpointType::control;
        uint16_t max = 64;
        bool stalled = false;
        bool offered = false;       // IN: a packet waits for the host; OUT: armed for the host's packet
        std::vector<uint8_t> data;  // IN: the offered packet; OUT: the received one
        uint16_t out_max = 0;
    };

    // ---- the UsbController contract ---------------------------------------------------
    static void connect(bool on) { pulled_up = on; }
    static void set_address(uint8_t a) { address = a; }
    static bool configure_endpoint(uint8_t addr, UsbEndpointType type, uint16_t max) {
        const uint8_t n = usb_ep_number(addr);
        if (n == 0u || n >= endpoint_count || max > 64u || refuse_configure) {
            return false;
        }
        Endpoint& e = ep(addr);
        e = Endpoint{};
        e.configured = true;
        e.type = type;
        e.max = max;
        ++configured_count;
        return true;
    }
    static void deconfigure_endpoints() {
        for (uint8_t n = 1; n < endpoint_count; ++n) {
            in_[n] = Endpoint{};
            out_[n] = Endpoint{};
        }
        ++deconfigures;
    }
    static void stall(uint8_t addr, bool on) {
        ep(addr).stalled = on;
        if (on) {
            ++stalls;
        }
    }
    static bool stalled(uint8_t addr) { return ep(addr).stalled; }
    static bool submit_in(uint8_t n, std::span<const uint8_t> data) {
        if (n >= endpoint_count || (n != 0u && !in_[n].configured) || data.size() > 64u) {
            return false;
        }
        Endpoint& e = in_[n];
        if (e.offered) {
            ++double_submits;
            return false;
        }
        e.data.assign(data.begin(), data.end());
        e.offered = true;
        return true;
    }
    static bool submit_out(uint8_t n, uint16_t max) {
        if (n >= endpoint_count || (n != 0u && !out_[n].configured) || max > 64u) {
            return false;
        }
        Endpoint& e = out_[n];
        if (e.offered) {
            ++double_submits;
            return false;
        }
        e.offered = true;
        e.out_max = max;
        return true;
    }
    static std::span<const uint8_t> out_data(uint8_t n) { return {out_[n].data.data(), out_[n].data.size()}; }
    static UsbSetup setup() { return last_setup; }
    static UsbEvents take_events() {
        const UsbEvents ev = pending;
        pending = UsbEvents{};
        return ev;
    }

    // ---- the host's side ---------------------------------------------------------------
    static void host_reset() {
        pending.reset = true;
        for (uint8_t n = 0; n < endpoint_count; ++n) {
            in_[n].offered = false;
            out_[n].offered = false;
        }
    }
    static void host_setup(const UsbSetup& s) {
        last_setup = s;
        pending.setup = true;
        in_[0].offered = false;   // a setup packet closes what was pending on endpoint zero
        out_[0].offered = false;
        in_[0].stalled = false;
        out_[0].stalled = false;
    }
    /// The host sends an IN token: the offered packet, or nothing (a NAK).
    static std::optional<std::vector<uint8_t>> host_in(uint8_t n) {
        Endpoint& e = in_[n];
        if (e.stalled) {
            ++host_saw_stalls;
            return std::nullopt;
        }
        if (!e.offered) {
            ++naks;
            return std::nullopt;
        }
        e.offered = false;
        pending.in_done |= static_cast<uint16_t>(1u << n);
        return e.data;
    }
    /// The host sends an OUT packet: taken when the endpoint is armed.
    static bool host_out(uint8_t n, std::span<const uint8_t> bytes) {
        Endpoint& e = out_[n];
        if (e.stalled) {
            ++host_saw_stalls;
            return false;
        }
        if (!e.offered || bytes.size() > e.out_max) {
            ++naks;
            return false;
        }
        e.offered = false;
        e.data.assign(bytes.begin(), bytes.end());
        pending.out_done |= static_cast<uint16_t>(1u << n);
        return true;
    }
    static void host_suspend() { pending.suspend = true; }
    static void host_resume() { pending.resume = true; }

    static Endpoint& ep(uint8_t addr) { return usb_ep_is_in(addr) ? in_[usb_ep_number(addr)] : out_[usb_ep_number(addr)]; }
    static bool in_offered(uint8_t n) { return in_[n].offered; }
    static bool out_armed(uint8_t n) { return out_[n].offered; }

    static void clear() {
        for (uint8_t n = 0; n < endpoint_count; ++n) {
            in_[n] = Endpoint{};
            out_[n] = Endpoint{};
        }
        pending = UsbEvents{};
        pulled_up = false;
        address = 0;
        stalls = naks = host_saw_stalls = double_submits = configured_count = deconfigures = 0;
        refuse_configure = false;
    }

    static Endpoint in_[endpoint_count];
    static Endpoint out_[endpoint_count];
    static inline UsbEvents pending{};
    static inline UsbSetup last_setup{};
    static inline bool pulled_up = false;
    static inline uint8_t address = 0;
    static inline uint32_t stalls = 0;
    static inline uint32_t naks = 0;
    static inline uint32_t host_saw_stalls = 0;
    static inline uint32_t double_submits = 0;
    static inline uint32_t configured_count = 0;
    static inline uint32_t deconfigures = 0;
    static inline bool refuse_configure = false;
};

inline SimUsb::Endpoint SimUsb::in_[SimUsb::endpoint_count]{};
inline SimUsb::Endpoint SimUsb::out_[SimUsb::endpoint_count]{};

static_assert(UsbController<SimUsb>);

} // namespace brio
