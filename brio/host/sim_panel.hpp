/*
 * sim_panel.hpp (host)
 *
 * The world's side of a front panel, shared with whoever is watching:
 * the contacts a viewer presses and the shafts it turns, read by the
 * program in its idle path and handed to the simulated devices.
 *
 * THE SAME MECHANISM AS THE FRAMEBUFFER, POINTING THE OTHER WAY, and
 * that is the whole design. A socket was the obvious answer and turned
 * out to be the wrong one: an input is a small SNAPSHOT OF STATE, not a
 * stream of gestures, and a shared segment gives for free everything a
 * datagram protocol would have had to specify. There is nothing to frame
 * and nothing to reassemble - a reader takes the current values.
 * Latest-wins is not a drain rule but the only thing that can happen.
 * "What if no viewer is attached" does not arise, because the PROGRAM
 * owns the segment and it reads as nothing-pressed until someone writes.
 * And nobody has to decide which side binds and which connects.
 *
 * The one thing a socket would have added is a notification, and the
 * program does not want one: it polls in idle(), which is what a pad is
 * for (design/gfx.md's input note - nothing pushes into the kernel,
 * because nothing pushes into it on silicon either).
 *
 * A SHAFT IS AN ABSOLUTE COUNT, not a delta, for the same reason the
 * contacts are levels: a snapshot that is lost costs nothing, because
 * the next one carries the whole truth. The viewer owns the count and
 * the program mirrors it.
 *
 * AND THE MIRROR SNAPS RATHER THAN WALKS. Setting the pads straight to
 * where the shaft now is - rather than stepping them through everything
 * it passed - is not a shortcut: it is what a real decoder sees. The
 * pads are sampled once a tick, so whatever the shaft did in between is
 * invisible except through where it ended up. A shaft that moved three
 * counts between two samples looks like one count backwards on real
 * hardware too, and a simulator that hid that would be hiding the one
 * failure this decoder has (util/quadrature.hpp).
 *
 * (docs/host/simulator.md.)
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <string>

#include "host/shared_segment.hpp"

namespace brio {

/// How many of each a panel may carry, and how long a name may be.
/// Fixed, so the segment is one size and a viewer needs no arithmetic to
/// find any of the arrays.
inline constexpr uint8_t sim_panel_buttons = 8;
inline constexpr uint8_t sim_panel_shafts = 4;
inline constexpr uint8_t sim_panel_name_max = 16;

/**
 * What sits in the segment. Fixed layout, little-endian, self-describing.
 *
 * TWO WRITERS, AND THEY NEVER OVERLAP. The viewer owns what the world is
 * DOING - the contacts and the shafts - and the program owns what the
 * panel IS, which is the names. Neither ever writes the other's fields,
 * so there is nothing to arbitrate.
 *
 * `seq` is the viewer's to bump when it changes anything; a program that
 * wants to know whether the world moved at all can watch it instead of
 * comparing every slot.
 */
struct SimPanelState {
    char magic[4];   ///< "BRIP"
    uint16_t version;
    uint16_t header_bytes;
    uint64_t boot_id;
    uint8_t buttons; ///< slots that mean anything
    uint8_t shafts;
    uint8_t reserved[6];
    uint32_t seq;                          ///< bumped by the viewer
    uint8_t pressed[sim_panel_buttons];    ///< 0 or 1, ACTIVE-true
    int32_t shaft[sim_panel_shafts];       ///< absolute quadrature counts

    /// What each control IS, written by the program. A viewer showing a
    /// grid of "button 3" and "shaft 1" is unusable past two controls,
    /// and WHICH CONTACT IS WHICH IS A FACT ABOUT THE PANEL - so it
    /// comes from the board file that describes the panel, like every
    /// other such fact. Not null-terminated of necessity: a reader takes
    /// at most the whole field.
    char button_name[sim_panel_buttons][sim_panel_name_max];
    char shaft_name[sim_panel_shafts][sim_panel_name_max];
};

/// Where the state begins, whatever the structure grows to inside it.
inline constexpr size_t sim_panel_bytes = 512;

static_assert(sizeof(SimPanelState) <= sim_panel_bytes);

/**
 * The program's end: it creates the segment, and reads it.
 *
 * Nothing here touches the simulated devices, because WHICH contact is
 * which button is a fact about the panel and belongs where the panel is
 * described - the board file wires `pressed(0)` to its own SimButton and
 * `shaft(0)` to its own SimEncoder, exactly as a target's board file
 * wires a pin.
 */
class SimPanel {
public:
    explicit SimPanel(const std::string& name)
        : seg_("/brio-in-", name, sim_panel_bytes) {
        SimPanelState* s = state();
        memcpy(s->magic, "BRIP", 4);
        s->version = 1;
        s->header_bytes = static_cast<uint16_t>(sim_panel_bytes);
        s->boot_id = fresh_boot_id();
        s->buttons = sim_panel_buttons;
        s->shafts = sim_panel_shafts;
        s->seq = 0;
    }

    /// Is contact `i` held down? Out of range reads as released, which
    /// is what an absent contact is.
    bool pressed(uint8_t i) const {
        return i < sim_panel_buttons && state()->pressed[i] != 0;
    }

    /// Where shaft `i` stands, in quadrature counts. Out of range reads
    /// as nought.
    int32_t shaft(uint8_t i) const {
        return i < sim_panel_shafts ? state()->shaft[i] : 0;
    }

    /// Say what a control is. The viewer shows it; nothing else reads
    /// it. A name longer than the field is cut rather than refused -
    /// this is a label and not an identifier.
    void name_button(uint8_t i, const std::string& label) {
        write_name(i < sim_panel_buttons ? state()->button_name[i] : nullptr,
                   label);
    }
    void name_shaft(uint8_t i, const std::string& label) {
        write_name(i < sim_panel_shafts ? state()->shaft_name[i] : nullptr,
                   label);
    }

    /// What a control was called, as a string a viewer can show.
    std::string button_name(uint8_t i) const {
        return i < sim_panel_buttons ? read_name(state()->button_name[i])
                                     : std::string();
    }
    std::string shaft_name(uint8_t i) const {
        return i < sim_panel_shafts ? read_name(state()->shaft_name[i])
                                    : std::string();
    }

    /// What the viewer has changed since the program started. A reader
    /// that has seen this number has seen everything.
    uint32_t seq() const { return state()->seq; }

    uint64_t boot_id() const { return state()->boot_id; }
    const std::string& name() const { return seg_.name(); }

private:
    static void write_name(char* field, const std::string& label) {
        if (field == nullptr) {
            return;
        }
        memset(field, 0, sim_panel_name_max);
        const size_t n = label.size() < sim_panel_name_max
                             ? label.size()
                             : static_cast<size_t>(sim_panel_name_max);
        memcpy(field, label.data(), n);
    }

    static std::string read_name(const char* field) {
        size_t n = 0;
        while (n < sim_panel_name_max && field[n] != '\0') {
            ++n;
        }
        return std::string(field, n);
    }

    SimPanelState* state() {
        return reinterpret_cast<SimPanelState*>(seg_.base());
    }
    const SimPanelState* state() const {
        return reinterpret_cast<const SimPanelState*>(seg_.base());
    }

    SharedSegment seg_;
};

} // namespace brio
