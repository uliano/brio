/*
 * sim_display.hpp (host)
 *
 * A PANEL THAT SOMETHING ELSE SCANS OUT. On a part with a display
 * controller the program writes pixels into memory and a peripheral
 * reads that memory on its own, without being asked and without the
 * program ever waiting. This publishes a framebuffer into POSIX shared
 * memory so a viewer in another process can do exactly that - read the
 * same physical pages, at its own rate, tearing where a real panel would
 * tear. It is a faithful model of the memory-mapped tier and not an
 * imitation of one.
 *
 * THIS IS NOT A SURFACE. It owns the mapping and publishes what the
 * mapping holds; the drawing surface is an ordinary Framebuffer over the
 * bytes it hands out. So nothing in gfx/ knows that its pixels live in
 * shared memory, and the same library code runs against a plain array in
 * a test and against a viewer's window here.
 *
 * The naming, the sizing and the boot id are host/shared_segment.hpp's,
 * because the inputs travel the same way in the other direction and the
 * rules that make both portable belong in one place.
 *
 * Host only, and free with the whole standard library: failures throw,
 * because a bench tool that cannot map its own memory has nothing useful
 * left to do (docs/host/simulator.md).
 */

#pragma once

#include <stdint.h>
#include <string.h>

#include <span>
#include <string>

#include "gfx/surface.hpp"
#include "host/shared_segment.hpp"

namespace brio {

/// The pixel format as a byte in the header, so a viewer written in
/// another language knows what it is looking at without guessing.
template <typename Fmt>
struct SimPixelCode;

template <>
struct SimPixelCode<Mono> {
    static constexpr uint8_t value = 1; ///< 1 bpp, row-major, MSB leftmost
};

template <>
struct SimPixelCode<Indexed8> {
    static constexpr uint8_t value = 8; ///< 1 byte a pixel, palette or grey
};

/**
 * What sits at the front of the segment. Fixed layout, little-endian
 * (every target brio has is), and SELF-DESCRIBING: a viewer reads this
 * and needs no arguments of its own.
 *
 * `frame` is bumped by publish(); a viewer that repaints only when it
 * moves spends nothing on a still picture. `buffers` and `front` are
 * one and zero today - the fields exist because double buffering is the
 * answer to tearing when tearing stops being acceptable, and adding them
 * later would change the layout under every viewer already written.
 */
struct SimDisplayHeader {
    char magic[4];          ///< "BRGX"
    uint16_t version;       ///< of this layout
    uint16_t header_bytes;  ///< where the pixels start
    uint64_t boot_id;       ///< fresh at every creation; see above
    uint16_t width;
    uint16_t height;
    uint16_t stride;        ///< bytes per row
    uint8_t format;         ///< SimPixelCode
    uint8_t buffers;        ///< 1 today
    uint8_t front;          ///< 0 today
    uint8_t reserved[3];
    uint32_t frame;         ///< bumped by publish()
    uint16_t palette_used;  ///< entries the format actually reads
    uint8_t palette[256][3]; ///< red, green, blue
};

/// The pixels start here, whatever the header grows to inside it.
inline constexpr size_t sim_display_header_bytes = 1024;

static_assert(sizeof(SimDisplayHeader) <= sim_display_header_bytes);

/// The bytes a segment holds by its own account - the header and the
/// pixel planes - which is what a viewer reads. It is NOT the size the
/// operating system reports for the object: macOS rounds that up to a
/// page (host/shared_segment.hpp), so the count comes from the header
/// the program wrote, and the object's size only bounds what may be
/// mapped.
inline size_t sim_display_bytes(const SimDisplayHeader& h) {
    return static_cast<size_t>(h.header_bytes) +
           static_cast<size_t>(h.stride) * h.height * h.buffers;
}

/**
 * Creates a segment, owns it, and hands out the bytes a Framebuffer
 * draws into. Destroying it unlinks the name: a viewer holding the old
 * mapping keeps seeing the last picture until it re-opens and finds the
 * boot id changed.
 */
template <typename Fmt, Extent W, Extent H>
class SimDisplay {
public:
    using Format = Fmt;
    using Surface = Framebuffer<Fmt, W, H>;

    static constexpr uint16_t stride = Fmt::stride_for(W);
    static constexpr size_t pixel_bytes = static_cast<size_t>(stride) * H;
    static constexpr size_t total_bytes = sim_display_header_bytes + pixel_bytes;

    /// `name` is the whole contract between the two processes. It is
    /// prefixed and length-checked so that what works here works where
    /// the cap is 31 characters.
    explicit SimDisplay(const std::string& name)
        : seg_("/brio-gfx-", name, total_bytes) {
        write_header();
    }

    SimDisplay(const SimDisplay&) = delete;
    SimDisplay& operator=(const SimDisplay&) = delete;

    /// The drawing surface over the mapped pixels. An ordinary
    /// Framebuffer: gfx/ never learns where these bytes live.
    Surface surface() {
        return Surface(std::span<uint8_t, pixel_bytes>(pixels(), pixel_bytes));
    }

    /// Say the picture changed, so a viewer repainting on change knows
    /// to. A viewer that repaints regardless - as a real panel scans
    /// regardless - is free to ignore it.
    void publish() { ++header()->frame; }

    /// The colours the format's values stand for, which is the whole of
    /// what an observer needs to show a panel as it really looks: white
    /// on blue, black on green, a grey ramp, or an application's own.
    void set_palette(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
        if (index >= 256) {
            return;
        }
        header()->palette[index][0] = r;
        header()->palette[index][1] = g;
        header()->palette[index][2] = b;
    }

    const std::string& name() const { return seg_.name(); }
    uint64_t boot_id() const { return header()->boot_id; }
    uint32_t frame() const { return header()->frame; }

private:
    SimDisplayHeader* header() {
        return reinterpret_cast<SimDisplayHeader*>(seg_.base());
    }
    const SimDisplayHeader* header() const {
        return reinterpret_cast<const SimDisplayHeader*>(seg_.base());
    }
    uint8_t* pixels() { return seg_.base() + sim_display_header_bytes; }

    void write_header() {
        SimDisplayHeader* h = header();
        memcpy(h->magic, "BRGX", 4);
        h->version = 1;
        h->header_bytes = static_cast<uint16_t>(sim_display_header_bytes);
        h->boot_id = fresh_boot_id();
        h->width = W;
        h->height = H;
        h->stride = stride;
        h->format = SimPixelCode<Fmt>::value;
        h->buffers = 1;
        h->front = 0;
        h->frame = 0;
        // A sensible default so a viewer shows something before an
        // application says otherwise: black on white for one bit, a grey
        // ramp for a byte.
        if constexpr (SimPixelCode<Fmt>::value == 1) {
            h->palette_used = 2;
            h->palette[0][0] = h->palette[0][1] = h->palette[0][2] = 0xFF;
            h->palette[1][0] = h->palette[1][1] = h->palette[1][2] = 0x00;
        } else {
            h->palette_used = 256;
            for (int i = 0; i < 256; ++i) {
                h->palette[i][0] = static_cast<uint8_t>(i);
                h->palette[i][1] = static_cast<uint8_t>(i);
                h->palette[i][2] = static_cast<uint8_t>(i);
            }
        }
    }

    SharedSegment seg_;
};

} // namespace brio
