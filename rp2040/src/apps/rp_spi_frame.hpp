// rp_spi_frame.hpp - spi.frame, THE POLLED LOOPS' FRAME AGAINST THE WIRE, the
// table docs/rp2040/spi.md and docs/rp2350/spi.md quote, written once over the
// ARM PL022 (brio/pl022/spi.hpp) for letter e of bench_rp2040 and bench_rp2350.
//
// What the including bench states before it includes this file, inside its own
// anonymous namespace with `using namespace brio` in force:
//
//   Plain        SpiHost<0, {18, 19, 16}> with no engines, in the bench's
//                placement, already init()ed
//   SpiSelect    its select
//   Ruler, serial
//
// The table: every named divisor, /2 to /256, 8- and 16-bit frames, modes 0 and
// 1 (SPH = 0 and SPH = 1), the receive shape (an in buffer) and a request with
// none (at /2 on 8-bit frames the transmit-only shape, elsewhere the receive
// shape discarding). MISO floats and nothing is judged but the status, the
// overrun flag and the time: the frame is the block's and the loop's own.
//
// A FRAME IS THE DIFFERENCE OF TWO LENGTHS, so the request's fixed part cancels.
// Both lengths are past the FIFO, where the wall is linear in the frames: the
// long one is 256, or what fits 100 000 cycles where 256 would not, the short
// one 16, or 12 where the long one is under 32. Why 100 000: SysTick's period
// is 125 000 cycles on the RP2040 at 125 MHz and the ruler there composes the
// tick count with SysTick's position (cortexm/ticker.hpp), the pending bit
// telling one reload under the mask and no more; a request's few hundred fixed
// cycles and 100 000 of frames stay inside one reload. Each the least
// of five runs after one thrown away (the first applies the rate), each run
// under the mask - no handler in the figure, and a polled request needs no
// vector - and nothing is printed until the whole table is measured.
//
// What a line says: `wire` the frame's SCK periods in core cycles, `gap` the
// block's 1.5 periods between the frames of a continuous transfer where SPH = 0,
// `frame` the measured cycles a frame, `over` what the frame takes beyond wire
// and gap: the loop's own, zero where the wire sets the pace.

#pragma once

namespace rp_spi_frame {

inline constexpr uint8_t codes = 8;   ///< /2 .. /256
inline constexpr SpiClock clocks[codes] = {SpiClocks::div2,  SpiClocks::div4,  SpiClocks::div8,   SpiClocks::div16,
                                           SpiClocks::div32, SpiClocks::div64, SpiClocks::div128, SpiClocks::div256};
inline constexpr uint32_t longest_cycles = 100'000u;

/// A frame's SCK periods and its gap, in half core cycles, at a divisor.
constexpr uint32_t frame_halves(uint32_t divisor, uint32_t ratio, bool wide, bool sph) {
    return (2u * (wide ? 16u : 8u) + (sph ? 0u : 3u)) * divisor * ratio;
}

constexpr uint16_t long_frames(uint32_t divisor, uint32_t ratio, bool wide, bool sph) {
    const uint32_t n = 2u * longest_cycles / frame_halves(divisor, ratio, wide, sph);
    return static_cast<uint16_t>(n < 256u ? n : 256u);
}
constexpr uint16_t short_frames(uint16_t long_n) { return long_n < 32u ? 12u : 16u; }

// Every row keeps two lengths past the FIFO, and the slowest fits the reload.
static_assert(long_frames(256u, 1u, true, false) == 22u && short_frames(22u) == 12u);
static_assert(long_frames(2u, 1u, false, true) == 256u && short_frames(256u) == 16u);

struct Row {
    uint32_t least[2];   ///< the short and the long length's least wall
    uint16_t frames[2];
};
/// [the shape][the width][the mode][the code]: BSS, so the table prints after.
inline Row rows[2][2][2][codes];
inline uint32_t bad_status = 0;
inline uint32_t overruns = 0;

/// The least wall of `frames` at this configuration: five masked runs after
/// one thrown away.
inline uint32_t least_wall(const uint8_t* out, uint8_t* in, SpiClock c, SpiDataSize bits, SpiMode m, uint16_t frames) {
    Plain::Request r{};
    r.cs = SpiSelect::ref();
    r.tx = lend<Lease::reply>(out);
    r.rx = lend<Lease::reply>(in);
    r.len = frames;
    r.clock = c;
    r.mode = m;
    r.bits = bits;
    r.polled = true;
    uint32_t least = 0xFFFF'FFFFu;
    for (uint8_t run = 0; run < 6u; ++run) {
        Plain::Resource::clear_pending(SpiInterrupt::overrun);
        disable_interrupts();
        const uint32_t t0 = Ruler::now();
        (void)Plain::start(r);
        const uint32_t wall = Ruler::now() - t0;
        enable_interrupts();
        if (Plain::status() != spi_ok) {
            ++bad_status;
        }
        if ((Plain::Resource::raw_pending() & SpiInterrupt::overrun) != 0u) {
            ++overruns;
        }
        if (run != 0u && wall < least) {
            least = wall;
        }
    }
    return least;
}

/// Measured first, printed after. `out` and `in` hold 512 bytes each, and the
/// console has fallen silent before the call.
inline void measure_and_print(const uint8_t* out, uint8_t* in) {
    const uint32_t ratio = Ruler::hz() / Plain::reference_hz();
    bad_status = 0;
    overruns = 0;
    for (uint8_t shape = 0; shape < 2u; ++shape) {
        for (uint8_t w = 0; w < 2u; ++w) {
            for (uint8_t m = 0; m < 2u; ++m) {
                for (uint8_t c = 0; c < codes; ++c) {
                    Row& row = rows[shape][w][m][c];
                    const uint16_t lon = long_frames(clocks[c].divisor(), ratio, w != 0u, m != 0u);
                    row.frames[0] = short_frames(lon);
                    row.frames[1] = lon;
                    for (uint8_t l = 0; l < 2u; ++l) {
                        row.least[l] = least_wall(out, shape == 0u ? in : nullptr, clocks[c],
                                                  w != 0u ? SpiDataSize::bits16 : SpiDataSize::bits8,
                                                  m != 0u ? SpiMode::mode1 : SpiMode::mode0, row.frames[l]);
                    }
                }
            }
        }
    }
    print(serial, "  spi.frame: core cycles a frame, the least of five under the mask; a bad status ", bad_status,
          " times, the overrun flag raised ", overruns, " times", crlf);
    for (uint8_t shape = 0; shape < 2u; ++shape) {
        for (uint8_t w = 0; w < 2u; ++w) {
            for (uint8_t m = 0; m < 2u; ++m) {
                for (uint8_t c = 0; c < codes; ++c) {
                    const Row& row = rows[shape][w][m][c];
                    const uint32_t divisor = clocks[c].divisor();
                    const uint32_t wire = (w != 0u ? 16u : 8u) * divisor * ratio;
                    const uint32_t gap = m != 0u ? 0u : 3u * divisor * ratio / 2u;
                    const uint32_t span = row.frames[1] - row.frames[0];
                    const uint32_t per = (row.least[1] - row.least[0]) * 100u / span;
                    const int32_t over = static_cast<int32_t>(per) - static_cast<int32_t>((wire + gap) * 100u);
                    const uint32_t abs_over = over < 0 ? static_cast<uint32_t>(-over) : static_cast<uint32_t>(over);
                    print(serial, "  spi.frame ", shape == 0u ? "rx" : "tx", " bits", w != 0u ? 16u : 8u, " mode", m,
                          " div", divisor, " wire ", wire, " gap ", gap, " frame ", per / 100u, '.',
                          per % 100u < 10u ? "0" : "", per % 100u, " over ", over < 0 ? "-" : "", abs_over / 100u, '.',
                          abs_over % 100u < 10u ? "0" : "", abs_over % 100u, " of ", row.frames[0], " and ",
                          row.frames[1], " walls ", row.least[0], " ", row.least[1], crlf);
                }
            }
        }
    }
}

}   // namespace rp_spi_frame
