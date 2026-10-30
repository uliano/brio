// twi_link_client.hpp - the CONTROLLER's end of twi_link.hpp: the command
// channel to a `twi_peer` on the bus under test, for any suite whose board
// is the bus controller, over whichever path to the wire the suite is
// measuring - a host engine driven directly, or a bus arbiter (util/
// i2c_bus.hpp) the tenures are posted to.
//
// APP-LEVEL BENCH TOOLING, not framework, kept beside the protocol header it
// speaks so that one reviewed copy of the request/response choreography
// serves every suite that drives a peer this way; nothing in brio knows it
// exists, and nothing here names a register. The wire format is
// twi_link.hpp beside it, included by relative path and not copied. A suite
// includes this file at file scope, before its own anonymous namespace.
//
// ONE COMMAND IS TWO TENURES of the path under test - a write carrying the
// frame, then a read of `response_bytes` collecting the answer - so the
// command channel is itself a measurement of that path. The peer acts only
// after the ack read completes (ack before act), and every action it runs
// ends at its own deadline, so a command that fails costs a deadline and
// never a power cycle. A failed attempt is followed by Port::ready() and
// 400 ms (the peer's own recovery bound) before the next; three attempts
// are made, and a command that succeeds is followed by twi_link.hpp's
// arm_ms before the caller's first measured edge.
//
// `Port` supplies the path, every function static:
//   uint8_t tenure(uint8_t addr, const uint8_t* tx, uint8_t tx_len,
//                  uint8_t* rx, uint8_t rx_len);   // one tenure, waited for: a brio status
//   void ready();             // the path back where a tenure is legal (a
//                             // host re-inited; nothing, where an arbiter's
//                             // timeout and recover() own that)
//   void wait_ms(uint32_t);   // a bounded wait, in the suite's own sense
//                             // of waiting (a spin, or a kernel's turns)
// The buffers passed to tenure() are this file's own locals, valid for the
// call; tenure() returns only when the tenure is over.

#pragma once

#include <stdint.h>

#include "util/i2c_bus.hpp"

#include "../../../avrxt/src/apps/twi_link.hpp"

template <typename Port>
struct TwiLinkClient {
    TwiLinkClient() = delete;

    using Op = twilink::Op;

    /// Write one frame to the command address: the first tenure.
    static bool send_frame(Op op, const uint8_t* p, uint8_t len) {
        uint8_t buf[twilink::max_payload + 4];
        uint8_t n = 0;
        twilink::write_frame(
            [&](uint8_t b) {
                if (n < sizeof buf) {
                    buf[n++] = b;
                }
            },
            op, p, len);
        return Port::tenure(twilink::command_addr, buf, n, nullptr, 0) == brio::i2c_ok;
    }

    /// Read one response and decode the first frame in it: the second.
    static bool recv_frame(twilink::Frame& out) {
        uint8_t buf[twilink::response_bytes] = {};
        if (Port::tenure(twilink::command_addr, nullptr, 0, buf, twilink::response_bytes) !=
            brio::i2c_ok) {
            return false;
        }
        twilink::Decoder dec;
        for (uint8_t i = 0; i < twilink::response_bytes; ++i) {
            if (dec.feed(buf[i]) == twilink::Decoder::Result::frame) {
                out = dec.frame();
                return true;
            }
        }
        return false;
    }

    /// One frame and its ack. The 2 ms between the two tenures is the
    /// peer's time to decode and prepare the answer.
    static bool command_once(Op op, const uint8_t* p, uint8_t len) {
        if (!send_frame(op, p, len)) {
            return false;
        }
        Port::wait_ms(2);
        twilink::Frame f;
        if (!recv_frame(f)) {
            return false;
        }
        return f.op == Op::ack && f.len == 2 && f.data[0] == twilink::byte_of(op);
    }

    /// Three attempts, the peer's own recovery bound (400 ms) between them,
    /// then the arm margin twi_link.hpp names before any measured edge.
    static bool command(Op op, const uint8_t* p = nullptr, uint8_t len = 0) {
        static const uint8_t none[1] = {0};
        const uint8_t* q = p != nullptr ? p : none;
        for (uint8_t k = 0; k < 3; ++k) {
            if (command_once(op, q, len)) {
                Port::wait_ms(twilink::arm_ms);
                return true;
            }
            Port::ready();
            Port::wait_ms(400);
        }
        return false;
    }

    /// A query: the command, then a second read collecting its data frame.
    static bool query(Op op, twilink::Frame& data) {
        if (!command(op)) {
            return false;
        }
        Port::wait_ms(2);
        return recv_frame(data);
    }

    static bool ident(twilink::Ident& id) {
        twilink::Frame f;
        if (!query(Op::ident, f) || f.op != Op::ident_data || f.len != twilink::ident_size) {
            return false;
        }
        id = twilink::get_ident(f.data);
        return true;
    }

    /// The report of the last action, asked up to four times (a report
    /// asked while the action's deadline is still running is not ready).
    static bool report(twilink::Report& r) {
        for (uint8_t k = 0; k < 4; ++k) {
            twilink::Frame f;
            if (query(Op::report, f) && f.op == Op::report_data &&
                f.len == twilink::report_size) {
                r = twilink::get_report(f.data);
                return true;
            }
            Port::wait_ms(60);
        }
        return false;
    }

    static bool act(Op op, const twilink::Params& a) {
        uint8_t p[twilink::params_size];
        twilink::put_params(p, a);
        return command(op, p, twilink::params_size);
    }
};
