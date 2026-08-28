/*
 * testbench.hpp
 *
 * TestBench: the one implementation of the bench SUITE GRAMMAR.
 *
 * A `test_<target>_<subject>` app is a menu of single-letter tests over
 * a serial console. Every one of them had grown its own copy of the same
 * ritual - a letter table, a verdict printer, a pair of counters, the
 * per-letter tally and the closing total - and the ritual is not free
 * decoration: `tools/bench.py` READS it. Its judge matches
 *
 *     N pass, M fail
 *
 * and waits for the "ALL:" marker followed by the "> " prompt. Sixteen
 * hand-written copies of a machine-read grammar is sixteen chances to
 * drift, so the grammar lives here and nowhere else:
 *
 *     -- <title>                       (one letter starts)
 *       PASS  <name>                   (verdict, two spaces of indent)
 *       FAIL  <name>
 *       -> N pass, M fail              (that letter's tally, blank line)
 *     ALL: N pass, M fail              (the all-key's total)
 *     >                                (the prompt bench.py waits for)
 *
 * A suite prints its OWN measurements between verdicts with plain
 * print(): the bench is about the scaffolding, not about what a test has
 * to say. Nothing here knows a target - the sink is any ByteSink, which
 * is what lets the host suite drive it with a capture sink.
 *
 * USE:
 *
 *     brio::TestBench<Serial> bench;          // 'z' = the all-key
 *     ...
 *     bench.letter('a', "the block and its registers", ta_block);
 *     bench.letter('v', "verify the survivors", tv_verify, false);  // not in z
 *     ...
 *     bench.menu();
 *     bench.prompt();
 *     for (;;) {
 *         uint8_t c;
 *         if (!Serial::read_byte(c)) continue;
 *         if (c == '\r' || c == '\n') continue;
 *         print(serial, static_cast<char>(c), crlf);
 *         if (c == '?') { bench.menu(); }
 *         else if (!bench.handle(static_cast<char>(c))) { ... "? for help" }
 *         bench.prompt();
 *     }
 *
 * and inside a test, `bench.verdict("what must be true", expression)`.
 *
 * COUNTERS. passed/failed are reset at the START of each letter, so a
 * test never inherits the previous one's tally; the all-key accumulates
 * the letters' totals separately. A test that spans a RESET (the reset
 * suites bank their tally in .noinit and resume) drives end_letter() by
 * hand - which is why it is public.
 */

#pragma once

#include <stdint.h>
#include <optional>

#include "util/print.hpp"
#include "util/stream.hpp"

namespace brio {

/// The letter menu, the verdict lines and the two tallies bench.py reads.
/// `max_letters` bounds the static table; registration beyond it fails.
template <ByteSink Sink, uint8_t max_letters = 24>
class TestBench {
public:
    /// A test is a plain function: it prints what it measured and calls
    /// verdict() for each thing that had to be true.
    using TestFn = void (*)();

    /// What a run scored. Returned by handle() for one letter and for
    /// the whole all-key set.
    struct Totals {
        uint16_t pass;
        uint16_t fail;
    };

    /// `all_key` is the letter that runs every registered test and
    /// closes with the ALL: line; it is also accepted in upper case.
    constexpr explicit TestBench(char all_key = 'z') : all_key_(all_key) {}

    /**
     * Register one letter. `title` is what menu() prints beside the key
     * and what handle() prints when the test starts. `in_all` false
     * keeps the letter OUT of the all-key set - the shape a test needs
     * when it costs endurance, a fuse round trip or a reflash, and must
     * be asked for by name.
     *
     * False when the table is full, when the key is already taken, or
     * when it collides with the all-key. Registration happens once at
     * startup, so a false return is a programming error the app can
     * report - it is not a runtime condition to recover from.
     */
    bool letter(char key, const char* title, TestFn fn, bool in_all = true) {
        if (count_ == max_letters || fn == nullptr) {
            return false;
        }
        if (is_all_key(key) || find(key) != nullptr) {
            return false;
        }
        letters_[count_] = Entry{key, title, fn, in_all};
        ++count_;
        return true;
    }

    /// The letter list, one per line, with the all-key last. A suite
    /// prints its own headline before calling this.
    void menu() const {
        for (uint8_t i = 0; i < count_; ++i) {
            if (letters_[i].in_all) {
                print(Sink{}, "  ", letters_[i].key, "  ", letters_[i].title, crlf);
            } else {
                print(Sink{}, "  ", letters_[i].key, "  ", letters_[i].title,
                      "   (by name only, not in ", all_key_, ")", crlf);
            }
        }
        print(Sink{}, "  ", all_key_, "  every letter above", crlf);
    }

    /// One verdict line, and one tick on the tally.
    void verdict(const char* name, bool ok) {
        count_verdict(ok);
        print(Sink{}, "  ", ok ? "PASS" : "FAIL", "  ", name, crlf);
    }

    /// The same, for a name built from two pieces (a fixed phrase and a
    /// runtime label) without a buffer to format it into.
    void verdict(const char* name, const char* suffix, bool ok) {
        count_verdict(ok);
        print(Sink{}, "  ", ok ? "PASS" : "FAIL", "  ", name, suffix, crlf);
    }

    /// Close a letter: its tally and a blank line. handle() calls this;
    /// a test that spans a reset calls it itself when it resumes.
    void end_letter() {
        print(Sink{}, "  -> ", passed_, " pass, ", failed_, " fail", crlf, crlf);
    }

    /**
     * Run what `c` names. A registered letter prints its title, runs
     * with the counters cleared and closes with its tally. The all-key
     * runs every letter marked in_all, in REGISTRATION order, and closes
     * with the ALL: line bench.py's default marker waits for.
     *
     * Nothing at all is printed for an unregistered key: what to say
     * about it is the app's business.
     */
    std::optional<Totals> handle(char c) {
        if (is_all_key(c)) {
            Totals t{0, 0};
            for (uint8_t i = 0; i < count_; ++i) {
                if (!letters_[i].in_all) {
                    continue;
                }
                run(letters_[i]);
                t.pass = static_cast<uint16_t>(t.pass + passed_);
                t.fail = static_cast<uint16_t>(t.fail + failed_);
            }
            print(Sink{}, "ALL: ", t.pass, " pass, ", t.fail, " fail", crlf);
            return t;
        }
        const Entry* e = find(c);
        if (e == nullptr) {
            return std::nullopt;
        }
        run(*e);
        return Totals{passed_, failed_};
    }

    /// The "> " bench.py waits for after the ALL: marker.
    void prompt() const { print(Sink{}, "> "); }

    /// The tally of the letter that just ran (or is running).
    uint16_t passed() const { return passed_; }
    uint16_t failed() const { return failed_; }

    /// Clear the tally by hand - the counters a resuming test resets
    /// before it prints its half of the verdicts.
    void reset_tally() { passed_ = failed_ = 0; }

    /// Restore a tally banked before a reset. The other half of
    /// end_letter()'s reason to be public: a test that reboots the board
    /// keeps its counts in reset-surviving storage and hands them back
    /// here when it resumes, so the letter's closing line counts the
    /// verdicts from BEFORE the reset as well as after.
    void resume_tally(uint16_t pass, uint16_t fail) {
        passed_ = pass;
        failed_ = fail;
    }

    /// How many letters are registered, and whether one is.
    uint8_t letter_count() const { return count_; }
    bool has_letter(char key) const { return find(key) != nullptr; }

private:
    struct Entry {
        char key = 0;
        const char* title = nullptr;
        TestFn fn = nullptr;
        bool in_all = true;
    };

    /// The all-key answers to either case, the way every hand-written
    /// suite accepted 'z' and 'Z'.
    bool is_all_key(char c) const {
        return c == all_key_ ||
               (all_key_ >= 'a' && all_key_ <= 'z' &&
                c == static_cast<char>(all_key_ - ('a' - 'A')));
    }

    const Entry* find(char key) const {
        for (uint8_t i = 0; i < count_; ++i) {
            if (letters_[i].key == key) {
                return &letters_[i];
            }
        }
        return nullptr;
    }

    void count_verdict(bool ok) {
        uint16_t& n = ok ? passed_ : failed_;
        if (n != UINT16_MAX) {
            ++n;
        }
    }

    void run(const Entry& e) {
        passed_ = failed_ = 0;
        print(Sink{}, "-- ", e.title, crlf);
        e.fn();
        end_letter();
    }

    Entry letters_[max_letters]{};
    uint8_t count_ = 0;
    uint16_t passed_ = 0;
    uint16_t failed_ = 0;
    char all_key_ = 'z';
};

} // namespace brio
