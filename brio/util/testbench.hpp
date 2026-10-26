/*
 * testbench.hpp
 *
 * TestBench: the one implementation of the bench suite grammar. A bench
 * suite, a `test_<target>_<subject>` app, is a menu of letters over a
 * serial console: each letter is one test, run by sending its key, and
 * the all-key ('z' by default) runs every letter registered for it in a
 * row. TestBench keeps the letter table, prints the menu, runs a letter
 * with its tally cleared, prints each verdict and the closing totals.
 * The smallest use, as the suites spell it:
 *
 *     brio::TestBench<Serial> bench;          // 'z' = the all-key; sized
 *                                             // by the build's selection
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
 * Who calls what. A bench is one object driven from one context, the
 * suite's main loop: letter() at startup, then handle(), which runs the
 * test function in the caller's context, to its end, before it returns.
 * Nothing here is guarded, so a verdict from an interrupt handler races
 * the tally. Every line is written with util/print.hpp, which waits
 * until the sink has taken every byte (print.hpp, "Delivery policy"); a
 * test that measures time should take its readings before it prints.
 * The sink is any ByteSink and nothing here knows a target, which is
 * what lets the host suite drive it with a capture sink.
 *
 * Through a writer. The sink may instead be a LineWriter
 * (util/line_writer.hpp), the one writer of the console's transport: a
 * sink like a transport, which formats each print into the caller's line
 * and writes the line whole when it ends, so every print here stays as
 * written. The bench's context must be one the writer lets wait (main
 * context, or the object its `wait_from` option names); then each line
 * leaves as the suite prints it, waiting for the ring as print() waits.
 * The prompt ends with flush_line, so that it leaves before its line
 * ends, and the two tallies carry the writer's count of refused lines:
 * when it lost lines during the letter or the run, the tally reads
 *
 *     -> N pass, M fail, L lost
 *
 * and bin/brio's judge, under --strict, fails a run that lost a line.
 *
 * The ritual around the letters - a letter table, a verdict printer, a
 * pair of counters, the per-letter tally and the closing total - is not
 * decoration: `bin/brio` reads it. Its judge matches
 *
 *     N pass, M fail
 *
 * and waits for the "ALL:" marker followed by the "> " prompt. A
 * machine-read grammar copied into every suite is one chance to drift
 * per copy, so the grammar lives here and nowhere else:
 *
 *     -- <title>                       (one letter starts)
 *       PASS  <name>                   (verdict, two spaces of indent)
 *       FAIL  <name>
 *       -> N pass, M fail              (that letter's tally, blank line)
 *     ALL: N pass, M fail              (the all-key's total)
 *     >                                (the prompt bin/brio waits for)
 *
 * A suite prints its own measurements between verdicts with plain
 * print(): the bench is about the scaffolding, not about what a test has
 * to say.
 *
 * Counters. passed/failed are reset at the start of each letter, so a
 * test never inherits the previous one's tally; the all-key accumulates
 * the letters' totals separately. A test that spans a reset (the reset
 * suites bank their tally in .noinit and resume) drives end_letter() by
 * hand - which is why it is public.
 *
 * A second set. A suite whose letters fall into two classes that must
 * never run under one key - the single-board letters, runnable with the
 * far board dark, and the two-board letters, which need a peer (a second
 * board wired to this one) and fail loudly without one - names a second
 * all-key as a template argument (TwoKeyTestBench<Serial, 'y'>) and
 * registers that class with second_letter(). The first all-key never
 * runs them, the second runs them alone in registration order and closes
 * with an ALL: line of its own, each is also run by name. An image whose
 * bench names no second key compiles none of it: the set's mask is an
 * empty member and every branch that reads it is `if constexpr`. The
 * mask is one bit per registered letter, as wide as the table needs: 32
 * bits up to 32 letters, 64 up to 64 (the keys a group can gate -
 * 'a'..'z', '0'..'9' and 'A'..'Z' - are 62).
 */

#pragma once

#include <stdint.h>
#include <concepts>
#include <optional>
#include <type_traits>

#include "util/print.hpp"
#include "util/stream.hpp"

namespace brio {

/**
 * The letters an image carries. A suite whose whole image does not fit
 * a part of its family declares its groups of letters in its build
 * header (`// build: groups = abg,cdf,e`), and on such a part the build
 * emits one image per group, defining BRIO_TEST_LETTERS as that group's
 * keys; every other image carries every letter (docs/design/overview.md,
 * "Target strata: drivers by role, not a HAL", the rule "A suite's image
 * fits the family's smallest chip"). A group names letters
 * and digits alike - a second set's digit keys belong to the group that
 * names them, as letters do - and upper-case keys too, once its line
 * names any (upper_mask_of()). letter() registers
 * nothing for a key outside the selection, so a letter the image does
 * not carry is never referenced and the linker drops it with its
 * verdict prose - the source is the same in every image, and the menu
 * says which letters this one holds.
 */
#ifdef BRIO_TEST_LETTERS
inline constexpr const char* test_letters = BRIO_TEST_LETTERS;
#else
inline constexpr const char* test_letters = nullptr;
#endif

/// A selection string as one bit per letter 'a'..'z', every bit set for
/// no selection (nullptr): an integer constant, so that letter()'s check
/// on a literal key folds to a constant at every call site - a loop over
/// the string would not have to, and a check that survives keeps the
/// test's address alive in main.
constexpr uint32_t letter_mask_of(const char* selection) {
    if (selection == nullptr) {
        return 0x03FFFFFFUL;
    }
    uint32_t mask = 0;
    for (const char* p = selection; *p != '\0'; ++p) {
        if (*p >= 'a' && *p <= 'z') {
            mask |= 1UL << (*p - 'a');
        }
    }
    return mask;
}

/// The same for the digit keys '0'..'9' - the keys a second set takes
/// (TwoKeyTestBench) - so that a group image carries a digit letter only
/// where its selection names it, as it carries a lower-case one.
constexpr uint16_t digit_mask_of(const char* selection) {
    if (selection == nullptr) {
        return 0x03FFu;
    }
    uint16_t mask = 0;
    for (const char* p = selection; *p != '\0'; ++p) {
        if (*p >= '0' && *p <= '9') {
            mask = static_cast<uint16_t>(mask | (1u << (*p - '0')));
        }
    }
    return mask;
}

/// The same for the upper-case keys 'A'..'Z', with one difference: a
/// selection that names no upper-case key carries every one of them. A
/// groups line that partitions only the lower-case letters and the
/// digits leaves each upper-case key in every image of the line; a line
/// that names any upper-case key partitions them as it partitions the
/// rest, and an upper-case key no group of it names is in no image. A
/// group sees only its own keys, so the line's choice reaches it as a
/// marker: the split build appends '.' to every group of a line that
/// names any upper-case key (each project's CMakeLists.txt), and a
/// selection carrying '.' names its upper-case keys explicitly, none
/// included.
constexpr uint32_t upper_mask_of(const char* selection) {
    if (selection == nullptr) {
        return 0x03FFFFFFUL;
    }
    uint32_t mask = 0;
    bool marked = false;
    for (const char* p = selection; *p != '\0'; ++p) {
        if (*p >= 'A' && *p <= 'Z') {
            mask |= 1UL << (*p - 'A');
        }
        marked = marked || *p == '.';
    }
    return mask == 0 && !marked ? 0x03FFFFFFUL : mask;
}

/// Whether a selection carries `key`: a lower-case letter or a digit when
/// the selection names it, an upper-case letter by upper_mask_of()'s rule
/// (every key of the three kinds when there is no selection); any other
/// key - a sign - always.
constexpr bool selection_carries(const char* selection, char key) {
    if (key >= '0' && key <= '9') {
        return ((digit_mask_of(selection) >> (key - '0')) & 1u) != 0u;
    }
    if (key >= 'A' && key <= 'Z') {
        return ((upper_mask_of(selection) >> (key - 'A')) & 1UL) != 0u;
    }
    if (key < 'a' || key > 'z') {
        return true;
    }
    return ((letter_mask_of(selection) >> (key - 'a')) & 1UL) != 0u;
}

constexpr uint32_t test_letter_mask() { return letter_mask_of(test_letters); }
inline constexpr uint32_t test_letters_carried = test_letter_mask();
inline constexpr uint16_t test_digits_carried = digit_mask_of(test_letters);
inline constexpr uint32_t test_uppers_carried = upper_mask_of(test_letters);

/// Whether this image carries `key`: every key when the build named no
/// selection; a lower-case letter or a digit when the selection names
/// it, an upper-case letter by upper_mask_of()'s rule; any other key
/// always. Read off the three masks, and always inlined: letter()'s
/// check must fold to a constant at the call site, and a call the
/// optimizer leaves standing (-Os weighs this body against its callers)
/// keeps every letter's address alive in every group image.
[[gnu::always_inline]] constexpr bool test_letter_carried(char key) {
    if (key >= '0' && key <= '9') {
        return ((test_digits_carried >> (key - '0')) & 1u) != 0u;
    }
    if (key >= 'A' && key <= 'Z') {
        return ((test_uppers_carried >> (key - 'A')) & 1UL) != 0u;
    }
    if (key < 'a' || key > 'z') {
        return true;
    }
    return ((test_letters_carried >> (key - 'a')) & 1UL) != 0u;
}

/// The registry's default size: a whole suite's 24, or the selection's
/// count of keys (upper_mask_of()'s marker is none) - the table is .data,
/// and a 2 KB part counts every byte of it.
constexpr uint8_t test_letters_default_max() {
    if (test_letters == nullptr) {
        return 24;
    }
    uint8_t n = 0;
    for (const char* p = test_letters; *p != '\0'; ++p) {
        if (*p != '.') ++n;
    }
    return n;
}

/// A sink that counts the lines it refused (util/line_writer.hpp's
/// LineWriter): the bench's tallies carry the count.
template <typename S>
concept LosesLines = requires {
    { S::lost() } -> std::same_as<uint16_t>;
};

namespace detail {
/// The second set's mask in a bench that has no second set.
struct NoSecondSet {};

/// The writer's count of refused lines as a letter and a run started, in
/// a bench that prints through a LineWriter (the header's "Through a
/// writer").
struct WriterMarks {
    uint16_t letter_lost = 0;
    uint16_t run_lost = 0;
};
/// The same in a bench that prints to a transport: nothing.
struct NoWriterMarks {};
} // namespace detail

/// The letter menu, the verdict lines and the two tallies bin/brio reads.
/// `Sink` is a transport or a LineWriter (util/line_writer.hpp), a ByteSink
/// either way. `max_letters` bounds the static table; registration
/// beyond it fails. `second_all_key`, when not NUL, names the second set's
/// all-key (the header's "A second set"). Main context, one caller (the
/// file header).
template <ByteSink Sink, uint8_t max_letters = test_letters_default_max(),
          char second_all_key = '\0'>
class TestBench {
    static constexpr bool has_second_set = second_all_key != '\0';
    /// Lines go to a LineWriter: the prompt ends its line, and the totals
    /// carry the lines the writer refused.
    static constexpr bool through_writer = LosesLines<Sink>;
    static_assert(!has_second_set || max_letters <= 64,
                  "the second set is a mask over at most 64 registered letters");

public:
    /// The second set's mask: a bit per registered letter, the narrower
    /// word wherever the table fits it (a bench of 32 letters or fewer
    /// pays no 64-bit shift on a 32-bit core, nor on an AVR).
    using SecondMask = std::conditional_t<(max_letters <= 32), uint32_t, uint64_t>;

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
     * keeps the letter out of the all-key set - the shape a test needs
     * when it costs endurance, a fuse round trip or a reflash, and must
     * be asked for by name.
     *
     * False when the table is full, when the key is already taken, or
     * when it collides with the all-key. Registration happens once at
     * startup, so a false return is a programming error the app can
     * report - it is not a runtime condition to recover from. A key
     * this image does not carry (test_letter_carried) is accepted and
     * not registered: the decision is at the call site, always inlined,
     * so the test's address is never taken in that image.
     */
    [[gnu::always_inline]] bool letter(char key, const char* title, TestFn fn, bool in_all = true) {
        if (!test_letter_carried(key)) {
            return true;
        }
        return register_letter(key, title, fn, in_all);
    }

    /// Register one letter of the second set: out of the first all-key,
    /// run by the second and by name. The same refusals as letter(), and
    /// the second all-key is refused as a key too.
    [[gnu::always_inline]] bool second_letter(char key, const char* title, TestFn fn)
        requires has_second_set
    {
        if (!test_letter_carried(key)) {
            return true;
        }
        const uint8_t at = count_;
        if (!register_letter(key, title, fn, false)) {
            return false;
        }
        second_ = static_cast<SecondMask>(second_ | (SecondMask{1} << at));
        return true;
    }

    /// The letter list, one per line, with the all-key last. A suite
    /// prints its own headline before calling this.
    void menu() const {
        for (uint8_t i = 0; i < count_; ++i) {
            if (letters_[i].in_all) {
                print(Sink{}, "  ", letters_[i].key, "  ", letters_[i].title, crlf);
            } else if (in_second_set(i)) {
                print(Sink{}, "  ", letters_[i].key, "  ", letters_[i].title,
                      "   (in ", second_all_key, ", not in ", all_key_, ")", crlf);
            } else {
                print(Sink{}, "  ", letters_[i].key, "  ", letters_[i].title,
                      "   (by name only, not in ", all_key_, ")", crlf);
            }
        }
        print(Sink{}, "  ", all_key_, "  every letter above", crlf);
        if constexpr (has_second_set) {
            print(Sink{}, "  ", second_all_key, "  every letter marked in ", second_all_key, crlf);
        }
        if (test_letters != nullptr) {
            print(Sink{}, "  (this image carries the letters ", test_letters,
                  " of the suite; the other letters are in its other images)", crlf);
        }
    }

    /// One verdict line, and one tick on the tally (saturating at
    /// UINT16_MAX). From the running test, in main context.
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
    /// a test that spans a reset calls it itself when it resumes. Through
    /// a writer the tally carries the lines the writer lost during the
    /// letter, when there were any.
    void end_letter() {
        if constexpr (through_writer) {
            tally("  -> ", passed_, failed_, marks_.letter_lost);
            print(Sink{}, crlf, crlf);
        } else {
            print(Sink{}, "  -> ", passed_, " pass, ", failed_, " fail", crlf, crlf);
        }
    }

    /**
     * Run what `c` names. A registered letter prints its title, runs
     * with the counters cleared and closes with its tally. The all-key
     * runs every letter marked in_all, in registration order, and closes
     * with the ALL: line bin/brio's default marker waits for. Each test
     * runs inside this call, in the caller's context; the call returns
     * when the last one has.
     *
     * Returns the run's totals, or nothing for an unregistered key.
     *
     * The second all-key, where the bench has one, runs every letter of
     * the second set the same way and closes with its own ALL: line.
     *
     * Nothing at all is printed for an unregistered key: what to say
     * about it is the app's business.
     */
    std::optional<Totals> handle(char c) {
        if (is_all_key(c)) {
            mark_run();
            Totals t{0, 0};
            for (uint8_t i = 0; i < count_; ++i) {
                if (!letters_[i].in_all) {
                    continue;
                }
                run(letters_[i]);
                t.pass = static_cast<uint16_t>(t.pass + passed_);
                t.fail = static_cast<uint16_t>(t.fail + failed_);
            }
            if constexpr (through_writer) {
                all_line(t);
            } else {
                print(Sink{}, "ALL: ", t.pass, " pass, ", t.fail, " fail", crlf);
            }
            return t;
        }
        if constexpr (has_second_set) {
            if (is_key(second_all_key, c)) {
                mark_run();
                Totals t{0, 0};
                for (uint8_t i = 0; i < count_; ++i) {
                    if (!in_second_set(i)) {
                        continue;
                    }
                    run(letters_[i]);
                    t.pass = static_cast<uint16_t>(t.pass + passed_);
                    t.fail = static_cast<uint16_t>(t.fail + failed_);
                }
                if constexpr (through_writer) {
                    all_line(t);
                } else {
                    print(Sink{}, "ALL: ", t.pass, " pass, ", t.fail, " fail", crlf);
                }
                return t;
            }
        }
        const Entry* e = find(c);
        if (e == nullptr) {
            return std::nullopt;
        }
        run(*e);
        return Totals{passed_, failed_};
    }

    /// The "> " bin/brio waits for after the ALL: marker. Through a
    /// writer it is a line ended by flush_line: written at once, closed by
    /// nothing.
    void prompt() const {
        if constexpr (through_writer) {
            print(Sink{}, "> ", flush_line);
        } else {
            print(Sink{}, "> ");
        }
    }

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
    /// verdicts from before the reset as well as after.
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

    /// The all-key answers to either case: 'z' and 'Z' both run the set.
    bool is_all_key(char c) const {
        return c == all_key_ ||
               (all_key_ >= 'a' && all_key_ <= 'z' &&
                c == static_cast<char>(all_key_ - ('a' - 'A')));
    }

    /// The second all-key by the same rule: a lower-case key answers to
    /// its upper case too, an upper-case key to itself alone.
    static constexpr bool is_key(char key, char c) {
        return c == key || (key >= 'a' && key <= 'z' && c == static_cast<char>(key - ('a' - 'A')));
    }

    /// Whether registered letter `i` belongs to the second set - never,
    /// in a bench without one.
    bool in_second_set(uint8_t i) const {
        if constexpr (has_second_set) {
            return ((second_ >> i) & 1u) != 0u;
        } else {
            (void)i;
            return false;
        }
    }

    const Entry* find(char key) const {
        for (uint8_t i = 0; i < count_; ++i) {
            if (letters_[i].key == key) {
                return &letters_[i];
            }
        }
        return nullptr;
    }

    bool register_letter(char key, const char* title, TestFn fn, bool in_all) {
        if (count_ == max_letters || fn == nullptr) {
            return false;
        }
        if (is_all_key(key) || find(key) != nullptr) {
            return false;
        }
        if constexpr (has_second_set) {
            if (is_key(second_all_key, key) || is_all_key(second_all_key)) {
                return false;
            }
        }
        letters_[count_] = Entry{key, title, fn, in_all};
        ++count_;
        return true;
    }

    void count_verdict(bool ok) {
        uint16_t& n = ok ? passed_ : failed_;
        if (n != UINT16_MAX) {
            ++n;
        }
    }

    void run(const Entry& e) {
        passed_ = failed_ = 0;
        if constexpr (through_writer) {
            marks_.letter_lost = Sink::lost();
        }
        print(Sink{}, "-- ", e.title, crlf);
        e.fn();
        end_letter();
    }

    /// The ALL: line through a writer, with the run's lost lines. (Over a
    /// transport handle() prints it in place, at each of its two sites.)
    void all_line(const Totals& t)
        requires through_writer
    {
        tally("ALL: ", t.pass, t.fail, marks_.run_lost);
        print(Sink{}, crlf);
    }

    /// An all-key run starts: the writer's count noted.
    void mark_run() {
        if constexpr (through_writer) {
            marks_.run_lost = Sink::lost();
        }
    }

    /// "<head>N pass, M fail" and, when the writer lost lines since
    /// `lost0` was noted, ", L lost": the line left open for its end.
    void tally(const char* head, uint16_t pass, uint16_t fail, uint16_t lost0)
        requires through_writer
    {
        print(Sink{}, head, pass, " pass, ", fail, " fail");
        const uint16_t lost = static_cast<uint16_t>(Sink::lost() - lost0);
        if (lost != 0u) {
            print(Sink{}, ", ", lost, " lost");
        }
    }

    Entry letters_[max_letters]{};
    uint8_t count_ = 0;
    uint16_t passed_ = 0;
    uint16_t failed_ = 0;
    char all_key_ = 'z';
    /// The second set as one bit per registered letter.
    [[no_unique_address]] std::conditional_t<has_second_set, SecondMask, detail::NoSecondSet> second_{};
    /// The writer's count of refused lines at the letter's and the run's
    /// start.
    [[no_unique_address]] std::conditional_t<through_writer, detail::WriterMarks, detail::NoWriterMarks> marks_{};
};

/// A bench with a second set: `second_all_key` runs the letters
/// second_letter() registers (the header's "A second set").
template <ByteSink Sink, char second_all_key, uint8_t max_letters = test_letters_default_max()>
using TwoKeyTestBench = TestBench<Sink, max_letters, second_all_key>;

} // namespace brio
