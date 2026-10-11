// Host tests for util/testbench.hpp: the bench suite GRAMMAR, which is
// not decoration but the contract bin/brio parses. Every check
// here is about characters on the wire and about the counters behind
// them. Run with: ctest --preset host (or ctest --preset host -R <suite name>)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest.h>

#include <cstdint>
#include <regex>
#include <string>

#include "util/testbench.hpp"

namespace {

// ---- a capture sink ---------------------------------------------------------
// A ByteSink is monostate (static write_byte), so the capture buffer is
// a static too; each test clears it first.
struct Capture {
    static inline std::string text;
    static bool write_byte(uint8_t b) {
        text.push_back(static_cast<char>(b));
        return true;
    }
    static void clear() { text.clear(); }
};
static_assert(brio::ByteSink<Capture>);

using Bench = brio::TestBench<Capture>;

// The judge in bin/brio, verbatim: whatever the harness emits must
// still be read by THIS.
const std::regex summary_re(R"((\d+)\s+pass,\s*(\d+)\s+fail)");

/// The lines bench.py would keep: the ones that carry a summary.
std::string find_marked(const std::string& text, const std::string& marker) {
    size_t pos = 0;
    while (pos <= text.size()) {
        const size_t nl = text.find('\n', pos);
        const std::string line =
            text.substr(pos, (nl == std::string::npos ? text.size() : nl) - pos);
        if (line.find(marker) != std::string::npos &&
            std::regex_search(line, summary_re)) {
            return line;
        }
        if (nl == std::string::npos) {
            break;
        }
        pos = nl + 1;
    }
    return {};
}

bool contains(const std::string& text, const std::string& what) {
    return text.find(what) != std::string::npos;
}

// ---- the tests the bench drives ---------------------------------------------
// They are plain functions, so they reach the bench through a pointer.
// One bench instance per test case keeps them independent; the pointer
// below is what the free functions call into.
Bench* active = nullptr;
std::string order;                 ///< letters, in the order they ran

void ta() {
    order.push_back('a');
    active->verdict("alpha one", true);
    active->verdict("alpha two", true);
}
void tb() {
    order.push_back('b');
    active->verdict("beta one", true);
    active->verdict("beta two", false);
    active->verdict("beta three", true);
}
void tc() {
    order.push_back('c');
    active->verdict("gamma ", "with a runtime label", false);
}
void td() {
    order.push_back('d');
    active->verdict("delta only by name", true);
}

/// A bench with a..c in z and d by name only.
Bench make() {
    Bench b;
    REQUIRE(b.letter('a', "the first thing", ta));
    REQUIRE(b.letter('b', "the second thing", tb));
    REQUIRE(b.letter('c', "the third thing", tc));
    REQUIRE(b.letter('d', "the expensive thing", td, false));
    return b;
}

} // namespace

TEST_CASE("a verdict line is two spaces, PASS or FAIL, two spaces, the name") {
    Capture::clear();
    Bench b;
    active = &b;
    b.verdict("a thing that holds", true);
    b.verdict("a thing that does not", false);
    b.verdict("two ", "pieces", true);

    CHECK(Capture::text ==
          "  PASS  a thing that holds\r\n"
          "  FAIL  a thing that does not\r\n"
          "  PASS  two pieces\r\n");
    CHECK(b.passed() == 2);
    CHECK(b.failed() == 1);
}

TEST_CASE("one letter prints its title, its verdicts and its own tally") {
    Capture::clear();
    order.clear();
    Bench b = make();
    active = &b;

    const auto t = b.handle('b');
    REQUIRE(t.has_value());
    CHECK(t->pass == 2);
    CHECK(t->fail == 1);
    CHECK(order == "b");
    CHECK(Capture::text ==
          "-- the second thing\r\n"
          "  PASS  beta one\r\n"
          "  FAIL  beta two\r\n"
          "  PASS  beta three\r\n"
          "  -> 2 pass, 1 fail\r\n"
          "\r\n");
}

TEST_CASE("the per-letter tally matches bench.py's summary regex") {
    Capture::clear();
    Bench b = make();
    active = &b;
    (void)b.handle('a');
    std::smatch m;
    REQUIRE(std::regex_search(Capture::text, m, summary_re));
    CHECK(m[1] == "2");
    CHECK(m[2] == "0");
}

TEST_CASE("counters are cleared at the start of every letter, never inherited") {
    Capture::clear();
    Bench b = make();
    active = &b;

    (void)b.handle('b');                       // 2 pass, 1 fail
    CHECK(b.passed() == 2);
    CHECK(b.failed() == 1);
    Capture::clear();
    const auto t = b.handle('a');              // 2 pass, 0 fail - not 4/1
    REQUIRE(t.has_value());
    CHECK(t->pass == 2);
    CHECK(t->fail == 0);
    CHECK(b.failed() == 0);
    CHECK(contains(Capture::text, "  -> 2 pass, 0 fail\r\n"));
    CHECK(!contains(Capture::text, "4 pass"));
}

TEST_CASE("the all-key runs every in_all letter in REGISTRATION order and "
          "closes with the ALL: line bench.py waits for") {
    Capture::clear();
    order.clear();
    Bench b = make();
    active = &b;

    const auto t = b.handle('z');
    REQUIRE(t.has_value());
    CHECK(order == "abc");                     // d is by name only
    CHECK(t->pass == 4);                       // 2 + 2 + 0
    CHECK(t->fail == 2);                       // 0 + 1 + 1

    // Each letter still closed with its own tally...
    CHECK(contains(Capture::text, "  -> 2 pass, 0 fail\r\n"));
    CHECK(contains(Capture::text, "  -> 2 pass, 1 fail\r\n"));
    CHECK(contains(Capture::text, "  -> 0 pass, 1 fail\r\n"));
    // ...and the run closed with the total, on a line bench.py picks by
    // its "ALL:" marker and parses with its own regex.
    const std::string marked = find_marked(Capture::text, "ALL:");
    CHECK(marked == "ALL: 4 pass, 2 fail\r");
    std::smatch m;
    REQUIRE(std::regex_search(marked, m, summary_re));
    CHECK(m[1] == "4");
    CHECK(m[2] == "2");
}

TEST_CASE("the all-key answers in upper case too, as every hand-written "
          "suite did") {
    Capture::clear();
    order.clear();
    Bench b = make();
    active = &b;
    const auto t = b.handle('Z');
    REQUIRE(t.has_value());
    CHECK(order == "abc");
    CHECK(contains(Capture::text, "ALL: 4 pass, 2 fail\r\n"));
}

TEST_CASE("an unknown letter prints nothing and returns nothing: what to "
          "say about it is the app's business") {
    Capture::clear();
    order.clear();
    Bench b = make();
    active = &b;

    CHECK(!b.handle('q').has_value());
    CHECK(!b.handle('?').has_value());
    CHECK(Capture::text.empty());
    CHECK(order.empty());
}

TEST_CASE("registration refuses a duplicate key, the all-key, a null "
          "function and an overfull table") {
    Bench b = make();
    active = &b;
    CHECK(b.letter_count() == 4);
    CHECK(b.has_letter('a'));
    CHECK(!b.has_letter('q'));

    CHECK(!b.letter('a', "a second first thing", tb));   // duplicate
    CHECK(!b.letter('z', "the all-key itself", tb));     // collides with z
    CHECK(!b.letter('Z', "the all-key in caps", tb));
    CHECK(!b.letter('e', "no function", nullptr));
    CHECK(b.letter_count() == 4);                        // nothing was added

    // ...and the table is bounded by its template argument.
    brio::TestBench<Capture, 2> small;
    CHECK(small.letter('a', "one", ta));
    CHECK(small.letter('b', "two", tb));
    CHECK(!small.letter('c', "three", tc));
    CHECK(small.letter_count() == 2);
}

TEST_CASE("a refused registration leaves the original letter running") {
    Capture::clear();
    order.clear();
    Bench b = make();
    active = &b;
    CHECK(!b.letter('a', "an impostor", tb));
    (void)b.handle('a');
    CHECK(order == "a");                        // ta ran, not tb
    CHECK(contains(Capture::text, "-- the first thing\r\n"));
}

TEST_CASE("the menu lists every letter and marks the ones the all-key skips") {
    Capture::clear();
    Bench b = make();
    active = &b;
    b.menu();

    CHECK(contains(Capture::text, "  a  the first thing\r\n"));
    CHECK(contains(Capture::text, "  b  the second thing\r\n"));
    CHECK(contains(Capture::text, "  c  the third thing\r\n"));
    CHECK(contains(Capture::text,
                   "  d  the expensive thing   (by name only, not in z)\r\n"));
    CHECK(contains(Capture::text, "  z  every letter above\r\n"));
    // The menu says nothing that could be mistaken for a tally.
    CHECK(!std::regex_search(Capture::text, summary_re));
}

TEST_CASE("the prompt is exactly what bench.py waits for after the marker") {
    Capture::clear();
    Bench b;
    active = &b;
    b.prompt();
    CHECK(Capture::text == "> ");
}

TEST_CASE("a by-name letter runs when asked and is not counted in the "
          "all-key's total") {
    Capture::clear();
    order.clear();
    Bench b = make();
    active = &b;

    const auto t = b.handle('d');
    REQUIRE(t.has_value());
    CHECK(t->pass == 1);
    CHECK(t->fail == 0);
    CHECK(order == "d");
    CHECK(contains(Capture::text, "-- the expensive thing\r\n"));
}

TEST_CASE("end_letter can be driven by hand, which is what a test that "
          "spans a reset needs") {
    Capture::clear();
    Bench b;
    active = &b;
    b.reset_tally();
    b.verdict("the half before the reset", true);
    b.verdict("the half after it", true);
    b.end_letter();
    CHECK(contains(Capture::text, "  -> 2 pass, 0 fail\r\n\r\n"));
    b.reset_tally();
    CHECK(b.passed() == 0);
    CHECK(b.failed() == 0);
}

TEST_CASE("an empty bench still closes an all-run with a well-formed line") {
    Capture::clear();
    Bench b;
    active = &b;
    const auto t = b.handle('z');
    REQUIRE(t.has_value());
    CHECK(t->pass == 0);
    CHECK(t->fail == 0);
    CHECK(Capture::text == "ALL: 0 pass, 0 fail\r\n");
    CHECK(!find_marked(Capture::text, "ALL:").empty());
}

TEST_CASE("a suite may name its own all-key") {
    Capture::clear();
    order.clear();
    brio::TestBench<Capture> b{'y'};
    active = &b;
    REQUIRE(b.letter('a', "the first thing", ta));
    CHECK(!b.letter('y', "the all-key", tb));
    CHECK(b.letter('z', "an ordinary z now", tb));

    (void)b.handle('y');
    CHECK(order == "ab");                     // ta then tb, in registration order
    CHECK(contains(Capture::text, "ALL: 4 pass, 1 fail\r\n"));
}

// ---- the second set ----------------------------------------------------------
// A bench with a second all-key: the single-board letters under z, the
// two-board ones under the second key, each set closed by its own ALL:.

namespace {

using TwoKey = brio::TwoKeyTestBench<Capture, 'y'>;
TwoKey* active2 = nullptr;

void t2a() {
    order.push_back('a');
    active2->verdict("single one", true);
}
void t2b() {
    order.push_back('b');
    active2->verdict("single two", false);
}
void t2j() {
    order.push_back('j');
    active2->verdict("peer one", true);
    active2->verdict("peer two", true);
}
void t2k() {
    order.push_back('k');
    active2->verdict("peer three", false);
}
void t2v() {
    order.push_back('v');
    active2->verdict("by name", true);
}

/// a, b in z; j, k in y, registered interleaved; v by name only.
TwoKey make_two() {
    TwoKey b;
    REQUIRE(b.letter('a', "single a", t2a));
    REQUIRE(b.second_letter('j', "peer j", t2j));
    REQUIRE(b.letter('b', "single b", t2b));
    REQUIRE(b.second_letter('k', "peer k", t2k));
    REQUIRE(b.letter('v', "by name v", t2v, false));
    return b;
}

} // namespace

TEST_CASE("a bench without a second set pays nothing for the possibility") {
    // The empty member of the default bench: its size is that of the same
    // table with no mask at all, which the explicit NUL argument spells.
    static_assert(sizeof(brio::TestBench<Capture>) == sizeof(brio::TestBench<Capture, 24, '\0'>));
    static_assert(sizeof(brio::TestBench<Capture>) < sizeof(brio::TwoKeyTestBench<Capture, 'y'>));
    CHECK(true);
}

TEST_CASE("the first all-key runs its own set alone, the second never") {
    Capture::clear();
    order.clear();
    TwoKey b = make_two();
    active2 = &b;
    const auto t = b.handle('z');
    REQUIRE(t.has_value());
    CHECK(order == "ab");
    CHECK(t->pass == 1);
    CHECK(t->fail == 1);
    CHECK(find_marked(Capture::text, "ALL:") == "ALL: 1 pass, 1 fail\r");
}

TEST_CASE("the second all-key runs its set alone, in registration order, "
          "and closes with an ALL: line of its own") {
    Capture::clear();
    order.clear();
    TwoKey b = make_two();
    active2 = &b;
    const auto t = b.handle('y');
    REQUIRE(t.has_value());
    CHECK(order == "jk");
    CHECK(t->pass == 2);
    CHECK(t->fail == 1);
    CHECK(contains(Capture::text, "-- peer j\r\n"));
    CHECK(contains(Capture::text, "  -> 2 pass, 0 fail\r\n"));
    CHECK(contains(Capture::text, "  -> 0 pass, 1 fail\r\n"));
    const std::string marked = find_marked(Capture::text, "ALL:");
    CHECK(marked == "ALL: 2 pass, 1 fail\r");
    std::smatch m;
    REQUIRE(std::regex_search(marked, m, summary_re));
    CHECK(m[1] == "2");

    Capture::clear();
    order.clear();
    REQUIRE(b.handle('Y').has_value());       // a lower-case key answers in caps too
    CHECK(order == "jk");
}

TEST_CASE("a second-set letter and a by-name letter each run by name") {
    Capture::clear();
    order.clear();
    TwoKey b = make_two();
    active2 = &b;
    const auto t = b.handle('k');
    REQUIRE(t.has_value());
    CHECK(t->fail == 1);
    (void)b.handle('v');
    CHECK(order == "kv");
    CHECK(Capture::text.find("ALL:") == std::string::npos);
}

TEST_CASE("the menu marks the second set and names its key") {
    Capture::clear();
    TwoKey b = make_two();
    active2 = &b;
    b.menu();
    CHECK(contains(Capture::text, "  a  single a\r\n"));
    CHECK(contains(Capture::text, "  j  peer j   (in y, not in z)\r\n"));
    CHECK(contains(Capture::text, "  v  by name v   (by name only, not in z)\r\n"));
    CHECK(contains(Capture::text, "  z  every letter above\r\n"));
    CHECK(contains(Capture::text, "  y  every letter marked in y\r\n"));
    CHECK(!std::regex_search(Capture::text, summary_re));
}

TEST_CASE("registration refuses the second all-key in either case and a key "
          "taken by the other set") {
    TwoKey b = make_two();
    active2 = &b;
    CHECK(!b.letter('y', "the second key", t2a));
    CHECK(!b.second_letter('Y', "the second key in caps", t2a));
    CHECK(!b.second_letter('a', "a first-set key", t2a));
    CHECK(!b.letter('j', "a second-set key", t2a));
    CHECK(!b.second_letter('z', "the first key", t2a));
    CHECK(b.letter_count() == 5);

    // An upper-case second key leaves its lower case free.
    brio::TwoKeyTestBench<Capture, 'Y'> upper;
    active2 = nullptr;
    CHECK(upper.letter('y', "an ordinary y", t2a));
    CHECK(upper.second_letter('j', "peer j", t2j));
    CHECK(!upper.letter('Y', "the key itself", t2a));
}

// ---- the group selection ----------------------------------------------------
// A split part's build names one group's keys; the selection must carry a
// key the group names and drop one it does not - a digit key (a second
// set's) as well as a letter, or a two-board set rides in every group image.

TEST_CASE("a selection carries the letters and the digits it names, and no others") {
    constexpr const char* g1 = "abcdefghij";
    constexpr const char* g3 = "123456789";
    static_assert(brio::selection_carries(g1, 'a') && brio::selection_carries(g1, 'j'));
    static_assert(!brio::selection_carries(g1, 'k'));
    static_assert(!brio::selection_carries(g1, '1'), "a letter group does not carry a digit key");
    static_assert(brio::selection_carries(g3, '1') && brio::selection_carries(g3, '9'));
    static_assert(!brio::selection_carries(g3, '0'));
    static_assert(!brio::selection_carries(g3, 'a'), "a digit group does not carry a letter key");
    CHECK(brio::selection_carries("ab12", '2'));
    CHECK(!brio::selection_carries("ab12", '3'));
}

TEST_CASE("no selection carries every key, and a key that is neither letter nor digit is always carried") {
    static_assert(brio::selection_carries(nullptr, 'z') && brio::selection_carries(nullptr, '0') &&
                  brio::selection_carries(nullptr, '9'));
    static_assert(brio::letter_mask_of(nullptr) == 0x03FFFFFFUL && brio::digit_mask_of(nullptr) == 0x03FFu);
    static_assert(brio::selection_carries("abc", 'T') && brio::selection_carries("123", '#'));
    // This test binary names no selection: the image carries everything.
    CHECK(brio::test_letter_carried('a'));
    CHECK(brio::test_letter_carried('7'));
}
