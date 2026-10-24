/*
 * proto/line_parser.hpp
 *
 * Line assembly and command parsing, the protocol layer above the byte
 * transports: what turns a console's bytes into a command and calls the
 * function that handles it. Three pieces, each usable alone:
 *
 *  - LineAssembler, fed one byte at a time, returns each completed line
 *    (a push model: it holds no reference to any transport, so it works
 *    with any source - a UART ring, a test vector, a host-side unit
 *    test). util/line_reader.hpp's LineReader runs two of them and
 *    delivers the lines as events, which is how a program usually meets
 *    it;
 *  - ConsoleCommandParser ("CMD arg1 arg2") and ScpiCommandParser
 *    (":SUB:CMD? arg1,arg2") cut a line into a command word and argument
 *    pointers, in place;
 *  - CommandRouter, templated on the reply sink (any util/stream.hpp
 *    ByteSink), calls the handler whose route names the command, with
 *    the sink to answer on: no endpoint indirection, the sinks and
 *    sources resolved at compile time.
 *
 * The smallest use is a console's line sink, the active object that
 * handles LineReader's LineReceived:
 *
 *     using Parser = brio::ConsoleCommandParser<4>;
 *     using Router = brio::CommandRouter<Serial, 4>;
 *     using Cmd = Router::CommandType;
 *     static constexpr Router::Route routes[] = {{"HELP", cmd_help}, {"LED", cmd_led}};
 *
 *     static void handle_line(char* line) {     // from the LineReceived dispatch
 *         Cmd cmd;
 *         if (Parser::parse(line, cmd)) {
 *             if (!Router::dispatch(cmd, routes, route_count, serial)) {
 *                 brio::print(serial, "unknown command (try HELP)", brio::crlf);
 *             }
 *         }
 *     }
 *
 * A program that drains a transport itself feeds an assembler directly:
 *
 *     brio::LineAssembler<80> line_in;
 *     uint8_t b;
 *     while (Serial::read_byte(b)) {
 *         if (char* line = line_in.push(b)) { ...parse and dispatch... }
 *     }
 *
 * Who owns what, and for how long. Everything here works in the
 * caller's context, waits for nothing, masks nothing and keeps no
 * pointer the caller gave it beyond the call (TokenCursor keeps its line
 * for as long as the cursor lives). The text is never copied: a line
 * lives in its assembler's buffer, the parsers write into it (separators
 * become NULs, the command word is uppercased), and a ParsedCommand
 * points into it. So a command is valid for exactly as long as the line
 * is: until the assembler's next push(), or, for a line LineReader
 * lends, until the end of the LineReceived dispatch (line_reader.hpp,
 * "Who owns a line, and for how long"). A router's handler runs inside
 * dispatch(), in the caller's context, and answers through the sink it
 * is handed; a print there waits as util/print.hpp says.
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "util/stream.hpp"

namespace brio {

// ---- character / number helpers ---------------------------------------------
//
// ASCII only, case folded by ascii_upper(). parse_long() and parse_ulong()
// call the C library's strtol()/strtoul(): true when the whole text is one
// number in `base`. A 32-bit image links no C library
// (docs/design/runtime.md, "What it is for"), so a call to either there
// fails to link; the AVR keeps avr-libc, which has both.

inline char ascii_upper(char c) {
    if (c >= 'a' && c <= 'z') {
        return static_cast<char>(c - ('a' - 'A'));
    }
    return c;
}

inline void upper_inplace(char *text) {
    if (!text) {
        return;
    }
    while (*text) {
        *text = ascii_upper(*text);
        ++text;
    }
}

inline bool parse_long(const char *text, long &value, uint8_t base = 10) {
    if (!text) {
        return false;
    }
    char *end;
    value = strtol(text, &end, base);
    return (end != text) && (*end == '\0');
}

inline bool parse_ulong(const char *text, unsigned long &value, uint8_t base = 16) {
    if (!text) {
        return false;
    }
    char *end;
    value = strtoul(text, &end, base);
    return (end != text) && (*end == '\0');
}

inline bool command_equals(const char *left, const char *right) {
    if (!left || !right) {
        return false;
    }
    while (*left && *right) {
        if (ascii_upper(*left) != ascii_upper(*right)) {
            return false;
        }
        ++left;
        ++right;
    }
    return (*left == '\0') && (*right == '\0');
}

// ---- line assembly (push model) ---------------------------------------------

/// Assembles newline-terminated lines from a pushed byte stream.
///
/// - '\r' is ignored
/// - '\n' completes the line: push() returns the NUL-terminated text
/// - a line holds up to max_line_length - 1 characters and the NUL; the
///   character after that overflows it: the line is discarded and bytes
///   are dropped until the next '\n' (counted in overflow_count), and
///   that '\n' completes no line
///
/// The returned pointer refers to the internal buffer and stays valid
/// until the next push(): consume (or copy) the line before feeding more
/// bytes. The caller may write the line in place (the parsers do).
template <uint8_t max_line_length = 96>
class LineAssembler {
    static_assert(max_line_length >= 4, "line buffer should be at least 4 chars");

private:
    char m_line[max_line_length]{};
    uint8_t m_length{0};
    bool m_drop_until_eol{false};
    uint8_t m_overflow_count{0};

public:
    /// Feed one byte; returns the completed line or nullptr. Never waits;
    /// one assembler takes one feeder (nothing here is guarded).
    char *push(uint8_t byte) {
        if (m_drop_until_eol) {
            if (byte == '\n') {
                m_drop_until_eol = false;
                m_length = 0;
            }
            return nullptr;
        }

        if (byte == '\r') {
            return nullptr;
        }

        if (byte == '\n') {
            m_line[m_length] = '\0';
            m_length = 0;
            return m_line;
        }

        if (m_length + 1 >= max_line_length) {
            m_overflow_count = static_cast<uint8_t>(m_overflow_count + 1);
            m_length = 0;
            m_drop_until_eol = true;
            return nullptr;
        }

        m_line[m_length++] = static_cast<char>(byte);
        return nullptr;
    }

    /// Lines discarded for overflowing the buffer since the last
    /// clear_counters(); eight bits, wrapping at 256.
    uint8_t overflow_count() const { return m_overflow_count; }
    void clear_counters() { m_overflow_count = 0; }
};

// ---- tokenization -----------------------------------------------------------

/// Non-owning tokenizer on top of a mutable C string. Tokenization
/// happens in place by replacing separators with '\0'. next() hands out
/// the next token (a pointer into the line) and false at the end; the
/// line must outlive the cursor and the tokens.
class TokenCursor {
private:
    char *m_next;
    bool m_comma_is_separator;

    inline bool is_separator(char c) const {
        if (c == ' ' || c == '\t') {
            return true;
        }
        return m_comma_is_separator && c == ',';
    }

public:
    explicit TokenCursor(char *line, bool comma_is_separator = false)
        : m_next(line), m_comma_is_separator(comma_is_separator) {}

    bool next(char *&token) {
        token = nullptr;
        if (!m_next) {
            return false;
        }

        while (*m_next && is_separator(*m_next)) {
            ++m_next;
        }
        if (!*m_next) {
            return false;
        }

        token = m_next;
        while (*m_next && !is_separator(*m_next)) {
            ++m_next;
        }
        if (*m_next) {
            *m_next = '\0';
            ++m_next;
        }
        return true;
    }
};

// ---- parsed command representation ------------------------------------------

/// Generic parsed command: command word + argument pointers. The command
/// string and the argument pointers refer to memory inside the line
/// buffer (LineAssembler): consume them before pushing more bytes.
template <uint8_t max_arguments = 8>
struct ParsedCommand {
    char *command;
    char *arguments[max_arguments];
    uint8_t argument_count;
    bool is_query;

    inline void clear() {
        command = nullptr;
        argument_count = 0;
        is_query = false;
        for (uint8_t i = 0; i < max_arguments; ++i) {
            arguments[i] = nullptr;
        }
    }
};

// ---- parsers ----------------------------------------------------------------

/// Console-style parser: "CMD arg1 arg2", spaces/tabs separate. The
/// command word is uppercased in place. parse() is false for an empty
/// line or one with more than max_arguments arguments; on true,
/// out_command points into `line`.
template <uint8_t max_arguments = 8>
class ConsoleCommandParser {
public:
    using CommandType = ParsedCommand<max_arguments>;

    static bool parse(char *line, CommandType &out_command) {
        out_command.clear();
        if (!line) {
            return false;
        }

        TokenCursor cursor(line, false);
        char *token = nullptr;
        if (!cursor.next(token)) {
            return false;
        }

        upper_inplace(token);
        out_command.command = token;

        while (cursor.next(token)) {
            if (out_command.argument_count >= max_arguments) {
                return false;
            }
            out_command.arguments[out_command.argument_count++] = token;
        }

        return true;
    }
};

/// Minimal SCPI-like parser: ":SUB:SYS:CMD? arg1,arg2".
///
/// Separators: spaces, tabs, commas. Normalization: command uppercased in
/// place, optional leading ':' removed, trailing '?' sets is_query.
/// parse() is false for an empty line, a command that is only ':' or '?',
/// or more than max_arguments arguments; on true, out_command points into
/// `line`.
template <uint8_t max_arguments = 8>
class ScpiCommandParser {
public:
    using CommandType = ParsedCommand<max_arguments>;

    static bool parse(char *line, CommandType &out_command) {
        out_command.clear();
        if (!line) {
            return false;
        }

        TokenCursor cursor(line, true);
        char *token = nullptr;
        if (!cursor.next(token)) {
            return false;
        }

        upper_inplace(token);
        if (token[0] == ':') {
            ++token;
        }
        if (!*token) {
            return false;
        }

        const uint8_t len = static_cast<uint8_t>(strlen(token));
        if (len && token[len - 1] == '?') {
            token[len - 1] = '\0';
            out_command.is_query = true;
        }
        if (!*token) {
            return false;
        }

        out_command.command = token;

        while (cursor.next(token)) {
            if (out_command.argument_count >= max_arguments) {
                return false;
            }
            out_command.arguments[out_command.argument_count++] = token;
        }

        return true;
    }
};

// ---- routing ----------------------------------------------------------------

/// Static command router, templated on the reply sink.
///
/// Route names are matched case-insensitively (command_equals()), so the
/// canonical uppercase form is the convention, not a requirement. Handlers
/// receive the parsed command and the sink tag to answer on (compile-time
/// dispatch, no stream indirection). dispatch() calls the first route
/// whose name matches, in the caller's context, and returns true; false
/// when no route matches (nothing is called).
template <ByteSink Sink, uint8_t max_arguments = 8>
class CommandRouter {
public:
    using CommandType = ParsedCommand<max_arguments>;
    using Handler = void (*)(const CommandType &, Sink);

    struct Route {
        const char *command;
        Handler handler;
    };

    static bool dispatch(
        const CommandType &command,
        const Route *routes,
        uint8_t route_count,
        Sink sink
    ) {
        if (!command.command || !routes) {
            return false;
        }

        for (uint8_t i = 0; i < route_count; ++i) {
            if (!routes[i].command || !routes[i].handler) {
                continue;
            }
            if (command_equals(command.command, routes[i].command)) {
                routes[i].handler(command, sink);
                return true;
            }
        }
        return false;
    }
};

} // namespace brio
