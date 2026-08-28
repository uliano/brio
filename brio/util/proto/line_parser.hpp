/*
 * proto/line_parser.hpp
 *
 * Line assembly and command parsing, protocol layer on top of the byte
 * transports. Rewritten 07/2026 from the AVR-Multislope pull-based design
 * following the framework decisions:
 *
 *  - PUSH model: LineAssembler is fed one byte at a time and returns the
 *    completed line - it holds no reference to any transport, so it works
 *    with any source (UART ring, test vector, host-side unit test);
 *  - the virtual endpoint layer (IParserEndpoint / StreamParserEndpoint /
 *    ParserHub) is gone: with sinks and sources resolved at compile time
 *    the glue is a few lines in the app's main loop (see below);
 *  - the CommandRouter is templated on the reply sink (brio::ByteSink) so
 *    handlers can answer without any runtime stream indirection.
 *
 * Typical glue in an app:
 *
 *   brio::LineAssembler<80> line_in;
 *   ...
 *   uint8_t b;
 *   while (Serial::read_byte(b)) {
 *       if (char *line = line_in.push(b)) {
 *           Cmd cmd;
 *           if (brio::ConsoleCommandParser<>::parse(line, cmd)) {
 *               Router::dispatch(cmd, routes, n_routes, serial);
 *           }
 *       }
 *   }
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "util/stream.hpp"

namespace brio {

// ---- character / number helpers ---------------------------------------------

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

/**
 * @brief Assembles newline-terminated lines from a pushed byte stream.
 *
 * - '\r' is ignored
 * - '\n' completes the line: push() returns the NUL-terminated text
 * - on overflow the line is discarded and bytes are dropped until the
 *   next '\n' (counted in overflow_count)
 *
 * The returned pointer refers to the internal buffer and stays valid until
 * the next push(): consume (or copy) the line before feeding more bytes.
 */
template <uint8_t max_line_length = 96>
class LineAssembler {
    static_assert(max_line_length >= 4, "line buffer should be at least 4 chars");

private:
    char m_line[max_line_length]{};
    uint8_t m_length{0};
    bool m_drop_until_eol{false};
    uint8_t m_overflow_count{0};

public:
    /// Feed one byte; returns the completed line or nullptr.
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

    uint8_t overflow_count() const { return m_overflow_count; }
    void clear_counters() { m_overflow_count = 0; }
};

// ---- tokenization -----------------------------------------------------------

/**
 * @brief Non-owning tokenizer on top of a mutable C string.
 *
 * Tokenization happens in place by replacing separators with '\0'.
 */
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

/**
 * @brief Generic parsed command: command word + argument pointers.
 *
 * The command string and argument pointers refer to memory inside the line
 * buffer (LineAssembler): consume them before pushing more bytes.
 */
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

/**
 * @brief Console-style parser: "CMD arg1 arg2", spaces/tabs separate.
 * The command word is uppercased in place.
 */
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

/**
 * @brief Minimal SCPI-like parser: ":SUB:SYS:CMD? arg1,arg2".
 *
 * Separators: spaces, tabs, commas. Normalization: command uppercased in
 * place, optional leading ':' removed, trailing '?' sets is_query.
 */
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

/**
 * @brief Static command router, templated on the reply sink.
 *
 * Route names should be provided in canonical uppercase form. Handlers
 * receive the parsed command and the sink tag to answer on (compile-time
 * dispatch, no stream indirection).
 */
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
