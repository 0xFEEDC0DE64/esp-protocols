#pragma once

#include <cstddef>
#include <cstdint>
#include <cctype>
#include <array>
#include "esp_log.h"

namespace esp_modem {

inline void log_buffer_escaped(esp_log_level_t level, const char * tag, const char * prefix, const uint8_t *data, size_t data_len) {
    std::array<uint8_t, 128> line_buffer{};
    std::array<uint8_t, 4> hex_buffer{ '\\', 'x', '0', '0' };
    size_t line_index = 0;

    auto flush = [&] {
        if (line_index > 0) {
            ESP_LOG_LEVEL(level, tag, "%s\"%.*s\"", prefix, line_index, line_buffer.data());
            line_index = 0;
        }
    };
    auto write_chars = [&](const uint8_t *data, size_t size) {
        if (line_index + size > line_buffer.size()) {
            flush();
        }

        memcpy(&line_buffer[line_index], data, size);
        line_index += size;
    };
    auto write_str = [&]<size_t N>(const char (&str)[N]) {
        write_chars((const uint8_t*)str, N-1);
    };
    auto write_array = [&]<size_t N>(const std::array<uint8_t, N>& arr) {
        write_chars(arr.data(), arr.size());
    };
    auto write_char = [&](uint8_t c) {
        write_chars(&c, 1);
    };
    auto nibble2hex = [](uint8_t nibble) { return "0123456789abcdef"[nibble]; };

    for (size_t data_index = 0; data_index < data_len; data_index++) {
        uint8_t c = data[data_index];
        if (isprint(c)) { write_char(c); }
        else if (c == '\r') { write_str("\\r"); }
        else if (c == '\n') { write_str("\\n"); }
        else { hex_buffer[2] = nibble2hex(c >> 4); hex_buffer[3] = nibble2hex(c & 0xf); write_array(hex_buffer); }
    }
    flush();
}

}
