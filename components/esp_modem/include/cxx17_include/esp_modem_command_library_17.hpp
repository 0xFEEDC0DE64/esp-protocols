/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string>
#include <list>
#include "esp_log.h"
#include "cxx_include/esp_modem_dte.hpp"
#include "cxx_include/esp_modem_dce_module.hpp"

namespace esp_modem::dce_commands {
command_result generic_command(CommandableIf *t, std::string_view command,
                               std::span<const std::string_view> pass_phrase,
                               std::span<const std::string_view> fail_phrase,
                               uint32_t timeout_ms);
}
