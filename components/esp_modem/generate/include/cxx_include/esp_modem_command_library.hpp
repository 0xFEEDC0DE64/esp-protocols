/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "cxx_include/esp_modem_dte.hpp"
#include "cxx_include/esp_modem_dce_module.hpp"
#include "cxx_include/esp_modem_types.hpp"

//  --- ESP-MODEM command module starts here ---
namespace esp_modem {
namespace dce_commands {

/**
 * @defgroup ESP_MODEM_DCE_COMMAND ESP_MODEM DCE command library
 * @brief Library of the most useful DCE commands
 */
/** @addtogroup ESP_MODEM_DCE_COMMAND
 * @{
 */

/**
 * @brief Generic AT command to be send with pass and fail phrases
 *
 * @param t Commandable object (anything that can accept commands)
 * @param command Command to be sent do the commandable object
 * @param pass_phrase String to be present in the reply to pass this command
 * @param fail_phrase String to be present in the reply to fail this command
 * @param timeout_ms Timeout in ms
 */
command_result generic_command(CommandableIf *t, std::string_view command,
                               std::string_view pass_phrase,
                               std::string_view fail_phrase, uint32_t timeout_ms);

/**
 * @brief Declaration of all commands is generated from esp_modem_command_declare.inc
 */
#include "esp_modem_command_declare_helper.inc"
#define ESP_MODEM_DECLARE_DCE_COMMAND(name, return_type, ...) \
        return_type name(CommandableIf *t  ESP_MODEM_COMMAND_PARAMS_AFTER(__VA_ARGS__));

#include "esp_modem_command_declare.inc"

#undef ESP_MODEM_DECLARE_DCE_COMMAND

/**
 * @brief Following commands that are different for some specific modules
 */
command_result get_battery_status_sim7xxx(CommandableIf *t, int &voltage, int &bcs, int &bcl);
command_result set_gnss_power_mode_sim76xx(CommandableIf *t, int mode);
command_result power_down_sim76xx(CommandableIf *t);
command_result power_down_sim70xx(CommandableIf *t);
command_result set_network_bands_sim76xx(CommandableIf *t, const std::string &mode, const int *bands, int size);
command_result power_down_sim8xx(CommandableIf *t);
command_result set_data_mode_alt(CommandableIf *t);
command_result set_pdp_context(CommandableIf *t, PdpContext &pdp, uint32_t timeout_ms);

/**
 * @}
 */

} // dce_commands
} // esp_modem
