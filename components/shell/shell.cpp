// Copyright 2026 ESP-Claw Authors
// SPDX-License-Identifier: MIT

#include "shell.h"

#include <cassert>
#include <cstring>
#include <list>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

namespace shell {

static constexpr const char* kTag = "shell";

ExecuteResult Execute(std::string_view line) {
    ExecuteResult result;
    if (line.empty()) {
        result.error = "empty command";
        return result;
    }

    std::string command(line);
    int cmd_ret = 1;
    esp_err_t err = esp_console_run(command.c_str(), &cmd_ret);
    if (err != ESP_OK) {
        result.error = esp_err_to_name(err);
        result.exit_code = 1;
        return result;
    }

    result.ok = true;
    result.exit_code = cmd_ret;
    return result;
}

// ---------------------------------------------------------------------------
// Impl — private state
// ---------------------------------------------------------------------------

struct Shell::Impl {
    Config              config;
    esp_console_repl_t* repl{nullptr};

    // We keep our own copy of registered commands so the esp_console_cmd_t
    // structs (which hold raw const char* pointers) remain valid for the
    // lifetime of the Shell.
    struct CmdEntry {
        std::string         name;
        std::string         help;
        std::string         hint;
        Handler             func;
        esp_console_cmd_t   raw{};
    };
    // std::list gives stable element addresses — critical because
    // esp_console_cmd_register stores raw char* pointers into entry.name /
    // entry.help.  std::vector reallocation would move SSO strings and leave
    // esp_console holding dangling pointers, causing "command not found".
    std::list<CmdEntry> commands;

    // Trampoline — bridges the raw C callback into the stored std::function.
    // The IDF console stores a single void* user data via cmd.func_w_context;
    // we use that to carry a pointer to the CmdEntry.
    static int CommandTrampoline(void* ctx, int argc, char** argv) {
        auto* entry = static_cast<CmdEntry*>(ctx);
        if (!entry || !entry->func) return 1;
        return entry->func(argc, argv);
    }
};

// ---------------------------------------------------------------------------
// Shell
// ---------------------------------------------------------------------------

Shell::Shell() : Shell(Config{}) {}

Shell::Shell(Config config) : impl_(new Impl{}) {
    impl_->config = config;
}

Shell::~Shell() {
    Stop();
    delete impl_;
}

bool Shell::RegisterCommand(const Command& cmd) {
    if (!cmd.func) {
        ESP_LOGE(kTag, "RegisterCommand: null handler for '%.*s'",
                 static_cast<int>(cmd.name.size()), cmd.name.data());
        return false;
    }

    auto& entry = impl_->commands.emplace_back();
    entry.name = std::string(cmd.name);
    entry.help = std::string(cmd.help);
    entry.hint = cmd.hint.empty() ? std::string{} : std::string(cmd.hint);
    entry.func = cmd.func;

    entry.raw.command        = entry.name.c_str();
    entry.raw.help           = entry.help.c_str();
    entry.raw.hint           = entry.hint.empty() ? nullptr : entry.hint.c_str();
    entry.raw.func           = nullptr;           // not used when func_w_context is set
    entry.raw.func_w_context = &Impl::CommandTrampoline;
    entry.raw.context        = &entry;

    esp_err_t err = esp_console_cmd_register(&entry.raw);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_console_cmd_register('%s') failed: %s",
                 entry.name.c_str(), esp_err_to_name(err));
        impl_->commands.pop_back();
        return false;
    }
    ESP_LOGD(kTag, "Registered command: %s", entry.name.c_str());
    return true;
}

bool Shell::Start() {
    if (IsRunning()) return true;

    // NOTE: esp_console_new_repl_uart internally calls:
    //   uart_driver_install → esp_console_common_init (→ esp_console_init +
    //   esp_console_register_help_command) → linenoise setup.
    // We must NOT call any of those ourselves before this point.

    // ---- REPL config ----
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt             = impl_->config.prompt;
    repl_cfg.max_history_len    = impl_->config.max_history_len;
    repl_cfg.task_stack_size    = impl_->config.task_stack_size;
    repl_cfg.task_priority      = impl_->config.task_priority;
    repl_cfg.max_cmdline_length = 256;   // 0 → use default (256); explicit for clarity

    if (impl_->config.history_nvs_namespace) {
        ESP_LOGW(kTag, "NVS history persistence not implemented yet");
    }

    // ---- Create REPL — backend selected by sdkconfig ----
    // esp_console_new_repl_xxx internally handles driver install + esp_console_init.
    // Do NOT call esp_console_init / uart_driver_install ourselves.
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t hw_cfg =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    esp_err_t err = esp_console_new_repl_usb_serial_jtag(&hw_cfg, &repl_cfg, &impl_->repl);
#else
    esp_console_dev_uart_config_t hw_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    esp_err_t err = esp_console_new_repl_uart(&hw_cfg, &repl_cfg, &impl_->repl);
#endif
    if (err != ESP_OK) {
        // On failure, esp_console_new_repl_xxx already called esp_console_deinit()
        // internally — do NOT deinit again here.
        esp_rom_printf("[shell] esp_console_new_repl failed: 0x%x (%s)\n",
                       (int)err, esp_err_to_name(err));
        impl_->repl = nullptr;
        return false;
    }

    // Print custom welcome banner now that the console VFS is ready.
    // Must happen BEFORE esp_console_start_repl so the banner appears
    // before the first prompt.
    if (impl_->config.welcome_message) {
        printf("%s", impl_->config.welcome_message);
    }

    err = esp_console_start_repl(impl_->repl);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_console_start_repl failed: %s", esp_err_to_name(err));
        impl_->repl->del(impl_->repl);  // also calls esp_console_deinit internally
        impl_->repl = nullptr;
        return false;
    }

    ESP_LOGI(kTag, "REPL started (prompt=\"%s\")", impl_->config.prompt);
    return true;
}

void Shell::Stop() {
    if (!IsRunning()) return;

    // IDF 5.x does not provide an esp_console_stop_repl(); deleting the REPL
    // object is the documented way to stop it.
    // repl->del calls esp_console_deinit + uart_driver_delete internally.
    esp_err_t err = impl_->repl->del(impl_->repl);
    if (err != ESP_OK) {
        ESP_LOGW(kTag, "repl->del failed: %s", esp_err_to_name(err));
    }
    impl_->repl = nullptr;
    ESP_LOGI(kTag, "REPL stopped");
}

bool Shell::IsRunning() const {
    return impl_ && impl_->repl != nullptr;
}

}  // namespace shell
