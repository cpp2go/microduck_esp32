// Copyright 2026 ESP-Claw Authors
// SPDX-License-Identifier: MIT
//
// Shell — modern C++ wrapper around ESP-IDF esp_console (linenoise REPL).
//
// Usage:
//   shell::Shell sh;
//   sh.RegisterCommand({"hello", "Print greeting", "[name]",
//       [](int argc, char** argv) {
//           printf("Hello, %s!\n", argc > 1 ? argv[1] : "world");
//           return 0;
//       }});
//   sh.Start();  // Launches REPL task; returns immediately.

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <vector>

namespace shell {

// ---------------------------------------------------------------------------
// Handler
// ---------------------------------------------------------------------------

/// Handler signature matches the IDF / POSIX convention: returns 0 on success,
/// non-zero on error.  argc/argv are identical to what the shell passes in.
using Handler = std::function<int(int argc, char** argv)>;

// ---------------------------------------------------------------------------
// Command
// ---------------------------------------------------------------------------

struct Command {
    std::string_view name;   ///< Command token typed by the user, e.g. "reboot"
    std::string_view help;   ///< One-line description shown by "help"
    std::string_view hint;   ///< Argument synopsis, e.g. "<ms>" or "[name]"
    Handler          func;   ///< Callback invoked when the command is entered
};

// ---------------------------------------------------------------------------
// Config  (defined outside Shell to avoid GCC nested-class default-arg bug)
// ---------------------------------------------------------------------------

struct Config {
    /// Prompt string displayed before each input line.
    const char* prompt = "esp> ";

    /// REPL task stack size in bytes.
    uint32_t task_stack_size = 4096;

    /// REPL task FreeRTOS priority.
    int task_priority = 2;

    /// Maximum number of history entries kept in RAM.
    uint32_t max_history_len = 100;

    /// If non-null, history is loaded from / saved to this NVS namespace.
    /// Pass nullptr to disable NVS persistence.
    const char* history_nvs_namespace = nullptr;

    /// Optional banner / welcome text printed to the console after the driver
    /// is initialised but before the REPL prompt appears.
    /// Use ANSI escape codes freely — the VFS is ready at this point.
    /// Pass nullptr to skip.
    const char* welcome_message = nullptr;
};

// ---------------------------------------------------------------------------
// Execute helper
// ---------------------------------------------------------------------------

/// Result of executing one shell command line through esp_console.
struct ExecuteResult {
    bool        ok = false;      ///< true if command was dispatched successfully
    int         exit_code = 1;   ///< command return code (0 = success)
    std::string error;           ///< transport/framework error, empty on success
};

/// Execute one command line through the shell backend.
/// Example: Execute("disp fill red")
/// NOTE: Shell/console must already be started.
[[nodiscard]] ExecuteResult Execute(std::string_view line);

// ---------------------------------------------------------------------------
// Shell
// ---------------------------------------------------------------------------

class Shell {
 public:
    // Convenience alias so callers can write either shell::Config or
    // shell::Shell::Config.
    using Config = shell::Config;

    /// Create a shell with default configuration.
    Shell();

    /// Create a shell with custom configuration.
    explicit Shell(Config config);

    ~Shell();

    Shell(const Shell&) = delete;
    Shell& operator=(const Shell&) = delete;
    Shell(Shell&&) = delete;
    Shell& operator=(Shell&&) = delete;

    /// Register a command.  May be called before or after Start().
    /// Returns true on success.
    bool RegisterCommand(const Command& cmd);

    /// Start the REPL task.  Returns true if the task was launched successfully.
    /// Calling Start() more than once is a no-op (returns true if already running).
    bool Start();

    /// Stop the REPL and free resources.  Blocks until the task has exited.
    void Stop();

    /// Returns true if the REPL task is currently running.
    [[nodiscard]] bool IsRunning() const;

 private:
    struct Impl;
    Impl* impl_{nullptr};
};

}  // namespace shell
