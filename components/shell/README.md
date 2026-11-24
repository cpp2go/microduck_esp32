# shell

A modern C++ wrapper around the ESP-IDF `esp_console` component (linenoise REPL).

Provides a persistent interactive command-line shell over UART with line editing,
command history, and a clean `std::function`-based registration API — without
leaking any IDF headers into your application code.

---

## Features

- **REPL in a background task** — `Start()` returns immediately; the shell runs independently.
- **Lambda / `std::function` handlers** — no raw function pointers required.
- **Built-in `help` command** — lists all registered commands with descriptions.
- **Line editing & history** — powered by `linenoise` (arrow keys, Ctrl-R, etc.).
- **RAII lifecycle** — destructor calls `Stop()` automatically.
- **Pimpl pattern** — IDF headers stay inside the `.cpp`, your code only sees `shell.h`.

---

## Quick Start

```cpp
#include "shell/include/shell.h"

// Optional: customise the shell
shell::Shell::Config cfg;
cfg.prompt          = "claw> ";
cfg.task_stack_size = 4096;
cfg.max_history_len = 50;

shell::Shell sh(cfg);

// Register commands before or after Start()
sh.RegisterCommand({
    .name = "reboot",
    .help = "Reboot the device",
    .hint = "",
    .func = [](int, char**) { esp_restart(); return 0; }
});

sh.RegisterCommand({
    .name = "info",
    .help = "Print chip information",
    .hint = "",
    .func = [](int, char**) {
        esp_chip_info_t info{};
        esp_chip_info(&info);
        printf("Cores : %d\n", info.cores);
        printf("Model : %d\n", static_cast<int>(info.model));
        return 0;
    }
});

sh.Start();   // Non-blocking; REPL runs in its own FreeRTOS task
```

Type `help` in the terminal to list all available commands.

---

## API Reference

### `Command`

| Field  | Type                         | Description                          |
|--------|------------------------------|--------------------------------------|
| `name` | `std::string_view`           | Token the user types, e.g. `"reboot"` |
| `help` | `std::string_view`           | One-line description shown by `help`  |
| `hint` | `std::string_view`           | Argument synopsis, e.g. `"<ms>"`     |
| `func` | `std::function<int(int, char**)>` | Handler; return 0 for success   |

---

### `Shell::Config`

| Field                    | Default    | Description                                      |
|--------------------------|------------|--------------------------------------------------|
| `prompt`                 | `"esp> "`  | Prompt string displayed before each input line   |
| `task_stack_size`        | `4096`     | REPL task stack size in bytes                    |
| `task_priority`          | `2`        | REPL task FreeRTOS priority                      |
| `max_history_len`        | `100`      | Maximum number of history entries kept in RAM    |
| `history_nvs_namespace`  | `nullptr`  | Reserved for NVS history persistence (future)    |

---

### `Shell` Methods

#### `bool RegisterCommand(const Command& cmd)`

Register a command. Can be called before or after `Start()`.

- Returns `true` on success, `false` if the name is already taken or `func` is null.

```cpp
sh.RegisterCommand({"hello", "Greet someone", "[name]",
    [](int argc, char** argv) {
        printf("Hello, %s!\n", argc > 1 ? argv[1] : "world");
        return 0;
    }
});
```

---

#### `bool Start()`

Start the REPL task. Non-blocking — returns immediately after spawning the task.

- Returns `true` on success.
- Calling `Start()` when already running is a no-op (returns `true`).

---

#### `void Stop()`

Stop the REPL and free all resources. Blocks until the task has exited.

---

#### `bool IsRunning() const`

Returns `true` if the REPL task is currently running.

---

## CMakeLists Integration

Add `shell` to your component's `REQUIRES` or `PRIV_REQUIRES`:

```cmake
idf_component_register(
    SRCS "main.cpp"
    PRIV_REQUIRES shell
)
```

---

## Notes

- The shell runs on the default console UART (configured via `menuconfig →
  Component config → ESP System Settings → Channel for console output`).
- Each `Shell` instance owns the UART console; do **not** create more than one
  `Shell` at the same time.
- Command names are case-sensitive.
- The built-in `help` command is registered automatically on `Start()`.
