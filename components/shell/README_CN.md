# shell

基于 ESP-IDF `esp_console` 组件（linenoise REPL）的现代 C++ 封装。

在 UART 上提供持久化的交互式命令行 shell，支持行编辑、命令历史记录，以及简洁的
`std::function` 注册 API——无需在应用代码中引入任何 IDF 头文件。

---

## 特性

- **后台任务运行 REPL** — `Start()` 立即返回，shell 在独立 FreeRTOS 任务中运行
- **Lambda / `std::function` 处理函数** — 无需裸函数指针
- **内置 `help` 命令** — 自动列出所有已注册命令及说明
- **行编辑与历史记录** — 由 `linenoise` 提供（方向键、Ctrl-R 等）
- **RAII 生命周期管理** — 析构函数自动调用 `Stop()`
- **Pimpl 模式** — IDF 头文件全部封装在 `.cpp` 内，应用代码只看到 `shell.h`

---

## 快速上手

```cpp
#include "shell/include/shell.h"

// 可选：自定义 shell 配置
shell::Shell::Config cfg;
cfg.prompt          = "claw> ";
cfg.task_stack_size = 4096;
cfg.max_history_len = 50;

shell::Shell sh(cfg);

// 在 Start() 前后均可注册命令
sh.RegisterCommand({
    .name = "reboot",
    .help = "重启设备",
    .hint = "",
    .func = [](int, char**) { esp_restart(); return 0; }
});

sh.RegisterCommand({
    .name = "info",
    .help = "打印芯片信息",
    .hint = "",
    .func = [](int, char**) {
        esp_chip_info_t info{};
        esp_chip_info(&info);
        printf("核心数 : %d\n", info.cores);
        printf("芯片型号: %d\n", static_cast<int>(info.model));
        return 0;
    }
});

sh.Start();   // 非阻塞，REPL 在独立 FreeRTOS 任务中运行
```

在终端输入 `help` 可列出所有可用命令。

---

## API 参考

### `Command`

| 字段   | 类型                              | 说明                                  |
|--------|-----------------------------------|---------------------------------------|
| `name` | `std::string_view`                | 用户输入的命令名，如 `"reboot"`        |
| `help` | `std::string_view`                | `help` 命令显示的单行描述             |
| `hint` | `std::string_view`                | 参数提示，如 `"<ms>"` 或 `"[name]"`   |
| `func` | `std::function<int(int, char**)>` | 命令处理函数，成功返回 0              |

---

### `Shell::Config`

| 字段                    | 默认值     | 说明                                         |
|-------------------------|------------|----------------------------------------------|
| `prompt`                | `"esp> "`  | 每行输入前显示的提示符                       |
| `task_stack_size`       | `4096`     | REPL 任务栈大小（字节）                      |
| `task_priority`         | `2`        | REPL 任务的 FreeRTOS 优先级                  |
| `max_history_len`       | `100`      | 内存中保留的最大历史条数                     |
| `history_nvs_namespace` | `nullptr`  | 预留 NVS 历史持久化接口（暂未实现）          |

---

### `Shell` 方法

#### `bool RegisterCommand(const Command& cmd)`

注册一条命令。可在 `Start()` 调用前或调用后注册。

- 成功返回 `true`；若命令名已存在或 `func` 为空则返回 `false`。

```cpp
sh.RegisterCommand({"hello", "打招呼", "[name]",
    [](int argc, char** argv) {
        printf("你好，%s！\n", argc > 1 ? argv[1] : "世界");
        return 0;
    }
});
```

---

#### `bool Start()`

启动 REPL 任务，非阻塞——任务启动后立即返回。

- 成功返回 `true`；已在运行时调用为无操作（仍返回 `true`）。

---

#### `void Stop()`

停止 REPL 并释放所有资源。阻塞直到任务退出。

---

#### `bool IsRunning() const`

若 REPL 任务当前正在运行则返回 `true`。

---

## CMakeLists 集成

在组件的 `REQUIRES` 或 `PRIV_REQUIRES` 中添加 `shell`：

```cmake
idf_component_register(
    SRCS "main.cpp"
    PRIV_REQUIRES shell
)
```

---

## 注意事项

- Shell 使用默认控制台 UART（通过 `menuconfig → Component config →
  ESP System Settings → Channel for console output` 配置）。
- 每个 `Shell` 实例独占控制台 UART，**不要同时创建多个** `Shell` 实例。
- 命令名区分大小写。
- `help` 命令在 `Start()` 时自动注册，无需手动添加。
