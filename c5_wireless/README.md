# ESP32-C5 无线从片（SDIO）

**C5 不跑策略。** 它作为 **esp_hosted SDIO 从机** 挂在 P4 上，WiFi/BLE 栈在 C5，遥控命令经 **peer-data** 送到 P4 `cmd_bus`。

## 1. 烧录 C5：官方 esp_hosted 从机 + Microduck 补丁

```bash
git clone https://github.com/espressif/esp-hosted-mcu.git
cd esp-hosted-mcu/slave
idf.py set-target esp32c5
```

`menuconfig`：

- **Bus** → **SDIO**（与 P4 一致，4-bit）
- 打开 **Peer data transfer**（`CONFIG_ESP_HOSTED_ENABLE_PEER_DATA_TRANSFER`）

集成补丁：

```bash
cp /path/to/microduck_esp32/c5_wireless/esp_hosted_slave_patch/microduck_cmd_publisher.c main/
cp /path/to/microduck_esp32/common/include/microduck_link.h main/
# CMakeLists.txt 的 SRCS 增加 microduck_cmd_publisher.c
# esp_hosted_coprocessor.c 启动后调用 microduck_cmd_publisher_start();
```

```bash
idf.py build flash    # 只烧 C5
```

从机预编译镜像也可能在 Component Registry / 乐鑫文档中提供 — 仍需合入 `microduck_cmd_publisher.c`。

## 2. 烧录 P4：控制固件

```bash
cd ../microduck_esp32
idf.py set-target esp32p4
idf.py menuconfig
#   ESP-Hosted → SDIO 引脚 / 开发板预设（与 C5 同板）
#   Microduck P4←C5 command link → SDIO（默认）
idf.py build flash monitor
```

P4 上电后会 `esp_hosted_connect_to_slave()`，并注册 `MICRODUCK_MSG_ID_CMD` 回调。

## 3. 帧格式

不变：`microduck_link_frame_t`（见 `common/include/microduck_link.h`），经 `esp_hosted_send_custom_data(MICRODUCK_MSG_ID_CMD, …)` 走 SDIO。

## 4. 废弃：UART 独立 `c5_wireless/main`

根目录下旧的 **UART-only** 工程仅用于无 SDIO 台架。**P4+C5 产品请只用 esp_hosted SDIO 路径。**

## 5. 引脚

SDIO CMD/CLK/D0–D3 与 C5 RESET 在 **P4 `menuconfig` → ESP-Hosted → SDIO** 和 **C5 slave menuconfig** 中成对配置。  
使用 **ESP32-P4-C5 Core Board** 时可勾选 `CONFIG_ESP_HOSTED_P4_C5_CORE_BOARD`（P4 侧）。