/*
 * Flat C API over the VL53L8CX ULD — ESP-IDF port of microduck tof/vendor shim.c.
 * Uses ESP I2C master instead of Linux /dev/i2c.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "vl53l8cx_api.h"

static VL53L8CX_Configuration dev;
static VL53L8CX_ResultsData results;
static bool s_opened;

int vl8_open_i2c(i2c_master_bus_handle_t bus, uint8_t addr_7bit)
{
    memset(&dev, 0, sizeof(dev));
    if (!bus) {
        return -1;
    }
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_7bit,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &cfg, &dev.platform.i2c_dev) != ESP_OK) {
        return -1;
    }
    dev.platform.i2c_bus = bus;
    dev.platform.owns_bus = 0;
    dev.platform.address = (uint16_t)(addr_7bit << 1);
    s_opened = true;
    return 0;
}

int vl8_open(int i2c_port, int sda_gpio, int scl_gpio, uint8_t addr_7bit)
{
    memset(&dev, 0, sizeof(dev));
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = i2c_port,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &dev.platform.i2c_bus) != ESP_OK) {
        return -1;
    }
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_7bit,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(dev.platform.i2c_bus, &cfg, &dev.platform.i2c_dev) != ESP_OK) {
        i2c_del_master_bus(dev.platform.i2c_bus);
        return -1;
    }
    dev.platform.owns_bus = 1;
    dev.platform.address = (uint16_t)(addr_7bit << 1);
    s_opened = true;
    return 0;
}

void vl8_close(void)
{
    if (!s_opened) {
        return;
    }
    if (dev.platform.i2c_dev) {
        i2c_master_bus_rm_device(dev.platform.i2c_dev);
        dev.platform.i2c_dev = NULL;
    }
    if (dev.platform.owns_bus && dev.platform.i2c_bus) {
        i2c_del_master_bus(dev.platform.i2c_bus);
        dev.platform.i2c_bus = NULL;
    }
    s_opened = false;
}

int vl8_is_alive(void)
{
    if (!s_opened) {
        return 0;
    }
    uint8_t alive = 0;
    if (vl53l8cx_is_alive(&dev, &alive)) {
        return 0;
    }
    return alive ? 1 : 0;
}

int vl8_init(void)
{
    if (!s_opened) {
        return -1;
    }
    return (int)vl53l8cx_init(&dev);
}

int vl8_start(uint8_t freq_hz)
{
    uint8_t s = vl53l8cx_set_resolution(&dev, VL53L8CX_RESOLUTION_8X8);
    if (s) {
        return (int)s;
    }
    s = vl53l8cx_set_ranging_frequency_hz(&dev, freq_hz);
    if (s) {
        return (int)s;
    }
    return (int)vl53l8cx_start_ranging(&dev);
}

int vl8_stop(void) { return (int)vl53l8cx_stop_ranging(&dev); }

int vl8_data_ready(void)
{
    uint8_t ready = 0;
    if (vl53l8cx_check_data_ready(&dev, &ready)) {
        return -1;
    }
    return ready ? 1 : 0;
}

int vl8_get_frame(int16_t *dist_mm, uint8_t *status)
{
    uint8_t s = vl53l8cx_get_ranging_data(&dev, &results);
    if (s) {
        return (int)s;
    }
    memcpy(dist_mm, results.distance_mm, 64 * sizeof(int16_t));
    memcpy(status, results.target_status, 64);
    return 0;
}
