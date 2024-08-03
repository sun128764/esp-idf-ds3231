
#include "private.h"
#include "string.h"
#define DS3231_ADDR 0x68
#define I2C_SPEED_HZ 100000
#define DEBUG false

esp_err_t ds3231_io_read(i2c_port_num_t port, gpio_num_t scl, gpio_num_t sda, uint8_t addr, uint8_t *data, size_t sz)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    esp_err_t retval;

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg_mode = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_ADDR,
        .scl_speed_hz = I2C_SPEED_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_mode, &dev_handle));

    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &addr, sizeof(addr), data, sz, -1));

    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));

    if (DEBUG)
    {
        printf("READ addr: %d\n", addr);
        printf("data: ");
        for (int i = 0; i < sz; i++)
        {
            printf("%d ", *(data + i));
        }
        printf("\n");
    }

    return ESP_OK;
}

esp_err_t ds3231_io_write(i2c_port_num_t port, gpio_num_t scl, gpio_num_t sda, uint8_t addr, const uint8_t *data, size_t sz)
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    esp_err_t retval;

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = port,
        .scl_io_num = scl,
        .sda_io_num = sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg_mode = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_ADDR,
        .scl_speed_hz = I2C_SPEED_HZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg_mode, &dev_handle));
    uint8_t *buf = (uint8_t *)malloc(sz + 1);

    if (buf == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    buf[0] = addr;
    memcpy(buf + 1, data, sz);

    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, sz + 1, -1));

    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));

    if (DEBUG)
    {
        printf("WRITE addr: %d\n", addr);
        printf("data: ");
        for (int i = 0; i < sz + 1; i++)
        {
            printf("%d ", *(buf + i));
        }
        printf("\n");
    }

    free(buf);

    return ESP_OK;
}
