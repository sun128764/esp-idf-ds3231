
#include "private.h"
#define DS3231_ADDR 0xD0

esp_err_t ds3231_io_read(i2c_port_t port, uint8_t addr, void *data, size_t sz)
{
    i2c_cmd_handle_t cmd;
    esp_err_t retval;

    if ((cmd = i2c_cmd_link_create()))
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, DS3231_ADDR | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, addr, true);

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, DS3231_ADDR | I2C_MASTER_READ, true);
        i2c_master_read(cmd, data, sz, I2C_MASTER_LAST_NACK);

        i2c_master_stop(cmd);

        retval = i2c_master_cmd_begin(port, cmd, 20);
        i2c_cmd_link_delete(cmd);
        return retval;
    }

    return ESP_ERR_NO_MEM;
}

esp_err_t ds3231_io_write(i2c_port_t port, uint8_t addr, const void *data, size_t sz)
{
    i2c_cmd_handle_t cmd;
    esp_err_t retval;

    if ((cmd = i2c_cmd_link_create()))
    {
        for (int i = 0; i < sz; ++ i)
        {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, DS3231_ADDR | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, addr, true);
            i2c_master_write_byte(cmd, ((uint8_t*)data)[i], true);
        }
        i2c_master_stop(cmd);

        retval = i2c_master_cmd_begin(port, cmd, 20);
        i2c_cmd_link_delete(cmd);
        return retval;
    }

    return ESP_ERR_NO_MEM;
}
