#include "private.h"
#include <errno.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define F_INIT (1 << 0)

struct ds3231_t
{
    int flags;
    i2c_port_num_t port;
    int sda_gpio;
    int scl_gpio;
    int rw_timeout_ms;
    uint32_t clock_speed_hz;
};

#define ASSERT_DRV()                                     \
    do                                                   \
    {                                                    \
        if (driver == NULL || !(driver->flags & F_INIT)) \
        {                                                \
            errno = EINVAL;                              \
            return -1;                                   \
        }                                                \
    } while (0)

DS3231_API ds3231_t ds3231_create(i2c_port_num_t port, int sda_gpio, int scl_gpio, int rw_timeout_ms, uint32_t clock_speed_hz)
{
    if (port > I2C_NUM_MAX)
    {
        errno = EINVAL;
        return NULL;
    }

    ds3231_t driver = malloc(sizeof(struct ds3231_t));
    if (driver)
    {
        driver->flags = 0;
        driver->port = port;
        driver->sda_gpio = sda_gpio;
        driver->scl_gpio = scl_gpio;
        driver->rw_timeout_ms = rw_timeout_ms;
        driver->clock_speed_hz = clock_speed_hz;
    }
    return driver;
}

DS3231_API void ds3231_destroy(ds3231_t driver)
{
    free(driver);
}

DS3231_API int ds3231_initialize(ds3231_t driver, int *opt_out_osf)
{
    if (driver == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    // Read status regıister
    uint8_t status;
    if (ds3231_io_read(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_STATUS, &status, sizeof status))
    {
        errno = EIO;
        return -1;
    }

    if (opt_out_osf)
    {
        *opt_out_osf = !!(status & DS3231_STATUS_OSF);
    }

    driver->flags |= F_INIT;

    int err;
    {
        // Wait until busy flag clears.
        ds3231_status_t status;
        while (!(err = ds3231_getStatus(driver, &status)) && (status & DS3231_STATUS_BSY))
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    return err;
}

DS3231_API int ds3231_setSquareWaveOutput(ds3231_t driver, ds3231_rate_t rate, int battery_backed)
{
    ASSERT_DRV();

    uint8_t control = battery_backed ? DS3231_CONTROL_BBSQW : 0;
    switch (rate)
    {
    default:
        errno = EINVAL;
        return -1;
    case DS3231_RATE_1HZ:
        control |= DS3231_CONTROL_RS_1HZ;
        break;
    case DS3231_RATE_1024HZ:
        control |= DS3231_CONTROL_RS_1024HZ;
        break;
    case DS3231_RATE_4096HZ:
        control |= DS3231_CONTROL_RS_4096HZ;
        break;
    case DS3231_RATE_8192HZ:
        control |= DS3231_CONTROL_RS_8192HZ;
        break;
    }

    if (ds3231_io_write(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_CONTROL, &control, sizeof control))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

DS3231_API int ds3231_setInterrupt(ds3231_t driver, int alarm1, int alarm2)
{
    ASSERT_DRV();

    uint8_t control =
        DS3231_CONTROL_INTCN |
        (alarm1 ? DS3231_CONTROL_A1IE : 0) |
        (alarm2 ? DS3231_CONTROL_A2IE : 0);

    if (ds3231_io_write(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_CONTROL, &control, sizeof control))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

DS3231_API int ds3231_getStatus(ds3231_t driver, ds3231_status_t *out_status)
{
    ASSERT_DRV();

    if (out_status == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    uint8_t rstatus;
    if (ds3231_io_read(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_STATUS, &rstatus, sizeof rstatus))
    {
        errno = EIO;
        return -1;
    }

    *out_status = (ds3231_status_t)rstatus;

    return 0;
}

DS3231_API int ds3231_getAgingOffset(ds3231_t driver, u_int8_t *out_offset)
{
    ASSERT_DRV();

    if (out_offset == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    if (ds3231_io_read(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_OFFSET, out_offset, sizeof *out_offset))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

DS3231_API int ds3231_setAgingOffset(ds3231_t driver, u_int8_t offset)
{
    ASSERT_DRV();

    if (ds3231_io_write(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_OFFSET, &offset, sizeof offset))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

inline static void ds3231_fill_yday(struct tm *time)
{
    time->tm_yday = time->tm_mday;
    switch (time->tm_mon)
    {
    case 0: // Jan
        break;
    case 1: // Feb
        time->tm_yday += 31;
        break;
    case 2: // Mar
        time->tm_yday += 59;
        break;
    case 3: // Apr
        time->tm_yday += 90;
        break;
    case 4: // May
        time->tm_yday += 120;
        break;
    case 5: // Jun
        time->tm_yday += 151;
        break;
    case 6: // Jul
        time->tm_yday += 181;
        break;
    case 7: // Aug
        time->tm_yday += 212;
        break;
    case 8: // Sep
        time->tm_yday += 243;
        break;
    case 9: // Oct
        time->tm_yday += 273;
        break;
    case 10: // Nov
        time->tm_yday += 304;
        break;
    case 11: // Dec
        time->tm_yday += 334;
        break;
    default:
        break;
    }

    if ((time->tm_year % 4) || (time->tm_year == 2100))
    {
        time->tm_yday -= 1;
    }
}

DS3231_API int ds3231_getTime(ds3231_t driver, struct tm *time)
{
    uint8_t rtime[7];

    ASSERT_DRV();

    if (time == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    if (ds3231_io_read(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_SECONDS, rtime, sizeof rtime))
    {
        errno = EIO;
        return -1;
    }

    time->tm_sec = BCD_TO_DEC(rtime[0]);
    time->tm_min = BCD_TO_DEC(rtime[1]);
    if (rtime[2] & (1 << 6))
    {
        uint8_t bcd = DS3231_12_TO_24(rtime[2]);
        time->tm_hour = BCD_TO_DEC(bcd);
    }
    else
    {
        time->tm_hour = BCD_TO_DEC(rtime[2]);
    }
    time->tm_wday = rtime[3];
    time->tm_mday = BCD_TO_DEC(rtime[4]);
    time->tm_mon = BCD_TO_DEC((rtime[5] & 0x1F)) - 1;
    time->tm_year = 100 + BCD_TO_DEC(rtime[6]) + ((rtime[5] & (1 << 7)) ? 100 : 0);
    ds3231_fill_yday(time);

    return 0;
}

DS3231_API int ds3231_setTime(ds3231_t driver, const struct tm *time)
{
    ASSERT_DRV();

    if (time == NULL)
    {
        errno = EIO;
        return -1;
    }

    uint8_t rtime[7];
    rtime[0] = DEC_TO_BCD(time->tm_sec);
    rtime[1] = DEC_TO_BCD(time->tm_min);
    rtime[2] = DEC_TO_BCD(time->tm_hour);
    rtime[3] = time->tm_wday;
    rtime[4] = DEC_TO_BCD(time->tm_mday);
    rtime[5] = DEC_TO_BCD(time->tm_mon + 1) + ((time->tm_year >= 200) ? (1 << 7) : 0);
    rtime[6] = DEC_TO_BCD(time->tm_year % 100);
    if (ds3231_io_write(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_SECONDS, rtime, sizeof rtime))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

DS3231_API int ds3231_setAlarm1(ds3231_t driver, ds3231_alarmType_t type, const struct tm *time)
{
    ASSERT_DRV();

    if (type > DS3231_ALARM1_SMHWD || time == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    uint8_t rtime[4];
    rtime[0] = DEC_TO_BCD(time->tm_sec);
    rtime[1] = DEC_TO_BCD(time->tm_min);
    rtime[2] = DEC_TO_BCD(time->tm_hour);
    switch (type)
    {
    case DS3231_ALARM1_PERSEC:
        rtime[0] |= (1 << 7);
    case DS3231_ALARM1_S:
        rtime[1] |= (1 << 7);
    case DS3231_ALARM1_SM:
        rtime[2] |= (1 << 7);
    case DS3231_ALARM1_SMH:
        rtime[3] = (1 << 7);
        break;
    case DS3231_ALARM1_SMHWD: // Week day.
        rtime[3] = (1 << 6) | DEC_TO_BCD(time->tm_wday);
        break;
    case DS3231_ALARM1_SMHMD: // Month day.
        rtime[3] = DEC_TO_BCD(time->tm_mday);
        break;
    }

    if (ds3231_io_write(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_ALARM1_SECONDS, rtime, sizeof rtime))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

DS3231_API int ds3231_setAlarm2(ds3231_t driver, ds3231_alarmType_t type, const struct tm *time)
{
    ASSERT_DRV();

    if (type > DS3231_ALARM2_MHWD || time == NULL)
    {
        errno = EIO;
        return -1;
    }

    uint8_t rtime[3];
    rtime[0] = DEC_TO_BCD(time->tm_min);
    rtime[1] = DEC_TO_BCD(time->tm_hour);
    switch (type)
    {
    case DS3231_ALARM2_PREMIN:
        rtime[0] |= (1 << 7);
    case DS3231_ALARM2_M:
        rtime[1] |= (1 << 7);
    case DS3231_ALARM2_MH:
        rtime[2] = (1 << 7);
        break;
    case DS3231_ALARM2_MHMD:
        rtime[2] = DEC_TO_BCD(time->tm_mday);
        break;
    case DS3231_ALARM2_MHWD:
        rtime[2] = (1 << 6) | DEC_TO_BCD(time->tm_wday);
        break;
    default:
        errno = EINVAL;
        return -1;
    }

    if (ds3231_io_write(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_ALARM2_MINUTES, rtime, sizeof rtime))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

DS3231_API int ds3231_beginTemperature(ds3231_t driver)
{
    ASSERT_DRV();

    uint8_t control;
    if (ds3231_io_read(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_CONTROL, &control, sizeof control))
    {
        errno = EIO;
        return -1;
    }

    control |= DS3231_CONTROL_CONV;
    if (ds3231_io_write(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_CONTROL, &control, sizeof control))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

DS3231_API int ds3231_endTemperature(ds3231_t driver, int16_t *out_temperature)
{
    ASSERT_DRV();

    if (out_temperature == NULL)
    {
        errno = EIO;
        return -1;
    }

    uint8_t status = 0xFF;
    uint8_t tempr[2];

    while (
        !ds3231_io_read(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_CONTROL, &status, sizeof status) &&
        (status & DS3231_RSTATUS_BSY))
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        status = 0xFF;
    }

    if (
        (status & DS3231_RSTATUS_BSY) ||
        ds3231_io_read(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_TEMP_H, tempr, sizeof tempr))
    {
        errno = EIO;
        return -1;
    }

    *out_temperature = (int16_t)((tempr[0] << 8) | (tempr[1]));
    *out_temperature >>= 6;

    return 0;
}

int ds3231_clearInt(ds3231_t driver)
{
    ASSERT_DRV();

    uint8_t status = (1 << 3);
    if (ds3231_io_write(driver->port, driver->scl_gpio, driver->sda_gpio, DS3231_REGISTER_STATUS, &status, sizeof status))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}
