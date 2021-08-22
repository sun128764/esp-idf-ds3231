#ifndef _ESP_IDF_DS3231_H_
#define _ESP_IDF_DS3231_H_

#define DS3231_API

#include <driver/i2c.h>
#include <time.h>
#include <errno.h>

typedef struct ds3231_t *ds3231_t;

typedef enum ds3231_alarmType_t 
{
    DS3231_ALARM1_PERSEC = 0,
    DS3231_ALARM1_S,
    DS3231_ALARM1_SM,
    DS3231_ALARM1_SMH,
    DS3231_ALARM1_SMHMD,
    DS3231_ALARM1_SMHWD,

    DS3231_ALARM2_PREMIN = 0,
    DS3231_ALARM2_M,
    DS3231_ALARM2_MH,
    DS3231_ALARM2_MHMD,
    DS3231_ALARM2_MHWD
} ds3231_alarmType_t;

typedef enum ds3231_rate_t 
{
    DS3231_RATE_1HZ = 0,
    DS3231_RATE_1024HZ = 1,
    DS3231_RATE_4096HZ = 2,
    DS3231_RATE_8192HZ = 3
} ds3231_rate_t;

typedef enum ds3231_status_t
{
    DS3231_STATUS_OSF = 1 << 7,
    DS3231_STATUS_BSY = 1 << 2,
    DS3231_STATUS_AL2 = 1 << 1,
    DS3231_STATUS_AL1 = 1 << 0,
} ds3231_status_t;

DS3231_API ds3231_t ds3231_create(i2c_port_t port);

DS3231_API void ds3231_destroy(ds3231_t driver);

DS3231_API int ds3231_initialize(ds3231_t driver, int *opt_out_osf);

DS3231_API int ds3231_setSquareWaveOutput(ds3231_t driver, ds3231_rate_t rate, int battery_backed);

DS3231_API int ds3231_setInterrupt(ds3231_t driver, int alarm1, int alarm2);

DS3231_API int ds3231_getStatus(ds3231_t driver, ds3231_status_t *out_status);

DS3231_API int ds3231_clearInt(ds3231_t driver);

DS3231_API int ds3231_getAgingOffset(ds3231_t driver, int8_t *out_offset);

DS3231_API int ds3231_setAgingOffset(ds3231_t driver, int8_t offset);

DS3231_API int ds3231_getTime(ds3231_t driver, struct tm *time);

DS3231_API int ds3231_setTime(ds3231_t driver, const struct tm *time);

DS3231_API int ds3231_setAlarm1(ds3231_t driver, ds3231_alarmType_t type, const struct tm *time);

DS3231_API int ds3231_setAlarm2(ds3231_t driver, ds3231_alarmType_t type, const struct tm *time);

DS3231_API int ds3231_beginTemperature(ds3231_t driver);

DS3231_API int ds3231_endTemperature(ds3231_t driver, int16_t *out_temperature);

inline static int ds3231_endTemperatureF(ds3231_t driver, float *out_temperature)
{
    if (out_temperature == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    int16_t iTemp;
    if (ds3231_endTemperature(driver, &iTemp))
    {
        return -1;
    }

    *out_temperature = (float)iTemp / 4.0f;
    return 0;
}

#endif
