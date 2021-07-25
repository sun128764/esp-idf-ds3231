#ifndef _DS3231_REGISTERS_H_
#define _DS3231_REGISTERS_H_

#define DS3231_REGISTER_SECONDS         (0x00)
#define DS3231_REGISTER_MINUTES         (0x01)
#define DS3231_REGISTER_HOURS           (0x02)
#define DS3231_REGISTER_DAY             (0x03)
#define DS3231_REGISTER_DATE            (0x04)
#define DS3231_REGISTER_MONTH           (0x05)
#define DS3231_REGISTER_YEAR            (0x06)
#define DS3231_REGISTER_ALARM1_SECONDS  (0x07)
#define DS3231_REGISTER_ALARM1_MINUTES  (0x08)
#define DS3231_REGISTER_ALARM1_HOURS    (0x09)
#define DS3231_REGISTER_ALARM1_DAY_DATE (0x0A)
#define DS3231_REGISTER_ALARM2_MINUTES  (0x0B)
#define DS3231_REGISTER_ALARM2_HOURS    (0x0C)
#define DS3231_REGISTER_ALARM2_DAY_DATE (0x0D)
#define DS3231_REGISTER_CONTROL         (0x0E)
#define DS3231_REGISTER_STATUS          (0x0F)
#define DS3231_REGISTER_OFFSET          (0x10)
#define DS3231_REGISTER_TEMP_H          (0x11)
#define DS3231_REGISTER_TEMP_L          (0x12)

#define DS3231_HOURS_12H_BIT            (1 << 6)
#define DS3231_DAY_DATE_BIT             (1 << 6)
#define DS3231_CENTURY_BIT              (1 << 7)
#define DS3231_ALARM_EN_BIT             (1 << 7)

#define DS3231_CONTROL_EOSC             (1 << 7)
#define DS3231_CONTROL_BBSQW            (1 << 6)
#define DS3231_CONTROL_CONV             (1 << 5)
#define DS3231_CONTROL_RS_POS           3
#define DS3231_CONTROL_RS_MASK          (3 << DS3231_CONTROL_RS_POS)
#define DS3231_CONTROL_RS_1HZ           (0 << DS3231_CONTROL_RS_POS)
#define DS3231_CONTROL_RS_1024HZ        (1 << DS3231_CONTROL_RS_POS)
#define DS3231_CONTROL_RS_4096HZ        (2 << DS3231_CONTROL_RS_POS)
#define DS3231_CONTROL_RS_8192HZ        (3 << DS3231_CONTROL_RS_POS)
#define DS3231_CONTROL_INTCN            (1 << 2)
#define DS3231_CONTROL_A2IE             (1 << 1)
#define DS3231_CONTROL_A1IE             (1 << 0)

#define DS3231_RSTATUS_OSF              (1 << 7)
#define DS3231_RSTATUS_EN32KHZ          (1 << 3)
#define DS3231_RSTATUS_BSY              (1 << 2)
#define DS3231_RSTATUS_A2F              (1 << 1)
#define DS3231_RSTATUS_A1F              (1 << 0)

#endif
