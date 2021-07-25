#ifndef _DS3231_PRIVATE_H_
#define _DS3231_PRIVATE_H_

#include <ds3231.h>
#include "registers.h"

#define BCD_TO_DEC(bcd) ((((bcd) >> 4) * 10) + (bcd & 0x0F))
#define DEC_TO_BCD(dec) ((((dec) / 10) << 4) | (dec % 10))

inline static uint8_t DS3231_12_TO_24(uint8_t _12)
{
    // This lookup table brought to you by maxim.
    // Thanks really thanks.
    switch (_12 & 0x3F)
    {
        default:   return 0xFF;
        case 0x12: return 0x00;
        case 0x01: return 0x01;
        case 0x02: return 0x02;
        case 0x03: return 0x03;
        case 0x04: return 0x04;
        case 0x05: return 0x05;
        case 0x06: return 0x06;
        case 0x07: return 0x07;
        case 0x08: return 0x08;
        case 0x09: return 0x09;
        case 0x10: return 0x10;
        case 0x11: return 0x11;
        case 0x32: return 0x12;
        case 0x21: return 0x13;
        case 0x22: return 0x14;
        case 0x23: return 0x15;
        case 0x24: return 0x16;
        case 0x25: return 0x17;
        case 0x26: return 0x18;
        case 0x27: return 0x19;
        case 0x28: return 0x20;
        case 0x29: return 0x21;
        case 0x30: return 0x22;
        case 0x31: return 0x23;
    }
}

inline static uint8_t DS3231_24_TO_12(uint8_t _24)
{
    switch (_24 & 0x3F)
    {
        default:   return 0xFF;
        case 0x00: return 0x12;
        case 0x01: return 0x01;
        case 0x02: return 0x02;
        case 0x03: return 0x03;
        case 0x04: return 0x04;
        case 0x05: return 0x05;
        case 0x06: return 0x06;
        case 0x07: return 0x07;
        case 0x08: return 0x08;
        case 0x09: return 0x09;
        case 0x10: return 0x10;
        case 0x11: return 0x11;
        case 0x12: return 0x32;
        case 0x13: return 0x21;
        case 0x14: return 0x22;
        case 0x15: return 0x23;
        case 0x16: return 0x24;
        case 0x17: return 0x25;
        case 0x18: return 0x26;
        case 0x19: return 0x27;
        case 0x20: return 0x28;
        case 0x21: return 0x29;
        case 0x22: return 0x30;
        case 0x23: return 0x31;
    }
}

esp_err_t ds3231_io_read(i2c_port_t port, uint8_t addr, void *data, size_t sz);
esp_err_t ds3231_io_write(i2c_port_t port, uint8_t addr, const void *data, size_t sz);

#endif
