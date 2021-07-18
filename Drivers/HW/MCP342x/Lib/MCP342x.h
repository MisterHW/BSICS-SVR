//
// MCP342x 18-Bit, Multi-Channel ΔΣ Analog-to-Digital Converter with I2C and On-Board Reference
// MCP3422, MCP3423 (2-channel), MCP3424 (4-channel)
//

#ifndef HW_MCP342X_H
#define HW_MCP342X_H

#include "stm32f7xx_hal.h"

// MCP3422 A0 address (default, others only available on request)
enum MCP3422_address {
    MCP3422_DEFAULT_ADDR_0x68 = 0x68, // fixed address
};
// MCP3423, MCP3424 addresses
enum MCP342x_address {
    MCP342x_ADDR_0x68 = 0x68, // ADR0 = 0    , ADR1 = 0 or ADR0 = Float, ADR1 = Float
    MCP342x_ADDR_0x6C = 0x6C, // ADR0 = 1    , ADR1 = 0
    MCP342x_ADDR_0x6A = 0x6A, // ADR0 = 0    , ADR1 = 1
    MCP342x_ADDR_0x6E = 0x6E, // ADR0 = 1    , ADR1 = 1

    MCP342x_ADDR_0x69 = 0x69, // ADR0 = 0    , ADR1 = Float
    MCP342x_ADDR_0x6B = 0x6B, // ADR0 = Float, ADR1 = 0
    MCP342x_ADDR_0x6D = 0x6D, // ADR0 = 1    , ADR1 = Float
    MCP342x_ADDR_0x6F = 0x6F, // ADR0 = Float, ADR1 = 1
};

enum MCP342x_conversion_mode {
    MCP342x_CONV_MODE_ONESHOT    = 0x00,
    MCP342x_CONV_MODE_CONTINUOUS = 0x10,
};

/* Channel definitions
 * MCP3421 & MCP3425 have only the one channel and ignore this param
 * MCP3422, MCP3423, MCP3426 & MCP3427 have two channels and treat 3 & 4 as repeats of 1 & 2 respectively
 * MCP3424 & MCP3428 have all four channels
 */
enum MCP3422_channel {
    MCP3422_CHANNEL_1 = 0x00,
};
enum MCP3423_channel {
    MCP3423_CHANNEL_1 = 0x00,
    MCP3423_CHANNEL_2 = 0x20,
};
enum MCP3424_channel {
    MCP3424_CHANNEL_1 = 0x00,
    MCP3424_CHANNEL_2 = 0x20,
    MCP3424_CHANNEL_3 = 0x40,
    MCP3424_CHANNEL_4 = 0x60,
};

/* Sample size definitions - these also affect the sampling rate
 * 12-bit has a max sample rate of 240sps
 * 14-bit has a max sample rate of  60sps
 * 16-bit has a max sample rate of  15sps
 * 18-bit has a max sample rate of   3.75sps (MCP3421, MCP3422, MCP3423, MCP3424 only)
 */
enum MCP342x_resolution {
    MCP342X_RES_12BIT = 0x00,
    MCP342X_RES_14BIT = 0x04,
    MCP342X_RES_16BIT = 0x08,
    MCP342X_RES_18BIT = 0x0C,
};

// Programmable Gain definitions
enum MCP342x_PGA {
    MCP342X_GAIN_1X = 0x00,
    MCP342X_GAIN_2X = 0x01,
    MCP342X_GAIN_4X = 0x02,
    MCP342X_GAIN_8X = 0x03,
};

enum MCP342x_bit_mask {
    MCP342X_RDY          = 0x80, // RDY flag
    MCP342X_GAIN_MASK    = 0x03,
    MCP342X_CHANNEL_MASK = 0x60,
    MCP342X_SIZE_MASK    = 0x0C,
};

typedef union MCP342x_config_ {
    uint8_t val;
    struct __attribute__ ((__packed__)) {
        uint8_t pga        :2;
        uint8_t resolution :2;
        uint8_t conv_mode  :1;
        uint8_t channel    :2;
        uint8_t RDY        :1;
    } bits;
    inline MCP342x_config_& operator= (const uint8_t rhs) {
        val = rhs;
        return *this;
    };
} MCP342x_config;


template < typename MCP342x_address , typename MP342x_channel>
class MCP342x {
    I2C_HandleTypeDef* hI2C {nullptr};
    MCP342x_address deviceAddress{};
    MCP342x_config cfg[4]{};
public:
    bool init(I2C_HandleTypeDef *_hI2C, MCP342x_address addr);
    bool isReady( );
};

template<typename MCP342x_address, typename MP342x_channel>
bool
MCP342x<MCP342x_address, MP342x_channel>::init(I2C_HandleTypeDef *_hI2C, MCP342x_address addr) {
    hI2C = _hI2C;
    deviceAddress = addr;
    return isReady();
}

template<typename MCP342x_address, typename MP342x_channel>
bool MCP342x<MCP342x_address, MP342x_channel>::isReady() {
    return HAL_OK == HAL_I2C_IsDeviceReady(hI2C, deviceAddress << 1, 3, 5);
}


using MCP3422 = MCP342x < MCP3422_address , MCP3422_channel >;
using MCP3423 = MCP342x < MCP342x_address , MCP3423_channel >;
using MCP3424 = MCP342x < MCP342x_address , MCP3424_channel >;

#endif //HW_MCP342X_H
