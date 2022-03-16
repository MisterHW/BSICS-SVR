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

// Programmable Gain definitions
enum MCP342x_PGA {
    MCP342X_GAIN_1X = 0 << 0,
    MCP342X_GAIN_2X = 1 << 0,
    MCP342X_GAIN_4X = 2 << 0,
    MCP342X_GAIN_8X = 3 << 0,
};

/* Sample size definitions - these also affect the sampling rate
 * 12-bit has a max sample rate of 240sps
 * 14-bit has a max sample rate of  60sps
 * 16-bit has a max sample rate of  15sps
 * 18-bit has a max sample rate of   3.75sps (MCP3421, MCP3422, MCP3423, MCP3424 only)
 */
enum MCP342x_resolution {
    MCP342X_RES_12BIT = 0 << 2,
    MCP342X_RES_14BIT = 1 << 2,
    MCP342X_RES_16BIT = 2 << 2,
    MCP342X_RES_18BIT = 3 << 2,
};

enum MCP342x_conversion_mode {
    MCP342X_MODE_ONESHOT       = 0 << 4, // no new conversion started, ongoing conversion can finish.
    MCP342X_MODE_ONESHOT_START = (0 << 4) | (1 << 7), // Set mode oneshot, start single conversion (RDY = 1).
    MCP342X_MODE_CONTINUOUS    = 1 << 4,
};

enum MCP342x_bit_mask {
    MCP342X_RDY          = 0x80, // RDY flag
    MCP342X_GAIN_MASK    = 0x03,
    MCP342X_MODE_MASK    = 0x10,
    MCP342X_CHANNEL_MASK = 0x60,
    MCP342X_RES_MASK     = 0x0C,
};

/* Channel definitions
 * MCP3421 & MCP3425 have only the one channel and ignore this param
 * MCP3422, MCP3423, MCP3426 & MCP3427 have two channels and treat 3 & 4 as repeats of 1 & 2 respectively
 * MCP3424 & MCP3428 have all four channels
 */
enum MCP3422_channel {
    MCP3422_CHANNEL_1 = 0 << 5,
};
enum MCP3423_channel {
    MCP3423_CHANNEL_1 = 0 << 5,
    MCP3423_CHANNEL_2 = 1 << 5,
};
enum MCP3424_channel {
    MCP3424_CHANNEL_1 = 0 << 5,
    MCP3424_CHANNEL_2 = 1 << 5,
    MCP3424_CHANNEL_3 = 2 << 5,
    MCP3424_CHANNEL_4 = 3 << 5,
};

/* MCP342x_rx_bytes : number of bytes to be read back from MCP342x.
 * 2 or 3 data bytes are followed by the config byte, which keeps repeating.
 * For decoding without knowledge of the last config sent to the device,
 * 4 bytes need to be read. Data alignment is inferred from S1:0 bits
 * ("Sample Rate" -> number of bits, 12 .. 18).
 */
enum MCP342x_rx_bytes {
    MCP342x_rx_lte16bit_no_status   = 2, // <= 16 bit resolution, no status byte
    MCP342x_rx_lte16bit_plus_status = 3, // <= 16 bit resolution, last byte = status byte
    MCP342x_rx_18bit_no_status      = 3, // 18 bit resolution, no status byte
    MCP342x_rx_18bit_plus_status    = 4, // 18 bit resolution, last byte = status byte
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
    // int conversion operator to use MCP342x_config in algebraic expressions.
#if defined(__cplusplus) && (__cplusplus >= 201103L) // require C++11 or later for "explicit"
    inline explicit operator uint8_t() const {
        return val;
    };
#else
    inline operator uint8_t() const {
            return val;
    };
#endif
} MCP342x_config;


template < typename MCP342x_address , typename MP342x_channel>
class MCP342x {
    I2C_HandleTypeDef* hI2C {nullptr};
    MCP342x_address deviceAddress{};
    // MCP342x_config cfg[4]{};
public:
    bool initialized {false};

    bool init(I2C_HandleTypeDef *_hI2C, MCP342x_address addr);
    bool isReady( );

    bool writeConfig( MCP342x_config cfg );
    bool read( uint8_t *data, MCP342x_rx_bytes len );
    bool readConvResult(int32_t& value);

    /* Convert MCP342x raw values scaled by coef = -4.095 .. +4.095 (default: 1.0). Returns scaled uV.
     * abs(coef_x4000) must be < 2^14. If too large, divide by 10 (100) to obtain result in x10 uV (x100 uV).
     * Assumes MCP342X_GAIN_1X. To generalize, multiply coefficient coef_x4000 by (1 << config_byte.bits.pga)
     * or use raw_to_uV(...) << config_byte.bits.pga.
     */
    static int32_t raw_to_uV( int32_t adc_raw, int16_t coef_x4000 = 4000 )
    {
        return ( adc_raw * coef_x4000 ) >> 8 ; // V = (2.49+1.05)/1.05 * 2.048 * (adc_raw / 131072) = adc_raw * 0.00005267857
    };

    /* Convert MCP342x raw values scaled by coef = -16.383 .. +16.383 (default: 1.0). Returns scaled mV.
     * abs(coef_x1024) must be < 2^14. If too large, divide by 10 (100) to obtain result in x10 uV (x100 uV).
     * Assumes MCP342X_GAIN_1X. To generalize, multiply coefficient coef_x4000 by (1 << config_byte.bits.pga)
     * or raw_to_uV(...) << config_byte.bits.pga.
     */
    static int32_t raw_to_mV( int32_t adc_raw, int16_t coef_x1024 = 1024 )
    {
        return ( adc_raw * coef_x1024 ) >> 16 ; // V = (13.3+1.05)/1.05 * 2.048 * (adc_raw / 131072) = adc_raw * 0.00021354166
    };
};

template<typename MCP342x_address, typename MP342x_channel>
bool MCP342x<MCP342x_address, MP342x_channel>::init( I2C_HandleTypeDef *_hI2C, MCP342x_address addr ) {
    hI2C = _hI2C;
    deviceAddress = addr;
    initialized = isReady();
    return initialized;
}

template<typename MCP342x_address, typename MP342x_channel>
bool MCP342x<MCP342x_address, MP342x_channel>::isReady() {
    return HAL_OK == HAL_I2C_IsDeviceReady(hI2C, deviceAddress << 1, 3, 5 );
}

template<typename MCP342x_address, typename MP342x_channel>
bool MCP342x<MCP342x_address, MP342x_channel>::writeConfig( MCP342x_config data ) {
    // S
    // deviceAddress << 1 | 0 : ACK : ADDR : ACK : CONFIG
    // P
    // cfg[data.bits.channel] = data; // update shadow copy
    uint8_t txbuf =(uint8_t)data;
    return HAL_OK == HAL_I2C_Master_Transmit( hI2C, deviceAddress << 1, &txbuf, 1, 20 );
}

template<typename MCP342x_address, typename MP342x_channel>
bool MCP342x<MCP342x_address, MP342x_channel>::read( uint8_t* data, MCP342x_rx_bytes len ) {
    // S
    // deviceAddress << 1 | 1 : ACK [ : DATA2 : ACK ] : DATA1 : ACK : DATA0 : ACK [ : CFG : ACK [ : CFG : ACK [ : ... ]]
    // P
    return HAL_OK == HAL_I2C_Master_Receive( hI2C, deviceAddress << 1, data, len, 20 );
}

template<typename MCP342x_address, typename MP342x_channel>
bool MCP342x<MCP342x_address, MP342x_channel>::readConvResult(int32_t &value) {
    uint8_t data[MCP342x_rx_18bit_plus_status] {};
    bool success = read(data, MCP342x_rx_18bit_plus_status);
    success = success && (data[3] & MCP342X_RDY);
    uint8_t  res = data[3] & MCP342X_RES_MASK;
    uint32_t tmp = data[0] << 24 | data[1] << 16;
    if(res == MCP342X_RES_18BIT)
    {
        value = ( (int32_t)( ((tmp<<6)^(1<<31)) | (data[2]<<14) ) - (1<<31) ) >> 14;
        return success;
    } else if (res == MCP342X_RES_16BIT) {
        value = ( (int32_t)((tmp   )^(1<<31)) - (1<<31) ) >> 14;
        return success;
    } else if (res == MCP342X_RES_14BIT) {
        value = ( (int32_t)((tmp<<2)^(1<<31)) - (1<<31) ) >> 14;
        return success;
    } else { // MCP342X_RES_12BIT
        value = ( (int32_t)((tmp<<4)^(1<<31)) - (1<<31) ) >> 14;
        return success;
    }
}


using MCP3422 = MCP342x < MCP3422_address , MCP3422_channel >;
using MCP3423 = MCP342x < MCP342x_address , MCP3423_channel >;
using MCP3424 = MCP342x < MCP342x_address , MCP3424_channel >;

#endif //HW_MCP342X_H
