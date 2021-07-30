//
// I2C and SMBus I/O Expander With Configuration Registers
// PCA9535 (16 bit) / PCA9536 (4 bit)
//

#ifndef HW_PCA953X_H
#define HW_PCA953X_H

#include "stm32f7xx_hal.h"

/* Ti SCPS129K:
 * "PCA9535 is identical to the PCA9555, except for the removal of the internal I/O pullup resistor, which greatly
 * reduces power consumption when the I/Os are held low.
 * Three hardware pins (A0, A1, and A2) are used to program and vary the fixed I2C address and allow up to
 * eight devices to share the same I2C bus or SMBus. The fixed I2C address of the PCA9535 is the same as the
 * PCA9555, PCF8575, PCF8575C, and PCF8574, allowing up to eight of these devices in any combination to
 * share the same I2C bus or SMBus.
 */

//  addresses applicable to PCA9535, PCA955
enum PCA9535_address {
    PCA9535_ADDR_0x20 = 0x20,
    PCA9535_ADDR_0x21 = 0x21,
    PCA9535_ADDR_0x22 = 0x22,
    PCA9535_ADDR_0x23 = 0x23,
    PCA9535_ADDR_0x24 = 0x24,
    PCA9535_ADDR_0x25 = 0x25,
    PCA9535_ADDR_0x26 = 0x26,
    PCA9535_ADDR_0x27 = 0x27,
};

// addresses applicable to PCA9536
enum PCA9536_address {
    PCA9536_ADDR_0x41 = 0x41,
};

// registers applicable to PCA9535, PCA9555
enum PCA9535_register {
    PCA9535_PORT0_INPUT     = 0x00,
    PCA9535_PORT1_INPUT     = 0x01,
    PCA9535_PORT0_OUTPUT    = 0x02,
    PCA9535_PORT1_OUTPUT    = 0x03,
    PCA9535_PORT0_INVERT    = 0x04, // invert read-back bits
    PCA9535_PORT1_INVERT    = 0x05, // invert read-back bits
    PCA9535_PORT0_DIRECTION = 0x06,
    PCA9535_PORT1_DIRECTION = 0x07,
};

// registers applicable to PCA9536
enum PCA9536_register {
    PCA9536_PORT0_INPUT     = 0x00,
    PCA9536_PORT0_OUTPUT    = 0x01,
    PCA9536_PORT0_INVERT    = 0x02, // invert read-back bits
    PCA9536_PORT0_DIRECTION = 0x03,
};


template < typename PCA953x_address, typename PCA953x_register >
class PCA953x {
    I2C_HandleTypeDef* hI2C {nullptr};
    PCA953x_address deviceAddress{};
public:
    bool initialized {false};

    bool init(I2C_HandleTypeDef *_hI2C, PCA953x_address addr);
    bool isReady( );
    bool write(PCA953x_register reg, uint8_t *data, unsigned int len );
    bool read(PCA953x_register reg, uint8_t *data, unsigned int len );

    bool writeRegister( PCA953x_register reg, uint8_t value ) {
        return write( reg, &value, 1);
    }
    bool readRegister( PCA953x_register reg , uint8_t &value) {
        return read( reg, &value, 1 );
    }

    static inline uint8_t REG_VALUE_SET_AS_OUTPUTS ( uint8_t mask, uint8_t previous_dir = 0 )  {
        return previous_dir | mask;
    }
    static inline uint8_t REG_VALUE_SET_AS_INPUTS ( uint8_t mask, uint8_t previous_dir = 0xFF )  {
        return previous_dir & ~mask;
    }
};

template<typename PCA953x_address, typename PCA953x_register>
bool PCA953x<PCA953x_address, PCA953x_register>::init(I2C_HandleTypeDef *_hI2C, PCA953x_address addr) {
    hI2C = _hI2C;
    deviceAddress = addr;
    initialized = isReady();
    return initialized;
}

template<typename PCA953x_address, typename PCA953x_register>
bool PCA953x<PCA953x_address, PCA953x_register>::isReady( ) {
    return HAL_OK == HAL_I2C_IsDeviceReady( hI2C, deviceAddress << 1, 3, 5 );
}

template<typename PCA953x_address, typename PCA953x_register>
bool PCA953x<PCA953x_address, PCA953x_register>::write(PCA953x_register reg, uint8_t *data, unsigned int len ) {
    // S
    // deviceAddress << 1 | 0 : ACK : ADDR : ACK
    // BYTE0 : ACK [ : BYTE1 : ACK [ : ... ]]
    // P
    return HAL_OK == HAL_I2C_Mem_Write( hI2C, deviceAddress << 1, reg, 1, data, len, 5 );
}

template<typename PCA953x_address, typename PCA953x_register>
bool PCA953x<PCA953x_address, PCA953x_register>::read(PCA953x_register reg, uint8_t *data, unsigned int len ) {
    // S
    // deviceAddress << 1 | 0 : ACK : ADDR : ACK
    // S
    // deviceAddress << 1 | 1 : ACK : BYTE0 : ACK [ : BYTE1 : ACK [ : ... ]]
    // P
    return HAL_OK == HAL_I2C_Mem_Read( hI2C, deviceAddress << 1, reg, 1, data, len, 5 );
}

using PCA9535 = PCA953x< PCA9535_address, PCA9535_register >;
using PCA9536 = PCA953x< PCA9536_address, PCA9536_register >;

#endif //HW_PCA953X_H
