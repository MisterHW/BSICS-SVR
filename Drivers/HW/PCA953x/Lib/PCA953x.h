//
// I2C and SMBus I/O Expander With Configuration Registers
// PCA9535 (16 bit) / PCA9536 (4 bit)
//

#ifndef HW_PCA953X_H
#define HW_PCA953X_H

#include "i2c_def.h"

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
    PCA9535_PORT0_INVERT    = 0x04,
    PCA9535_PORT1_INVERT    = 0x05,
    PCA9535_PORT0_DIRECTION = 0x06,
    PCA9535_PORT1_DIRECTION = 0x07,
};

// registers applicable to PCA9536
enum PCA9536_register {
    PCA9536_PORT0_INPUT     = 0x00,
    PCA9536_PORT0_OUTPUT    = 0x01,
    PCA9536_PORT0_INVERT    = 0x02,
    PCA9536_PORT0_DIRECTION = 0x03,
};


template < typename PCA953x_address, typename PCA953x_register >
class PCA953x {
    I2C_peripheral hI2C;
    PCA953x_address deviceAddress;
public:
    bool init(I2C_peripheral _hI2C, PCA953x_address addr );
    bool isReady( );
    bool write( const uint8_t* data, unsigned int len );
    bool read(PCA953x_register reg, uint8_t &data, unsigned int len);
    inline bool writeRegister(PCA953x_register reg, uint8_t value) {
        uint8_t data[2] = {reg, value};
        return write(&data[0], 2);
    }
    inline uint8_t readRegister(PCA953x_register reg) {
        uint8_t data;
        read(reg, &data, 1);
        return data;
    }
};

template<typename PCA953x_address, typename PCA953x_register>
bool PCA953x<PCA953x_address, PCA953x_register>::init(I2C_peripheral _hI2C, PCA953x_address addr) {
    return false;
}

template<typename PCA953x_address, typename PCA953x_register>
bool PCA953x<PCA953x_address, PCA953x_register>::isReady( ) {
    return false;
}

template<typename PCA953x_address, typename PCA953x_register>
bool PCA953x<PCA953x_address, PCA953x_register>::write(const uint8_t *data, const unsigned int len) {
    return false;
}

template<typename PCA953x_address, typename PCA953x_register>
bool PCA953x<PCA953x_address, PCA953x_register>::read(PCA953x_register reg, uint8_t &data, unsigned int len) {
    return false;
}

using PCA9535 = PCA953x< PCA9535_address, PCA9535_register >;
using PCA9536 = PCA953x< PCA9536_address, PCA9536_register >;

#endif //HW_PCA953X_H
