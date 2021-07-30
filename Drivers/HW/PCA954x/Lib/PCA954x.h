/*
 * PCA954x series 4- and 8-channel I2C-bus switches (multi-path) and multiplexers (1-of-n)
 *
 * PCA9544A : 4-channel I2C-bus multiplexer with interrupt logic
 * PCA9545A : PCA9545A Low Voltage 4-channel I2C and SMbus Switch With Interrupt Logic and Reset Functions
 * PCA9546A : PCA9546A Low Voltage 4-Channel I2C and SMBus Switch with Reset Function
 * PCA9547  : 8-channel I2C-bus multiplexer with reset
 * PCA9548A : 8-channel I2C-bus switch with reset
 *
 * PCA9545A, PCA9546A, PCA9548A:
 * Care should be taken not to exceed the maximum bus capacitance. Default condition is all zeroes.
 */
//

#ifndef HW_PCA954x_H
#define HW_PCA954x_H

#include "stm32f7xx_hal.h"

/* PCA954x non-shifted device addresses.
 * HAL UM1905: "device 7 bits address value in datasheet must be shift at right before call."
 */

enum PCA9544A_address {
    PCA9544A_ADDR_0x70 = 0x70,
    PCA9544A_ADDR_0x71 = 0x71,
    PCA9544A_ADDR_0x72 = 0x72,
    PCA9544A_ADDR_0x73 = 0x73,
    PCA9544A_ADDR_0x74 = 0x74,
    PCA9544A_ADDR_0x75 = 0x75,
    PCA9544A_ADDR_0x76 = 0x76,
    PCA9544A_ADDR_0x77 = 0x77,
};

enum PCA9545A_address {
    PCA9545A_ADDR_0x70 = 0x70,
    PCA9545A_ADDR_0x71 = 0x71,
    PCA9545A_ADDR_0x72 = 0x72,
    PCA9545A_ADDR_0x73 = 0x73,
};

using PCA9546A_address = PCA9544A_address;
using PCA9547_address  = PCA9544A_address;
using PCA9548A_address = PCA9544A_address;


// PCA9544A channel selection bits (single selection only)
typedef union PCA9544A_CR_ {
    uint8_t val;
    struct __attribute__ ((__packed__)) {
        uint8_t channel  : 2 ;
        uint8_t enable   : 1 ;
        uint8_t unused   : 1 ;
        uint8_t INT0     : 1 ;
        uint8_t INT1     : 1 ;
        uint8_t INT2     : 1 ;
        uint8_t INT3     : 1 ;
    } bits;
    inline PCA9544A_CR_& operator= (const uint8_t rhs) {
        val = rhs;
        return *this;
    };
} PCA9544A_CR;

using PCA9545A_CR = uint8_t;
using PCA9546A_CR = uint8_t;

// PCA9547 channel selection bits (single selection only)
typedef union PCA9547_CR_ {
    uint8_t val;
    struct __attribute__ ((__packed__)) {
        uint8_t channel  : 3 ;
        uint8_t enable   : 1 ;
        uint8_t INT0     : 1 ;
        uint8_t INT1     : 1 ;
        uint8_t INT2     : 1 ;
        uint8_t INT3     : 1 ;
    } bits;
    inline PCA9547_CR_& operator= (const uint8_t rhs) {
        val = rhs;
        return *this;
    };
} PCA9547_CR;

using PCA9548A_CR = uint8_t;


enum PCA9544A_CR_bits {
    PCA9544A_ENA  = 0x04, // read/write
    PCA9544A_INT0 = 0x10, // read-only
    PCA9544A_INT1 = 0x20, // read-only
    PCA9544A_INT2 = 0x40, // read-only
    PCA9544A_INT3 = 0x80, // read-only
};

// PCA9545A channel selection bits (mutiple selection permitted)
enum PCA9545A_CR_bits {
    PCA9545A_CH0  = 0x01, // read/write
    PCA9545A_CH1  = 0x02, // read/write
    PCA9545A_CH2  = 0x04, // read/write
    PCA9545A_CH3  = 0x08, // read/write
    PCA9545A_INT0 = 0x10, // read-only
    PCA9545A_INT1 = 0x20, // read-only
    PCA9545A_INT2 = 0x40, // read-only
    PCA9545A_INT3 = 0x80, // read-only
};

// PCA9546A channel selection bits (mutiple selection permitted)
enum PCA9546A_CR_bits {
    PCA9546A_CH0  = 0x01, // read/write
    PCA9546A_CH1  = 0x02, // read/write
    PCA9546A_CH2  = 0x04, // read/write
    PCA9546A_CH3  = 0x08, // read/write
    // bits 4 thru 7 ignored
};

enum PCA9547A_CR_bits {
    PCA9547_ENA  = 0x08, // read/write
    PCA9547_INT0 = 0x10, // read-only
    PCA9547_INT1 = 0x20, // read-only
    PCA9547_INT2 = 0x40, // read-only
    PCA9547_INT3 = 0x80, // read-only
};

// PCA9548A channel selection bits (mutiple selection permitted)
enum PCA9548A_CR_bits {
    PCA9548A_CH0 = 0x01,
    PCA9548A_CH1 = 0x02,
    PCA9548A_CH2 = 0x04,
    PCA9548A_CH3 = 0x08,
    PCA9548A_CH4 = 0x10,
    PCA9548A_CH5 = 0x20,
    PCA9548A_CH6 = 0x40,
    PCA9548A_CH7 = 0x80,
};


template < typename PCA954x_address, typename PCA954x_control_register >
class PCA954x {
    I2C_HandleTypeDef*  hI2C {nullptr};
    PCA954x_address deviceAddress{};
public:
    bool initialized {false};

    bool init(I2C_HandleTypeDef *_hI2C, PCA954x_address addr);
    bool isReady( );
    bool writeControlReg( PCA954x_control_register  reg );
    bool readControlReg ( PCA954x_control_register& reg );
};

template < typename PCA954x_address, typename PCA954x_control_register >
bool PCA954x::init(I2C_HandleTypeDef *_hI2C, PCA954x_address addr) {
    hI2C = _hI2C;
    deviceAddress = addr;
    initialized = isReady();
    return initialized;
};

template < typename PCA954x_address, typename PCA954x_control_register >
bool PCA954x::isReady( ) {
    return HAL_OK == HAL_I2C_IsDeviceReady(hI2C, deviceAddress << 1, 3, 5);
};

template < typename PCA954x_address, typename PCA954x_control_register >
bool PCA954x::writeControlReg(PCA954x_control_register reg) {
    // S
    // deviceAddress << 1 | 0 : ACK : CONTROL_REG
    // P
    return HAL_OK == HAL_I2C_Master_Transmit(hI2C, deviceAddress << 1, &reg, 1, 5);
};

template < typename PCA954x_address, typename PCA954x_control_register >
bool PCA954x::readControlReg( PCA954x_control_register& reg ) {
    // S
    // deviceAddress << 1 | 1 : ACK : CONTROL_REG
    // P
    return HAL_OK == HAL_I2C_Master_Receive(hI2C, deviceAddress << 1, &reg, 1, 5);
};


using PCA9544A = PCA954x < PCA9544A_address , PCA9544A_CR > ;
using PCA9545A = PCA954x < PCA9545A_address , PCA9545A_CR > ;
using PCA9546A = PCA954x < PCA9546A_address , PCA9546A_CR > ;
using PCA9547  = PCA954x < PCA9547_address  , PCA9547_CR  > ;
using PCA9548A = PCA954x < PCA9548A_address , PCA9548A_CR > ;

#endif //HW_PCA954x_H
