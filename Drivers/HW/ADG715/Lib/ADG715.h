//
// ADG715 CMOS, Low Voltage Serially Controlled, Octal SPST Switches
//

#ifndef HW_ADG715_H
#define HW_ADG715_H

#include <cstdint>
#include "i2c_def.h"


/* ADG715 non-shifted device addresses.
 * HAL UM1905: "device 7 bits address value in datasheet must be shift at right before call."
 */
enum ADG715_address {
    ADG715_ADDR_0x48 = 0x48,
    ADG715_ADDR_0x49 = 0x49,
    ADG715_ADDR_0x4A = 0x4A,
    ADG715_ADDR_0x4B = 0x4B,
};


// ADG715 switch bits
enum ADG715_switch {
    ADG715_S1 = 0x01,
    ADG715_S2 = 0x02,
    ADG715_S3 = 0x04,
    ADG715_S4 = 0x08,
    ADG715_S5 = 0x10,
    ADG715_S6 = 0x20,
    ADG715_S7 = 0x40,
    ADG715_S8 = 0x80,
};


class ADG715 {
    I2C_peripheral hI2C;
    ADG715_address deviceAddress;
public:
    bool init(I2C_peripheral _hI2C, ADG715_address addr);
    bool isReady( );
    bool writeSwitchStates(uint8_t states );
    uint8_t readSwitchStates( );
};

#endif //HW_ADG715_H
