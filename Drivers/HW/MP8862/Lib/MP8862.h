//
// MP8862 : 2.8V-22V VIN, 2A IOUT, 4-Switch, Integrated Buck-Boost Converter with I2C Interface
//

#ifndef HW_MP8862_H
#define HW_MP8862_H

#include "i2c_def.h"

// MP8862 device addresses
enum MP8862_address {
    MP8862_ADDR_0x69 = 0x69, // ADD = 0.00 .. 0.25 * VCC
    MP8862_ADDR_0x6B = 0x6B, // ADD = 0.25 .. 0.50 * VCC
    MP8862_ADDR_0x6D = 0x6D, // ADD = 0.50 .. 0.75 * VCC
    MP8862_ADDR_0x6F = 0x6F, // ADD = 0.75 .. 1.00 * VCC
};

class MP8862 {
    I2C_peripheral hI2C;
    MP8862_address deviceAddress;
public:
    bool init(I2C_peripheral _hI2C, MP8862_address addr);
    bool isReady( );
};


#endif //HW_MP8862_H
