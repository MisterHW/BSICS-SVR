//
//
//

#include "MP8862.h"

bool MP8862::init(I2C_MASTER_IF &I2C_master, I2C_peripheral _hI2C, MP8862_address addr) {
    I2CM = &I2C_master;
    hI2C = _hI2C;
    deviceAddress = addr;
    return isReady();
}

bool MP8862::isReady() {
    return true;
}
