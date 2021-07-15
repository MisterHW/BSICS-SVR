//
//
//


#include "ADG715.h"

bool ADG715::init(I2C_MASTER_IF &I2C_master, I2C_peripheral _hI2C, ADG715_address addr) {
    I2CM = &I2C_master;
    hI2C = _hI2C;
    deviceAddress = addr;
    return isReady();
}

bool ADG715::isReady( ) {
    return true;
}

bool ADG715::writeSwitchStates(uint8_t states) {
    return false;
}

uint8_t ADG715::readSwitchStates( ) {
    return 0;
}

