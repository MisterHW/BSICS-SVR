//
//
//


#include "ADG715.h"

bool ADG715::init(I2C_peripheral _hI2C, ADG715_address addr) {
    this->hI2C = _hI2C;
    this->deviceAddress = addr;
    return this->isReady();
}

bool ADG715::isReady( ) {
    return false;
}

bool ADG715::writeSwitchStates(uint8_t states) {
    return false;
}

uint8_t ADG715::readSwitchStates( ) {
    return 0;
}

