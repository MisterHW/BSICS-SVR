//
//
//

#include "MP8862.h"

bool MP8862::init(I2C_peripheral _hI2C, MP8862_address addr) {
    this->hI2C = _hI2C;
    this->deviceAddress = addr;
    return this->isReady();
}

bool MP8862::isReady() {
    return true;
}
