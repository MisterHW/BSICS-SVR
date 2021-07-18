//
//
//

#include "MP8862.h"

bool MP8862::init(I2C_HandleTypeDef *_hI2C, MP8862_address addr) {
    hI2C = _hI2C;
    deviceAddress = addr;
    return isReady();
}

bool MP8862::isReady() {
    return HAL_OK == HAL_I2C_IsDeviceReady(hI2C, deviceAddress << 1, 3, 5);
}
