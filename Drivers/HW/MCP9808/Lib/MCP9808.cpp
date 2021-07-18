//
//
//

#include "MCP9808.h"


bool MCP9808::init(I2C_HandleTypeDef *_hI2C, MCP9808_address addr) {
    hI2C = _hI2C;
    deviceAddress = addr;
    return isReady();
}

bool MCP9808::isReady() {
    return HAL_OK == HAL_I2C_IsDeviceReady(hI2C, deviceAddress << 1, 3, 5);
}
