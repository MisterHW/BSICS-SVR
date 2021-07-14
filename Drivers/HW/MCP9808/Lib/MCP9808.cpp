//
//
//

#include "MCP9808.h"


bool MCP9808::init(I2C_peripheral _hI2C, MCP9808_address addr) {
    this->hI2C = _hI2C;
    this->deviceAddress = addr;
    return this->isReady();
}

bool MCP9808::isReady() {
    return true;
}
