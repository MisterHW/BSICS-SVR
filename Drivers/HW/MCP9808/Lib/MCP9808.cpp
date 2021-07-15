//
//
//

#include "MCP9808.h"


bool MCP9808::init(I2C_MASTER_IF &I2C_master, I2C_peripheral _hI2C, MCP9808_address addr) {
    I2CM = &I2C_master;
    hI2C = _hI2C;
    deviceAddress = addr;
    return isReady();
}

bool MCP9808::isReady() {
    return true;
}
