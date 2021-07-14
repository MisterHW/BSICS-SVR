//
// MCP9808
//

#ifndef HW_MCP9808_H
#define HW_MCP9808_H

#include "i2c_def.h"

// MCP9808 addresses (ALT block: "contact factory for this address code.")
enum MCP9808_address {
    // standard type
    MCP9808_ADDR_0x18 = 0x18,
    MCP9808_ADDR_0x19 = 0x19,
    MCP9808_ADDR_0x1A = 0x1A,
    MCP9808_ADDR_0x1B = 0x1B,
    MCP9808_ADDR_0x1C = 0x1C,
    MCP9808_ADDR_0x1D = 0x1D,
    MCP9808_ADDR_0x1E = 0x1E,
    MCP9808_ADDR_0x1F = 0x1F,
    // special order type
    MCP9808_ALT_ADDR_0x48 = 0x18,
    MCP9808_ALT_ADDR_0x49 = 0x49,
    MCP9808_ALT_ADDR_0x4A = 0x4A,
    MCP9808_ALT_ADDR_0x4B = 0x4B,
    MCP9808_ALT_ADDR_0x4C = 0x4C,
    MCP9808_ALT_ADDR_0x4D = 0x4D,
    MCP9808_ALT_ADDR_0x4E = 0x4E,
    MCP9808_ALT_ADDR_0x4F = 0x4F,
};

class MCP9808 {
    I2C_peripheral hI2C;
    MCP9808_address deviceAddress;
public:
    bool init(I2C_peripheral _hI2C, MCP9808_address addr);
    bool isReady( );
};

#endif //HW_MCP9808_H
