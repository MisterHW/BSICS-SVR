//
// MCP342x 18-Bit, Multi-Channel ΔΣ Analog-to-Digital Converter with I2C and On-Board Reference
// MCP3422, MCP3423 (2-channel), MCP3424 (4-channel)
//

#ifndef HW_MCP342X_H
#define HW_MCP342X_H

#include "i2c_def.h"

// MCP3422 A0 address (default, others only available on request)
enum MCP3422_address {
    MCP3422_ADDR_0x68 = 0x68, // fixed address
};

// MCP3423, MCP3424 addresses
enum MCP3423_address {
    MCP342x_ADDR_0x68 = 0x68, // ADR0 = 0    , ADR1 = 0 or ADR0 = Float, ADR1 = Float
    MCP342x_ADDR_0x6C = 0x6C, // ADR0 = 1    , ADR1 = 0
    MCP342x_ADDR_0x6A = 0x6A, // ADR0 = 0    , ADR1 = 1
    MCP342x_ADDR_0x6E = 0x6E, // ADR0 = 1    , ADR1 = 1

    MCP342x_ADDR_0x69 = 0x69, // ADR0 = 0    , ADR1 = Float
    MCP342x_ADDR_0x6B = 0x6B, // ADR0 = Float, ADR1 = 0
    MCP342x_ADDR_0x6D = 0x6D, // ADR0 = 1    , ADR1 = Float
    MCP342x_ADDR_0x6F = 0x6F, // ADR0 = Float, ADR1 = 1
};

template < typename MCP342x_address >
class MCP342x {
    I2C_MASTER_IF* I2CM = nullptr;
    I2C_peripheral hI2C{};
    MCP342x_address deviceAddress{};
public:
    bool init(I2C_MASTER_IF &I2C_master, I2C_peripheral _hI2C, MCP342x_address addr);
    bool isReady( );
};

template<typename MCP342x_address>
bool MCP342x<MCP342x_address>::init(I2C_MASTER_IF &I2C_master, I2C_peripheral _hI2C, MCP342x_address addr) {
    I2CM = &I2C_master;
    hI2C = _hI2C;
    deviceAddress = addr;
    return isReady();
}

template<typename MCP342x_address>
bool MCP342x<MCP342x_address>::isReady() {
    return true;
}


using MCP3422 = MCP342x < MCP3422_address >;
using MCP3423 = MCP342x < MCP3423_address >;
using MCP3424 = MCP342x < MCP3423_address >;

#endif //HW_MCP342X_H
