//
//
//

#include "MCP9808.h"

bool MCP9808::init(I2C_HandleTypeDef *_hI2C, MCP9808_address addr) {
    hI2C = _hI2C;
    deviceAddress = addr;
    initialized = isReady();
    return initialized;
}

bool MCP9808::isReady() {
    return HAL_OK == HAL_I2C_IsDeviceReady( hI2C, deviceAddress << 1, 3, 5 );
}

bool MCP9808::write(MCP9808_register reg, uint8_t *data, unsigned int len) {
    // S
    // deviceAddress << 1 | 0 : ACK : ADDR : ACK
    // MSB0 : ACK : LSB0 : ACK [ : MSB1 : ACK : LSB1 : ACK [ : ... ]]
    // P
    return HAL_OK == HAL_I2C_Mem_Write(hI2C, deviceAddress << 1, reg, 1, data, len, 5);
}

bool MCP9808::read(MCP9808_register reg, uint8_t* data, unsigned int len) {
    // S
    // deviceAddress << 1 | 0 : ACK : ADDR : ACK
    // S
    // deviceAddress << 1 | 1 : ACK : MSB0 : ACK : LSB0 : ACK [ : MSB1 : ACK : LSB1 : ACK [ : ... ]]
    // P
    return HAL_OK == HAL_I2C_Mem_Read(hI2C, deviceAddress << 1, reg, 1, data, len, 5);
}

bool MCP9808::writeReg16(MCP9808_register reg, uint16_t value) {
    // S
    // deviceAddress << 1 | 0 : ACK : ADDR : ACK
    // S
    // deviceAddress << 1 | 1 : ACK : MSB0 : ACK : LSB0 : ACK
    // P
    uint8_t data[2];
    data[0] = value >> 8;
    data[1] = value & 0xFF;
    return write(reg, &data[0], 2);
}

bool MCP9808::writeReg8(MCP9808_register reg, uint8_t value) {
    // S
    // deviceAddress << 1 | 0 : ACK
    // BYTE
    // P
    return write(reg, &value, 1);
}

bool MCP9808::readReg16(MCP9808_register reg, uint16_t &value) {
    uint8_t data[2];
    bool success = read(reg, &data[0], 2);
    value = data[0] << 8 | data[1];
    return success;
}

bool MCP9808::readReg8(MCP9808_register reg, uint8_t &value) {
    // S
    // deviceAddress << 1 | 0 : ACK
    // MSB
    // P
    return read(reg, &value, 1);
}

