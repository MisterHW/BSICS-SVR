//
//
//

#include "MP8862.h"

bool MP8862::init(I2C_HandleTypeDef *_hI2C, MP8862_address addr) {
    hI2C = _hI2C;
    deviceAddress = addr;
    initialized = isReady();
    return initialized;
}

bool MP8862::isReady() {
    return HAL_OK == HAL_I2C_IsDeviceReady(hI2C, deviceAddress << 1, 3, 5);
}


bool MP8862::write(MP8862_register reg, uint8_t *data, uint8_t len ) {
    // S
    // deviceAddress << 1 | 0 : ACK : REG_ADDR : ACK
    // DATA0 : ACK [ : DATA1* : ACK [ : ... ]]
    // P
    // *: Register addresses are post-incremented, and *** read-only registers will be skipped ***.
    return HAL_OK == HAL_I2C_Mem_Write( hI2C, deviceAddress << 1, reg, 1, data, len, 5 );
}


bool MP8862::read(MP8862_register reg, uint8_t *data, uint8_t len ) {
    // S
    // deviceAddress << 1 | 0 : ACK : REG_ADDR_K : ACK
    // Sr
    // deviceAddress << 1 | 1 : ACK : BYTE K [: ACK : BYTE K+1* [ : ... ]] : NACK
    // P
    // *: Datasheet p.23 I2C write and read examples do not mention multi-byte read, but experimental evidence shows:
    //    - when master ACK is sent instead of NACK, multiple bytes can be read from auto-incrementing reg addresses
    //    - unmapped register addresses 0x0D ... 0x26 and 0x2A .. 0xFF read as 0x00
    //    -
    return HAL_OK == HAL_I2C_Mem_Read( hI2C, deviceAddress << 1, reg, 1, data, len, 5 );
}

bool MP8862::hardwarePowerUp(bool (*callback_set_enable_pin)(uint8_t)) {
    return false;
}

bool MP8862::setEnable(bool soft_EN) {
    return false;
}

bool MP8862::setCurrentLimit_mA(uint16_t current_mA) {
    return false;
}

bool MP8862::readCurrentLimit_mA(uint16_t &current_mA) {
    return false;
}

bool MP8862::setVoltageSetpoint_mV(uint16_t voltage_mV) {
    return false;
}

bool MP8862::readVoltageSetpoint_mV(uint16_t &voltage_mV) {
    return false;
}
