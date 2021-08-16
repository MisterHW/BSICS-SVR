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
    // TODO: implement time-cirtical power-up sequence
    return false;
}

bool MP8862::setEnable(bool soft_EN) {
    uint8_t reg;
    bool success = read( MP8862_REG_CTL1 , &reg, 1 );
    reg |= MP8862_REG_CTL1_bits::MP8862_EN;
    return success & write( MP8862_REG_CTL1 , &reg, 1 );
}

bool MP8862::setCurrentLimit_mA(uint16_t current_mA) {
    if(current_mA > 4049){ // highest permitted value producing int part 80
        current_mA = 4049; // 4.0 A max (I_LIM = 0x50)
    }
    uint32_t limit_raw = (current_mA * 1311) >> 16; // * 0.02 (~ 1311/65536)
    uint8_t tmp = limit_raw & 0x7F;
    return write( MP8862_REG_IOUT_LIM , &tmp, 1 );
}

bool MP8862::readCurrentLimit_mA(uint16_t &current_mA) {
    bool success;
    uint8_t tmp;
    success = read( MP8862_REG_IOUT_LIM , &tmp, 1 );
    current_mA = tmp * 50; // scale by 50 mA / LSB
    return success;
}

bool MP8862::setVoltageSetpoint_mV(uint16_t voltage_mV) {
    if(voltage_mV > 20485){ // highest permitted value producing int part 2047
        voltage_mV = 20485; // 20.47 V max (VOUT = 2047)
    }
    uint8_t tmp[2];
    uint32_t voltage_raw = (voltage_mV * 819) >> 13; // * 0.1 (~ 819/8192)
    tmp[0] = voltage_raw & 0x07; // VOUT_L value
    tmp[1] = (voltage_raw >> 3) & 0xFF; // VOUT_H value
    return write( MP8862_REG_VOUT_L , tmp, 2 );
}

bool MP8862::readVoltageSetpoint_mV(uint16_t &voltage_mV) {
    bool success;
    uint8_t tmp[2];
    success = read( MP8862_REG_VOUT_L , tmp, 2 );
    voltage_mV = (tmp[1] << 3 | (tmp[0] & 0x07)) * 10; // scale by 10 mV / LSB
    return success;
}
