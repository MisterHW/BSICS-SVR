//
//
//

#include "ADG715.h"

bool ADG715::init(I2C_HandleTypeDef *_hI2C, ADG715_address addr) {
    hI2C = _hI2C;
    deviceAddress = addr;
    return isReady();
}

bool ADG715::isReady( ) {
    return HAL_OK == HAL_I2C_IsDeviceReady(hI2C, deviceAddress << 1, 3, 5);
}

bool ADG715::writeSwitchStates(uint8_t states) {
    // S
    // deviceAddress << 1 | 0 : ACK : STATES
    // P
    return HAL_OK == HAL_I2C_Master_Transmit(hI2C, deviceAddress << 1, &states, 1, 5);
}

bool ADG715::readSwitchStates( uint8_t& states ) {
    // S
    // deviceAddress << 1 | 1 : ACK : STATES
    // P
    return HAL_OK == HAL_I2C_Master_Receive(hI2C, deviceAddress << 1, &states, 1, 5);
}



