//
//
//

#include "i2c_wrapper.h"
#include "stm32f7xx_hal_i2c.h"

I2C_wrapper_stm32hal I2C_handler;


bool I2C_wrapper_stm32hal::sendToSlave(I2C_peripheral device, uint8_t *data, uint16_t len) {
    return true;
}

bool I2C_wrapper_stm32hal::sendToSlaveBegin(I2C_peripheral device, uint8_t len) {
    return false;
}

bool I2C_wrapper_stm32hal::sendToSlaveByte(I2C_peripheral device, uint8_t data) {
    return false;
}

bool I2C_wrapper_stm32hal::sendToSlaveBytes(I2C_peripheral device, uint8_t *data, uint8_t len) {
    return false;
}

bool I2C_wrapper_stm32hal::sendToSlaveEnd(I2C_peripheral device) {
    return false;
}


bool I2C_wrapper_stm32hal::receiveFromSlave(I2C_peripheral device, uint32_t count, uint8_t *buffer) {
    return false;
}

bool I2C_wrapper_stm32hal::receiveFromSlave(I2C_peripheral device, uint8_t len) {
    return false;
}
