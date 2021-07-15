//
//
//

#ifndef BSICS_SVR_I2C_WRAPPER_H
#define BSICS_SVR_I2C_WRAPPER_H

#include "i2c_def.h"

class I2C_wrapper_stm32hal : public I2C_MASTER_IF {
    bool sendToSlave(I2C_peripheral device, uint8_t* data, uint16_t len) override ;
    bool sendToSlaveBegin(I2C_peripheral device, uint8_t len) override ;
    bool sendToSlaveByte(I2C_peripheral device, uint8_t data) override ;
    bool sendToSlaveBytes(I2C_peripheral device, uint8_t* data, uint8_t len) override ;
    bool sendToSlaveEnd(I2C_peripheral device) override ;
    bool receiveFromSlave(I2C_peripheral device, uint32_t count, uint8_t* buffer) override ;
    bool receiveFromSlave(I2C_peripheral device, uint8_t len) override ;
};

extern I2C_wrapper_stm32hal I2C_handler;

#endif //BSICS_SVR_I2C_WRAPPER_H
