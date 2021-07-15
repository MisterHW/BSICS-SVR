//
//
//

#ifndef __I2C_DEF_H
#define __I2C_DEF_H

#include "main.h"

using I2C_peripheral = I2C_HandleTypeDef* ;

class I2C_MASTER_IF {
public:
    virtual bool sendToSlave(I2C_peripheral device, uint8_t* data, uint16_t len) = 0;
    virtual bool sendToSlaveBegin(I2C_peripheral device, uint8_t len) = 0;
    virtual bool sendToSlaveByte(I2C_peripheral device, uint8_t data) = 0;
    virtual bool sendToSlaveBytes(I2C_peripheral device, uint8_t* data, uint8_t len) = 0;
    virtual bool sendToSlaveEnd(I2C_peripheral device) = 0;
    virtual bool receiveFromSlave(I2C_peripheral device, uint32_t count, uint8_t* buffer) = 0;
    virtual bool receiveFromSlave(I2C_peripheral device, uint8_t len) = 0;
};

class I2C_SLAVE_IF {
public:
    virtual bool sendToMaster(I2C_peripheral device, uint8_t* data, uint8_t len) = 0;
    virtual bool receiveFromMaster(I2C_peripheral device, uint32_t count, uint8_t* buffer) = 0;
};

#endif // __DEF_H
