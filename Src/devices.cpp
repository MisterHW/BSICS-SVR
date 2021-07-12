#include "devices.h"
#include "ssd1306.h"

/* ---------------------------------------------------------------------------*/

bool TPeripheralDeviceGroup::init(I2C_HandleTypeDef _hI2C)
{
    this->hI2C = _hI2C;
    return true;
};

/* ---------------------------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

TPeripheralDeviceGroup DeviceGroups[2];

bool Devices_Init(void) {
    bool res;
    res  = DeviceGroups[0].init(hi2c1);
    res &= DeviceGroups[1].init(hi2c2);
    return res;
};
