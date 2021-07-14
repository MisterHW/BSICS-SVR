#include "devices.h"

/* ---------------------------------------------------------------------------*/

bool PeripheralDeviceGroup::init(I2C_HandleTypeDef* _hI2C)
{
    bool res = true;

    hI2C = _hI2C;

    res &= gpio_exp.init( hI2C, PCA9536_ADDR_0x41 );

    res &= octal_spst[0].init( hI2C, ADG715_ADDR_0x49 );
    res &= octal_spst[1].init( hI2C, ADG715_ADDR_0x4A );
    res &= octal_spst[2].init( hI2C, ADG715_ADDR_0x4B );

    return res;
};

/* ---------------------------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

PeripheralDeviceGroup DeviceGroups[2];

bool Devices_Init( ) {
    bool res;
    res  = DeviceGroups[0].init(&hi2c1);
    res &= DeviceGroups[1].init(&hi2c2);
    return res;
};


void Devices_Update( ) {

}