#include "devices.h"
#include "i2c_wrapper.h"
/* ---------------------------------------------------------------------------*/



bool PeripheralDeviceGroup::init(I2C_HandleTypeDef* _hI2C)
{
    bool res = true;

    hI2C = _hI2C;

    res &= gpio_exp.init(I2C_handler, hI2C, PCA9536_ADDR_0x41);

    res &= dcdc_hi.init(I2C_handler, hI2C, MP8862_ADDR_0x6D);
    res &= dcdc_lo.init(I2C_handler, hI2C, MP8862_ADDR_0x6F);

    res &= octal_spst[0].init(I2C_handler, hI2C, ADG715_ADDR_0x49);
    res &= octal_spst[1].init(I2C_handler, hI2C, ADG715_ADDR_0x4A);
    res &= octal_spst[2].init(I2C_handler, hI2C, ADG715_ADDR_0x4B);

    res &= temp_sensor[0].init(I2C_handler, hI2C, MCP9808_ADDR_0x19);
    res &= temp_sensor[1].init(I2C_handler, hI2C, MCP9808_ADDR_0x1A);
    res &= temp_sensor[2].init(I2C_handler, hI2C, MCP9808_ADDR_0x1B);

    res &= adc[0].init(I2C_handler, hI2C, MCP342x_ADDR_0x6C);
    res &= adc[1].init(I2C_handler, hI2C, MCP342x_ADDR_0x6A);
    res &= adc[2].init(I2C_handler, hI2C, MCP342x_ADDR_0x6E);

    return res;
}

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