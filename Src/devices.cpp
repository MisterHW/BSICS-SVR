#include "devices.h"

/* ---------------------------------------------------------------------------*/



bool PeripheralDeviceGroup::init(I2C_HandleTypeDef* _hI2C)
{
    bool res = true;

    hI2C = _hI2C;

    res &= gpio_exp.init(hI2C, PCA9536_ADDR_0x41);

    // res &= dcdc_hi.init(I2C_handler, hI2C, MP8862_ADDR_0x6D);
    // res &= dcdc_lo.init(I2C_handler, hI2C, MP8862_ADDR_0x6F);

    res &= octal_spst[0].init(hI2C, ADG715_ADDR_0x49);
    res &= octal_spst[1].init(hI2C, ADG715_ADDR_0x4A);
    res &= octal_spst[2].init(hI2C, ADG715_ADDR_0x4B);

    res &= temp_sensor[0].init(hI2C, MCP9808_ADDR_0x19);
    res &= temp_sensor[1].init(hI2C, MCP9808_ADDR_0x1A);
    res &= temp_sensor[2].init(hI2C, MCP9808_ADDR_0x1B);

    res &= adc[0].init(hI2C, MCP342x_ADDR_0x6C);
    res &= adc[1].init(hI2C, MCP342x_ADDR_0x6A);
    res &= adc[2].init(hI2C, MCP342x_ADDR_0x6E);

    // res &= disp.init(I2C_handler, hI2C, SSD1306_ADDR_0x3C);

    return res;
}

bool PeripheralDeviceGroup::configureDefaults() {
    bool res = true;

    res &= octal_spst[0].writeSwitchStates( ADG715_S1 | ADG715_S2 | ADG715_S3 | ADG715_S4 );
    res &= octal_spst[1].writeSwitchStates( ADG715_S1 | ADG715_S2 | ADG715_S3 | ADG715_S4 );
    res &= octal_spst[2].writeSwitchStates( ADG715_S1 | ADG715_S2 | ADG715_S3 | ADG715_S4 );

    res &= gpio_exp.writeRegister( PCA9536_PORT0_DIRECTION, PCA9536::REG_VALUE_SET_AS_INPUTS(0xF) );

    return res;
}

/* ---------------------------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

PeripheralDeviceGroup DeviceGroups[2];

bool Devices_init( ) {
    bool res;
    res  = DeviceGroups[0].init(&hi2c1);
    res &= DeviceGroups[1].init(&hi2c2);
    return res;
};

bool Devices_configure_defaults() {
    bool res;
    res  = DeviceGroups[0].configureDefaults( );
    res &= DeviceGroups[1].configureDefaults( );
    return res;
};

void Devices_update( ) {

}