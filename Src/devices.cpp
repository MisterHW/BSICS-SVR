#include "devices.h"

#include "stdio.h"
#include "stm32f7xx_hal.h"

/* ---------------------------------------------------------------------------*/

extern UART_HandleTypeDef huart3;

bool PeripheralDeviceGroup::init(I2C_HandleTypeDef* _hI2C)
{
    bool res = true;

    hI2C = _hI2C;

    res &= report(gpio_exp.init(hI2C, PCA9536_ADDR_0x41) , "0x41 PCA9536" );

    // res &= report( dcdc_hi.init(I2C_handler, hI2C, MP8862_ADDR_0x6D), "0x6D MP8862" );
    // res &= report( dcdc_lo.init(I2C_handler, hI2C, MP8862_ADDR_0x6F), "0x6F MP8862" );

    res &= report( octal_spst[0].init(hI2C, ADG715_ADDR_0x49) , "0x49 CH1 ADG715" );
    res &= report( octal_spst[1].init(hI2C, ADG715_ADDR_0x4A) , "0x4A CH2 ADG715" );
    res &= report( octal_spst[2].init(hI2C, ADG715_ADDR_0x4B) , "0x4B CH3 ADG715" );

    res &= report( temp_sensor[0].init(hI2C, MCP9808_ADDR_0x19) , "0x19 CH1 MCP9808" );
    res &= report( temp_sensor[1].init(hI2C, MCP9808_ADDR_0x1A) , "0x1A CH2 MCP9808" );
    res &= report( temp_sensor[2].init(hI2C, MCP9808_ADDR_0x1B) , "0x1B CH3 MCP9808" );

    res &= report( adc[0].init(hI2C, MCP342x_ADDR_0x6C) , "0x6C CH1 MCP3423" );
    res &= report( adc[1].init(hI2C, MCP342x_ADDR_0x6A) , "0x6A CH2 MCP3423" );
    res &= report( adc[2].init(hI2C, MCP342x_ADDR_0x6E) , "0x6E CH3 MCP3423" );

    // res &= report( disp.init(I2C_handler, hI2C, SSD1306_ADDR_0x3C), "0x3C SSD1306" );

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

bool PeripheralDeviceGroup::report(bool success, const char *s) {
    printf(success ? "[x] " : "[_] ");
    printf(s);
    printf("\n");
    return success;
}

/* ---------------------------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

PeripheralDeviceGroup DeviceGroups[2];

bool Devices_init( ) {
    bool res;
    printf("I2C1 :\n");
    res  = DeviceGroups[0].init(&hi2c1);
    printf("I2C2 :\n");
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