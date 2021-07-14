/**
  ******************************************************************************
  * File Name          : devices.h
  * Description        : BSICS Server peripheral devices wrapper.
  *
  ******************************************************************************
  */

#ifndef BSICS_SVR_DEVICES_H
#define BSICS_SVR_DEVICES_H

#include "main.h"

#ifdef __cplusplus
/* --------------------------------------------------------------------------- */

#include "ADG715.h"
#include "PCA953x.h"
#include "MCP9808.h"
#include "MCP342x.h"
#include "MP8862.h"

class PeripheralDeviceGroup {
public:
    I2C_HandleTypeDef* hI2C;

    // common devices
    PCA9536 gpio_exp;
    MP8862 dcdc_hi;
    MP8862 dcdc_lo;
    // SSD1306 status_display;
    // 24CXX eeprom;

    // per-channel devices (CH1, CH2, CH3)
    ADG715  octal_spst[3];
    MCP9808 temp_sensor[3];
    MCP3423 adc[3]; // MCP3423 dual-channel 18 bit ADC

    bool init(I2C_HandleTypeDef* _hI2C);
};

/* --------------------------------------------------------------------------- */
#endif


#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif

bool Devices_Init( );
void Devices_Update( );

#ifdef __cplusplus
}
#endif

#endif /* BSICS_SVR_DEVICES_H */

/**** END OF FILE ****/
