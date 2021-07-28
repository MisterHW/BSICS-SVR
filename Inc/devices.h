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
#include "SSD1306.h"

class PeripheralDeviceGroup {
private:
    static bool report(bool success, const char *s);
public:
    bool initialized = false;
    I2C_HandleTypeDef* hI2C = nullptr;

    // common devices
    PCA9536 gpio_exp{};
    // MP8862 dcdc_hi{};
    // MP8862 dcdc_lo{};
    SSD1306_128x32 status_display;
    // 24CXX eeprom;

    // per-channel devices (CH1, CH2, CH3)
    ADG715  octal_spst[3]{};
    MCP9808 temp_sensor[3]{};
    MCP3423 adc[3]{}; // MCP3423 dual-channel 18 bit ADC

    bool init(I2C_HandleTypeDef* _hI2C);
    bool configureDefaults( );
};

#define DeviceGroupCount 2
extern PeripheralDeviceGroup DeviceGroups[DeviceGroupCount];
extern uint8_t DeviceGroupIndex;

/* --------------------------------------------------------------------------- */
#endif


#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif

bool Devices_init( );
void Devices_update( );
bool Devices_configure_defaults();

#ifdef __cplusplus
}
#endif

#endif /* BSICS_SVR_DEVICES_H */

/**** END OF FILE ****/
