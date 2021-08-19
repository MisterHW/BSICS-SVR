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

enum gpio_exp_bits {
    GPIO_EXP_0_DC_EN1_HI = 1 << 0 ,
    GPIO_EXP_1_DC_EN2_LO = 1 << 1 ,
    GPIO_EXP_2_DC_ALTN   = 1 << 2 ,
    GPIO_EXP_3_RDY       = 1 << 3 ,
};

class PeripheralDeviceGroup {
private:
    uint8_t refresh_phase = 0xFF; // ADC channel sequencer state (0xFF: before first conversion, then 0->1->0->1>...)
    static bool report(bool success, const char *s);

    void draw_start_screen();
    void draw_page_summary();
    void draw_page_channel_info();
public:
    struct {
        uint8_t gpo = 0;      // value to be sent to device (async mode)
        uint8_t prev_gpo = 0; // last value sent
        uint8_t gpi = 0;      // read-back
    } gpio_exp_data;

    struct {
        uint8_t value = 0; // value to be sent to device (async mode)
        uint8_t prev  = 0; // last value sent
    } octal_spst_data[3];

    struct {
        /* CH1 : V_COM = 3.371428(5) * V_CH1 -> coef_x1024 ~= 3.3714285 * 1024 ~=  3452
         * CH2 : VCC   = 13.66666(6) * V_CH2 -> coef_x1024 ~= 13.666666 * 1024 ~= 13995
         * MCP342x coef_x1024 and coef_x4000 must not exceed 16383. Shift result decimal point to x10 µV
         * by dividing scaling factors by 10 : {13485, 54667} -> {1349, 5467}
         */
        int16_t coef_x4000[2] = {1349, 5467};   // x10 µV conversion factors (reflecting input scaling x PGA)
        int16_t coef_x1024[2] = {3452, 13995};  // mV conversion factors (reflecting input scaling x PGA)
        int32_t ch_raw[2] = {0, 0};             // conversion results
        int32_t device_voltages_mV[2] = {0, 0}; // calculated voltages (V_lo, V_hi w.r.t. COM)
    } adc_data[3];

    struct {
        MCP9808_T T_raw {};
        int32_t T_mdegC = 0;
    } temp_sensor_data[3];

    typedef struct dcdc_data {
        uint16_t VOUT_mV;
        uint16_t VOUT_prev_mV;
        MP8862_REG_CTL1_bits CTL1;
        MP8862_REG_CTL1_bits CTL1_prev;
        uint16_t fault_counter;
    };
    dcdc_data dcdc_hi_data { 3900 , 3900 , MP8862_CTL1_DEFAULT_OUTPUT_ON , MP8862_CTL1_DEFAULT_OUTPUT_ON};
    dcdc_data dcdc_lo_data { 2700 , 2700 , MP8862_CTL1_DEFAULT_OUTPUT_ON , MP8862_CTL1_DEFAULT_OUTPUT_ON};

    // common devices
    PCA9536 gpio_exp{};
    MP8862 dcdc_hi{};
    MP8862 dcdc_lo{};
    SSD1306_128x32 status_display;
    bool status_display_rotated180 {false};
    char identifier_string[21] {};
    uint8_t display_page_index {0};
    // 24CXX eeprom;

    // per-channel devices (CH1, CH2, CH3)
    ADG715  octal_spst[3]{};
    MCP9808 temp_sensor[3]{};
    MCP3423 adc[3]{}; // MCP3423 dual-channel 18 bit ADC

    // device group state and peripheral device handle
    bool initialized {false};
    uint8_t group_index = 0;
    I2C_HandleTypeDef* hI2C {nullptr};

    // device group methods

    // stage 0
    bool init_0(I2C_HandleTypeDef* _hI2C, uint8_t index = 0);
    bool configureDefaults_0( );
    // stage 1
    bool init_1();
    bool configureDefaults_1( );

    bool readConversionResults( );
    bool writeChanges( );
    bool updateDisplay( );
};

#define DeviceGroupCount 2
#define DeviceGroupChannelCount 3
extern PeripheralDeviceGroup DeviceGroup[DeviceGroupCount];
extern uint8_t DeviceGroupIndex;

/* --------------------------------------------------------------------------- */
#endif


#ifdef __cplusplus
extern "C" {
#else
#include <stdbool.h>
#endif

bool Devices_init_0();
bool Devices_configure_defaults_0();

bool Devices_init_1();
bool Devices_configure_defaults_1();

bool Devices_refresh(bool read_slow_conversion_results);

#ifdef __cplusplus
}
#endif

#endif /* BSICS_SVR_DEVICES_H */

/**** END OF FILE ****/
