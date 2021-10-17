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

/* GPIO pinout names:
 *
 *   D0   D1   D2   D3   D4   D5   D6   D7
 *  PG9  PG14 PF15 PE13 PF14 PE11 PE9  PF13
 *
 *   D8   D9   D10  D11  D12  D13  D14  D15
 *  PF12 PD15 PD14  PA7  PA6  PA5  PB9  PB8
 *
 *  D11 / PA7 is already in use (RMII CRS_DV)
 */

typedef struct {
    GPIO_TypeDef * port;
    uint16_t pin;
} gpio_lookup_t;

typedef uint32_t GPIO_packed_bits_t;

const uint8_t GPIO_map_size = 14; // must be 0 .. 31, corresponding to uint32_t used for mask and bits.
const gpio_lookup_t GPIO_map[] = {
        {GPIOG, GPIO_PIN_9 }, // D0
        {GPIOG, GPIO_PIN_14}, // D1
        {GPIOF, GPIO_PIN_15}, // D2
        {GPIOE, GPIO_PIN_13}, // D3
        {GPIOF, GPIO_PIN_14}, // D4
        {GPIOE, GPIO_PIN_11}, // D5
        {GPIOE, GPIO_PIN_9 }, // D6
        {GPIOF, GPIO_PIN_13}, // D7
        {GPIOF, GPIO_PIN_12}, // D8
        {GPIOD, GPIO_PIN_15}, // D9
        {GPIOD, GPIO_PIN_14}, // D10
        {NULL , GPIO_PIN_7 }, // D11 : GPIOA pin 7 not available (RMII CRS_DV)
        {GPIOA, GPIO_PIN_6 }, // D12
        {GPIOA, GPIO_PIN_5 }, // D13
        {NULL , GPIO_PIN_9 }, // D14 : GPIOB pin 9 not available (I2C1)
        {NULL , GPIO_PIN_8 }, // D15 : GPIOB pin 8 not available (I2C1)
};

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
    uint8_t refresh_phase = 0xFF; // ADC channel sequencer state (0xFF: before first conversion, then 0->1->0->1>...)
    static bool report(bool success, const char *s);

    void draw_start_screen();
    void draw_page_summary();
    void draw_page_channel_info();
public:
    static const size_t n_channels = 3;

    enum dcdc_list_indices {
        dcdc_hi = 0, // "DCDC1" address 0x6D
        dcdc_lo = 1, // "DCDC2" address 0x6F
        n_dcdcs // length (last index + 1)
    };

    enum gpio_exp_bits {
        GPIO_EXP_0_DC_EN1_HI = 1 << 0 ,
        GPIO_EXP_1_DC_EN2_LO = 1 << 1 ,
        GPIO_EXP_2_DC_ALTN   = 1 << 2 ,
        GPIO_EXP_3_RDY       = 1 << 3 ,
    };

    struct {
        uint8_t gpo = 0;      // value to be sent to device (async mode)
        uint8_t prev_gpo = 0; // last value sent
        uint8_t gpi = 0;      // read-back
    } gpio_exp_data;

    struct {
        uint8_t value = 0; // value to be sent to device (async mode)
        uint8_t prev  = 0; // last value sent
    } octal_spst_data[n_channels];

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
    } adc_data[n_channels];

    struct {
        MCP9808_T T_raw {};
        int32_t T_mdegC {};
    } temp_sensor_data[n_channels];

    struct {
        uint16_t VOUT_mV      {};
        uint16_t VOUT_prev_mV {};
        uint16_t IOUT_mA      {};
        uint16_t IOUT_prev_mA {};
        MP8862_REG_CTL1_bits CTL1      {MP8862_CTL1_DEFAULT_OUTPUT_ON};
        MP8862_REG_CTL1_bits CTL1_prev {MP8862_CTL1_DEFAULT_OUTPUT_ON};
        uint16_t fault_counter {};
    } dcdc_data[n_dcdcs] = {
            // start-up defaults
            { 3900 , 3900, 3000 , 3000},
            { 2700 , 2700, 3000 , 3000},
    };

    // common devices
    PCA9536 gpio_exp{};
    MP8862 dcdc[n_dcdcs];
    SSD1306_128x32 status_display;
    bool status_display_rotated180 {false};
    char identifier_string[21] {};
    uint8_t display_page_index {0};
    // 24CXX eeprom;

    // per-channel devices (CH1, CH2, CH3)
    ADG715  octal_spst[n_channels]{};
    MCP9808 temp_sensor[n_channels]{};
    MCP3423 adc[n_channels]{}; // MCP3423 dual-channel 18 bit ADC

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
extern PeripheralDeviceGroup DeviceGroup[DeviceGroupCount];
extern uint8_t DeviceGroupIndex;
extern bool PeriodMeasReportingViaUART;

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

void updateDigitalOutputs(GPIO_packed_bits_t mask, GPIO_packed_bits_t bits);
GPIO_packed_bits_t getDigitalOutputs();
bool getDigitalOutput(uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* BSICS_SVR_DEVICES_H */

/**** END OF FILE ****/
