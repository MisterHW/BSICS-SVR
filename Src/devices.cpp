#include "devices.h"

#include "stdio.h"
#include "stm32f7xx_hal.h"

/* ---------------------------------------------------------------------------*/

extern UART_HandleTypeDef huart3;

bool PeripheralDeviceGroup::init(I2C_HandleTypeDef* _hI2C, uint8_t index)
{
    bool res = true;

    hI2C = _hI2C;
    group_index = index;

    res &= report(gpio_exp.init(hI2C, PCA9536_ADDR_0x41) , "0x41 PCA9536" );

    res &= report( dcdc_hi.init(hI2C, MP8862_ADDR_0x6D), "0x6D HI MP8862" );
    res &= report( dcdc_lo.init(hI2C, MP8862_ADDR_0x6F), "0x6F LO MP8862" );

    res &= report( octal_spst[0].init(hI2C, ADG715_ADDR_0x49) , "0x49 CH1 ADG715" );
    res &= report( octal_spst[1].init(hI2C, ADG715_ADDR_0x4A) , "0x4A CH2 ADG715" );
    res &= report( octal_spst[2].init(hI2C, ADG715_ADDR_0x4B) , "0x4B CH3 ADG715" );

    res &= report( temp_sensor[0].init(hI2C, MCP9808_ADDR_0x19) , "0x19 CH1 MCP9808" );
    res &= report( temp_sensor[1].init(hI2C, MCP9808_ADDR_0x1A) , "0x1A CH2 MCP9808" );
    res &= report( temp_sensor[2].init(hI2C, MCP9808_ADDR_0x1B) , "0x1B CH3 MCP9808" );

    res &= report( adc[0].init(hI2C, MCP342x_ADDR_0x6C) , "0x6C CH1 MCP3423" );
    res &= report( adc[1].init(hI2C, MCP342x_ADDR_0x6A) , "0x6A CH2 MCP3423" );
    res &= report( adc[2].init(hI2C, MCP342x_ADDR_0x6E) , "0x6E CH3 MCP3423" );

    res &= report(status_display.init(hI2C, SSD1306_ADDR_0x3C, false, false), "0x3C SSD1306" );

    initialized = res;
    return res;
}

bool PeripheralDeviceGroup::configureDefaults() {
    bool res = true;

    octal_spst_data[0].value = ADG715_S1 | ADG715_S2 | ADG715_S3 | ADG715_S4;
    octal_spst_data[1].value = ADG715_S1 | ADG715_S2 | ADG715_S3 | ADG715_S4;
    octal_spst_data[2].value = ADG715_S1 | ADG715_S2 | ADG715_S3 | ADG715_S4;
    octal_spst_data[0].prev = octal_spst_data[0].value;
    octal_spst_data[1].prev = octal_spst_data[1].value;
    octal_spst_data[2].prev = octal_spst_data[2].value;

    res &= octal_spst[0].writeSwitchStates( octal_spst_data[0].value );
    res &= octal_spst[1].writeSwitchStates( octal_spst_data[1].value );
    res &= octal_spst[2].writeSwitchStates( octal_spst_data[2].value );

    res &= gpio_exp.writeRegister( PCA9536_PORT0_DIRECTION, PCA9536::REG_VALUE_SET_AS_INPUTS(0xF) );

    res &= temp_sensor[0].writeReg16(MCP9808_REG16_config, MP9808_CFG_default);
    res &= temp_sensor[1].writeReg16(MCP9808_REG16_config, MP9808_CFG_default);
    res &= temp_sensor[2].writeReg16(MCP9808_REG16_config, MP9808_CFG_default);

    MCP342x_config adc_startup_cfg;
    adc_startup_cfg = MCP342X_RES_16BIT | MCP342X_GAIN_1X | MCP342X_MODE_ONESHOT | MCP3423_CHANNEL_1;
    res &= adc[0].writeConfig( adc_startup_cfg );
    res &= adc[1].writeConfig( adc_startup_cfg );
    res &= adc[2].writeConfig( adc_startup_cfg );

    return res;
}

bool PeripheralDeviceGroup::report(bool success, const char *s) {
    printf(success ? "[x] " : "[_] ");
    printf(s);
    printf("\r\n");
    return success;
}

bool PeripheralDeviceGroup::readConversionResults() {
    bool res = true;

    // read ADC channels, trigger conversion of next channel, calculate voltages when set of channels acquired
    MCP342x_config cfg;
    uint8_t refresh_phase_next;

    switch(refresh_phase){
        case 0x00: {
            cfg = MCP342X_GAIN_1X | MCP342X_RES_16BIT | MCP342X_MODE_ONESHOT_START | MCP3423_CHANNEL_2;
            refresh_phase_next = 0x01;
        }; break;
        default : { // case 0x01, 0xFF
            cfg = MCP342X_GAIN_1X | MCP342X_RES_16BIT | MCP342X_MODE_ONESHOT_START | MCP3423_CHANNEL_1;
            refresh_phase_next = 0x00;
        }
    }

    for(int i = 0; i < 3; i++)
    {
        // read temperature sensors
        if(temp_sensor[i].initialized){
            if(temp_sensor[i].readReg16(MCP9808_REG16_T_ambient, temp_sensor_data[i].T_raw)){
                temp_sensor_data[i].T_mdegC = MCP9808::raw_to_millidegC(temp_sensor_data[i].T_raw);
            } else {
                // temp_sensor[i].initialized = false;
                temp_sensor_data[i].T_raw   = 0;
                temp_sensor_data[i].T_mdegC = 0;
                res = false;
            }
        } else {
            temp_sensor_data[i].T_raw   = 0;
            temp_sensor_data[i].T_mdegC = 0;
        }

        if(adc[i].initialized){
            switch(refresh_phase){
                case 0:{
                    // read CH1 result, next conversion will be CH2
                    adc_data[i].ch_raw[0] = 0;
                    res &= adc[i].readConvResult(adc_data[i].ch_raw[0]);
                }; break;
                case 1:{
                    // read CH2 result, next conversion will be CH1
                    adc_data[i].ch_raw[1] = 0;
                    res &= adc[i].readConvResult(adc_data[i].ch_raw[1]);
                    /* calculate new voltages:
                     * V_COM = 3.371428(5) * V_CH1
                     * VCC   = 13.66666(6) * V_CH2
                     * V_LO = -V_COM
                     * V_HI = VCC - V_COM
                     */
                    adc_data[i].device_voltages_mV[0] = // V_lo =
                            -MCP3423::raw_to_mV(adc_data[i].ch_raw[0], adc_data[i].coef_x1024[0]);
                    adc_data[i].device_voltages_mV[1] = // V_hi =
                            MCP3423::raw_to_mV(adc_data[i].ch_raw[1],adc_data[i].coef_x1024[1])
                            + adc_data[i].device_voltages_mV[0];
                    adc_data[i].ch_raw[0] = 0;
                    adc_data[i].ch_raw[1] = 0;
                }; break;
                default:; // No conversion results available, continue with writeConfig() to trigger first conversion.
            }
            if(not adc[i].writeConfig(cfg)) { // try to initiate next conversion
                adc[i].initialized = false;
                res = false;
            }
        } else {
            adc_data[i].device_voltages_mV[0] = 0;
            adc_data[i].device_voltages_mV[1] = 0;
            adc_data[i].ch_raw[0] = 0;
            adc_data[i].ch_raw[1] = 0;
        }
    }

    if(refresh_phase == 1){
        res &= updateDisplay();
    }

    refresh_phase = refresh_phase_next;

    return res;
}

bool PeripheralDeviceGroup::writeChanges() {
    bool res = true;

    // readConversionResults common devices
    if(gpio_exp.initialized && (gpio_exp_data.gpo != gpio_exp_data.prev_gpo)){
        if(gpio_exp.writeRegister(PCA9536_PORT0_OUTPUT, gpio_exp_data.gpo)){
            gpio_exp_data.prev_gpo = gpio_exp_data.gpo;
            res &= gpio_exp.readRegister(PCA9536_PORT0_INPUT, gpio_exp_data.gpi);
        } else {
            // gpio_exp.initialized = false; // un-commment to remove device and prevent retry
            res = false;
        }
    }

    for(int i = 0; i < 3; i++) {
        // update SPSTs
        if (octal_spst[i].initialized && (octal_spst_data[i].value != octal_spst_data[i].prev)) {
            if (octal_spst[i].writeSwitchStates((ADG715_switches) octal_spst_data[i].value)) {
                octal_spst_data[i].prev = octal_spst_data[i].value;
            } else {
                // octal_spst[i].initialized = false; // un-commment to remove device and prevent retry
                res = false;
            }
        }
    }

    return res;
}

bool PeripheralDeviceGroup::updateDisplay() {
    printf("\r\nGroup%d:\r\n   \tCH1\tCH2\tCH3\r\n", group_index);

    // update OLED display
    // todo

    /// debug output via UART
    printf("SW \t0x%02x\t0x%02x\t0x%02x\r\n",
           octal_spst_data[0].prev,
           octal_spst_data[1].prev,
           octal_spst_data[2].prev );
    printf("HI \t%d\t%d\t%d\r\n",
           (int)adc_data[0].device_voltages_mV[1],
           (int)adc_data[1].device_voltages_mV[1],
           (int)adc_data[2].device_voltages_mV[1] );
    printf("LO \t%d\t%d\t%d\r\n",
           (int)adc_data[0].device_voltages_mV[0],
           (int)adc_data[1].device_voltages_mV[0],
           (int)adc_data[2].device_voltages_mV[0] );
    printf("T  \t%d\t%d\t%d\r\n",
           (int)temp_sensor_data[0].T_mdegC,
           (int)temp_sensor_data[1].T_mdegC,
           (int)temp_sensor_data[2].T_mdegC );

    return true;
}

/* ---------------------------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

PeripheralDeviceGroup DeviceGroup[DeviceGroupCount];
uint8_t DeviceGroupIndex = 0;

bool Devices_init( ) {
    bool res;
    printf("I2C1 :\r\n");
    res  = DeviceGroup[0].init(&hi2c1, 0);
    printf("I2C2 :\r\n");
    res &= DeviceGroup[1].init(&hi2c2, 1);
    printf("\n");
    return res;
};

bool Devices_configure_defaults() {
    bool res;
    res  = DeviceGroup[0].configureDefaults( );
    res &= DeviceGroup[1].configureDefaults( );
    return res;
};

bool Devices_refresh(bool read_slow_conversion_results) {
    bool res;
    res  = DeviceGroup[0].writeChanges();
    res &= DeviceGroup[1].writeChanges();
    if(read_slow_conversion_results) {
        res &= DeviceGroup[0].readConversionResults();
        res &= DeviceGroup[1].readConversionResults();
    }
    return res;
}