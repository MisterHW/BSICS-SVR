#include "devices.h"
#include "scpi_server.h"
#include "main.h"

#include "stdio.h"
#include "stm32f7xx_hal.h"

/* ---------------------------------------------------------------------------*/

extern UART_HandleTypeDef huart3;
extern uint8_t IP_ADDRESS[4];

#define DCDC_ID(group_idx, device_idx) (((group_idx) << 8) | ((device_idx) & 0xFF))
#define DCDC_ID_TO_GROUP(ID)  ((ID) >> 8)
#define DCDC_ID_TO_DEVICE(ID) ((ID) & 0xFF)

bool dcdc_hw_EN(uint16_t ID, uint8_t state){
    uint8_t gidx = DCDC_ID_TO_GROUP(ID);
    uint8_t gpio_bit_val;
    uint8_t gpio_bit_mask;

    switch(DCDC_ID_TO_DEVICE(ID)){
        case PeripheralDeviceGroup::dcdc_list_indices::dcdc_hi :
            gpio_bit_val  = state == 0 ? 0 : PeripheralDeviceGroup::GPIO_EXP_0_DC_EN1_HI ;
            gpio_bit_mask = ~PeripheralDeviceGroup::GPIO_EXP_0_DC_EN1_HI;
            break;
        case PeripheralDeviceGroup::dcdc_list_indices::dcdc_lo:
            gpio_bit_val  = state == 0 ? 0 : PeripheralDeviceGroup::GPIO_EXP_1_DC_EN2_LO ;
            gpio_bit_mask = ~PeripheralDeviceGroup::GPIO_EXP_1_DC_EN2_LO;
            break;
        default:
            return false; // Invalid device index.
    }

    if(DeviceGroup[gidx].gpio_exp.initialized){
        DeviceGroup[gidx].gpio_exp_data.gpo = (DeviceGroup[gidx].gpio_exp_data.gpo & gpio_bit_mask) | gpio_bit_val;
        bool success = DeviceGroup[gidx].gpio_exp.writeRegister(
                PCA9536_PORT0_OUTPUT,
                DeviceGroup[gidx].gpio_exp_data.gpo );
        if(success){
            DeviceGroup[gidx].gpio_exp_data.prev_gpo = DeviceGroup[gidx].gpio_exp_data.gpo;
        }
        return success;
    }
    return false;
}

// initializatoin stage 0: primary-side devices
bool PeripheralDeviceGroup::init_0(I2C_HandleTypeDef* _hI2C, uint8_t index)
{
    bool res = true;

    hI2C = _hI2C;
    group_index = index;

    res &= report(gpio_exp.init(hI2C, PCA9536_ADDR_0x41) , "0x41 PCA9536" );

    // Set up MP8862 class instances, result may be false if device is in power-down.
    dcdc[dcdc_hi].init(hI2C, MP8862_ADDR_0x6D);
    dcdc[dcdc_lo].init(hI2C, MP8862_ADDR_0x6F);

    res &= report(status_display.init(hI2C, SSD1306_ADDR_0x3C, false, false), "0x3C SSD1306" );

    initialized = res;
    return res;
}

// configuration stage 0: primary-side devices, DCDC power-up
bool PeripheralDeviceGroup::configureDefaults_0() {
    bool res = true, success;

    // read and apply identifier string if EEPROM  present
    draw_start_screen();

    gpio_exp_data.gpo      = 0x0;
    gpio_exp_data.prev_gpo = 0x0;
    res &= gpio_exp.writeRegister( PCA9536_PORT0_OUTPUT   , 0x0 ); // all outputs will be LOW
    res &= gpio_exp.writeRegister( PCA9536_PORT0_DIRECTION, PCA9536::REG_VALUE_SET_AS_OUTPUTS(GPIO_EXP_0_DC_EN1_HI | GPIO_EXP_1_DC_EN2_LO) );

    for(int i = 0; i < n_dcdcs; i++){
        success = dcdc[i].hardwarePowerUp( dcdc_hw_EN , DCDC_ID(group_index, i) , MP8862_RETRY_I2C_400kHz );
        if( success ) {
            dcdc[i].setVoltageSetpoint_mV(dcdc_data[i].VOUT_mV);
            dcdc[i].write(MP8862_REG_CTL1, dcdc_data[i].CTL1);
            for(int t = 0; i < 250000 / 50; t++){ // timeout ~ 250 ms (assume readPG takes 50µs at 400 kHz I2C clock - adjust when changing bus speed)
                if(dcdc[i].readPG()){
                    break;
                }
            }
            success &= dcdc[i].readPG();
        } else {
            res = false;
        }

        switch(i){
            case dcdc_list_indices::dcdc_hi : report(success, "Power-Up : 0x6D HI MP8862"); break;
            case dcdc_list_indices::dcdc_lo : report(success, "Power-Up : 0x6F LO MP8862"); break;
            default:;
        }

    }

    return res;
}

/* initialization stage 1: re-init DCDC class instances, initialize secondary-side devices
 * Secondary-side devices are now guaranteed to be supplied with power (either externally or via DCDCs.
 * If DCDC initialization failed, devices won't show up (see debug output reported via UART).
 */
bool PeripheralDeviceGroup::init_1()
{
    bool res = true;

    res &= report( dcdc[dcdc_hi].init(hI2C, MP8862_ADDR_0x6D), "0x6D HI MP8862" );
    res &= report( dcdc[dcdc_lo].init(hI2C, MP8862_ADDR_0x6F), "0x6F LO MP8862" );

    res &= report( octal_spst[0].init(hI2C, ADG715_ADDR_0x49) , "0x49 CH1 ADG715" );
    res &= report( octal_spst[1].init(hI2C, ADG715_ADDR_0x4A) , "0x4A CH2 ADG715" );
    res &= report( octal_spst[2].init(hI2C, ADG715_ADDR_0x4B) , "0x4B CH3 ADG715" );

    res &= report( temp_sensor[0].init(hI2C, MCP9808_ADDR_0x19) , "0x19 CH1 MCP9808" );
    res &= report( temp_sensor[1].init(hI2C, MCP9808_ADDR_0x1A) , "0x1A CH2 MCP9808" );
    res &= report( temp_sensor[2].init(hI2C, MCP9808_ADDR_0x1B) , "0x1B CH3 MCP9808" );

    res &= report( adc[0].init(hI2C, MCP342x_ADDR_0x6C) , "0x6C CH1 MCP3423" );
    res &= report( adc[1].init(hI2C, MCP342x_ADDR_0x6A) , "0x6A CH2 MCP3423" );
    res &= report( adc[2].init(hI2C, MCP342x_ADDR_0x6E) , "0x6E CH3 MCP3423" );

    initialized = res;
    return res;
}

// configuration stage 1: secondary-side devices
bool PeripheralDeviceGroup::configureDefaults_1() {
    bool res = true;

    // apply contents (calibration coefficients, ...)
    // ...

    octal_spst_data[0].value = ADG715_S1 | ADG715_S2 | ADG715_S3 | ADG715_S4;
    octal_spst_data[1].value = ADG715_S1 | ADG715_S2 | ADG715_S3 | ADG715_S4;
    octal_spst_data[2].value = ADG715_S1 | ADG715_S2 | ADG715_S3 | ADG715_S4;
    octal_spst_data[0].prev = octal_spst_data[0].value;
    octal_spst_data[1].prev = octal_spst_data[1].value;
    octal_spst_data[2].prev = octal_spst_data[2].value;

    res &= octal_spst[0].writeSwitchStates( octal_spst_data[0].value );
    res &= octal_spst[1].writeSwitchStates( octal_spst_data[1].value );
    res &= octal_spst[2].writeSwitchStates( octal_spst_data[2].value );

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

    for(int i = 0; i < PeripheralDeviceGroup::n_channels; i++)
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
    if(gpio_exp.initialized) {
        if ( gpio_exp_data.prev_gpo != gpio_exp_data.gpo ) {
            if (gpio_exp.writeRegister(PCA9536_PORT0_OUTPUT, gpio_exp_data.gpo)) {
                gpio_exp_data.prev_gpo = gpio_exp_data.gpo;
            } else {
                // gpio_exp.initialized = false; // un-comment to remove device and prevent retry
                res = false;
            }
        }
        // always read gpi register (!ALT, RDY)
        res &= gpio_exp.readRegister(PCA9536_PORT0_INPUT, gpio_exp_data.gpi);
    }

    for(int i = 0; i < PeripheralDeviceGroup::n_channels; i++) {
        // update SPSTs
        if (octal_spst[i].initialized && (octal_spst_data[i].prev != octal_spst_data[i].value)) {
            if (octal_spst[i].writeSwitchStates((ADG715_switches) octal_spst_data[i].value)) {
                octal_spst_data[i].prev = octal_spst_data[i].value;
            } else {
                // octal_spst[i].initialized = false; // un-comment to remove device and prevent retry
                res = false;
            }
        }
    }

    for(int i = 0; i < PeripheralDeviceGroup::n_dcdcs; i++) {
        if (dcdc[i].initialized) {

            bool update_attempted = false;
            bool update_failed    = false;

            if (dcdc_data[i].VOUT_prev_mV != dcdc_data[i].VOUT_mV) {
                update_attempted = true;
                if (dcdc[i].setVoltageSetpoint_mV(dcdc_data[i].VOUT_mV)) {
                    dcdc_data[i].VOUT_prev_mV = dcdc_data[i].VOUT_mV;
                } else {
                    update_failed = true;
                }
            }

            if (dcdc_data[i].IOUT_prev_mA != dcdc_data[i].IOUT_mA) {
                update_attempted = true;
                if (dcdc[i].setCurrentLimit_mA(dcdc_data[i].IOUT_mA)) {
                    dcdc_data[i].IOUT_prev_mA = dcdc_data[i].IOUT_mA;
                } else {
                    update_failed = true;
                }
            }

            if (dcdc_data[i].CTL1_prev != dcdc_data[i].CTL1) {
                update_attempted = true;
                if (dcdc[i].write(MP8862_REG_CTL1, dcdc_data[i].CTL1)) {
                    dcdc_data[i].CTL1_prev = dcdc_data[i].CTL1;
                } else {
                    update_failed = true;
                }
            }

            // Uncomment if() below to make periodic communication test mandatory.
            if (not update_attempted) {
                update_attempted = true;
                update_failed |= not dcdc[i].isReady();
            }

            if (update_attempted && update_failed) {
                // Increment dcdc_hi_data.fault_counter.
                // If dcdc_hi_data.fault_counter exceeds limit, there could have been a communication or brown-out problem.
                // In this case, re-initialize gpio_exp and set all hardware EN pins to 0, set dcdc_hi.initialized = false.
            }
        }
    }

    return res;
}

void PeripheralDeviceGroup::draw_start_screen() {
    if(status_display.initialized) {
        char line[22];
        // At first execution, buffer should be initialized with 0x00. Uncomment when configureDefaults_1() is used during later execution.
        // status_display.clearBuffer();

        // line 0
        status_display.setCursor( 0,0 );
        status_display.writeString(identifier_string, Font_7x10,SSD1306_color::monochrome_white);
        // line 1
        // IPv4 address IP_ADDRESS[4] is set in MX_LWIP_Init() with long execution time
        sprintf(line, "IP %d.%d.%d.%d", IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3] );
        status_display.setCursor( 0, (Font_7x10.FontHeight + 1) * 1 - 1 );
        status_display.writeString(line, Font_7x10,SSD1306_color::monochrome_white);
        // line 2
        sprintf(line, "   :%d SCPI-RAW", SCPI_DEVICE_PORT );
        status_display.setCursor( 0, (Font_7x10.FontHeight + 1) *  2 - 1 );
        status_display.writeString(line, Font_7x10,SSD1306_color::monochrome_white);

        status_display.configure_orientation(status_display_rotated180, status_display_rotated180);
        status_display.updateDisplay();
    }
}

void printBinary(char* buf, uint8_t val, uint8_t digits = 8){
    buf += (digits - 1);
    for(uint8_t i = 0; i < digits; i++){
        *buf-- = '0' + (val & 0x01);
        val >>= 1;
    }
}

#define max(A,B) ((A) > (B) ? (A) : (B))
#define min(A,B) ((A) < (B) ? (A) : (B))

void printDecimalFP(char* buf, int32_t val, uint8_t size,
                    uint8_t max_decimal_places = 0 , uint8_t log10_denominator = 0 ){
    /*
     *  log_denom    size 5          out
     *               |-----|       |-----|
     *      0       -654321         -####
     *      1        -65432.1       -####
     *      2         -6543.21      -6543
     *      3          -654.321      -654
     *      4         -65.4321      -65.4
     *      5          -6.54321      -6.5
     *      6          -0.654321     -0.6
     *      7          -0.0654321    -0.0
     */
    bool neg = val < 0;
    if(val < 0){
        val = (int16_t)(-val);
    }

    char tmp[22];
    int8_t n_total = sprintf(tmp, "%d", val); // digits before division, without sign
    int8_t n_left  = n_total - log10_denominator; // absolute digits after division
    int8_t n_left_printed  = (neg ? 1 : 0) + max( n_left , 1 ); // printed digits, at least leading zero if decimal places are printed

    if(n_left_printed > size){ // handle overflow if signed number without decimal places cannot be printed
        memset(buf + 1, '#', size - 1);
        buf[0] = neg ? '-' : '+';
        return;
    }

    bool leading_zero  = n_left <= 0;
    bool decimal_point = ( size - n_left_printed >= 2 ) && ( max_decimal_places > 0 );

    int8_t n_remaining        = size - n_left_printed - (decimal_point ? 1 : 0);
    int8_t n_right_printed    = decimal_point ? min( n_remaining , max_decimal_places ) : 0 ;
    int8_t n_left_whitespaces = n_remaining - n_right_printed;

    for(int8_t i = 0; i < n_left_whitespaces ; i++){
        *buf++ = ' ';
    }
    if(neg){
        *buf++ = '-';
    }
    if(leading_zero){
        *buf++ = '0';
    } else {
        for(int8_t i = 0; i < n_left ; i++){
            *buf++ = tmp[i];
        }
    }
    if(decimal_point){
        *buf++ = '.';
        for(int8_t i = n_left ; i < n_left + n_right_printed; i++){
            if((i >= 0) && (i < n_total)){
                *buf++ = tmp[i];
            } else {
                *buf++ = '0';
            }
        }
    }
}

void PeripheralDeviceGroup::draw_page_summary() {
    if(status_display.initialized){
        uint8_t ch_idx[3] = {2, 1, 0}; // normal display: CH3, CH2, CH1.
        if(status_display_rotated180){ // HS group needs 180° rotation and text in reversed order
            ch_idx[0] = 0;
            ch_idx[2] = 2;
        }

        status_display.clearBuffer();
        status_display.setCursor(0, 0);

        // print line 0 : SPST config for all channels
        char line[22];
        sprintf(line, "CFG 0x%02X 0x%02X 0x%02X", // non-aligned " 0x%02X  0x%02X  0x%02X",
                    octal_spst_data[ch_idx[0]].prev,
                    octal_spst_data[ch_idx[1]].prev,
                    octal_spst_data[ch_idx[2]].prev );
        status_display.writeString(line, Font_7x10, SSD1306_color::monochrome_white);

        // print line 1
        status_display.setCursor(0, (Font_7x10.FontHeight + 1) * 1 - 1);
        strcpy(line, "VHI xx.x xx.x xx.x");
        printDecimalFP(&line[ 4], adc_data[ch_idx[0]].device_voltages_mV[1], 4, 1, 3);
        printDecimalFP(&line[ 9], adc_data[ch_idx[1]].device_voltages_mV[1], 4, 1, 3);
        printDecimalFP(&line[14], adc_data[ch_idx[2]].device_voltages_mV[1], 4, 1, 3);
        status_display.writeString(line, Font_7x10, SSD1306_color::monochrome_white);
        // print line 2
        status_display.setCursor(0, (Font_7x10.FontHeight + 1) * 2 - 1);
        strcpy(line, "VLO xx.x xx.x xx.x");
        printDecimalFP(&line[ 4], adc_data[ch_idx[0]].device_voltages_mV[0], 4, 1, 3);
        printDecimalFP(&line[ 9], adc_data[ch_idx[1]].device_voltages_mV[0], 4, 1, 3);
        printDecimalFP(&line[14], adc_data[ch_idx[2]].device_voltages_mV[0], 4, 1, 3);
        status_display.writeString(line, Font_7x10, SSD1306_color::monochrome_white);


    } else {
        // success = false; // Uncomment to intepret missing display as failure to update.
    }
}


void PeripheralDeviceGroup::draw_page_channel_info() {
    if(status_display.initialized){
        uint8_t ch_idx[3] = {2, 1, 0}; // normal display: CH3, CH2, CH1.
        if( status_display_rotated180 ){ // HS group: rotated 180° (upside-down, reverse order)
            ch_idx[0] = 0;
            ch_idx[2] = 2;
        }

        status_display.clearBuffer();

        // print line 0
        status_display.setCursor(0, 0);
        char line[22];
        sprintf( line, "CH%d hhhh-llll 0x%02X",
                    ch_idx[display_page_index - 1] + 1,
                    octal_spst_data[ch_idx[display_page_index - 1]].prev );
        printBinary(&line[4],octal_spst_data[ch_idx[display_page_index - 1]].prev >> 4, 4);
        printBinary(&line[9],octal_spst_data[ch_idx[display_page_index - 1]].prev     , 4);

        status_display.writeString(line, Font_7x10, SSD1306_color::monochrome_white);

        // print line 1
        status_display.setCursor(0, (Font_7x10.FontHeight + 1) * 1 - 1);
        strcpy(line, "VHI xx.xx T xxx.xC");
        printDecimalFP(&line[ 4], adc_data[ch_idx[display_page_index - 1]].device_voltages_mV[1], 5, 2, 3);
        printDecimalFP(&line[12], temp_sensor_data[ch_idx[display_page_index - 1]].T_mdegC, 5, 2, 3);
        status_display.writeString(line, Font_7x10, SSD1306_color::monochrome_white);
        // print line 2
        status_display.setCursor(0, (Font_7x10.FontHeight + 1) * 2 - 1);
        strcpy(line, "VLO -x.xx         ");
        printDecimalFP(&line[ 4], adc_data[ch_idx[display_page_index - 1]].device_voltages_mV[0], 5, 2, 3);
        status_display.writeString(line, Font_7x10, SSD1306_color::monochrome_white);
    } else {
        // success = false; // Uncomment to intepret missing display as failure to update.
    }
}


bool PeripheralDeviceGroup::updateDisplay() {
    bool success = true;
    if(PeriodMeasReportingViaUART) {
        printf("\r\nGroup%d:\r\n   \tCH1\tCH2\tCH3\r\n", group_index);
    }
    // draw display contents
    switch( display_page_index ){
        case 1:
        case 2:
        case 3:
            draw_page_channel_info();
            break;
        default:
            draw_page_summary();
    }

    // update display contents
    if(status_display.initialized){
        success &= status_display.configure_orientation( status_display_rotated180, status_display_rotated180 );
        success &= status_display.updateDisplay();
    }

    /// debug output via UART
    if(PeriodMeasReportingViaUART){
        printf("SW \t0x%02X\t0x%02X\t0x%02X\r\n",
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
    }

    return success;
}

/* ---------------------------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

PeripheralDeviceGroup DeviceGroup[DeviceGroupCount];
uint8_t DeviceGroupIndex = 0;
bool PeriodMeasReportingViaUART = false;

bool Devices_init_0( ) {
    bool res;
    printf("INIT0 I2C1 :\r\n");
    res  = DeviceGroup[0].init_0(&hi2c1, 0);
    printf("INIT0 I2C2 :\r\n");
    res &= DeviceGroup[1].init_0(&hi2c2, 1);
    return res;
};

bool Devices_configure_defaults_0() {
    bool res;
    // set group-specific defaults (general defaults are already initialized)
    printf("CONF0 I2C1 :\r\n");
    DeviceGroup[0].status_display_rotated180 = false;
    strcpy(DeviceGroup[0].identifier_string, "BSiCS-DRV-2A  LS");
    res  = DeviceGroup[0].configureDefaults_0();
    printf("CONF0 I2C2 :\r\n");
    DeviceGroup[1].status_display_rotated180 = true;
    strcpy(DeviceGroup[1].identifier_string, "BSiCS-DRV-2A  HS");
    res &= DeviceGroup[1].configureDefaults_0();
    printf("\n");
    return res;
}

bool Devices_init_1( ) {
    bool res;
    printf("INIT1 I2C1 :\r\n");
    res  = DeviceGroup[0].init_1();
    printf("INIT1 I2C2 :\r\n");
    res &= DeviceGroup[1].init_1();
    return res;
};

bool Devices_configure_defaults_1() {
    bool res;
    // TODO: read and apply presets from EEPROM if available (identifier strings, calibration coefficients)

    // set-up peripherals with default configuration
    res  = DeviceGroup[0].configureDefaults_1();
    res &= DeviceGroup[1].configureDefaults_1();
    printf("\r\n");
    return res;
};

bool Devices_refresh(bool read_slow_conversion_results) {
    bool res;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    res  = DeviceGroup[0].writeChanges();
    res &= DeviceGroup[1].writeChanges();
    if(read_slow_conversion_results) {
        res &= DeviceGroup[0].readConversionResults();
        res &= DeviceGroup[1].readConversionResults();
    }
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    return res;
}

void Devices_increment_display_page_index(){
    for(int i = 0; i < DeviceGroupCount; i++){
        uint8_t idx = DeviceGroup[i].display_page_index;
        idx = (idx + 1) % PeripheralDeviceGroup::DisplayPageCount;
        DeviceGroup[i].display_page_index = idx;
    }
}

GPIO_packed_bits_t getDigitalOutputs(){
    GPIO_packed_bits_t tmp = 0;
    for(int i = GPIO_map_size-1; i >= 0 ; i--) {
        tmp = tmp << 1;
        tmp |= HAL_GPIO_ReadPin( GPIO_map[i].port, GPIO_map[i].pin );
    }
    return tmp;
}

void updateDigitalOutputs(GPIO_packed_bits_t mask, GPIO_packed_bits_t bits) {
    for(int i = 0; i < GPIO_map_size; i++){
        if(mask & 1){
            HAL_GPIO_WritePin(GPIO_map[i].port, GPIO_map[i].pin, (GPIO_PinState)(bits & 1));
        }
       bits = bits >> 1;
       mask = mask >> 1;
    }
}

bool getDigitalOutput(uint8_t n){
    assert_param(n < GPIO_map_size);
    if(n >= GPIO_map_size){
        return false;
    }
    return GPIO_PIN_SET == HAL_GPIO_ReadPin( GPIO_map[n].port, GPIO_map[n].pin );
}

void setDigitalOutput(uint8_t n, bool value){
    assert_param(n < GPIO_map_size);
    if(n >= GPIO_map_size){
        return;
    }
    HAL_GPIO_WritePin( GPIO_map[n].port, GPIO_map[n].pin, value? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void Devices_full_init(){
    // stage 0
    Devices_init_0();
    Devices_configure_defaults_0();
    // stage 1
    Devices_init_1();
    Devices_configure_defaults_1();
}