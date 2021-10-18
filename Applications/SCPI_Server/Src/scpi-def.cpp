#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"
#include "scpi/scpi.h"
#include "scpi-def.h"
#include "devices.h"

typedef struct {
    uint32_t start_ticks;
    uint32_t end_ticks;
} tick_interval_t;

volatile tick_interval_t busy_state_data {0, 0};

void SpecifyOperationFinishedIn(uint32_t from_now_ms) {
    tick_interval_t tmp;
    tmp.start_ticks = HAL_GetTick();
    uint32_t interval_duration_ticks = from_now_ms / HAL_GetTickFreq();
    if(interval_duration_ticks < 1){
        interval_duration_ticks = 1;
    }
    tmp.end_ticks = (uint32_t)(tmp.start_ticks + interval_duration_ticks);

    bool mixed_wrap_around =
            (busy_state_data.end_ticks < busy_state_data.start_ticks) ^
            (tmp.end_ticks < tmp.start_ticks);

    // bool update = false;
    // // Distinguish 4 possible cases:
    // if (mixed_wrap_around) {
    //     // 2 cases: one of two intervals wraps around, update only if tmp has wrap-around.
    //     update = tmp.end_ticks < busy_state_data.end_ticks;
    // } else {
    //     // 2 cases: default case, no wrap-around in either, or both ends are wrapped-around already.
    //     update = tmp.end_ticks > busy_state_data.end_ticks;
    // }
    bool update = (tmp.end_ticks > busy_state_data.end_ticks) ^ mixed_wrap_around;

    if(update){
        busy_state_data.start_ticks = tmp.start_ticks;
        busy_state_data.end_ticks   = tmp.end_ticks;
    }
}

static scpi_result_t My_CoreWai(scpi_t * context) {
    uint32_t timestamp =  HAL_GetTick();
    bool wrap_around = busy_state_data.end_ticks < busy_state_data.start_ticks ;
    bool gte_start = timestamp >= busy_state_data.start_ticks;
    bool lt_end    = timestamp <  busy_state_data.end_ticks;
    // Device is busy only when timestamp is within interval. When wrap_around is true, test for
    // "not inside the interval with swapped bounds" :
    bool busy = ((gte_start ^ wrap_around) && (lt_end ^ wrap_around)) ^ wrap_around ;
    if(busy){
        uint32_t delay_ms;
        if ( wrap_around && gte_start ) {
            // Wrap-around and timestamp is in the high part of the split interval. Piece together total remaining ms.
            delay_ms = (busy_state_data.end_ticks + (INT32_MAX - timestamp)) * HAL_GetTickFreq();
        } else {
            // No wrap-around or timestamp is in the low part of the split interval.
            delay_ms = (busy_state_data.end_ticks - timestamp) * HAL_GetTickFreq();
        }
        osDelay( delay_ms );
        // Finally, invalidate interval to no delay interval is entered twice.
        busy_state_data.start_ticks = busy_state_data.end_ticks;

    }
    return SCPI_RES_OK;
}


/* Chained commands can produce multiple returned values, separated with ';'.
 * To make responses identifiable, they ought to be formatted as key-value pairs.
 * Here, the SCPI command itself is chosen as a key string to identify the value returned.
 * When BSICS_PrependCommandToResult == true, the reply e.g.to  is
 *     cmd1?;cmd2?
 * can be
 *     "cmd1?",0;"cmd2?",0
 */
bool do_prepend_command_to_result = true;

size_t SCPI_ResultCommand(scpi_t * context){
    // detected pattern with unresolved argument placeholders (escaped):
    // return SCPI_ResultText(context, context->param_list.cmd->pattern);

    // detected pattern with unresolved argument placeholders (non-escaped):
    // return SCPI_ResultMnemonic(context, context->param_list.cmd->pattern);

    // received command with arguments and abbreviations :
    size_t trim = 0;
    if(*(char*)(context->param_list.cmd_raw.data + context->param_list.cmd_raw.length -1) == '?') {
        trim = 1;
    }
    return SCPI_ResultCharacters( context, context->param_list.cmd_raw.data + context->param_list.cmd_raw.position, context->param_list.cmd_raw.length - trim);
}

size_t BSICS_PrependCommandToResult(scpi_t * context){
    if( do_prepend_command_to_result){
        return SCPI_ResultCommand(context);
    } else {
        return 0;
    }
}

static scpi_result_t BSICS_SetPrependCommandToResponse(scpi_t * context) {
    scpi_bool_t param0;
    if (!SCPI_ParamBool(context, &param0, TRUE)) {
        return SCPI_RES_ERR;
    }
     do_prepend_command_to_result = param0;
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_PrependCommandToResponseQ(scpi_t * context) {
    BSICS_PrependCommandToResult(context);
    SCPI_ResultBool(context,  do_prepend_command_to_result);
    return SCPI_RES_OK;
}


/**
 * Reimplement IEEE488.2 *TST?
 *
 * Result should be 0 if everything is ok
 * Result should be 1 if something goes wrong
 *
 * Return SCPI_RES_OK
 */
static scpi_result_t My_CoreTstQ(scpi_t * context) {
    SCPI_ResultInt32(context, 0);
    return SCPI_RES_OK;
}

/* set DeviceGroupIndex
 * param: integer from 0 to DeviceGroupCount - 1 */
static scpi_result_t BSICS_SelectGroup(scpi_t * context) {
    scpi_number_t param0;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param0, TRUE)
        || (param0.unit != SCPI_UNIT_NONE)
        || (param0.content.value < 0) || (param0.content.value >= DeviceGroupCount) )
    { return SCPI_RES_ERR; }

    DeviceGroupIndex = (uint8_t)param0.content.value;
    return SCPI_RES_OK;
}

// read back DeviceGroupIndex
static scpi_result_t BSICS_GroupQ(scpi_t * context) {
    BSICS_PrependCommandToResult(context);
    SCPI_ResultInt32(context, DeviceGroupIndex);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SetPeriodicMeasReporting(scpi_t * context) {
    scpi_bool_t param0;
    if (!SCPI_ParamBool(context, &param0, TRUE)) {
        return SCPI_RES_ERR;
    }
    PeriodMeasReportingViaUART = param0;
    return SCPI_RES_OK;
}

// read back DeviceGroupIndex
static scpi_result_t BSICS_PeriodicMeasReportingQ(scpi_t * context) {
    BSICS_PrependCommandToResult(context);
    SCPI_ResultBool(context, PeriodMeasReportingViaUART);
    return SCPI_RES_OK;
}

enum BSICS_SetValue_dest {
    BSICS_group_voltage_lo,
    BSICS_group_voltage_hi,
    BSICS_group_current_lo,
    BSICS_group_current_hi,
};

/* BSICS_SetFloatingPointValue : [GRP#]:SOURce:(VOLTage, CURrent):(LO, HI) <double value> [<unit>]
 * param0 : int, optional
 * param1 : double
 */
static scpi_result_t BSICS_SetFloatingPointValue(scpi_t * context, BSICS_SetValue_dest dest, scpi_unit_t optional_unit) {
    // parse optional argument0 (group index GRP0, GRP1)
    int32_t idx = DeviceGroupIndex, new_idx;
    if(SCPI_CommandNumbers(context, &new_idx, 1, -1)){
        if( (new_idx >= 0) && (new_idx < DeviceGroupCount)){
            idx = new_idx;
            // Optionally set group index to last used value here.
            // DeviceGroupIndex = idx;
        }
    }

    // read param0 (new setpoint, float)
    scpi_number_t param0;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param0, TRUE)
        || (not ((param0.unit == SCPI_UNIT_NONE) || (param0.unit == optional_unit)))
        || (param0.content.value > 65.535) || (param0.content.value < 0) ) {
        return SCPI_RES_ERR;
    }
    uint16_t val_x1000 = (uint16_t)(param0.content.value * 1000 + 0.5);

    switch(dest){
        case BSICS_group_voltage_lo: DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_lo].VOUT_mV = val_x1000; break;
        case BSICS_group_voltage_hi: DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_hi].VOUT_mV = val_x1000; break;
        case BSICS_group_current_lo: DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_lo].IOUT_mA = val_x1000; break;
        case BSICS_group_current_hi: DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_hi].IOUT_mA = val_x1000; break;
        default:; // handle unknown dest
    };

    return SCPI_RES_OK;
}

// set DCDC_lo output voltage
static scpi_result_t BSICS_SetVoltageLo(scpi_t * context) {
    SpecifyOperationFinishedIn(500); // specify settling time estimate
    return BSICS_SetFloatingPointValue(context, BSICS_group_voltage_lo, SCPI_UNIT_VOLT);
}

// set DCDC_hi output voltage
static scpi_result_t BSICS_SetVoltageHi(scpi_t * context) {
    SpecifyOperationFinishedIn(500); // specify settling time estimate
    return BSICS_SetFloatingPointValue(context, BSICS_group_voltage_hi, SCPI_UNIT_VOLT);
}

// set DCDC_lo current limit
static scpi_result_t BSICS_SetCurrentLo(scpi_t * context) {
    return BSICS_SetFloatingPointValue(context, BSICS_group_current_lo, SCPI_UNIT_AMPER);
}

// set DCDC_hi current limit
static scpi_result_t BSICS_SetCurrentHi(scpi_t * context) {
    return BSICS_SetFloatingPointValue(context, BSICS_group_current_hi, SCPI_UNIT_AMPER);
}

// Return last DRV2A-CHn negative supply voltage (scaled ADC.ch0).
// Previously read value will be returned for simplicity.
static scpi_result_t BSICS_ChannelVoltageLoQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultFloat(context, (float)(DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].device_voltages_mV[0] / 1000.0) );
    return SCPI_RES_OK;
}

/* Return last DRV2A-CHn positive supply voltage (scaled ADC.ch1 - scaled ADC.ch0).
 * Previously read values need to be used as MCP342x channels have
 * to be converted sequentially and read back asynchonously. */
static scpi_result_t BSICS_ChannelVoltageHiQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultFloat(context, (float)(DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].device_voltages_mV[1] / 1000.0) );
    return SCPI_RES_OK;
}

// Return most recent CHn board temperature recorded near driver ICs.
static scpi_result_t BSICS_ChannelTemperatureQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultFloat(context, (float)(DeviceGroup[commandNumber[0]].temp_sensor_data[commandNumber[1]-1].T_mdegC / 1000.0) );
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SetDigitalOut(scpi_t * context) {
    uint32_t param0;
    if (!SCPI_ParamUInt32(context, &param0, TRUE)) {
        return SCPI_RES_ERR;
    }
    uint32_t mask = (1 << GPIO_map_size) - 1; // select all mapped bits
    updateDigitalOutputs(mask , param0);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_DigitalOutQ(scpi_t * context) {
    BSICS_PrependCommandToResult(context);
    SCPI_ResultUInt32(context, getDigitalOutputs());
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SetDigitalBit(scpi_t * context) {
    int32_t commandNumber[1];
    if( not SCPI_CommandNumbers(context, commandNumber, 1, -1)
        || (commandNumber[0] < 0)
        || (commandNumber[0] >= GPIO_map_size) )
    { return SCPI_RES_ERR; }
    GPIO_packed_bits_t tmp = 1 << commandNumber[0];

    scpi_bool_t value;
    if( not SCPI_ParamBool(context, &value, true) )
    { return SCPI_RES_ERR; }

    updateDigitalOutputs(tmp, value ? tmp : 0);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_DigitalBitQ(scpi_t * context) {
    int32_t commandNumber[1];
    if( not SCPI_CommandNumbers(context, commandNumber, 1, -1)
        || (commandNumber[0] < 0)
        || (commandNumber[0] >= GPIO_map_size) )
    { return SCPI_RES_ERR; }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultBool( context, getDigitalOutput(commandNumber[0]) );
    return SCPI_RES_OK;
}

// Configure CHn mux configuration
static scpi_result_t BSICS_SetChannelDriverMux(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    int32_t param0;
    if (!SCPI_ParamInt32(context, &param0, TRUE)
        || (param0 < 0) || (param0 > 0xFF) )
    { return SCPI_RES_ERR; }

    DeviceGroup[commandNumber[0]].octal_spst_data[commandNumber[1]-1].value = (uint8_t) param0;
    return SCPI_RES_OK;
}

// read back CHn mux configuration
static scpi_result_t BSICS_ChannelDriverMuxQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultUInt8( context, (uint8_t)(DeviceGroup[commandNumber[0]].octal_spst_data[commandNumber[1]-1].prev) );
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SetCalibration(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    size_t o_count;
    int32_t data[4];
    if(not SCPI_ParamArrayInt32(context, data, 4, &o_count, SCPI_FORMAT_ASCII, true)
        || (o_count != 4)
        || (data[0] < 0) || (data[1] < 0) || (data[2] < 0) || (data[3] < 0)
        || (data[0] > INT16_MAX) || (data[1] > INT16_MAX) || (data[2] > INT16_MAX) || (data[3] > INT16_MAX) )
    { return SCPI_RES_ERR; }

    DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x1024[0] = (int16_t)data[0];
    DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x4000[0] = (int16_t)data[1];
    DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x1024[1] = (int16_t)data[2];
    DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x4000[1] = (int16_t)data[3];

    return SCPI_RES_OK;
}

static scpi_result_t BSICS_CalibrationQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    uint16_t data[4];
    data[0] = DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x1024[0];
    data[1] = DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x4000[0];
    data[2] = DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x1024[1];
    data[3] = DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x4000[1];

    BSICS_PrependCommandToResult(context);
    SCPI_ResultArrayUInt16(context, data, 4, SCPI_FORMAT_ASCII);
    return SCPI_RES_OK;
}

// return most recent shared ~DCDC_ALT
static scpi_result_t BSICS_StatusDCDCsQ(scpi_t * context) {
    int32_t commandNumber[1];
    if( not SCPI_CommandNumbers(context, commandNumber, 1, -1)
        || (commandNumber[0] >= DeviceGroupCount) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultBool(context,not ( DeviceGroup[commandNumber[0]].gpio_exp_data.gpi & PeripheralDeviceGroup::GPIO_EXP_2_DC_ALTN) );
    return SCPI_RES_OK;
}

// return most recent group combined drivers RDY flag
static scpi_result_t BSICS_StatusDriversReadyQ(scpi_t * context) {
    int32_t commandNumber[1];
    if( not SCPI_CommandNumbers(context, commandNumber, 1, -1)
        || (commandNumber[0] >= DeviceGroupCount) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultBool(context, DeviceGroup[commandNumber[0]].gpio_exp_data.gpi & PeripheralDeviceGroup::GPIO_EXP_3_RDY);
    return SCPI_RES_OK;
}

// set text shown on OLED add-on to identify group associated with board / I2C channel
static scpi_result_t BSICS_SetDisplayText(scpi_t * context) {
    return SCPI_RES_OK;
}

// read back display text (shadowed but optionally read from EEPROM when recalled)
static scpi_result_t BSICS_DisplayTextQ(scpi_t * context) {
    return SCPI_RES_OK;
}

// store group display text and ADC conversion coefficients
static scpi_result_t BSICS_StoreCalibration(scpi_t * context) {
    return SCPI_RES_OK;
}

// recall group display text and ADC conversion coefficients
static scpi_result_t BSICS_RecallCalibration(scpi_t * context) {
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_HelpQ(scpi_t * context) {
    int i = 0;
    for(;;){
        SCPI_ResultCharacters(context, "\r\n", 2);
        SCPI_ResultMnemonic(context, scpi_commands[i].pattern);
        if(scpi_commands[++i].pattern == NULL){
            break;
        }
    }
    return SCPI_RES_OK;
}


const scpi_command_t scpi_commands[] = {
    {.pattern = "HELP", .callback = BSICS_HelpQ,},
    {.pattern = "HELP?", .callback = BSICS_HelpQ,},

    /* IEEE Mandated Commands (SCPI std V1999.0 4.1.1) */

    { .pattern = "*CLS", .callback = SCPI_CoreCls,},
    { .pattern = "*ESE", .callback = SCPI_CoreEse,},
    { .pattern = "*ESE?", .callback = SCPI_CoreEseQ,},
    { .pattern = "*ESR?", .callback = SCPI_CoreEsrQ,},
    { .pattern = "*IDN?", .callback = SCPI_CoreIdnQ,},
    { .pattern = "*OPC", .callback = SCPI_CoreOpc,},
    { .pattern = "*OPC?", .callback = SCPI_CoreOpcQ,},
    { .pattern = "*RST", .callback = SCPI_CoreRst,},
    { .pattern = "*SRE", .callback = SCPI_CoreSre,},
    { .pattern = "*SRE?", .callback = SCPI_CoreSreQ,},
    { .pattern = "*STB?", .callback = SCPI_CoreStbQ,},
    { .pattern = "*TST?", .callback = My_CoreTstQ,},
    { .pattern = "*WAI", .callback = My_CoreWai,},

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */

    {.pattern = "SYSTem:ERRor[:NEXT]?", .callback = SCPI_SystemErrorNextQ,},
    {.pattern = "SYSTem:ERRor:COUNt?", .callback = SCPI_SystemErrorCountQ,},
    {.pattern = "SYSTem:VERSion?", .callback = SCPI_SystemVersionQ,},
    {.pattern = "STATus:QUEStionable[:EVENt]?", .callback = SCPI_StatusQuestionableEventQ,},
    {.pattern = "STATus:QUEStionable:ENABle", .callback = SCPI_StatusQuestionableEnable,},
    {.pattern = "STATus:QUEStionable:ENABle?", .callback = SCPI_StatusQuestionableEnableQ,},
    {.pattern = "STATus:PRESet", .callback = SCPI_StatusPreset,},

    /* BSICS-SVR commands */

    {.pattern = "COMMunication:PREPend[:ENAble]", .callback = BSICS_SetPrependCommandToResponse,},
    {.pattern = "COMMunication:PREPend[:ENAble]?", .callback = BSICS_PrependCommandToResponseQ,},
    {.pattern = "COMMunication:REPorting[:ENAble]", .callback = BSICS_SetPeriodicMeasReporting,},
    {.pattern = "COMMunication:REPorting[:ENAble]?", .callback = BSICS_PeriodicMeasReportingQ,},

    {.pattern = "GPIO:OUTput[:SET]", .callback = BSICS_SetDigitalOut,},
    {.pattern = "GPIO:OUTput?", .callback = BSICS_DigitalOutQ,},
    {.pattern = "GPIO:BIT#[:SET]", .callback = BSICS_SetDigitalBit,},
    {.pattern = "GPIO:BIT#?", .callback = BSICS_DigitalBitQ,},

    {.pattern = "GRP[:SELect]", .callback = BSICS_SelectGroup,},
    {.pattern = "GRP?", .callback = BSICS_GroupQ,},

    {.pattern = "[GRP#]:SOURce:VOLTage:LO", .callback = BSICS_SetVoltageLo,},
    {.pattern = "[GRP#]:SOURce:VOLTage:HI", .callback = BSICS_SetVoltageHi,},
    {.pattern = "[GRP#]:SOURce:CURRent:LO", .callback = BSICS_SetCurrentLo,},
    {.pattern = "[GRP#]:SOURce:CURRent:HI", .callback = BSICS_SetCurrentHi,},

    {.pattern = "[GRP#]:MEASure:CH#:LO?", .callback = BSICS_ChannelVoltageLoQ,},
    {.pattern = "[GRP#]:MEASure:CH#:HI?", .callback = BSICS_ChannelVoltageHiQ,},
    {.pattern = "[GRP#]:MEASure:CH#:TEMP?", .callback = BSICS_ChannelTemperatureQ,},

    {.pattern = "[GRP#]:CONFigure:CH#:DRIVer[:STATe]", .callback = BSICS_SetChannelDriverMux,},
    {.pattern = "[GRP#]:CONFigure:CH#:DRIVer[:STATe]?", .callback = BSICS_ChannelDriverMuxQ,},

    {.pattern = "[GRP#]:STATus:DCDC[:OPERating]?", .callback = BSICS_StatusDCDCsQ,},
    {.pattern = "[GRP#]:STATus:DRIVers[:RDY]?", .callback = BSICS_StatusDriversReadyQ,},

//    {.pattern = "[GRP#]:DISPlay:TEXT", .callback = BSICS_SetDisplayText,},
//    {.pattern = "[GRP#]:DISPlay:TEXT?", .callback = BSICS_DisplayTextQ,},

    {.pattern = "[GRP#]:CALibration:CH#[:SET]", .callback = BSICS_SetCalibration,},
    {.pattern = "[GRP#]:CALibration:CH#?", .callback = BSICS_CalibrationQ,},
//    {.pattern = "[GRP#]:CALibration:STOre", .callback = BSICS_StoreCalibration,},
//    {.pattern = "[GRP#]:CALibration:RECall", .callback = BSICS_RecallCalibration,},

    SCPI_CMD_LIST_END
};

scpi_interface_t scpi_interface = {
    .error = SCPI_Error,
    .write = SCPI_Write,
    .control = SCPI_Control,
    .flush = SCPI_Flush,
    .reset = SCPI_Reset,
};

char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

scpi_t scpi_context;
