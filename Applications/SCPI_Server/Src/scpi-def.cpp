#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os.h"
#include "scpi/scpi.h"
#include "scpi-def.h"
#include "devices.h"
#include "scpi_server.h"

extern const uint8_t IP_ADDRESS[4];
extern const uint8_t NETMASK_ADDRESS[4];
extern const uint8_t GATEWAY_ADDRESS[4];

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
 * To make responses automatically identifiable, they ought to be formatted as key-value pairs.
 * Here, the SCPI command itself is chosen as a key string to identify the value returned.
 * When BSICS_PrependCommandToResult == true, the reply e.g.to
 *     cmd1?;cmd2?
 * is
 *     cmd1,0;cmd2,0
 */
bool do_prepend_command_to_result = true;

size_t SCPI_ResultCommand(scpi_t * context) {
    // detected pattern with unresolved argument placeholders (escaped, including '?'):
    // return SCPI_ResultText(context, context->param_list.cmd->pattern);

    // detected pattern with unresolved argument placeholders (non-escaped, including '?'):
    // return SCPI_ResultMnemonic(context, context->param_list.cmd->pattern);

    // detected pattern with unresolved argument placeholders (non-escaped, excluding '?'):
    size_t trim_right = 0;
    if(*(char*)(context->param_list.cmd_raw.data + context->param_list.cmd_raw.length - 1) == '?') {
        trim_right = 1;
    }
    return SCPI_ResultCharacters (
            context,
            context->param_list.cmd_raw.data + context->param_list.cmd_raw.position,
            context->param_list.cmd_raw.length - trim_right );
}

size_t BSICS_PrependCommandToResult(scpi_t * context){
    return do_prepend_command_to_result ? SCPI_ResultCommand(context) : 0;
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

bool BSICS_GetGroupCommandNumbers(scpi_t * context, int32_t * numbers, size_t count){
    if( not SCPI_CommandNumbers(context, numbers, count, -1)
        || (numbers[0] >= DeviceGroupCount) )
    { return false; }

    if(numbers[0] < 0){
        numbers[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }
    return true;
}

template<typename T>
inline bool inRange(T min, T number, T max){
    return (number >= min) && (number <= max);
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

/**
 * *RST
 * @param context
 * @return
 */
scpi_result_t My_CoreRst(scpi_t * context) {
    _scpi_result_t result = SCPI_RES_OK;
    if (context && context->interface && context->interface->reset) {
        result = context->interface->reset(context);
    }

    request_reinitialization();
    return result;
}

/* set DeviceGroupIndex
 * param: integer from 0 to DeviceGroupCount - 1 */
static scpi_result_t BSICS_SelectGroup(scpi_t * context) {
    int32_t param0;
    if ( not SCPI_ParamInt32(context, &param0, TRUE) ||
         not inRange<int32_t>(0, param0, DeviceGroupCount) )
    { return SCPI_RES_ERR; }

    DeviceGroupIndex = (uint8_t)param0;
    return SCPI_RES_OK;
}

// read back DeviceGroupIndex
static scpi_result_t BSICS_GroupQ(scpi_t * context) {
    BSICS_PrependCommandToResult(context);
    SCPI_ResultInt32(context, DeviceGroupIndex);
    return SCPI_RES_OK;
}

static scpi_result_t  BSICS_ProbeI2CQ(scpi_t * context) {
    int32_t idx; // command number 0: group index
    if( not BSICS_GetGroupCommandNumbers(context, &idx, 1) )
    { return SCPI_RES_ERR; }

    int32_t param0; // coomand param 0: I2C address
    if ( not SCPI_ParamInt32(context, &param0, TRUE) ||
         not inRange<int32_t>(1, param0, 0x7F) ) {
        return SCPI_RES_ERR;
    }

    bool result = false;
    // todo: test if I2C slave device responds, update result
    BSICS_PrependCommandToResult(context);
    SCPI_ResultBool(context, result);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SetPeriodicMeasReporting(scpi_t * context) {
    scpi_bool_t param0;
    if ( not SCPI_ParamBool(context, &param0, TRUE) )
    { return SCPI_RES_ERR; }

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

/* BSICS_SetFloatingPointValue : GRP#:SOURce:(VOLTage, CURrent):(LO, HI) <double value> [<unit>]
 * param0 : int, optional
 * param1 : double
 */
static scpi_result_t BSICS_SetFloatingPointValue(scpi_t * context, BSICS_SetValue_dest dest, scpi_unit_t optional_unit) {
    int32_t idx;
    if( not BSICS_GetGroupCommandNumbers(context, &idx, 1) )
    { return SCPI_RES_ERR; }

    // read param0 (new setpoint, float)
    scpi_number_t param0;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param0, TRUE)
        || not ((param0.unit == SCPI_UNIT_NONE) || (param0.unit == optional_unit))
        || not inRange<double>(0, param0.content.value, 65.535 ) ) {
        return SCPI_RES_ERR;
    }
    uint16_t val_x1000 = (uint16_t)(param0.content.value * 1000 + 0.5);

    switch(dest){
        case BSICS_group_voltage_lo: DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_lo].VOUT_mV = val_x1000; break;
        case BSICS_group_voltage_hi: DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_hi].VOUT_mV = val_x1000; break;
        case BSICS_group_current_lo: DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_lo].IOUT_mA = val_x1000; break;
        case BSICS_group_current_hi: DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_hi].IOUT_mA = val_x1000; break;
        default: return SCPI_RES_ERR; // handle unknown dest
    };

    return SCPI_RES_OK;
}

static scpi_result_t BSICS_GetFloatingPointValue(scpi_t * context, BSICS_SetValue_dest dest, scpi_unit_t unit) {
    int32_t idx;
    if (not BSICS_GetGroupCommandNumbers(context, &idx, 1)) { return SCPI_RES_ERR; }

    uint16_t val_x1000 = 0;
    switch(dest){
        case BSICS_group_voltage_lo: val_x1000 = DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_lo].VOUT_mV; break;
        case BSICS_group_voltage_hi: val_x1000 = DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_hi].VOUT_mV; break;
        case BSICS_group_current_lo: val_x1000 = DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_lo].IOUT_mA; break;
        case BSICS_group_current_hi: val_x1000 = DeviceGroup[idx].dcdc_data[PeripheralDeviceGroup::dcdc_hi].IOUT_mA; break;
        default: return SCPI_RES_ERR; // handle unknown dest
    };

    BSICS_PrependCommandToResult(context);
    SCPI_ResultFloat(context, (float)(val_x1000/1000.0));
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

static scpi_result_t BSICS_VoltageLoQ(scpi_t * context) {
    return BSICS_GetFloatingPointValue(context, BSICS_group_voltage_lo, SCPI_UNIT_VOLT);
}

static scpi_result_t BSICS_VoltageHiQ(scpi_t * context) {
    return BSICS_GetFloatingPointValue(context, BSICS_group_voltage_hi, SCPI_UNIT_VOLT);
}

static scpi_result_t BSICS_CurrentLoQ(scpi_t * context) {
    return BSICS_GetFloatingPointValue(context, BSICS_group_current_lo, SCPI_UNIT_AMPER);
}

static scpi_result_t BSICS_CurrentHiQ(scpi_t * context) {
    return BSICS_GetFloatingPointValue(context, BSICS_group_current_hi, SCPI_UNIT_AMPER);
}

// Return last DRV2A-CHn negative supply voltage (scaled ADC.ch0).
// Previously read value will be returned for simplicity.
static scpi_result_t BSICS_ChannelVoltageLoQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 2) ||
        not inRange<int32_t>(1, commandNumber[1], PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultFloat(context, (float)(DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].device_voltages_mV[0] / 1000.0) );
    return SCPI_RES_OK;
}

/* Return last DRV2A-CHn positive supply voltage (scaled ADC.ch1 - scaled ADC.ch0).
 * Previously read values need to be used as MCP342x channels have
 * to be converted sequentially and read back asynchonously. */
static scpi_result_t BSICS_ChannelVoltageHiQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 2) ||
        not inRange<int32_t>(1, commandNumber[1], PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultFloat(context, (float)(DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].device_voltages_mV[1] / 1000.0) );
    return SCPI_RES_OK;
}

// Return most recent CHn board temperature recorded near driver ICs.
static scpi_result_t BSICS_ChannelTemperatureQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 2) ||
        not inRange<int32_t>(1, commandNumber[1], PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultFloat(context, (float)(DeviceGroup[commandNumber[0]].temp_sensor_data[commandNumber[1]-1].T_mdegC / 1000.0) );
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SetDigitalOut(scpi_t * context) {
    uint32_t param0;
    if ( not SCPI_ParamUInt32(context, &param0, TRUE) )
    { return SCPI_RES_ERR; }

    uint32_t mask = (1 << GPIO_map_size) - 1; // select all mapped bits
    updateDigitalOutputs(mask , param0);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_DigitalOutQ(scpi_t * context) {
    BSICS_PrependCommandToResult(context);
    SCPI_ResultUInt32Base(context, getDigitalOutputs(), 16);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SetDigitalBit(scpi_t * context) {
    int32_t commandNumber[1];
    if( not SCPI_CommandNumbers(context, commandNumber, 1, -1) ||
        not inRange<int32_t>(0, commandNumber[0], GPIO_map_size-1) )
    { return SCPI_RES_ERR; }

    scpi_bool_t value;
    if( not SCPI_ParamBool(context, &value, true) )
    { return SCPI_RES_ERR; }

    setDigitalOutput(commandNumber[0], value);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_DigitalBitQ(scpi_t * context) {
    int32_t commandNumber[1];
    if( not SCPI_CommandNumbers(context, commandNumber, 1, -1) ||
        not inRange<int32_t>(0, commandNumber[0], GPIO_map_size-1) )
    { return SCPI_RES_ERR; }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultBool( context, getDigitalOutput(commandNumber[0]) );
    return SCPI_RES_OK;
}

enum BSICS_XIO_dest {
    BSICS_group_xio_output,
    BSICS_group_xio_input,
    BSICS_group_xio_mode,
    BSICS_group_xio_direction,
};

static scpi_result_t BSICS_XIO_SetValue(scpi_t * context, BSICS_XIO_dest dest){
    int32_t commandNumber[3]; // 0: group index, 1:ext. IO expander I2C address, 2: port no.
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 3) ||
        not inRange<int32_t>(1, commandNumber[1], 0x7F) ||
        not inRange<int32_t>(0, commandNumber[2], 3) )
    { return SCPI_RES_ERR; }
    uint8_t idx = commandNumber[0]; // group index

    uint32_t param0; // param0: new register value
    if ( not SCPI_ParamUInt32(context, &param0, TRUE) ||
         not inRange<uint32_t>(0, param0, 0xFF) )
    { return SCPI_RES_ERR; }

    switch(dest){
        case BSICS_group_xio_output   :;break; // todo: write reg
        case BSICS_group_xio_mode     :;break; // todo: write reg
        case BSICS_group_xio_direction:;break; // todo: write reg
        case BSICS_group_xio_input: // write to input reg not supported, fall through
        default: return SCPI_RES_ERR;
    }
    return SCPI_RES_OK; // only return OK it I2C transfer successful, return error if NAK
}

static scpi_result_t BSICS_XIO_GetValue(scpi_t * context, BSICS_XIO_dest dest){
    int32_t commandNumber[3]; // 0: group index, 1:ext. IO expander I2C address, 2: port no.
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 3) ||
        not inRange<int32_t>(1, commandNumber[1], 0x7F) ||
        not inRange<int32_t>(0, commandNumber[2], 3) )
    { return SCPI_RES_ERR; }
    uint8_t idx = commandNumber[0]; // group index

    uint8_t result = 0;
    switch(dest){
        case BSICS_group_xio_output   :;break; // todo: read write reg
        case BSICS_group_xio_mode     :;break; // todo: read write reg
        case BSICS_group_xio_direction:;break; // todo: read write reg
        case BSICS_group_xio_input    :;break; // todo: read write reg
        default: return SCPI_RES_ERR;
    }
    BSICS_PrependCommandToResult(context);
    SCPI_ResultUInt32Base(context, result, 16);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SetXIODir(scpi_t * context) {
    return BSICS_XIO_SetValue(context, BSICS_XIO_dest::BSICS_group_xio_direction);
}

static scpi_result_t BSICS_XIODirQ(scpi_t * context) {
    return BSICS_XIO_GetValue(context, BSICS_XIO_dest::BSICS_group_xio_direction);
}

static scpi_result_t BSICS_SetXIOMode(scpi_t * context) {
    return BSICS_XIO_SetValue(context, BSICS_XIO_dest::BSICS_group_xio_mode);
}

static scpi_result_t BSICS_XIOModeQ(scpi_t * context) {
    return BSICS_XIO_GetValue(context, BSICS_XIO_dest::BSICS_group_xio_mode);
}

static scpi_result_t BSICS_SetXIOOutput(scpi_t * context) {
    return BSICS_XIO_SetValue(context, BSICS_XIO_dest::BSICS_group_xio_output);
}

static scpi_result_t BSICS_XIOOutputQ(scpi_t * context) {
    return BSICS_XIO_GetValue(context, BSICS_XIO_dest::BSICS_group_xio_output);
}

static scpi_result_t BSICS_XIOInputQ(scpi_t * context) {
    return BSICS_XIO_GetValue(context, BSICS_XIO_dest::BSICS_group_xio_input);
}

// Configure CHn mux configuration
static scpi_result_t BSICS_SetChannelDriverMux(scpi_t * context) {
    int32_t commandNumber[2];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 2) ||
        not inRange<int32_t>(1, commandNumber[1], PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    int32_t param0;
    if (!SCPI_ParamInt32(context, &param0, TRUE) ||
            not inRange<int32_t>(0, param0, 0xFF) )
    { return SCPI_RES_ERR; }

    DeviceGroup[commandNumber[0]].octal_spst_data[commandNumber[1]-1].value = (uint8_t) param0;
    return SCPI_RES_OK;
}
// read back CHn mux configuration
static scpi_result_t BSICS_ChannelDriverMuxQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 2) ||
        not inRange<int32_t>(1, commandNumber[1], PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultUInt32Base(context, DeviceGroup[commandNumber[0]].octal_spst_data[commandNumber[1]-1].prev, 16);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SetCalibration(scpi_t * context) {
    int32_t commandNumber[2];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 2) ||
        not inRange<int32_t>(1, commandNumber[1], PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    size_t o_count;
    int32_t data[4];
    if( not SCPI_ParamArrayInt32(context, data, 4, &o_count, SCPI_FORMAT_ASCII, true) ||
        not (o_count == 4) ||
        not inRange<int32_t>(0, data[0], INT16_MAX) ||
        not inRange<int32_t>(0, data[0], INT16_MAX) ||
        not inRange<int32_t>(0, data[0], INT16_MAX) ||
        not inRange<int32_t>(0, data[0], INT16_MAX) )
    { return SCPI_RES_ERR; }

    DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x1024[0] = (int16_t)data[0];
    DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x4000[0] = (int16_t)data[1];
    DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x1024[1] = (int16_t)data[2];
    DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x4000[1] = (int16_t)data[3];

    return SCPI_RES_OK;
}

static scpi_result_t BSICS_CalibrationQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 2) ||
        not inRange<int32_t>(1, commandNumber[1], PeripheralDeviceGroup::n_channels) )
    { return SCPI_RES_ERR; }

    int16_t data[4];
    data[0] = DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x1024[0];
    data[1] = DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x4000[0];
    data[2] = DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x1024[1];
    data[3] = DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].coef_x4000[1];

    BSICS_PrependCommandToResult(context);
    SCPI_ResultArrayInt16(context, data, 4, SCPI_FORMAT_ASCII);
    return SCPI_RES_OK;
}

// return most recent shared ~DCDC_ALT
static scpi_result_t BSICS_StatusDCDCsQ(scpi_t * context) {
    int32_t commandNumber[1];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 2) )
    { return SCPI_RES_ERR; }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultBool(context,not ( DeviceGroup[commandNumber[0]].gpio_exp_data.gpi & PeripheralDeviceGroup::GPIO_EXP_2_DC_ALTN) );
    return SCPI_RES_OK;
}

// return most recent group combined drivers RDY flag
static scpi_result_t BSICS_StatusDriversReadyQ(scpi_t * context) {
    int32_t commandNumber[1];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 1) )
    { return SCPI_RES_ERR; }

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
    int32_t commandNumber[1];
    if( not BSICS_GetGroupCommandNumbers(context, commandNumber, 1) )
    { return SCPI_RES_ERR; }

    BSICS_PrependCommandToResult(context);
    SCPI_ResultText(context, DeviceGroup[commandNumber[0]].identifier_string);
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



static scpi_result_t BSICS_IPv4(scpi_t * context) {
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_IPv4Q(scpi_t * context) {
    BSICS_PrependCommandToResult(context);
    SCPI_ResultArrayUInt8(context, IP_ADDRESS, 4, SCPI_FORMAT_ASCII);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SubnetMask(scpi_t * context) {
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_SubnetMaskQ(scpi_t * context) {
    BSICS_PrependCommandToResult(context);
    SCPI_ResultArrayUInt8(context, NETMASK_ADDRESS, 4, SCPI_FORMAT_ASCII);
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_Gateway(scpi_t * context) {
    return SCPI_RES_OK;
}

static scpi_result_t BSICS_GatewayQ(scpi_t * context) {
    BSICS_PrependCommandToResult(context);
    SCPI_ResultArrayUInt8(context, GATEWAY_ADDRESS, 4, SCPI_FORMAT_ASCII);
    return SCPI_RES_OK;
}


// Conditional initializer macros. Note order must remain: pattern, callback, [description,] [tag,].
#if USE_COMMAND_DESCRIPTIONS
#define SCPI_CMD_DESC(S) .description=(S),
#else
#define SCPI_CMD_DESC(S)
#endif

#if USE_COMMAND_TAGS
#define SCPI_CMD_TAG(T) .tag=(T),
#else
#define SCPI_CMD_TAG(T)
#endif

#define STR_INDIR(s) #s
#define STR_HELPER(s) STR_INDIR(s)

#define SCPI_CONTROL_PORT_STR STR_HELPER(SCPI_CONTROL_PORT)

const scpi_command_t scpi_commands[] = {
    /* Optional help commands */
    {.pattern = "HELP?", .callback = SCPI_HelpQ, SCPI_CMD_DESC("\t - list all supported commands")},
    {.pattern = "HELP", .callback = SCPI_HelpQ, SCPI_CMD_DESC("\t - HELP? alias")},
    
    /* IEEE Mandated Commands (SCPI std V1999.0 4.1.1) */

    { .pattern = "*CLS", .callback = SCPI_CoreCls, SCPI_CMD_DESC("\t - clear status byte, error queue, event reg")},
    { .pattern = "*ESE", .callback = SCPI_CoreEse,SCPI_CMD_DESC("\t - event status enable")},
    { .pattern = "*ESE?", .callback = SCPI_CoreEseQ,},
    { .pattern = "*ESR?", .callback = SCPI_CoreEsrQ,SCPI_CMD_DESC("\t - event status enable reg")},
    { .pattern = "*IDN?", .callback = SCPI_CoreIdnQ, SCPI_CMD_DESC("\t - return device identifier")},
    { .pattern = "*OPC", .callback = SCPI_CoreOpc,SCPI_CMD_DESC("\t - operation complete cmd")},
    { .pattern = "*OPC?", .callback = SCPI_CoreOpcQ,SCPI_CMD_DESC("\t - complete overlapped cmd (1)")},
    { .pattern = "*RST", .callback = My_CoreRst, SCPI_CMD_DESC("\t - reset interface and re-initialize")},
    { .pattern = "*SRE", .callback = SCPI_CoreSre,SCPI_CMD_DESC("\t - service request enable")},
    { .pattern = "*SRE?", .callback = SCPI_CoreSreQ,},
    { .pattern = "*STB?", .callback = SCPI_CoreStbQ,SCPI_CMD_DESC("\t - status byte query")},
    { .pattern = "*TST?", .callback = My_CoreTstQ, SCPI_CMD_DESC("\t - self-test (0:pass)")},
    { .pattern = "*WAI", .callback = My_CoreWai, SCPI_CMD_DESC("\t - halt cmd execution until pending operations complete")},

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */

    {.pattern = "STATus:QUEStionable[:EVENt]?", .callback = SCPI_StatusQuestionableEventQ,},
    {.pattern = "STATus:QUEStionable:ENABle", .callback = SCPI_StatusQuestionableEnable,},
    {.pattern = "STATus:QUEStionable:ENABle?", .callback = SCPI_StatusQuestionableEnableQ,},
    {.pattern = "STATus:PRESet", .callback = SCPI_StatusPreset,},
    {.pattern = "SYSTem:ERRor[:NEXT]?", .callback = SCPI_SystemErrorNextQ,},
    {.pattern = "SYSTem:ERRor:COUNt?", .callback = SCPI_SystemErrorCountQ,},
    {.pattern = "SYSTem:VERSion?", .callback = SCPI_SystemVersionQ,},

    {.pattern = "SYSTem:COMMunication:TCPIP:CONTROL?", .callback = SCPI_SystemCommTcpipControlQ,SCPI_CMD_DESC("(" SCPI_CONTROL_PORT_STR ") - control port")},
//    {.pattern = "SYSTem:COMMunication:TCPIP:ADDR", .callback = BSICS_IPv4, SCPI_CMD_DESC("<B>,<B>,<B>,<B> - set IP v4 address (restart to update)")},
    {.pattern = "SYSTem:COMMunication:TCPIP:ADDR?", .callback = BSICS_IPv4Q, SCPI_CMD_DESC(" - read stored IP v4 address")},
//    {.pattern = "SYSTem:COMMunication:TCPIP:MASK", .callback = BSICS_SubnetMask, SCPI_CMD_DESC("<B>,<B>,<B>,<B> - set subnet mask (restart to update)")},
    {.pattern = "SYSTem:COMMunication:TCPIP:MASK?", .callback = BSICS_SubnetMaskQ, SCPI_CMD_DESC(" - read stored subnet mask")},
//    {.pattern = "SYSTem:COMMunication:TCPIP:GATeway", .callback = BSICS_Gateway, SCPI_CMD_DESC("<B>,<B>,<B>,<B> - gateway (restart to update)")},
    {.pattern = "SYSTem:COMMunication:TCPIP:GATeway?", .callback = BSICS_GatewayQ, SCPI_CMD_DESC(" - read stored gateway address")},

    /* BSICS-SVR commands */

    {.pattern = "SYSTem:COMMunication:PREPend[:ENAble]", .callback = BSICS_SetPrependCommandToResponse, SCPI_CMD_DESC("<0:False:1:True> - prepend cmd to response")},
    {.pattern = "SYSTem:COMMunication:PREPend[:ENAble]?", .callback = BSICS_PrependCommandToResponseQ, SCPI_CMD_DESC("(0|1)")},
    {.pattern = "SYSTem:COMMunication:REPorting[:ENAble]", .callback = BSICS_SetPeriodicMeasReporting, SCPI_CMD_DESC("<0:False:1:True> - periodically print to UART")},
    {.pattern = "SYSTem:COMMunication:REPorting[:ENAble]?", .callback = BSICS_PeriodicMeasReportingQ, SCPI_CMD_DESC("(0|1)")},

    {.pattern = "GPIO:OUTput", .callback = BSICS_SetDigitalOut, SCPI_CMD_DESC("<#Hxxxx> - set board output pins D0 .. D15 (D11, D14, D15 : N/A)")},
    {.pattern = "GPIO:OUTput?", .callback = BSICS_DigitalOutQ, SCPI_CMD_DESC("(#Hxxxx) - D0 .. D15 pin read-back (D11, D14, D15 : N/A)")},
    {.pattern = "GPIO:BIT#", .callback = BSICS_SetDigitalBit, SCPI_CMD_DESC("<0|False|1|True>")},
    {.pattern = "GPIO:BIT#?", .callback = BSICS_DigitalBitQ, SCPI_CMD_DESC("(0|1)")},

    {.pattern = "GRP[:SELect]", .callback = BSICS_SelectGroup, SCPI_CMD_DESC("<0|1>")},
    {.pattern = "GRP?", .callback = BSICS_GroupQ, SCPI_CMD_DESC("(0|1) - current group index")},

    {.pattern = "GRP#:PROBe?", .callback = BSICS_ProbeI2CQ, SCPI_CMD_DESC("<#H01..#H7F> - test if I2C slave device is present (0|1)")},

    {.pattern = "GRP#:XIOaddr#:DIRection#", .callback = BSICS_SetXIODir, SCPI_CMD_DESC("<#Hxx> - set GRP<0,1>:XIO<dev addr>:DIR<port> direction reg")},
    {.pattern = "GRP#:XIOaddr#:DIRection#?", .callback = BSICS_XIODirQ, SCPI_CMD_DESC("(#Hxx) - read direction<port> reg")},
    {.pattern = "GRP#:XIOaddr#:MODE#", .callback = BSICS_SetXIOMode, SCPI_CMD_DESC("<#Hxx> - set GRP<0,1>:XIO<dev addr>:MODE<port> mode reg")},
    {.pattern = "GRP#:XIOaddr#:MODE#?", .callback = BSICS_XIOModeQ, SCPI_CMD_DESC("(#Hxx) - read mode<port> reg")},
    {.pattern = "GRP#:XIOaddr#:OUTput#", .callback = BSICS_SetXIOOutput, SCPI_CMD_DESC("<#Hxx> - set GRP<0,1>:XIO<dev addr>:OUT<port> output")},
    {.pattern = "GRP#:XIOaddr#:OUTput#?", .callback = BSICS_XIOOutputQ, SCPI_CMD_DESC("(#Hxx) - read output<port> reg")},
    {.pattern = "GRP#:XIOaddr#:INput#?", .callback = BSICS_XIOInputQ, SCPI_CMD_DESC("(#Hxx) - read input<port> reg")},

    {.pattern = "GRP#:SOURce:VOLTage:LO", .callback = BSICS_SetVoltageLo, SCPI_CMD_DESC("<float> - [V] LO DCDC setpoint")},
    {.pattern = "GRP#:SOURce:VOLTage:LO?", .callback = BSICS_VoltageLoQ, SCPI_CMD_DESC("(float) - [V] last LO setpoint")},
    {.pattern = "GRP#:SOURce:VOLTage:HI", .callback = BSICS_SetVoltageHi, SCPI_CMD_DESC("<float> - [V] HI DCDC setpoint")},
    {.pattern = "GRP#:SOURce:VOLTage:HI?", .callback = BSICS_VoltageHiQ, SCPI_CMD_DESC("(float) - [V] last HI setpoint")},
    {.pattern = "GRP#:SOURce:CURRent:LO", .callback = BSICS_SetCurrentLo, SCPI_CMD_DESC("<float> - [A] limit for LO DCDC")},
    {.pattern = "GRP#:SOURce:CURRent:LO?", .callback = BSICS_CurrentLoQ, SCPI_CMD_DESC("(float) - [A] last LO DCDC limit")},
    {.pattern = "GRP#:SOURce:CURRent:HI", .callback = BSICS_SetCurrentHi, SCPI_CMD_DESC("<float> - [A] limit for HI DCDC")},
    {.pattern = "GRP#:SOURce:CURRent:HI?", .callback = BSICS_CurrentHiQ, SCPI_CMD_DESC("(float) - [A] last HI DCDC limit")},

    {.pattern = "GRP#:MEASure:CH#:LO?", .callback = BSICS_ChannelVoltageLoQ, SCPI_CMD_DESC("<float> - [V] CH1..CH3 secondary-side ADC LO readback")},
    {.pattern = "GRP#:MEASure:CH#:HI?", .callback = BSICS_ChannelVoltageHiQ, SCPI_CMD_DESC("<float> - [V] CH1..CH3 secondary-side ADC HI readback")},
    {.pattern = "GRP#:MEASure:CH#:TEMP?", .callback = BSICS_ChannelTemperatureQ, SCPI_CMD_DESC("<float> - [°C] CH1..CH3 secondary-side T sensor")},

    {.pattern = "GRP#:CONFigure:CH#:DRIVer[:STATe]", .callback = BSICS_SetChannelDriverMux, SCPI_CMD_DESC("<#Hxx> - CH1..CH3 HS 7:4 and LS 3:0 switch config")},
    {.pattern = "GRP#:CONFigure:CH#:DRIVer[:STATe]?", .callback = BSICS_ChannelDriverMuxQ, SCPI_CMD_DESC("(#Hxx)")},

    {.pattern = "GRP#:STATus:DCDC[:OPERating]?", .callback = BSICS_StatusDCDCsQ, SCPI_CMD_DESC("(0|1) - combined DCDC PG/~ALERT")},
    {.pattern = "GRP#:STATus:DRIVers[:RDY]?", .callback = BSICS_StatusDriversReadyQ, SCPI_CMD_DESC("(0|1) - combined RDY state")},

//    {.pattern = "GRP#:DISPlay:TEXT", .callback = BSICS_SetDisplayText,},
    {.pattern = "GRP#:DISPlay:TEXT?", .callback = BSICS_DisplayTextQ,},

    {.pattern = "GRP#:CALibration:CH#[:SET]", .callback = BSICS_SetCalibration,},
    {.pattern = "GRP#:CALibration:CH#?", .callback = BSICS_CalibrationQ,},
//    {.pattern = "GRP#:CALibration:STOre", .callback = BSICS_StoreCalibration,},
//    {.pattern = "GRP#:CALibration:RECall", .callback = BSICS_RecallCalibration,},

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
