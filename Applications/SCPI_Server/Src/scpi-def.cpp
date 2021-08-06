/*-
 * BSD 2-Clause License
 *
 * Copyright (c) 2012-2018, Jan Breuer
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   scpi-def.c
 * @date   Thu Nov 15 10:58:45 UTC 2012
 *
 * @brief  SCPI parser test
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "scpi/scpi.h"
#include "scpi-def.h"
#include "devices.h"


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
    SCPI_ResultInt32(context, DeviceGroupIndex);
    return SCPI_RES_OK;
}

// set DCDC_lo output voltage
static scpi_result_t BSICS_SetVoltageLo(scpi_t * context) {
    return SCPI_RES_OK;
}

// set DCDC_hi output voltage
static scpi_result_t BSICS_SetVoltageHi(scpi_t * context) {
    return SCPI_RES_OK;
}

// set DCDC_lo current limit
static scpi_result_t BSICS_SetCurrentLo(scpi_t * context) {
    return SCPI_RES_OK;
}

// set DCDC_hi current limit
static scpi_result_t BSICS_SetCurrentHi(scpi_t * context) {
    return SCPI_RES_OK;
}

// Return last DRV2A-CHn negative supply voltage (scaled ADC.ch0).
// Previously read value will be returned for simplicity.
static scpi_result_t BSICS_ChannelVoltageLoQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > DeviceGroupChannelCount) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

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
        || (commandNumber[1] > DeviceGroupChannelCount) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    SCPI_ResultFloat(context, (float)(DeviceGroup[commandNumber[0]].adc_data[commandNumber[1]-1].device_voltages_mV[1] / 1000.0) );
    return SCPI_RES_OK;
}

// Return most recent CHn board temperature recorded near driver ICs.
static scpi_result_t BSICS_ChannelTemperatureQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > DeviceGroupChannelCount) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    SCPI_ResultFloat(context, (float)(DeviceGroup[commandNumber[0]].temp_sensor_data[commandNumber[1]-1].T_mdegC / 1000.0) );
    return SCPI_RES_OK;
}

// Set/Clear group drivers enable (GPIO -> XRDVEN)
static scpi_result_t BSICS_SetDriversEna(scpi_t * context) {
    return SCPI_RES_OK;
}

// return group drivers enable status (XRDVEN)
static scpi_result_t BSICS_DriversEnaQ(scpi_t * context) {
    return SCPI_RES_OK;
}

// Configure CHn mux configuration
static scpi_result_t BSICS_SetChannelDriverMux(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > DeviceGroupChannelCount) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }

    scpi_number_t param0;
    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param0, TRUE)
        || (param0.unit != SCPI_UNIT_NONE)
        || (param0.content.value < 0) || (param0.content.value >= 0xFF) )
    { return SCPI_RES_ERR; }

    DeviceGroup[commandNumber[0]].octal_spst_data[commandNumber[1]-1].value = (uint8_t) param0.content.value;
    return SCPI_RES_OK;
}

// read back CHn mux configuration
static scpi_result_t BSICS_ChannelDriverMuxQ(scpi_t * context) {
    int32_t commandNumber[2];
    if( not SCPI_CommandNumbers(context, commandNumber, 2, -1)
        || (commandNumber[0] >= DeviceGroupCount)
        || (commandNumber[1] < 1)
        || (commandNumber[1] > DeviceGroupChannelCount) )
    { return SCPI_RES_ERR; }

    if(commandNumber[0] < 0){
        commandNumber[0] = DeviceGroupIndex;
    } else {
        // Un-comment to cause explicit GRPx:MEAS:... to change DeviceGroupIndex to x.
        // DeviceGroupIndex = commandNumber[0]; // also set DeviceGroupIndex for subsequent operations
    }
    SCPI_ResultUInt8( context, (uint8_t)(DeviceGroup[commandNumber[0]].octal_spst_data[commandNumber[1]-1].prev) );
    return SCPI_RES_OK;
}

// return most recent shared ~DCDC_ALT
static scpi_result_t BSICS_StatusDCDCsQ(scpi_t * context) {
    return SCPI_RES_OK;
}

// return most recent group combined drivers RDY flag
static scpi_result_t BSICS_StatusDriversReadyQ(scpi_t * context) {
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

// set CH_x ADC_ch_y conversion coefficient
static scpi_result_t BSICS_SetChannelADCCalibration(scpi_t * context) {
    return SCPI_RES_OK;
}

// return CH_x ADC_ch_y conversion coefficient (previously set or read from EEPROM when recalled)
static scpi_result_t BSICS_ChannelADCCalibrationQ(scpi_t * context) {
    return SCPI_RES_OK;
}





//static scpi_result_t DMM_MeasureVoltageDcQ(scpi_t * context) {
//    scpi_number_t param1, param2;
//    char bf[15];
//    fprintf(stderr, "meas:volt:dc\r\n"); /* debug command name */
//
//    /* read first parameter if present */
//    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param1, FALSE)) {
//        /* do something, if parameter not present */
//    }
//
//    /* read second paraeter if present */
//    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param2, FALSE)) {
//        /* do something, if parameter not present */
//    }
//
//
//    SCPI_NumberToStr(context, scpi_special_numbers_def, &param1, bf, 15);
//    fprintf(stderr, "\tP1=%s\r\n", bf);
//
//
//    SCPI_NumberToStr(context, scpi_special_numbers_def, &param2, bf, 15);
//    fprintf(stderr, "\tP2=%s\r\n", bf);
//
//    SCPI_ResultDouble(context, 0);
//
//    return SCPI_RES_OK;
//}

//static scpi_result_t DMM_MeasureVoltageAcQ(scpi_t * context) {
//    scpi_number_t param1, param2;
//    char bf[15];
//    fprintf(stderr, "meas:volt:ac\r\n"); /* debug command name */
//
//    /* read first parameter if present */
//    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param1, FALSE)) {
//        /* do something, if parameter not present */
//    }
//
//    /* read second paraeter if present */
//    if (!SCPI_ParamNumber(context, scpi_special_numbers_def, &param2, FALSE)) {
//        /* do something, if parameter not present */
//    }
//
//
//    SCPI_NumberToStr(context, scpi_special_numbers_def, &param1, bf, 15);
//    fprintf(stderr, "\tP1=%s\r\n", bf);
//
//
//    SCPI_NumberToStr(context, scpi_special_numbers_def, &param2, bf, 15);
//    fprintf(stderr, "\tP2=%s\r\n", bf);
//
//    SCPI_ResultDouble(context, 0);
//
//    return SCPI_RES_OK;
//}
//
//static scpi_result_t DMM_ConfigureVoltageDc(scpi_t * context) {
//    double param1, param2;
//    fprintf(stderr, "conf:volt:dc\r\n"); /* debug command name */
//
//    /* read first parameter if present */
//    if (!SCPI_ParamDouble(context, &param1, TRUE)) {
//        return SCPI_RES_ERR;
//    }
//
//    /* read second paraeter if present */
//    if (!SCPI_ParamDouble(context, &param2, FALSE)) {
//        /* do something, if parameter not present */
//    }
//
//    fprintf(stderr, "\tP1=%lf\r\n", param1);
//    fprintf(stderr, "\tP2=%lf\r\n", param2);
//
//    return SCPI_RES_OK;
//}

//static scpi_result_t TEST_Bool(scpi_t * context) {
//    scpi_bool_t param1;
//    fprintf(stderr, "TEST:BOOL\r\n"); /* debug command name */
//
//    /* read first parameter if present */
//    if (!SCPI_ParamBool(context, &param1, TRUE)) {
//        return SCPI_RES_ERR;
//    }
//
//    fprintf(stderr, "\tP1=%d\r\n", param1);
//
//    return SCPI_RES_OK;
//}
//
//scpi_choice_def_t trigger_source[] = {
//    {"BUS", 5},
//    {"IMMediate", 6},
//    {"EXTernal", 7},
//    SCPI_CHOICE_LIST_END /* termination of option list */
//};
//
//static scpi_result_t TEST_ChoiceQ(scpi_t * context) {
//
//    int32_t param;
//    const char * name;
//
//    if (!SCPI_ParamChoice(context, trigger_source, &param, TRUE)) {
//        return SCPI_RES_ERR;
//    }
//
//    SCPI_ChoiceToName(trigger_source, param, &name);
//    fprintf(stderr, "\tP1=%s (%ld)\r\n", name, (long int) param);
//
//    SCPI_ResultInt32(context, param);
//
//    return SCPI_RES_OK;
//}
//
// static scpi_result_t TEST_Numbers(scpi_t * context) {
//     int32_t numbers[2];
//     SCPI_CommandNumbers(context, numbers, 2, 1);
//
//     fprintf(stderr, "TEST numbers %d %d\r\n", numbers[0], numbers[1]);
//
//     return SCPI_RES_OK;
// }

//static scpi_result_t TEST_Text(scpi_t * context) {
//    char buffer[100];
//    size_t copy_len;
//
//    if (!SCPI_ParamCopyText(context, buffer, sizeof (buffer), &copy_len, FALSE)) {
//        buffer[0] = '\0';
//    }
//
//    fprintf(stderr, "TEXT: ***%s***\r\n", buffer);
//
//    return SCPI_RES_OK;
//}
//
//static scpi_result_t TEST_ArbQ(scpi_t * context) {
//    const char * data;
//    size_t len;
//
//    if (SCPI_ParamArbitraryBlock(context, &data, &len, FALSE)) {
//        SCPI_ResultArbitraryBlock(context, data, len);
//    }
//
//    return SCPI_RES_OK;
//}
//
//struct _scpi_channel_value_t {
//    int32_t row;
//    int32_t col;
//};
//typedef struct _scpi_channel_value_t scpi_channel_value_t;
//
///**
// * @brief
// * parses lists
// * channel numbers > 0.
// * no checks yet.
// * valid: (@1), (@3!1:1!3), ...
// * (@1!1:3!2) would be 1!1, 1!2, 2!1, 2!2, 3!1, 3!2.
// * (@3!1:1!3) would be 3!1, 3!2, 3!3, 2!1, 2!2, 2!3, ... 1!3.
// *
// * @param channel_list channel list, compare to SCPI99 Vol 1 Ch. 8.3.2
// */
//static scpi_result_t TEST_Chanlst(scpi_t *context) {
//    scpi_parameter_t channel_list_param;
//#define MAXROW 2    /* maximum number of rows */
//#define MAXCOL 6    /* maximum number of columns */
//#define MAXDIM 2    /* maximum number of dimensions */
//    scpi_channel_value_t array[MAXROW * MAXCOL]; /* array which holds values in order (2D) */
//    size_t chanlst_idx; /* index for channel list */
//    size_t arr_idx = 0; /* index for array */
//    size_t n, m = 1; /* counters for row (n) and columns (m) */
//
//    /* get channel list */
//    if (SCPI_Parameter(context, &channel_list_param, TRUE)) {
//        scpi_expr_result_t res;
//        scpi_bool_t is_range;
//        int32_t values_from[MAXDIM];
//        int32_t values_to[MAXDIM];
//        size_t dimensions;
//
//        bool for_stop_row = FALSE; /* true if iteration for rows has to stop */
//        bool for_stop_col = FALSE; /* true if iteration for columns has to stop */
//        int32_t dir_row = 1; /* direction of counter for rows, +/-1 */
//        int32_t dir_col = 1; /* direction of counter for columns, +/-1 */
//
//        /* the next statement is valid usage and it gets only real number of dimensions for the first item (index 0) */
//        if (!SCPI_ExprChannelListEntry(context, &channel_list_param, 0, &is_range, NULL, NULL, 0, &dimensions)) {
//            chanlst_idx = 0; /* call first index */
//            arr_idx = 0; /* set arr_idx to 0 */
//            do { /* if valid, iterate over channel_list_param index while res == valid (do-while cause we have to do it once) */
//                res = SCPI_ExprChannelListEntry(context, &channel_list_param, chanlst_idx, &is_range, values_from, values_to, 4, &dimensions);
//                if (is_range == FALSE) { /* still can have multiple dimensions */
//                    if (dimensions == 1) {
//                        /* here we have our values
//                         * row == values_from[0]
//                         * col == 0 (fixed number)
//                         * call a function or something */
//                        array[arr_idx].row = values_from[0];
//                        array[arr_idx].col = 0;
//                    } else if (dimensions == 2) {
//                        /* here we have our values
//                         * row == values_fom[0]
//                         * col == values_from[1]
//                         * call a function or something */
//                        array[arr_idx].row = values_from[0];
//                        array[arr_idx].col = values_from[1];
//                    } else {
//                        return SCPI_RES_ERR;
//                    }
//                    arr_idx++; /* inkrement array where we want to save our values to, not neccessary otherwise */
//                    if (arr_idx >= MAXROW * MAXCOL) {
//                        return SCPI_RES_ERR;
//                    }
//                } else if (is_range == TRUE) {
//                    if (values_from[0] > values_to[0]) {
//                        dir_row = -1; /* we have to decrement from values_from */
//                    } else { /* if (values_from[0] < values_to[0]) */
//                        dir_row = +1; /* default, we increment from values_from */
//                    }
//
//                    /* iterating over rows, do it once -> set for_stop_row = false
//                     * needed if there is channel list index isn't at end yet */
//                    for_stop_row = FALSE;
//                    for (n = values_from[0]; for_stop_row == FALSE; n += dir_row) {
//                        /* usual case for ranges, 2 dimensions */
//                        if (dimensions == 2) {
//                            if (values_from[1] > values_to[1]) {
//                                dir_col = -1;
//                            } else if (values_from[1] < values_to[1]) {
//                                dir_col = +1;
//                            }
//                            /* iterating over columns, do it at least once -> set for_stop_col = false
//                             * needed if there is channel list index isn't at end yet */
//                            for_stop_col = FALSE;
//                            for (m = values_from[1]; for_stop_col == FALSE; m += dir_col) {
//                                /* here we have our values
//                                 * row == n
//                                 * col == m
//                                 * call a function or something */
//                                array[arr_idx].row = n;
//                                array[arr_idx].col = m;
//                                arr_idx++;
//                                if (arr_idx >= MAXROW * MAXCOL) {
//                                    return SCPI_RES_ERR;
//                                }
//                                if (m == (size_t)values_to[1]) {
//                                    /* endpoint reached, stop column for-loop */
//                                    for_stop_col = TRUE;
//                                }
//                            }
//                            /* special case for range, example: (@2!1) */
//                        } else if (dimensions == 1) {
//                            /* here we have values
//                             * row == n
//                             * col == 0 (fixed number)
//                             * call function or sth. */
//                            array[arr_idx].row = n;
//                            array[arr_idx].col = 0;
//                            arr_idx++;
//                            if (arr_idx >= MAXROW * MAXCOL) {
//                                return SCPI_RES_ERR;
//                            }
//                        }
//                        if (n == (size_t)values_to[0]) {
//                            /* endpoint reached, stop row for-loop */
//                            for_stop_row = TRUE;
//                        }
//                    }
//
//
//                } else {
//                    return SCPI_RES_ERR;
//                }
//                /* increase index */
//                chanlst_idx++;
//            } while (SCPI_EXPR_OK == SCPI_ExprChannelListEntry(context, &channel_list_param, chanlst_idx, &is_range, values_from, values_to, 4, &dimensions));
//            /* while checks, whether incremented index is valid */
//        }
//        /* do something at the end if needed */
//        /* array[arr_idx].row = 0; */
//        /* array[arr_idx].col = 0; */
//    }
//
//    {
//        size_t i;
//        fprintf(stderr, "TEST_Chanlst: ");
//        for (i = 0; i< arr_idx; i++) {
//            fprintf(stderr, "%d!%d, ", array[i].row, array[i].col);
//        }
//        fprintf(stderr, "\r\n");
//    }
//    return SCPI_RES_OK;
//}



const scpi_command_t scpi_commands[] = {
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
    { .pattern = "*WAI", .callback = SCPI_CoreWai,},

    /* Required SCPI commands (SCPI std V1999.0 4.2.1) */
    {.pattern = "SYSTem:ERRor[:NEXT]?", .callback = SCPI_SystemErrorNextQ,},
    {.pattern = "SYSTem:ERRor:COUNt?", .callback = SCPI_SystemErrorCountQ,},
    {.pattern = "SYSTem:VERSion?", .callback = SCPI_SystemVersionQ,},
    // {.pattern = "STATus:OPERation?", .callback = scpi_stub_callback,},
    // {.pattern = "STATus:OPERation:EVENt?", .callback = scpi_stub_callback,},
    // {.pattern = "STATus:OPERation:CONDition?", .callback = scpi_stub_callback,},
    // {.pattern = "STATus:OPERation:ENABle", .callback = scpi_stub_callback,},
    // {.pattern = "STATus:OPERation:ENABle?", .callback = scpi_stub_callback,},
    {.pattern = "STATus:QUEStionable[:EVENt]?", .callback = SCPI_StatusQuestionableEventQ,},
    // {.pattern = "STATus:QUEStionable:CONDition?", .callback = scpi_stub_callback,},
    {.pattern = "STATus:QUEStionable:ENABle", .callback = SCPI_StatusQuestionableEnable,},
    {.pattern = "STATus:QUEStionable:ENABle?", .callback = SCPI_StatusQuestionableEnableQ,},
    {.pattern = "STATus:PRESet", .callback = SCPI_StatusPreset,},

    /* BSICS-SVR commands */

    {.pattern = "GRP[:SELect]", .callback = BSICS_SelectGroup,},
    {.pattern = "GRP?", .callback = BSICS_GroupQ,},

    {.pattern = "[GRP#]:SOURce:VOLtage:LO", .callback = BSICS_SetVoltageLo,},
    {.pattern = "[GRP#]:SOURce:VOLtage:HI", .callback = BSICS_SetVoltageHi,},
    {.pattern = "[GRP#]:SOURce:CURrent:LO", .callback = BSICS_SetCurrentLo,},
    {.pattern = "[GRP#]:SOURce:CURrent:HI", .callback = BSICS_SetCurrentHi,},

    {.pattern = "[GRP#]:MEASure:CH#:LO?", .callback = BSICS_ChannelVoltageLoQ,},
    {.pattern = "[GRP#]:MEASure:CH#:HI?", .callback = BSICS_ChannelVoltageHiQ,},
    {.pattern = "[GRP#]:MEASure:CH#:TEMP?", .callback = BSICS_ChannelTemperatureQ,},

    {.pattern = "[GRP#]:CONFigure:DRIVers[:ENAble]", .callback = BSICS_SetDriversEna,},
    {.pattern = "[GRP#]:CONFigure:DRIVers[:ENAble]?", .callback = BSICS_DriversEnaQ,},
    {.pattern = "[GRP#]:CONFigure:CH#:DRIVer[:STATe]", .callback = BSICS_SetChannelDriverMux,},
    {.pattern = "[GRP#]:CONFigure:CH#:DRIVer[:STATe]?", .callback = BSICS_ChannelDriverMuxQ,},

    {.pattern = "[GRP#]:STATus:DCDC[:OPERating]?", .callback = BSICS_StatusDCDCsQ,},
    {.pattern = "[GRP#]:STATus:DRIVers[:RDY]?", .callback = BSICS_StatusDriversReadyQ,},

    {.pattern = "[GRP#]:DISPlay:TEXT", .callback = BSICS_SetDisplayText,},
    {.pattern = "[GRP#]:DISPlay:TEXT?", .callback = BSICS_DisplayTextQ,},

    {.pattern = "[GRP#]:CALibration:STOre", .callback = BSICS_StoreCalibration,},
    {.pattern = "[GRP#]:CALibration:RECall", .callback = BSICS_RecallCalibration,},
    {.pattern = "[GRP#]:CALibration:CH#:ADC#", .callback = BSICS_SetChannelADCCalibration,},
    {.pattern = "[GRP#]:CALibration:CH#:ADC#?", .callback = BSICS_ChannelADCCalibrationQ,},

//    /* DMM demo commands (libscpi example) */
//    {.pattern = "MEASure:VOLTage:DC?", .callback = DMM_MeasureVoltageDcQ,},
//    {.pattern = "CONFigure:VOLTage:DC", .callback = DMM_ConfigureVoltageDc,},
//    {.pattern = "MEASure:VOLTage:DC:RATio?", .callback = SCPI_StubQ,},
//    {.pattern = "MEASure:VOLTage:AC?", .callback = DMM_MeasureVoltageAcQ,},
//    {.pattern = "MEASure:CURRent:DC?", .callback = SCPI_StubQ,},
//    {.pattern = "MEASure:CURRent:AC?", .callback = SCPI_StubQ,},
//    {.pattern = "MEASure:RESistance?", .callback = SCPI_StubQ,},
//    {.pattern = "MEASure:FRESistance?", .callback = SCPI_StubQ,},
//    {.pattern = "MEASure:FREQuency?", .callback = SCPI_StubQ,},
//    {.pattern = "MEASure:PERiod?", .callback = SCPI_StubQ,},

//    /* TEST commands (libscpi) */
//    {.pattern = "TEST:BOOL", .callback = TEST_Bool,},
//    {.pattern = "TEST:CHOice?", .callback = TEST_ChoiceQ,},
//    {.pattern = "TEST#:NUMbers#", .callback = TEST_Numbers,},
//    {.pattern = "TEST:TEXT", .callback = TEST_Text,},
//    {.pattern = "TEST:ARBitrary?", .callback = TEST_ArbQ,},
//    {.pattern = "TEST:CHANnellist", .callback = TEST_Chanlst,},

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
