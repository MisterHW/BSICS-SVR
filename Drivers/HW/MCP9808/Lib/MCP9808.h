//
// MCP9808 : Digital Temperature Sensor
//

#ifndef HW_MCP9808_H
#define HW_MCP9808_H

#include "stm32f7xx_hal.h"

// MCP9808 addresses (ALT block: "contact factory for this address code.")
enum MCP9808_address {
    // standard type
    MCP9808_ADDR_0x18 = 0x18,
    MCP9808_ADDR_0x19 = 0x19,
    MCP9808_ADDR_0x1A = 0x1A,
    MCP9808_ADDR_0x1B = 0x1B,
    MCP9808_ADDR_0x1C = 0x1C,
    MCP9808_ADDR_0x1D = 0x1D,
    MCP9808_ADDR_0x1E = 0x1E,
    MCP9808_ADDR_0x1F = 0x1F,
    // special order type
    MCP9808_ALT_ADDR_0x48 = 0x48,
    MCP9808_ALT_ADDR_0x49 = 0x49,
    MCP9808_ALT_ADDR_0x4A = 0x4A,
    MCP9808_ALT_ADDR_0x4B = 0x4B,
    MCP9808_ALT_ADDR_0x4C = 0x4C,
    MCP9808_ALT_ADDR_0x4D = 0x4D,
    MCP9808_ALT_ADDR_0x4E = 0x4E,
    MCP9808_ALT_ADDR_0x4F = 0x4F,
};

enum MCP9808_register {
    MCP9808_REG16_RFU        = 0x00 , // Reserved ( read-only, default 0x001F )
    MCP9808_REG16_config     = 0x01 , // default 0x0000
    MCP9808_REG16_T_upper    = 0x02 , // default 0x0000
    MCP9808_REG16_T_lower    = 0x03 , // default 0x0000
    MCP9808_REG16_T_critical = 0x04 , // default 0x0000
    MCP9808_REG16_T_ambient  = 0x05 , // conversion results and comparator flags
    MCP9808_REG16_mfr_ID     = 0x06 , // default 0x0054
    MCP9808_REG16_dev_ID     = 0x07 , // default 0x0400
    MCP9808_REG8_resolution  = 0x08 , // default 0x03
};

typedef union MCP9808_config_ {
    uint16_t val;
    struct __attribute__ ((__packed__)) {
        // lo byte
        uint8_t alert_mode  : 1 ; // alert behavior, 0: comparator (instantaneous), 1: latching (interrupt-type)
        uint8_t alert_pol   : 1 ; // alert polarity, 0: active-low + PU, 1: active-high
        uint8_t alert_sel   : 1 ; // alert source selection (locked by window_lock = 1)
        uint8_t alert_cntrl : 1 ; // alert output enable (locked by window_lock = 1 or crit_lock = 1)
        uint8_t alert_stat  : 1 ;
        uint8_t int_clear   : 1 ;
        uint8_t window_lock : 1 ;
        uint8_t crit_lock   : 1 ;
        // hi byte
        uint8_t shdn        : 1 ;
        uint8_t T_hyst      : 2 ; // (locked through lock bits 6 and 7)
        uint8_t reserved    : 5 ;
    } bits;
    inline MCP9808_config_& operator= (const uint16_t rhs) {
        val = rhs;
        return *this;
    };
} MCP9808_config;

enum MCP9808_config_attr {
    // lo byte
    MCP9808_CFG_alert_mode_comparator = 0 << 0, // (default)
    MCP9808_CFG_alert_mode_interrupt  = 1 << 0,
    MCP9808_CFG_alert_pol_active_low  = 0 << 1, // open-drain, PU required (default)
    MCP9808_CFG_alert_pol_active_high = 1 << 1,
    MCP9808_CFG_alert_sel_Tu_Tl_Tcrit    = 0 << 2, // (default)
    MCP9808_CFG_alert_sel_Ta_above_Tcrit = 1 << 2,
    MCP9808_CFG_alert_output_disabled = 0 << 3, // cntrl = 0 (default)
    MCP9808_CFG_alert_output_enabled  = 1 << 3, // cntrl = 1
    MCP9808_CFG_alert_stat_no_alert = 0 << 4, // no alert
    MCP9808_CFG_alert_stat_asserted = 1 << 4, // alert is active (see polarity)
    MCP9808_CFG_int_clear_no_action = 0 << 5, // can be issued in shutdown mode
    MCP9808_CFG_int_clear_initiate  = 1 << 5, // cannot be set in shutdown mode
    MCP9808_CFG_window_unlocked = 0 << 6, // T_upper, T_lower are writable
    MCP9808_CFG_window_locked   = 1 << 6, // lock T_upper, T_lower (can only be unlocked through POR)
    MCP9808_CFG_Tcrit_unlocked = 0 << 7, // T_critical is writable
    MCP9808_CFG_Tcrit_locked   = 1 << 7, // T_critical is locked (can only be unlocked through POR)
    // hi byte
    MCP9808_CFG_normal_operation = 0 << 8, // device is in active (contiuous conversion) mode
    MCP9808_CFG_shutdown_mode    = 1 << 8, // device shutdown
    MCP9808_CFG_hyst_0p0degC = 0 << 9,
    MCP9808_CFG_hyst_1p5degC = 1 << 9,
    MCP9808_CFG_hyst_3p0degC = 2 << 9,
    MCP9808_CFG_hyst_6p0degC = 3 << 9,
};

enum MCP9808_T_mask {
    MCP9808_T_abs_temp_mask = 0x0FFF,
    MCP9808_T_sign_bit_mask = 0x1000,
    MCP9808_T_Tlow_mask     = 0x2000,
    MCP9808_T_Thigh_mask    = 0x4000,
    MCP9808_T_Tcrit_mask    = 0x8000,
};

typedef union MCP9808_T_ {
    uint16_t val;
    struct __attribute__ ((__packed__)) {
        // lo byte
        uint16_t abs_temp  : 12;
        uint8_t sign       :  1;
        uint8_t comp_Tlo   :  1; // T_ambient only - otherwise reads as 0, read-only
        uint8_t comp_Thi   :  1; // T_ambient only - otherwise reads as 0, read-only
        uint8_t comp_Tcrit :  1; // T_ambient only - otherwise reads as 0, read-only
    } bits;
    // assignment operator to treat MCP9808_T as an int type
    inline MCP9808_T_& operator= (const uint16_t rhs) {
        val = rhs;
        return *this;
    };
    // int conversion operator to use MCP9808_T in algebraic expressions.
#if defined(__cplusplus) && (__cplusplus >= 201103L) // require C++11 or later for "explicit"
    inline explicit operator uint16_t() const {
        return val;
    };
#else
    inline operator uint16_t() const {
            return val;
    };
#endif
} MCP9808_T;


class MCP9808 {
    I2C_HandleTypeDef* hI2C {nullptr};
    MCP9808_address deviceAddress{};
public:
    bool init(I2C_HandleTypeDef* _hI2C, MCP9808_address addr);
    bool isReady( );

    bool write( MCP9808_register reg, uint8_t* data, unsigned int len );
    bool read ( MCP9808_register reg, uint8_t* data, unsigned int len );
    bool writeReg16( MCP9808_register reg, uint16_t value );
    bool writeReg8 ( MCP9808_register reg, uint8_t value );
    bool readReg16( MCP9808_register reg, uint16_t &value );
    bool readReg8 ( MCP9808_register reg, uint8_t &value );

    static int32_t raw_to_millidegC( MCP9808_T raw_temp )
    {
        int32_t tmp = (raw_temp.bits.abs_temp * 131) >> 1;
        return raw_temp.bits.sign ? -tmp : tmp;

    };

    static int16_t raw_to_degC( MCP9808_T raw_temp )
    {
        uint16_t tmp = raw_temp.bits.abs_temp >> 4;
        return raw_temp.bits.sign ? -tmp : tmp;
    };
};

#endif //HW_MCP9808_H
