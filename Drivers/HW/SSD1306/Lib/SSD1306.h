
#ifndef HW_SSD1306_H
#define HW_SSD1306_H

#include "stm32f7xx_hal.h"
#include "fonts.h"

// Slave addresses. D/~C pin acts as SA0 for slave address selection.
enum SSD1306_address {
    SSD1306_ADDR_0x3C = 0x3C, // D/~C low
    SSD1306_ADDR_0x3D = 0x3D, // D/~C high
};

//
enum SSD1306_control_bits {
    SSD1306_CTRL_WORD_CONTAINS_COMMAND         = 0 << 6, // byte following control byte is a command
    SSD1306_CTRL_WORD_CONTAINS_DATA            = 1 << 6, // byte following control byte is data
    SSD1306_CTRL_ALL_FOLLOWING_BYTES_ARE_DATA  = 0 << 7, // continue = 0 : stop looking for control bytes, only data bytes will follow
    SSD1306_CTRL_NEXT_WORD_STARTS_WITH_CONTROL = 1 << 7, // continue = 1 : expect more words starting with control bytes
};

enum SSD1306_command {
	SSD1306_MEMORY_ADDR_MODE      = 0x20, //
	SSD1306_SET_COLUMN_ADDR       = 0x21, // See datasheet
	SSD1306_SET_PAGE_ADDR         = 0x22, // See datasheet
	SSD1306_SET_CONTRAST_CONTROL  = 0x81, // See datasheet
	SSD1306_SET_PAGE_START_ADDR   = 0xB0,
	SSD1306_CHARGE_PUMP           = 0x8D, // See datasheet
	SSD1306_SET_SEGMENT_REMAP     = 0xA0, // See datasheet
	SSD1306_SET_SEGMENT_REMAP_INV = 0xA1, // See datasheet
	SSD1306_DISPLAY_ALL_ON_RESUME = 0xA4, // See datasheet
	SSD1306_DISPLAY_ALL_ON        = 0xA5, // Not currently used
	SSD1306_NORMAL_DISPLAY        = 0xA6, // See datasheet
	SSD1306_INVERTDISPLAY         = 0xA7, // See datasheet
	SSD1306_SET_MULTIPLEX_RATIO   = 0xA8, // See datasheet
	SSD1306_DISPLAY_OFF           = 0xAE, // See datasheet
	SSD1306_DISPLAY_ON            = 0xAF, // See datasheet
	SSD1306_COM_SCAN_DIR_INC      = 0xC0, // Not currently used
	SSD1306_COM_SCAN_DIR_DEC      = 0xC8, // See datasheet
	SSD1306_SET_DISPLAY_OFFSET    = 0xD3, // See datasheet
	SSD1306_SET_PRECHARGE_PERIOD  = 0xD9, // See datasheet
	SSD1306_SET_COM_PINS          = 0xDA, // See datasheet
	SSD1306_SET_VCOM_DESELECT     = 0xDB, // See datasheet
	SSD1306_SET_LOWER_COLUMN      = 0x00, // Not currently used
	SSD1306_SET_HIGHER_COLUMN     = 0x10, // Not currently used
	SSD1306_START_LINE_CMD_BASE   = 0x40, // 0x40 .. 0x7F commands select RAM row 0 .. 63 mapping to COM0. 0x40+n: n->COM0, n+1->COM1, ...
	SSD1306_EXTERNALVCC           = 0x01, // External display voltage source
	SSD1306_SWITCHCAPVCC          = 0x02, // Gen. display voltage from 3.3V
	SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO          = 0xD5, // See datasheet
	SSD1306_RIGHT_HORIZONTAL_SCROLL              = 0x26, // Init rt scroll
	SSD1306_LEFT_HORIZONTAL_SCROLL               = 0x27, // Init left scroll
	SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL = 0x29, // Init diag scroll
	SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  = 0x2A, // Init diag scroll
	SSD1306_DEACTIVATE_SCROLL                    = 0x2E, // Stop scroll
	SSD1306_ACTIVATE_SCROLL                      = 0x2F, // Start scroll
	SSD1306_SET_VERTICAL_SCROLL_AREA             = 0xA3  // Set scroll range
};

#define SSD1306_SET_START_LINE(n) (SSD1306_command)((int)SSD1306_START_LINE_CMD_BASE + (n % 64))

enum SSD1306_colors {
    black = 0x00,   // Black color, no pixel
    white = 0x01,   // Pixel is set. Color depends on LCD
};

template <size_t disp_width , size_t disp_height>
class SSD1306 {
protected:
    I2C_HandleTypeDef*  hI2C {};
    SSD1306_address deviceAddress;
	uint16_t width  = disp_width;
	uint16_t height = disp_height;
    uint8_t buffer[disp_width * (disp_height >> 3)];
	uint8_t rotation = 0;

    uint16_t currentX;
    uint16_t currentY;

    bool inv_x = false;
    bool inv_y = false;
    bool inv_c = false; // invert color (B/W)

    bool send_command(SSD1306_command cmd);
    bool send_command(SSD1306_command cmd, uint8_t data);
	bool send_data(uint8_t* bytes, uint16_t len);

    virtual bool init_panel_specifics() = 0;
public:
    bool initialized {false};

    bool init(I2C_HandleTypeDef *_hI2C, SSD1306_address addr, bool invert_x = false, bool invert_y = false);

    bool isReady();

    void clearBuffer();
    bool updateDisplay();
	void displayFullyOn(bool on = true);
	void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                    uint16_t bmp_width, uint16_t bmp_height, SSD1306_colors color);
	void drawPixel(int16_t x, int16_t y, SSD1306_colors color);
	void invertColors();
	char writeChar(char ch, FontDef Font, SSD1306_colors color);
	uint32_t writeString(char* str, FontDef Font, SSD1306_colors color);
	void setCursor(uint8_t x, uint8_t y);

};

template <size_t disp_width , size_t disp_height>
bool SSD1306<disp_width, disp_height>::send_command(SSD1306_command cmd) {
    uint8_t buf[2];
    buf[0] = SSD1306_CTRL_WORD_CONTAINS_COMMAND ;
    buf[1] = cmd;
    return HAL_OK == HAL_I2C_Master_Transmit( hI2C, deviceAddress << 1, buf, 2, 5 );
}

template <size_t disp_width , size_t disp_height>
bool SSD1306<disp_width, disp_height>::send_command(SSD1306_command cmd, uint8_t data) {
    uint8_t buf[3];
    buf[0] = SSD1306_CTRL_WORD_CONTAINS_COMMAND | SSD1306_CTRL_ALL_FOLLOWING_BYTES_ARE_DATA ;
    buf[1] = cmd;
    buf[2] = data;
    return HAL_OK == HAL_I2C_Master_Transmit( hI2C, deviceAddress << 1, buf, 3, 5 );
}

template <size_t disp_width , size_t disp_height>
bool SSD1306<disp_width, disp_height>::send_data(uint8_t *bytes, uint16_t len) {
    return false;
}

template <size_t disp_width , size_t disp_height>
bool
SSD1306<disp_width, disp_height>::init(I2C_HandleTypeDef *_hI2C, SSD1306_address addr, bool invert_x, bool invert_y) {
    hI2C = _hI2C;
    deviceAddress = addr;
    inv_x = invert_x;
    inv_y = invert_y;

    initialized = isReady();
    if( not initialized){
        return false;
    }
    bool i = true;

    i &= send_command( SSD1306_DISPLAY_OFF );

    // Timing and driving.
    i &= send_command( SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO , 0x80 );
    i &= send_command( SSD1306_SET_PRECHARGE_PERIOD , 0x22 );
    i &= send_command( SSD1306_SET_VCOM_DESELECT    , 0x40 );
    i &= send_command( SSD1306_SET_CONTRAST_CONTROL , 0x7F );
    i &= send_command( SSD1306_SET_DISPLAY_OFFSET   , 0x00 );

    // Scrolling
    i &= send_command( SSD1306_DEACTIVATE_SCROLL );

    // memory use and mapping to COM and SEGMENT lines (panel-specific)
    i &= init_panel_specifics();

    // Charge pump.
    i &= send_command( SSD1306_CHARGE_PUMP , 0x14 );

    // Display back on.
    i &= send_command( SSD1306_DISPLAY_ALL_ON_RESUME );
    i &= send_command( SSD1306_NORMAL_DISPLAY );
    i &= send_command( SSD1306_DISPLAY_ON );

    initialized = i;
    return initialized;
}

template<size_t disp_width, size_t disp_height>
bool SSD1306<disp_width, disp_height>::isReady() {
    return HAL_OK == HAL_I2C_IsDeviceReady(hI2C, deviceAddress << 1, 3, 5 );
}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::clearBuffer() {

}

template<size_t disp_width, size_t disp_height>
bool SSD1306<disp_width, disp_height>::updateDisplay() {
    return false;
}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::displayFullyOn(bool on) {

}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t bmp_width, uint16_t bmp_height,
                                          SSD1306_colors color) {

}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::drawPixel(int16_t x, int16_t y, SSD1306_colors color) {

}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::invertColors() {

}

template<size_t disp_width, size_t disp_height>
char SSD1306<disp_width, disp_height>::writeChar(char ch, FontDef Font, SSD1306_colors color) {
    return 0;
}

template<size_t disp_width, size_t disp_height>
uint32_t SSD1306<disp_width, disp_height>::writeString(char *str, FontDef Font, SSD1306_colors color) {
    return 0;
}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::setCursor(uint8_t x, uint8_t y) {

}


class SSD1306_128x32 : public SSD1306< 128, 32 > {
protected:
    bool init_panel_specifics() override {
        bool i = true;
        // Addressing
        i &= send_command(SSD1306_MEMORY_ADDR_MODE, 0x10);        // Page addressing mode.
        i &= send_command(SSD1306_SET_START_LINE(0));

        // Hardware config
        i &= send_command(SSD1306_SET_SEGMENT_REMAP_INV);
        i &= send_command(SSD1306_SET_MULTIPLEX_RATIO, height - 1);
        i &= send_command(SSD1306_COM_SCAN_DIR_DEC);
        i &= send_command(SSD1306_SET_COM_PINS, 0x12);

        return i;
    }
};

class SSD1306_128x64 : public SSD1306< 128, 64 > {
protected:
    bool init_panel_specifics() override {
        bool i = true;
        // Addressing
        i &= send_command(SSD1306_MEMORY_ADDR_MODE, 0x10);        // Page addressing mode.
        i &= send_command(SSD1306_SET_START_LINE(0));

        // Hardware config
        i &= send_command(SSD1306_SET_SEGMENT_REMAP_INV);
        i &= send_command(SSD1306_SET_MULTIPLEX_RATIO, height - 1);
        i &= send_command(SSD1306_COM_SCAN_DIR_DEC);
        i &= send_command(SSD1306_SET_COM_PINS, 0x12);

        return i;
    }
};

#endif // HW_SSD1306_H
