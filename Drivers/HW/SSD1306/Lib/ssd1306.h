
#ifndef HW_SSD1306_H
#define HW_SSD1306_H

#include "stm32f7xx_hal.h"
#include "fonts.h"

// Slave addresses. D/~C pin acts as SA0 for slave address selection.
enum SSD1306_address {
    SSD1306_ADDR_0x3C = 0x3C, // D/~C low
    SSD1306_ADDR_0x3D = 0x3D, // D/~C high
};

enum SSD1306_command {
	SSD1306_MEMORY_ADDR_MODE      = 0x20, //
	SSD1306_SET_COLUMN_ADDR       = 0x21, // See datasheet
	SSD1306_SET_PAGE_ADDR         = 0x22, // See datasheet
	SSD1306_SET_CONTRAST_CONTROL  = 0x81, // See datasheet
	SSD1306_SET_PAGE_START_ADDDR  = 0xB0,
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
	SSD1306_SET_START_LINE        = 0x40, // See datasheet
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


enum SSD1306_colors {
    black = 0x00,   // Black color, no pixel
    white = 0x01,   // Pixel is set. Color depends on LCD
};

template <size_t _width , size_t _height>
class SSD1306 {
    I2C_HandleTypeDef*  hI2C {};
    SSD1306_address deviceAddress;
	uint16_t width  = _width;
	uint16_t height = _height;
    uint8_t buffer[_width * (_height >> 3)];
	uint8_t rotation = 0;
	bool inverted = false;
    uint16_t currentX;
    uint16_t currentY;
	
    void send_command(SSD1306_command cmd);
    void send_command(SSD1306_command cmd, uint8_t data);
	void send_data(uint8_t byte);
	bool send_data(uint8_t* bytes, uint16_t len);
public:
    bool init(I2C_HandleTypeDef *_hI2C, SSD1306_address addr);
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

template<size_t _width, size_t _height>
void SSD1306<_width, _height>::send_command(SSD1306_command cmd) {

}

template<size_t _width, size_t _height>
void SSD1306<_width, _height>::send_command(SSD1306_command cmd, uint8_t data) {

}

template<size_t _width, size_t _height>
void SSD1306<_width, _height>::send_data(uint8_t byte) {

}

template<size_t _width, size_t _height>
bool SSD1306<_width, _height>::send_data(uint8_t *bytes, uint16_t len) {
    return false;
}

template<size_t _width, size_t _height>
bool SSD1306<_width, _height>::init(I2C_HandleTypeDef *_hI2C, SSD1306_address addr) {
    return false;
}

template<size_t _width, size_t _height>
bool SSD1306<_width, _height>::isReady() {
    return false;
}

template<size_t _width, size_t _height>
void SSD1306<_width, _height>::clearBuffer() {

}

template<size_t _width, size_t _height>
bool SSD1306<_width, _height>::updateDisplay() {
    return false;
}

template<size_t _width, size_t _height>
void SSD1306<_width, _height>::displayFullyOn(bool on) {

}

template<size_t _width, size_t _height>
void SSD1306<_width, _height>::drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, uint16_t bmp_width, uint16_t bmp_height,
                                          SSD1306_colors color) {

}

template<size_t _width, size_t _height>
void SSD1306<_width, _height>::drawPixel(int16_t x, int16_t y, SSD1306_colors color) {

}

template<size_t _width, size_t _height>
void SSD1306<_width, _height>::invertColors() {

}

template<size_t _width, size_t _height>
char SSD1306<_width, _height>::writeChar(char ch, FontDef Font, SSD1306_colors color) {
    return 0;
}

template<size_t _width, size_t _height>
uint32_t SSD1306<_width, _height>::writeString(char *str, FontDef Font, SSD1306_colors color) {
    return 0;
}

template<size_t _width, size_t _height>
void SSD1306<_width, _height>::setCursor(uint8_t x, uint8_t y) {

}


using SSD1306_128x32 = SSD1306< 128, 32 >;
using SSD1306_128x64 = SSD1306< 128, 64 >;

#endif // HW_SSD1306_H
