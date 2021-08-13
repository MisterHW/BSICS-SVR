
#ifndef HW_SSD1306_H
#define HW_SSD1306_H

#include "stm32f7xx_hal.h"
#include "fonts.h"
#include "string.h"

// Slave addresses. D/~C pin acts as SA0 for slave address selection.
enum SSD1306_address {
    SSD1306_ADDR_0x3C = 0x3C, // D/~C low
    SSD1306_ADDR_0x3D = 0x3D, // D/~C high
};

/*  SSD1306 I2C Write Operations:
 *  The first byte transmitted is a CONTROL byte. It determines whether:
 *  1) the second byte is a command or display data, and
 *  2) whether the third byte is another control byte or whether only data bytes follow after the second byte.
 *  The datasheet isn't very clear on the I2C write format, leading to many implementations using fragmented
 *  transmissions containing a word made up of
 *      S : ADDR<<1|W : ACK : CONTROL : ACK : COMMAND : ACK : P - where CONTROL = 0x00
 *  or
 *      S : ADDR<<1|W : ACK : CONTROL : ACK : DATA : ACK : P - where CONROL = 0x00.
 *  The SSD1306 interface state machine seems to allow this, but the datasheet wording "triple byte command"
 *  seems to imply e.g.
 *      S : ADDR<<1|W : ACK : CONTROL : ACK : COMMAND : ACK : DATA0 : ACK : DATA1: ACK : P - where CONTROL = 0x00.
 *
 *  Commands with > 1 byte operands would require the SSD1306 state machine to interpret commands and know how many
 *  operand bytes to expect, which does not seem to be the case (Table 8-7 shows > 1 byte operands as necessarily
 *  followed by a STOP, and SSD1306_CTRL_NEXT_WORD_STARTS_WITH_CONTROL would not be necessary). So only word instructions
 *  containing CONTROL + Byte can precede a subsequent instruction.
 *
 *  In this implementation, overloaded send_command() methods are provided for non-fragmented write operations.
 *  Multi-byte commands can be sent via send_raw() using an array of bytes. Initializer macros
 *  SSD1306_3B_COMMAND_LEN4, SSD1306_6B_COMMAND_LEN7 and SSD1306_7B_COMMAND_LEN8
 *  are given to cover these cases (see scroll commands).
 */
enum SSD1306_control_bits {
    SSD1306_CTRL_WORD_CONTAINS_COMMAND         = 0 << 6, // byte following control byte is a command
    SSD1306_CTRL_WORD_CONTAINS_DATA            = 1 << 6, // byte following control byte is data
    SSD1306_CTRL_ALL_FOLLOWING_BYTES_ARE_DATA  = 0 << 7, // continue = 0 : stop looking for control bytes, only data bytes will follow
    SSD1306_CTRL_NEXT_WORD_STARTS_WITH_CONTROL = 1 << 7, // continue = 1 : expect more words starting with control bytes
};

enum SSD1306_command {
    // Fundamental Commands
    SSD1306_SET_CONTRAST_CONTROL  = 0x81, // 0x81, A[7:0] - contrast / brightness A = 0 .. 255
    SSD1306_DISPLAY_ALL_ON_RESUME = 0xA4, // 0xA4 - Output follows RAM content
    SSD1306_DISPLAY_ALL_ON        = 0xA5, // 0xA5 - Output ignores RAM content
    SSD1306_NORMAL_DISPLAY        = 0xA6, // 0xA6 - 0: dark pixel, 1: bright pixel (default)
    SSD1306_INVERSE_DISPLAY       = 0xA7, // 0xA7 - 0: bright pixel, 1: dark pixel
    SSD1306_DISPLAY_OFF           = 0xAE, // 0xAE - Display OFF, sleep mode
    SSD1306_DISPLAY_ON            = 0xAF, // 0xAF - Display ON, normal mode

    // Scrolling Commands
    SSD1306_CONT_HORIZONTAL_SCROLL_RIGHT         = 0x26, // 0x26, 0x00, B[2:0], C[2:0], D[2:0], 0x00, 0xFF - start page B, C : {5, 64, 128, 256, 3, 4, 35, 2} frames, end page D
    SSD1306_CONT_HORIZONTAL_SCROLL_LEFT          = 0x27, // 0x27, 0x00, B[2:0], C[2:0], D[2:0], 0x00, 0xFF - start page B, C : {5, 64, 128, 256, 3, 4, 35, 2} frames, end page D
    SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL = 0x29, // 0x29, 0x00, B[2:0], C[2:0], D[2:0], E[5:0] - B, C, D: see 0x26 and 0x27; E[5:0] = 1..63 rows vert. scrolling offset
    SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL  = 0x2A, // 0x2A, 0x00, B[2:0], C[2:0], D[2:0], E[5:0] - see 0x29
    SSD1306_DEACTIVATE_SCROLL                    = 0x2E, // 0x2E - Stop scrolling (deactivate before changing H / V scroll mode to avoid RAM corruption)
    SSD1306_ACTIVATE_SCROLL                      = 0x2F, // 0x2F - Start scrolling
    SSD1306_SET_VERTICAL_SCROLL_AREA             = 0xA3, // 0xA3, A[5:0], B[6:0] - top fixed rows A, scrolled rows (starts at first row below last fixed)

    // Addressing Setting Commands
    SSD1306_COLUMN_ADDR_LOWER_NIBBLE_BASE = 0x00, // 0x0X - single-byte commands range 0x00 .. 0x0F, Column Start Address [3:0] for Page Addressing Mode
    SSD1306_COLUMN_ADDR_UPPER_NIBBLE_BASE = 0x10, // 0x1X - single-byte commands range 0x10 .. 0x1F, Column Start Address [7:4] for Page Addressing Mode
    SSD1306_MEMORY_ADDR_MODE              = 0x20, // 0x20, A[1:0] - A = {horiz. addr mode, vert. addr mode, page addr mode, 11b: invalid}
    SSD1306_SET_COLUMN_ADDR               = 0x21, // 0x21, A[6:0], B[6:0] - col start A = 0..127, col end B = 0..127 (H and V addr mode only)
    SSD1306_SET_PAGE_ADDR                 = 0x22, // 0x22, A[2:0], B[2:0] - page start A = 0..7, page end B = 0..7. Also sets page address pointer to start.
    SSD1306_PAGE_START_ADDR_BASE          = 0xB0, // 0xBX - single-byte commands range 0xB0 .. 0xB7, page start address 0..7 (for page addr mode only)

    // Hardware Configuration Commands
    SSD1306_START_LINE_CMD_BASE     = 0x40, // 0xXX single-byte commands range  0x40 .. 0x7F, commands select RAM row 0 .. 63 mapping to COM0. 0x40+n: n->COM0, n+1->COM1, ...
    SSD1306_SET_SEGMENT_REMAP_OFF   = 0xA0, // 0xA0 - column addr 0   -> SEG0
    SSD1306_SET_SEGMENT_REMAP_INV   = 0xA1, // 0xA1 - column addr 127 -> SEG0 (typical)
    SSD1306_SET_MULTIPLEX_RATIO     = 0xA8, // 0xA8, A[5:0] - A = 0.. 63 (= height - 1)
    SSD1306_COM_SCAN_DIR_INC        = 0xC0, // 0xC0 - COM scan direction COM0 to COM[N–1] (N : multiplex ratio)
    SSD1306_COM_SCAN_DIR_DEC        = 0xC8, // 0XC8 - COM scan direction COM[N-1] to COM0 (N : multiplex ratio) (typical)
    SSD1306_SET_VERT_DISPLAY_OFFSET = 0xD3, // 0xD3, A[5:0] - Set vertical shift by COM, A = 0 .. 63
    SSD1306_SET_COM_PINS            = 0xDA, // 0xDA, A[7:0] - COM sequence: A[5] = 0/1: COM L-R remap off / on. A[4] = 0: sequential, A[4] = 1: alternative (default), A[3:0] = 0x2, others 0

    // Timing & Driving Scheme Setting Commands
    SSD1306_SET_FOSC_AND_CLOCK_DIV_RATIO = 0xD5, // 0xD5, A[7:0] - A[7:4]: F_OSC (280 .. 520 kHz, see datasheet), A[3:0]: display clock DCLK = F_OSC / (A[3:0] + 1)
    SSD1306_SET_PRECHARGE_PERIOD         = 0xD9, // 0xD9, A[7:0] - A[7:4]: Phase2 period (0 is invalid), A[3:0]: Phase1 period (0 is invalid)
    SSD1306_SET_VCOMH_DESELECT_LEVEL     = 0xDB, // 0xDB, A[6:4] - A = 0x00: ~0.65*Vcc, 0x20: ~0.77*Vcc, 0x30: ~0.83*Vcc, 0x40: ~0.89*Vcc? (unspecified, brighter than 0x30)
    SSD1306_NOP                          = 0xE3, // 0xE3 - no operation (not needed for I2C interface)

    // Advanced Graphics Commands
    SSD1306_SET_FADE_OUT_AND_BLINKING = 0x23, // 0x23, A[6:0] - A[5:4]: 00b: disabled, 01b: unknown, 10b: fade-out, 11b: blinking, time interval A[3:0]*8 frames
    SSD1306_SET_ZOOM_IN               = 0xD6, // 0xD6, A[0] - A[0] = 0b: disable zoom mode, 1b: enable zoom mode (requires alternative COM mode)

    // Charge Pump Commands
    SSD1306_CHARGE_PUMP_STATE = 0x8D, // 0x8D, A[7:0] - 0x10: disable charge pump, 0x14: enable charge pump (NOTE: command must ultimately be followed by Display ON command: 0x8D, 0x14, 0xAF)
};

// macros for packed single-byte commands
#define SSD1306_SET_START_LINE(n)      (SSD1306_command)((int)SSD1306_START_LINE_CMD_BASE + ((n) % 64))
#define SSD1306_SET_COLUMN_ADDR_LOWER_NIBBLE(addr) (SSD1306_command)((int)SSD1306_COLUMN_ADDR_LOWER_NIBBLE_BASE + ( (addr) & 0x0F) )
#define SSD1306_SET_COLUMN_ADDR_UPPER_NIBBLE(addr) (SSD1306_command)((int)SSD1306_COLUMN_ADDR_UPPER_NIBBLE_BASE + ( ((addr) >> 4) & 0x0F) )
#define SSD1306_SET_PAGE_START_ADDR(n) (SSD1306_command)((int)SSD1306_PAGE_START_ADDR_BASE + ((n) % 8))

// uint8_t buf[] array initializer macros for send_raw() calls
#define SSD1306_DOUBLE_COMMAND_LEN4(cmd1, cmd2) {SSD1306_CTRL_WORD_CONTAINS_COMMAND | SSD1306_CTRL_NEXT_WORD_STARTS_WITH_CONTROL, (cmd1), SSD1306_CTRL_WORD_CONTAINS_COMMAND, (cmd2)}
#define SSD1306_3B_COMMAND_LEN4(cmd, A, B)             {SSD1306_CTRL_WORD_CONTAINS_COMMAND, (cmd), (A), (B)}
#define SSD1306_6B_COMMAND_LEN7(cmd, A, B, C, D, E)    {SSD1306_CTRL_WORD_CONTAINS_COMMAND, (cmd), (A), (B), (C), (D), (E)}
#define SSD1306_7B_COMMAND_LEN8(cmd, A, B, C, D, E, F) {SSD1306_CTRL_WORD_CONTAINS_COMMAND, (cmd), (A), (B), (C), (D), (E), (F)}

/* Parallel interface use D/~C as data/control select. "The serial interface mode is always in write mode."
 * "In I2C mode, [D/~C] acts as SA0 for slave address selection." N
 * No GDDRAM data can be read back, but reading the status byte may be possible.
 */
enum SSD1306_status {
    SSD1306_STATUS_DISPLAY_OFF = 1 << 6, // bit mask for "not display on"
    // All other bits are reserved.
};

enum SSD1306_color {
    monochrome_black = 0x00,   // Pixel is off (normal mode)
    monochrome_white = 0xFF,   // Pixel is on  (normal mode)
};

enum SSD1306_copy_mode {
    SSD1306_CM_NORMAL,          // pixels get copied as they are
//    SSD1306_CM_INVERTED,        // pixels get copied as their inverse color
    SSD1306_CM_COMPOSITE_COLOR, // pixel is set to a color, bitmap acts as mask
//    SSD1306_CM_COMPOSITE_XOR,   // pixels are inverted, bitmap acts as mask
};


template <size_t disp_width , size_t disp_height>
class SSD1306 {
protected:
    I2C_HandleTypeDef*  hI2C {};
    SSD1306_address deviceAddress;

    uint16_t width  = disp_width;
    uint16_t height = disp_height;
    uint8_t buffer[disp_width * (disp_height >> 3)];

    uint16_t currentX = 0;
    uint16_t currentY = 0;

    bool inv_x = false;
    bool inv_y = false;

    virtual bool init_panel_specifics() = 0;
public:
    bool initialized {false};

    // primary methods
    bool init(I2C_HandleTypeDef *_hI2C, SSD1306_address addr, bool invert_x = false, bool invert_y = false);
    bool isReady();
    bool send_command(SSD1306_command cmd);
    bool send_command(SSD1306_command cmd, uint8_t data);
    bool send_raw(uint8_t *data, uint16_t len);
    bool send_data(uint8_t* bytes, uint16_t len);
    bool read_status(uint8_t& status);
    virtual bool configure_orientation(bool mirror_H, bool mirror_V); // default SSD1306_COM_SCAN_DIR_DEC;

    // canvas methods
    void clearBuffer();
    bool updateDisplay(uint8_t page_offset = 0, uint8_t column_offset = 0);
    void drawBitmap(int16_t src_x, int16_t src_y, int16_t dest_x, int16_t dest_y,
                    const uint8_t *bitmap, uint16_t bmp_width, uint16_t bmp_height,
                    SSD1306_color color, SSD1306_copy_mode mode = SSD1306_CM_NORMAL);
    void drawPixel(int16_t x, int16_t y, int color);
    char writeChar(char ch, FontDef Font, SSD1306_color color);
    char * writeString(char* str, FontDef Font, SSD1306_color color);
    void setCursor(uint8_t x, uint8_t y);

};

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

    // Display OFF
    i &= send_command( SSD1306_DISPLAY_OFF );

    // Charge pump.
    i &= send_command( SSD1306_CHARGE_PUMP_STATE ,            0x14 ); // charge pump on

    // Timing and driving.
    i &= send_command( SSD1306_SET_PRECHARGE_PERIOD         , 0xF1 ); // pre-charge timing. For background, see US Pat. 7126568 (" a capacitive aspect of a display element is precharged" .. before forward current is driven)
    i &= send_command( SSD1306_SET_FOSC_AND_CLOCK_DIV_RATIO , 0x80 ); // medium F_OSC, DCLK = F_OSC/1
    i &= send_command( SSD1306_SET_VCOMH_DESELECT_LEVEL ,     0x30 ); // VCOM regulator setting affecting brightness
    i &= send_command( SSD1306_SET_CONTRAST_CONTROL ,         0x5F ); // formal brightness level 0x00 .. 0xFF

    // Scrolling
    i &= send_command( SSD1306_DEACTIVATE_SCROLL ); // no H or V scrolling

    // Addressing
    i &= send_command(SSD1306_MEMORY_ADDR_MODE, 2); // page addressing mode
    i &= send_command(SSD1306_SET_START_LINE(0)); // RAM to COM offset = 0

    // Memory Use and Mapping to COM and SEGMENT lines (general case or panel-specific)
    i &= send_command(SSD1306_SET_SEGMENT_REMAP_INV); // inverse column mapping
    i &= send_command(SSD1306_COM_SCAN_DIR_DEC); // scan dir reverse
    i &= send_command(SSD1306_SET_VERT_DISPLAY_OFFSET, 0);
    i &= init_panel_specifics(); // panel-specific commands (may override commands above)

    //  Display ON
    i &= send_command( SSD1306_DISPLAY_ALL_ON_RESUME ); // display GDDRAM data
    i &= send_command( SSD1306_NORMAL_DISPLAY ); // 0 maps to dark pixels
    i &= send_command( SSD1306_DISPLAY_ON ); // display on (charge pump activated above)

    initialized = i;
    return initialized;
}

template<size_t disp_width, size_t disp_height>
bool SSD1306<disp_width, disp_height>::isReady() {
    return HAL_OK == HAL_I2C_IsDeviceReady(hI2C, deviceAddress << 1, 3, 5 );
}

template <size_t disp_width , size_t disp_height>
bool SSD1306<disp_width, disp_height>::send_command(SSD1306_command cmd) {
    // S
    // deviceAddress << 1 | 0 : ACK : CONTROL 0x00 : ACK : COMMAND : ACK
    // P
    uint8_t buf[2];
    buf[0] = SSD1306_CTRL_WORD_CONTAINS_COMMAND ;
    buf[1] = cmd;
    return HAL_OK == HAL_I2C_Master_Transmit( hI2C, deviceAddress << 1, buf, 2, 5 );
}

template <size_t disp_width , size_t disp_height>
bool SSD1306<disp_width, disp_height>::send_command(SSD1306_command cmd, uint8_t data) {
    // S
    // deviceAddress << 1 | 0 : ACK : CONTROL 0x00 : ACK : COMMAND : ACK : DATA : ACK
    // P
    uint8_t buf[3];
    buf[0] = SSD1306_CTRL_WORD_CONTAINS_COMMAND | SSD1306_CTRL_ALL_FOLLOWING_BYTES_ARE_DATA ;
    buf[1] = cmd;
    buf[2] = data;
    return HAL_OK == HAL_I2C_Master_Transmit( hI2C, deviceAddress << 1, buf, 3, 5 );
}

template<size_t disp_width, size_t disp_height>
bool SSD1306<disp_width, disp_height>::send_raw(uint8_t *data, uint16_t len) {
    // Note: D0 must be a CONTROL byte.
    // S
    // deviceAddress << 1 | 0 : ACK : D0 : ACK : D1 : ACK : ... : Dn : ACK
    // P
    return HAL_OK == HAL_I2C_Master_Transmit( hI2C, deviceAddress << 1, data, len, 5 );
}

template <size_t disp_width , size_t disp_height>
bool SSD1306<disp_width, disp_height>::send_data(uint8_t *bytes, uint16_t len) {
    // S
    // deviceAddress << 1 | 0 : ACK : CONTROL 0x40 : ACK : D0 : ACK : D1 : ACK : ... : Dn : ACK
    // P
    return HAL_OK == HAL_I2C_Mem_Write( hI2C, deviceAddress << 1,
                                        SSD1306_CTRL_WORD_CONTAINS_DATA | SSD1306_CTRL_ALL_FOLLOWING_BYTES_ARE_DATA , 1 ,
                                        bytes, len, 5);
}

template<size_t disp_width, size_t disp_height>
bool SSD1306<disp_width, disp_height>::read_status(uint8_t &status) {
    // S
    // deviceAddress << 1 | 1 : ACK : STATUS
    // P
    return HAL_OK == HAL_I2C_Master_Receive(hI2C, deviceAddress << 1, &status, 1, 5);
}

template<size_t disp_width, size_t disp_height>
bool SSD1306<disp_width, disp_height>::configure_orientation(bool mirror_H, bool mirror_V) {
    bool success;
    success  = send_command( mirror_H ? SSD1306_COM_SCAN_DIR_INC      : SSD1306_COM_SCAN_DIR_DEC      );
    success &= send_command( mirror_V ? SSD1306_SET_SEGMENT_REMAP_OFF : SSD1306_SET_SEGMENT_REMAP_INV );
    return success;
}

template<size_t disp_width, size_t disp_height>
bool SSD1306<disp_width, disp_height>::updateDisplay(uint8_t page_offset, const uint8_t column_offset) {
    // Writing buffer to page_offset != 0: useful for double-buffering displays with height = 16, 32.
    bool success = true;
    uint8_t buf[6]; // Send 3 commands in one frame.
    buf[0] = SSD1306_CTRL_WORD_CONTAINS_COMMAND | SSD1306_CTRL_NEXT_WORD_STARTS_WITH_CONTROL ;
    buf[1] = SSD1306_SET_COLUMN_ADDR_LOWER_NIBBLE(column_offset) ;
    buf[2] = SSD1306_CTRL_WORD_CONTAINS_COMMAND | SSD1306_CTRL_NEXT_WORD_STARTS_WITH_CONTROL ;
    buf[3] = SSD1306_SET_COLUMN_ADDR_UPPER_NIBBLE(column_offset) ;
    buf[4] = SSD1306_CTRL_WORD_CONTAINS_COMMAND ;

    for(int i = 0; i < height >> 3; i++){
        buf[5] = SSD1306_SET_PAGE_START_ADDR(page_offset + i) ;
        success &= send_raw(buf, sizeof(buf));
        success &= send_data(&buffer[i * width], width);
    }
    return success;
}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::clearBuffer() {
    memset(buffer, 0, width * (height >> 3));
}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::drawPixel(int16_t x, int16_t y, int color) {
    /* screen  x
     *     _1__2__3____
     *   0| A0 B0 C0 ..   One byte (e.g. C) of GDDRAM image data maps to eight vertically consecutive pixels.
     *   1| A1 B1 C1 ..   SSD1306 organizes blocks of 128x8 pixels into "pages" of 128 bytes (page0 = {A,B,C,..}.).
     *   2| A2 B2 C2 ..   This library uses the same byte-oriented vertical pixel grouping, but serializes
     *   ~| .. .. .. ..   image data in 1-dim arrays. In the case of buffer[disp_width * (disp_height/8)] ,
     * y 7| A7 B7 C7 ..   the pixel at (x|y) is the (y mod 8)-th bit of the byte buffer[x + (y div 8) * disp_width] .
     *   8| U0 V0 W0 ..   Monochrome bitmap data used with SSD1306 functions can follow the same organization
     *   9| U1 V1 W1 ..   as a "native format" to avoid the transpose-8x8 problem:
     *  10| U2 V2 W2 ..   https://stackoverflow.com/questions/6930667/what-is-the-fastest-way-to-transpose-the-bits-in-an-8x8-block-on-bits
     *   ~| .. .. .. ..
     */
    if((x < disp_width) && (y < disp_height)) {   // prevent out-of-range access (just return, no wrap-around)
        uint8_t mask = 1 << (y & 0x07);           // bit mask is 1 << (y mod 8)
        uint16_t idx = x + (y >> 3) * disp_width; // calculate buffer array index
        // bitwise-and with the complementary mask (all 1's except bit to mask) selectively clears masked bit, while
        buffer[idx] = (buffer[idx] & (~mask)) | (color & mask); // or-ing conditionally sets bit again if color is 0xFF.
    }
}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::drawBitmap( int16_t src_x, int16_t src_y, int16_t dest_x, int16_t dest_y,
                                                   const uint8_t *bitmap, uint16_t bmp_width, uint16_t bmp_height,
                                                   SSD1306_color color, SSD1306_copy_mode mode ) {
    // TODO: Implement general-purpose blitting method.
    // TODO: Switch fonts to bitmap-based character ROM format so characters can be drawn using drawBitmap with SSD1306_copy_mode.
    // TODO: Re-implement writeChar() using drawBitmap().
}

template<size_t disp_width, size_t disp_height>
char SSD1306<disp_width, disp_height>::writeChar(char ch, FontDef Font, SSD1306_color color) {
    uint32_t i, b, j;

    // Check remaining space on current line
    if (width  <= (currentX + Font.FontWidth) ||
        height <= (currentY + Font.FontHeight))
    {
        // Not enough space on current line
        return 0;
    }

    if(ch < 0x20){
        ch = '?';
    }

    // Translate font to screenbuffer
    for (i = 0; i < Font.FontHeight; i++)
    {
        b = Font.data[(ch - 32) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++)
        {
            if ((b << j) & 0x8000)
            {
                drawPixel(currentX + j, (currentY + i), color);
            }
            else
            {
                drawPixel(currentX + j, (currentY + i), (SSD1306_color)(~color));
            }
        }
    }

    // The current space is now taken
    currentX += Font.FontWidth;

    // Return written char for validation
    return ch;
}

template<size_t disp_width, size_t disp_height>
char * SSD1306<disp_width, disp_height>::writeString(char *str, FontDef Font, SSD1306_color color) {
    // Write until null-byte
    while (*str)
    {
        if (writeChar(*str, Font, color) != *str)
        {
            // Char could not be written
            return str;
        }

        // Next char
        str++;
    }

    // Everything ok
    return str;
}

template<size_t disp_width, size_t disp_height>
void SSD1306<disp_width, disp_height>::setCursor(uint8_t x, uint8_t y) {
    currentX = x;
    currentY = y;
}


class [[maybe_unused]] SSD1306_64x48 : public SSD1306< 64, 48 > {
protected:
    bool init_panel_specifics() override {
        bool i = true;
        // Hardware config
        i &= send_command(SSD1306_SET_MULTIPLEX_RATIO, height - 1); // 48x multiplexing
        i &= send_command(SSD1306_SET_COM_PINS, 0x10 | 0x02); // sequential COM pin config, no L-R remap. 0x12 if height > 32 else 0x02?
        return i;
    }
};


class [[maybe_unused]] SSD1306_96x16 : public SSD1306< 96, 16 > {
protected:
    bool init_panel_specifics() override {
        bool i = true;
        // Hardware config
        i &= send_command(SSD1306_SET_MULTIPLEX_RATIO, height - 1); // 16x multiplexing
        i &= send_command(SSD1306_SET_COM_PINS, 0x00 | 0x02); // sequential COM pin config, no L-R remap. 0x12 if height > 32 else 0x02?
        return i;
    }
};


class [[maybe_unused]] SSD1306_128x32 : public SSD1306< 128, 32 > {
protected:
    bool init_panel_specifics() override {
        bool i = true;
        // Hardware config
        i &= send_command(SSD1306_SET_MULTIPLEX_RATIO, height - 1); // 32x multiplexing
        i &= send_command(SSD1306_SET_COM_PINS, 0x00 | 0x02); // sequential COM pin config, no L-R remap. 0x12 if height > 32 else 0x02?
        return i;
    }
};


class [[maybe_unused]] SSD1306_128x64 : public SSD1306< 128, 64 > {
protected:
    bool init_panel_specifics() override {
        bool i = true;
        // Hardware config
        i &= send_command(SSD1306_SET_MULTIPLEX_RATIO, height - 1); // 64x multiplexing
        i &= send_command(SSD1306_SET_COM_PINS, 0x10 | 0x02); // alternative COM pin config, no L-R remap. 0x12 if height > 32 else 0x02?
        return i;
    }
};

// for SH1106 configuration see https://github.com/greiman/SSD1306Ascii/blob/master/src/SSD1306init.h

#endif // HW_SSD1306_H
