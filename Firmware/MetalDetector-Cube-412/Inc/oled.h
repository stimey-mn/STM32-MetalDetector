/**
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 *
 * https://github.com/afiskon/stm32-ssd1306
 */

#ifndef __OLED_H__
#define __OLED_H__

#include <stdint.h>
#include <stddef.h>
#include "i2c1.h"

#define OLED_INCLUDE_GNU_UNIFONT

typedef struct {
	const uint8_t FontWidth;    /*!< Font width in pixels */
	const uint8_t FontHeight;   /*!< Font height in pixels */
	const uint16_t *data; /*!< Pointer to data font data array */
} FontDef;

extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

// OPTIMIZED FONTS - use 8 or 16 pixel tall glyphs - this allows byte write operations to the display memory
extern FontDef UniFont_8x16; // GNU UniFont 8x16 pixels

#ifndef OLED_I2C_ADDR
#define OLED_I2C_ADDR        (0x3C)
#endif

// OLED OLED height in pixels
#ifndef OLED_HEIGHT
#define OLED_HEIGHT          64
#endif

// OLED width in pixels
#ifndef OLED_WIDTH
#define OLED_WIDTH           132
#endif

// some LEDs don't display anything in first two columns
// #define OLED_WIDTH           130

// Enumeration for screen colors
typedef enum {
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} OLED_COLOR;

// Struct to store transformations
typedef struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Inverted;
    uint8_t Initialized;
} OLED_t;


// Procedure definitions
void OLED_Init(void);
void OLED_Fill(OLED_COLOR color);
void OLED_UpdateScreen(void);
void OLED_DrawPixel(uint8_t x, uint8_t y, OLED_COLOR color);
char OLED_WriteChar(char ch, FontDef Font, OLED_COLOR color);
char OLED_WriteString(char* str, FontDef Font, OLED_COLOR color);
void OLED_SetCursor(uint8_t x, uint8_t y);

// Low-level procedures
void OLED_Reset(void);
void OLED_WriteCommand(uint8_t byte);
void OLED_WriteData(uint8_t* buffer, size_t buff_size);

#endif // __OLED_H__
