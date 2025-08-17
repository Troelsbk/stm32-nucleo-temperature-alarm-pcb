# include "oled_functions.h"
# include "ssd1306.h"
# include <string.h>

char send_charXY(char ch, uint8_t x, uint8_t y);
void send_strXY(char *str, uint8_t x, uint8_t y, uint8_t size);
void clear_oled(void);


// oled screen object
static SSD1306_t SSD1306;


void clear_oled(void){
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}




void send_strXY(char *str, uint8_t x, uint8_t y, uint8_t size){
	for(uint8_t index = 0; index < size; index++){
		send_charXY(str[index], index + x, y);
	}
	ssd1306_UpdateScreen();
}


char send_charXY(char ch, uint8_t x, uint8_t y){
	// internal helper function
	SSD1306_Font_t Font = Font_7x10;
	SSD1306_COLOR color = 1;
	uint32_t i, b, j;
	SSD1306.CurrentX = x * Font.width;
	SSD1306.CurrentY = y * Font.height;
	// Check if character is valid
	if (ch < 32 || ch > 126)
		return 0;
	// Char width is not equal to font width for proportional font
	const uint8_t char_width = Font.char_width ? Font.char_width[ch-32] : Font.width;
	// Check remaining space on current line
	if (SSD1306_WIDTH < (SSD1306.CurrentX + char_width) ||
		SSD1306_HEIGHT < (SSD1306.CurrentY + Font.height))
	{
		// Not enough space on current line
		return 0;
	}

	// Use the font to write
	for(i = 0; i < Font.height; i++) {
		b = Font.data[(ch - 32) * Font.height + i];
		for(j = 0; j < char_width; j++) {
			if((b << j) & 0x8000)  {
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
			} else {
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}
	return ch;
}

