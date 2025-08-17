#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "ssd1306_fonts.h"

void send_strXY(char *str, uint8_t x, uint8_t y, uint8_t size);
char send_charXY(char ch, uint8_t x, uint8_t y);
void clear_oled(void);
