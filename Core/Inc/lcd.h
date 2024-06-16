#ifndef __LCD_H__
#define __LCD_H__


static void Delay(const uint32_t Count);
static void Delay_us(const uint32_t Count);
void LCD_init(void);
void E_Pulse(void);
void TLCD_DATA(unsigned char data);
void Func_Set(void);
void lcd_char(char s);
void lcd_disp(char x, char y);
void move_disp(char p );
void disp_ON_OFF(char d, char c, char b);
void clrscr(void);
void lcd(char x, char y, char *str);

#define ON      1
#define OFF     2

#define RIGHT   1
#define LEFT    2
#endif
