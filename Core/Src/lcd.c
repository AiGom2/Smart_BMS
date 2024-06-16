#include "stm32f4xx.h"
#include "lcd.h"

#define TLCD_RS  GPIO_PIN_0
#define TLCD_RW  GPIO_PIN_1
#define TLCD_E   GPIO_PIN_2
#define TLCD_EN { HAL_GPIO_WritePin(GPIOB, TLCD_E, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, TLCD_E, GPIO_PIN_SET); }
#define DATA     GPIOA
#define DATAPIN 5


static void Delay(const uint32_t Count)
{
  __IO uint32_t index = 0; 
  for(index = (16800 * Count); index != 0; index--);
}

static void Delay_us(const uint32_t Count)
{
  __IO uint32_t index = 0; 
  for(index = (16 * Count); index != 0; index--);
}

void E_Pulse(void)
{
  HAL_GPIO_WritePin(GPIOB, TLCD_E, GPIO_PIN_SET);
  Delay(5);
  HAL_GPIO_WritePin(GPIOB, TLCD_E, GPIO_PIN_RESET);
}

void TLCD_DATA(unsigned char data)
{
  GPIOA->ODR = (data << DATAPIN);
  TLCD_EN;
} 

void Func_Set(void)
{
  HAL_GPIO_WritePin(GPIOB, TLCD_RW, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, TLCD_RS, GPIO_PIN_RESET);
  GPIOA->ODR = (0x38 << DATAPIN);
  E_Pulse();                            // Enable Pulse
}


void LCD_init(void)
{
  HAL_GPIO_WritePin(GPIOB, TLCD_E, GPIO_PIN_RESET);
  Delay(15);
  Func_Set();
  Delay(10);
  Func_Set();
  Delay_us(150);
  Func_Set();
  GPIOA->ODR = (0x0f << DATAPIN);
  E_Pulse();
  GPIOA->ODR = (0x06 << DATAPIN);
  E_Pulse();
  GPIOA->ODR = (0x01 << DATAPIN);
  E_Pulse();
}
  

void lcd_char(char s)
{
  HAL_GPIO_WritePin(GPIOB, TLCD_RS, GPIO_PIN_SET);
  GPIOA->ODR = (s << DATAPIN);
  E_Pulse();
}

void lcd_disp(char x, char y)
{
  HAL_GPIO_WritePin(GPIOB, TLCD_RS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, TLCD_RW, GPIO_PIN_RESET);
  if(y==0) GPIOA->ODR = ((x + 0x80) << DATAPIN);
  else if(y==1) GPIOA->ODR = ((x + 0xC0) << DATAPIN);
  E_Pulse();
 }
 
void move_disp(char p )
{
  HAL_GPIO_WritePin(GPIOB, TLCD_RS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, TLCD_RW, GPIO_PIN_RESET);

  if(p==LEFT ) GPIOA->ODR = (0x18 << DATAPIN);
  else if(p==RIGHT) GPIOA->ODR = (0x1c << DATAPIN);

  E_Pulse();
}

void disp_ON_OFF(char d, char c, char b)
{
  char disp = 0x08;


  HAL_GPIO_WritePin(GPIOB, TLCD_RS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, TLCD_RW, GPIO_PIN_RESET);

  if (d==ON) d = 0x04;
  else         d = 0x00;

  if(c == ON) c = 0x02;
  else          c = 0x00;

  if(b == ON) b = 0x01;
  else          b = 0x00;

  GPIOA->ODR = ((disp | d | c| b) << DATAPIN);
  E_Pulse();
            
}   
 
void clrscr(void)
{
  HAL_GPIO_WritePin(GPIOB, TLCD_RS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, TLCD_RW, GPIO_PIN_RESET);
  GPIOA->ODR = (0x01 << DATAPIN);
  E_Pulse();

  Delay(10);
} 



void lcd(char x, char y, char *str)
{
  lcd_disp(x, y);
  while(*str) lcd_char(*str++);
}
