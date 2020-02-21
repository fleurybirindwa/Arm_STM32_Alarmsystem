/*
 * lcd.c
 *
 *  Created on: 3 maj 2017
 *      Author: suan1696
 */

#include "lcd.h"

void TextLCD_Strobe(TextLCDType *lcd);
void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd);
void TextLCD_Data(TextLCDType *lcd, uint8_t data);
char *convert(unsigned int num, int base);



void TextLCD_Strobe(TextLCDType *lcd)

{
lcd->controlPort->ODR &= ~(lcd->strbPin);
HAL_Delay(1);
lcd->controlPort->ODR |= lcd->strbPin;
HAL_Delay(1);

}

void TextLCD_Cmd(TextLCDType *lcd, uint8_t cmd) {
lcd->controlPort->ODR &= ~(lcd->rsPin);
lcd->controlPort->ODR &= ~lcd->rwPin;
lcd->dataPort->ODR = cmd;
TextLCD_Strobe(lcd);
delay_us(45);
}

void TextLCD_Data(TextLCDType *lcd, uint8_t data) {
lcd->controlPort->ODR |= lcd->rsPin;
lcd->controlPort->ODR &= ~lcd->rwPin;
lcd->dataPort->ODR = data;
TextLCD_Strobe(lcd);
delay_us(45);
}

void TextLCD_Init(TextLCDType *lcd, GPIO_TypeDef *controlPort, uint16_t rsPin,
	uint16_t rwPin, uint16_t enPin, GPIO_TypeDef *dataPort, uint16_t dataPins) {
lcd->dataPort = dataPort;
lcd->controlPort = controlPort;
lcd->rwPin = rwPin;
lcd->strbPin = enPin;
lcd->rsPin = rsPin;

lcd->controlPort->ODR &= ~(lcd->rsPin);

HAL_Delay(15);

TextLCD_Cmd(lcd, 0x38);
HAL_Delay(5);

TextLCD_Cmd(lcd, 0x38);
delay_us(100);

TextLCD_Cmd(lcd, 0x38);
//	delay_us(40);

TextLCD_Cmd(lcd, 0x38);
//delay_us(40);

TextLCD_Cmd(lcd, 0x06);
//	delay_us(40);

TextLCD_Cmd(lcd, 0x0E);
//	delay_us(40);

TextLCD_Cmd(lcd, 0x01);

HAL_Delay(2);

//TextLCD_Cmd(lcd, 0x80);
//delay_us(40);

}

void TextLCD_Home(TextLCDType *lcd) {

	lcd->controlPort->ODR &= ~(lcd->rsPin);
			lcd->controlPort->ODR &= ~lcd->rwPin;
			lcd->dataPort->ODR = 0x02;
			TextLCD_Strobe(lcd);
			delay_us(45);

}

void TextLCD_Clear(TextLCDType *lcd) {
	lcd->controlPort->ODR &= ~(lcd->rsPin);
			lcd->controlPort->ODR &= ~lcd->rwPin;
			lcd->dataPort->ODR = 0x01;
			TextLCD_Strobe(lcd);
			HAL_Delay(2);
}

void TextLCD_Position(TextLCDType *lcd, int x, int y)


 {

	if (x == 1)

	TextLCD_Cmd(lcd, 0x80 + y);

	if (x == 2)

	TextLCD_Cmd(lcd, 0xC0 + y);

}

void TextLCD_Putchar(TextLCDType *lcd, uint8_t data)
{
	TextLCD_Data(lcd,data);
}

void TextLCD_Puts(TextLCDType*lcd, char*string){

	while(* string !='\0')
	{
	 TextLCD_Data(lcd, *string);
	 string++;

	}
}
void TextLCD_Printf(TextLCDType *lcd, char *format, ...);
/*char *traverse;
unsigned int i;
char *s;
va_list arg;
va_start(arg, format);
for (traverse = format; *traverse != '\0'; traverse++) {
	while (*traverse != '%') {
		TextLCD_Putchar(lcd, *traverse);
		traverse++;
	}
	traverse++;

	switch(*traverse){
	case 'c' : i = va_arg(arg,int);

	TextLCD_Putchar(lcd, i);
	break;

	case 'd':
i = va_arg(arg,int);
if(i<0) {
	i=-i;
TextLCD_Putchar(lcd, '-');
}
TextLCD_Puts(lcd, convert(i,10));
	break;


	case 'o' : i = va_arg(arg,unsigned int);
	TextLCD_Puts(lcd, convert(i,8));
	break;

	case 's' : s = va_arg(arg,char*);
		TextLCD_Puts(lcd,s);
		break;

	case 'x' : i = va_arg(arg,unsigned int);
		TextLCD_Puts(lcd, convert(i,16));
		break;
}
}
va_end(arg);

}
char *convert(unsigned int num, int base) {
	static char representation[] = "0123456789ABCDEF";
	static char buffer[50];
	char *ptr;
	ptr = &buffer[49];
	*ptr = '\0';

	do {
		*--ptr = representation[num%base];
		num /= base;
	} while (num != 0);
	return (ptr);
}*/
