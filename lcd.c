/*
 * lcd.c
 *
 *  Created on: 10-06-2014
 *      Author: dawid
 */

#include <avr/io.h>
#include <util/delay.h>
#include <ctype.h>
#include "lcd.h"

//positions
const position beg = {1,0};
const position end = {2,15};
const position line2 = {2,0};

//sets older/higher half of the given byte
static inline void lcd_send_half_byte(uint8_t byte);

//sends byte to lcd
static void lcd_send_byte(uint8_t byte);

//gives command do lcd
void lcd_send_cmd(uint8_t cmd);

//sends data(character) to lcd
void lcd_send_data(uint8_t data);

//return older/higher part of byte
static inline uint8_t lcd_read_half_byte();

//returns whole byte
static inline uint8_t lcd_read_byte();

//return non 0 value if busy flag is set
static inline int check_busy_flag();

static inline void lcd_send_half_byte(uint8_t byte)
{
	if(byte & 0x80)
		SET_D7;
	else
		CLR_D7;

	if(byte & 0x40)
		SET_D6;
	else
		CLR_D6;

	if(byte & 0x20)
		SET_D5;
	else
		CLR_D5;

	if(byte & 0x10)
		SET_D4;
	else
		CLR_D4;
}

static void lcd_send_byte(uint8_t byte)
{
	DATA_OUT;
	RW_OUT;

	//zapis informacji
	CLR_RW;

	//rozpoczęcie wysyłania
	E_OUT;
	SET_E;
	lcd_send_half_byte(byte); //starsza część
	CLR_E;

	SET_E;
	lcd_send_half_byte(byte<<4); //młodsza część
	CLR_E;

	while(check_busy_flag())
		continue;
}

static inline  uint8_t lcd_read_half_byte()
{
	uint8_t byte=0x00;
	if(COMMON_LCD_PIN & D7)
		byte |= 0x80;
	else
		byte &= ~0x80;

	if(COMMON_LCD_PIN & D6)
		byte |= 0x40;
	else
		byte &= ~0x40;

	if(COMMON_LCD_PIN & D5)
		byte |= 0x20;
	else
		byte &= ~0x20;

	if(COMMON_LCD_PIN & D4)
		byte |= 0x10;
	else
		byte &= ~0x10;

	return byte;
}

static inline uint8_t lcd_read_byte()
{
	uint8_t byte;

	DATA_IN;

	SET_RW;

	SET_E;
	byte = lcd_read_half_byte();
	CLR_E;

	SET_E;
	byte |= (lcd_read_half_byte() >> 4);
	CLR_E;

	return byte;
}

static inline int check_busy_flag()
{
	CLR_RS;
	SET_RW;
	DATA_IN;
	return lcd_read_byte() & (1<<7);
}

void lcd_send_cmd(uint8_t cmd)
{
	RS_OUT;
	RW_OUT;
	CLR_RS;
	CLR_RW;
	lcd_send_byte(cmd);
}

void lcd_send_data(uint8_t data)
{
	RS_OUT;
	RW_OUT;
	SET_RS;
	CLR_RW;
	lcd_send_byte(data);
}

void lcd_init()
{
	DATA_OUT;
	RS_OUT;
	RW_OUT;
	E_OUT;

	SET_RS;
	SET_RW;
	SET_E;

	_delay_ms(15);

	CLR_RS;
	CLR_RW;
	CLR_E;

	SET_E;
	lcd_send_half_byte(0x30);
	CLR_E;
	_delay_ms(4.1);

	SET_E;
	lcd_send_half_byte(0x30);
	CLR_E;
	_delay_us(100);

	SET_E;
	lcd_send_half_byte(0x30);
	CLR_E;
	_delay_us(100);

	SET_E;
	lcd_send_half_byte(0x20);
	CLR_E;
	_delay_us(100);

	//busy_flag in use <3 full speed on! <3
	lcd_send_cmd(FUNCTION|DBL_LINE);
	lcd_send_cmd(DISP_ON);
	lcd_send_cmd(DISP_ON|TURN_ON);
	lcd_send_cmd(ENTRY_MODE|MV_RIGHT);

	lcd_clr();
}

void lcd_clr()
{
	lcd_send_cmd(CLR_DISP);
}

void lcd_cursor_home()
{
	lcd_send_cmd(CURSOR_HOME);
}

void lcd_cursor_on()
{
	lcd_send_cmd(DISP_ON| TURN_ON | CURSOR_ON);
}

void lcd_cursor_and_blink_on()
{
	lcd_send_cmd(DISP_ON| TURN_ON | CURSOR_ON | CURSOR_BLINK);
}

void lcd_cursor_off()
{
	lcd_send_cmd(DISP_ON|TURN_ON);
}

void lcd_char_disp(char unsigned sign)
{
	lcd_send_data((uint8_t)sign);
}

void lcd_str_disp(const char string[])
{
	while(*string != '\0')
	{
		lcd_send_data(*string);
		string++;
	}
}

void lcd_str_disp_delay(const char string[])
{
	while(*string != '\0')
	{
		lcd_send_data(*string);
		string++;
		_delay_ms(100);
	}
}


void lcd_int_disp(int16_t number)
{
	uint16_t divider;
	uint16_t tmp;	//value of the oldest digit in number

	if(number < 0)
	{
		lcd_send_data('-');
		number -= 2*number;
	}

	_Bool previous_displayed=0;
	//10000 means that highest displayed number has 5 digits + sign
	for(divider = 10000;divider >0;divider/=10)
	{
		tmp = number/divider;

		if(tmp == 0 && previous_displayed==0)
			continue;

		previous_displayed=1;
		lcd_send_data('0'+tmp);
		number -= tmp*divider;
	}
}

void lcd_set_pos(uint8_t verse, uint8_t column)
{
	uint8_t position = 0x00;

	if(verse == 1)
		position = DDRAM_LINE1;
	else if(verse == 2)
		position = DDRAM_LINE2;

	position += column;

	lcd_send_cmd(DDRAM + position);
}

void lcd_set_pos2(position const *pos)
{
	uint8_t tmp = 0x00;

		if(pos->verse == 1)
			tmp = DDRAM_LINE1;
		else if(pos->verse == 2)
			tmp = DDRAM_LINE2;

		tmp += pos->column;

		lcd_send_cmd(DDRAM + tmp);
}

void lcd_port_disp(char port_number)
{
	uint8_t port_val;
	port_number=toupper(port_number);
	uint8_t flag;

	switch(port_number)
	{
	case 'A': port_val = PORTA;
				break;
	case 'B': port_val = PORTB;
				break;
	case 'C': port_val = PORTC;
				break;
	case 'D': port_val = PORTD;
				break;
	default : port_val = 0x00;
				break;
	}

	lcd_clr();

	lcd_set_pos2(&beg);
	lcd_str_disp("PORT");
	lcd_send_data(port_number);

	lcd_set_pos2(&line2);
	lcd_str_disp("0b");

	//displays bits from highest to lowest
	for(flag = 0x80; flag > 0; flag >>= 1 )
	{
		if(port_val & flag)
			lcd_send_data('1');
		else
			lcd_send_data('0');
	}

}

void lcd_pin_disp(char pin_number)
{
	uint8_t pin_val;
	pin_number=toupper(pin_number);
	uint8_t flag;

	switch(pin_number)
	{
	case 'A': pin_val = PINA;
				break;
	case 'B': pin_val = PINB;
				break;
	case 'C': pin_val = PINC;
				break;
	case 'D': pin_val = PIND;
				break;
	default : pin_val = 0x00;
				break;
	}

	lcd_set_pos2(&beg);
	lcd_str_disp("PIN");
	lcd_send_data(pin_number);

	lcd_set_pos2(&line2);
	lcd_str_disp("0b");

	//displays bits from highest to lowest
	for(flag = 0x80; flag > 0; flag >>= 1 )
	{
		if(pin_val & flag)
			lcd_send_data('1');
		else
			lcd_send_data('0');
	}

}

/* treść licencji:
 * Copyright (c) 2014, Dawid Pilarski
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


