/*
 * lcd.h
 *
 *  Created on: 11-06-2014
 *      Author: dawid
 */

#ifndef LCD_H_
#define LCD_H_

/* Biblioteka działa tylko z wykorzystanym RW i dla wyświetlaczy zgodnych ze standardem HD44780
 * o rozmiarze 2x16. Inne wyświetlacze nie są obsługiwane przez bibliotekę. Wszystkie wyprowadzenia
 * muszą być podłączone do tego samego portu wyprowadzeń. Obsługuje jedynie pamięć RAM. Brak obsługi np. EEPROM.
 * Autor: Dawid Pilarski. Maksymalnie wyświetla 5cyfrową liczbę + znak
 */

//commands
enum {CLR_DISP = 0x01};
enum {CURSOR_HOME = 0x02};
enum {ENTRY_MODE= 0x04, MV_RIGHT= 0x02, MV_WINDOW= 0x01};
enum {DISP_ON= 0x08, TURN_ON=0x04, CURSOR_ON= 0x02, CURSOR_BLINK= 0x01};
enum {SHIFT=0x10, SH_MV_WINDOW= 0x08, SH_MV_RIGHT= 0x04};
enum {FUNCTION= 0x20, INT8BIT=0x10, DBL_LINE=0x08, SIZE5x10= 0x04};
enum {CGRAM=0x40};
enum {DDRAM=0x80};

//connections
#define COMMON_LCD_PORT PORTA
#define COMMON_LCD_PIN	PINA
#define COMMON_LCD_DDR	DDRA

#define DATA_PORT_BEGIN PA3

#define RS	(1<<PA0)
#define RW	(1<<PA1)
#define E	(1<<PA2)

#define D4	(1<<PA3)
#define D5	(1<<PA4)
#define D6	(1<<PA5)
#define D7	(1<<PA6)

//basic commands
#define DATA_IN		(COMMON_LCD_DDR &= ~(D4 | D5 | D6 | D7))
#define DATA_OUT	(COMMON_LCD_DDR |= (D4 | D5 | D6 | D7))
#define RS_OUT		(COMMON_LCD_DDR |= RS)
#define RW_OUT		(COMMON_LCD_DDR |= RW)
#define E_OUT		(COMMON_LCD_DDR |= E)

//masks
#define COMMON_LCD_MASK	0x7F
#define DATA_MASK 0x78

//set/clr pins
#define SET_RS		(COMMON_LCD_PORT |= RS)
#define SET_RW		(COMMON_LCD_PORT |= RW)
#define SET_E		(COMMON_LCD_PORT |= E)
#define SET_D4		(COMMON_LCD_PORT |= D4)
#define SET_D5		(COMMON_LCD_PORT |= D5)
#define SET_D6		(COMMON_LCD_PORT |= D6)
#define SET_D7		(COMMON_LCD_PORT |= D7)

#define CLR_RS		(COMMON_LCD_PORT &= ~RS)
#define CLR_RW		(COMMON_LCD_PORT &= ~RW)
#define CLR_E		(COMMON_LCD_PORT &= ~E)
#define CLR_D4		(COMMON_LCD_PORT &= ~D4)
#define CLR_D5		(COMMON_LCD_PORT &= ~D5)
#define CLR_D6		(COMMON_LCD_PORT &= ~D6)
#define CLR_D7		(COMMON_LCD_PORT &= ~D7)

//check if set
#define IS_E_SET	(COMMON_LCD_PIN & E)
#define IS_E_CLR	(!(COMMON_LCD_PIN & E))

//DDRAM addresses for 2x16 lcd
#define DDRAM_LINE1 0x00
#define DDRAM_LINE2 0x40


typedef struct
{
	uint8_t verse;
	uint8_t column;	//column is offset! start from 0!
}position;

//positions to use with lcd_set_pos2 function
extern const position beg,end,line2;

//interface
void lcd_init();						//inits lcd screen, so it can be used
void lcd_clr();							//clears lcd display
void lcd_cursor_on();					//makes cursor visible
void lcd_cursor_and_blink_on();			//-||- and makes it blink
void lcd_cursor_off();					//makes cursor invisible
void lcd_cursor_home();					//moves cursor home
void lcd_str_disp(char string[]);		//displays string
void lcd_str_disp_delay(char string[]);	//displays string with delay between every sign (75ms)
void lcd_int_disp(int16_t number);		//displays number (max 5 digits + sign)
void lcd_char_disp(unsigned char);				//displays character
void lcd_set_pos(uint8_t verse, uint8_t column); //column is offset so start at 0!
void lcd_set_pos2(position const *);			//as higher but uses ptr for structure position

//functions helpful for debugging

//takes long time(especially with cheap LCDs), so port must be stable in a time, when executed
void lcd_port_disp(char port_name);	//where argument can be 'A' 'B' 'C' 'D'
void lcd_pin_disp(char pin_name);	//not tested!!


#endif /* LCD_H_ */
