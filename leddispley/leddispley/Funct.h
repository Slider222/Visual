#define	BLACK	  0x0000
#define	BLUE	  0x001F
#define	RED 	  0xF800
#define	GREEN     0x07E0
#define CYAN	  0x07FF
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0
#define WHITE	  0xFFFF


#define LCD_RES PORTC1
#define LCD_LED PORTC0
#define LCD_CS  PORTC2
#define LCD_RS  PORTC3
#define LCD_WR  PORTC4
#define LCD_RD  PORTC5

#define MOSI PORTB5
#define MISO PORTB6
#define SCK  PORTB7
#define SS   PORTB4

//#define dataEnable1 PORTA0
//#define dataEnable2 PORTB0
//#define dataInput   PORTA1
//#define clockInput  PORTA2


#define A1 (1 << PORTD4)  //A
#define A2 (1 << PORTD5)  //B
#define B1 (1 << PORTD6)  //C
#define B2 (1 << PORTD7)  //D




#define Hi(Int) (char) ((Int)>>8)
#define Low(Int) (char) (Int)



#define DATA_REGISTER_EMPTY (1<<UDRE0)
#define RX_COMPLETE			(1<<RXC0)
#define FRAMING_ERROR		(1<<FE0)
#define PARITY_ERROR		(1<<UPE0)
#define DATA_OVERRUN		(1<<DOR0)
#define RX_BUFFER_SIZE      4








#define F_CPU 16000000UL

#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <util/atomic.h>

#include "Ad7799.h"









#ifndef FUNCT_H_
#define FUNCT_H_




uint64_t millis ();









#endif /* FUNCT_H_ */








