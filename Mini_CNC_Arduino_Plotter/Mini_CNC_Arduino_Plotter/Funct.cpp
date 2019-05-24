#include "Funct.h"

void SPI_SendByte(unsigned long long byte){
	PORTA |= (1 << dataInput);
	PORTA |= (1 << clockInput);
	PORTA &= ~(1 << clockInput);
	
	for (int i = 0; i < 36; i++)
	{
		if ((byte & 0x01) == 0x00)
		{
			PORTA &= ~(1 << dataInput);
		}
		else {PORTA |= (1 << dataInput);}
		byte = byte >> 1;
		PORTA |= (1 << clockInput);
		PORTA &= ~(1 << clockInput);
	}
	PORTA |= (1 << dataEnable1);
	PORTB |= (1 << dataEnable2);
	
	
}


void sendNumber(signed int digit){
	volatile char slovo1=0, slovo2=0, slovo3=0, slovo4=0, slovo5=0, slovo6=0, minus = 0;
	volatile unsigned long long rezult1=0, rezult2 =0;
	static signed int data = 0;
	                         //0       1       2       3       4        5        6        7        8     9       _
	volatile char array[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x00, 0x40};
	data = labs(digit);
	if (digit < 0){
		minus = 11;
		} else {
		minus = 10;
	}
	
	if (data >= 100000){
		slovo1 = (data / 100000);           //0
		slovo2 = (data / 10000)%10;    //0
		slovo3 = (data / 1000)%10;     //1
		slovo4 = (data / 100)%10;      //2
		slovo5 = (data / 10)%10;      //3
		slovo6 = data%10;		     // 0
	}
	if (data < 100000 && data >= 10000){
		slovo1 = minus;
		slovo2 = (data / 10000)%10;    //0
		slovo3 = (data / 1000)%10;     //1
		slovo4 = (data / 100)%10;      //2
		slovo5 = (data / 10)%10;      //3
		slovo6 = data%10;		     // 0
	}
	if (data < 10000 && data >= 1000){
		slovo1 = 10;
		slovo2 = minus;
		slovo3 = (data / 1000)%10;    //1
		slovo4 = (data / 100)%10;      //2
		slovo5 = (data / 10)%10;      //3
		slovo6 = data%10;		     // 0
	}
	if (data < 1000 && data >= 100){
		slovo1 = 10;
		slovo2 = 10;
		slovo3 = minus;
		slovo4 = (data / 100)%10;     //2
		slovo5 = (data / 10)%10;      //3
		slovo6 = data%10;		     // 0
	}
	if (data < 100 && data >= 10){
		slovo1 = 10;
		slovo2 = 10;
		slovo3 = 10;
		slovo4	= minus;
		slovo5 = (data / 10)%10;      //3
		slovo6 = data%10;		     // 0
	}
	if (data < 10 && data >= 0){
		slovo1 = 10;
		slovo2 = 10;
		slovo3 = 10;
		slovo4 = 10;
		slovo5 = minus;
		slovo6 = data%10;		      // 0
	}
	
	rezult1 = (((rezult1 | array[slovo6])<<16)&0xff0000)|(((rezult1 | array[slovo5])<<8)&0xff00)|((rezult1 | array[slovo4])&0xff);
	rezult2 = (((rezult2 | array[slovo3])<<16)&0xff0000)|(((rezult2 | array[slovo2])<<8)&0xff00)|((rezult2 | array[slovo1])&0xff);
	PORTB &= ~(1 << dataEnable2);
	SPI_SendByte( rezult1);
	PORTA &= ~(1 << dataEnable1);
	SPI_SendByte( rezult2);
}

void port_init()
{
	DDRB |= (1 << PORTB0);                 //выходы
	PORTB |= (1 << dataEnable2);
	DDRA |= (1 << clockInput)|(1 << dataInput)|(1 << dataEnable1);
	PORTA |= (1 << dataEnable1);	
}