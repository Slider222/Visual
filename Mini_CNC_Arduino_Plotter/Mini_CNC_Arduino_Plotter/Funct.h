/*
 * Funct.h
 *
 * Created: 16.05.2019 14:54:07
 *  Author: Serg
 */ 


#ifndef FUNCT_H_
#define FUNCT_H_

#define dataEnable1 PA0
#define dataEnable2 PORTB0
#define dataInput   PA1
#define clockInput  PA2

#include <Arduino.h>


void SPI_SendByte(unsigned long long byte);
void sendNumber(signed int digit);
void port_init();
#endif /* FUNCT_H_ */