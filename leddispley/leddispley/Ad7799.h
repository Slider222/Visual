/*
 * Ad7799.h
 *
 * Created: 22.04.2019 8:04:33
 *  Author: Serg
 */ 


#ifndef AD7799_H_
#define AD7799_H_


#define ADC_VREF_TYPE 0xC0

#define ADC_CSACTIVE  PORTD &= ~(1<<PORTD5); /* CS=0 */
#define ADC_CSPASSIVE PORTD |= (1<<PORTD5);  /* CS=1 */

#include "Funct.h"

void WriteByteToAd7799(unsigned char data);
unsigned char ReadByteFromAd7799(void);
void AD7799_RESET(void);
void AD7799_INIT(void);
void WaiteRDY(void);
unsigned long ReadAd7799ConversionData(void);
unsigned char status_reg(void);
void init_spi(void);


#endif /* AD7799_H_ */