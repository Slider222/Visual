#include "Ad7799.h"


unsigned char CONH=0x10; /*CONFIGURATION REGISTER[00,BO(1),U/B(1),0(0),G2(0),G1(0),G0(0),0,0,REF_DET(0),BUF(1),0(0),CH2(0),CH1(0),CH0(0)]*/
unsigned char CONL=0x00; /* GAIN: 1x, AIN1 */

void init_spi(void)
{
	//SPCR |= (1<<SPE)|(1<<MSTR);//|(1<<SPR0);//включаем SPI,объ€вл€ем ведущим,
	//SPCR |= (1<<CPOL)|(1<<CPHA);//режим SPI "3"
	//SPCR |= (1<<CPOL);//режим SPI "2"
	//SPCR |=(1<<CPHA);//режим SPI "1"
	//SPSR |= (1<<SPI2X);//удвоение скорости, коэфециент предделител€ "2"
	
	//SPCR = 0x5C; // setup SPI
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<CPOL)|(1<<CPHA);
	SPCR |=(1 << SPR1);
}




/* Write byte 2 AD7799 */
void WriteByteToAd7799(unsigned char data)
{
	
	ADC_CSACTIVE;
	
	
	
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	
	ADC_CSPASSIVE;
}

/* Read byte from AD7799 */
unsigned char ReadByteFromAd7799(void)
{
	unsigned char returnData;
	
	ADC_CSACTIVE;
	
	SPDR = 0x00;
	while((SPSR & (1<<SPIF))==0);
	returnData = SPDR;
	
	ADC_CSPASSIVE;
	
	return (returnData);
}

/* Reset AD7799 */
void AD7799_RESET(void){
	
	WriteByteToAd7799(0xff);
	WriteByteToAd7799(0xff);
	WriteByteToAd7799(0xff);
	WriteByteToAd7799(0xff);
	
}

/* Init AD7799 */
void AD7799_INIT(void)
{
	WriteByteToAd7799(0x10);   //b0001 0000
	/* Writes to Communications Register Setting Next Operation as Write to CONFIGURATION Register*/
	WriteByteToAd7799(CONH);   //B0011 0111
	WriteByteToAd7799(CONL);   //B0011 0000
	/*CONFIGURATION REGISTER[00,BO(0),U/B(0),0(0),G2(1),G1(1),G0(1),0,0,REF_DET(0),BUF(1),0(0),CH2(0),CH1(0),CH0(0)]*/
	
	WriteByteToAd7799(0x08);
	//b0000 1000
	/* Writes to Communications Register Setting Next Operation as Write to Mode Register*/
	WriteByteToAd7799(0x90);
	WriteByteToAd7799(0x0F);
	/* Writes to Mode Register Initiating Internal Zero-Scale Calibration*/
	//WaiteRDY();
	_delay_ms(5);
	/* Wait for RDY pin to go low to indicate end of calibration cycle*/
	WriteByteToAd7799(0x08);
	/* Writes to Communications Register Setting Next Operation as Write to  Mode Register*/
	WriteByteToAd7799(0xb0);
	WriteByteToAd7799(0x0F);
	/* Writes to Mode Register Initiating Internal Full-Scale Calibration*/
	//WaiteRDY();
	_delay_ms(5);
	/* Wait for RDY pin to go low to indicate end of calibration cycle*/
	WriteByteToAd7799(0x08);//b0000 1000
	/* Writes to Communications Register Setting Next Operation as Write to Mode Register*/
	WriteByteToAd7799(0x00);   //00010000
	WriteByteToAd7799(0x0F);   //00001111 (16.7HZ65dB)
	
	/* Mode Register[MD2(0),MD1(0),MD0(0),PSW(0),0(0),0(0),0(0),0(0),(0),(0),0(0),0(0),FS3(1),FS2(0),FS1(1),FS0(0)]*/
	/* Continuous-Conversion Mode. Fadc=16.7HZ;*/
}


/* Wait for READY from AD7799 */
void WaiteRDY(void)
{
	unsigned int iint ;
	iint=0 ;
	while(PINB & (1 << PINB4)){            //?????
		iint++;
		if(iint>65530)
		{
			//reset ad7799
			AD7799_RESET();
			AD7799_INIT();
			break;
		}
	}
}


/* Read 24-bit of data from AD7799 */   
unsigned long ReadAd7799ConversionData(void)      
{      
    unsigned long ConverData = 0 ; 
    WriteByteToAd7799(0x58);  //0101 1000 ???????????????????J?????000      
    /* Writes to Communications Register Setting Next Operation as Continuous Read From Data Register*/ 
        ConverData=ReadByteFromAd7799();      
        ConverData=ConverData<<8 ;      
        ConverData=ReadByteFromAd7799()+ConverData;      
        ConverData=ConverData<<8 ;      
        ConverData=ReadByteFromAd7799()+ConverData;
    return(ConverData);      
}  

unsigned char status_reg(void)//функци€ чтени€ регистра статуса
{
	volatile unsigned char h = 0;
	
	WriteByteToAd7799(0x40);      //следуща€ операци€ будет чтение пегистра данных	   
	h = ReadByteFromAd7799();
	return h;          //возвращаем значение регистра статуса
} 