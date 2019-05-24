
#define stepRound -722
#define degree (int32_t)(-90/0.498614)
#define speedStep   (uint32_t)5

#include <avr/io.h>
#include "Funct.h"

extern uint32_t mstime;

extern uint8_t workStepLeft, workStepRight;



volatile char rx_buffer[RX_BUFFER_SIZE];
unsigned char rx_wr_index,rx_rd_index,rx_counter; 
char read_enable = 0;
unsigned char statusReg = 0;
uint32_t dataAdc = 0;







int main(void)
{     				   
	port_init();
	UART_init();
	tim_1_init ();
	init_adc ();
	sei();
	//init_spi();
	//AD7799_INIT();	
	//AD7799_RESET();
	uint16_t speed = speedStep;
	int32_t right = degree;
	uint32_t timeDelay1 = 1500;
	uint32_t prevTime1 = 0, prevTime2 = 0;
	uint16_t adcValue = 0;
    while (1) 
    {  
	runStep(right, speed);
	
	if (millis() - prevTime1 >= timeDelay1) {
		 
		right = -right;
		prevTime1 = millis();
	}
	
	//if (millis() - prevTime1 >= timeDelay1) {
		//right = right + 100;
		//timeDelay1 = 800;
		//prevTime1 = millis();
	//}
	
	
		
	}
	
	
	
	
	
	     //statusReg = status_reg();
		 //if (!(statusReg & 0b10000000)){
			//dataAdc = ReadAd7799ConversionData();
			//PORTD ^= (1 << PORTD6);	 
		 //}
		 //sendNumber(dataAdc);
	    //numb = ((uint32_t*)rx_buffer)[0];	
	
	
	
	
	  
	
	
	
		
		
	
		
	
    
}

ISR (TIMER1_COMPA_vect){
	mstime++;
	//TCNT1 = 65521;          //0.001 мс
	//TCNT1 = 65479;       //1 сек
	
}

ISR (USART0_RX_vect){
	//PORTD |= (1 << PORTD6);
	char data;	
	data = UDR0;	
	rx_buffer[rx_wr_index++] = data;
		if (rx_wr_index == RX_BUFFER_SIZE) rx_wr_index = 0;			
			if (++rx_counter == RX_BUFFER_SIZE)
			{
				rx_counter = 0;				
			}
}

ISR (ADC_vect){
	adcValue = ADCL4;
	ADCSRA |= (1 << ADSC);
}




//for (int i = 0; i < 50; i++){
	//digitalWrite(2, LOW);
	//digitalWrite(3, LOW);
	//digitalWrite(4, LOW);
	//digitalWrite(5, HIGH);
	//delay(timedelay);
	//digitalWrite(2, LOW);
	//digitalWrite(3, HIGH);
	//digitalWrite(4, LOW);
	//digitalWrite(5, LOW);
	//delay(timedelay);
	//digitalWrite(2, LOW);
	//digitalWrite(3, LOW);
	//digitalWrite(4, HIGH);
	//digitalWrite(5, LOW);
	//delay(timedelay);
	//digitalWrite(2, HIGH);
	//digitalWrite(3, LOW);
	//digitalWrite(4, LOW);
	//digitalWrite(5, LOW);
	//delay(timedelay);
//}
////////////////////////////
//for (int k = 0; k < 50; k++){
	//digitalWrite(2, LOW);
	//digitalWrite(3, LOW);
	//digitalWrite(4, HIGH);
	//digitalWrite(5, LOW);
	//delay(timedelay);
	//digitalWrite(2, LOW);
	//digitalWrite(3, HIGH);
	//digitalWrite(4, LOW);
	//digitalWrite(5, LOW);
	//delay(timedelay);
	//digitalWrite(2, LOW);
	//digitalWrite(3, LOW);
	//digitalWrite(4, LOW);
	//digitalWrite(5, HIGH);
	//delay(timedelay);
	//digitalWrite(2, HIGH);
	//digitalWrite(3, LOW);
	//digitalWrite(4, LOW);
	//digitalWrite(5, LOW);
	//delay(timedelay);