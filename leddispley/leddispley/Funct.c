#include "Funct.h"
#include "NewFont.h"
//5555

uint32_t mstime = 0;
uint8_t workStepLeft, workStepRight = 0;
 





void port_init()
{
	DDRB |= (1 << PORTB0)|(1 << PORTB1)|(1 << SS)|(1 << SCK)|(1 << MOSI);  //выходы
	PORTB |= (1 << dataEnable2);
	DDRA |= (1 << clockInput)|(1 << dataInput)|(1 << dataEnable1);
	PORTA |= (1 << dataEnable1);
    DDRD |= (1 << PORTD4)|(1 << PORTD6)|(1 << PORTD1)|(1 << PORTD5)|(1 << PORTD7);
	//PORTD |= (1 << PORTD5);
	
	
	
	//DDRD &= ~(1 << PORTD2);
	//PORTD &= ~(1<< PORTD2);
	//DDRB |= (1 << PORTB5);
}



void SPI_SendByte(uint64_t byte){	   	  
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

		
void sendNumber(int32_t digit){
		 volatile uint8_t slovo1=0, slovo2=0, slovo3=0, slovo4=0, slovo5=0, slovo6=0, minus = 0;
		 volatile uint64_t rezult1=0, rezult2 =0;
		 static int32_t data = 0;
	                                 //0    1      2     3     4     5     6     7     8     9    _
	     volatile uint8_t array[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x00, 0x40};	                          
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


void sendString(char* string){
         volatile unsigned char slovo[6] = {0}; 
          volatile uint64_t rezult1 = 0, rezult2 = 0;
                                     //_     a	  b		 c	  d		 e	  f		g     1     2     3     4     5     6     7     8    9      0
         volatile uint8_t array[] = {0x3f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x3d, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x00 };
      	for (int i = 0; i < 6; i++){
			 if (string[i] >= 96){
		     slovo[i] = ((string[i]) - 96); 
			 }
			 else if (string[i] >= 48){
				 if (string[i] == 48){
					 slovo[i] = 0;
				 } else {
					 slovo[i] = ((string[i]) - 41); 
				 }				
			 } else {
				if (string[i] == 0){
				    slovo[i] = ((string[i]) + 17);
				}else {
					slovo[i] = ((string[i]) + 7);
				}
				 
			 }
		}
        rezult1 = (((rezult1 | array[slovo[5]])<<16)&0xff0000)|(((rezult1 | array[slovo[4]])<<8)&0xff00)|((rezult1 | array[slovo[3]])&0xff);
        rezult2 = (((rezult2 | array[slovo[2]])<<16)&0xff0000)|(((rezult2 | array[slovo[1]])<<8)&0xff00)|((rezult2 | array[slovo[0]])&0xff);
		 PORTB &= ~(1 << dataEnable2);
		 SPI_SendByte( rezult1);
		 PORTA &= ~(1 << dataEnable1);
		 SPI_SendByte( rezult2);
         ICR1 = 49;
		}








void runLine(char* string){
       //char  bukva[] = {0};   
       while (*string){         
           sendString(string);
           string++;
           _delay_ms(300);				
       }
           
			//for (int j = 0; j < lenstr; j++){
				//sendString(string);
			//}
		}
	 
void clearLed(){
	PORTB &= ~(1 << dataEnable2);
   	SPI_SendByte(0x00000000);
	PORTA &= ~(1 << dataEnable1);
	SPI_SendByte(0x00000000);
}
	
	
	



									  





unsigned char SPI_ReceiveByte (void)
{
	//PORTB |= (1 << MISO)|(1 << SS);
	//PORTB &= ~(1 << SCK);
	unsigned char i, result=0;
	for (i = 0; i < 8; i ++)
	{
		PORTB|=(1 << SCK);                         //фронт на лапке SCK
		result = result << 1;
		if((PINB & (1 << MISO)) !=0x00)           //запишем новый бит в младший разряд
		result=result|0x01;                       //запишем считанный с лапки порта MISO бит
		PORTB&=~(1<<SCK);
		//asm("nop");
	}
	return result;
}


unsigned char SD_cmd (char dt0,char dt1,char dt2,char dt3,char dt4,char dt5)
{
	unsigned char result;
	long int cnt;
	SPI_SendByte(dt0); //индекс
	SPI_SendByte(dt1); //Аргумент
	SPI_SendByte(dt2);
	SPI_SendByte(dt3);
	SPI_SendByte(dt4);
	SPI_SendByte(dt5); //контрольная сумма
	cnt = 0;
	do
	{                                                      //Ждём ответ в формате R1 (даташит стр 109)
		result = SPI_ReceiveByte();
		cnt++;
	}
	while (((result&0x80)!=0x00)&& cnt < 0xFFFF);
	return result;
}


unsigned char SD_Init(void)
{
	
	volatile unsigned char i,temp;
	long int cnt;
	for(i=0; i<10; i++)
	{                               //80 импульсов (не менее 74)
		SPI_SendByte(0xFF);
	}
	
	PORTB&=~(1<<SS);                               //опускаем SS
	
	temp=SD_cmd(0x40,0x00,0x00,0x00,0x00,0x95);    //CMD0
	
	if(temp!=0x01) return 1;                       //Выйти если ответ не 0x01
	//asm("nop");
	SPI_SendByte(0xFF);
	
	cnt=0;
	do
	{
		temp=SD_cmd(0x41,0x00,0x00,0x00,0x00,0x95); //CMD8 передаем также, меняется только индекс
		SPI_SendByte(0xFF);
		cnt++;
	}
	while ((temp!=0x00) && cnt<0xFFFF);       //Ждём ответа R1
	if(cnt >= 0xFFFF)  return 2;
	
	return 0;
}






void write_command (unsigned char command)
{
	
	PORTB = (0x00 >> 6) & 0x03;
	PORTD = (0x00<<2)&(0xFC);                   //выставили старшие биты
	
	
	PORTC &= ~(1 << LCD_RS);          //команда
	PORTC &= ~(1 << LCD_CS);          //Выбор чипа
	PORTC &= ~(1 << LCD_WR);          //запись
	//_delay_ms(1);                     //длительность строба
	PORTC |=  (1 << LCD_WR);          //сброс записи
	
	PORTB = (command >> 6) & 0x03;
	PORTD = (command<<2) & (0xFC);         //младшие биты
	
	PORTC &=  ~(1 << LCD_WR);        //запись
	//_delay_ms(1);                    //длительность строба
	PORTC |=  (1 << LCD_WR);         //сброс записи
	PORTC |=  (1 << LCD_CS);         //Выбор чипа
	PORTC |=  (1 << LCD_RS);         //данные
	//PORTC &= ~(1 << 0x00);
	//PORTD &= ~(1 << 0x00);
	
}




void write_data (unsigned int data)
{
	
	PORTB = (data >> 14) & 0x03;
	PORTD = (data >> 6)&(0xFC);              //старший байт данных
	
	
	PORTC |=  (1<<LCD_RS);                         //данные
	PORTC &= ~(1<<LCD_CS);                         //выбор чипа
	PORTC &= ~(1<<LCD_WR);                         //запись
	//_delay_ms(1);                                  //длительность строба
	PORTC |=  (1<<LCD_WR);                         //сброс записи
	
	PORTB = (data >> 6) & 0x03;
	PORTD = (data<<2)&(0xFC);                      //выставили младший байт данных
	
	PORTC &= ~(1<<LCD_WR);
	//_delay_ms(5);
	PORTC |=  (1<<LCD_WR);
	PORTC |=  (1<<LCD_CS);
	PORTC |=  (1<<LCD_RS);
	//PORTC &= ~(1 << 0x00);
	//PORTD &= ~(1 << 0x00);
	
}


unsigned int read_data ()
{
	PORTD = 0x00;
	unsigned int value;
	PORTB |=  (1<<LCD_RS);                         //данные
	PORTB &= ~(1<<LCD_CS);                         //выбор чипа
	PORTB &= ~(1<<LCD_RD);                         //чтение
	_delay_ms(5);
	value = PORTD;
	PORTB |= (1<<LCD_RD);                         //сброс чтения
	value <<= 8;
	PORTB &= ~(1<<LCD_RD);                        //чтение
	_delay_ms(5);
	value += PORTD;
	PORTB |= (1<<LCD_RD);
	PORTB |=  (1<<LCD_CS);
	PORTB |=  (1<<LCD_RS);
	PORTD = 0xFF;
	return value;
}


void Display_Home ()
{
	write_command(0x20);        //устанавливаем счётчик адреса по Х
	write_data(0x0000);         //в начало, т.е. ноль
	write_command(0x21);        //устанавливаем счётчик адреса по У
	write_data(0x0000);         //в начало, т.е. ноль

	write_command(0x46);        // конечный и начальный адрес по горизонтали x
	write_data(0xef00);         // 239 и 0 EF00
	write_command(0x47);        //конечный адрес по вериткали y
	write_data(0x013f);         // 319 013F
	write_command(0x48);        //начальный адрес по вериткали y
	write_data(0x0000);         // 0

	write_command(0x22);
}


void display_rgb (unsigned int data)
{
	unsigned int i,j;
	
	Display_Home();
	for(i=0;i<320;i++)
	{
		for(j=0;j<240;j++)
		{
			write_data(data);
		}
	}
}


void init_TFT (void)
{
	PORTC &= ~(1 << LCD_RES);
	_delay_ms(200);
	PORTC |=  (1 << LCD_RES);
	_delay_ms(200);
	
	PORTC &= ~(1 << LCD_RES);
	_delay_ms(200);
	PORTC |=  (1 << LCD_RES);
	_delay_ms(200);
	
	
	
	write_command(0x11);    //0x11
	write_data(0x2004);     //2004
	
	write_command(0x12);
	write_data(0xCC00);
	
	write_command(0x15);
	write_data(0x2600);
	
	write_command(0x14);
	write_data(0x252A);
	
	write_command(0x12);
	write_data(0x0033);
	
	write_command(0x13);
	write_data(0xCC44);
	
	//_delay_ms(5);
	
	write_command(0x13);
	write_data(0xCC06);
	
	//_delay_ms(5);
	
	write_command(0x13);
	write_data(0xCC4F);
	
	//_delay_ms(5);
	
	write_command(0x13);
	write_data(0x674F);
	
	write_command(0x11);
	write_data(0x2003);
	
	//_delay_ms(5);
	
	write_command(0x30);
	write_data(0x2609);
	
	write_command(0x31);
	write_data(0x242C);

	write_command(0x32);
	write_data(0x1F23);
	
	write_command(0x33);
	write_data(0x2425);
	
	write_command(0x34);
	write_data(0x2226);
	
	write_command(0x35);
	write_data(0x2523);
	
	write_command(0x36);
	write_data(0x1C1A);
	
	write_command(0x37);
	write_data(0x131D);
	
	write_command(0x38);
	write_data(0x0B11);
	
	write_command(0x39);
	write_data(0x1210);
	
	write_command(0x3A);
	write_data(0x1315);
	
	write_command(0x3B);
	write_data(0x3619);
	
	write_command(0x3C);
	write_data(0x0D00);
	
	write_command(0x3D);
	write_data(0x000D);
	
	write_command(0x16);
	write_data(0x0007);
	
	write_command(0x02);
	write_data(0x0013);
	
	write_command(0x03);           //режим
	write_data(0x0001);
	
	write_command(0x01);
	write_data(0x0127);
	
	write_command(0x08);
	write_data(0x0303);
	
	write_command(0x0A);
	write_data(0x000B);
	
	write_command(0x0B);
	write_data(0x0003);
	
	write_command(0x0C);
	write_data(0x0000);
	
	write_command(0x4C);
	write_data(0x0000);
	
	write_command(0x50);
	write_data(0x0000);
	
	write_command(0x60);
	write_data(0x0005);
	
	write_command(0x70);
	write_data(0x000B);
	
	write_command(0x71);
	write_data(0x0000);
	
	write_command(0x78);
	write_data(0x0000);
	
	write_command(0x7A);
	write_data(0x0000);

	write_command(0x79);
	write_data(0x0007);
	
	write_command(0x07);
	write_data(0x0051);
	
	//_delay_ms(1);
	
	write_command(0x07);
	write_data(0x0053);
	
	write_command(0x79);
	write_data(0x0000);
	
	write_command(0x22);
	
	
	
}


void point (unsigned char size, unsigned int x, unsigned int y, unsigned int color)
{
	unsigned char j,i;

	for(j=0;j < size; j++)
	{
		write_command(0x20);
		write_data(x);
		write_command(0x21);
		write_data(y+j);
		write_command(0x22);
		for(i=0;i < size;i++) write_data(color);
	}
}



void circle (unsigned char size,int x0,int y0,int radius,unsigned int color)
{
	int x = 0;                                        //работает по алгоритму Брезенхема (см. Википедию)
	int y = radius;
	int delta = 2 - 2 * radius;
	int error = 0;
	while(y >= 0)
	{
		point(size,x0 + x, y0 + y,color);
		point(size,x0 + x, y0 - y,color);
		point(size,x0 - x, y0 + y,color);
		point(size,x0 - x, y0 - y,color);
		error = 2 * (delta + y) - 1;
		if(delta < 0 && error <= 0)
		{
			++x;
			delta += 2 * x + 1;
			continue;
		}
		error = 2 * (delta - x) - 1;
		if(delta > 0 && error > 0)
		{
			--y;
			delta += 1 - 2 * y;
			continue;
		}
		++x;
		delta += 2 * (x - y);
		--y;
	}
}


void drawBitmap(unsigned int x, unsigned int y, const unsigned int *array)
{
	unsigned int i, j, f;
	f = 0;
	for(j=0; j < 30; j++)       //размер по вертикали(высота рисунка)
	{
		write_command(0x0020);
		write_data(x);
		write_command(0x0021);
		write_data(y - j);
		for(i=0; i < 50; i++ )                     //размер по горизонтали(ширина рисунка)
		{
			write_command(0x0022);
			write_data(pgm_read_word(&array[i + f]));
			
		}
		f = f + 50;
	}
}


void drawBitmap_SD(unsigned int x, unsigned int y, uint16_t* array)
{
	volatile unsigned int i, j, f;
	f = 0;
	for(j = 0; j < 320; j++)       //размер по вертикали(высота рисунка)
	{
		write_command(0x0020);
		write_data(x);
		write_command(0x0021);
		write_data(y - j);		
		for(i = 240; i > 0; i--)                     //размер по горизонтали(ширина рисунка)
		{
			write_command(0x0022);
			write_data(array[f]);	
			f++;
		}
		
	}
	
}

void set_cursor(unsigned int x, unsigned int y)
{
	write_command(0x0020);
	write_data(x);
	write_command(0x0021);
	write_data(y);
	write_command(0x0022);
}     


void line (unsigned char size, int x1, int y1, int x2, int y2, unsigned int color)
{
	int deltaX = abs(x2 - x1);                   //работает по алгоритму Брезенхема
	int deltaY = abs(y2 - y1);
	int signX = x1 < x2 ? 1 : -1;
	int signY = y1 < y2 ? 1 : -1;
	int error = deltaX - deltaY;
	
	for (;;)
	{
		point (size,x1,y1,color);
		
		if(x1 == x2 && y1 == y2)
		break;
		
		int error2 = error * 2;
		
		if(error2 > -deltaY)
		{
			error -= deltaY;
			x1 += signX;
		}
		
		if(error2 < deltaX)
		{
			error += deltaX;
			y1 += signY;
		}
	}
}


void write_symbol(int x, int y, int Size, unsigned int Color, unsigned int Backcolor, unsigned char charcode)
{
	int v, h;
	for (v = 0; v < 8; v++)                        //движение по игрек вертикаль
	{
		
		for (int s = 0; s < Size; s++)                //размер символа в точках по вертикали
		{
			write_command(0x0020);
			write_data(x);
			write_command(0x0021);
			write_data(y + s + (v * Size));
			
			for (h = 0; h < 8; h++)             //движение по икс горизонталь
			{
				if ((pgm_read_byte(&NewFont8x8[(charcode << 3) + h])&(0x01 << v)))    //((pgm_read_byte(&Simb[charcode - 0x20][v]) >> (7 - h)) & 0x01)
				{
					for (int p = 0; p < Size; p++)                       //размер символа в точках по горизонтали
					{
						write_command(0x0022);
						write_data(Color);
					}
				}
				else
				{
					for (int p = 0; p < Size; p++)
					{
						write_command(0x0022);
						write_data(Backcolor);
					}
				}
			}
		}
	}
}


void write_String(unsigned int x, unsigned int y, unsigned int color, unsigned int phone, char *str, unsigned char size)
{
	port_init();
	while (*str)
	{
		if ((x+(size*8)) > 240)
		{
			x = 1;
			y = y + (size * 8);
		}
		write_symbol(x, y, size, color, phone, *str);
		x += size * 8;
		*str++;
	}
}


void init_adc()
{
	//(1 << REFS0)|(1 << REFS1)|(1 << ADLAR)
	ADMUX = (1 << MUX2)|(1 << REFS0)|(1 << REFS1);             //ADC4
	ADCSRA |= (1 << ADEN)|(1 << ADPS2)|(1 << ADPS1)|(1<< ADPS0)|(1 << ADIE);     //запуск, предделитель 128
	ADCSRA |= (1 << ADSC);
	//ACSR |= (1 << ACD);
	//DIDR0 |= (1 << ADC0D)|(1 << ADC1D)|(1 << ADC2D)|(1 << ADC3D)|(1 << ADC5D);
	//DIDR0 |= (1 << ADC4D);
}


void drawRect(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1,  unsigned int color, unsigned char size, unsigned int backolor)
{
	line (size, x0,y0,x1,y0,color);  //горизонт 1
	line (size, x0,y1,x1,y1,color);  //горизонт 2
	
	line (size, x0,y0,x0,y1,color);  //вертикаль 1
	line (size, x1,y0,x1,y1,color);	 //вертикаль 2
	
	unsigned int delta_y = abs(y1 - y0);
	
	
	for (unsigned int i = 0; i <= (delta_y - (size+1)); i++)
	{
		write_command(0x0020);
		write_data(x0 + size);
		write_command(0x0021);
		write_data((y0 + i)+size);
		write_command(0x0022);
		
		for (unsigned int j = x0; j <= (x1-(size+1)); j++)
		{
			write_data(backolor);
		}
	}
	
	
}


void tim_0_init ()
{
	TCNT0 = 5;                 //частота возникновения прерывания 8кГц
	TIMSK0 |= (1 << TOIE0);    //разрешение прерывания таймера
	TCCR0B |= (1 << CS01);//|(1 << CS02);    //предделитель 8
}

void tim_1_init ()
{
      OCR1A = 0x000E;
      TIMSK1 |= (1 << OCIE1A);
	  TCCR1B |= (1 << CS10)|(1 << CS12)|(1 << WGM12);       //1024
}

void get_coordinate_X()
{
	DDRC &= ~(1 << PORTC0);      //С0 - вход
	
	DDRC |= (1 << PORTC1);	     //С1 - выход
	PORTC &= ~(1 << PORTC1);     //Y-
	
	DDRB |= (1 << LCD_WR);	     //Y+
	PORTB |= (1 << LCD_WR);      //Y+
	_delay_ms(50);
	DDRB |= (1 << LCD_RS);	     //X-
	PORTB |= (1 << LCD_RS);     //X-
	_delay_ms(50);
	ADMUX = (1 << REFS0)|(1 << REFS1);     //ADC0
	ADCSRA |= (1 << ADEN)|(1 << ADPS2)|(1 << ADPS1)|(1<< ADPS0)|(1 << ADIE);
	ADCSRA |= (1 << ADSC);
}


void spi_init()
{
	
}


void UART_init() {
	UBRR0H = 0x00;     //Скорость 115200
	UBRR0L = 0x07;
	//UBRR0H=Hi(((F_CPU/16)/115200)-1);
	//UBRR0L=Low(((F_CPU/16)/115200)-1);
	UCSR0B = (1 << RXCIE0)|(1 << RXEN0)|(1 << TXEN0);  //Разрешение приема данных (1 << RXEN0)|(1 << RXCIE0)|(1 << TXEN0)
	UCSR0C |= (1 << UCSZ00)|(1 << UCSZ01);          //8 N 1
}

void send_UART(char value) {
	while(!( UCSR0A & (1 << UDRE0)));   // Ожидаем когда очистится буфер передачи
	UDR0 = value;         // Помещаем данные в буфер, начинаем передачу
}

uint64_t millis (){
	static uint32_t timeCurrent = 0;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		timeCurrent = mstime;
	}
	return timeCurrent;
}

void runStep(signed int steps, uint16_t speed){				
	static uint32_t prev = 0;
	static int8_t select = 0;	
	static signed int countStep = 0, diff = 0, prevCountL = 0, prevCountR = 0;
	static uint8_t direction = 0;
	if (steps > 0){
		direction = 1;
	} else {
		direction = 0;
	}	
	if (millis() - prev >= speed) {		
		if (direction){
			if (countStep < steps + prevCountR){
				select++;
				countStep++;
				if (select == 4) {
					select = 0;				
				}
				prevCountL = countStep;
			}
			} else {
				if (countStep > steps + prevCountL) {
					select--;
					countStep--;
					if (select == -1) {
						select = 3;
					}
					prevCountR = countStep;
				}
			 }								
			
		prev = millis();
		sendNumber(countStep*0.498);
	}
	if (select == 0){
		PORTD = (A1|B2) & 0xf0;
	}
	if (select == 1){
		PORTD = (A2|B2) & 0xf0;
	}
	if (select == 2){
		PORTD = (A2|B1) & 0xf0;
	}
	if (select == 3){
		PORTD = (A1|B1) & 0xf0;
	}
    	
}





