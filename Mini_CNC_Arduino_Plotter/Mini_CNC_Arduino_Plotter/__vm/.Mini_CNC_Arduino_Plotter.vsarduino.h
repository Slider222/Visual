/* 
	Editor: https://www.visualmicro.com/
			visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
			the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
			all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
			note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: avrnetio w/ atmega644p (optiboot.c), Platform=avr, Package=avrnetio
*/

#define __AVR_ATmega644p__
#define __AVR_ATmega644P__
#define ARDUINO 10805
#define ARDUINO_MAIN
#define F_CPU 14745600L
#define __AVR__
#define F_CPU 14745600L
#define ARDUINO 10805
#define ARDUINO_AVR_AVRNETIO_AVRNETIO_AVR_ANIO644P
#define ARDUINO_ARCH_AVR
//
//
void processIncomingLine( char* line, int charNB );
void drawLine(float x1, float y1);
void penUp();
void penDown();

#include "pins_arduino.h" 
#include "arduino.h"
#include "Mini_CNC_Arduino_Plotter.ino"
