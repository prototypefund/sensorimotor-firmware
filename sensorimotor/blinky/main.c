/* 
 * SENSORIMOTOR
 * ------------
 * File: blinky/main.c
 * Description: Toggels pin PB0 every 500ms
 * From: C-Programmierung mit AVR-GCC
 * 
 * 1) compile : avr-gcc -mmcu=atmega328p -Os -c main.c -o main.o
 * 2) link    : avr-gcc main.o -o main.elf
 * 3) to hex  : avr-objcopy -O ihex -j .text -j .data main.elf main.hex
 * 4) check   : avr-size --mcu=atmega328p -C main.elf
 * 5) burn    : avrdude -c arduino -p m328p -P /dev/ttyUSB0 -b 19200 -v -U flash:w:main.hex:a
 * or use make 
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

int main (void) {
	/*DDRB |= (1 << PB1); // PWM
	DDRD |= (1 << PD2); // DIR
	DDRD |= (1 << PD4); // VSO
	DDRD |= (1 << PD6); // DIS*/
	DDRD |= (1 << PD5); // LED
	
//	PORTD = (1 << PD4); // VSO ON, (enable H-Bridge)

	//PORTB = (1 << PB1); // PWM PIN HIGH, full drive

	while(1) {
		PORTD ^= (1 << PD5); // LED
		//PORTD ^= (1 << PD2); // switch DIR       	
		_delay_ms(100);
	}

   return 0;
}
