/*
 * prosjektuSys.c
 * This is a weather-station. It measures the temperature and the lightness, then determines when the weather is nice enough to go outside. 
 * Created: 30.04.2021 10.17.54
 * Author : Espen
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "USART.h"
#include <util/delay.h>
#include "math.h"

#define SAMPLE_DELAY  500 //ms,  controls the pace of the measurements.

static inline void initADC0(void) {
	ADMUX |= (1 << REFS0); /* reference voltage on AVCC */
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); /* ADC clock prescaler /128 */

	ADCSRA |= (1 << ADEN); /* enable ADC */
	
}

float getTemp(uint16_t adcValue) {
	
	float temperature;
	int B;
	double resistance;
	float val;
	float res1;
	float volt1;
	
	val = (adcValue/1024.0)*5.0;
	volt1 = 5.0 - val;
	res1 = (volt1/val)*10000.0; 
	B = 3950;
	resistance = 10000/res1;
	
	temperature = 1/((1/298.15)-((log(resistance)/B))) - 273.15;			//To get celsius
	
	return temperature;
}

void initInterrupt(void) {
	EIMSK |= (1<<INT0);				//Enable INT0
	EICRA |= (1<<ISC00);			//Any change in INT0
	sei();   //Enables global interrupts
}

int main(void)
{
	uint16_t thermistorValue;
	uint16_t photoValue;
	uint16_t adcValue;
	
	initUSART();
	initADC0();
	initInterrupt();
	//Set registers
	
	DDRD |= (1<<DDD2);			//PD2 output
	DDRB |= (1<<DDB1);			//PB1 output
	
	TCCR1A |= (1<<COM1A0);     //Toggle OC1A on Compare match
	TCCR1B |= (1<<CS12) | (1<<CS10) | (1<<WGM12); //1024 prescaler
	OCR1A = 0x3D08;				//toggler hvert sekund

    while (1) 
    {
			//Read sensordata with ADC
			
	ADCSRA |= (1 << ADSC); // start ADC conversion 
	loop_until_bit_is_clear(ADCSRA, ADSC); // wait until done
	
	if (!( ADMUX & (1<<MUX0) )) {					//Checks if the ADC is measuring ADC0 
		adcValue = ADC;	
		thermistorValue = getTemp(adcValue);			//Overwrites thermistorValue with new value
		printString("Thermistor; ");
		printWord(thermistorValue);
		transmitByte(10);				//Newline
	}
	
	else {
		photoValue = ADC;			//Overwrites photoValue with new value
		printString("Photoresistans; ");
		printWord(photoValue);
		transmitByte(10);				//Newline
	}
	
	ADMUX ^= (1<<MUX0);     //Toggles the ADC channel from ADC0 to ADC1 (or vice versa)
	 
    if ((thermistorValue >= 16) | (photoValue >= 1005)) {				//Checks if the weather is nice
		PORTD ^= (1<<PORTD2);					//Toggles the diode and interrupt
	}
	else { 
		PORTD &= ~(1<<PORTD2);				//Turns diode off. 
	}
	_delay_ms(SAMPLE_DELAY);
	}                                                  /* End event loop */
	return 0;                            /* This line is never reached */
}

ISR(INT0_vect) {				/* Run every time the interrupt happens */

	printString("The weather is nice, go outside");
	transmitByte(10);	  //newline

}


