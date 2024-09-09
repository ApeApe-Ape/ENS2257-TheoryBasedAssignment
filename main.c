
// for ENS2257/ENS6155 Lab Kits with ATmega328PB
// Theory Based Assignment Task 1  

#define F_CPU 16000000UL

#include <atmel_start.h>
#include <stdint-gcc.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/iom328pb.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <lcd.h>

// PIN DEFINITIONS:
// PB0 -- RELAY1_PIN
// PB1 -- RELAY2_PIN
// PB2 -- RELAY3_PIN

#define RELAY1_PIN  PB0  // Relay 1 connected to Port B Pin 0
#define RELAY2_PIN  PB1  // Relay 2 connected to Port B Pin 1
#define RELAY3_PIN  PB2  // Relay 3 connected to Port B Pin 2

#define CLAP_DETECTION_DELAY 300 // Time window for detecting claps in ms
#define MAX_CLAPS 4				 // Set limitation for claps counts.

void relays_init() {
	// Set relay pins as output
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2);
	
	// Initialize all relays to off
	PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2));
}

//**************************************************

void trigger_relay(uint8_t relay_number) {
	PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2));		// Turn off all relays first
	
	// Switch on the selected relay
	if (relay_number == 1) {
		PORTB |= (1 << PB0);
		} else if (relay_number == 2) {
		PORTB |= (1 << PB1);
		} else if (relay_number == 3) {
		PORTB |= (1 << PB2);
	}
}


//**************************************************

uint16_t read_microphone() {	
	ADCSRA |= (1 << ADSC);					// Start ADC conversion (assuming the microphone is connected to ADC pin)
		
	while (ADCSRA & (1 << ADSC));			// Wait for conversion to complete
		
	return ADC;								// Return the ADC value (10-bit resolution)
}

//**************************************************

void detect_claps(uint16_t clap_threshold) {
	uint8_t clap_count = 0;
	uint16_t sound_level;
	uint16_t last_clap_time = 0;
	
	while (1) {
		sound_level = read_microphone();
		
		if (sound_level > clap_threshold) {
			_delay_ms(CLAP_DETECTION_DELAY);  // Small debounce delay to avoid false positives
			clap_count++;
			
			// Wait for a short time to detect subsequent claps
			last_clap_time = 0;
			while (last_clap_time < CLAP_DETECTION_DELAY) {
				sound_level = read_microphone();
				if (sound_level > clap_threshold) {
					_delay_ms(CLAP_DETECTION_DELAY);  // Debounce delay
					clap_count++;
				}
				last_clap_time += CLAP_DETECTION_DELAY;
			}
			
			// Check number of claps and trigger the appropriate relay
			if (clap_count == 2) {
				trigger_relay(1);  // Trigger relay 1 for 2 claps
				} else if (clap_count == 3) {
				trigger_relay(2);  // Trigger relay 2 for 3 claps
				} else if (clap_count == 4) {
				trigger_relay(3);  // Trigger relay 3 for 4 claps
			}
			
			// Reset clap count after action
			clap_count = 0;
		}
	}
}

//**************************************************

int main(void) {
	// Initialize relays
	relays_init();
	
	// Initialize ADC (assuming microphone is connected to ADC pin)
	ADMUX = (1 << REFS0); // AVcc as reference, ADC0 as input (can change if necessary)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC with prescaler 64
	
	// Call detect_claps function with a threshold value to control sensitivity
	detect_claps(512);  // Example threshold value, adjust based on your microphone
	
	return 0;
}