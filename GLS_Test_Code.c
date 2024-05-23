#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"

// Constants:


// Global Variables:


// Function prototypes:
void init(void);

int main(void){

  // (FOR USAGE WITH MICROCONTROLLER, VS TERMINAL AND KEYBOARD)
 uart_init(); // open the communication to the microcontroller
 io_redirect(); // redirect input and output to the communication

 printf("**Code for GLS Arduino**\n(0) No light bulbs connected. The first MOSFET causes a short-circuit.\n(1-12) Connecting a number of light bulbs anywhere from 1-12\n");

  init(); // initializer - sets 13 pins as outputs - MOSFET controls

  int input;
  while (1){
    printf("Specify amount of bulbs:\n");
    scanf("%d", &input);
    printf("Number of bulbs: %d\n", input);

    if (input == 0){ // No bulbs (short circuit) PIN IS NOT CONNECTED ANYMORE
      PORTD = 0x00;
      PORTC = 0x00;

      PORTB = (1 << input);
    }
    if ((input < 7) && (input > 0)){ // Bulbs 1-6
      PORTB = 0x00;
      PORTD = 0x00;

      PORTC = (1 << (input-1));
    }
    if ((input >= 7) && (input < 13)){ // Bulbs 7-12
      PORTC = 0x00;
      PORTB = 0x00;

      PORTD = (1 << (input-5));
    }
    if (input > 12){
      printf("Your input has exceeded the maximum number of light bulbs(12).\n");
    }
  } // main while loop

} // main

// Function Definitions:

void init(void){
 // Data Direction Registers:
  DDRD |= ~((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7)); // make pins PD 2-7 as outputs
  DDRC |= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5)); // make pins PC0-PC5 as outputs
  DDRB |= ~(1 << PB0); // make pin PB0 as output

  // Setting all pins initially as low (0 Volts)
  PORTD = 0x00;
  PORTB = 0x00;
  PORTC = 0x00;
}
