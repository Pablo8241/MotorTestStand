#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h> // Library letting you work with interrupts.

#include "usart.h"


// Determine global variables:
volatile int changes = 0;
volatile unsigned int counter = 0;
volatile long int overflows = 0;


int main(void){

  // (FOR USAGE WITH MICROCONTROLLER, VS TERMINAL AND KEYBOARD)
 uart_init(); // open the communication to the microcontroller
 io_redirect(); // redirect input and output to the communication

 printf("**Code for optocoupler**");

 // Data Direction Registers:
 DDRB |= (0 << PB0); // Sets PB0 to be the input pin
 PORTB |= (1 << PB0); // Enables the pull up on pin PB0

 // Interrupt registers
 TCCR1A = 0x00; // Timer/Counter control register A (Normal port operation, OC1A/OC1B disconnected)
 TCCR1B |= (1 << CS12) | (1 << CS10) | (1 << ICNC1); // Timer/Counter control register B (1024 prescaler, noise canceler)

 TIMSK1 |= (1 << ICIE1) | (1 << TOIE1); // Setting up timer/counter1 interrupt mask register (Input Capture Interrupt Enable, Overflow Interrupt Enable)

 sei(); //enables interrupts

  while (1)
  {    
    _delay_ms(1500);
    printf("The optocoupler has detected %d changes.\n", changes);
    printf("The counter was at %.2f ms\n", counter*(1024/16000.0f)); // Converts cycles to milliseconds
    printf("%u", (unsigned int)ICR1);
  }
  
}

// Interrupts
ISR(TIMER1_CAPT_vect){ // Timer/Counter1 Capture Event
  // Event to be executed when PCINT0 occurrs. It measures the time between each interrupt.
  if (overflows > 0 || ICR1 > 3000){ // with fail-safe
    changes++;
    counter = ICR1 + (overflows*65535); // Reads the amount of cycles passed since last interrupt
    overflows = 0;
  }
   TCNT1 = 0; // resets the timer counter
}

ISR(TIMER1_OVF_vect){ // Timer/Counter1 Overflow Event (acts as a fail-safe)
 overflows++;
}
