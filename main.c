#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <math.h>
#include <avr/interrupt.h> // Library letting you work with interrupts.

#include "usart.h"


// Determine global variables:

int main(void){

  // (FOR USAGE WITH MICROCONTROLLER, VS TERMINAL AND KEYBOARD)
 uart_init(); // open the communication to the microcontroller
 io_redirect(); // redirect input and output to the communication


  while (1)
  {    

  }
  
}
