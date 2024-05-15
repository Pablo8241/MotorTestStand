/*
 * Code testing features of the nextion display
 * Current inludes:
 *  - Sine wave demo
 *  - Password keypad
 * 
 */ 


#include <stdio.h> 
#include <avr/io.h> 
#include <util/delay.h> 
#include <avr/interrupt.h>

#include "usart.h"
#include "math.h"
#include "nextion.h"

//---------CONSTANTS------------
#define F_CPU 16000000UL // needs to be defined for the delay functions to work.
#define BAUD 9600 // Tells the system how fast to communicate with the arduino and display. Need to have the same value in the arduino and Nextion display
#define ADC_PIN 1 // The ADC channel we will use


//################################________GLOBAL_VARIABLES________############################################

int state = 0;
char numb_index = 0;
volatile int new_ADC = 0; 
int ADC_value = 0;

//################################________FUNCTION_PRECALLS________############################################

void nextion_touch_event_detected();
void nextionPage0();
void nextionPage1();
void nextionPage2();
uint16_t readADC(int adcChannel); // Reads battery voltage, input the pin we wish to read the voltage value from
void init_interupt_ADC();

//################################_____________MAIN________________############################################

int main(void) 
{   

  nextion_init();
  init_interupt_ADC();

  printf("page 0%c%c%c",0xFF,0xFF,0xFF); // Displays page0 on the nextion display.
  _delay_ms(100);
  nextionPage0();
  
  return 0;
}

ISR(PCINT1_vect) // pin change in group 1 interrupt routine
{
  // Checks what value triggered the interrupt 
  // if((PINC & (1 << (PINC3))) == 1) // LOW to HIGH pin change (rising edge)
  // {
  new_ADC = 1;

  // }
  if(PINC == 0b00001000)
  {
    
  }
  else // HIGH to LOW pin change (falling edge)
  {
    new_ADC = 1;
  }
}

//################################___________FUNCTIONS_____________############################################

void nextion_touch_event_detected()
{

  //Read out command
	if(commandbuffer[0] == 0x65) // Touch Event
	{
		switch (commandbuffer[1]) //Checks page number
		{
		case 0: // Touch event happend on page 0
      nextionPage0(); 
      break;
    case 1: // Touch event happend on page 1
      nextionPage1(); 
      break;
    case 2: // Touch event happend on page 2
      nextionPage2(); 
      break;
		}
	}

}

void nextionPage0() // Commands on page 0
{
  
  if(commandComplete)
  {
    commandComplete = 0;

    // Code for when touch evnt on page occurs
    switch (commandbuffer[2]) //Checks component id
    {
    case 1:
      nextion_display_page(1);
      nextionPage1();
      break;
    case 2:
      nextion_display_page(2);
      numb_index = 0;
      nextionPage2();
      break;
    }
  }

  // Code runnign while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occured. If yes runs function
  }
}

void nextionPage1() // Commands on page 1
{
  if(commandComplete)
  {
    commandComplete = 0;

    // Code for when touch evnt on page occurs
    switch (commandbuffer[2]) //Checks component id
    {
    case 0x03: // Starts (Button)
      nextion_resume_execution();
      state = 0;
      break;
    case 0x04: // Stops (Button)
      nextion_stop_execution();
      state = 1;
      break;
    case 0x05: // Clears (Button)
      nextion_waveform_clear_channel(1);
      break;
    case 0x06: // change data scaling (Slider)
      nextion_write_value(1, "n0", nextion_read_value(1, "h0"));
      nextion_change_data_scaling(1, "s0", nextion_read_value(1, "h0"));
      break;
    case 8:
      nextion_display_page(0);
      nextionPage0();
      break;
    }
    return;
  }

  // Code runnign while on page
  while(1) 
  {

    for (int a = 0; a < 360; a++)
    { 
      if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occured. If yes runs function
      if(state) {break;}

      double angle_radians = (double)a * M_PI / 180.0; // Convert degrees to radians
      double sine_value = sin(angle_radians);
      

      nextion_waveform_write_value(1, 1, ((int)(sine_value*100))+100);
      nextion_write_value(1, "x0", (int)(sine_value*100));
      
      if(new_ADC == 1)
      {
        ADC_value = readADC(ADC_PIN);
        if(ADC_value < 10) {ADC_value = 10;}
        if(ADC_value > 1000) {ADC_value = 1000;}
        nextion_change_data_scaling(1, "s0", ADC_value);
        new_ADC = 0;

      }

    }
    

  }
}

void nextionPage2() // Commands on page 2
{

  if(commandComplete)
  {
    commandComplete = 0;

    // Code for when touch evnt on page occurs
    if((commandbuffer[2] <= 10) && (numb_index <= 3))
    {
      printf("page%d.n%d.val=%d%c%c%c", 2, numb_index, (commandbuffer[2]-1), 0xFF,0xFF,0xFF);
      printf("page%d.n%d.pco=%d%c%c%c", 2, numb_index, 0, 0xFF,0xFF,0xFF);
      numb_index++;
    }
    
    switch (commandbuffer[2]) //Checks component id
    {
    case 12: // Back button
      nextion_display_page(0);
      nextionPage0();
      break;
    case 17: // Clear/delete button
      numb_index--; 
      printf("page%d.n%d.pco=%ld%c%c%c", 2, numb_index, 65535L, 0xFF,0xFF,0xFF);
      break;
    case 18: // Enter/confirm button
      if((nextion_read_value(2, "n0") == 6) && (nextion_read_value(2, "n1") == 9) && (nextion_read_value(2, "n2") == 6) && (nextion_read_value(2, "n3") == 9))
      {
        nextion_display_page(1);
        nextionPage1();
      }
      break;
    }
  }

  // Code runnign while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occured. If yes runs function
  }
}

// Function that reads battery voltage
uint16_t readADC(int adcChannel) 
{
	ADMUX &= 0xF0; // Clears the input channel selections, but keep voltage selection reference
  ADMUX |= adcChannel; // Sets what pin the voltage is going to be read from

  // Starts a conversion 
  ADCSRA |= (1 << ADSC);

  // Waits for the convertion to complete, since ADSC will be set to 0 when the conversion is complete
  while((ADCSRA & (1 << ADSC)));

  // Retuns the voltage with a range from 0 to 1024, where 5V == 1024 
  return ADC; // 16 bit unsigned integer
}

void init_interupt_ADC()
{
  DDRC &= ~(1 << PC1); // Configures the pin PC1 to be an input, for pin ADC1 to be used for ADC reading
  DDRC &= ~(1 << PC3); // Configures the pin PC1 to be an input, for pin ADC3 to be used for External interrupt
  DDRC = 0x00;
  PORTC |= (1 << PC3); // Pull up

  // ACD Reading Setup
  ADMUX |= (1 << REFS0); // Voltage Reference Selection: AVCC with external capacitor at AREF pin.
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, ADC Prescaler Select Bits: 128 Division factor (Slow input clock)
  
  // External interrupt on pin INT0
  PCICR |= (1 << PCIE1); // Set PCIE0 to enable the group for PCINT14...PCINT8
  PCMSK1 |= (1 << PCINT11); // Enables only interrupts form the PCINT11 (A3) pin in the group  (any change on pin)
  sei(); // Enables interrupts
}
