/*
 * DC Moter Test Stand Code
 * 
 */ 


#include <stdio.h> 
#include <avr/io.h> 
#include <util/delay.h> 
#include <avr/interrupt.h>

#include "usart.h"
#include "math.h"
#include "nextion.h"
#include "HX711.h"

//---------CONSTANTS------------
#define F_CPU 16000000UL // needs to be defined for the delay functions to work.
#define BAUD 9600 // Tells the system how fast to communicate with the arduino and display. Need to have the same value in the arduino and Nextion display

//################################________GLOBAL_VARIABLES________############################################

unsigned char state_test = 0;
unsigned char numb_index = 0;
float calibration_factor = 1100; // gets changed can be positive or negative, depends on setup
float loadcell_value[100];
float max_loadcell_value = 0;
float max_RPM_value = 0;
volatile int changes = 0;
volatile unsigned int counter = 0;
volatile long int overflows = 0;

//################################________FUNCTION_PRECALLS________############################################

void nextion_touch_event_detected();
void nextionPage0();
void nextionPage1();
void nextionPage2();
void nextionPage3();
void functionF1();
void functionF2();
void runTest();
void optocoupler_init();

//################################_____________MAIN________________############################################

int main(void) 
{   

  nextion_init();
  //HX711_init(128);
  optocoupler_init();

  printf("page 0%c%c%c",0xFF,0xFF,0xFF); // Displays page0 on the nextion display.
  _delay_ms(100);
  nextionPage0();
  
  return 0;
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
    case 3:
      nextionPage3();
      break;
		}
	}

}

void nextionPage0() // Start/Page selection
{
  
  // Code for when touch evnt on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

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
    case 3:
      nextion_display_page(3);
      nextionPage3();
      break;
    }
  }

  // Code runnign while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occured. If yes runs function
  }
}

void nextionPage1() // Waveform test
{

  // Code for when touch evnt on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

    // Code for when touch evnt on page occurs
    switch (commandbuffer[2]) //Checks component id
    {
    case 3: // Back (Button)
      nextion_display_page(0);
      nextionPage0();
      break;
    case 7: // Start Rotation (Dual-state button)
      
      break;
    case 8: // Start Test (Dual-state button)
      runTest();
      break;
    }
    // return;
  }

  // Code runnign while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occured. If yes runs function
  }
  
}

void nextionPage2() // Keypad 
{

  // Code for when touch evnt on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

    if((commandbuffer[2] <= 10) && (numb_index <= 13))
    {
      printf("page%d.n%d.val=%d%c%c%c", 2, numb_index, (commandbuffer[2]-1), 0xFF,0xFF,0xFF);
      printf("page%d.n%d.pco=%d%c%c%c", 2, numb_index, 0, 0xFF,0xFF,0xFF);
      numb_index++;
    }
    
    switch (commandbuffer[2]) //Checks component id
    {
    case 11: // delete button
      numb_index--; 
      printf("page%d.n%d.pco=%ld%c%c%c", 2, numb_index, 65535L, 0xFF,0xFF,0xFF);
      break;
    case 12: // Enter/confirm button
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

void nextionPage3() // Drawing lines
{ 
  
  // Code for when touch evnt on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

    switch (commandbuffer[2]) //Checks component id
    {
    case 1: // F1
      functionF1();
      break;
    case 2: // F2
      functionF2();
      break;
    case 3: // Back button
      nextion_display_page(0);
      nextionPage0();
      break;
    }
  }

  // Code runnign while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occured. If yes runs function
  }
  
}

void functionF1() // Torque
{
  int x_last = 0, y_last = 480;
  int y_new_new = 0;
  int y_new_new_new = 0;
  float old_procentage = 0;

  int old_min = 0;
  int old_max = max_loadcell_value;
  int new_min = 0;
  int new_max = 480;

  for (int a = 0; a < 100; a++)
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occured. If yes runs function

    old_procentage = (loadcell_value[a] - old_min) / (old_max - old_min);
    y_new_new = (((new_max - new_min) * old_procentage) + new_min);
    y_new_new_new = 480-y_new_new;

    nextion_draw_line(x_last, y_last, 8*a, y_new_new_new, "BLUE");

    x_last = 8*a;
    y_last = y_new_new_new;

  }
}

void functionF2() // RPM
{
  int x_last = 0, y_last = 480;
  int y_new_new = 0;
  int y_new_new_new = 0;
  float old_procentage = 0;

  int old_min = 0;
  int old_max = max_RPM_value;
  int new_min = 0;
  int new_max = 480;

  for (int a = 0; a < 100; a++)
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occured. If yes runs function

    old_procentage = (loadcell_value[a] - old_min) / (old_max - old_min);
    y_new_new = (((new_max - new_min) * old_procentage) + new_min);
    y_new_new_new = 480-y_new_new;

    nextion_draw_line(x_last, y_last, 8*a, y_new_new_new, "RED");

    x_last = 8*a;
    y_last = y_new_new_new;

  }

}

void runTest() // Gathers and displays information from the optocoupler and loadcell
{
  HX711_tare(10); // tares scale, use no weight
  HX711_set_scale(calibration_factor); //Adjust to this calibration factor
  char loop_counter = 0; //counts number of loop and is used to control GLS

  for (int a = 0; a < 100; a++)
  { 
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occured. If yes runs function

    // Torque reading (Loadcell)
    loadcell_value[a] = HX711_get_mean_units(100);
    
    nextion_write_value(1, "x0", (int)(loadcell_value[a]*100));

    if(loadcell_value[a] < 0) {loadcell_value[a] = 0;}
    if(loadcell_value[a] > max_loadcell_value) {max_loadcell_value = loadcell_value[a];}
    
    nextion_waveform_write_value(1, 1, (int)(loadcell_value[a]));


    // RPM reading (Optocoupler)



    // GLS communication over i2c
    loop_counter++;
    
    if(loop_counter == 10)
    {
      // sendt "a" over i2c to GLS controler
      loop_counter = 0;
    }

  }
  

}

void optocoupler_init()
{
  // Data Direction Registers:
  DDRB |= (0 << PB0); // Sets PB0 to be the input pin
  PORTB |= (1 << PB0); // Enables the pull up on pin PB0

  // Interrupt registers
  TCCR1A = 0x00; // Timer/Counter control register A (Normal port operation, OC1A/OC1B disconnected)
  TCCR1B |= (1 << CS12) | (1 << CS10) | (1 << ICNC1); // Timer/Counter control register B (1024 prescaler, noise canceler)

  TIMSK1 |= (1 << ICIE1) | (1 << TOIE1); // Setting up timer/counter1 interrupt mask register (Input Capture Interrupt Enable, Overflow Interrupt Enable)
}
