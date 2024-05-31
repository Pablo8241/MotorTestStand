/*
 * DC motor Test Stand Code
 * 
 */ 


#include <stdio.h> 
#include <avr/io.h> 
#include <util/delay.h> 
#include <avr/interrupt.h>
#include <stdlib.h> 
#include <avr/eeprom.h>

#include "usart.h"
#include "math.h"
#include "nextion.h"
#include "HX711.h"

// ########################################## OTHER STUFF ############################################ //

typedef struct
{
  uint16_t mcc_voltage; // Volts [V]
  uint16_t mcc_current; // mili amps [mA]
  uint16_t RPM; // Radians pr.second = (RPM*2*pi)/60
  uint32_t torque; // Newton Meters [Nm]
} sample_t;

// ################################################################################################### //

// ########################################## DEFINITIONS ############################################ //

//------------ NEXTION DEFINITIONS ------------//
#define TX_pin PD0
#define RX_pin PD1

//-------------- GLS DEFINITIONS --------------//
#define GLS_0_bulb PD2
#define GLS_1_bulb PD3
#define GLS_2_bulb PD4
#define GLS_3_bulb PD5
#define GLS_4_bulb PD6
#define GLS_5_bulb PD7
#define GLS_6_bulb PC5 //!!!
#define GLS_7_bulb PB1
#define GLS_8_bulb PB2
#define GLS_9_bulb PB3
#define GLS_10_bulb PB4
#define GLS_11_bulb PB5

//----- MOTOR CONTROL CIRCUIT DEFINITIONS -----//
#define MCC_voltage_adc_pin PC0
#define MCC_current_adc_pin PC1
#define MCC_motor_enable_pin PC4

//--------- TORQUE METER DEFINITIONS ----------//
#define torque_CLK_pin PC2
#define torque_DO_pin PC3
#define calibration_factor 18680000
#define HX711_times 5

//---------- OPTOCOUPLER DEFINITIONS ----------//
#define opto_DO_pin PB0
#define HOLES 1

//------------- OTHER DEFINITIONS -------------//
#define ARRAY_SIZE 100
#define MAX_RPM_ADDRESS 0
#define MAX_TORQUE_ADDRESS 2

// ################################################################################################### //

// ######################################## GLOBAL VARIABLES ######################################### // 

unsigned char state_test = 0;
unsigned char numb_index = 0;
float max_torque_value = 0;
float max_RPM_value = 0;
unsigned char GLS_state = 0;
volatile int changes = 0;
volatile unsigned int counter = 0;
volatile long int overflows = 0;
sample_t data[ARRAY_SIZE]; 
int RPM_constant_test[100];
uint8_t status = 0;
#define v_ref 5.07
#define shunt 0.0025

// ################################################################################################### //

// ####################################### FUNCTION PROTOTYPES ####################################### //

void nextion_touch_event_detected();
void nextionPage0();
void nextionPage1();
void nextionPage2();
void nextionPage3();
void nextionPage4();
void nextionPage5();
void automaticTest();
void manualTest();
void trendline_RPM_torque(float *a_p, float *b_p);
void trendline_current_torque(float *a_p);
void trendline_power_torque();
void motor_constants(float RPM_torque_b, float RPM_torque_a, float current_torque_a);
void GLS_control(int numb_lightbulbs_on);
void GLS_init();
void optocoupler_init();
void timer0_init();
void ADC_init();
void motor(uint8_t run);
uint16_t read_main_voltage();
uint16_t read_current_adc();
void delay_milliseconds(unsigned int milliseconds);
uint16_t RPMReadValue();

// ################################################################################################### //

// ############################################## MAIN ############################################### //

int main(void) 
{   

  nextion_init(); // Enables interrupts
  HX711_init(128);
  optocoupler_init();
  GLS_init();
  timer0_init();
  // ADC_init();

  printf("page 0%c%c%c",0xFF,0xFF,0xFF); // Displays page0 on the nextion display.
  nextionPage0();
  
  return 0;
}

// ################################################################################################### //

// ############################################ INTERRUPTS ########################################### //

ISR(TIMER1_CAPT_vect) // Timer/Counter1 Capture Event
{ 
  // Event to be executed when PCINT0 occurs. It measures the time between each interrupt.
  if (overflows > 0 || ICR1 > 100){ // with fail-safe
    changes++;
    counter = ICR1 + (overflows*65535); // Reads the amount of cycles passed since last interrupt
    overflows = 0;
  }
  TCNT1 = 0; // resets the timer counter
}

ISR(TIMER1_OVF_vect) // Timer/Counter1 Overflow Event (acts as a fail-safe)
{ 
 overflows++;
}

// ################################################################################################### //

// ############################################ FUNCTIONS ############################################ //

void nextion_touch_event_detected()
{

  //Read out command
	if(commandbuffer[0] == 0x65) // Touch Event
	{
		switch (commandbuffer[1]) //Checks page number
		{
		case 0: // Touch event happened on page 0
      nextionPage0(); 
      break;
    case 1: // Touch event happened on page 1
      nextionPage1(); 
      break;
    case 2: // Touch event happened on page 2
      nextionPage2(); 
      break;
    case 3: // Touch event happened on page 3
      nextionPage3();
      break;
    case 4: // Touch event happened on page 4
      nextionPage4();
      break;
    case 5: // Touch event happened on page 5
      nextionPage5();
      break;
		}
	}

}

void nextionPage0() // Keypad 
{

  // Code for when touch event on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

    if((commandbuffer[2] <= 10) && (numb_index <= 13))
    {
      printf("page%d.n%d.val=%d%c%c%c", 0, numb_index, (commandbuffer[2]-1), 0xFF,0xFF,0xFF);
      printf("page%d.n%d.pco=%d%c%c%c", 0, numb_index, 0, 0xFF,0xFF,0xFF);
      numb_index++;
    }
    
    switch (commandbuffer[2]) //Checks component id
    {
    case 11: // delete button
      numb_index--; 

      printf("page%d.n%d.pco=%ld%c%c%c", 0, numb_index, 65535L, 0xFF,0xFF,0xFF);
      printf("page%d.n%d.val=%d%c%c%c", 0, numb_index, 0, 0xFF,0xFF,0xFF);
      break;
    case 12: // Enter/confirm button
      if((nextion_read_value(0, "n0") == 0) && (nextion_read_value(0, "n1") == 0) && (nextion_read_value(0, "n2") == 0) && (nextion_read_value(0, "n3") == 0))
      {
        numb_index = 0;
        nextion_display_page(1);
        nextionPage1();
      }
      break;
    }
  }

  // Code running while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occurred. If yes runs function
  }
}

void nextionPage1() // Setup Guide
{
  
  // Code for when touch event on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

    switch (commandbuffer[2]) //Checks component id
    {
    case 11: // Manual Test
      nextion_display_page(3);
      nextionPage3();
      break;
    case 5: // Automatic Test
      nextion_display_page(4);
      automaticTest();
      nextionPage4();
      break;
    }
  }

  // Calculate current and voltage
  nextion_write_value(1, "x0", ((double)v_ref * 100 * (double)read_main_voltage() * 4 / 1024)); // Prints voltage
  nextion_write_value(1, "x1", ((1.1 * 1000 * (double)read_current_adc() / 1024) / shunt)); // Prints current

  // Code running while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occurred. If yes runs function
  }
}

void nextionPage2() // Manual Test
{

  // Code for when touch event on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

    // Code for when touch event on page occurs
    switch (commandbuffer[2]) //Checks component id
    {
    case 3: // Back (Button)
      nextion_display_page(3);
      nextionPage3();
      break;
    case 7: // Start/stop Rotation (Dual-state button)
      motor(nextion_read_value(2, "bt0")); // starts/stops motor
      break;
    case 8: // Start/stop Test (Dual-state button)
      state_test = nextion_read_value(2, "bt1");
      manualTest();
      break;
    }
    // return;
  }

  // Code running while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occurred. If yes runs function
  }
  
}

void nextionPage3() // Manually Control Lightbulbs
{
  
  // Code for when touch event on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

    switch (commandbuffer[2]) //Checks component id
    {
    case 2: // Go to manual test page
      nextion_display_page(2);
      nextionPage2();
      break;
    case 3: // Back (Button) 
      nextion_display_page(1);
      nextionPage1();
      break;
    case 4: // Contorl lightbulb (Slider)
      nextion_write_value(3, "n0", nextion_read_value(3, "h0"));
      GLS_control(nextion_read_value(3, "h0"));
      if(GLS_state == 0) // GLS off
      {
        nextion_write_value(3, "bt0", 0);
        printf("page%d.n%d.pco=%d%c%c%c", 3, 0, 0, 0xFF,0xFF,0xFF);
        GLS_state = 1;
      }
      break;
    case 7: // Disable/enable GLS 
      if(GLS_state == 1) // GLS on
      {
        GLS_control(20);
        printf("page%d.n%d.pco=%ld%c%c%c", 3, 0, 65535L, 0xFF,0xFF,0xFF);
        GLS_state = 0;
      }
      else // GLS off
      {
        GLS_control(nextion_read_value(3, "h0"));
        printf("page%d.n%d.pco=%d%c%c%c", 3, 0, 0, 0xFF,0xFF,0xFF);
        GLS_state = 1;
      }
      break;
    }

  }

  // Code running while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occurred. If yes runs function
  }
}

void nextionPage4() // Automatic Test
{

  // Code for when touch event on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

    // Code for when touch event on page occurs
    switch (commandbuffer[2]) //Checks component id
    {
    case 10: // Show results
      // Corrects values on page 3 and 4
      nextion_write_value(3, "bt0", 0);
      nextion_write_value(3, "h0", 0);
      nextion_write_value(3, "n0", 0);
      printf("page%d.n%d.pco=%d%c%c%c", 3, 0, 0, 0xFF,0xFF,0xFF);
      nextion_write_value(2, "bt0", 0);

      // Turns off GLS and motor
      GLS_control(0); // Shorts GLS
      motor(0); // Turns off motor

      nextion_display_page(5);
      nextionPage5();
      break;
    }
  }

  // Code running while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occurred. If yes runs function
  }
  
}

void nextionPage5() // Results
{ 
  
  // Code for when touch event on page occurs
  if(commandComplete)
  {
    commandComplete = 0;

    switch (commandbuffer[2]) //Checks component id
    {
    case 26: // Return to page 1
      nextion_display_page(1);
      nextionPage1();
      break;
    case 19: // Repeat automatic test (page 4)
      nextion_display_page(4);
      automaticTest();
      nextionPage4();
      break;
    }

  }

  // Calculate and display equations
  float RPM_torque_a = 0, RPM_torque_b = 0, current_torque_a = 0;

  trendline_RPM_torque(&RPM_torque_a, &RPM_torque_b);
  trendline_current_torque(&current_torque_a);
  trendline_power_torque();

  // Calculate and display constants 
  motor_constants(RPM_torque_a, RPM_torque_b, current_torque_a);

  // Code running while on page
  while(1) 
  {
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occurred. If yes runs function
  }
  
}

void automaticTest() // Gathers and displays information from the optocoupler and loadcell
{
  // constant test
  GLS_control(20); // Open circuit
  delay_milliseconds(2000);

  motor(1); // Starts motor
  for(int i = 0; i < 100; i++) // Test wil run for 100*20 ms = 2 seconds
  {
    RPM_constant_test[i] = RPMReadValue();
    delay_milliseconds(20); 
  }
  motor(0); // Turns off motor
  delay_milliseconds(2000);

  // Load cell setup 
  HX711_tare(10); // tares scale, use no weight
  HX711_set_scale(calibration_factor); //Adjust to this calibration factor

  // Setup GLS
  char loop_counter = 0; //counts number of loop and is used to control GLS
  int8_t loop_counter2 = 10;
  GLS_control(10); // Turns on 10 light bulbs
  
  //****************************************************
  // For testing line segments
  int torque_x_last = 0, torque_y_last = 480;
  int torque_y_new_new_new = 0;
  int torque_y_new_new = 0;
  float torque_old_percentage = 0;
  
  int RPM_x_last = 0, RPM_y_last = 480;
  int RPM_y_new_new_new = 0;
  int RPM_y_new_new = 0;
  float RPM_old_percentage = 0;

  int torque_old_min = 0, RPM_old_min = 0;
  uint16_t torque_old_max = eeprom_read_word((uint16_t *)MAX_TORQUE_ADDRESS); // Reads previous max torque from eeprom
  uint16_t RPM_old_max = eeprom_read_word((uint16_t *)MAX_RPM_ADDRESS); 
  int new_min = 0;
  int new_max = 480;
  uint16_t temp_max_torque = 0;
  uint16_t temp_max_RPM = 0;
  
  //****************************************************

  motor(1); // Turns on motor
  for (int a = 0; a < ARRAY_SIZE; a++)
  { 
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occurred. If yes runs function

    // Reads input values
    data[a].torque = (int)(HX711_get_mean_units(HX711_times)*1000); // Torque reading (Loadcell)
    data[a].RPM = RPMReadValue(); // gives the RPM value; // RPM reading (Optocoupler)
    data[a].mcc_voltage = ((double)v_ref * 100 * (double)read_main_voltage() * 4 / 1024); // calculate the voltage V;
    data[a].mcc_current = ((1.1 * 1000 * (double)read_current_adc() / 1024) / shunt); // calculate the current mA
    
    // Writes digital values to Display
    nextion_write_value(4, "x1", (int)(data[a].torque)); // Writes digital value of torque
    nextion_write_value(4, "x0", (int)(data[a].RPM)); // Writes digital value of RPM
    nextion_write_value(4, "x2", (int)(data[a].mcc_voltage)); // Writes digital value of voltage
    nextion_write_value(4, "x3", (int)(data[a].mcc_current)); // Writes digital value of current

    if(data[a].torque < 0) {data[a].torque = 0;} // Makes any negative value equals to 0
    if(data[a].torque > temp_max_torque) {temp_max_torque = data[a].torque;}

    if(data[a].RPM < 0) {data[a].RPM = 0;} // Makes any negative value equals to 0
    if(data[a].RPM > temp_max_RPM) {temp_max_RPM = data[a].RPM;}

    //****************************************************
    // Graphs values using line segments
    // Torque
    torque_old_percentage = ((float)data[a].torque - torque_old_min) / (torque_old_max - torque_old_min);
    torque_y_new_new = (((new_max - new_min) * torque_old_percentage) + new_min);
    torque_y_new_new_new = 480-torque_y_new_new;

    nextion_draw_line(torque_x_last, torque_y_last, 6*a, torque_y_new_new_new, "BLUE");

    torque_x_last = 6*a;
    torque_y_last = torque_y_new_new_new;
    
    // RPM
    RPM_old_percentage = ((float)data[a].RPM - RPM_old_min) / (RPM_old_max - RPM_old_min);
    RPM_y_new_new = (((new_max - new_min) * RPM_old_percentage) + new_min);
    RPM_y_new_new_new = 480-RPM_y_new_new;

    nextion_draw_line(RPM_x_last, RPM_y_last, 6*a, RPM_y_new_new_new, "RED");
    RPM_x_last = 6*a;
    RPM_y_last = RPM_y_new_new_new;
    
    //****************************************************

    nextion_write_value(4, "j0", a+1); // Writes value to slider to display progress

    loop_counter++;
    if(loop_counter == 10)
    {
      loop_counter2--;
      
      GLS_control(loop_counter2);

      loop_counter = 0;
    }
    
  }

  motor(0); // Turns off motor
  eeprom_write_word((uint16_t *)MAX_TORQUE_ADDRESS, temp_max_torque); // saves the max torque reading in eeprom
  eeprom_write_word((uint16_t *)MAX_RPM_ADDRESS, temp_max_RPM); // saves the max RPM reading in eeprom
  printf("page%d.b%d.pco=%ld%c%c%c", 4, 3, 65535, 0xFF,0xFF,0xFF); // Makes the results button visible (changes font color to white)

}

void manualTest() // Gathers and displays information from the optocoupler and loadcell
{
  HX711_tare(10); // tares scale, use no weight
  HX711_set_scale(calibration_factor); //Adjust to this calibration factor
  uint16_t RPM_temp = 0;
  uint32_t torque_temp = 0;

  while(state_test)
  { 
    if(commandComplete) {nextion_touch_event_detected();} // checks if a touch event has occurred. If yes runs function
    
    // Reads input values
    torque_temp = (HX711_get_mean_units(HX711_times)*1000);
    RPM_temp = RPMReadValue(); // gives the RPM value

    // Writes digital values to Display
    nextion_write_value(2, "x1", (int)(torque_temp)); // Writes digital value of torque
    nextion_write_value(2, "x0", (int)RPM_temp); // Writes digital value of RPM

    if(torque_temp < 0) {torque_temp = 0;}
    if(RPM_temp < 0) {RPM_temp = 0;}
    
    // Graphs values on waveform
    for(int i = 0; i < 1; i++)
    {
      nextion_waveform_write_value(1, 2, (int)(torque_temp)); // Graphs torque on waveform
      nextion_waveform_write_value(1, 1, (int)RPM_temp); // Graphs torque on waveform
    }
  
  }
  
}

void trendline_RPM_torque(float *a_p, float *b_p)
{
  float a = 0, b = 0, sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;

  // Step 1 (calculating sums and stuff)
  for (int i = 0; i < ARRAY_SIZE; i++)
  {
    sum_x = sum_x + data[i].torque;
    sum_y = sum_y + data[i].RPM;
    sum_xy = sum_xy + (data[i].torque * data[i].RPM);
    sum_x2 = sum_x2 + (data[i].torque * data[i].torque);
  }

  // Step 2 (determining coefficients)
  a = (ARRAY_SIZE * sum_xy - sum_x * sum_y) / (ARRAY_SIZE * sum_x2 - sum_x * sum_x);
  b = (sum_y - a * sum_x) / ARRAY_SIZE;

  *a_p = a;
  *b_p = b;

  // Step 3 (write equation on display)
  printf("%s.txt=\"y = -%dx + %d\"%c%c%c", "t7", (int)a, (int)b, 0xFF,0xFF,0xFF);

}

void trendline_current_torque(float *a_p)
{
  float a = 0, b = 0, sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;

  // Step 1 (calculating sums and stuff)
  for (int i = 0; i < ARRAY_SIZE; i++)
  {
    sum_x = sum_x + data[i].torque;
    sum_y = sum_y + data[i].mcc_current;
    sum_xy = sum_xy + (data[i].torque * data[i].mcc_current);
    sum_x2 = sum_x2 + (data[i].torque * data[i].torque);
  }

  // Step 2 (determining coefficients)
  a = (ARRAY_SIZE * sum_xy - sum_x * sum_y) / (ARRAY_SIZE * sum_x2 - sum_x * sum_x);
  b = (sum_y - a * sum_x) / ARRAY_SIZE;

  *a_p = a;

  // Step 3 (write equation on display)
  printf("%s.txt=\"y = %dx + %d\"%c%c%c", "t9", (int)a, (int)b, 0xFF,0xFF,0xFF);

}

void trendline_power_torque()
{
  float a = 0, b = 0, c = 0, sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2y = 0, sum_x2 = 0, sum_x3 = 0, sum_x4 = 0;

  // Step 1 (collecting the data)
  for (int i = 0; i < ARRAY_SIZE; i++)
  {
    sum_x = sum_x + data[i].torque;
    sum_y = sum_y + (data[i].mcc_voltage * data[i].mcc_current);
    sum_xy = sum_xy + (data[i].torque * (data[i].mcc_voltage * data[i].mcc_current));
    sum_x2y = sum_x2y + (data[i].torque * data[i].torque * (data[i].mcc_voltage * data[i].mcc_current));
    sum_x2 = sum_x2 + (data[i].torque * data[i].torque);
    sum_x3 = sum_x3 + (data[i].torque * data[i].torque * data[i].torque);
    sum_x4 = sum_x4 + (data[i].torque * data[i].torque * data[i].torque * data[i].torque);
  }

  // Step 2 (calculating values to use in formulas)
  float xx = (sum_x2) - ((sum_x * sum_x) / ARRAY_SIZE);
  float xy = (sum_xy) - ((sum_x * sum_y) / ARRAY_SIZE);
  float xx2 = (sum_x3) - ((sum_x2 * sum_x) / ARRAY_SIZE);
  float x2y = (sum_x2y) - ((sum_x2 * sum_y) / ARRAY_SIZE);
  float x2x2 = (sum_x4) - ((sum_x2 * sum_x2) / ARRAY_SIZE);

  // Step 3 (determining coefficients)
  a = ((x2y * xx) - (xy * xx2)) / ((xx * x2x2) - (xx2 * xx2));
  b = ((xy * x2x2) - (x2y * xx2)) / ((xx * x2x2) - (xx2 * xx2));
  c = (sum_y / ARRAY_SIZE) - (b * (sum_x / ARRAY_SIZE)) - (a * (sum_x2 / ARRAY_SIZE));

  // Step 4 (write equation to display)
  printf("%s.txt=\"y = %dx^2 + %dx + %d\"%c%c%c", "t11", (int)a, (int)b, (int)c, 0xFF,0xFF,0xFF);

}


void motor_constants(float RPM_torque_b, float RPM_torque_a, float current_torque_a)
{

  // Step 1 (Calculate motor constants)
  float input_voltage_sum = 0;
  for (int i = 0; i < ARRAY_SIZE; i++)
  {
    input_voltage_sum = input_voltage_sum + data[i].mcc_voltage;
  }
  float input_voltage_average = input_voltage_sum / ARRAY_SIZE;

  float omega_no_load = RPM_torque_b * 0.1047198;                                       // 1. omega_no_load
  float stall_torque = -((RPM_torque_b * 0.1047198) / (RPM_torque_a * 0.1047198));      // 2. stall_torque
  float torque_constant = 1 / current_torque_a;                                         // 3. torque_constant
  float back_EMF_constant = input_voltage_average / omega_no_load;                      // 4. back_EMF_constant
  float armature_resistance = (input_voltage_average * torque_constant) / stall_torque; // 5. armature_resistance

  // Step 2 (write constants to display)
  nextion_write_value(5, "x4", (int)(omega_no_load*100)); // No load RPM 
  nextion_write_value(5, "x1", (int)(stall_torque*100)); // Stall Torque
  nextion_write_value(5, "x0", (int)(torque_constant*100)); // Torque constant 
  nextion_write_value(5, "x2", (int)(back_EMF_constant*100)); // Back EMF 
  nextion_write_value(5, "x3", (int)(armature_resistance*100)); // Armature resistance

}

void GLS_control(int numb_lightbulbs_on)
{
  
  PORTD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7)); 
  PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5)); 
  PORTC &= ~((1 << PC5));

  if(numb_lightbulbs_on == 0) // No bulbs (short circuit)
  { 
    PORTD |= (1 << PD2);
  }
    
  if((numb_lightbulbs_on < 6) && (numb_lightbulbs_on > 0)) // Bulbs 1-5
  { 
    PORTD |= (1 << (numb_lightbulbs_on+2));
  }

  if(numb_lightbulbs_on == 6) // Bulb 6
  { 
    PORTC |= (1 << PC5);
  }
    
  if ((numb_lightbulbs_on >= 7) && (numb_lightbulbs_on < 12)) // Bulbs 7-11
  { 
    PORTB |= (1 << (numb_lightbulbs_on-6));
  }

}

void GLS_init()
{
  // Data Direction Registers:
  DDRD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7); // make pins PD2-PD7 as outputs
  DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5); // make pins PB1-PB5 as outputs
  DDRC |= (1 << PC5); // make pin PC5 as output

  // Setting all light pins initially as low (0 Volts)
  PORTD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7)); 
  PORTB &= ~((1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5)); 
  PORTC &= ~((1 << PC5));
}

void optocoupler_init()
{
  // Data Direction Registers:
  DDRC &= ~(1 << PB0); // Sets PB0 to be the input pin
  PORTB |= (1 << PB0); // Enables the pull up on pin PB0

  // Interrupt registers
  TCCR1A = 0x00; // Timer/Counter control register A (Normal port operation, OC1A/OC1B disconnected)
  TCCR1B |= (1 << CS12) | (1 << CS10) | (1 << ICNC1); // Timer/Counter control register B (1024 prescaler, noise canceler)

  TIMSK1 |= (1 << ICIE1) | (1 << TOIE1); // Setting up timer/counter1 interrupt mask register (Input Capture Interrupt Enable, Overflow Interrupt Enable)



}

void timer0_init()
{
  TCCR0A |= (1 << WGM01); // Set the timer mode to CTC
  TCCR0B |= (1 << CS01) | (1 << CS00); // Sets prescaler to 64
  OCR0A = ((16000000 / 64) * 0.001) - 1; // set the top value to be 1 ms (Value when overflow flag is triggered)
}

void ADC_init()
{
  DDRC &= ~((1 << MCC_voltage_adc_pin)|(1 << MCC_voltage_adc_pin)); //assign input pins
  DDRC |= (1 << MCC_motor_enble_pin); //assign output pins
}

void motor(uint8_t run) // 1 = start, any other value = stop motor
{
  if(run != 1)
  {
    PORTC &= ~(1 << MCC_motor_enble_pin); //stops motor
    return;
  }

  PORTC |= (1 << MCC_motor_enble_pin); //starts motor
}

uint16_t read_main_voltage() //output voltage ADC 
{
  ADMUX &= 0x00; //clear previous channels
  ADCSRA &= 0x00;
  _delay_ms(1000);

  ADMUX |= (1 << REFS0); // VREF = AVcc
  _delay_ms(1000);

  ADCSRA |= (1 << ADPS0)|(1 << ADPS1)|(1 << ADPS2)|(1 << ADEN); // division factor 128 and enables adc conversion
  ADCSRA |= (1<<ADSC);
  while ( (ADCSRA & (1<<ADSC)) ); // Do nothing

  return ADC; 
}

uint16_t read_current_adc() // !output current ADC  (dosent work)
{
  ADC = 0;
  ADCSRA &= 0x00;
  ADMUX &= 0x00; //clear previous channels and refs
  ADMUX |= 0b11000001; // VREF = 1.1v and use ADC1
  ADCSRA |= (1 << ADPS0)|(1 << ADPS1)|(1 << ADPS2)|(1 << ADEN); // division factor 128 and enables adc conversion

  ADCSRA |= (1<<ADSC); //first try always fails
  while ( (ADCSRA & (1<<ADSC)) ){};  // Do nothing
  _delay_ms(100);
  
  ADCSRA |= (1<<ADSC);  //second is good
  while ( (ADCSRA & (1<<ADSC)) ){}; // Do nothing
  
  return ADC; 
  
}

void delay_milliseconds(unsigned int milliseconds)
{
  for(int i = 0; i < milliseconds; i++)
  {
    TIFR0 = (1 << OCF0A); // Resets the overflow flag
    
    while((TIFR0 & (1 << OCF0A)) == 0) // Wait for the overflow event
    {

    }
  }
}

uint16_t RPMReadValue() // gives the RPM value
{
  uint16_t RPM = 0;
  
  if (overflows < 1)
  {
    RPM = (60000/(counter*(1024/16000.0f)*(float)HOLES)); // gives the RPM value
    
    if (RPM > 200000)
    {
      RPM = 0;
    }

  }
  else
  {
    RPM = 0;
    
  }

  return RPM;
}

// ################################################################################################### //
