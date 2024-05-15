/*
THIS IS FOR CALIBRATING SCALE
 
 
THESE ARE ACTUALLY THE CONNECTIONS!!!
ARDUINO      HX711
 A2    ->    CLK
 A3    ->    DO
 3V    ->    3V
 GND   ->    GND
*/

#include "HX711.h"
#include <util/delay.h>
#include <stdio.h>
#include <avr/io.h>
#include "usart.h"




float calibration_factor = 1100; // gets changed can be positive or negative, depends on setup

float change; // 


void init(void) {
 uart_init();
 io_redirect();
 HX711_init(128);
}






int main(void) {
    init();
   
     HX711_tare(10); // tares scale, use no weight
//---------------------------------------------------------------ONCE-----------------------------------------------------------------//
     HX711_set_scale(calibration_factor); //Adjust to this calibration factor
        printf("Reading: %f g calibration_factor: %f\n", HX711_get_mean_units(1), calibration_factor);
       scanf("%f",&change); // adjust to weight on scale
       int Initial = change;
//------------------------------------------------------------------------------------------------------------------------------------//
    while(1) {
        HX711_set_scale(calibration_factor); //Adjust to this calibration factor
        float Reading =(HX711_get_mean_units(1)+ Initial);
        printf("Reading: %f g Adjusted Reading: %f calibration_factor: %f\n", HX711_get_mean_units(1), Reading, calibration_factor);
       scanf("%f",&change); // adjust to weight on scale
       if(change != 0)
       {
       calibration_factor /= change;
       }
       // _delay_ms(500);
    }
}