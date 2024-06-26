/*
 * nextion.c
 *
 * Component IDs: 
 *  - number: "n"
 *  - float: "x"
 *  - Progress Bar: "j"
 *  - Gauge: "z"
 *  - Slider: "h"
 *  - Hotspot: "m"
 *  - Button: "b"
 *  - Waveform: "s"
 *  
 * Values send over usart: (Touch Event)
 *  - We know it's a touch even when commandbuffer[0] == 0x65 (Command/communication id)
 *  - commandbuffer[1]: Page number
 *  - commandbuffer[2]: Component id
 *  - commandbuffer[3]: Event type (0x01 = Press and 0x00 = release)
 * 
 * Nextion Color constants: (Can be used when drawing colors)
 *  - "BLACK" (566 color value: 0)
 *  - "BLUE" (566 color value: 31)
 *  - "BROWN" (566 color value: 48192)
 *  - "GREEN" (566 color value: 2016)
 *  - "YELLOW" (566 color value: 65504)
 *  - "RED" (566 color value: 63488)
 *  - "GRAY" (566 color value: 33840)
 *  - "WHITE" (566 color value: 65535)
 * 
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h> 

#include "usart.h"
#include "nextion.h"

#ifndef CPU_CLOCK
#define CPU_CLOCK 16000000UL
#endif
#ifndef BAUD
#define BAUD 9600
#endif
#define NUMBER_STRING 1001 // 1001 is just a random number. The variable is used to check if the number input in the number array is a string.

//-------GLOBAL-VARIABLES------
volatile unsigned char commandbuffer[8] = {0};
volatile unsigned char bufferIndex = 0;
volatile unsigned char commandStarted = 0;
volatile unsigned char commandComplete = 0;
volatile unsigned char endflagsRead = 0;
volatile unsigned char waitForInterrupts = 0; 

//################################________Commands________############################################

// Initialize communication with the Nxtion display 
void nextion_init() 
{
  uart_init(); //initialize communication with PC - debugging
	io_redirect(); // redirect input and output to the communication

	UCSR0B |= 1 << RXCIE0; //enable receive interrupt
	sei(); //Enable interrupts
	_delay_ms(1000);

	// Resets component values on page 3
	nextion_write_value(3, "bt0", 1);
	nextion_write_value(3, "h0", 0);
	nextion_write_value(3, "n0", 0);
  printf("page%d.n%d.pco=%ld%c%c%c", 3, 0, 65535L, 0xFF,0xFF,0xFF);

	// Resets component values on page 2
	nextion_write_value(2, "bt0", 0);

	printf("page 0%c%c%c",255,255,255); // Goes to page 0
}

// Display page on the nextion display
void nextion_display_page(int page_number)
{
  printf("page %d%c%c%c", page_number, 0xFF,0xFF,0xFF); 
}

// Clear data waveform channel on the display
void nextion_waveform_clear_channel(int Component_id)
{
	printf("cle %d,255%c%c%c", Component_id, 0xFF,0xFF,0xFF);
}

// Stop execution of instructions received from Serial (USART)
void nextion_stop_execution() 
{
	printf("com_stop%c%c%c",255,255,255);
}

// Resume execution of instructions received from Serial (USART)
void nextion_resume_execution() 
{
	printf("com_start%c%c%c",255,255,255);
}

// Draw line between two points on the display
void nextion_draw_line(int x_last, int y_last, int x_new, int y_new, char* line_color)
{
	printf("line %d,%d,%d,%d,%s%c%c%c", x_last, y_last, x_new, y_new, line_color, 0xFF,0xFF,0xFF);
}

//################################________Writing_Data________############################################

// Write value to a component on the display
void nextion_write_value(int page_number, char* component_name, int value)
{
  printf("page%d.%s.val=%d%c%c%c", page_number, component_name, value, 0xFF,0xFF,0xFF);
}

// Write text to a component on the display
void nextion_write_text(char* component_name, char* text) 
{
  printf("%s.txt=\"%s\"%c%c%c", component_name, text, 0xFF,0xFF,0xFF);
}

// Write value to waveform channel on the display
void nextion_waveform_write_value(int Component_id, int channel_number, int value)
{
  printf("add %d,%d,%d%c%c%c", Component_id, channel_number-1, value, 0xFF,0xFF,0xFF); // Channel numbers start at 0, so channel 1 = 0
}

// Change data scaling of component (waveform)
void nextion_change_data_scaling(int page_number, char* component_name, int value)
{
  printf("page%d.%s.dis=%d%c%c%c", page_number, component_name, value, 0xFF,0xFF,0xFF);
}

//################################________Reading_Data________############################################

// Read value from a component on the display
uint32_t nextion_read_value(int page_number, char* component_name)  
{
	uint32_t readValue = 0; // 32 bit unsigned integer
	
	// Gets value from component 
	printf("get page%d.%s.val%c%c%c", page_number, component_name, 0xFF,0xFF,0xFF);
	
	waitForInterrupts = 1; 
	while(waitForInterrupts); // Waits for the for the display to stop sending data over USART
	
	// Checks if the first package/string stored in the array is the "indicator" 0x71 and that the last 3 stings are 0xFF, which is used to end a command. 
	if(commandbuffer[0] == (int)0x71 && commandbuffer[5] == (int)0xFF && commandbuffer[6] == (int)0xFF && commandbuffer[7] == (int)0xFF)//Just to make completely sure i check if first is a number and we end with 0xFF 3 times. This is a complete number return
	{
		readValue = commandbuffer[1] | (commandbuffer[2] << 8) | (commandbuffer[3] << 16) | (commandbuffer[4] << 24); // I use left shifting "<<" and bitwise or "|" To combine the 4 packages into one 32 bit string that is then put into the variable "readvalue"
	}

	commandComplete = 0;

	return readValue; // Returns a 32bit unsigned integer
}

//################################________Touch_Event________############################################

ISR(USART_RX_vect)
{
	commandbuffer[bufferIndex] = UDR0;

	if(commandbuffer[bufferIndex] == 0x65)
	{
		commandStarted = 1;
	}
	
	if(commandbuffer[bufferIndex] == 0x71)
	{
		commandStarted = 1;
	}
	
	if(commandbuffer[bufferIndex] == 0xFF)
	{
		endflagsRead++;
		if(endflagsRead == 3)
		{
			commandComplete = 1;
			endflagsRead = 0;
			commandStarted = 0;
			bufferIndex = 0;
			waitForInterrupts = 0;
		}
	}
	
	if(commandStarted == 1)
	{
		bufferIndex++;//Note character limit
	}

}



