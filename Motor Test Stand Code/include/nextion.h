/*
 * nextion.h
 *
 *  Created on: April 23, 2024
 *      Author: Tobias
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

#ifndef NEXTION_H_INCLUDED // checks whether NEXTION_H_INCLUDED is not declared.
#define NEXTION_H_INCLUDED // Will declare NEXTION_H_INCLUDED once #ifndef generates true.

extern volatile unsigned char commandbuffer[8];
extern volatile unsigned char commandComplete;

//function headers 
void nextion_init();
void nextion_display_page(int page_number);
void nextion_waveform_clear_channel(int Component_id);
void nextion_stop_execution(); 
void nextion_resume_execution();
void nextion_draw_line(int x_last, int y_last, int x_new, int y_new, char* line_color);

void nextion_write_value(int page_number, char* component_name, int value);
void nextion_write_text(char* component_name, char* text);
void nextion_waveform_write_value(int Component_id, int channel_number, int value);
void nextion_change_data_scaling(int page_number, char* component_name, int value);

uint32_t nextion_read_value(int page_number, char* component_name);

#endif // Is to know the scope of #ifndef i.e end of #ifndef.

