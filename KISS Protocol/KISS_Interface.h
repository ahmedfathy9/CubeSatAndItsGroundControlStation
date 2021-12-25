/************************************************/
/* Author      : Ahmed Fathy Abd El-Qadir	*/
/* Last Update : 24-12-2021                     */
/* Version     : V 1.0                          */
/************************************************/

#ifndef KISS_INTERFACE_H
#define KISS_INTERFACE_H

#include "STD_TYPES.h"
#include "KISS_Private.h"

/*
Packet Format : FEND(8-bit) -- Command_Field(8-bit) -- Data Field(Variable) -- FEND(8-bit)
*/
typedef struct
{
	u32 Data_Length;
	u8  Command_Field;
	u8 *PtrtoData;       // pointer to the first address in the memory for the data field 
	u16 Stuffing_bytes;
	u16 Destuffing_bytes;
	
} Kiss_Packet_Frame;     

/******** Functions Declaration ********/
u8* Construct_Frame(u8* data,u32*data_len);        // function to build and construct all frame
void Add_FEND(Kiss_Packet_Frame *KissFrame);           // function to add FEND at the last byte of the frame 
void Replace_FEND(Kiss_Packet_Frame *KissFrame,u32 i); // function replace (FEND) with (FESC and TFEND) 
void Replace_FESC(Kiss_Packet_Frame *KissFrame,u32 i); // function replace (FESC) with (FESC and TFESC) 


