/************************************************/
/* Author      : Ahmed Fathy Abd El-Qadir	    */
/* Last Update : 24-12-2021                     */
/* Version     : V 1.0                          */
/************************************************/

#ifndef KISS_INTERFACE_H
#define KISS_INTERFACE_H

#include "STD_TYPES.h"
#include "KISS_Private.h"


typedef struct
{
	u32 Data_Length;      // it contain the frame length
	u8  Command_Field;    // it contain one of the command defined in private.h file
	u8 *PtrToFrame;        // pointer to the first address in the memory for the frame 
	u16 Stuffing_bytes;   // it contain no.of Stuffing bytes to the frame
	u16 Destuffing_bytes; // it contain no.of Destuffing bytes from the frame
	
} Kiss_Packet_Frame;     

/************************************** Functions Prototype ********************************************/
/********** For the Transmitted Frame *******************/
u8* Construct_Frame(u8* data,u32*data_len);              // function to build and construct the frame
void Add_FEND(Kiss_Packet_Frame *KissFrame);             // function to add FEND to start and end of the frame
void Replace_FEND(Kiss_Packet_Frame *KissFrame,u32 i);   // function replace (FEND) with (FESC + TFEND) 
void Replace_FESC(Kiss_Packet_Frame *KissFrame,u32 i);   // function replace (FESC) with (FESC + TFESC) 

/********** For the Recieved Frame **********************/
void Delete_FEND(Kiss_Packet_Frame *KissFrame);          // function to delete FEND from start and end of the frame 
void Extract_FEND(Kiss_Packet_Frame *KissFrame);         // function to extract FEND from (FESC+TFEND)
void Extract_FESC(Kiss_Packet_Frame *KissFrame);         // function to extract FESC from (FESC+TFESC) 
/**********************************************************************************************************/

