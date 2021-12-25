/************************************************/
/* Author      : Ahmed Fathy Abd El-Qadir	    */
/* Last Update : 24-12-2021                     */
/* Version     : V 1.0                          */
/************************************************/

#ifndef KISS_PRIVATE_H
#define KISS_PRIVATE_H

#include "STD_TYPES.h"

/* Special Characters */
#define FEND   0xC0   // Frame End flag
#define FESC   0xDB   // Frame Escape flag
#define TFEND  0xDC   // Transposed Frame End flag
#define TFESC  0xDD   // Transposed Frame Escape flag


//////////////////////////////////  KISS Frame Comand Code ///////////////////////////////////////////

/* 1- It is contains data that should be sent out of the TNC. 
      The maximum number of bytes is determined by the amount of memory in the TNC.*/ 
#define KISS_DATA_FRAME		 0x00 
  
/* 2- The amount of time to wait between keying the transmitter and beginning
      to send data (in 10 ms units)                                           */
#define KISS_TXDELAY		 0x01  

/* 3- The next byte is the slot interval in 10 ms units. The default is 10 (i.e., 100ms). */
#define KISS_PERSISTENCE	 0x02    
 
/* 4- The next byte is the slot interval in 10 ms units.The default is 10 (i.e., 100ms). */
#define KISS_SLOT_TIME		 0x03   

/* 5- The next byte is the time to hold up the TX after the FCS has been sent, 
      in 10 ms units. This command is obsolete, and is included here only for 
      compatibility with some existing implementations.                       */  
#define KISS_TXTAIL		     0x04   

/* 6- The next byte is 0 for half duplex, nonzero for full duplex.
      The default is 0 (i.e., half duplex).                        */ 
#define KISS_FULL_DUPLEX	 0x05   
 
 /* 7- Specific for each TNC. In the TNC-1, this command sets the modem speed.
       Other implementations may use this function for other hardware-specific functions. */ 
#define KISS_SET_HARDWARE	 0x06    

/* 8- Exit KISS and return control to a higher-level program. This is useful
      only when KISS is incorporated into the TNC along with other applications. */
#define KISS_RETURN		     0xff     
 
///////////////////////////////////////////////////////////////////////////////////////////////////////


#endif
