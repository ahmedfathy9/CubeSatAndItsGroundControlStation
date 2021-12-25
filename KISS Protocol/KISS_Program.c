/************************************************/
/* Author      : Ahmed Fathy Abd El-Qadir	    */
/* Last Update : 24-12-2021                     */
/* Version     : V 1.0                          */
/************************************************/

#include <stdlib.h>
#include <stdio.h>   
#include <string.h>
#include"BIT_MATH.h"
#include "STD_TYPES.h"
#include "KISS_Private.h"
#include "KISS_Interface.h"


///////////////////////// function to build and construct all frame ////////////////////////////////
u8* Construct_Frame(u8* data, u32*data_len)  
{
    u8* frame;
    Kiss_Packet_Frame* KissFrame;      // pointer to structure
	KissFrame =(Kiss_Packet_Frame*) malloc(sizeof(Kiss_Packet_Frame)); // Kiss_Packet_Frame memory allocate 
    
    KissFrame->Data_Length = *data_len;
    KissFrame->PtrtoData = (u8*) malloc(*data_len);
    memcpy(KissFrame->PtrtoData,data,*data_len);     // copy incoming data to pointer to data address  
	KissFrame->Command_Field=0;        // initialize Command_Field to zero
	KissFrame->Stuffing_bytes = 0;	   // initialize Stuffing bytes to zero
    KissFrame->Destuffing_bytes = 0;   // initialize Destuffing bytes to zero
    

    u16 CRC=CRC16_u16CCITTFalse(TXframe->PtrtoFrame, TXframe->FrameLength);
    TXframe->PtrtoFrame=(u8*)realloc((TXframe->PtrtoFrame),(TXframe->FrameLength+2)*sizeof(u8));
    TXframe->PtrtoFrame[TXframe->FrameLength]=(CRC & 0xff);
    TXframe->PtrtoFrame[TXframe->FrameLength+1]=(CRC>>8 & 0xff);
    TXframe->FrameLength+=2;
	
	
	/*Replace FEND withe (FESC + TFEND) and FESC withe (FESC + TFESC) in the data */
    for(u32 i=0;i<KissFrame->Data_Length;i++)
    {
        if(KissFrame->PtrtoData[i]==FEND)
          {
			Replace_FEND(KissFrame,i);
            i++; // to skip the next byte
          }
    }
	
	  for(u32 i=0;i<KissFrame->Data_Length;i++)
    {
        if(KissFrame->PtrtoData[i]==FESC)
          {		  
			Replace_FESC(KissFrame,i);
            i++; // to skip the next byte
          }
    }
	
	
	Add_FEND(KissFrame) ;   // add FEND to first and last byte of hte frame
	
	
	*data_len=KissFrame->Data_Length;
    frame=(u8*)malloc(*data_len);
    memcpy(frame,KissFrame->PtrtoData,*data_len);
    free(KissFrame->PtrtoData); // remove data from the memory 
    free(KissFrame);            // remove data from the memory 
    free(data);                 // remove data from the memory 
    return frame;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////// function replace (FESC) with (FESC and TFEND) /////////////////////////
void Replace_FEND(Kiss_Packet_Frame *KissFrame,u32 i) // function replace (FEND) with (FESC and TFEND) 
{  
	/* using realloc function to increase the array frame size */
	KissFrame->PtrtoData=(u8*)realloc((KissFrame->PtrtoData),(KissFrame->Data_Length+1)*sizeof(u8));
	KissFrame->Data_Length++;        // increment frame length by one for adding one bytes
	KissFrame->Stuffing_bytes++;	 // increment stuffing bytes for adding one byte
		
	/* shift right the array by one byte for a position we want to stuffing an element */
	memcpy(KissFrame->PtrtoData+i+1, KissFrame->PtrtoData+i, KissFrame->Data_Length-i-1);  // ex: ahmed_FEND_fathy       
			
	/*replace frame end flag (FEND=C0) with two flags (FESC=DB && TFEND=DC) */
	KissFrame->PtrtoData[i]=FESC;
	KissFrame->PtrtoData[i+1]=TFEND;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////// function replace (FESC) with (FESC and TFESC) /////////////////////////
void Replace_FESC(Kiss_Packet_Frame *KissFrame,u32 i)  
{
	/* using realloc function to increase the array frame size */
	KissFrame->PtrtoData=(u8*)realloc((KissFrame->PtrtoData),(KissFrame->Data_Length+1)*sizeof(u8));
	KissFrame->Data_Length++;        // increment frame length by one for adding one bytes
	KissFrame->Stuffing_bytes++;	 // increment stuffing bytes for adding one byte
	
	/* shift right the array by one byte for a position we want to stuffing an element */
	memcpy(KissFrame->PtrtoData+i+1, KissFrame->PtrtoData+i, KissFrame->Data_Length-i-1);  // ex: ahmed_FEND_fathy       
			
	/*replace frame end flag (FEND=C0) with two flags (FESC=DB && TFEND=DC) */
	KissFrame->PtrtoData[i]=FESC;
	KissFrame->PtrtoData[i+1]=TFESC;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////// function to add FEND at the last byte of the frame ///////////////////////
void Add_FEND(Kiss_Packet_Frame *KissFrame)           
{
	 /* add two bytes for C0 at the begining and at the end of the frame*/
    KissFrame->PtrtoData=(u8*)realloc((KissFrame->PtrtoData),(KissFrame->Data_Length+2)*sizeof(u8));
    KissFrame->Data_Length+=2;     //increment frame length by two for adding two bytes
    KissFrame->Stuffing_bytes+=2;  // increment stuffing bytes for adding two bytes
    memcpy(KissFrame->PtrtoData+1,KissFrame->PtrtoData,KissFrame->Data_Length-2); // shift right the array one byte
    KissFrame->PtrtoData[0]=FEND;     // put C0 at the frame start 
	KissFrame->PtrtoData[(KissFrame->Data_Length)-1]=FEND; // put C0 at the frame end
}
///////////////////////////////////////////////////////////////////////////////////////////////////////


