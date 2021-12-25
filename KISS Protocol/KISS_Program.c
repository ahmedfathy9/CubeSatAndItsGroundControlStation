/************************************************/
/* Author      : Ahmed Fathy Abd El-Qadir	    */
/* Last Update : 24-12-2021                     */
/* Version     : V 1.0                          */
/************************************************/

#include <stdlib.h>
#include <stdio.h>   
#include <string.h>
#include "BIT_MATH.h"
#include "STD_TYPES.h"
#include "KISS_Private.h"
#include "KISS_Interface.h"


///////////////////////// function to build and construct all frame ////////////////////////////////
u8* Construct_Frame(u8* data, u32*data_len)  
{
	/*
    Packet Format : FEND(8-bit) -- Command_Field(8-bit) -- Data Field(Variable) -- FEND(8-bit)
   */
    Kiss_Packet_Frame* KissFrame;      // pointer to Kiss_Packet_Frame structure
	KissFrame =(Kiss_Packet_Frame*) malloc(sizeof(Kiss_Packet_Frame)); // Kiss_Packet_Frame memory allocate 
    
    KissFrame->Data_Length = *data_len;
    KissFrame->PtrToFrame = (u8*) malloc(KissFrame->Data_Length+1);  //data length +1 byte for Command field
    memcpy(KissFrame->PtrToFrame+1,data,KissFrame->Data_Length); // copy incoming data to pointer to data address  
	KissFrame->Command_Field=0;  	                   // initialize Command_Field to zero
	KissFrame->Stuffing_bytes = 0;	  		     	   // initialize Stuffing bytes to zero
	KissFrame->Destuffing_bytes = 0;  			       // initialize Destuffing bytes to zero
	/*add command field for the first byte */
	KissFrame->PtrToFrame[0]= KissFrame->Command_Field
	/*increment the stuffing bytes and data length by one byte for command field */
	KissFrame->Stuffing_bytes +=1;
    KissFrame->Data_Length +=1;	
	
	/*Replace FEND withe (FESC + TFEND) and FESC withe (FESC + TFESC) in the data */
    for(u32 i=0;i<KissFrame->Data_Length;i++)
    {
        if(KissFrame->PtrToFrame[i]==FEND)
          {
			Replace_FEND(KissFrame,i);   // Calling the Replace_FEND Function to replace FEND with (FESC+TFEND)
            i++;                         // to skip the next byte
          }
    }
	
	  for(u32 i=0;i<KissFrame->Data_Length;i++)
    {
        if(KissFrame->PtrToFrame[i]==FESC)
          {		  
			Replace_FESC(KissFrame,i);   // Calling the Replace_FESC Function to replace FESC with (FESC+TFESC)
            i++;						 // to skip the next byte
          }
    }
	
	Add_FEND(KissFrame) ;   // add FEND to first and last byte of hte frame
	
	/* Return the Frame After Construction*/
	u8* frame;
    frame=(u8*)malloc(KissFrame->Data_Length);
    memcpy(frame,KissFrame->PtrToFrame,KissFrame->Data_Length);
    free(KissFrame->PtrToFrame); // remove PtrToFrame from the memory 
    free(KissFrame);            // remove KissFrame from the memory 
    free(data);                 // remove data from the memory 
    return frame;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////// function replace (FESC) with (FESC and TFEND) /////////////////////////
void Replace_FEND(Kiss_Packet_Frame *KissFrame,u32 i)  
{  
    //a h m e d _FEND_     f     a t h y     -->11 Byte  
	//0 1 2 3 4   i      (i+1)   7 8 9 10 11 
	//a h m e d _FESC_  _TFEND_  f a t h  y  -->12 Byte
	
	/* using realloc function to increase the array frame size */
	KissFrame->PtrToFrame=(u8*)realloc((KissFrame->PtrToFrame),(KissFrame->Data_Length+1)*sizeof(u8));
	KissFrame->Data_Length++;        // increment frame length by one for adding one bytes
	KissFrame->Stuffing_bytes++;	 // increment stuffing bytes for adding one byte
	/* shift right by one byte form a position we want to stuff an element */
	memcpy(KissFrame->PtrToFrame+i+1, KissFrame->PtrToFrame+i, KissFrame->Data_Length-i-1);
	/*replace (FEND) with (FESC=DB + TFEND=DC) */
	KissFrame->PtrToFrame[i]=FESC;
	KissFrame->PtrToFrame[i+1]=TFEND;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////// function replace (FESC) with (FESC and TFESC) /////////////////////////
void Replace_FESC(Kiss_Packet_Frame *KissFrame,u32 i)  
{
	/* using realloc function to increase the array frame size */
	KissFrame->PtrToFrame=(u8*)realloc((KissFrame->PtrToFrame),(KissFrame->Data_Length+1)*sizeof(u8));
	KissFrame->Data_Length++;        // increment frame length by one for adding one bytes
	KissFrame->Stuffing_bytes++;	 // increment stuffing bytes for adding one byte
	
	/* shift right the array by one byte for a position we want to stuffing an element */
	memcpy(KissFrame->PtrToFrame+i+1, KissFrame->PtrToFrame+i, KissFrame->Data_Length-i-1);//a h m e d  ~  _FESC_  f a t h  y  -->11 Byte   
																						 //0 1 2 3 4  i  (i+1)   7 8 9 10 11 -->12 Byte      
			
	/*replace frame end flag (FEND=C0) with two flags (FESC=DB && TFEND=DC) */
	KissFrame->PtrToFrame[i]=FESC;
	KissFrame->PtrToFrame[i+1]=TFESC;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////// function to add FEND to start and end of the frame /////////////////////
void Add_FEND(Kiss_Packet_Frame *KissFrame)           
{
	//_Command _  a      h m e d f a t h y              -->11 Byte 
	//   0        1      2 3 4 5 6 7 8 9 10 11   12      
	//_FEND_  _Command _ a h m e d f a t h  y  _FEND_   -->13 Byte  
	
	/* add two bytes for C0 at the start and end of the frame*/
    KissFrame->PtrToFrame=(u8*)realloc((KissFrame->PtrToFrame),(KissFrame->Data_Length+2)*sizeof(u8));
	/* shift right the array one byte */
	memcpy(KissFrame->PtrToFrame+1,KissFrame->PtrToFrame,KissFrame->Data_Length+1); 
    /*increment frame length by two for adding two bytes */
    KissFrame->Data_Length+=2;  
	/* increment stuffing bytes for adding two bytes */
    KissFrame->Stuffing_bytes+=2;  
	/* put FEND at the frame start and at the frame end*/
    KissFrame->PtrToFrame[0]=FEND;     
	KissFrame->PtrToFrame[(KissFrame->Data_Length)-1]=FEND; 
}
///////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////// function to delete FEND from start and end of the frame ///////////////////////
void Delete_FEND(Kiss_Packet_Frame *KissFrame)
{   
    //_FEND_  _Command _ a h m e d f a t h  y  _FEND_   -->13 Byte  
	//   0        1      2 3 4 5 6 7 8 9 10 11   12     
    //_Command _  a      h m e d f a t h y              -->11 Byte  
	
	/* shifting left the frame one byte to delete start FEND*/
	memccpy(KissFrame->PtrToFrame, KissFrame->PtrToFrame+1,KissFrame->Data_Length-1) 
	/* deleted FEND at the end of the frame */
    KissFrame->PtrToFrame = (u8*) realloc((KissFrame->PtrToFrame), (KissFrame->Data_Length-2)*sizeof(u8));
	/* decrement the frame length by 2*/
    KissFrame->Data_Length-=2; 			 
	/* for destuffing two bytes of FEND at the end and start of the frame */
    KissFrame->Destuffing_bytes+=2;      
}
///////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// function to extract FEND from (FESC + TFEND) /////////////////////////////
void Extract_FEND(Kiss_Packet_Frame *KissFrame)
{
	for(u32 i=0;i<KissFrame->Data_Length;i++)
    {
		//a h m e d  _FESC_  _TFEND_  f a t h  y  -->12 Byte  
		//0 1 2 3 4     i     (i+1)   7 8 9 10 11 
		//a h m e d  _FEND_     f     a t h y     -->11 Byte
		
        if((KissFrame->PtrToFrame[i]==FESC) && (KissFrame->PtrToFrame[i+1]==TFEND))   
          {
			/*shift TFEND to left by one byte and remove FESC*/
			memccpy(KissFrame->PtrToFrame+i,KissFrame->PtrToFrame+i+1,KissFrame->Data_Length-i-1);
			/*removing the last free byte*/
			 KissFrame->PtrToFrame=(u8*)realloc((KissFrame->PtrToFrame),(KissFrame->Data_Length-1)*sizeof(u8));
			/*replace TFEND with FEND in byte no i */
			KissFrame->PtrToFrame[i]=FEND;
			/* decrement the frame length by 1 byte */
			KissFrame->Data_Length-=1; 		
            /* increment Destuffing bytes by 1 byte */ 			
            KissFrame->Destuffing_bytes+=1;     
          }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////// function to extract FESC from (FESC + TFESC) //////////////////////////////
void Extract_FESC(Kiss_Packet_Frame *KissFrame)
{	
	  for(u32 i=0;i<KissFrame->Data_Length;i++)
    {
        //a h m e d  _FESC_   _TFESC_  f a t h  y  -->12 Byte  
		//0 1 2 3 4     i      (i+1)   7 8 9 10 11 
		//a h m e d  _FESC_      f     a t h y     -->11 Byte	
		
        if((KissFrame->PtrToFrame[i]==FESC) && (KissFrame->PtrToFrame[i+1]==TFESC))   
          {
			/*shift TFESC to left by one byte and remove FESC*/
			memccpy(KissFrame->PtrToFrame+i,KissFrame->PtrToFrame+i+1,KissFrame->Data_Length-i-1);
			/*removing the last free byte*/
			 KissFrame->PtrToFrame=(u8*)realloc((KissFrame->PtrToFrame),(KissFrame->Data_Length-1)*sizeof(u8));
			/*replace TFESC with FESC in byte no i */
			KissFrame->PtrToFrame[i]=FESC;
			/* decrement the frame length by 1 byte */
			KissFrame->Data_Length-=1; 		
            /* increment Destuffing bytes by 1 byte */ 			
            KissFrame->Destuffing_bytes+=1; 
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////