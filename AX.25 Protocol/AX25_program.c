/************************************************/
/* Author      : Ahmed Fathy Abd El-Qadir       */
/* Last Update : 20-2-2021                      */
/* Version     : V 1.0                          */
/************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "BIT_MATH.h"
#include "STD_TYPES.h"
#include "AX25_interface.h"
#include "AX25_private.h"


AX25_Frame_t *AX25_frame_tPtrCreatSFrame(char*DestAddress, u8 DestSSID, char*SrcAddress,u8 SrcSSID,u8 CMDRES,u8 PollFinal,enum SFrameControlFieldType ControllFieldType,u8* Info,u32 InfoLength)
{
	// DestAddress[6]    : send value of A1 : A6 octets
	// DestSSID          : send 0000
	// SrcAddress[6]     : send value of A8 : A13 octets
	// SrcSSID           : send 0000
	// CMDRES		     : send AX25_COMMAND_FRAME(0)or AX25_RESPONSE_FRAME(1)
	// PollFinal		 : send 1 or 0
	// ControllFieldType : send RR or RNR or REJ or SREJ
	// Info				 : data N*byte
	// InfoLength		 : no of data bytes

	/* Allocate Memory for S Frame*/
	AX25_Frame_t* SFrame = (AX25_Frame_t*)malloc(sizeof(AX25_Frame_t));

	/* Insert Destenation , Destination SSID , Source & Sourse SSID in the frame*/
	memcpy(SFrame->FrameAddress.Destination,DestAddress, sizeof(u8)*6);
	SFrame->FrameAddress.Dest_SSID = DestSSID;
	memcpy(SFrame->FrameAddress.Source,SrcAddress,sizeof(u8)*6);
	SFrame->FrameAddress.Src_SSID = SrcSSID;

	//...   SFrame->FrameAddress.RepeaterUsed = AX25_NO_REPEATER;
	//...   SFrame->FrameAddress.NOfRepeaters = 0;

	SFrame->CmdRes = CMDRES;
	SFrame->FrameType = AX25_S_FRAME;
	SFrame->ControlFieldType = ControllFieldType;
	SFrame->PollFinal = PollFinal;
 	SFrame->Info = Info;
	SFrame->InfoLength = InfoLength;
	/* Call Construct Frame Function*/
	AX25_voidConstructFrame(SFrame);

	return SFrame;
}


AX25_Frame_t* AX25_frame_tPtrCreatUFrame(char*DestAddress,u8 DestSSID,char*SrcAddress,u8 SrcSSID,u8 CMDRES,u8 PollFinal,enum UFrameControlFieldType ControllFieldType,u8 *Info,u32 InfoLength)
{
	// DestAddress[6]    : send value of A1 : A6 octets
	// DestSSID          : send 0000
	// SrcAddress[6]     : send value of A8 : A13 octets
	// SrcSSID           : send 0000
	// CMDRES		     : send AX25_COMMAND_FRAME(0)or AX25_RESPONSE_FRAME(1)
	// PollFinal		 : send 1 or 0
	// ControllFieldType : send SABME or SABM or DISC or DM or UA or FRMR or UI or XID or TEST
	// Info				 : data N*byte
	// InfoLength		 : no of data bytes

	/* Allocate Memory for S Frame*/
	AX25_Frame_t* UFrame = (AX25_Frame_t*)malloc(sizeof(AX25_Frame_t));

    /* Insert Destenation , DestinationSSID , Source & SourseSSID in the frame*/
	memcpy(UFrame->FrameAddress.Destination,DestAddress,sizeof(u8)*6);
	UFrame->FrameAddress.Dest_SSID = DestSSID;
	memcpy(UFrame->FrameAddress.Source,SrcAddress,sizeof(u8)*6);
	UFrame->FrameAddress.Src_SSID = SrcSSID;

	//...	UFrame->FrameAddress.RepeaterUsed = AX25_NO_REPEATER;
	//...	UFrame->FrameAddress.NOfRepeaters = 0;

	UFrame->CmdRes = CMDRES;
	UFrame->FrameType = AX25_U_FRAME;
	UFrame->ControlFieldType = ControllFieldType;
	UFrame->PollFinal = PollFinal;
	if(ControllFieldType != UI){
        UFrame->Info = Info;
	}
	else {
        UFrame->Info = (u8*)malloc(sizeof(u8)*InfoLength);
		memcpy(UFrame->Info,Info,InfoLength);
	}
	UFrame->InfoLength = InfoLength;
    //...  UFrame->NR = 255;
    //...  UFrame->NS = 255;

	/* Call Construct Frame Function*/
	AX25_voidConstructFrame(UFrame);


	return UFrame;
}


AX25_Frame_t *AX25_frame_tPtrCreatIFrame(char*DestAddress,u8 DestSSID,char*SrcAddress,u8 SrcSSID,u8 CMDRES,u8 PollFinal,enum Protocol_Identifier_PID PID,u8 *Info,u32 InfoLength)
{
	// DestAddress[6]    : send value of A1 : A6 octets
	// DestSSID          : send 0000
	// SrcAddress[6]     : send value of A8 : A13 octets
	// SrcSSID           : send 0000
	// CMDRES		     : send AX25_COMMAND_FRAME(0)or AX25_RESPONSE_FRAME(1)
	// PollFinal		 : send 1 or 0
	// PID               : send any of the enum Protocol_Identifier_PID
	// Info				 : data N*byte
	// InfoLength		 : no of data bytes


	/* Allocate Memory for S Frame*/
	AX25_Frame_t* IFrame = (AX25_Frame_t*)malloc(sizeof(AX25_Frame_t));

    /* Insert Destenation , DestinationSSID , Source & SourseSSID in the frame*/
	memcpy(IFrame->FrameAddress.Destination,DestAddress,sizeof(u8)*6);
	IFrame->FrameAddress.Dest_SSID = DestSSID;
	memcpy(IFrame->FrameAddress.Source,SrcAddress,sizeof(u8)*6);
	IFrame->FrameAddress.Src_SSID = SrcSSID;

	//...  IFrame->FrameAddress.RepeaterUsed = AX25_NO_REPEATER;
	//...  IFrame->FrameAddress.NOfRepeaters = 0;

	IFrame->CmdRes = CMDRES;
	IFrame->FrameType = AX25_I_FRAME;
	IFrame->PIDField = PID;
	IFrame->PollFinal = PollFinal;
	IFrame->Info = (u8*)malloc(sizeof(u8)*InfoLength);
	memcpy(IFrame->Info,Info,InfoLength);
	IFrame->InfoLength = InfoLength;

	/* Call Construct Frame Function*/
	AX25_voidConstructFrame(IFrame);

	return IFrame;
}


/*A function to construct frame*/
void AX25_voidConstructFrame (AX25_Frame_t* Frame)  //*** address input
{
	/*To point at the current byte number in the frame*/
    u8 AddressLength= 14;
    //... if (Frame->FrameAddress.RepeaterUsed == AX25_REPEATER)
    //...	AddressLength += 7*Frame->FrameAddress.NOfRepeaters;

    /*Allocate memory to the address field*/
    u8 *ConstructedAddress = (u8*)malloc(sizeof(u8)*AddressLength);

	u8 PID_Len=0;
    /* Check If frame type is I-frame*/
    if (Frame->FrameType == AX25_I_FRAME)
	{
        PID_Len = 1;       // for the PID field (1 byte)
        Frame->NR = NR;
        Frame->NS = NS;
    }

    if (Frame->FrameType == AX25_S_FRAME)
    {
       Frame->NR = NR;
    }

	u8 Control_Len=0;
    /*If the Control Modulo is 16 bits*/
    if (AX25_MODULO_TYPE == AX25_Modulo_8)
        Control_Len = 1;       //  set Control_Len to 1 for the Control field (1 byte)
	else if (AX25_MODULO_TYPE == AX25_Modulo_128)
        Control_Len = 2;       //  set Control_Len to 2 for the Control field (2 byte)

	 /*
	     to get the frame length
	     Start Flag   +          Address           +  Control   +      PID    +   Info   +    FCS     +    End Flag
	      1 Byte          14 Byte + 7*No.Repeater     1/2 Byte      1 Byte(I)      Op       2 Byte         1 Byte
	 */
    Frame->FrameLength = AddressLength + Control_Len + PID_Len + Frame->InfoLength + 4 ;

    /*Allocate memory to the frame*/
    Frame->PtrtoFrame = (u8*)malloc(sizeof(u8) * Frame->FrameLength);

    /*Check Command/Respons */
	if(Frame->CmdRes == AX25_COMMAND_FRAME)
	{
		Frame->FrameAddress.Dest_CmdRes = 1;
		Frame->FrameAddress.Src_CmdRes = 0;
	}
	else if(Frame->CmdRes == AX25_RESPONSE_FRAME)
	{
		Frame->FrameAddress.Dest_CmdRes = 0;
		Frame->FrameAddress.Src_CmdRes = 1;
	}


    /*Call function to Construct the Address Field*/
    AX25_voidConstructAddress (&(Frame->FrameAddress) , ConstructedAddress);
    u8 Index = 1;
    /*Copy the constructed address field to the frame and Skip the start flag*/
    memcpy(Frame->PtrtoFrame+Index , ConstructedAddress, AddressLength);
    /*free the Allocated memory for the address field*/
    free(ConstructedAddress);
    Index += AddressLength;      //Index = 15

    /*Call function to  Construct the Control Field*/
    Frame->ControlField = AX25_u16ConstructControlField(Frame->FrameType, Frame->ControlFieldType ,Frame->PollFinal);
    /*Add the Control Field to the frame*/
    if(AX25_MODULO_TYPE == AX25_Modulo_8)
	{                                                   //  0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19
		Frame->PtrtoFrame[Index] = Frame->ControlField; //                                     ^
	    Index += 1;		//Index = 16					//                                   index
	}
	else if(AX25_MODULO_TYPE == AX25_Modulo_128)
    {
        Frame->PtrtoFrame[Index] = Frame->ControlField;
	    Index += 2;

    }


    /* Add the PID field if the frame is I frame*/
    if(Frame->FrameType == AX25_I_FRAME)
	{
        Frame->PtrtoFrame[Index]= Frame->PIDField;
	    Index += 1;	   //Index = 17
	}

    /*Copy the Info to the frame*/
    memcpy(Frame->PtrtoFrame+Index , Frame->Info , sizeof(u8)*Frame->InfoLength);
    Index +=Frame->InfoLength;


    /*Generate the CRC starting from the first byte in the Address field to the last byte of the Info field*/
	//...                                                         u16 CRC = CRC16_u16CCITTFalse(Frame->PtrtoFrame, Frame->FrameLength-2);
	/*Add in the CRC code to the frame in FCS Field*/
	//...                                                                                                 Frame->PtrtoFrame[Index] = CRC;
	Index += 2;


	/*Add the Start and End Flags to the frame*/
	Frame->PtrtoFrame[0] = AX25_FLAG;
	Frame->PtrtoFrame[Frame->FrameLength-1] = AX25_FLAG;
}


/*Constructs Address field for a specific frame type*/
void AX25_voidConstructAddress (Address_Field_t* Address , u8* ConstructedAddress)
{
    /*Copy the first 6th octets in both destination and source */
	memcpy (ConstructedAddress , Address->Destination, sizeof(u8)*6);  // copy A1 A2 A3 A4 A5 A6
    memcpy (ConstructedAddress+7 , Address->Source , sizeof(u8)*6);    // copy A8 A9 A10 A11 A12 A13

   // if (Address->RepeaterUsed == AX25_NO_REPEATER)
   // {
		/*
		//-- Set the Destination Octets to its value
		ConstructedAddress[0] = A1;    // N
		ConstructedAddress[1] = A2;    // J
		ConstructedAddress[2] = A3;    // 7
		ConstructedAddress[3] = A4;    // P
		ConstructedAddress[4] = A5;    // Space
		ConstructedAddress[5] = A6;    // Space
		ConstructedAddress[6] = A7;    // SSID  >> CRRSSID0 >> 11100000 (C=1 , RR=11 , SSID:0000 & 0-bit =0)

		//-- Set the Source Octets to its value
		ConstructedAddress[7] = A8;    // N
		ConstructedAddress[8] = A9;    // 7
		ConstructedAddress[9] = A10;   // L
		ConstructedAddress[10] = A11;  // E
		ConstructedAddress[11] = A12;  // M
		ConstructedAddress[12] = A13;  // Space
		ConstructedAddress[13] = A14;  // SSID  >> CRRSSID1 >> 01100001 (C=0 , RR=11 , SSID:0000 & 0-bit =1)
		*/

		//-- put the Command/Response bit, Reserved bits and the SSID for the destination and source addresses
        ConstructedAddress[6] = (Address->Dest_CmdRes<<7) | (R_Bits<<5) | (Address->Dest_SSID<<1) ;
        ConstructedAddress[13] = (Address->Src_CmdRes<<7) | (R_Bits<<5) | (Address->Src_SSID<<1);

		CLR_BIT(ConstructedAddress[6], 0);
		//-- Set the 0-bit in the last Octet in Source address To indicate that this is the last octet
		SET_BIT(ConstructedAddress[13],0);
  // }

  /* else if (Address->RepeaterUsed == AX25_REPEATER)
     {
		//--if A repeater is used, Copy its address to its place in the constructed address array
		u8 i;
    	for(i=1;i <= Address->NOfRepeaters;i++)
    	{
		//-- Copy the seven repeater bytes ([0]:[6]) to the first 6th octets in repeater
    	memcpy(ConstructedAddress+i*14 , Address->Repeater[i-1] ,6);
		//-- Copy the 7th octets in repeater
		ConstructedAddress[13 + 7*i] = (R_Bits<<5) | (Address->Rep_SSID[i-1]<<1);
    	}

		//-- Set the 0-bit in the last Octet
    	SET_BIT(ConstructedAddress[13+7*(i-1)],0);
    }*/

}


/*Constructs control field for a specific frame type*/
u16 AX25_u16ConstructControlField(u8 Copy_u8FrameType, u8 Copy_u8ControlFieldType, u8 Copy_u8PollFinal)
{
	// Copy_u8FrameType         :  AX25_S_FRAME or AX25_U_FRAME or AX25_I_FRAME
	// Copy_u8ControlFieldType  :  RR or RNR or REJ or SREJ  --> S_FRAME
    //                          :  SABME or SABM or DISC or DM or UA or FRMR or UI or XID or TEST --> U_FRAME
	// Copy_u8PollFinal         : 1 or 0
	// set NR & NS value
	switch(Copy_u8FrameType)
	{
		case AX25_S_FRAME:
		{
			/*
			    S Frame in Case of Modulo 8 (8-Bits)
				7   6   5     4    3   2   1   0
				~~ N(R)~~    P/F   S   S   0   1             ==>  S  S  Bits  to select   RR, RNR, REJ & SREJ  (Control Field Types)
			*/
			if (AX25_MODULO_TYPE == AX25_Modulo_8)
			{
				u8 Local_u8ControlField = 0;
				Local_u8ControlField = (NR<<5) | (Copy_u8PollFinal<<4) | (Copy_u8ControlFieldType<<2) | 1;
				return Local_u8ControlField;
			}
			/*
			    S Frame in Case of Modulo 128 (16-Bits)
				15   14   13   12   10   9      8    7  6  5  4  3  2  1  0
				~~~~~~~     N(R)    ~~~~~~     P/F   0  0  0  0  S  S  0  1    ==> S  S  Bits  to select   RR, RNR, REJ & SREJ
			*/
			else if (AX25_MODULO_TYPE == AX25_Modulo_128)
			{
				u16 Local_u16ControlField = 0;
				Local_u16ControlField = (NR<<9) | (Copy_u8PollFinal<<8) | (Copy_u8ControlFieldType<<2) | 1;
				return Local_u16ControlField;
			}
			else
			{
				return 0;
			}

		}
		case AX25_U_FRAME:
		{
			/*
			    U Frame in Case of Modulo 8 (8-Bits)
				7   6   5     4    3   2   1   0
				M   M   M    P/F   M   M   1   1      ==>  M M M M M  Bits  to select  SABM,SABME,DISC,DM,UA,FRMR,UI,XID & TEST
			*/
			if (AX25_MODULO_TYPE == AX25_Modulo_8)
			{
                u8 Local_u8ControlField = 0;
                Local_u8ControlField =  Copy_u8ControlFieldType | (Copy_u8PollFinal<<4);
                return Local_u8ControlField;
			}
			else
			{
				return 0;
			}
		}
		case AX25_I_FRAME:
		{
			/*
			    I Frame in Case of Modulo 8 (8-Bits)
				7   6   5       4      3   2   1      0
				~~ N(R)~~      P/F     ~~ N(S)~~      0
			*/

			if (AX25_MODULO_TYPE == AX25_Modulo_8)
			{
				u8 Local_u8ControlField = 0;
				Local_u8ControlField = (NR <<5) | (Copy_u8PollFinal<<4) | (NS <<1) | 0;
				return Local_u8ControlField;
			}

			/*
			    I Frame in Case of Modulo 128 (16-Bits)
				15   14   13   12   10   9           8         7  6  5  4   3  2   1      0
				~~~~~~~     N(R)    ~~~~~~          P/F        ~~~~~~   N(S)  ~~~~~~      0
			*/
			else if (AX25_MODULO_TYPE == AX25_Modulo_128)
			{
				u16 Local_u16ControlField = 0;
				Local_u16ControlField = (NR << 9)  | (Copy_u8PollFinal<<8)  |  (NS <<1) | 0;
				return Local_u16ControlField;
			}
			else
			{
				return 0;
			}
		}
	    default:
	    {
			return 0;
	    }
	}
}


/*A function to do the BIT stuffing upon Reception of a frame*/
void AX25_voidStuff(AX25_Frame_t* Frame)
{
    Frame->StuffBits=0;
    /*Bit Counter*/
	u32 Local_u32BitCounter = 0;

	u8 OnesCounter = 0;

	while(Local_u32BitCounter < (Frame->FrameLength)*8 - 1)
	{
		if(GET_BIT((Frame->PtrtoFrame[Local_u32BitCounter/8]),(7-Local_u32BitCounter%8)) == 1)
			OnesCounter++;
        else
            OnesCounter = 0;

		if (OnesCounter == 5)
		{
            /*if 5 consicutive ones are detected, insert a zero after them*/
			AX25_voidInsertZero(Frame, Local_u32BitCounter+1);
			/*Reset OnesCounter*/
			OnesCounter = 0;
		}

		Local_u32BitCounter++;
	}
}

/*A function that inserts a 0 at the position Offset Bits in a frame created By malloc*/
void AX25_voidInsertZero(AX25_Frame_t* Frame, u32 Offset)
{
	/*Check if StuffBits has a number divisible by 8*/
	if(Frame->StuffBits%8 == 0)  // 0 or 8 or 16 or ....
	{
		/*reallocate by one byte, increment the length and the StuffBits*/
		Frame -> PtrtoFrame =(u8*) realloc(Frame -> PtrtoFrame, (Frame -> FrameLength+1)*sizeof(u8));
		Frame -> PtrtoFrame[Frame->FrameLength] = 0;  // Put 0 in last byte in the frame
		Frame -> FrameLength++;
	}

	/*Shift all Bits to the right by one and insert a zero at the offset location*/
	AX25_voidShiftArray(AX25_SHIFTRIGHT, Frame, Offset);

	/*Insert Zero at the Offset*/
	CLR_BIT((Frame->PtrtoFrame[Offset/8]),(7-(Offset%8)));

	(Frame->StuffBits)++;
}


void AX25_voidShiftArray(u8 Copy_u8Direction, AX25_Frame_t *Frame, u32 Copy_u32Offset)
{
	if (Copy_u8Direction == AX25_SHIFTRIGHT)
	{
		u16 Local_u16Counter = 0;

		u8 i;
		/*Starting from the last byte, shift the current byte right by 1, then copy the 0's bit
		from the previous byte to the 7's bit in the current byte, do this for all bytes with
		index less than the index of the offset*/
		for (Local_u16Counter=Frame->FrameLength-1; (Local_u16Counter)*8 > Copy_u32Offset; Local_u16Counter--)
		{
			Frame->PtrtoFrame[Local_u16Counter] >>= 1;
			Frame->PtrtoFrame[Local_u16Counter] |= (GET_BIT((Frame->PtrtoFrame[Local_u16Counter-1]), 0) << 7);
		}
		/*starting from the last bit of the offset bit, copy the second to last bit to the last bit,
		do this for all bits finishing with the offset bit*/
		for (i = 7; i > Copy_u32Offset%8 ;i--)
		{
			CLR_BIT((Frame->PtrtoFrame[Local_u16Counter]),(7-i));
			Frame->PtrtoFrame[Local_u16Counter] |= (GET_BIT((Frame->PtrtoFrame[Local_u16Counter]), (8-i)) << (7-i)) ;
		}

	}
	else if (Copy_u8Direction == AX25_SHIFTLEFT)
	{
		u16 Local_u16Counter = 0;

		u8 i;

		/*Starting with the offset bit, copy the next bet to the current bit, do this for all
        bits greater than the offset bit*/
		for (i= Copy_u32Offset%8; i < 7 ;i++)
		{
		    CLR_BIT((Frame->PtrtoFrame[Copy_u32Offset/8]),(7-i%8));
			Frame->PtrtoFrame[Copy_u32Offset/8] |= GET_BIT((Frame->PtrtoFrame[Copy_u32Offset/8]),(6-i)) << (7-i);

		}
		/*Stasrting from the offset byte, copy the 7's bit of the next byte to the 0's bit
		of the current byte and shift the next byte left by 1*/
		for (Local_u16Counter=Copy_u32Offset/8; (Local_u16Counter) < Frame->FrameLength-1; Local_u16Counter++)
		{
			CLR_BIT(Frame->PtrtoFrame[Local_u16Counter],0);
			Frame->PtrtoFrame[Local_u16Counter] |= GET_BIT(Frame->PtrtoFrame[Local_u16Counter+1],7);
			Frame->PtrtoFrame[Local_u16Counter+1] = (Frame->PtrtoFrame[Local_u16Counter+1]<<1) & 0xFF;
		}
	}
}



u16 CRC16_u16CCITTFalse(u8* Copy_u8PtrData, u32 Copy_u32DataLength)
{

	if(Copy_u32DataLength <= 0)
		return 0xFFFF;

	u8 B1 = 0, B0 = 0;
	u32 Bindex;

	#define CRC_POLY		0x1021

	/*16 bit CRC register with 0xFFFF initial value*/
	u16 CRC_REG = 0xFFFF;

	/*Check the first two Bytes if they exist*/
	if(Copy_u32DataLength >= 1)
		B1 = Copy_u8PtrData[0];
	if(Copy_u32DataLength >= 2)
		B0 = Copy_u8PtrData[1];

	/*Get The Actual Initial Value*/
	CRC_REG ^= ((B1<<8)|B0);

	/*looping on the data bytes starting from the third element to the last element
	 * of the data + 2 additional bytes all zeros									*/
	for(Bindex=2;Bindex<Copy_u32DataLength+2;Bindex++)
	{
		/*Get the current byte of the data, if the data is finished, add two bytes of 0 */
		u8 CurrentByte = Bindex >= Copy_u32DataLength?	0:Copy_u8PtrData[Bindex];
		u8 i;

		/*Looping on the bits of the current byte*/
		for(i=0;i<8;i++)
		{
			/*Check if the MSB of the CRC register is set*/
			if(GET_BIT(CRC_REG,15))
			{
				/*Shift in the next bit of data starting from the MSB to LSB
				 * and XOR the CRC register with the polynomial*/
				CRC_REG <<=1;
				CRC_REG |= GET_BIT(CurrentByte,(7-i));
				CRC_REG ^= CRC_POLY;
			}
			else
			{
				/*if Not, shift in the next bit of data starting from MSB to LSB*/
				CRC_REG <<= 1;
				CRC_REG |= GET_BIT(CurrentByte,(7-i));
			}

		}
	}

	/*After finishing the loop, return the CRC bytes*/
	return CRC_REG;
}


u8 AX25_u8DeConstructFrame (frame_t* Frame)
{

  u8 *TempArr = (u8*)malloc(sizeof(u8)*(Frame->FrameLength-2));
  memcpy(TempArr,Frame->PtrtoFrame+1,Frame->FrameLength-2);
  memcpy(Frame->PtrtoFrame,TempArr,Frame->FrameLength-2);
  free(TempArr);
  Frame->PtrtoFrame = (u8*)realloc(Frame->PtrtoFrame,Frame->FrameLength-2);
  Frame->FrameLength -= 2;


    /*Apply Bit DeStuffing on the Frame*/
    AX25_voidDestuff(Frame);

    /*Generate the CRC on the frame without Start and End Flags*/
    u16 NewCRC = CRC16_u16CCITTFalse(Frame->PtrtoFrame, Frame->FrameLength);

    /*Check if CRC is equal zero or no */
    /*if equal zero that mean correct Frame*/
    /*if not that mean Wrong Frame*/
    if(NewCRC == 0x00)
    {
        /*Correct Frame*/
        /*This variable to count from the frame beginning*/
        u32 FromStartFrame=0;
        /*Copy Destination to frame address destination*/
        memcpy(Frame->FrameAddress.Destination , Frame->PtrtoFrame+FromStartFrame , 6);
        Frame->FrameAddress.Destination[6] = 0;
        FromStartFrame+=6;
        /*Get Destination SSID value*/
        Frame->FrameAddress.DestinationSSID = ((Frame->PtrtoFrame[FromStartFrame]>>1) &0xF);
        /*Get Destination Command and Response bit*/
        Frame->FrameAddress.DestCommandResponse = GET_BIT(Frame->PtrtoFrame[FromStartFrame++],7);
        /*Copy Source to frame address source*/
        memcpy(Frame->FrameAddress.Source , Frame->PtrtoFrame+FromStartFrame ,6);
        Frame->FrameAddress.Source[6] = 0;
        FromStartFrame+=6;
        /*Get Source SSID value*/
        Frame->FrameAddress.SourceSSID = ((Frame->PtrtoFrame[FromStartFrame]>>1) &0xF);
        /*Get Source Command and Response bit*/
        Frame->FrameAddress.SrcCommandResponse = GET_BIT(Frame->PtrtoFrame[FromStartFrame],7);
        /*Check if there is Repeater or no ?!*/
        u8 i=0;

        if((Frame->FrameAddress.DestCommandResponse == 0 && Frame->FrameAddress.SrcCommandResponse == 0)
        		||(Frame->FrameAddress.DestCommandResponse == 1 && Frame->FrameAddress.SrcCommandResponse == 0))
        {
        	Frame->CMDRES = AX25_COMMAND_FRAME;
        }
        else if ((Frame->FrameAddress.DestCommandResponse == 1 && Frame->FrameAddress.SrcCommandResponse == 1)
        		||(Frame->FrameAddress.DestCommandResponse == 0 && Frame->FrameAddress.SrcCommandResponse == 1))
        {
        	Frame->CMDRES = AX25_RESPONSE_FRAME;
        }

        if(GET_BIT(Frame->PtrtoFrame[FromStartFrame++],0) ==0)
        {
        	/*if frame has repeater address*/
			Frame->FrameAddress.RepeaterUsed = AX25_REPEATER;
			Frame->FrameAddress.NOfRepeaters = 1;
			while (GET_BIT(Frame->PtrtoFrame[FromStartFrame],0) ==0)
			{
				Frame->FrameAddress.Repeater[i] = (char *)malloc(7);
				/*Copy Repeater to frame address Repeater*/
				memcpy (Frame->FrameAddress.Repeater[i] , Frame->PtrtoFrame+FromStartFrame +7*i , 6);
				FromStartFrame+=6;
				/*Get Repeater SSID value*/
				Frame->FrameAddress.RepeaterSSID[i] = ((Frame->PtrtoFrame[FromStartFrame]>>1) &0xF);
				/*Get Has-Been-Repeated bit*/
				Frame->FrameAddress.HRepeated[i] = GET_BIT(Frame->PtrtoFrame[FromStartFrame++],7);
				i++;
				Frame->FrameAddress.NOfRepeaters++;
			}
		}
        else
        {
            /*if frame does't have repeater address*/
            Frame->FrameAddress.RepeaterUsed = AX25_NO_REPEATER;
        }

        /*Shift left then and with 0x7F for each of Destination , Source and repeater if it found, to back them as original addresses*/
        for(i=0;i<6;i++)
        {
        	Frame->FrameAddress.Destination[i] = (Frame->FrameAddress.Destination[i]>>1)&0x7F;
        	Frame->FrameAddress.Source[i] = (Frame->FrameAddress.Source[i]>>1)&0x7F;
        }
        i=0;

        /*Check if Control Module is 16Bits or 8Bits ?!*/
        /*if 16Bits then copy two bytes*/
        /*if 8Bits then Copy one byte*/
        if (AX25_CONTROL_MODULO_TYPE == AX25_16BITS)
        {
            /*Copy two byte to ControlField*/
            memcpy (&(Frame->ControlField) , Frame->PtrtoFrame+FromStartFrame ,2);
            FromStartFrame+=2;
        }
        else
        {
            /*Copy one byte to ControlField*/
            Frame->ControlField = Frame->PtrtoFrame[FromStartFrame++];
        }
        /*Get the Frame Type*/
        if (GET_BIT(Frame->ControlField,0) == 0)
        {
        	Frame->FrameType = AX25_I_FRAME;
        	Frame->PollFinal = (AX25_CONTROL_MODULO_TYPE == AX25_16BITS)? GET_BIT(Frame->ControlField,8):GET_BIT(Frame->ControlField,4);
            Frame->NR = (AX25_CONTROL_MODULO_TYPE == AX25_16BITS)? (Frame->ControlField>>9)&0x7f:(Frame->ControlField>>5)&7;
            Frame->NS = (AX25_CONTROL_MODULO_TYPE == AX25_16BITS)? (Frame->ControlField>>1)&0x7f:(Frame->ControlField>>1)&7;

        }
        else
        {
        	if(GET_BIT(Frame->ControlField,1) == 0)
        	{
        		Frame->FrameType = AX25_S_FRAME;
        		Frame->ControlFieldType = (Frame->ControlField>>2) &3;
        		Frame->PollFinal = (AX25_CONTROL_MODULO_TYPE == AX25_16BITS)? GET_BIT(Frame->ControlField,8):GET_BIT(Frame->ControlField,4);
        		Frame->NR = (AX25_CONTROL_MODULO_TYPE == AX25_16BITS)? (Frame->ControlField>>9)&0x7f:(Frame->ControlField>>5)&7;
        		Frame->NS = 255;
        	}
        	else
        	{
        		Frame->FrameType = AX25_U_FRAME;
        		Frame->ControlFieldType = (Frame->ControlField);
        		CLR_BIT(Frame->ControlFieldType,4);
        		Frame->PollFinal = GET_BIT(Frame->ControlField,4);
        		Frame->NR = 255;
        		Frame->NS = 255;
        	}
        }
        /*Check if Frame Type is I_Frame or no ?!*/
        /*if yes, That meaning there is PID in the frame*/
        /*if no, That meaning there is no PID in the frame*/
        if (Frame->FrameType == AX25_I_FRAME)
        {
            /*Copy PID to frame PID*/
            Frame->PID = Frame->PtrtoFrame[FromStartFrame++];
        }
        /*Calculate Info length*/
        Frame->InfoLength = Frame->FrameLength-2 - FromStartFrame;
        /*check if Info length equal 1 or bigger ?! */
        /*if yes, copy Info to frame Info*/
        /*if no, so Info length equal zero that meaning there is no Info in the frame */
        if(Frame->InfoLength>=1)
        {
	  Frame->Info = Frame->PtrtoFrame+FromStartFrame;

        else
        {
        	Frame->Info = 0;
        	Frame->InfoLength = 0;
        }
        return AX25_FRAME_CORRECT;
    }
    else
    {
        /*Wrong Frame*/
        return AX25_WRONG_CRC;
    }
}


#endif



void AX25_voidDestuff(frame_t* Frame)
{
    /*Counts the bits of the frame*/
	u32 Local_u32BitCounter = 0;

    /*Counts consecutive ones in a frame*/
	u8 OnesCounter = 0;
    Frame->DeStuffBits = 0;
	while(Local_u32BitCounter < (Frame -> FrameLength) *8-1)
	{
		if(GET_BIT((Frame->PtrtoFrame[Local_u32BitCounter/8]),(7-Local_u32BitCounter%8)) == 1)
			OnesCounter++;
        else
            OnesCounter = 0;

		if (OnesCounter == 5)
		{
			/*Shift all Bits to the left by one*/
            AX25_voidShiftArray(AX25_SHIFTLEFT, Frame, Local_u32BitCounter+1);

            /*Increase DeStuffBits indicating that a destuffing operation has done*/
            Frame -> DeStuffBits++;

            /*reset OnesCounter*/
			OnesCounter = 0;
		}

		Local_u32BitCounter++;
	}
    if (Frame->DeStuffBits == 0)
    {
        return;
    }
    else if(Frame->DeStuffBits %8 != 0 )
    {
        Frame->PtrtoFrame = realloc(Frame->PtrtoFrame,Frame->FrameLength - (Frame->DeStuffBits/8 +1));
        Frame->FrameLength -= Frame->DeStuffBits/8 +1;
    }
    else if (Frame->DeStuffBits%8 == 0 )
    {
        Frame->PtrtoFrame = realloc(Frame->PtrtoFrame,Frame->FrameLength - (Frame->DeStuffBits/8));
        Frame->FrameLength -= Frame->DeStuffBits/8;
    }
}


