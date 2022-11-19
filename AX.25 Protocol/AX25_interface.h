/************************************************/
/* Author      : Ahmed Fathy Abd El-Qadir       */
/* Last Update : 20-2-2021                      */
/* Version     : V 1.0                          */
/************************************************/
#include "STD_TYPES.h"

#ifndef KISS_INTERFACE_H
#define KISS_INTERFACE_H

/************************************************** Macros ***********************************************/
#define AX25_S_FRAME										0
#define AX25_U_FRAME										1
#define AX25_I_FRAME										2

#define AX25_Modulo_8										0
#define AX25_Modulo_128										1

#define AX25_FLAG       									0x7E  // 0111 1110

#define AX25_COMMAND_FRAME									0
#define AX25_RESPONSE_FRAME									1

#define AX25_XID_FI							                0x82
#define AX25_XID_GI	 					                    0x80

///////////////// For Address Field //////////////////////////////
#define AX25_REPEATER       1
#define AX25_NO_REPEATER    0

///////////////// Classes of Procedures //////////////////////////
#define AX25_XID_CLASSES_OF_PROCEDURES              		2

#define AX25_XID_TRANSMISSION_MODE							1	// The default is half duplex.(Choose 1 or 2 )
/*Transmission mode options*/
#define AX25_XID_HALFDUPLEX									1
#define AX25_XID_FULLDUPLEX									2


///////////////// HDLC Optional Functions ////////////////////////
#define AX25_XID_HDLC_OPTIONAL_FUNCTIONS					3

#define AX25_XID_HDLC_Reject_Mode							3      // The default is selective reject-reject. (Choose 1, 2 or 3 )
#define AX25_XID_HDLC_Modulo  							    1      // The default is modulo 8. (Choose 1 or 2 )

/*Reject Type Options*/
#define AX25_XID_IMPLICIT_REJECT							1
#define AX25_XID_SELECTIVE_REJECT							2
#define AX25_XID_SELECTIVE_REJECT_REJECT					3
/*XID Modulo Options*/
#define AX25_XID_MODULO_8    								1
#define AX25_XID_MODULO_128									2

#define AX25_XID_TRANSMIT				  					0
#define AX25_XID_RECEIVE				 					1


///////////////////// I Field Length Tx //////////////////////////
#define AX25_XID_I_TX_LENGTH								5
#define AX25_XID_I_TX_DEFAULT								256    // The default is 256 octets (2048 bits)


///////////////////// I Field Length Rx //////////////////////////
#define AX25_XID_I_RX_LENGTH								6
#define AX25_XID_I_RX_DEFAULT								256     // The default is 256 octets (2048 bits)


///////////////////// Wnidow Size Tx /////////////////////////////
#define AX25_XID_TX_WINDOW_SIZE								7
#define AX25_XID_TX_DEFAUL_WINDOW_SIZE_MODULO8				4      // The default is 4 for modulo 8 and 32 for modulo 128.
#define AX25_XID_TX_DEFAUL_WINDOW_SIZE_MODULO128			32


///////////////////// Wnidow Size Rx /////////////////////////////
#define AX25_XID_RX_WINDOW_SIZE								8
#define AX25_XID_RX_DEFAUL_WINDOW_SIZE_MODULO8				4      // The default is 4 for modulo 8 and 32 for modulo 128.
#define AX25_XID_RX_DEFAUL_WINDOW_SIZE_MODULO128			32


///////////////////// Acknowledgement Timer //////////////////////
#define AX25_XID_ACK_TIMER									9
#define AX25_XID_DEFAULT_ACK_TIMER						    3000   // The default is 3000 MSec


///////////////////// Wnidow Size Tx /////////////////////////////
#define AX25_XID_RETRIES								    10
#define AX25_XID_DEFAULT_RETRIES							10      // The default is 10 retries

/*********************************************************************************************************/


typedef struct
{
    u8 Destination[7];   // From A1 to A7 Octets
    u8 Source[7];        // From A8 to A14 Octets
	//u8 *Repeater[7];   // From A15 to A21 Octets
    //u8 NOfRepeaters;
    //u8 RepeaterUsed;
	u8 Dest_SSID;
    u8 Src_SSID;
	//u8 *Rep_SSID;
	u8 Dest_CmdRes;     // C-Bit
    u8 Src_CmdRes;      // C-Bit
}Address_Field_t ;

typedef struct
{
	Address_Field_t FrameAddress;
	u8 FrameType;
	u8 ControlFieldType ;
	u8 PollFinal;
	u16 ControlField;
	u8 CmdRes;
	u8 NR;
	u8 NS;
	u8 PIDField;
	u8 *Info;
	u32 InfoLength;
	u8 *PtrtoFrame;
	u32 FrameLength;
	u32 StuffBits;
	u32 DeStuffBits;
} AX25_Frame_t;


typedef struct
{
	u8 FormatIndicator;   		 // FI
	u8 GroupIdentifier;   		 // GI
	u16 GroupLength;			 // GL
	u16 ClassesOfProcedures;
	u32 HDLCOptionalFunctions;
	u64 MaxTXIFieldLength;
	u64 MaxRXIFieldLength;
	u8 TXWindowsize;
	u8 RXWindowsize;
	u32 ACKTimer;
	u32 RetryCount;
	u8* XIDInfo;
	u32 XIDInfoLength;
	u8 State;
}XID_Info;

enum AddressFieldParameter{
	A1=0x9C,               // N
	A2=0x94,               // J
	A3=0x6E,               // 7
	A4=0xA0,               // P
	A5=0x40,               // SPACE
	A6=0x40,               // SPACE
	A7=0xE0,               // SSID   >> CRRSSID0 >> 11100000 (C=1 , RR=11 , SSID:0000)

	A8=0x9C,               // N
	A9=0x6E,               // 7
	A10=0x98,              // L
	A11=0x8A,              // E
	A12=0x9A,              // M
	A13=0x40,              // SPACE
	A17=0x60,              // SSID   >> CRRSSID0 >> 01100000 (C=1 , RR=11 , SSID:0000)
};

enum SFrameControlFieldType{
	RR=0,                // Receive Ready      S  S  ==>  0  0
	RNR=1,               // Receive Not Ready  S  S  ==>  0  1
	REJ=2,               // Reject             S  S  ==>  1  0
	SREJ=3               // Selective Reject   S  S  ==>  1  1
};


enum UFrameControlFieldType{
	SABME = 0x6F,     // Set Async Balanced mode Extended  => Command   0110 1111
	SABM = 0x2F,      // Set Async Balanced mode           => Command   0010 1111
	DISC = 0x43,      // Disconnect                        => Command   0100 0011
	DM = 0x0F,        // Disconnect Mode                   => Response  0000 1111
	UA = 0x63,        // Unnumbered Acknowledge            => Response  0110 0011
	FRMR = 0x87,      // Frame Reject                      => Response  1000 0111
	UI = 0x03,        // Unnumbered Information            => Frame     0000 0011
	XID= 0xAF,        // Exchange Identification           => Frame     1010 1111
	TEST = 0xE3       // TEST                              => Frame     1110 0011
};

enum Protocol_Identifier_PID {
	ISO_8208_CCITT_X_25_PLP = 0x01 ,
	Compressed_TCP_IP_packet = 0x06 ,
	Uncompressed_TCP_IP_packet = 0x07 ,
	Segmentation_fragment = 0x08 ,
	TEXNET_datagram_protocol = 0xC3 ,
	Link_Quality_Protocol = 0xC4 ,
	Appletalk = 0xCA ,
	Appletalk_ARP = 0xCB ,
	ARPA_Internet_Protocol = 0xCC ,
	ARPA_Address_resolution = 0xCD ,
    FlexNet = 0xCE ,
    Net_ROM = 0xCF ,
    No_layer_3_protocol_implemented = 0xF0 ,
    Escape_character = 0xFF
};

u8 NR;
u8 NS;
u8 VR;
u8 VS;
u8 VA;


/************************************** Functions Prototype ********************************************/

/*A function to construct S frame*/
AX25_Frame_t* AX25_frame_tPtrCreatSFrame(char*DestAddress,u8 DestSSID,char*SrcAddress,u8 SrcSSID,u8 CMDRES,u8 PollFinal,enum SFrameControlFieldType ControllFieldType,u8 *Info,u32 InfoLength);

/*A function to construct U frame*/
AX25_Frame_t* AX25_frame_tPtrCreatUFrame(char*DestAddress,u8 DestSSID,char*SrcAddress,u8 SrcSSID,u8 CMDRES,u8 PollFinal,enum UFrameControlFieldType ControllFieldType,u8 *Info,u32 InfoLength);

/*A function to construct I frame*/
AX25_Frame_t* AX25_frame_tPtrCreatIFrame(char*DestAddress,u8 DestSSID,char*SrcAddress,u8 SrcSSID,u8 CMDRES,u8 PollFinal,enum Protocol_Identifier_PID PID,u8 *Info,u32 InfoLength);

/*A function to construct frame*/
void AX25_voidConstructFrame (AX25_Frame_t* Frame);

/*A function to Construct address field*/
void AX25_voidConstructAddress (Address_Field_t* Address , u8* ConstructedAddress);

/*Constructs control field for a specific frame type*/
u16 AX25_u16ConstructControlField(u8 Copy_u8FrameType, u8 Copy_u8ControlFieldType, u8 Copy_u8PollFinal);

/*A function to do the BIT stuffing before transmitting a frame*/
void AX25_voidStuff( AX25_Frame_t* Frame);

/*A function that inserts a 0 at the position Offset Bits in a frame created By malloc*/
void AX25_voidInsertZero( AX25_Frame_t* Frame, u32 Offset);

void AX25_voidShiftArray(u8 Copy_u8Direction, AX25_Frame_t *Frame, u32 Copy_u32Offset);

/* CRC Function*/
u16 CRC16_u16CCITTFalse(u8* Copy_u8PtrData, u32 Copy_u32DataLength);

/*A Function to DeConstructed*/
u8 AX25_u8DeConstructFrame ( AX25_Frame_t* Frame);

void AX25_voidDestuff( AX25_Frame_t* Frame);

#endif
