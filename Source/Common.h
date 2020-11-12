

#ifndef _COMMON_H_
#define _COMMON_H_


//Data Type const
#ifdef	OS_GLOBALS
#define	OS_EXT  extern
#else
#define	OS_EXT
#endif

//********************************************************************************
typedef bit										BOOLEAN, BIT;	
typedef unsigned char					INT8U, u8, U8, uint8, uint8_t, UINT8, BYTE;  				/* Unsigned  8 bit quantity        */
typedef signed   char					INT8S, s8, S8, int8, INT8;  												/* Signed    8 bit quantity        */
typedef unsigned int	  			INT16U, u16, U16, uint16, uint16_t, UINT16, WORD; 	/* Unsigned 16 bit quantity        */
typedef signed   int	  			INT16S, s16, S16, int16, INT16; 										/* Signed   16 bit quantity        */
typedef unsigned long					INT32U, u32, U32, uint32, uint32_t, UINT32, DWORD; 	/* Unsigned 32 bit quantity        */
typedef signed   long					INT32S, s32, S32, int32, INT32; 										/* Signed   32 bit quantity        */
typedef float									FP32;   																						/* Single precision floating point */
//********************************************************************************

#define FOSC_160000

#define	CID_READ					0x0B
#define	DID_READ					0x0C

#define	ERASE_APROM				0x22
#define	READ_APROM				0x00
#define	PROGRAM_APROM			0x21
#define	ERASE_LDROM				
#define	READ_LDROM				
#define	PROGRAM_LDROM			
#define	READ_CFG					0xC0
#define	PROGRAM_CFG				0xE1
#define	READ_UID					0x04
//********************************************************************************

//Key
enum {
		_NO_KEY_,
		_KEY_1_,
		_KEY_2_,
		_MAX_KEY_,
};

#if 0

//P30
#define _ALL_KEY_MASK_1_  0x01
#define KEY_PORT_1				P3
//P17
#define _ALL_KEY_MASK_2_  0x80
#define KEY_PORT_2				P1
//ALL key
#define _ALL_KEY_MASK_		0x81

#else

//P13
#define _ALL_KEY_MASK_1_  0x08
#define KEY_PORT_1				P1
//P14
#define _ALL_KEY_MASK_2_  0x10
#define KEY_PORT_2				P1
//ALL key
#define _ALL_KEY_MASK_		0x18

#endif

#define _HOLD_TIMER_KEY_	4
OS_EXT INT8U		HoldKeyCtr;

OS_EXT BOOLEAN	F_Key;
OS_EXT BOOLEAN	F_PushKey;
OS_EXT BOOLEAN	F_NewKey;
OS_EXT BOOLEAN	F_HoldKey;
OS_EXT BOOLEAN	F_TwoKey;
OS_EXT INT8U		Key;
OS_EXT INT8U		OldKey;

#define RST_98DX3136				P05
enum {
		_MODE_0_,
		_MODE_1_,
		_MODE_2_,
		_MODE_3_,
		_MODE_MAX_,
};
OS_EXT INT8U		ModeCtr;
OS_EXT INT8U		ModeCtr1;
OS_EXT INT8U		ModeCtr2;
OS_EXT INT8U		WriteFlag;

enum {
		_LED_ON_,
		_LED_OFF_,
};
#define LED1							P03
#define LED2							P04

OS_EXT BOOLEAN 	F_Timer;

//TEST
OS_EXT u8 test[20];
OS_EXT u8 Ctr;
//TEST

//********************************************************************************
void  InitialUART0_Timer1(UINT32 u32Baudrate); //T1M = 1, SMOD = 1
void  InitialUART0_Timer3(UINT32 u32Baudrate); //Timer3 as Baudrate, SMOD=1, Prescale=0
void  InitialUART1_Timer3(UINT32 u32Baudrate);
void  Send_Data_To_UART0(UINT8 c);
UINT8 Receive_Data_From_UART0(void);
void  Send_Data_To_UART1(UINT8 c);
UINT8 Receive_Data_From_UART1(void);
void  InitialUART1(UINT32 u32Baudrate);

extern bit BIT_TMP;


//-------------------------------------------------------------------------------
void ScanKey(void);
void PushKeyFunc(void);
void ReleKeyFunc(void);
BOOLEAN HoldKeyCom(void);
BOOLEAN SettingCom(void);

#endif


