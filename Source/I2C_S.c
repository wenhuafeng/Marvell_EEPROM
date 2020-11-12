/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : May/1/2017
//***********************************************************************************************************

//***********************************************************************************************************
//  File Function: N76E003 I2C Slave demo code
//***********************************************************************************************************

#ifndef  OS_MASTER_FILE
#define  OS_GLOBALS
#include "includes.h"
#endif


#if _HARDWARE_I2C_SLAVE_


//***********************************************************************************************************
//  N76E003-series I2C slave mode demo code, the Slave address = 0xA4
//
//   ____________            _____________ 
//  |            |   SDA    |             |
//  |            |<-------->|             |
//  |            |          |             |
//  |N76E003(M)  |          | N76E003(S)  |
//  |(I2C_Master)|          | (I2C_Slave) |
//  |            |   SCL    |             |
//  |            |--------->|             |
//  |____________|          |_____________|
//
//  The protocol of I2C is same the "24LC64"
//***********************************************************************************************************


//#define I2C_CLOCK               13
#define EEPROM_SLA                0xA0//0xA0


UINT8 data_received[34];
UINT8 data_num;
UINT8 data mem;

//========================================================================================================
void I2C_ISR(void) interrupt 6
{
    //TEST
    u8 i = I2STAT;
    test[Ctr++] = i;
    //TEST
  
    switch (i)
    {
        case 0x00:
            STO = 1;
            break;
        
        case 0x60:
            AA = 1;
            break;
        
        case 0x68:
            while(1);
            break;
        
        case 0x80:
            data_received[data_num++] = I2DAT;
            AA = 1;
            break;
        
        case 0x88:
            data_num = 0;
            data_received[data_num] = I2DAT;
            AA = 1;
            break;
        
        case 0xA0:
            data_num = 0x00;
            AA = 1;
            break;
        
        case 0xA8:
            I2DAT = IP1717B10Full[data_num++];//data_received[data_num];
            AA = 1;
            break;
        
        case 0xB8:
            I2DAT = IP1717B10Full[data_num++];//data_received[data_num];
            AA = 1;
            break;
        
        case 0xC0:
            AA = 1;
            break;
        
        case 0xC8:
            AA = 1;
            break;
    }
    
    SI = 0;
    while(STO);
}

//========================================================================================================
void Init_I2C(void)
{
    P13_Quasi_Mode;                         //set SCL (P13) is Quasi mode
    P14_Quasi_Mode;                         //set SDA (P14) is Quasi mode
    
    SDA = 1;                                //set SDA and SCL pins high
    SCL = 1;
    
    set_P1SR_3;                             //set SCL (P13) is  Schmitt triggered input select.
    
    set_EI2C;                               //enable I2C interrupt by setting IE1 bit 0
    set_EA;

    I2ADDR = EEPROM_SLA;                    //define own slave address
    set_I2CEN;                              //enable I2C circuit
    set_AA;
}

//========================================================================================================

#endif

