

#ifndef  OS_MASTER_FILE
#define  OS_GLOBALS
#include "includes.h"
#endif


#if _TPS23861_


//---------------------------------------------------------------------------------------------
void DelayUs(INT8U count)
{
  u16 i = count;
  
  while(i--);
}

void DelayMs(INT8U count)
{
  Timer0_Delay1ms(count);
}

void I2C_Start(void)
{
  SDAOUT();
  DelayUs(5);
  SDIO = 1;
  DelayUs(2);
  SCLK = 1;
  DelayUs(2);
  SDIO = 0;
  DelayUs(2);
  SCLK = 0;
  DelayUs(5);
}

void I2C_Stop(void)
{
  SDAOUT();
  DelayUs(5);
  SDIO = 0;
  SCLK = 1;
  DelayUs(2);
  SDIO = 1;
  DelayUs(5);
  SCLK = 1;
}

void I2C_Senddata(INT8U dat)
{
  INT8U i;

  SDAOUT();
  DelayUs(10);
  for(i = 0; i < 8; i++)
  {
    SCLK = 0;
    SDIO = (BOOLEAN)(dat & 0x80);
    DelayUs(5);
    SCLK = 1;
    DelayUs(2);  
    dat = dat << 1;
  }
  SCLK = 0;
}

INT8U I2C_Receive(INT8U remaining_bytes)
{
  INT8U result = 0;
  INT8U i,j;

  SDAIN();
  DelayUs(10);
  for(i=0; i<8; i++)
  {
    result = result << 1;
    SCLK = 0;
    DelayUs(2);
    SCLK = 1;
    DelayUs(5);
    j = SDA_PORT;
    if ((j & SDA_NUMBER) == SDA_NUMBER) {
      result = result + 1;
    }
  }
  SCLK = 0;

  SDAOUT();
  DelayUs(5);
  if (remaining_bytes == 0) {
    SDIO = 1;
  } else { 
    SDIO = 0;
  }
  DelayUs(2);
  SCLK = 1;
  DelayUs(2);
  SCLK = 0;

  return result;
}

BOOLEAN I2C_Ack(void)
{
  BOOLEAN result = 0;

  SDAIN();
  DelayUs(10);
  SCLK = 1;
  DelayUs(2);
  result = SDIO;
  SCLK = 0;

  return result;
}

u8 Write_I2C(u16 slaveAddress, u8 RegisterAddr, u8 *readBuffer, u8 numBytes)
{
  //u8 i = 0x00;
  
  I2C_Start();
  I2C_Senddata(slaveAddress << 1);
  if (I2C_Ack()) {
      _nop_();
  }
  I2C_Senddata(RegisterAddr);
  if (I2C_Ack()) {
      _nop_();
  }
  while(numBytes--)
  {
    I2C_Senddata(*readBuffer++);
    if (I2C_Ack()) {
        _nop_();
    }
  }
  I2C_Stop();
  
  return (I2C_COMMAND_STARTED);
}

u8 Read_I2C(u16 slaveAddress, u8 RegisterAddr, u8 *readBuffer, u8 numBytes)
{
  //u8 i = 0x00;
  
  I2C_Start();
  I2C_Senddata(slaveAddress << 1);
  if (I2C_Ack()) {
      _nop_();
  }
  I2C_Senddata(RegisterAddr);
  if (I2C_Ack()) {
      _nop_();
  }
  I2C_Start();
  I2C_Senddata((slaveAddress << 1)|Read);
  if (I2C_Ack()) {
      _nop_();
  }
  while(numBytes--)
  {
    *readBuffer++ = I2C_Receive(numBytes);
  }
  I2C_Stop();
  
  return (I2C_COMMAND_STARTED);
}

/**************************************************************************************************************************************************
*                                 I2C Write Routines                                                                                              *
**************************************************************************************************************************************************/
u8 tps_WriteI2CReg (u16 i2cAddress, u8 registerAddress, u8 writeValue)
{
    u8 rtn;
    
    rtn = Write_I2C (i2cAddress, registerAddress, &writeValue, 1);
    
    return (rtn);
}

u8 tps_WriteI2CMultiple (u16 i2cAddress, u8 registerAddress, u8 *writeValues, u8 numWriteBytes)
{
    u8 rtn;
    
    rtn = Write_I2C (i2cAddress, registerAddress, writeValues, numWriteBytes);
    
    return (rtn);
}

/**************************************************************************************************************************************************
*                                 I2C Read Routines                                                                                               *
**************************************************************************************************************************************************/
u8 tps_ReadI2CReg (u16 i2cAddress, u8 registerAddress, u8 *readValue)
{
    u8 rtn;

    rtn = Read_I2C(i2cAddress, registerAddress, readValue, 1);

    return (rtn);
}

u8 tps_ReadI2CMultiple (u16 i2cAddress, u8 registerAddress, u8 *readValue, u8 numReadBytes)
{
    u8 rtn;

    rtn = Read_I2C(i2cAddress, registerAddress, readValue, numReadBytes);

    return (rtn);
}

//---------------------------------------------------------------------------------------------
#define DETAILED_STATUS   1
#define NUM_OF_TPS23861   2

u8 i2cAddList[NUM_OF_TPS23861] = {0x20, 0x28};
TPS238x_On_Off_t autoMode[NUM_OF_TPS23861] = {TPS_OFF, TPS_OFF};
TPS238X_Interrupt_Mask_Register_t intMask;

uint8_t IntFlag = 0;
uint8_t PrintPower = 0;
uint8_t PerformDetection = 1;   // Initially perform a detection

uint8_t intDelayTime = 0;
uint8_t sysPortNum, sysPortNum1, sysPortNum2, sysPortNum3, sysPortNum4;
uint8_t             sysPortNum5, sysPortNum6, sysPortNum7, sysPortNum8;
TPS238x_Ports_t powerEnablePortEvents, powerGoodPortEvents, detectionPortEvents, classificationPortEvents, icutPortEvents, disconnectPortEvents, inrushPortEvents, ilimPortEvents;
TPS238X_Interrupt_Register_t intStatus;
TPS238X_Supply_Event_Register_t supplyEvents;
volatile uint8_t rtn;
TPS238x_Detection_Status_t detectStatus;
TPS238x_Classification_Status_t classStatus;
uint16_t current, voltage;
uint32_t outNum;
uint8_t powerGood;
uint8_t powerEnable;
uint8_t temperature;
uint8_t current_address;
//uint8_t i;
uint8_t failClassOnce[4] = {0, 0, 0, 0};

TPS238x_Ports_t  inactivePorts[NUM_OF_TPS23861];
uint8_t devNum;
uint8_t oldAutoBitSetting;
uint8_t addressChangeNeeded = FALSE;


#ifdef DETAILED_STATUS
TPS238x_Ports_t  powerEnablePorts;
TPS238x_Ports_t  powerGoodPorts;
#endif

// MSP430 Launchpad has P2.0 connected to nReset
void tpsReset (void)
{
    // Toggle nReset Pin
    //P2OUT &= ~BIT0;
    //__delay_cycles(1000);
    //P2OUT |=  BIT0;
    TPS23861_RST = 0;
    DelayMs(10);
    TPS23861_RST = 1;
}


/*************************************************************************************************************************************************/
/*!     \file TPS23861.c   
*
*       \brief Functions that configure and control the TPS23861 Power over Ethernet controller
*                                                                                                                                                 
*       \date January 2013                                                                                                          
*
*       These software application programming interface functions will allow the user to configure and control the TPS23861 PoE controller.
*
*       These functions are written in C programming language. In order to support numerous processors and hardware systems,
*       the hardware interfaces are all abstracted with functions located in the TPS238x_Glue.c file. The TPS2387 functions 
*       will reference a generic I2C read and write function, which the user will convert in the glue layer functions into a 
*       specific I2C function for the hardware and OS in use by the user solution.
*
*       \note that the functions in this file are not re-entrant. It is the user's responsibility to assure that these functions
*       are not called until the previous function has completed. 
*/                                                                                                                                                 
/**************************************************************************************************************************************************
*       Copyright ? 2013-2014 Texas Instruments Incorporated - http://www.ti.com/                                                                      *
***************************************************************************************************************************************************
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************************************************************************************************************************************************
*                                 MODULE CHANGE LOG                                                                                               *
*                                                                                                                                                 *
*       Date Changed:             { Date }                        Developer:       { Name }                                                       *
*       Change Description:       { Description }                                                                                                 *
*                                                                                                                                                 *
**************************************************************************************************************************************************/
/**************************************************************************************************************************************************
*                                 Included Headers                                                                                                *
**************************************************************************************************************************************************/
//#include "msp430.h"


/**************************************************************************************************************************************************
*                                 Definitions                                                                                                     *
**************************************************************************************************************************************************/

/**************************************************************************************************************************************************
*                                 Prototypes                                                                                                      *
**************************************************************************************************************************************************/

/**************************************************************************************************************************************************
*                                 Global Variables                                                                                                *
**************************************************************************************************************************************************/
uint8_t ICUT12 = 0x11;
uint8_t ICUT34 = 0x11;
uint8_t ILIM = 0;
/*************************************************************************************************************************************************/
/*  This section of code contains public functions that are generally used by standard applications. The parameter sets for public API functions */
/*  provide an abstraction layer that will be maintained throughout updates and changes to underlying processor functions.                       */
/*                                                                                                                                               */   
/*! \cond PUBLIC                                                                                                                                 */
/*************************************************************************************************************************************************/
static TPS238x_System_Port_Map_t TPS_PortMap[TPS_MAX_SYSTEM_PORTS] =  {{TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},

                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},

                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},

                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},

                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},

                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},

                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},

                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
                                                                       {TPS_PORT_NOT_REGISTERED_VALUE, (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE},
};

/*************************************************************************************************************************************************
*  tps_RegisterPort
**************************************************************************************************************************************************/
/*!
* @brief Allocates a system level port number for a given TPS23861 device and it's specific port number.
*
* It is easier for users to maintain a system with 4 TPS23861 with 4 ports each, when each port has a unique number. By registering each of the 16
* individual ports (in this example), the user can work with ports 0-15, rather than have to maintain device 1 - port 0, through device 4 - port 3.
*
* This function must be called prior to any function call that has a systemPortNum as an input. The current logic has room for
* TPS_MAX_SYSTEM_PORTS (64) ports. The TPS_PortMap[] variable can be modified to support more ports (or save some memory by supporting fewer).
*
* @param[in]   device_i2c_address  7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   devicePortNum       TPS238X_PORT_1 - TPS238X_PORT_4, The port number ON THE SPECIFIC TPS23861 device
*
* @return  uint8_t  systemPortNum     Unique handle that will be used to refer to this device and port number in future function calls.
* @return  TPS_ERR_NO_PORT_AVAILABLE  No port number available, all are in use
*
* @sa tps_ReleasePort ()
**************************************************************************************************************************************************/
uint8_t tps_RegisterPort (uint16_t device_i2c_address, TPS238x_PortNum_t devicePortNum)
{
    uint8_t i;
    uint8_t found = 0xff;

    for (i=0; i<TPS_MAX_SYSTEM_PORTS; i++)
    {
        if (TPS_PortMap[i].i2cAddress == TPS_PORT_NOT_REGISTERED_VALUE)
        {
            found = i;
            TPS_PortMap[i].i2cAddress = device_i2c_address;
            TPS_PortMap[i].devicePortNum = devicePortNum;
            break;
        }
    }

    if (found == 0xff)
        return (TPS_ERR_NO_PORT_AVAILABLE);
    else
        return (found);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceI2CAddress
**************************************************************************************************************************************************/
/*!
* @brief Get the TPS23861 I2C Address associated with a registered System Port Number
*
* This function returns the I2C address for the TPS23861 device registered for a given System Port Number
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint16_t    I2C Address number associated with this specific port.
* @return  TPS_PORT_NOT_REGISTERED_VALUE - Indicates no port associated.
*
* @sa tps_RegisterPort ()
* @sa tps_GetDevicePortNum ()
**************************************************************************************************************************************************/
uint16_t tps_GetDeviceI2CAddress (uint8_t systemPortNum)
{
    return (TPS_PortMap[systemPortNum].i2cAddress);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceI2CAddress
**************************************************************************************************************************************************/
/*!
* @brief Get the TPS23861 I2C Address associated with a registered System Port Number
*
* This function returns the I2C address for the TPS23861 device registered for a given System Port Number
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint16_t    Device Port Number associated with this specific System Port Number.
* @return  TPS_PORT_NOT_REGISTERED_VALUE - Indicates no port associated.
*
* @sa tps_RegisterPort ()
* @sa tps_GetDeviceI2CAddress ()
**************************************************************************************************************************************************/
TPS238x_PortNum_t tps_GetDevicePortNum (uint8_t systemPortNum)
{
    return (TPS_PortMap[systemPortNum].devicePortNum);
}

/*************************************************************************************************************************************************
*  tps_GetSystemPortNumber
**************************************************************************************************************************************************/
/*!
* @brief Returns the handle already allocated for the given TPS23861 and device port number.
*
* This function can be used if the System Port Number returned in the tps_RegisterPort() function is lost or not maintained.
*
* @param[in]   device_i2c_address   7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   devicePortNum        TPS238X_PORT_1 - TPS238X_PORT_4, The port number ON THE SPECIFIC TPS23861 device
*
* @return  uint8_t  systemPortNum         Unique handle that will be used to refer to this device and port number in future function calls.
* @return  TPS_PORT_NOT_REGISTERED_VALUE  This combination of device and port number was not registered
*
* @sa tps_RegisterPort ()
**************************************************************************************************************************************************/
uint8_t tps_GetSystemPortNumber (uint16_t device_i2c_address, TPS238x_PortNum_t devicePortNum)
{
    uint8_t i;

    for (i=0; i<TPS_MAX_SYSTEM_PORTS; i++)
    {
        if ((TPS_PortMap[i].i2cAddress == device_i2c_address) && (TPS_PortMap[i].devicePortNum == devicePortNum))
        {
            return (i);
        }
    }

    return (TPS_PORT_NOT_REGISTERED_VALUE);
}

/*************************************************************************************************************************************************
*  tps_SetI2CAddresses
**************************************************************************************************************************************************/
/*!
* @brief Allows a set of TPS23861 to be configured with unique I2C addresses
*
* The system is designed so a number of TPS23861 can be placed on the same I2C bus. For this to work, each device must have a unique I2C address to
* allow them to be individually commanded. This function will program a set of TPS23861 devices on the same I2C bus with different addresses. The new
* I2C address will be stored in non-volatile memory and become the new I2C address for the specific part, even after power off.
*
* @param[in]   temp_i2cAddress           A temporary 7 bit I2C address that will be used during the address programming of the TPS238x parts
*                                             (do not included R/W bit)
* @param[in]   numDevices                Number of TPS23861 devices on the I2C bus
* @param[in]   list_ofAddresses          A pointer to a list of uint8_t 7 bit I2C addresses for each of the TPS23861 parts on the I2C bus  
*                                             (do not included R/W bit)
* @param[in]   list_ofAutoMode           A pointer to a list of TPS_On_Off_t values that contain TPS_ON for the devices that operate in AUTO mode
*                                              and TPS_OFF for those that do not operate in AUTO mode
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/
uint8_t tps_SetI2CAddresses (uint8_t temp_i2cAddress, uint8_t numDevices, uint8_t *list_ofAddresses, TPS238x_On_Off_t *list_ofAutoMode)
{
    uint8_t rtn = I2C_OPERATION_SUCCESSFUL;
    unsigned char current_address;
    uint8_t value;
    uint8_t i;
    uint8_t newAutoBitSetting;

    ////uart_puts ("\r\nSetting I2C Addresses\r\n\n");
    rtn = tps_WriteI2CReg (TPS238X_BROADCAST_ADDRESS, TPS238X_UNLOCK_CODE, temp_i2cAddress);

    // Program each devices I2C address
    for (i=0; i<numDevices; i++)
    {
        value = list_ofAddresses[i];

        if (list_ofAutoMode[i] == TPS_ON)
        {
          value |= AUTO_BIT;
        }

        temp_i2cAddress |= (list_ofAddresses[i] & 0x08);

        ////uart_puts ("   - value: ");
        ////uart_putLong ((uint32_t)(value));
        ////uart_puts ("\r\n");

        rtn |= tps_WriteI2CReg (temp_i2cAddress, TPS238X_I2C_SLAVE_ADDRESS_COMMAND, value);
    }

    // Datasheet lists a maximum program time of 100 ms for the EEPROM
    // We will wait 110 ms with 8 MHz count to allow the EEPROM change to take effect
    //__delay_cycles (880000);
    DelayMs(110);

    // Verify that each device responds to it's new address. Read back the Address Register
    for (i=0; i<numDevices; i++)
    {
        rtn |= tps_ReadI2CReg (list_ofAddresses[i], TPS238X_I2C_SLAVE_ADDRESS_COMMAND, &current_address);

        newAutoBitSetting = (current_address & AUTO_BIT);  // Save the new auto bit setting
        current_address &= 0x7F;            // The most significant bit of the slave address register is not an address bit

        if (((newAutoBitSetting == AUTO_BIT) && (list_ofAutoMode[i] != TPS_ON)) || ((newAutoBitSetting != AUTO_BIT) && (list_ofAutoMode[i] == TPS_ON)))
        {
            ////uart_puts ("\r\nI2C Auto Bit Mismatch in Address ");
            ////uart_putLong ((uint32_t)(current_address));
            ////uart_puts (",  I2C Auto Bit Set to ");
            ////uart_putLong ((uint32_t)((newAutoBitSetting >> 7)));
            ////uart_puts ("\r\n");
            rtn |= TPS_ERR_I2C_AUTOBIT_MISMATCH;
        }

        if (current_address != list_ofAddresses[i])
        {
            ////uart_puts ("\r                                           \r");
            ////uart_puts ("I2C Address Mismatch\r\n");
          rtn |= TPS_ERR_I2C_ADDRESS_MISMATCH;
        }
    }

    return (rtn);
}

/**************************************************************************************************************************************************
*                                 Interrupt Configuration Functions
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_SetDeviceInterruptMask
**************************************************************************************************************************************************/
/*!
* @brief Set the interrupt mask register
*
* The function sets which TPS23861 events/faults are allowed to generate interrupts (unmasked).
*
* @param[in]   device_i2c_address 7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   intMask            TPS238X_Interrupt_Mask_Register_t variable that contains the conditions that are allowed to generate interrupts
*                                        PEMSK_Power_Enable_Unmask            
*                                        PGMSK_Power_Good_Unmask              
*                                        DIMSK_Disconnect_Unmask              
*                                        DEMSK_Detection_Cycle_Unmask         
*                                        CLMSK_Classificiation_Cycle_Unmask   
*                                        IFMSK_IFAULT_Unmask                  
*                                        INMSK_Inrush_Fault_Unmask            
*                                        SUMSK_Supply_Event_Fault_Unmask  
* @param[in] intDelayTime         Amount of time, with a 10ms lsb, to defer non-critical interrupts. Allows for "grouping" of events
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptStatus ()
* @sa tps_GetDeviceAllInterruptEvents ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceInterruptMask (uint8_t device_i2c_address, TPS238X_Interrupt_Mask_Register_t intMask, uint8_t intDelayTime)
{
    uint8_t rtn;
    uint8_t value;
  
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_INTERRUPT_MASK_COMMAND, *(uint8_t*)&intMask);

    rtn |= tps_ReadI2CReg (device_i2c_address, TPS238X_CLASS_FIVE_TIMER_ENABLE_COMMAND, &value);
    value = (value & ~TMR_MASK) | intDelayTime;
    rtn |= tps_WriteI2CReg (device_i2c_address, TPS238X_CLASS_FIVE_TIMER_ENABLE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceInterruptMask
**************************************************************************************************************************************************/
/*!
* @brief Get the current setting of the interrupt mask register
*
* The function returns the TPS23861 events/faults that are able to generate interrupts (unmasked).
*
* @param[in]   device_i2c_address  7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *intMask            Address of a TPS238X_Interrupt_Mask_Register_t variable that will the current events that can generate interrupts
*                                        PEMSK_Power_Enable_Unmask            
*                                        PGMSK_Power_Good_Unmask              
*                                        DIMSK_Disconnect_Unmask              
*                                        DEMSK_Detection_Cycle_Unmask         
*                                        CLMSK_Classificiation_Cycle_Unmask   
*                                        IFMSK_IFAULT_Unmask                  
*                                        INMSK_Inrush_Fault_Unmask            
*                                        SUMSK_Supply_Event_Fault_Unmask      
* @param[out]  *intDelayTime       Address of a uint8_t variable that will indicate the amount of defer time, with a 10ms lsb, of non-critical interrupts.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptStatus ()
* @sa tps_GetDeviceAllInterruptEvents ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceInterruptMask (uint8_t device_i2c_address, TPS238X_Interrupt_Mask_Register_t *intMask, uint8_t *intDelayTime)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_INTERRUPT_MASK_COMMAND, (uint8_t *)intMask);

    rtn |= tps_ReadI2CReg (device_i2c_address, TPS238X_CLASS_FIVE_TIMER_ENABLE_COMMAND, &value);
    *intDelayTime = value & TMR_MASK;

    return (rtn);
}

/**************************************************************************************************************************************************
*                                 Interrupt Based Event Functions                                                                                                *
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_GetDeviceInterruptStatus
**************************************************************************************************************************************************/
/*!
* @brief Get the current interrupt status for the indicated TPS23861 part.
*
* The function returns a variable that has a one bit indicator for each of the interrupts in the TPS23861. 
*
* The interrupt mask register identifies which events/faults will generate an interrupt. The status register will still indicate
* events/faults that are masked and would not generate an interrupt. If the user wants to only process unmasked interrupts, the results 
* from this function must be combined with the interrupt mask (tps_GetInterruptMask)
*
* @param[in]   device_i2c_address  7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *status             Address of a TPS238X_Interrupt_Register_t variable that will receive the current interrupt status
*                                      PEC_Power_Enable_Change          
*                                      PGC_Power_Good_Change            
*                                      DISF_Disconnect_Event            
*                                      DETC_Detection_Cycle             
*                                      CLASC_Classification_Cycle       
*                                      IFAULT_ICUT_ILIM_Fault           
*                                      INRF_Inrush_Fault                
*                                      SUPF_Supply_Event_Fault          
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceAllInterruptEvents ()
* @sa tps_SetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptMask ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceInterruptStatus (uint8_t device_i2c_address, TPS238X_Interrupt_Register_t *status)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_INTERRUPT_COMMAND, &value);

    *status = *(TPS238X_Interrupt_Register_t *)&value;
    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceAllInterruptEvents
**************************************************************************************************************************************************/
/*!
* @brief Returns all of the individual event/fault registers that generated a TPS23861 interrupt
*
* There are a number of events and faults that can generate an interrupt. This function returns the individual indicators for each of the
* interruptable event/fault conditions.
*
* Calling this function will clear the interrupts associated with these events/faults, so the user should process all indicators present.

* There are individual functions that return the event/fault status for an individual register. 
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   clearEvent                TPS_ON will cause the events to be cleared in the hardware, TPS_OFF will just read the current value of the events
* @param[out]  *powerEnablePortEvents    Address of a TPS238x_Ports_t variable that will receive the ports containing a power enable status change
* @param[out]  *powerGoodPortEvents      Address of a TPS238x_Ports_t variable that will receive the ports containing a power good status change
* @param[out]  *detectionPortEvents      Address of a TPS238x_Ports_t variable that will receive the ports that had a detection cycle
* @param[out]  *classificationPortEvents Address of a TPS238x_Ports_t variable that will receive the ports that had a classification cycle
* @param[out]  *icutPortEvents           Address of a TPS238x_Ports_t variable that will receive the ports that had a ICUT fault
* @param[out]  *disconnectPortEvents     Address of a TPS238x_Ports_t variable that will receive the ports that had a disconnect event
* @param[out]  *inrushPortEvents         Address of a TPS238x_Ports_t variable that will receive the ports that had a inrush fault at port turn on
* @param[out]  *ilimPortEvents           Address of a TPS238x_Ports_t variable that will receive the ports that had a ILIM fault occurred
* @param[out]  *supplyEvents             Address of a TPS238X_Supply_Event_Register_t variable that will receive the supply event faults
*                                               VPUV_VPower_Undervoltage_Event          
*                                               VDUV_Vdd_UVLO_Event                     
*                                               TSD_Thermal_Shutdown_Event              
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @note Since the individual hardware registers contain more than one status in them, event status cannot be processed individually.
*
* @sa tps_SetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptMask ()
* @sa tps_GetDeviceInterruptStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceAllInterruptEvents (uint8_t device_i2c_address,
                                         TPS238x_On_Off_t clearEvent,
                                        TPS238x_Ports_t *powerEnablePortEvents,
                                     TPS238x_Ports_t *powerGoodPortEvents,
                                     TPS238x_Ports_t *detectionPortEvents,
                                     TPS238x_Ports_t *classificationPortEvents,
                                         TPS238x_Ports_t *icutPortEvents,
                                     TPS238x_Ports_t *disconnectPortEvents,
                                     TPS238x_Ports_t *inrushPortEvents,
                                     TPS238x_Ports_t *ilimPortEvents,
                                     TPS238X_Supply_Event_Register_t *supplyEvents)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t value_out;

    // Assure that the interrupt pin is released
//    tps_ResetInterruptPin (device_i2c_address);

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_EVENT_COMMAND + clearEvent, &value);
    value_out = value & 0xf;
    *powerEnablePortEvents = value_out;

    value_out = value >> POWER_ENABLE_EVENT_SHIFT;
    *powerGoodPortEvents = value_out;

    rtn += tps_ReadI2CReg (device_i2c_address, TPS238X_DETECTION_EVENT_COMMAND + clearEvent, &value);
    value_out = value & 0xf;
    *detectionPortEvents = value_out;

    value_out = value >> CLASSIFICATION_EVENT_SHIFT;
    *classificationPortEvents = value_out;

    rtn += tps_ReadI2CReg (device_i2c_address, TPS238X_FAULT_EVENT_COMMAND + clearEvent, &value);
    value_out = value & 0xf;
    *icutPortEvents = value_out;

    value_out = value >> DISCONNECT_EVENT_SHIFT;
    *disconnectPortEvents = value_out;

    rtn += tps_ReadI2CReg (device_i2c_address, TPS238X_INRUSH_LIMIT_EVENT_COMMAND + clearEvent, &value);
    value_out = value & 0xf;
    *inrushPortEvents = value_out;

    value_out = value >> ILIM_EVENT_SHIFT;
    *ilimPortEvents = value_out;

    rtn += tps_ReadI2CReg (device_i2c_address, TPS238X_SUPPLY_EVENT_COMMAND + clearEvent, &value);
    *supplyEvents = *(TPS238X_Supply_Event_Register_t*)&value;

    return (rtn);
}

/**************************************************************************************************************************************************
*                                 System Status Functions                                                                                                *
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_GetPortDetectClassStatus                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection and classification status of the specified port
*
* This function will return the most recent detection and classification result for the indicated port on the TPS23861.
*
* The function will return a 0 (CLASS_UNKNOWN) when the port is turned off.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectionStatus       Address of a TPS238x_Detection_Status_t variable that will receive the detecttion status
*                                            for the indicated port (DETECT_UNKNOWN, DETECT_SHORT_CIRCUIT, DETECT_RESIST_LOW,
*                                            DETECT_RESIST_VALID, DETECT_RESIST_HIGH, DETECT_OPEN_CIRCUIT, DETECT_MOSFET_FAULT,
*                                            DETECT_LEGACY, DETECT_CAP_INVALID_CLAMP_VOLTAGE, DETECT_CAP_INVALID_DELTA_V, or
*                                            DETECT_CAP_VALID_LEGACY_RANGE)
* @param[out]  *classificationStatus  Address of a TPS238x_Classification_Status_t variable that will receive the classification status
*                                            for the indicated port (CLASS_UNKNOWN, CLASS_1, CLASS_2, CLASS_3, CLASS_4,
*                                            CLASS_0, CLASS_OVERCURRENT, CLASS_MISMATCH, or CLASS_5) 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectionStatus ()
* @sa tps_GetPortClassificationStatus ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectClassStatus (uint8_t systemPortNum, TPS238x_Detection_Status_t *detectionStatus,
                                      TPS238x_Classification_Status_t *classificationStatus)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
    uint8_t command = TPS238X_PORT_1_STATUS_COMMAND + (uint8_t) portNum - 1;   // Determine which Port Status command has been requested

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, &value);

    *classificationStatus = (TPS238x_Classification_Status_t)GET_CLASS(value);
    *detectionStatus = (TPS238x_Detection_Status_t)GET_DETECT(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortDetectionStatus                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection status of the specified port
*
* This function will return the most recent detection result for the indicated port on the TPS23861.
*
* The function will return a 0 (CLASS_UNKNOWN) when the port is turned off.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectionStatus       Address of a TPS238x_Detection_Status_t variable that will receive the detecttion status
*                                            for the indicated port (DETECT_UNKNOWN, DETECT_SHORT_CIRCUIT, DETECT_RESIST_LOW,
*                                            DETECT_RESIST_VALID, DETECT_RESIST_HIGH, DETECT_OPEN_CIRCUIT, DETECT_MOSFET_FAULT,
*                                            DETECT_LEGACY, DETECT_CAP_INVALID_CLAMP_VOLTAGE, DETECT_CAP_INVALID_DELTA_V, or
*                                            DETECT_CAP_INVALID_LEGACY_RANGE)
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortClassificationStatus ()
* @sa tps_GetPortDetectClassStatus ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectionStatus (uint8_t systemPortNum, TPS238x_Detection_Status_t *detectionStatus)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
    uint8_t command = TPS238X_PORT_1_STATUS_COMMAND + (uint8_t) portNum - 1;   // Determine which Port Status command has been requested

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, &value);

    *detectionStatus = (TPS238x_Detection_Status_t)GET_DETECT(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortClassificationStatus                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Returns the classification status of the specified port
*
* This function will return the most recent classification result for the indicated port on the TPS23861.
*
* The function will return a 0 (CLASS_UNKNOWN) when the port is turned off.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *classificationStatus  Address of a TPS238x_Classification_Status_t variable that will receive the classification status
*                                            for the indicated port (CLASS_UNKNOWN, CLASS_1, CLASS_2, CLASS_3, CLASS_4,
*                                            CLASS_0, CLASS_OVERCURRENT, CLASS_MISMATCH, or CLASS_5) 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectionStatus ()
* @sa tps_GetPortDetectClassStatus ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortClassificationStatus (uint8_t systemPortNum, TPS238x_Classification_Status_t *classificationStatus)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
    uint8_t command = TPS238X_PORT_1_STATUS_COMMAND + (uint8_t) portNum - 1;   // Determine which Port Status command has been requested

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), command, &value);

    *classificationStatus = (TPS238x_Classification_Status_t)GET_CLASS(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDevicePowerStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power enable and power good status of the 4 ports in the TPS23861
*
* This function will return both the power enable and the power good state of each port.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *powerEnablePorts         Address of a TPS238x_Ports_t variable that will receive the power enable status with
*                                            one bit for each port in power enabled state. 
* @param[out]  *powerGoodPorts           Address of a TPS238x_Ports_t variable that will receive the power good status with
*                                            one bit for each port in power good state. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDevicePowerStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerEnablePorts, TPS238x_Ports_t *powerGoodPorts)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t value_out;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, &value);

    value_out = GET_POWER_ENABLE_STATUS (value);
    *powerEnablePorts = value_out;

    value_out = GET_POWER_GOOD_STATUS(value);
    *powerGoodPorts = value_out;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDevicePowerEnableStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power enable status of the 4 ports in the TPS23861
*
* This function will return the power good state of each port.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *powerEnablePorts         Address of a TPS238x_Ports_t variable that will receive the power enable status with
*                                            one bit for each port in power enabled state. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDevicePowerEnableStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerEnablePorts)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, &value);

    value = GET_POWER_ENABLE_STATUS(value);
    *powerEnablePorts = *(TPS238x_Ports_t*)&value;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortPowerEnableStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power enable status of one of the 4 ports in the TPS23861
*
* This function will return the power enable state of the indicated port.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t     TPS_ON if Power Enabled
*                      TPS_OFF is Power is NOT enabled,
*                      TPS_ERR_I2C_ERROR on I2C error status
*
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortPowerEnableStatus (uint8_t systemPortNum)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POWER_STATUS_COMMAND, &value);
    if (rtn != I2C_SUCCESSFUL)
        return (TPS_ERR_I2C_ERROR);

    value = GET_POWER_ENABLE_STATUS(value);
    if (value & CONVERT_PORT_NUM(portNum))
        return (TPS_ON);
    else
        return (TPS_OFF);

}

/*************************************************************************************************************************************************
*  tps_GetDevicePowerGoodStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power good status of the 4 ports in the TPS23861
*
* This function will return the power good state of each port.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *powerGoodPorts           Address of a TPS238x_Ports_t variable that will receive the power good status with
*                                            one bit for each port in power good state. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortPowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDevicePowerGoodStatus (uint8_t device_i2c_address, TPS238x_Ports_t *powerGoodPorts)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_POWER_STATUS_COMMAND, &value);

    value = GET_POWER_GOOD_STATUS(value);
    *powerGoodPorts = value;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortPowerGoodStatus
**************************************************************************************************************************************************/
/*!
* @brief Returns the power good status of one of the 4 ports in the TPS23861
*
* This function will return the power good state of the indicated port.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t     TPS_ON if Power Good
*                      TPS_OFF is Power is NOT good,
*                      TPS_ERR_I2C_ERROR on I2C error status
*
* @sa tps_GetDevicePowerGoodStatus ()
* @sa tps_GetDevicePowerEnableStatus ()
* @sa tps_GetPortPowerEnableStatus ()
* @sa tps_GetDevicePowerStatus ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortPowerGoodStatus (uint8_t systemPortNum)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POWER_STATUS_COMMAND, &value);
    if (rtn != I2C_SUCCESSFUL)
        return (TPS_ERR_I2C_ERROR);

    value = GET_POWER_GOOD_STATUS(value);
    if (value & CONVERT_PORT_NUM(portNum))
        return (TPS_ON);
    else
        return (TPS_OFF);

}

/**************************************************************************************************************************************************
*                                 System Control Functions                                                                                                *
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_SetDevicePowerOn
**************************************************************************************************************************************************/
/*!
* @brief Configures the power on state for each of the device ports on a specific TPS23861
*
* This function will power on multiple ports at the same time. Each of the ports indicated in the TPS23861 will be powered on. 
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   portsPoweredOn            A TPS238x_Ports_t variable that identifies the ports that will be powered on. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDevicePowerOff ()
* @sa tps_SetPortPower ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_SetDevicePowerOn (uint8_t device_i2c_address, TPS238x_Ports_t portsPoweredOn)
{
    uint8_t rtn;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_POWER_ENABLE_COMMAND, *(uint8_t*)&portsPoweredOn);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDevicePowerOff
**************************************************************************************************************************************************/
/*!
* @brief Configures the power down (off) state for each of the device ports on a specific TPS23861
*
* This function will power down (off) multiple ports at the same time. Each of the ports indicated in the TPS23861 will be powered off. 
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   portsPoweredOff           A TPS238x_Ports_t variable that identifies the ports that will be powered off. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDevicePowerOn ()
* @sa tps_SetPortPower ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_SetDevicePowerOff (uint8_t device_i2c_address, TPS238x_Ports_t portsPoweredOff)
{
    uint8_t rtn;
    uint8_t value;

    value = *(uint8_t*)&portsPoweredOff << POWER_OFF_SHIFT;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_POWER_ENABLE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortPower                                                                                                                   
**************************************************************************************************************************************************/
/*!
* @brief Power on or off a single specified system port
*
* This function will either power on or power down (off) a single port on one of the registered TPS23861 ports in the system.
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   on_off                 Indicates whether to power on (TPS_ON) or power off (TPS_OFF) the specified port
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDevicePowerOn ()
* @sa tps_SetDevicePowerOff ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortPower (uint8_t systemPortNum, TPS238x_On_Off_t on_off)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portBit = tps_GetDevicePortNum(systemPortNum);

    portBit = CONVERT_PORT_NUM(portBit);

    // This TPS register is Write Only with bits for forcing a Power Off and separate bits for Power On
    if (on_off == TPS_OFF)
        value = (portBit << POWER_OFF_SHIFT);
    else
        value = (portBit << POWER_ON_SHIFT);

    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POWER_ENABLE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceDetectionEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns the device ports that are enabled for detection operation on the specific TPS23861
*
* This function will return the ports that are configured for detection operations. 
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *detectPorts              Address of a TPS238x_Ports_t variable that will receive the device ports that are enabled for detection
*                                            with one bit for each port in detection enabled state. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectionEnable ()
* @sa tps_GetDeviceClassificationEnable ()
* @sa tps_GetDeviceDetectClassEnable ()
* @sa tps_SetDeviceDetectClassEnable ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceDetectionEnable (uint8_t device_i2c_address, TPS238x_Ports_t *detectPorts)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    *(uint8_t *)detectPorts = GET_DETECT(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortDetectionEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns whether the indicated system port is enabled for detection operation
*
* This function will return a TPS_ON or TPS_OFF to indicate that the specific system port is enabled for detection or not
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    TPS_ON  - Port is enabled for detection
*                     TPS_OFF - Port is NOT enabled for detection
*
* @sa tps_GetDeviceDetectionEnable ()
* @sa tps_GetPortClassificationEnable ()
* @sa tps_SetPortDetectClassEnable ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectionEnable (uint8_t systemPortNum)
{
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    value = GET_DETECT(value);

    if (value & CONVERT_PORT_NUM(portNum))
        return (TPS_ON);
    else
        return (TPS_OFF);

}

/*************************************************************************************************************************************************
*  tps_GetDeviceClassificationEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns the device ports that are enabled for classification operation for a specific TPS23861 device
*
* This function will return the ports that are configured for classification operations. 
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *classPorts               Address of a TPS238x_Ports_t variable that will receive the ports that are enabled for classification
*                                            with one bit for each port in classification enabled state. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortClassificationEnable ()
* @sa tps_GetDeviceDetectionEnable ()
* @sa tps_GetDeviceDetectClassEnable ()
* @sa tps_SetDeviceDetectClassEnable ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceClassificationEnable (uint8_t device_i2c_address, TPS238x_Ports_t *classPorts)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    *(uint8_t *)classPorts  = GET_CLASS(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortClassificationEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns whether the indicated system port is enabled for classification operation
*
* This function will return whether the indicated system port is configured for classification operations.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    TPS_ON  - Port is enabled for classification
*                     TPS_OFF - Port is NOT enabled for classification
*
* @sa tps_GetDeviceClassificationEnable ()
* @sa tps_GetPortDetectionEnable ()
* @sa tps_SetPortDetectClassEnable ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortClassificationEnable (uint8_t systemPortNum)
{
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    value = GET_CLASS(value);

    if (value & CONVERT_PORT_NUM(portNum))
        return (TPS_ON);
    else
        return (TPS_OFF);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceDetectClassEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns the device ports that are enabled for detection and classification operation on the specified TPS23861.
*
* This function will return the device ports that are configured for detection and classification operations.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *detectPorts              Address of a TPS238x_Ports_t variable that will receive the ports that are enabled for detection
*                                            with one bit for each port in detection enabled state. 
* @param[out]  *classPorts               Address of a TPS238x_Ports_t variable that will receive the ports that are enabled for classification
*                                            with one bit for each port in classification enabled state. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceClassificationEnable ()
* @sa tps_GetDeviceDetectionEnable ()
* @sa tps_SetDeviceDetectClassEnable ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceDetectClassEnable (uint8_t device_i2c_address, TPS238x_Ports_t *detectPorts, TPS238x_Ports_t *classPorts)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    *(uint8_t *)detectPorts = GET_DETECT(value);
    *(uint8_t *)classPorts  = GET_CLASS(value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDeviceDetectClassEnable
**************************************************************************************************************************************************/
/*!
* @brief Configures the device ports that are enabled for detection and classification operation for a specified TPS23861
*
* This function will set the detection and classification operations for all of the ports on a specific TPS23861.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   detectPorts               A TPS238x_Ports_t variable that identifies the ports that will be enabled for detection
*                                            with one bit for each port in detection enabled state. 
* @param[in]   classPorts                A TPS238x_Ports_t variable that identifies the ports that will be enabled for classification
*                                            with one bit for each port in classification enabled state. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetPortDetectClassEnable ()
* @sa tps_GetDeviceClassificationEnable ()
* @sa tps_GetDeviceDetectionEnable ()
* @sa tps_GetDeviceDetectClassEnable ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceDetectClassEnable (uint8_t device_i2c_address, TPS238x_Ports_t detectPorts, TPS238x_Ports_t classPorts)
{
    uint8_t rtn;
    uint8_t value;

    value = (*(uint8_t*)&classPorts << CLASS_SHIFT) | *(uint8_t*)&detectPorts;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_ENABLE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortDetectClassEnable                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Configures a single port to enable or disable the detection and classification operation
*
* This function will set the detection and classification operations for the selected port of the TPS23861.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   on_off_detect             TPS_ON to enable detection operations for the indicated port, TPS_OFF otherwise
* @param[in]   on_off_class              TPS_ON to enable classification operations for the indicated port, TPS_OFF otherwise
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceDetectClassEnable ()
* @sa tps_SetPortDetectClassEnable ()
* @sa tps_GetPortClassificationEnable ()
* @sa tps_GetPortDetectionEnable ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortDetectClassEnable (uint8_t systemPortNum, TPS238x_On_Off_t on_off_detect, TPS238x_On_Off_t on_off_class)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_ENABLE_COMMAND, &value);

    if (on_off_class == TPS_ON)
        value |= (CONVERT_PORT_NUM(portNum) << CLASS_SHIFT);       // Set the classification enable bit
    else
        value &= ~(CONVERT_PORT_NUM(portNum) << CLASS_SHIFT);      // Clear the classification enable bit

    if (on_off_detect == TPS_ON)
        value |= (CONVERT_PORT_NUM(portNum));                      // Set the detection enable bit
    else
        value &= ~(CONVERT_PORT_NUM(portNum));                     // Clear the detection enable bit

    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_ENABLE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceDisconnectEnable
**************************************************************************************************************************************************/
/*!
* @brief Returns the ports that are enabled for 2 pair disconnection operation
*
* This function will return the ports that are configured for 2 pair disconnect operations. Disconnect operation consists of measuring the port
* current at the SENn pin, starting the TDIS timer when the current drops below the threshold, and turning the port off if the timer times out.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *disconnectPorts          Address of a TPS238x_Ports_t variable that will receive the ports that are enabled for 2 pair 
*                                            disconnection with one bit for each port in disconnect enabled state. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetPortDisconnectEnable ()
* @sa tps_SetDeviceDisconnectEnable ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceDisconnectEnable (uint8_t device_i2c_address, TPS238x_Ports_t *disconnectPorts)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_DISCONNECT_ENABLE_COMMAND, &value);

    *disconnectPorts = value;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDeviceDisconnectEnable
**************************************************************************************************************************************************/
/*!
* @brief Configures the ports to configure those that are enabled for 2 pair disconnection operation
*
* This function will configure all four ports to identify which ones support disconnection operations.
* If the user wishes to configure a single port, the tps_SetPortDisconnectEnable() is used.
*
* Disconnect operation consists of measuring the port current at the SENn pin, starting the TDIS timer when the current drops below the threshold,
* and turning the port off if the timer times out.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   disconnectPorts           A TPS238x_Ports_t variable that contains the ports that will be enabled for 2 pair 
*                                            disconnection with one bit for each port.
* @param[in]   disconnectThreshold1      Disconnection threshold for port 1 [DCTH_7_5_MILLIAMP, DCTH_15_MILLIAMP, DCTH_30_MILLIAMP or DCTH_50_MILLIAMP]
* @param[in]   disconnectThreshold2      Disconnection threshold for port 2 [DCTH_7_5_MILLIAMP, DCTH_15_MILLIAMP, DCTH_30_MILLIAMP or DCTH_50_MILLIAMP]
* @param[in]   disconnectThreshold3      Disconnection threshold for port 3 [DCTH_7_5_MILLIAMP, DCTH_15_MILLIAMP, DCTH_30_MILLIAMP or DCTH_50_MILLIAMP]
* @param[in]   disconnectThreshold4      Disconnection threshold for port 4 [DCTH_7_5_MILLIAMP, DCTH_15_MILLIAMP, DCTH_30_MILLIAMP or DCTH_50_MILLIAMP]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetPortDisconnectEnable ()
* @sa tps_GetDeviceDisconnectEnable ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceDisconnectEnable (uint8_t device_i2c_address, TPS238x_Ports_t disconnectPorts, TPS238x_Disconnect_Threshold_t disconnectThreshold1,
                                       TPS238x_Disconnect_Threshold_t disconnectThreshold2, TPS238x_Disconnect_Threshold_t disconnectThreshold3,
                                       TPS238x_Disconnect_Threshold_t disconnectThreshold4)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DISCONNECT_ENABLE_COMMAND, *(uint8_t*)&disconnectPorts);

    value = (disconnectThreshold4 << 6) | (disconnectThreshold3 << 4) | (disconnectThreshold2 << 2) | disconnectThreshold1;
    rtn += tps_WriteI2CReg (device_i2c_address, TPS238X_DISCONNECT_THRESHOLD_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortDisconnectEnable                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Configures a single port to enable or disable the 2 pair disconnection operation
*
* This function will configure one of the four ports to enable or disable disconnection operations.
* If the user wishes to configure a all ports at once, the tps_SetDisconnectEnable() is used.
*
* Disconnect operation consists of measuring the port current at the SENn pin, starting the TDIS timer when the current drops below the threshold,
* and turning the port off if the timer times out.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   on_off                    TPS_ON to enable disconnection operations for the indicated port, TPS_OFF otherwise
* @param[in]   disconnectThreshold       Disconnection threshold for indicated port [DCTH_7_5_MILLIAMP, DCTH_15_MILLIAMP,
*                                                                                    DCTH_30_MILLIAMP or DCTH_50_MILLIAMP]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceDisconnectEnable ()
* @sa tps_GetDeviceDisconnectEnable ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortDisconnectEnable (uint8_t systemPortNum, TPS238x_On_Off_t on_off, TPS238x_Disconnect_Threshold_t disconnectThreshold)
{
    uint8_t rtn;
    uint8_t disconnectValue;
    uint8_t thresholdValue;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
    uint16_t i2cAddress = tps_GetDeviceI2CAddress(systemPortNum);

    rtn = tps_ReadI2CReg (i2cAddress, TPS238X_DISCONNECT_ENABLE_COMMAND, &disconnectValue);

    if (on_off == TPS_ON)
    {
        disconnectValue |= CONVERT_PORT_NUM(portNum);              // Set the disconnect enable bit

        // Load the Disconnect Threshold
        rtn += tps_ReadI2CReg (i2cAddress, TPS238X_DISCONNECT_THRESHOLD_COMMAND, &thresholdValue);
        thresholdValue &= ~(DISCONNECT_THRESHOLD_MASK << (2 * ((uint8_t) portNum - 1)));         // Clear old value for this port number
        thresholdValue |= ((uint8_t)disconnectThreshold << (2 * ((uint8_t) portNum - 1)));       // Each port gets 2 bits
        rtn += tps_WriteI2CReg (i2cAddress, TPS238X_DISCONNECT_THRESHOLD_COMMAND, thresholdValue);
    }
    else
        disconnectValue &= ~(CONVERT_PORT_NUM(portNum));           // Clear the disconnect enable bit

    rtn += tps_WriteI2CReg (i2cAddress, TPS238X_DISCONNECT_ENABLE_COMMAND, disconnectValue);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDeviceTiming
**************************************************************************************************************************************************/
/*!
* @brief Configures the timing of the various current limits and disconnection determinations for a given device in the system.
*
* The TPS23861 has three levels of overcurrent protections, disconnection detection, and time out after fault conditions. In order to prevent 
* glitches or transients from causing unnecessary shutdowns or faults, the system uses various timers to define the amount of time required
* over the threshold level before a fault or shutdown is declared.

* This function sets up the various timing parameters used in the system. These timings are applicable on all active ports.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   ilimTiming                The ILIM Fault Timing which is the foldback current time limit before port turn off
*                                             [TLIM_60_MS, TLIM_30_MS, TLIM_15_MS, or TLIM_10_MS] 
* @param[in]   startTiming               The length of the TSTART period, which is the maximum amount of allowed overcurrent time during inrush
*                                             [TSTART_30_MS, TSTART_60_MS, or TSTART_120_MS]
* @param[in]   icutTiming                ICUT Fault timing, which is the overcurrent time duration before port turn off
*                                             [TICUT_30_MS, TICUT_60_MS, TICUT_120_MS, or TICUT_240_MS]
* @param[in]   disconnectTiming          Disconnect delay, which is the time to turn off a port once there is a disconnect condition
*                                             [TDIS_90_MS, TDIS_180_MS, TDIS_360_MS, or TDIS_720_MS]
* @param[in]   coolDownFaultTiming       Amount of time before a port can turn back on after fault
*                                             [COOL_DOWN_1_SEC, COOL_DOWN_2_SEC, or COOL_DOWN_4_SEC]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_ConfigPort ()
* @sa tps_ConfigDevice4Pair ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceTiming (uint8_t device_i2c_address, TPS238x_ILIM_Timing_t ilimTiming, TPS238x_TStart_Timing_t startTiming,
                             TPS238x_TICUT_Timing_t icutTiming, TPS238x_TDIS_Timing_t disconnectTiming,
                             TPS238x_Cool_Down_Timing_t coolDownFaultTiming)
{
    uint8_t rtn;
    uint8_t value;

    value = (ilimTiming << TLIM_SHIFT) | ( startTiming << TSTART_SHIFT) |
            (icutTiming << TICUT_SHIFT) | (disconnectTiming << TDIS_SHIFT);

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_TIMING_CONFIGURATION_COMMAND, value);

    rtn   +=  tps_ReadI2CReg (device_i2c_address, TPS238X_COOL_DOWN_GATE_DRIVE_COMMAND, &value);
    value &= ~(CLDN_MASK);
    value |=  (coolDownFaultTiming << CLDN_SHIFT);
    rtn   +=  tps_WriteI2CReg (device_i2c_address, TPS238X_COOL_DOWN_GATE_DRIVE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_FastShutdownDeviceEnable
**************************************************************************************************************************************************/
/*!
* @brief Configures the fast shutdown enable for each of the ports on the TPS23861
*
* This function will enable or disable the fast shutdown on each of the ports of the TPS23861
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   ports                     A TPS238x_Ports_t variable that identifies the ports that will be enabled for fast shutdown
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_FastShutdownPortEnable ()
**************************************************************************************************************************************************/
uint8_t tps_FastShutdownDeviceEnable (uint8_t device_i2c_address, TPS238x_Ports_t ports)
{
    uint8_t rtn;
    uint8_t value;

    value = *(uint8_t*)&ports << FSE_SHIFT;

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_PORT_POWER_PRIORITY_COMMAND, value);

    return (rtn);

}

/*************************************************************************************************************************************************
*  tps_FastShutdownPortEnable
**************************************************************************************************************************************************/
/*!
* @brief Configures the fast shutdown enable for a specific port on the TPS23861
*
* This function will enable or disable the fast shutdown on a specific port of the TPS23861
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   on_off                    Enable (TPS_ON) or disable (TPS_OFF) the fast shutdown for the indicated port
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_FastShutdownDeviceEnable ()
**************************************************************************************************************************************************/
uint8_t tps_FastShutdownPortEnable (uint8_t systemPortNum, TPS238x_On_Off_t on_off)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_PORT_POWER_PRIORITY_COMMAND, &value);

    if (on_off == TPS_ON)
        value |=  ( CONVERT_PORT_NUM(portNum) << FSE_SHIFT);
    else
        value &= ~( CONVERT_PORT_NUM(portNum) << FSE_SHIFT);

    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_PORT_POWER_PRIORITY_COMMAND, value);

    return (rtn);

}

/*************************************************************************************************************************************************
*  tps_ConfigPort                                                                                                                      
**************************************************************************************************************************************************/
/*!
* @brief Configures the current thresholds and operating modes for the specified port 
*
* The TPS23861 has a number of the threshold values and operating modes that determine the characteristic capabilities of a given port. This
* includes whether the port can detect class 5 devices or operate in legacy detection modes. This function allows the user to configure the 
* characteristics for a given port.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   operatingMode             Define the operating mode for the port 
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO] 
* @param[in]   legacyDetect              The system can perform combinations of standard and legacy detections 
*                                             [LEGACY_DETECT_DISABLED, LEGACY_DETECT_ONLY, or LEGACY_DETECT_STANDARD_THEN_LEGACY]
* @param[in]   twoEvent                  The system can attempt two event physical layer classifications when a class 4 or class 5 PD is found
*                                             [TWO_EVENT_DISABLE, TWO_EVENT_AFTER_CLASS_4, TWO_EVENT_AFTER_CLASS_5, or TWO_EVENT_AFTER_CLASS_4_OR_5]
* @param[in]   class5Enable              Allows the system to attempt Class 5 current levels [TPS_ON or TPS_OFF]
* @param[in]   disconnectThreshold       Current threshold for disconnection 
*                                             [DCTH_7_5_MILLIAMP, DCTH_15_MILLIAMP, DCTH_30_MILLIAMP, or DCTH_50_MILLIAMP]
* @param[in]   icutCurrentThreshold      ICUT Current threshold 
*                                             [ICUT_110_MILLIAMP, ICUT_204_MILLIAMP, ICUT_374_MILLIAMP, ICUT_592_MILLIAMP,
*                                              ICUT_686_MILLIAMP, ICUT_754_MILLIAMP, or ICUT_920_MILLIAMP]
* @param[in]   poepFoldbackCurve         Defines the foldback curve applied to a port when it is turned on
*                                             [_1X_ILIM_FOLDBACK_CURVE or _2X_ILIM_FOLDBACK_CURVE]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_ConfigDevice4Pair ()
**************************************************************************************************************************************************/
uint8_t tps_ConfigPort (uint8_t systemPortNum, TPS238x_Operating_Modes_t operatingMode, TPS238x_Legacy_Detect_t legacyDetect,
                        TPS238x_Two_Event_t twoEvent, TPS238x_On_Off_t class5Enable,
                        TPS238x_Disconnect_Threshold_t disconnectThreshold, TPS238x_ICUT_Config_t icutCurrentThreshold,
                        TPS238x_POE_Plus_Foldback_t poepFoldbackCurve)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    // Load the Operating Mode
    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_OPERATING_MODE_COMMAND, &value);
    value &= ~(OPERATING_MODE_MASK << (2 * ((uint8_t) portNum - 1)));               // Clear old value for this port number
    value |= ((uint8_t)operatingMode << (2 * ((uint8_t) portNum - 1)));             // Each port gets 2 bits
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_OPERATING_MODE_COMMAND, value);

    // Load the Legacy Detection
    rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_LEGACY_DETECT_MODE_COMMAND, &value);
    value &= ~(LEGACY_MODE_MASK << (2 * ((uint8_t) portNum - 1)));                  // Clear old value for this port number
    value |= ((uint8_t)legacyDetect << (2 * ((uint8_t) portNum - 1)));              // Each port gets 2 bits
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_LEGACY_DETECT_MODE_COMMAND, value);

    // Load the Two Event Setting
    rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_TWO_EVENT_CLASSIFICATION_COMMAND, &value);
    value &= ~(TWO_EVENT_MASK << (2 * ((uint8_t) portNum - 1)));                  // Clear old value for this port number
    value |= ((uint8_t)twoEvent << (2 * ((uint8_t) portNum - 1)));                  // Each port gets 2 bits
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_TWO_EVENT_CLASSIFICATION_COMMAND, value);

    // Load the Class 5 Enable Setting
    rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_CLASS_FIVE_TIMER_ENABLE_COMMAND, &value);
    if (class5Enable == TPS_ON)
        value |= (CONVERT_PORT_NUM(portNum) << CLASS_5_ENABLE_SHIFT);                 // Set the Class 5 Enable for this port
    else
        value &= ~(CONVERT_PORT_NUM(portNum) << CLASS_5_ENABLE_SHIFT);                // Clear the Class 5 Enable for this port
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_CLASS_FIVE_TIMER_ENABLE_COMMAND, value);

    // Load the Disconnect Threshold
    rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DISCONNECT_THRESHOLD_COMMAND, &value);
    value &= ~(DISCONNECT_THRESHOLD_MASK << (2 * ((uint8_t) portNum - 1)));         // Clear old value for this port number
    value |= ((uint8_t)disconnectThreshold << (2 * ((uint8_t) portNum - 1)));       // Each port gets 2 bits
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DISCONNECT_THRESHOLD_COMMAND, value);

    // Load the ICUT Current Threshold
    if ((uint8_t) portNum <= 2)
    {
        rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT21_CONFIGURATION_COMMAND, &value);
        if ((uint8_t) portNum == 2)
        {
            value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_2_SHIFT);                      // Clear old value for this port number
            value |= ((uint8_t)icutCurrentThreshold << ICUT_PORT_2_SHIFT);             // Each port gets 3 bits
        }
        else
        {
            value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_1_SHIFT);                      // Clear old value for this port number
            value |= ((uint8_t)icutCurrentThreshold << ICUT_PORT_1_SHIFT);             // Each port gets 3 bits
        }

        rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT21_CONFIGURATION_COMMAND, value);
    }
    else
    {
        rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT43_CONFIGURATION_COMMAND, &value);
        if ((uint8_t) portNum == 4)
        {
            value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_4_SHIFT);                      // Clear old value for this port number
            value |= ((uint8_t)icutCurrentThreshold << ICUT_PORT_4_SHIFT);             // Each port gets 3 bits
        }
        else
        {
            value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_3_SHIFT);                      // Clear old value for this port number
            value |= ((uint8_t)icutCurrentThreshold << ICUT_PORT_3_SHIFT);             // Each port gets 3 bits
        }
        rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT43_CONFIGURATION_COMMAND, value);
    }

    // Load the PoE Plus Setting
    rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POE_PLUS_COMMAND, &value);
    if (poepFoldbackCurve == _2X_ILIM_FOLDBACK_CURVE)
        value |= (CONVERT_PORT_NUM(portNum) << POE_PLUS_SHIFT);                      // Set the PoE Plus setting, so we use the 2x Foldback Curve
    else
        value &= ~(CONVERT_PORT_NUM(portNum) << POE_PLUS_SHIFT);                     // Clear the PoE Plus setting, so we use the 1x Foldback Curve
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POE_PLUS_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_ConfigDevice4Pair
**************************************************************************************************************************************************/
/*!
* @brief Configures two ports to be used together as 4 pair operation
*
* The TPS23861 allows ports 1 & 2 or ports 3 & 4 to operate together to perform 4 pair PSE. There a a number of different disconnect modes that can
* be applied to a 4-pair port set. This function allows the user to enable the 4-pair port sets and define the disconnection mode to be used.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   on_off_Port12             Enables ports 1 & 2 to operate in 4-pair mode (TPS_ON)
* @param[in]   disconnectModePort12      Disconnection mode for the 4-pair port set using ports 1 and 2
*                                             [FOUR_PAIR_DISCONNECT_DISABLED, FOUR_PAIR_DISCONNECT_BASED_ON_LOWER_PORT,
*                                              FOUR_PAIR_DISCONNECT_BASED_ON_HIGHER_PORT, FOUR_PAIR_DISCONNECT_BASED_ON_EITHER_PORT, or
*                                              FOUR_PAIR_DISCONNECT_BASED_ON_BOTH_PORTS]
* @param[in]   on_off_Port34             Enables ports 3 & 4 to operate in 4-pair mode (TPS_ON)
* @param[in]   disconnectModePort34      Disconnection mode for the 4-pair port set using ports 3 and 4
*                                             [FOUR_PAIR_DISCONNECT_DISABLED, FOUR_PAIR_DISCONNECT_BASED_ON_LOWER_PORT,
*                                              FOUR_PAIR_DISCONNECT_BASED_ON_HIGHER_PORT, FOUR_PAIR_DISCONNECT_BASED_ON_EITHER_PORT, or
*                                              FOUR_PAIR_DISCONNECT_BASED_ON_BOTH_PORTS]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_ConfigPort ()
**************************************************************************************************************************************************/
uint8_t tps_ConfigDevice4Pair (uint8_t device_i2c_address, TPS238x_On_Off_t on_off_Port12, TPS238x_Four_Pair_t disconnectModePort12,
                               TPS238x_On_Off_t on_off_Port34, TPS238x_Four_Pair_t disconnectModePort34)
{
    uint8_t rtn;
    uint8_t value;

    value = ((uint8_t)on_off_Port12 << _4P12EN_SHIFT) |
          ((uint8_t)disconnectModePort12 << _4P12DIS_SHIFT) |
          ((uint8_t)on_off_Port34 << _4P34EN_SHIFT) |
          ((uint8_t)disconnectModePort34 << _4P34DIS_SHIFT);

    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_FOUR_PAIR_MODE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetPortOpMode
**************************************************************************************************************************************************/
/*!
* @brief Set the Operating Mode of a single port
*
* The TPS23861 has a number of operating modes that determine the characteristic capabilities of a given port. This function allows the user to configure the
* operating for a given port.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[in]   operatingMode             Define the operating mode for the port
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetDeviceOpMode ()
* @sa tps_ConfigPort ()
**************************************************************************************************************************************************/
uint8_t tps_SetPortOpMode (uint8_t systemPortNum, TPS238x_Operating_Modes_t operatingMode)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    // Load the Operating Mode
    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_OPERATING_MODE_COMMAND, &value);
    value &= ~(OPERATING_MODE_MASK << (2 * ((uint8_t) portNum - 1)));               // Clear old value for this port number
    value |= ((uint8_t)operatingMode << (2 * ((uint8_t) portNum - 1)));             // Each port gets 2 bits
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_OPERATING_MODE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_SetDeviceOpMode
**************************************************************************************************************************************************/
/*!
* @brief Set the Operating Mode for all of the ports on the TPS23861
*
* The TPS23861 has a number of operating modes that determine the characteristic capabilities of a given port. This function allows the user to configure the
* operating for all of the ports.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   operatingMode1            Define the operating mode for port 1
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
* @param[in]   operatingMode2            Define the operating mode for port 2
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
* @param[in]   operatingMode3            Define the operating mode for port 3
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
* @param[in]   operatingMode4            Define the operating mode for port 4
*                                            [OPERATING_MODE_OFF, OPERATING_MODE_MANUAL, OPERATING_MODE_SEMI_AUTO, or OPERATING_MODE_AUTO]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_SetPortOpMode ()
* @sa tps_ConfigPort ()
**************************************************************************************************************************************************/
uint8_t tps_SetDeviceOpMode (uint8_t device_i2c_address, TPS238x_Operating_Modes_t operatingMode1, TPS238x_Operating_Modes_t operatingMode2,
                             TPS238x_Operating_Modes_t operatingMode3, TPS238x_Operating_Modes_t operatingMode4)
{
    uint8_t rtn;
    uint8_t value;

    // Load the Operating Mode
    value = (((uint8_t)operatingMode4 << 6) | ((uint8_t)operatingMode3 << 4) | ((uint8_t)operatingMode2 << 2) | ((uint8_t)operatingMode1));             // Each port gets 2 bits
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_OPERATING_MODE_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_RestartDeviceDetection
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart of the detection process on the indicated ports on a given TPS23861 in the system
*
* This function will restart the detection process on the indicated ports on the specified TPS23861.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   detectPorts               A TPS238x_Ports_t variable that identifies the ports that will be restarted for detection
*                                            with one bit for each port restarted. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartPortDetection ()
* @sa tps_RestartDeviceClassification ()
* @sa tps_RestartDeviceDetectClass ()
* @sa tps_GetSystemPortNumber ()
**************************************************************************************************************************************************/
uint8_t tps_RestartDeviceDetection (uint8_t device_i2c_address, TPS238x_Ports_t detectPorts)
{
    uint8_t rtn;
    uint8_t value;

    value = *(uint8_t*)&detectPorts;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_RestartPortDetection                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart of the detection process on the indicated device ports for a given TPS23861
*
* This function will restart the detection process on the indicated device ports on the given TPS23861.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartPortClassification ()
* @sa tps_RestartDeviceDetection ()
* @sa tps_RestartDeviceDetection ()
**************************************************************************************************************************************************/
uint8_t tps_RestartPortDetection (uint8_t systemPortNum)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    value = CONVERT_PORT_NUM(portNum);

    // Load the Operating Mode
    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_RestartDeviceClassification
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart of the classification process on the indicated ports
*
* This function will restart the classification process on the indicated ports on the TPS23861. 
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   classPorts                A TPS238x_Ports_t variable that identifies the ports that will be restarted for classification
*                                            with one bit for each port restarted. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartPortClassification ()
* @sa tps_RestartDeviceDetection ()
* @sa tps_RestartDeviceDetectClass ()
**************************************************************************************************************************************************/
uint8_t tps_RestartDeviceClassification (uint8_t device_i2c_address, TPS238x_Ports_t classPorts)
{
    uint8_t rtn;
    uint8_t value;

    value = (*(uint8_t*)&classPorts << CLASS_SHIFT);
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_RestartPortClassification                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart of the classification process on the single indicated port
*
* This function will restart the classification process on the specified port on the TPS23861. 
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartDeviceClassification ()
* @sa tps_RestartPortDetection ()
**************************************************************************************************************************************************/
uint8_t tps_RestartPortClassification (uint8_t systemPortNum)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    value = (CONVERT_PORT_NUM(portNum) << CLASS_SHIFT);

    // Load the Operating Mode
    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_RestartDeviceDetectClass
**************************************************************************************************************************************************/
/*!
* @brief Forces a restart both the classification and detection processes on the indicated ports
*
* This function will restart both the classification and detection process on the indicated ports on the TPS23861. 
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[in]   detectPorts               A TPS238x_Ports_t variable that identifies the ports that will be restarted for detection
*                                            with one bit for each port restarted. 
* @param[in]   classPorts                A TPS238x_Ports_t variable that identifies the ports that will be restarted for classification
*                                            with one bit for each port restarted. 
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_RestartDeviceClassification ()
* @sa tps_RestartDeviceDetection ()
**************************************************************************************************************************************************/
uint8_t tps_RestartDeviceDetectClass (uint8_t device_i2c_address, TPS238x_Ports_t detectPorts, TPS238x_Ports_t classPorts)
{
    uint8_t rtn;
    uint8_t value;

    value = (*(uint8_t *)&classPorts << CLASS_SHIFT) | *(uint8_t *)&detectPorts;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_DETECT_CLASS_RESTART_COMMAND, value);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_ResetDevicePort
**************************************************************************************************************************************************/
/*!
* @brief Forces a reset of the indicated device ports on the indicated TPS23861 device.
*
* This function will reset the indicated device ports on the identified TPS23861.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_ResetPort ()
**************************************************************************************************************************************************/
uint8_t tps_ResetDevicePort (uint8_t systemPortNum)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    value = CONVERT_PORT_NUM(portNum);

    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_RESET_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_ResetPort                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Forces a reset of the single specified system port number identifying a single port on a specific registered TPS23861.
*
* This function will reset the single specified port registered with the system for a given TPS23861.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_ResetDevicePort ()
**************************************************************************************************************************************************/
uint8_t tps_ResetPort (uint8_t systemPortNum)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    value = CONVERT_PORT_NUM(portNum);

    // Load the Operating Mode
    rtn = tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_RESET_COMMAND, value);

    return (rtn);
}

/**************************************************************************************************************************************************
*                                 System Measurements
**************************************************************************************************************************************************/

/*************************************************************************************************************************************************
*  tps_GetPortMeasurements                                                                                                                   
**************************************************************************************************************************************************/
/*!
* @brief Returns the voltage and current of the registered system port.
*
* This function will return the voltage and current on the registered port on a specific TPS23861.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *voltage                  The address of a uint16_t variable where the voltage of the indicated port will be placed. The voltage 
*                                           will be a scaled integer with an LSB of 3.662 mVolts
* @param[out]  *current                  The address of a uint16_t variable where the current of the indicated port will be placed. The current
*                                           will be a scaled integer with an LSB that is dependent on the current-sense resistor. 
*                                           For a 250 mOhm resistor, the LSB will be 62.260 micro-Amps.
*                                           For a 255 mOhm resistor, the LSB will be 61.039 micro-Amps.
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetDeviceInputVoltage ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortMeasurements (uint8_t systemPortNum, uint16_t *voltage, uint16_t *current)
{
    uint8_t rtn;
    uint8_t command;
    uint8_t value_t[2];
    uint16_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    command = TPS238X_PORT_1_CURRENT_COMMAND + (4 * ((uint8_t) portNum - 1));
    rtn = tps_ReadI2CMultiple (tps_GetDeviceI2CAddress(systemPortNum), command, value_t, 2);
    value = (value_t[1] << 8);
    value |= value_t[0];
    *current = value & TPS2368X_PORT_CURRENT_MASK_SHORT;

    command = TPS238X_PORT_1_VOLTAGE_COMMAND + (4 * ((uint8_t) portNum - 1));
    rtn += tps_ReadI2CMultiple (tps_GetDeviceI2CAddress(systemPortNum), command, value_t, 2);
    value = (value_t[1] << 8);
    value |= value_t[0];
    *voltage = value & TPS2368X_PORT_VOLTAGE_MASK_SHORT;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceInputVoltage
**************************************************************************************************************************************************/
/*!
* @brief Returns the input voltage of the specified TPS23861
*
* This function will return the input voltage of the specified TPS23861.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *voltage                  The address of a uint16_t variable where the input voltage of the TPS23861. The voltage 
*                                           will be a scaled integer with an LSB of 3.662 mVolts
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortMeasurements ()
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceInputVoltage (uint8_t device_i2c_address, uint16_t *voltage)
{
    uint8_t rtn;
    uint8_t value_t[2];
    uint16_t value;

    rtn = tps_ReadI2CMultiple (device_i2c_address, TPS238X_INPUT_VOLTAGE_COMMAND, value_t, 2);
    value = (value_t[1] << 8);
    value |= value_t[0];
    *voltage = value & TPS2368X_INPUT_VOLTAGE_MASK_SHORT;

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetDeviceTemperature
**************************************************************************************************************************************************/
/*!
* @brief Returns the temperature of the specified TPS23861
*
* This function will return the temperature of the specified TPS23861.
*
* @param[in]   device_i2c_address        7 bit I2C address of the TPS238x part being controlled (do not included R/W bit)
* @param[out]  *temperature              The address of a uint8_t variable where the temperature of the TPS23861. The temperature 
*                                           will be a scaled integer with an LSB of 0.652 degrees Celsius
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
**************************************************************************************************************************************************/
uint8_t tps_GetDeviceTemperature (uint8_t device_i2c_address, uint8_t *temperature)
{
    uint8_t rtn;

    rtn = tps_ReadI2CReg (device_i2c_address, TPS238X_TEMPERATURE_COMMAND, (uint8_t *)temperature);

    return (rtn);
}


/*************************************************************************************************************************************************
*  tps_GetPortDetectResistance
**************************************************************************************************************************************************/
/*!
* @brief Returns the detection resistance value for the registered system port number on a specific TPS23861
*
* This function will return the detection resistance and detection status for the registered system port on a TPS23861.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectResistance         The address of a uint16_t variable where the detection resistance of the port. The 
*                                           detection resistance will be a scaled integer with an LSB of 11.0966 ohms.
* @param[out]  *detectResistanceStatus   The address of a TPS238x_Detect_Resistance_Status_t variable for the most recent detection result status
*                                            [RS_STATUS_GOOD, RS_STATUS_SHORT_CIRCUIT, RS_STATUS_OPEN_CIRCUIT, or RS_STATUS_MOSFET_SHORT_FAULT]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectVoltageDifference ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectResistance (uint8_t systemPortNum, uint16_t *detectResistance,
                                     TPS238x_Detect_Resistance_Status_t *detectResistanceStatus)
{
    uint8_t rtn;
    uint16_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
    uint8_t command = TPS238X_PORT_1_DETECT_RESISTANCE_COMMAND + (2 * ((uint8_t)portNum - 1));         // 2 bytes for each port

    rtn = tps_ReadI2CMultiple (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value, 2);

    *detectResistance = value & PORT_RESISTANCE_MASK_SHORT;
    *detectResistanceStatus = (TPS238x_Detect_Resistance_Status_t) (value >> RS_SHIFT_SHORT);

    return (rtn);
}

/*************************************************************************************************************************************************
*  tps_GetPortDetectVoltageDifference
**************************************************************************************************************************************************/
/*!
* @brief Returns the voltage difference for the indicated system port number on one of the TPS23861 in the system
*
* The system will attempt to determine a legacy PD by measuring the PD input capacitance on the PI. A fixed charge is injected into the PI and 
* the resulting voltage difference is reported. The reported voltage difference is only usable when the status indicates a valid measurement.
*
* @param[in]   systemPortNum             Port Number Value returned from the port registration function, tps_RegisterPort()
* @param[out]  *detectVoltageDiff        The address of a uint16_t variable where the detection resistance of the port. The 
*                                           detection resistance will be a scaled integer with an LSB of 11.0966 ohms.
* @param[out]  *detectVoltageDiffStatus  The address of a TPS238x_Detect_Voltage_Difference_Status_t variable for the most recent detect voltage
*                                            difference result status [VDS_STATUS_POWER_ON_RESET, VDS_STATUS_VALID_MEASUREMENT, VDS_STATUS_TIMEOUT,
*                                            VDS_STATUS_FIRST_MEASUREMENT_EXCESS, VDS_STATUS_SECOND_MEASUREMENT_EXCESS, or 
*                                            VDS_STATUS_INSUFFICIENT_SIGNAL]
*
* @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
*
* @sa tps_GetPortDetectResistance ()
**************************************************************************************************************************************************/
uint8_t tps_GetPortDetectVoltageDifference (uint8_t systemPortNum, uint16_t *detectVoltageDiff,
                                            TPS238x_Detect_Voltage_Difference_Status_t *detectVoltageDiffStatus)
{
    uint8_t rtn;
    uint16_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
    uint8_t command = TPS238X_PORT_1_DETECT_VOLTAGE_DIFF_COMMAND + (2 * ((uint8_t)portNum - 1));         // 2 bytes for each port

    rtn = tps_ReadI2CMultiple (tps_GetDeviceI2CAddress(systemPortNum), command, (uint8_t *)&value, 2);

    *detectVoltageDiff = value & PORT_VOLTAGE_DIFFERENCE_MASK_SHORT;
    *detectVoltageDiffStatus = (TPS238x_Detect_Voltage_Difference_Status_t) (value >> VDS_SHIFT_SHORT);

    return (rtn);
}



/*************************************************************************************************************************************************
*  tps_ReleasePort
**************************************************************************************************************************************************/
/*!
* @brief Releases a register system port number from the port mapping table
*
* This function would undo the registration performed in tps_RegisterPort().
*
* @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
*
* @return  uint8_t    TPS_SUCCESSFUL or TPS_ERR_PORT_NOT_IN_USE
*
* @sa tps_RegisterPort ()
**************************************************************************************************************************************************/
uint8_t tps_ReleasePort (uint8_t systemPortNum)
{
    if (TPS_PortMap[systemPortNum].i2cAddress == TPS_PORT_NOT_REGISTERED_VALUE)
    {
        return (TPS_ERR_PORT_NOT_IN_USE);
    }
    else
    {
        TPS_PortMap[systemPortNum].i2cAddress   = TPS_PORT_NOT_REGISTERED_VALUE;
        TPS_PortMap[systemPortNum].devicePortNum = (TPS238x_PortNum_t)TPS_PORT_NOT_REGISTERED_VALUE;
        return (TPS_SUCCESSFUL);
    }
}


uint8_t tps_ResetInterruptPin (uint8_t device_i2c_address)
{
    uint8_t rtn;
    const uint8_t value = CLINP;

    // Load the Operating Mode
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_RESET_COMMAND, value);

    return (rtn);


}


uint8_t tps_SetPortICUT(uint8_t systemPortNum,TPS238x_ICUT_Config_t icutCurrentThreshold)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    // Load the ICUT Current Threshold
    if ((uint8_t) portNum <= 2)
    {
      rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT21_CONFIGURATION_COMMAND, &value);
      ICUT12 = value;
      if ((uint8_t) portNum == 2)
      {
        value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_2_SHIFT);                      // Clear old value for this port number
        value |= ((uint8_t)icutCurrentThreshold << ICUT_PORT_2_SHIFT);             // Each port gets 3 bits
      }
      else
      {
        value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_1_SHIFT);                      // Clear old value for this port number
        value |= ((uint8_t)icutCurrentThreshold << ICUT_PORT_1_SHIFT);             // Each port gets 3 bits
      }

      rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT21_CONFIGURATION_COMMAND, value);
      rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT21_CONFIGURATION_COMMAND, &value);
      ICUT12 = value;
    }
    else
    {
      rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT43_CONFIGURATION_COMMAND, &value);
      ICUT34 = value;
      if ((uint8_t) portNum == 4)
      {
        value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_4_SHIFT);                      // Clear old value for this port number
        value |= ((uint8_t)icutCurrentThreshold << ICUT_PORT_4_SHIFT);             // Each port gets 3 bits
      }
      else
      {
        value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_3_SHIFT);                      // Clear old value for this port number
        value |= ((uint8_t)icutCurrentThreshold << ICUT_PORT_3_SHIFT);             // Each port gets 3 bits
      }
      rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT43_CONFIGURATION_COMMAND, value);
      rtn += tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT43_CONFIGURATION_COMMAND, &value);
      ICUT34 = value;
    }
    return (rtn);
}

uint8_t tps_GetPortICUT(uint8_t systemPortNum)
{
    uint8_t value;
    uint8_t rtn;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);
    
    if ((uint8_t) portNum <= 2)
    {
      rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT21_CONFIGURATION_COMMAND, &value);
      ICUT12 = value;
    }
    else
    {
      rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_ICUT43_CONFIGURATION_COMMAND, &value);
      ICUT34 = value;

    }

    return (rtn);
}

uint8_t tps_SetPortILIM(uint8_t systemPortNum, TPS238x_POE_Plus_Foldback_t poepFoldbackCurve)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    // Load the PoE Plus Setting
    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POE_PLUS_COMMAND, &value);
    if (poepFoldbackCurve == _2X_ILIM_FOLDBACK_CURVE)
        value |= (CONVERT_PORT_NUM(portNum) << POE_PLUS_SHIFT);                      // Set the PoE Plus setting, so we use the 2x Foldback Curve
    else
        value &= ~(CONVERT_PORT_NUM(portNum) << POE_PLUS_SHIFT);                     // Clear the PoE Plus setting, so we use the 1x Foldback Curve
    rtn += tps_WriteI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POE_PLUS_COMMAND, value);

    return (rtn);
}

uint8_t tps_GetPortILIM(uint8_t systemPortNum)
{
    uint8_t rtn;
    uint8_t value;

    rtn = tps_ReadI2CReg (tps_GetDeviceI2CAddress(systemPortNum), TPS238X_POE_PLUS_COMMAND, &value);
    ILIM = value;

    return(rtn);
}



uint8_t tps_SetDeviceTwoEventEnable (uint8_t device_i2c_address,TPS238x_Two_Event_t twoEvent1,TPS238x_Two_Event_t twoEvent2,
    TPS238x_Two_Event_t twoEvent3,TPS238x_Two_Event_t twoEvent4)
{
    uint8_t rtn;
    uint8_t value;

    value = (twoEvent4 << 6) | (twoEvent3 << 4) | (twoEvent2 << 2) | twoEvent1;
    rtn = tps_WriteI2CReg (device_i2c_address, TPS238X_TWO_EVENT_CLASSIFICATION_COMMAND, value);

    return (rtn);
}

/*************************************************************************************************************************************************
 *  tps_SetPortPoEP
 **************************************************************************************************************************************************/
/*!
 * @brief Set the PoEP bit and set the Icut level for the the particular port.
 *
 * This function will set the PoEP bit and Icut level based upon the expected foldback setting.
 *
 * @param[in]   systemPortNum          Port Number Value returned from the port registration function, tps_RegisterPort()
 * @param[in]   poepFoldbackCurve      The appropriate Foldback setting setting for the specified port
 * @param[in]   icutCurrentThreshold   The appropriate Icut setting for the specified port
 *
 * @return  uint8_t    (I2C_SUCCESSFUL or I2C error status)
 *
 **************************************************************************************************************************************************/
uint8_t tps_SetPortPoEP(uint8_t systemPortNum,
        TPS238x_POE_Plus_Foldback_t poepFoldbackCurve,
        TPS238x_ICUT_Config_t icutCurrentThreshold)
{
    uint8_t rtn;
    uint8_t value;
    uint8_t portNum = tps_GetDevicePortNum(systemPortNum);

    // Load the PoE Plus Setting
    rtn = tps_ReadI2CReg(tps_GetDeviceI2CAddress(systemPortNum),
            TPS238X_POE_PLUS_COMMAND, &value);

    if (poepFoldbackCurve == _2X_ILIM_FOLDBACK_CURVE)
        value |= (CONVERT_PORT_NUM(portNum) << POE_PLUS_SHIFT); // Set the PoE Plus setting, so we use the 2x Foldback Curve
    else
        value &= ~(CONVERT_PORT_NUM(portNum) << POE_PLUS_SHIFT); // Clear the PoE Plus setting, so we use the 1x Foldback Curve

    rtn += tps_WriteI2CReg(tps_GetDeviceI2CAddress(systemPortNum),
            TPS238X_POE_PLUS_COMMAND, value);

    // Load the ICUT Current Threshold
    if ((uint8_t) portNum <= 2) {
        rtn += tps_ReadI2CReg(tps_GetDeviceI2CAddress(systemPortNum),
                TPS238X_ICUT21_CONFIGURATION_COMMAND, &value);
        if ((uint8_t) portNum == 2) {
            value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_2_SHIFT);           // Clear old value for this port number
            value |= ((uint8_t) icutCurrentThreshold << ICUT_PORT_2_SHIFT); // Each port gets 3 bits
        } else {
            value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_1_SHIFT);           // Clear old value for this port number
            value |= ((uint8_t) icutCurrentThreshold << ICUT_PORT_1_SHIFT); // Each port gets 3 bits
        }

        rtn += tps_WriteI2CReg(tps_GetDeviceI2CAddress(systemPortNum),
                TPS238X_ICUT21_CONFIGURATION_COMMAND, value);
    } else {
        rtn += tps_ReadI2CReg(tps_GetDeviceI2CAddress(systemPortNum),
                TPS238X_ICUT43_CONFIGURATION_COMMAND, &value);
        if ((uint8_t) portNum == 4) {
            value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_4_SHIFT);           // Clear old value for this port number
            value |= ((uint8_t) icutCurrentThreshold << ICUT_PORT_4_SHIFT); // Each port gets 3 bits
        } else {
            value &= ~(ICUT_THRESHOLD_MASK << ICUT_PORT_3_SHIFT);           // Clear old value for this port number
            value |= ((uint8_t) icutCurrentThreshold << ICUT_PORT_3_SHIFT); // Each port gets 3 bits
        }
        rtn += tps_WriteI2CReg(tps_GetDeviceI2CAddress(systemPortNum),
                TPS238X_ICUT43_CONFIGURATION_COMMAND, value);
    }
    return (rtn);
}

/*************************************************************************************************************************************************/
/* End of PUBLIC section                                                                                                                        */
/*                                                                                                                                               */
/*! \endcond                                                                                                                                     */
/*************************************************************************************************************************************************/
void TPS23861_Init(void)
{
    u8 i;
    
    tpsReset();
    
    // Check the standard i2c address to discover the part. If not found, program the address(es)
    for (i=0; i < NUM_OF_TPS23861; i++)
    {
        rtn = tps_ReadI2CReg (i2cAddList[i], TPS238X_I2C_SLAVE_ADDRESS_COMMAND, &current_address);

        oldAutoBitSetting = (current_address & AUTO_BIT);  //save the old AUTO bit setting

        current_address &= 0x7F;

        if (autoMode[i] == TPS_ON) {
          current_address |= AUTO_BIT;
        }
        if ((current_address != (oldAutoBitSetting | i2cAddList[i])) || (rtn != I2C_SUCCESSFUL)) {
          addressChangeNeeded = TRUE;
        }
    }

    // If the address does not match standard, OR a NACK is received,
    //  reprogram the address(es) of the TPS23861's
    if(addressChangeNeeded == TRUE)
    {
        tps_SetI2CAddresses (0x14, NUM_OF_TPS23861, i2cAddList, autoMode);
    }
    
    sysPortNum1 = tps_RegisterPort (i2cAddList[0], TPS238X_PORT_1);
    sysPortNum2 = tps_RegisterPort (i2cAddList[0], TPS238X_PORT_2);
    sysPortNum3 = tps_RegisterPort (i2cAddList[0], TPS238X_PORT_3);
    sysPortNum4 = tps_RegisterPort (i2cAddList[0], TPS238X_PORT_4);

    sysPortNum5 = tps_RegisterPort (i2cAddList[1], TPS238X_PORT_1);
    sysPortNum6 = tps_RegisterPort (i2cAddList[1], TPS238X_PORT_2);
    sysPortNum7 = tps_RegisterPort (i2cAddList[1], TPS238X_PORT_3);
    sysPortNum8 = tps_RegisterPort (i2cAddList[1], TPS238X_PORT_4);

    rtn =  tps_GetDeviceInterruptStatus (i2cAddList[0], &intStatus);
    rtn =  tps_GetDeviceInterruptStatus (i2cAddList[1], &intStatus);

    // Read and Clear all Events
    rtn =  tps_GetDeviceAllInterruptEvents (i2cAddList[0], TPS_ON, &powerEnablePortEvents, &powerGoodPortEvents, &detectionPortEvents,
                                            &classificationPortEvents, &icutPortEvents, &disconnectPortEvents,
                                            &inrushPortEvents, &ilimPortEvents, &supplyEvents);

    rtn =  tps_GetDeviceAllInterruptEvents (i2cAddList[1], TPS_ON, &powerEnablePortEvents, &powerGoodPortEvents, &detectionPortEvents,
                                            &classificationPortEvents, &icutPortEvents, &disconnectPortEvents,
                                            &inrushPortEvents, &ilimPortEvents, &supplyEvents);

    intMask.CLMSK_Classificiation_Cycle_Unmask = 1;
    intMask.DEMSK_Detection_Cycle_Unmask = 1;
    intMask.DIMSK_Disconnect_Unmask = 1;
    intMask.PGMSK_Power_Good_Unmask = 1;
    intMask.PEMSK_Power_Enable_Unmask  = 1;
    intMask.IFMSK_IFAULT_Unmask = 1;
    intMask.INMSK_Inrush_Fault_Unmask = 1;

    tps_SetDeviceInterruptMask (i2cAddList[0], intMask, intDelayTime);
    tps_SetDeviceInterruptMask (i2cAddList[1], intMask, intDelayTime);

    //tps_SetDeviceOpMode (i2cAddList[0], OPERATING_MODE_MANUAL, OPERATING_MODE_MANUAL, OPERATING_MODE_MANUAL, OPERATING_MODE_MANUAL);
    //tps_SetDeviceOpMode (i2cAddList[1], OPERATING_MODE_MANUAL, OPERATING_MODE_MANUAL, OPERATING_MODE_MANUAL, OPERATING_MODE_MANUAL);
    tps_SetDeviceOpMode (i2cAddList[0], OPERATING_MODE_AUTO, OPERATING_MODE_AUTO, OPERATING_MODE_AUTO, OPERATING_MODE_AUTO);
    tps_SetDeviceOpMode (i2cAddList[1], OPERATING_MODE_AUTO, OPERATING_MODE_AUTO, OPERATING_MODE_AUTO, OPERATING_MODE_AUTO);

    //Set two event classification if a class 4 classification occurs
    tps_SetDeviceTwoEventEnable(i2cAddList[0],TWO_EVENT_AFTER_CLASS_4,TWO_EVENT_AFTER_CLASS_4,TWO_EVENT_AFTER_CLASS_4,TWO_EVENT_AFTER_CLASS_4);
    tps_SetDeviceTwoEventEnable(i2cAddList[1],TWO_EVENT_AFTER_CLASS_4,TWO_EVENT_AFTER_CLASS_4,TWO_EVENT_AFTER_CLASS_4,TWO_EVENT_AFTER_CLASS_4);

    inactivePorts[0] = TPS238X_ALL_PORTS;
    inactivePorts[1] = TPS238X_ALL_PORTS;

    // Power off all ports in case we are re-running this application without physically shutting down ports from previous run
    tps_SetDevicePowerOff(i2cAddList[0], inactivePorts[0]);
    tps_SetDevicePowerOff(i2cAddList[1], inactivePorts[1]);
    
    //tps_SetDevicePowerOn(i2cAddList[0], inactivePorts[0]);
    //tps_SetDevicePowerOn(i2cAddList[1], inactivePorts[1]);
    //tps_SetPortPower(sysPortNum1,TPS_ON);
    //tps_SetPortPower(sysPortNum8,TPS_ON);
    //tps_SetPortPower(sysPortNum1,TPS_OFF);
    //tps_SetPortPower(sysPortNum8,TPS_OFF);

    // Set up all ports for Disconnect Enable at 7.5 mA
    tps_SetDeviceDisconnectEnable (i2cAddList[0], inactivePorts[0], DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP);
    tps_SetDeviceDisconnectEnable (i2cAddList[1], inactivePorts[1], DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP);
    
    printf ("\r POE 23861 - Hit 'S' to start");
    printf ("Welcome to the POE 23861 - Auto Mode\r\n");
}
    
void TPS23861_Run(void)
{
    IntFlag = 1;    // Set IntFlag to clear SUPF Fault at POR

    //while (1)
    //{

        if (!IntFlag)
        {
            //__enable_interrupt();
            //LPM0;
            //__no_operation();
        }
        else
        {
            IntFlag = 0;

            for (devNum = 0; devNum < NUM_OF_TPS23861; devNum++)
            {
                // Get the interrupt and interrupt statuses
                rtn =  tps_GetDeviceInterruptStatus (i2cAddList[devNum], &intStatus);
                rtn =  tps_GetDeviceAllInterruptEvents (i2cAddList[devNum], TPS_ON, &powerEnablePortEvents, &powerGoodPortEvents, &detectionPortEvents,
                                    &classificationPortEvents, &icutPortEvents, &disconnectPortEvents,
                                    &inrushPortEvents, &ilimPortEvents, &supplyEvents);

                // Did we have a new Detection Event?
                if (intStatus.DETC_Detection_Cycle) {
                  uint8_t target = detectionPortEvents;
                  uint8_t i;

                  for (i=TPS238X_PORT_1; i<=TPS238X_PORT_4; i++)
                  {
                    if (target & 0x1) {
                      sysPortNum = tps_GetSystemPortNumber (i2cAddList[devNum], (TPS238x_PortNum_t)i);

                      rtn = tps_GetPortDetectClassStatus (sysPortNum, &detectStatus, &classStatus);
                      if (detectStatus == DETECT_RESIST_VALID) { // Valid
                        // Set the valid port as inactive, so it will stop doing Detections
                        inactivePorts[devNum] &= ~(CONVERT_PORT_NUM(i));

                        // Start a classification for this port
                        tps_SetPortDetectClassEnable (sysPortNum, TPS_OFF, TPS_ON);
                        
                        printf ("\nDetection Event Port %X\r", (i+(devNum * 4)));
                        printf (" - Detect Status: %X\r\n", detectStatus);
                      }
                    }
                    target >>= 1;
                  }
                }

                // Did we have a new Classification Event?
                if (intStatus.CLASC_Classification_Cycle)
                {
                  uint8_t target = classificationPortEvents;
                  uint8_t i;

                  for (i=TPS238X_PORT_1; i<=TPS238X_PORT_4; i++)
                  {
                    if (target & 0x1)
                    {
                      sysPortNum = tps_GetSystemPortNumber (i2cAddList[devNum], (TPS238x_PortNum_t)i);

                      rtn = tps_GetPortDetectClassStatus (sysPortNum, &detectStatus, &classStatus);
                      
                      printf ("\nClassification Event Port %X",(i + (devNum * 4)));
                      printf (" - Classification Status: %X\r\n",classStatus);
                      
                      if ((classStatus != CLASS_OVERCURRENT) &&
                        (classStatus != CLASS_UNKNOWN) &&
                        (classStatus != CLASS_MISMATCH))
                      {
                        // Power ON!!
                        rtn = tps_SetPortPower (sysPortNum, TPS_ON);
                        printf ("Turn on Power - Port %X\r\n",(i + (devNum * 4)));
                        
                        failClassOnce[i-1] = 0;              // Subtract 1 as this register is 0 index based, but Port Numbers are 1 index based

                        if(classStatus == CLASS_4)
                        {
                          tps_SetPortPoEP(sysPortNum, _2X_ILIM_FOLDBACK_CURVE, ICUT_686_MILLIAMP);
                        }
                        else
                        {
                          tps_SetPortPoEP(sysPortNum, _1X_ILIM_FOLDBACK_CURVE, ICUT_374_MILLIAMP);
                        }
                      }
                      else
                      {
                        if (failClassOnce[i-1] == 0)        // Subtract 1 as this register is 0 index based, but Port Numbers are 1 index based
                        {
                          // Restart Classification
                          tps_SetPortDetectClassEnable (sysPortNum, TPS_OFF, TPS_ON);
                          failClassOnce[i-1] = 1;         // Subtract 1 as this register is 0 index based, but Port Numbers are 1 index based
                        }
                        else
                        {
                          // Set the valid port as inactive, so it will attempt another Detection sequence
                          inactivePorts[devNum] |=  (CONVERT_PORT_NUM(i));

                          failClassOnce[i-1] = 0;         // Subtract 1 as this register is 0 index based, but Port Numbers are 1 index based

                        }
                      }
                    }
                    target >>= 1;
                  }
                }

                // Did we have a new Disconnection Event?
                if (intStatus.DISF_Disconnect_Event)
                {
                  uint8_t target = disconnectPortEvents;
                  uint8_t i;

                  // Set all of the disconnected ports as inactive
                  *(uint8_t *)&inactivePorts[devNum] |= target;

                  PrintPower = 1;

                  for (i=TPS238X_PORT_1; i<=TPS238X_PORT_4; i++)
                  {
                    if (target & 0x1)
                    {
                      printf ("\nDisconnection Event Port %X\r\n",(i + (devNum * 4)));
                    }
                    target >>= 1;
                  }
                }

                // Did we have a new Power Enable Event?
                if (intStatus.PEC_Power_Enable_Change)
                {
                  uint8_t target = powerEnablePortEvents;
                  uint8_t i;

                  for (i=TPS238X_PORT_1; i<=TPS238X_PORT_4; i++)
                  {
                    if (target & 0x1)
                    {
                      sysPortNum = tps_GetSystemPortNumber (i2cAddList[devNum], (TPS238x_PortNum_t)i);

                      powerEnable = tps_GetPortPowerEnableStatus (sysPortNum);

                      if (powerEnable == TPS_ON)          // Valid
                      {
                        printf ("Power Enable Event Port %X\r\n",(i + (devNum * 4)));
                        PrintPower = 1;
                      }
                      else
                      {
                        printf ("Power Disable Event Port %X\r\n",(i + (devNum * 4)));
                      }
                    }
                    target >>= 1;
                  }
                }

                // Did we have a new Power Good Event?
                if (intStatus.PGC_Power_Good_Change)
                {
                  uint8_t target = powerGoodPortEvents;
                  uint8_t i;

                  for (i=TPS238X_PORT_1; i<=TPS238X_PORT_4; i++)
                  {
                    if (target & 0x1)
                    {
                      sysPortNum = tps_GetSystemPortNumber (i2cAddList[devNum], (TPS238x_PortNum_t)i);

                      powerGood = tps_GetPortPowerGoodStatus (sysPortNum);

                      if (powerGood == TPS_ON)          // Valid
                      {
                        printf ("Power Good Event Port %X\r\n",(i + (devNum * 4)));
                        PrintPower = 1;
                      }
                      else
                      {
                        printf ("Power No Longer Good Event Port %X\r\n",(i + (devNum * 4)));
                      }
                    }
                    target >>= 1;
                  }
                }

                //Fault conditions
                if(intStatus.SUPF_Supply_Event_Fault)
                {
                  uint8_t *temp = (uint8_t *)&supplyEvents;
                  uint8_t target = *temp;

                  if(target & 0x10)
                  {
                    printf("VPWR undervlotage occurred\r\n");
                  }
                  else if (target & 0x20)
                  {
                    printf("VDD undervlotage occurred\r\n");
                  }

                  else if (target & 0x80)
                  {
                    printf("Thermal shutdown occurred\r\n");
                  }
                }

                if(intStatus.INRF_Inrush_Fault)
                {
                  uint8_t target = inrushPortEvents;
                  uint8_t i;

                  // Set all of the start fault ports as inactive
                  *(uint8_t *)&inactivePorts[devNum] |= target;

                  for (i = TPS238X_PORT_1; i <= TPS238X_PORT_4; i++)
                  {
                    if (target & 0x1)
                    {
                      printf ("Start Fault Port %X\r\n",(i + (4 * devNum)));
                    }
                    target >>= 1;
                  }
                }

                if(intStatus.IFAULT_ICUT_ILIM_Fault)
                {
                    uint8_t targetIcut = icutPortEvents;
                    uint8_t targetIlim = ilimPortEvents;
                    uint8_t i;

                    // Set all of the ICUT fault fault ports as inactive
                    *(uint8_t *)&inactivePorts[devNum] |= icutPortEvents;

                    // Set all of the ICUT fault fault ports as inactive
                    *(uint8_t *)&inactivePorts[devNum] |= ilimPortEvents;

                    for (i = TPS238X_PORT_1; i <= TPS238X_PORT_4; i++)
                    {
                        if (targetIcut & 0x01)
                        {
                          printf("ICUT Fault Port %X\r\n",(i + (4 * devNum)));
                        }

                        if(targetIlim & 0x01)
                        {
                          printf("ILIM Fault Port %X\r\n",(i + (4 * devNum)));
                        }

                        targetIcut >>= 1;
                        targetIlim >>= 1;
                    }
                }
            }
        }  // if (IntFlag)

        PrintPower = 1;
        if (PrintPower)
        {
            PrintPower = 0;

            tps_GetDeviceInputVoltage (i2cAddList[0], &voltage);
            outNum = ((uint32_t)voltage * 3662) / 1000;
            printf ("Input Voltage: %ld mV \r\n",outNum);

            tps_GetDeviceTemperature (i2cAddList[0], &temperature);
            outNum = CONVERT_TEMP((uint32_t)temperature);
            printf ("Device Temperature: %ld degrees C\r\n",outNum);

            if (!(inactivePorts[0] & PORT_1_VALUE))
            {
                tps_GetPortMeasurements (sysPortNum1, &voltage, &current);
                outNum = ((uint32_t)voltage * 3662) / 1000;
                printf ("Port 1 Voltage: %ldmV \r\n",outNum);
                outNum = ((uint32_t)current * 62260) / 1000000;
                printf ("Port 1 Current: %ldmA \r\n",outNum);
            }

            if (!(inactivePorts[0] & PORT_2_VALUE))
            {
                tps_GetPortMeasurements (sysPortNum2, &voltage, &current);
                outNum = ((uint32_t)voltage * 3662) / 1000;
                printf ("Port 2 Voltage: %ldmV \r\n",outNum);
                outNum = ((uint32_t)current * 62260) / 1000000;
                printf ("Port 2 Current: %ldmA \r\n",outNum);
            }

            if (!(inactivePorts[0] & PORT_3_VALUE))
            {
                tps_GetPortMeasurements (sysPortNum3, &voltage, &current);
                outNum = ((uint32_t)voltage * 3662) / 1000;
                printf ("Port 3 Voltage: %ldmV \r\n",outNum);
                outNum = ((uint32_t)current * 62260) / 1000000;
                printf ("Port 3 Current: %ldmA \r\n",outNum);
            }

            if (!(inactivePorts[0] & PORT_4_VALUE))
            {
                tps_GetPortMeasurements (sysPortNum4, &voltage, &current);
                outNum = ((uint32_t)voltage * 3662) / 1000;
                printf ("Port 4 Voltage: %ldmV \r\n",outNum);
                outNum = ((uint32_t)current * 62260) / 1000000;
                printf ("Port 4 Current: %ldmA \r\n",outNum);
            }

            if (!(inactivePorts[1] & PORT_1_VALUE))
            {
                tps_GetPortMeasurements (sysPortNum5, &voltage, &current);
                outNum = ((uint32_t)voltage * 3662) / 1000;
                printf ("Port 5 Voltage: %ldmV \r\n",outNum);
                outNum = ((uint32_t)current * 62260) / 1000000;
                printf ("Port 5 Current: %ldmA \r\n",outNum);
            }

            if (!(inactivePorts[1] & PORT_2_VALUE))
            {
                tps_GetPortMeasurements (sysPortNum6, &voltage, &current);
                outNum = ((uint32_t)voltage * 3662) / 1000;
                printf ("Port 6 Voltage: %ldmV \r\n",outNum);
                outNum = ((uint32_t)current * 62260) / 1000000;
                printf ("Port 6 Current: %ldmA \r\n",outNum);
            }

            if (!(inactivePorts[1] & PORT_3_VALUE))
            {
                tps_GetPortMeasurements (sysPortNum7, &voltage, &current);
                outNum = ((uint32_t)voltage * 3662) / 1000;
                printf ("Port 7 Voltage: %ldmV \r\n",outNum);
                outNum = ((uint32_t)current * 62260) / 1000000;
                printf ("Port 7 Current: %ldmA \r\n",outNum);
            }

            if (!(inactivePorts[1] & PORT_4_VALUE))
            {
                tps_GetPortMeasurements (sysPortNum8, &voltage, &current);
                outNum = ((uint32_t)voltage * 3662) / 1000;
                printf ("Port 8 Voltage: %ldmV \r\n",outNum);
                outNum = ((uint32_t)current * 62260) / 1000000;
                printf ("Port 8 Current: %ldmA \r\n",outNum);
            }

//#ifdef DETAILED_STATUS
#if 0
            for (devNum=0; devNum < NUM_OF_TPS23861; devNum++)
            {
                // read current value of all event registers (Do not clear)
                rtn =  tps_GetDeviceAllInterruptEvents (i2cAddList[devNum], TPS_OFF, &powerEnablePortEvents, &powerGoodPortEvents, &detectionPortEvents,
                                    &classificationPortEvents, &icutPortEvents, &disconnectPortEvents,
                                    &inrushPortEvents, &ilimPortEvents, &supplyEvents);

                uart_puts ("\n---- Event Registers -----Dev : ");
                uartPutHex (devNum);
                uart_puts ("----- \r\n0x");
                uartPutHex ((powerGoodPortEvents<<4) | powerEnablePortEvents);
                uart_puts ("   0x");
                uartPutHex ((classificationPortEvents << 4) | detectionPortEvents);
                uart_puts ("   0x");
                uartPutHex ((disconnectPortEvents << 4) | icutPortEvents);
                uart_puts ("   0x");
                uartPutHex ((ilimPortEvents << 4) | inrushPortEvents);
                uart_puts ("   0x");
                uartPutHex ((*(unsigned char*)&supplyEvents << 4));
                uart_puts ("\r\n\n");

                uart_puts ("---- Port Status -----\r\n0x");
                sysPortNum = tps_GetSystemPortNumber (i2cAddList[devNum], TPS238X_PORT_1);
                rtn = tps_GetPortDetectClassStatus (sysPortNum, &detectStatus, &classStatus);
                uartPutHex ((classStatus<<4) | detectStatus);
                uart_puts ("   0x");
                sysPortNum = tps_GetSystemPortNumber (i2cAddList[devNum], TPS238X_PORT_2);
                rtn = tps_GetPortDetectClassStatus (sysPortNum, &detectStatus, &classStatus);
                uartPutHex ((classStatus<<4) | detectStatus);
                uart_puts ("   0x");
                sysPortNum = tps_GetSystemPortNumber (i2cAddList[devNum], TPS238X_PORT_3);
                rtn = tps_GetPortDetectClassStatus (sysPortNum, &detectStatus, &classStatus);
                uartPutHex ((classStatus<<4) | detectStatus);
                uart_puts ("   0x");
                sysPortNum = tps_GetSystemPortNumber (i2cAddList[devNum], TPS238X_PORT_4);
                rtn = tps_GetPortDetectClassStatus (sysPortNum, &detectStatus, &classStatus);
                uartPutHex ((classStatus<<4) | detectStatus);
                uart_puts ("\r\n\n");
                uart_puts ("---- Power Status -----\r\n0x");
                rtn = tps_GetDevicePowerStatus (i2cAddList[devNum], &powerEnablePorts, &powerGoodPorts);
                uartPutHex ((powerGoodPorts<<4) | powerEnablePorts);
                uart_puts ("\r\n\n");
            }
#endif
            //uart_puts ("************************************************\r\n");
        }

        if (PerformDetection)
        {
            PerformDetection = 0;
            rtn =  tps_SetDeviceDetectClassEnable (i2cAddList[0], inactivePorts[0], 0);   // Start Detection for all ports
            rtn |= tps_SetDeviceDetectClassEnable (i2cAddList[1], inactivePorts[1], 0);   // Start Detection for all ports
            if (rtn != I2C_SUCCESSFUL)
            {
                //uart_puts ("I2C Issue\r\n");
            }

        }

    //}
}

void TPS23861_ModeChange(u8 Mode)
{
    if (Mode == 0x00) {
        TPS23861_Init();
        PerformDetection = 1;
        //tps_SetDeviceOpMode (i2cAddList[0], OPERATING_MODE_AUTO, OPERATING_MODE_AUTO, OPERATING_MODE_AUTO, OPERATING_MODE_AUTO);
        //tps_SetDeviceOpMode (i2cAddList[1], OPERATING_MODE_AUTO, OPERATING_MODE_AUTO, OPERATING_MODE_AUTO, OPERATING_MODE_AUTO);
        //tps_SetDeviceDisconnectEnable (i2cAddList[0], inactivePorts[0], DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP);
        //tps_SetDeviceDisconnectEnable (i2cAddList[1], inactivePorts[1], DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP, DCTH_7_5_MILLIAMP);
        //inactivePorts[0] = TPS238X_ALL_PORTS;
        //inactivePorts[1] = TPS238X_ALL_PORTS;
        //tps_SetDevicePowerOn(i2cAddList[0], inactivePorts[0]);
        //tps_SetDevicePowerOn(i2cAddList[1], inactivePorts[1]);
    } else {
        TPS23861_RST = 0;
        
        printf("TPS23861 Power Off\r\n");
        ///*
        inactivePorts[0] = TPS238X_ALL_PORTS;
        inactivePorts[1] = TPS238X_ALL_PORTS;
        // Power off all ports in case we are re-running this application without physically shutting down ports from previous run
        tps_SetDevicePowerOff(i2cAddList[0], inactivePorts[0]);
        tps_SetDevicePowerOff(i2cAddList[1], inactivePorts[1]);
        
        tps_SetDeviceOpMode (i2cAddList[0], OPERATING_MODE_OFF, OPERATING_MODE_OFF, OPERATING_MODE_OFF, OPERATING_MODE_OFF);
        tps_SetDeviceOpMode (i2cAddList[1], OPERATING_MODE_OFF, OPERATING_MODE_OFF, OPERATING_MODE_OFF, OPERATING_MODE_OFF);
        //*/
    }
}

#endif

