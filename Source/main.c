/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2017 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : Apr/21/2017
//***********************************************************************************************************
#include "includes.h"


u8 SOFTWARE_VERSION[3] = {1,0,2};

void ModeLED(void);
extern UINT8 read_APROM_BYTE(UINT16 code *u16_addr);
extern void write_DATAFLASH_BYTE(UINT16 u16_addr,UINT8 u8_data);

/*******************************************************************************
 * FUNCTION_PURPOSE: Main function 
 ******************************************************************************/
void main (void)
{
    u8 Ctr = 4;
    u8 Ctr1;
    
    Set_All_GPIO_Quasi_Mode;
#if (_IP1717B_)
    P13_Input_Mode;
    P14_Input_Mode;
#elif (_98DX316_)
    P03_PushPull_Mode;
    P05_PushPull_Mode;
    P13_PushPull_Mode;
    P14_Input_Mode;
#endif
    Init_I2C();
    InitialUART0_Timer3(115200);
    
    //Timer2 init
    TIMER2_DIV_256;
    TIMER2_Auto_Reload_Delay_Mode;
    RCMP2L = TIMER_DIV256_VALUE_500ms;
    RCMP2H = TIMER_DIV256_VALUE_500ms>>8;
    TL2 = 0;
    TH2 = 0;
    set_ET2;                                    // Enable Timer2 interrupt
    set_EA;
    set_TR2;                                    // Timer2 run
    
#if _FLASH_EEPROM_
    //read eeprom, get mode.
    WriteFlag = read_APROM_BYTE(0x4781);
    if (WriteFlag == 0xA8) {
        ModeCtr1 = read_APROM_BYTE(0x4782);
        ModeCtr2 = read_APROM_BYTE(0x4783);
        if (ModeCtr1 > _MODE_1_) {
            ModeCtr1 = _MODE_0_;
        }
        if (ModeCtr2 > _MODE_1_) {
            ModeCtr2 = _MODE_0_;
        }
        ModeLED();
    }
#endif
    
    //TEST
    //TEST
    
#if _TPS23861_
    TPS23861_Init();
#endif
    
    //Reset 98DX3136
    RST_98DX3136 = 0;
    Timer1_Delay10ms(50);
    RST_98DX3136 = 1;
    
    do
    {
        if (F_Timer) {
            F_Timer = 0;
            HoldKeyCtr++;
            //Send_Data_To_UART0(0x7E);
            //printf("\nTEST\n");
            Ctr1++;
            if (Ctr1 > 1) {
                Ctr1 = 0x00;
                #if _TPS23861_
                PerformDetection = 1;
                #endif
            }
            Ctr++;
            if (Ctr > 4) {
                Ctr = 0x00;
                if (TPS23861_WorkMode == _MODE_AUTO_) {
                    #if _TPS23861_
                    TPS23861_Run();
                    #endif
                }
            }
        }
        
        ScanKey();
        if (F_PushKey) {
            if ((Key < _MAX_KEY_) && (Key != _NO_KEY_)) {
                PushKeyFunc();
            }
        } else {
            if ((OldKey < _MAX_KEY_) && (OldKey != _NO_KEY_)) {
                ReleKeyFunc();
            }
        }
    } while (1);
}
  
void ScanKey(void)
{
    u8 i,j;
    u8 l;
  
    l = KEY_PORT_2;
    l = (l & _ALL_KEY_MASK_2_);
    i = KEY_PORT_1;
    i = (i & _ALL_KEY_MASK_1_);
    i = i|l;
  
    if (i == _ALL_KEY_MASK_) {
        F_PushKey = 0;
        F_NewKey  = 0;
        F_HoldKey  = 0;
        F_TwoKey  = 0;
        goto normal_quit_scan_key;
    }
    
    Timer1_Delay10ms(1);
    
    l = KEY_PORT_2;
    l = (l & _ALL_KEY_MASK_2_);
    j = KEY_PORT_1;
    j = (j & _ALL_KEY_MASK_1_);
    j = j|l;
    
    if (j == _ALL_KEY_MASK_) {
        F_PushKey = 0;
        F_NewKey  = 0;
        F_HoldKey  = 0;
        F_TwoKey  = 0;
        goto normal_quit_scan_key;
    }
    
    if (i == j) {
        if ((i & (~_ALL_KEY_MASK_1_)) == 0x00) {
            //push key1
            Key = _KEY_1_;
        }
        else if ((i & (~_ALL_KEY_MASK_2_)) == 0x00) {
            //push key2
            Key = _KEY_2_;
        }
        else {
            //release key
            Key = _NO_KEY_;
        }
    } else {
        //error
        Key = _NO_KEY_;
    }
    
    if (Key == _NO_KEY_) {
        //Key  = _NO_KEY_;
        //F_PushKey = 0;
        F_NewKey  = 0;
        F_HoldKey  = 0;
        F_TwoKey  = 0;
        goto normal_quit_scan_key;
    } else {
        if (F_PushKey == 0) {
            OldKey = Key;
            F_NewKey = 1;
            F_PushKey = 1;
            //TEST
            //P10 = 1;
        }
    }
    
normal_quit_scan_key:
    nop;
}

/********************************************************************//**
 * @brief:      KEY HOLD COM Function subprogram
 *              
 *
 * @param[in]:  NONE
 *               
 * @return:     NONE
 *********************************************************************/
BOOLEAN HoldKeyCom(void)
{
  if (F_NewKey == 1) {
    HoldKeyCtr = 0;
  } else {
    if (F_HoldKey == 0) {
      if (HoldKeyCtr == _HOLD_TIMER_KEY_) {
        F_HoldKey = 1;
        return 1;
      }
    }
  }
  
  return 0;
}


/********************************************************************//**
 * @brief:      KEY SETTING COM Function subprogram
 *              
 *
 * @param[in]:  NONE
 *               
 * @return:     NONE
 *********************************************************************/
//BOOLEAN SettingCom(void)
//{
//  if (F_NewKey == 1) {
//    HoldKeyCtr = 0;
//    //KEY_TONE();
//    return 1;
//  } else {
//    if (F_HoldKey == 0) {
//      if (HoldKeyCtr == _HOLD_TIMER_KEY_) {
//        F_HoldKey = 1;
//        //Timer2_init(_T2_125MS_INT_);
//        return 1;
//      }
//    } else {
//      return 1;
//    }
//  }
//  
//  return 0;
//}

void ModeLED(void)
{
    if ((ModeCtr1 == _MODE_0_) && (ModeCtr2 == _MODE_0_)) {
        ModeCtr = _MODE_0_;
        LED1 = _LED_OFF_;
        LED2 = _LED_OFF_;
    }
    else if ((ModeCtr1 == _MODE_0_) && (ModeCtr2 == _MODE_1_)) {
        ModeCtr = _MODE_1_;
        LED1 = _LED_OFF_;
        LED2 = _LED_ON_;
    }
    else if ((ModeCtr1 == _MODE_1_) && (ModeCtr2 == _MODE_0_)) {
        ModeCtr = _MODE_2_;
        LED1 = _LED_ON_;
        LED2 = _LED_OFF_;
    }
    else if ((ModeCtr1 == _MODE_1_) && (ModeCtr2 == _MODE_1_)) {
        ModeCtr = _MODE_3_;
        LED1 = _LED_ON_;
        LED2 = _LED_ON_;
    }
    
#if _FLASH_EEPROM_
    write_DATAFLASH_BYTE (0x4781,0xA8);
    write_DATAFLASH_BYTE (0x4782,ModeCtr1);
    write_DATAFLASH_BYTE (0x4783,ModeCtr2);
#endif
}

/********************************************************************//**
 * @brief:      push key Function subprogram
 *              
 *
 * @param[in]:  NONE
 *               
 * @return:     NONE
 *********************************************************************/
void PushKeyFunc(void)
{
    switch (Key)
    {
        case _KEY_1_:
          if (HoldKeyCom()) {
              OldKey = 0x00;
              #if _TPS23861_
              TPS23861_WorkMode = ~TPS23861_WorkMode;
              TPS23861_ModeChange(TPS23861_WorkMode);
              
              LED1 = _LED_ON_;
              DelayMs(200);
              LED1 = _LED_OFF_;
              DelayMs(200);
              LED1 = _LED_ON_;
              DelayMs(200);
              LED1 = _LED_OFF_;
              DelayMs(200);
              ModeLED();
              #endif
          }
          break;
        case _KEY_2_:
          if (HoldKeyCom()) {
              //
              OldKey = 0x00;
          }
          break;
        default:
          break;
    }
    
    F_NewKey = 0;
}

/********************************************************************//**
 * @brief:      rele key Function subprogram
 *              
 * @param[in]:  NONE
 *               
 * @return:     NONE
 *********************************************************************/
void ReleKeyFunc(void)
{
    INT8U tmp;
  
    tmp = OldKey;
    OldKey = 0x00;
    
    switch (tmp)
    {
        case _KEY_1_:
          //if (SettingCom()) {
          //    if (F_NewKey) {
                  RST_98DX3136 = 0;
                  Timer1_Delay10ms(50);
                  RST_98DX3136 = 1;
                
                  ModeCtr1++;
                  if (ModeCtr1 > _MODE_1_) {
                      ModeCtr1 = _MODE_0_;
                  }
                  ModeLED();
          //    }
          //}
          break;
        case _KEY_2_:
          //if (SettingCom()) {
          //    if (F_NewKey) {
#if (_IP1717B_)
              RST_98DX3136 = 0;
              Timer1_Delay10ms(1);
              RST_98DX3136 = 1;
              
              ModeCtr2++;
              if (ModeCtr2 > _MODE_1_) {
                  ModeCtr2 = _MODE_0_;
              }
              ModeLED();
#elif (_98DX316_)
              
#endif
          //    }
          //}
          break;
        default:
          break;
    }
    
    
}

/********************************************************************//**
 * @brief:      Timer 2 Init Function subprogram
 *              
 * @param[in]:  NONE
 *               
 * @return:     NONE
 *********************************************************************/
void Timer2_ISR (void) interrupt 5
{
    clr_TF2;                                //Clear Timer2 Interrupt Flag
    P12 = ~P12;                              // GPIO1 toggle
    F_Timer = 1;
}

