

#ifndef  OS_MASTER_FILE
#define  OS_GLOBALS
#include "includes.h"
#endif

#if _SOFTWARE_I2C_SLAVE_

#define _SEND_DATA_NUMBER_    48

data u8 gg,ii,tt;
data u8 DEVICE_ADR;
data u8 send_ptr;
data u8 WORD_ADR;
data u8 START_flag;
//u8 WRITE_FLAG;
//u8 READ_FLAG;
data u8 temp;
BOOLEAN PreState,NowState;

//u8 WRITE_BUF[100];
//u8 READ_BUF[10];

data u16 i16,DataLength;

void Init_I2C(void)
{
    P17_Input_Mode;
    P30_Input_Mode;
    //P17_Quasi_Mode;
    //P30_Quasi_Mode;
    IE |= 0x01;
}

void ACK(void)
{
    //while(!SCL);
    //P30_PushPull_Mode;
    //SDA = 0;
    temp = 100;
    do
    {
      temp--;
    }while(!SCL && temp);
    SDA=0;//?9?CLK??????,SDA??0
    //while(SCL);
    temp = 100;
    do
    {
      temp--;
    }while(SCL && temp);
    //SDA=1;//?9?CLK??????,SDA??1
    //P30_Input_Mode;
}

//void delay_24C02(void)
//{
//    _nop_();
//}

void IIC() interrupt 0
{
    EX0=0;
    if (SCL) {
      //while(SCL);
      temp = 100;
      do
      {
        temp--;
      }while(SCL && temp);
      for (gg=8;gg>0;gg--)
      {
        //while(!SCL);
        temp = 100;
        do
        {
          temp--;
        }while(!SCL && temp);
        DEVICE_ADR<<=1;
        if (SDA) {
            DEVICE_ADR|=0x01;
        }
        //while(SCL);
        temp = 100;
        do
        {
          temp--;
        }while(SCL && temp);
      }
      P30_Quasi_Mode;
      SDA = 0;
      ACK();
      //SDA=1;
      P30_Input_Mode;
      
      if ((DEVICE_ADR&0xfe) == 0xa0) {
        
        tt = 2;
        do
        {
          for (gg=8;gg>0;gg--)
          {
            //while(!SCL);
            temp = 100;
            do
            {
              temp--;
            }while(!SCL && temp);
            WORD_ADR<<=1;
            if (SDA) {
              WORD_ADR|=0x01;
            }
            //while(SCL);
            temp = 100;
            do
            {
              temp--;
            }while(SCL && temp);
          }
          P30_Quasi_Mode;
          SDA = 0;
          ACK();
          //SDA=1;
          P30_Input_Mode;
          tt--;
        } while (tt);
        
        //while(!SCL);
        temp = 100;
        do
        {
          temp--;
        }while(!SCL && temp);
        
        //Get start signel
        temp=100;
        PreState=SDA;
        while (SCL&&temp--) {
          NowState=SDA;
          if (PreState && !NowState) {
              START_flag=1;//start
          }
        }
        
        for(gg=8;gg>0;gg--)
        {
          DEVICE_ADR<<=1;
          //while(!SCL);
          temp = 100;
          do
          {
            temp--;
          }while(!SCL && temp);
          if (SDA) {
            DEVICE_ADR|=0x01;
          }
          //while(SCL);
          temp = 100;
          do
          {
            temp--;
          }while(SCL && temp);
        }
        P30_Quasi_Mode;
        SDA = 0;
        ACK();
        //SDA=1;
        //P30_Input_Mode;
        if (DEVICE_ADR==0xa1) {
#if (_IP1717B_)
          for(ii=0;ii<_SEND_DATA_NUMBER_;ii++)
          {
            switch (ModeCtr)
            {
              case _MODE_0_:
                send_ptr = IP1717BAN[ii];
                break;
              case _MODE_1_:
                send_ptr = IP1717B10Full[ii];
                break;
              case _MODE_2_:
                send_ptr = IP1717BFull10M[ii];
                break;
              case _MODE_3_:
                send_ptr = IP1717BFull100M[ii];
                break;
              default:
                break;
            }
#elif (_98DX316_)
//          switch (ModeCtr1)
//          {
//            case _MODE_0_:
//              DataLength = 4818;//regular
//              break;
//            case _MODE_1_:
//              DataLength = 5274;//vlan
//              break;
//            default:
//              break;
//          }
          if (ModeCtr1 == _MODE_0_) {
              DataLength = 4818;//regular
          } else {
              DataLength = 5274+18;//vlan
          }
          for(i16=0; i16<DataLength; i16++)
          {
            switch (ModeCtr1)
            {
              case _MODE_0_:
                send_ptr = REGULAR_TABLE[i16];
                break;
              case _MODE_1_:
//                if (i16 < 4800) {
//                    send_ptr = REGULAR_TABLE[i16];
//                } else {
//                    send_ptr = VLAN_TABLE[i16-4800];
//                }
                
                if (i16 == 4800) i16 = 4818;
                send_ptr = REGULAR_TABLE[i16];
                break;
              default:
                break;
            }
#endif
            for(gg=8; gg>0; gg--)
            {
              if (send_ptr&0x80) {
                SDA=1;
              }else{
                SDA=0;
              }
              //while(!SCL);
              temp = 100;
              do
              {
                temp--;
              }while(!SCL && temp);
              //while(SCL);
              temp = 100;
              do
              {
                temp--;
              }while(SCL && temp);
              send_ptr<<=1;
            }
            //P30_Quasi_Mode;
            //SDA = 0;
            ACK();
            //SDA=1;
            //P30_Input_Mode;
          }
          //EX0=1;
        } else {
          //EX0=1;
        }
      } else {
        //EX0=1;
      }
    } else {
      //EX0=1;
    }
    
    P30_Input_Mode;
    EX0=1;
}

#endif

