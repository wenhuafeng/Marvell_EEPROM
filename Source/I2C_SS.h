

#ifndef _I2C_SS_H_
#define _I2C_SS_H_


//#define SCL          P13
//#define SDA          P14


#define _SDA_PORT_  P1
#define _SDA_PIN_    (1<<4)
#define _SCL_PORT_  P1
#define _SCL_PIN_    (1<<3)




void Init_I2C(void);
void ACK(void);
void delay_24C02(void);


#endif

