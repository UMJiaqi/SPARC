#include "AS5600_IIC.h"

#define Y_SCK_H(SCK) {digitalWrite(SCK, 1);}
#define Y_SCK_L(SCK) {digitalWrite(SCK, 0);}
#define Y_SDA_H(SDA) {pinMode(SDA,OUTPUT); digitalWrite(SDA, 1);}
#define Y_SDA_L(SDA) {pinMode(SDA,OUTPUT); digitalWrite(SDA, 0);}
#define Y_READ_SDA(SDA) digitalRead(SDA)

#define AS5600_DEFAULT_ADDRESS 0x36

void Y_IIC_Start(const int SCK, const int SDA) {
    Y_SDA_H(SDA);
    Y_SCK_H(SCK);
    delayMicroseconds(5);
    Y_SDA_L(SDA);
    delayMicroseconds(5);
    Y_SCK_L(SCK);
    delayMicroseconds(2);
}

void Y_IIC_Stop(const int SCK, const int SDA) {
    Y_SCK_L(SCK);
    Y_SDA_L(SDA);
    delayMicroseconds(5);
    Y_SCK_H(SCK);
    delayMicroseconds(5);
    Y_SDA_H(SDA);
    delayMicroseconds(5);
}

u8 Y_IIC_Wait_Ack(const int SCK, const int SDA) {
    u8 ucErrTime = 0;
    Y_SDA_H(SDA);
    delayMicroseconds(2);
    Y_SCK_H(SCK);
    delayMicroseconds(2);

    pinMode(SDA, INPUT); 
    while(Y_READ_SDA(SDA)) {
        ucErrTime++;
        if(ucErrTime > 250) {
        Y_IIC_Stop(SCK, SDA);
        return 1;
        }
    }

    Y_SCK_L(SCK);             
    delayMicroseconds(2);  
    return 0;  
}

void Y_IIC_Ack(const int SCK, const int SDA) {
    Y_SCK_L(SCK);
    Y_SDA_L(SDA);
    delayMicroseconds(2);
    Y_SCK_H(SCK);
    delayMicroseconds(5);
    Y_SCK_L(SCK);
    delayMicroseconds(2);
    Y_SDA_H(SDA);
    delayMicroseconds(2);
}

void Y_IIC_Nack(const int SCK, const int SDA) {
    Y_SCK_L(SCK);
    Y_SDA_H(SDA);
    delayMicroseconds(2);
    Y_SCK_H(SCK);
    delayMicroseconds(5);
    Y_SCK_L(SCK);
    delayMicroseconds(2);
    Y_SDA_H(SDA);
    delayMicroseconds(2);
}

void Y_IIC_SendByte(u8 Txdata, const int SCK, const int SDA) {
    for (u8 i = 0; i < 8; ++i) {
        Y_SCK_L(SCK);
        if (Txdata & 0x80) {
        Y_SDA_H(SDA)
        } else {
        Y_SDA_L(SDA);
        }
        delayMicroseconds(5);
        Y_SCK_H(SCK);
        delayMicroseconds(5); 
        Y_SCK_L(SCK);
        Txdata <<= 1;
        delayMicroseconds(5);
    }
    delayMicroseconds(2);  
    Y_SDA_H(SDA);
    delayMicroseconds(2);
}

u8 Y_IIC_ReadByte(u8 ack, const int SCK, const int SDA) {
    u8 i, receive = 0;
    delayMicroseconds(2);
    Y_SDA_H(SDA);
    delayMicroseconds(2);
    for (i = 0; i < 8; ++i) {
        Y_SCK_L(SCK);
        delayMicroseconds(5);
        Y_SCK_H(SCK);
        receive <<= 1;
        delayMicroseconds(5);
        pinMode(SDA, INPUT);
        if(Y_READ_SDA(SDA)) {
        receive |= 0x01;
        } else {
        receive &= 0xfe;
        }
        delayMicroseconds(5);
    }
    if(!ack) {
        Y_IIC_Nack(SCK, SDA);
    } else {
        Y_IIC_Ack(SCK, SDA);
    }

    Y_SCK_L(SCK);
    delayMicroseconds(2);

    return receive;
}

void AS5600_IIC_Write_OneByte(u8 deviceaddr, u8 writeaddr, u8 writedata, const int SCK, const int SDA) {
    Y_IIC_Start(SCK, SDA);
    Y_IIC_SendByte(deviceaddr & 0xfe, SCK, SDA);
    Y_IIC_Wait_Ack(SCK, SDA);
    Y_IIC_SendByte(writeaddr, SCK, SDA);
    Y_IIC_Wait_Ack(SCK, SDA);
    Y_IIC_SendByte(writedata, SCK, SDA);
    Y_IIC_Wait_Ack(SCK, SDA);
    Y_IIC_Stop(SCK, SDA);
    delay(10);               
}

u8 AS5600_IIC_Read_OneByte(u8 deviceaddr,u8 readaddr, const int SCK, const int SDA) {
    u8 temp;
    Y_IIC_Start(SCK, SDA);                          // 12us
    Y_IIC_SendByte(deviceaddr & 0xfe, SCK, SDA);    // 124us
    Y_IIC_Wait_Ack(SCK, SDA);
    Y_IIC_SendByte(readaddr, SCK, SDA);             // 124us
    Y_IIC_Wait_Ack(SCK, SDA);

    Y_IIC_Start(SCK, SDA);                          // 12us
    Y_IIC_SendByte(deviceaddr | 0x01, SCK, SDA);    // 124us
    Y_IIC_Wait_Ack(SCK, SDA);
    temp = Y_IIC_ReadByte(0, SCK, SDA);             // 140us
    Y_IIC_Stop(SCK, SDA);                           // 15us

    // total: about 1ms latency
    return temp;
}

int read_sensor_values(const int SCK, const int SDA) {
    word value = AS5600_IIC_Read_OneByte((0x36<<1), 0x0e, SCK, SDA);   
    value <<= 8;
    value |= AS5600_IIC_Read_OneByte((0x36<<1), 0x0f, SCK, SDA); 
    return value;
}
