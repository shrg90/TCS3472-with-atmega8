//*****************************************************************************
// Source Of Usefull Function In This Library For I2C
// Copyright :              wWw.RoboticNGO.com     
// Author :                 S_Ahmad (Seyyed Ahmad Mousavi)
// Remarks :
// known Problems :         None
// Version :                1.2
// Date :                   1392/10/23
// Company :                wWw.RoboticNGO.com
// Compiler:                CodeVisionAVR V2.05.3+
//*****************************************************************************
#include "I2C_UseFullLib\UFF4I2C.h"

//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
unsigned char readByte_i2c(unsigned char DevAddr, unsigned char Reg)
{
    unsigned char Data;
    i2c_start();
    i2c_write(DevAddr);
    i2c_write(Reg);
    i2c_start();
    i2c_write(DevAddr + 1);
    //delay_us(1);
    Data=i2c_read(0);       // Ack = 0
    i2c_stop();
    return Data;
}
//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
void writeByte_i2c(unsigned char DevAddr, unsigned char Reg , unsigned char Data)
{
    i2c_start();
    i2c_write(DevAddr);
    i2c_write(Reg);
    i2c_write(Data);
    i2c_stop();
}
//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
unsigned char readBits_i2c(unsigned char DevAddr, unsigned char Reg, unsigned char BitStart, unsigned char Length)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    unsigned char Temp;
    unsigned char mask;
    if(Length > 0) 
    {
		Temp = readByte_i2c(DevAddr, Reg);
		if (Temp != 0) 
        {
			mask = ((1<<Length) - 1) << (BitStart - Length + 1);
			Temp &= mask;
			Temp >>= (BitStart - Length + 1);
		}
    }
    return Temp;
}
//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
unsigned char readBit_i2c(unsigned char DevAddr, unsigned char Reg, unsigned char Bit)
{
    if( readByte_i2c(DevAddr, Reg) & (1<<Bit) )
        return 1;
    else 
        return 0;
}
//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
void writeBits_i2c(unsigned char DevAddr, unsigned char Reg, unsigned char BitStart, unsigned char Length, unsigned char Data) 
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    unsigned char mask;
    unsigned char Temp;
	if(Length > 0)
    {
		Temp = 0;
        Temp = readByte_i2c(DevAddr, Reg);  //get current data
		if(Temp != 0)   
        {
		    mask = ((1<<Length) - 1) << (BitStart - Length + 1);
			Data <<= (BitStart - Length + 1); // shift data into correct position
			Data &= mask;    // zero all non-important bits in data
			Temp &= ~(mask); // zero all important bits in existing byte
			Temp |= Data;    // combine data with existing byte
			writeByte_i2c(DevAddr, Reg, Temp);
		}
	}
}
//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
void writeBit_i2c(unsigned char DevAddr, unsigned char Reg, unsigned char BitNum, unsigned char Data)
{
    if(Data == true)
        writeByte_i2c(DevAddr, Reg, (readByte_i2c(DevAddr, Reg) | (1<<BitNum)));
    else if(Data == false)
        writeByte_i2c(DevAddr, Reg, (readByte_i2c(DevAddr, Reg) & (~(1<<BitNum))));
    
    delay_ms(20);
}
//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
char readBytes_i2c(unsigned char DevAddr, unsigned char RegAddr, unsigned char length, unsigned char *data) 
{
    unsigned char i = 0;
    char count = 0;
    if(length > 0) 
    {
        for(i=0; i<length; i++) 
        {
            count++;
            if(i==(length-1))
            {
                i2c_start();
                i2c_write(DevAddr);
                i2c_write(RegAddr);
                i2c_start();
                i2c_write(DevAddr + 1);
                delay_us(10);
                data[i] = i2c_read(0);       // Ack = 0
                i2c_stop();
            }
            else
            {
                i2c_start();
                i2c_write(DevAddr);
                i2c_write(RegAddr);
                i2c_start();
                i2c_write(DevAddr + 1);
                delay_us(10);
                data[i] = i2c_read(1);       // Ack = 1
                i2c_stop();
            }
        }
        i2c_stop();
    }
    return count;
    
//  uint8_t i = 0;
//	int8_t count = 0;
//	if(length > 0) {
//		//request register
//		i2c_start(MPU6050_ADDR | I2C_WRITE);
//		i2c_write(RegAddr);
//		_delay_us(10);
//		//read data
//		i2c_start(MPU6050_ADDR | I2C_READ);
//		for(i=0; i<length; i++) {
//			count++;
//			if(i==length-1)
//				data[i] = i2c_readNak();
//			else
//				data[i] = i2c_readAck();
//		}
//		i2c_stop();
//	}
//	return count;
}
//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
void writeBytes_i2c(unsigned char DevAddr, unsigned char RegAddr, unsigned char length, unsigned char *data) 
{
unsigned char i=0;
	if(length > 0) 
    {
		for (i = 0; i < length; i++) 
        {
			writeByte_i2c(DevAddr, RegAddr, (unsigned char)data[i]);
            delay_ms(1);
		}
	}
}
//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
void writeWord_i2c(unsigned char DevAddr, unsigned char RegAddr, unsigned int Data) 
{
    writeByte_i2c(DevAddr, RegAddr, (Data>>8));       // send MSB
    writeByte_i2c(DevAddr, RegAddr+1, (Data&0xff));   // send LSB
}