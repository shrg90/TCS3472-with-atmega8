//*****************************************************************************
// Header Of Usefull Function In This Library For I2C
// Copyright :              wWw.RoboticNGO.com     
// Author :                 S_Ahmad (Seyyed Ahmad Mousavi)
// Remarks :
// known Problems :         None
// Version :                1.2
// Date :                   1392/10/23
// Company :                wWw.RoboticNGO.com
// Compiler:                CodeVisionAVR V2.05.3+
//*****************************************************************************
#ifndef _I2C_UFF_H_
    #define _I2C_UFF_H_

    #include <i2c.h>
    #include <delay.h>

#pragma used+

    #ifndef true
        #define true    1
    #endif
    #ifndef false
        #define false   0
    #endif    

//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
    extern unsigned char readByte_i2c(unsigned char DevAddr, unsigned char Reg);
    extern void writeByte_i2c(unsigned char DevAddr, unsigned char Reg , unsigned char Data);
    extern unsigned char readBits_i2c(unsigned char DevAddr, unsigned char Reg, unsigned char BitStart, unsigned char Length);
    extern unsigned char readBit_i2c(unsigned char DevAddr, unsigned char Reg, unsigned char Bit);
    extern void writeBits_i2c(unsigned char DevAddr, unsigned char Reg, unsigned char BitStart, unsigned char Length, unsigned char Data); 
    extern void writeBit_i2c(unsigned char DevAddr, unsigned char Reg, unsigned char BitNum, unsigned char Data);
    extern char readBytes_i2c(unsigned char DevAddr, unsigned char RegAddr, unsigned char length, unsigned char *data); 
    extern void writeBytes_i2c(unsigned char DevAddr, unsigned char RegAddr, unsigned char length, unsigned char *data);
    extern void writeWord_i2c(unsigned char DevAddr, unsigned char RegAddr, unsigned int Data); 
//////--<><><>----<><><>  S_Ahmad  <<<<  www.RoboticNGO.com  >>>>  I2C Lib  <><><>----<><><>--
#pragma used-
#endif /* _I2C_UFF_H_ */
