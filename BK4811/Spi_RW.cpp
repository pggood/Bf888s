#include "App.h"
#include "Spi_RW.h"
#include <Arduino.h>    
   
/***************************************************************************  
//Wire 3  
***************************************************************************/   
   
#define WIRE3_MODE   
   
   
   
   
#define BK_DATA_HIGH()   digitalWrite(sbit_spi_sdio,1);   
#define BK_DATA_LOW()    digitalWrite(sbit_spi_sdio,0);   
#define BK_DATA_READ()    sbit_spi_sdio   
#define SDADIROUT()          
#define SDADIRIN()        digitalWrite(sbit_spi_sdio,1);   
   
#define SCLDIROUT()          
#define BK_CLK_HIGH()     digitalWrite(sbit_spi_clk,1);   
#define BK_CLK_LOW()      digitalWrite(sbit_spi_clk,0);   
   
   
   
   
   
void Wire3_Spi0_SPI_Addr(UINT8 wWord)   
{   
    INT8 i;   
    digitalWrite(sbit_spi_clk,0);   
    for(i=7;i>=0;i--)   
    {   
        if(wWord&0x80)   
        {   
            digitalWrite(sbit_spi_sdio,1);   
        }   
        else   
        {   
            digitalWrite(sbit_spi_sdio,0);   
        }   
   
//      digitalWrite(sbit_spi_clk,0);//delay   
        digitalWrite(sbit_spi_clk,1);   
        wWord=wWord<<1;   
        digitalWrite(sbit_spi_clk,0);   
    }   
}   
   
void Wire3_Spi0_SPI_WR(UINT16 wWord)   
{   
    INT8 i;    
    digitalWrite(sbit_spi_clk,0);   
    for(i=15;i>=0;i--)   
    {   
        if(wWord&0x8000)   
        {   
            digitalWrite(sbit_spi_sdio,1);   
        }   
        else   
        {   
            digitalWrite(sbit_spi_sdio,0);   
        }   
   
//      digitalWrite(sbit_spi_clk,0);//delay   
        digitalWrite(sbit_spi_clk,1);   
        wWord=wWord<<1;   
        digitalWrite(sbit_spi_clk,0);   
    }   
}   
   
   
UINT16 Wire3_Spi0_SPI_RD()   
{   
    INT8 i;   
    UINT16 wWord=0;   
    UINT16 temp_dat;
    digitalWrite(sbit_spi_sdio,1);
    pinMode(sbit_spi_sdio, INPUT);   
    for(i=15;i>=0;i--)   
    {   
        temp_dat = digitalRead(sbit_spi_sdio); 
        temp_dat = temp_dat << i;
        wWord |= temp_dat;
        digitalWrite(sbit_spi_clk,1);   
        digitalWrite(sbit_spi_clk,0);   
        delayMicroseconds(1);   
    }
    pinMode(sbit_spi_sdio, OUTPUT);   
    return wWord;   
}   
   
   
   
void BK_Write_Reg(UINT8 addr,UINT16 value)   
{   
    UINT16 tmpAddr;   
   
    //DISABLE_GLOBAL_INTRRUPT   
    digitalWrite(sbit_spi_sen,1);   
   
    tmpAddr=addr;   
    tmpAddr=tmpAddr<<1;   
   
    digitalWrite(sbit_spi_clk,0);   
   
    delayMicroseconds(1);   
    digitalWrite(sbit_spi_sen,0);   
   
   
    Wire3_Spi0_SPI_Addr(tmpAddr);//high bit 8,base-index 0   
   
    Wire3_Spi0_SPI_WR(value);//high bit 15,base-index 0   
   
    digitalWrite(sbit_spi_sen,1);   
   
    delayMicroseconds(1);   
   
    digitalWrite(sbit_spi_clk,1);   
    digitalWrite(sbit_spi_sdio,1);   
    //ENABLE_GLOBAL_INTRRUPT   
   
}   
   
UINT16 BK_Read_Reg(UINT8 addr)   
{   
    UINT16 tmpAddr;   
    UINT16 tmpValue;   
   
        //DISABLE_GLOBAL_INTRRUPT   
        digitalWrite(sbit_spi_sen,1);   
   
        tmpAddr=addr;   
   
        tmpAddr=tmpAddr<<1;   
        tmpAddr|=0x01;   
   
        digitalWrite(sbit_spi_clk,0);   
       
        delayMicroseconds(1);   
   
        digitalWrite(sbit_spi_sen,0);   
   
        Wire3_Spi0_SPI_Addr(tmpAddr);//high bit 8,base-index 0   
   
        tmpValue=Wire3_Spi0_SPI_RD();   
   
        digitalWrite(sbit_spi_sen,1);   
   
       
    delayMicroseconds(1);   
   
    digitalWrite(sbit_spi_clk,1);   
    digitalWrite(sbit_spi_sdio,1);   
       
    //ENABLE_GLOBAL_INTRRUPT   
   
    return tmpValue;   
   
}   
   
   
