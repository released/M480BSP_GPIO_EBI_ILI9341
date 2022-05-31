/*********************************************************************
*                 SEGGER Software GmbH                               *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2018  SEGGER Microcontroller GmbH                *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.48 - Graphical user interface for embedded applications **
All  Intellectual Property rights in the Software belongs to  SEGGER.
emWin is protected by  international copyright laws.  Knowledge of the
source code may not be used to write a similar product. This file may
only be used in accordance with the following terms:

The  software has  been licensed by SEGGER Software GmbH to Nuvoton Technology Corporationat the address: No. 4, Creation Rd. III, Hsinchu Science Park, Taiwan
for the purposes  of  creating  libraries  for its 
Arm Cortex-M and  Arm9 32-bit microcontrollers, commercialized and distributed by Nuvoton Technology Corporation
under  the terms and conditions  of  an  End  User  
License  Agreement  supplied  with  the libraries.
Full source code is available at: www.segger.com

We appreciate your understanding and fairness.
----------------------------------------------------------------------
Licensing information
Licensor:                 SEGGER Software GmbH
Licensed to:              Nuvoton Technology Corporation, No. 4, Creation Rd. III, Hsinchu Science Park, 30077 Hsinchu City, Taiwan
Licensed SEGGER software: emWin
License number:           GUI-00735
License model:            emWin License Agreement, signed February 27, 2018
Licensed platform:        Cortex-M and ARM9 32-bit series microcontroller designed and manufactured by or for Nuvoton Technology Corporation
----------------------------------------------------------------------
Support and Update Agreement (SUA)
SUA period:               2018-03-26 - 2019-03-27
Contact to extend SUA:    sales@segger.com
----------------------------------------------------------------------
File        : LCDConf.c
Purpose     : Display controller configuration (single layer)
---------------------------END-OF-HEADER------------------------------
*/

#include <stddef.h>
#include <stdio.h>

#include "GUI.h"
#include "GUIDRV_FlexColor.h"

#include "NuMicro.h"

// #include "TouchPanel.h"

/*********************************************************************
*
*       Layer configuration
*
**********************************************************************
*/

#define ENABLE_GPIO_EMULATE_EBI


/*
	PANEL pin define : 240 x 320 , 2.4" inch
	
	PAENL/M487EVB
        [LEFT]						[RIGHT]
        X		/PB2				X		/PB1   /PB6
        X		/PB3				LCD_RST	/PB0   /PB7
        LCD_D2	/PC9				LCD_CS	/PB9   /PB8
        LCD_D3	/PC10				LCD_RS	/PB8   /PB9
        LCD_D4	/PC11				LCD_WR	/PB7   /PB0
        LCD_D5	/PC12				LCD_RD	/PB6   /PB1
        LCD_D6	/PE4	
        LCD_D7	/PE5				X		/VIN
                                    GND		/VSS
        LCD_D0	/PA5				X		/VSS
        LCD_D1	/PA4				5V		/5VCC
        SD_SS	/PA3				3V3		/3VCC
        SD_DI	/PA0				X		/nRESET
        SD_DO	/PA1						/VDD
        SD_SCL	/PA2						/NC
        X		/VSS
        X		/VREF
                /PG1
                /PG0

*/

#ifndef HIGH
#define HIGH              					(1)
#endif

#ifndef LOW
#define LOW              					(0)
#endif

#ifndef COUNTOF
#define COUNTOF(a)                          (sizeof((a))/sizeof((a)[0]))
#endif

#define LCD_D0                              (PA5)
#define LCD_D1                              (PA4)
#define LCD_D2                              (PC9)
#define LCD_D3                              (PC10)
#define LCD_D4                              (PC11)
#define LCD_D5                              (PC12)
#define LCD_D6                              (PE4)
#define LCD_D7                              (PE5)

#define LCD_RST	                            (PB0)
#define LCD_CS	                            (PB9)
#define LCD_RS	                            (PB8)
#define LCD_WR	                            (PB7)
#define LCD_RD	                            (PB6)



#define CD_COMMAND                          (LCD_RS = LOW)
#define CD_DATA                             (LCD_RS = HIGH)

#define RD_ACTIVE                           (LCD_RD = LOW) 
#define RD_IDLE                             (LCD_RD = HIGH)
#define WR_ACTIVE                           (LCD_WR = LOW) 
#define WR_IDLE                             (LCD_WR = HIGH)
#define CS_ACTIVE                           (LCD_CS = LOW)
#define CS_IDLE                             (LCD_CS = HIGH)
#define CS_ACTIVE_CD_COMMAND	            { CS_ACTIVE; CD_COMMAND; }
#define WR_STROBE                           { WR_ACTIVE; WR_IDLE; }


//
// Physical display size
//
#define XSIZE_PHYS 240
#define YSIZE_PHYS 320

//
// Color conversion
//
#define COLOR_CONVERSION GUICC_565

//
// Display driver
//
#define DISPLAY_DRIVER GUIDRV_FLEXCOLOR

//
// Orientation
//
#define DISPLAY_ORIENTATION (0)

//
// Hardware related
//

/* LCD Module "RESET" */
#define SET_RST                             (LCD_RST = 1)
#define CLR_RST                             (LCD_RST = 0)

/*********************************************************************
*
*       Configuration checking
*
**********************************************************************
*/
#ifndef   VXSIZE_PHYS
#define VXSIZE_PHYS XSIZE_PHYS
#endif
#ifndef   VYSIZE_PHYS
#define VYSIZE_PHYS YSIZE_PHYS
#endif
#ifndef   XSIZE_PHYS
#error Physical X size of display is not defined!
#endif
#ifndef   YSIZE_PHYS
#error Physical Y size of display is not defined!
#endif
#ifndef   COLOR_CONVERSION
#error Color conversion not defined!
#endif
#ifndef   DISPLAY_DRIVER
#error No display driver defined!
#endif
#ifndef   DISPLAY_ORIENTATION
#define DISPLAY_ORIENTATION 0
#endif

/*********************************************************************
*
*       Static code
*
**********************************************************************/

const uint8_t tlb_Power_Control_A_CBh[] =
{
    0xCB ,
    0x39 ,
    0x2C ,
    0x00 ,
    0x34 , 
    0x02 
};

const uint8_t tlb_Power_Control_B_CFh[] =
{
    0xCF ,
    0x00 ,
    0xC1 ,      // DATASHEET : 0x81
    0x30
};


const uint8_t tlb_Driver_Timing_Control_A_E8h[] =
{
    0xE8 ,
    0x85 ,      // DATASHEET : 0x84
    0x00 ,      // DATASHEET : 0x11
    0x78        // DATASHEET : 0x7A
};

const uint8_t tlb_Driver_Timing_Control_B_EAh[] =
{
    0xEA ,
    0x00 ,      // DATASHEET : 0x66
    0x00
};

const uint8_t tlb_Power_On_Sequence_Control_EDh[] =
{
    0xED ,
    0x64 ,      // DATASHEET : 0x55
    0x03 ,      // DATASHEET : 0x01
    0x12 ,      // DATASHEET : 0x23
    0x81        // DATASHEET : 0x01
};

const uint8_t tlb_Pump_ratio_control_F7h[] =
{
    0xF7 ,
    0x20        // DATASHEET : 0x10
};

const uint8_t tlb_Power_Control_1_C0h[] =
{
    0xC0 ,
    0x23        // DATASHEET : 0x21
};

const uint8_t tlb_Power_Control_2_C1h[] =
{
    0xC1 ,
    0x10
};

const uint8_t tlb_VCOM_Control_1_C5h[] =
{
    0xC5 ,
    0x3E ,      // DATASHEET : 0x31
    0x28        // DATASHEET : 0x3C
};

const uint8_t tlb_VCOM_Control_2_C7h[] =
{
    0xC7 ,
    0x86        // DATASHEET : 0xC0
};

const uint8_t tlb_Memory_Access_Control_36h[] =
{
    0x36 ,
    0x48        // DATASHEET : 0x00
};

const uint8_t tlb_COLMOD_Pixel_Format_Set_3Ah[] =
{
    0x3A ,
    0x55        // DATASHEET : 0x66
};

const uint8_t tlb_Frame_Rate_Control_In_Normal_Mode_Full_Colors_B1h[] =
{
    0xB1 ,
    0x00 ,
    0x18        // DATASHEET : 0x1B  
};

const uint8_t tlb_Display_Function_Control_B6h[] =
{
    0xB6 ,
    0x08 ,      // DATASHEET : 0x0A
    0x82 , 
    0x27   
};

const uint8_t tlb_Enable_3G_F2h[] =
{
    0xF2 ,
    0x00        // DATASHEET : 0x02
};

const uint8_t tlb_Gamma_Set_26h[] =
{
    0x26 ,
    0x01
};

const uint8_t tlb_Positive_Gamma_Correction_E0h[] =
{
    0xE0 ,
    0x0F ,
    0x31 ,
    0x2B ,
    0x0C ,
    0x0E ,
    0x08 ,
    0x4E ,
    0xF1 ,
    0x37 ,
    0x07 ,
    0x10 ,
    0x03 ,
    0x0E ,
    0x09 ,
    0x00 
};

const uint8_t tlb_Negative_Gamma_Correction_E1h[] =
{
    0xE1 ,
    0x00 ,
    0x0E ,
    0x14 ,
    0x03 ,
    0x11 ,
    0x07 ,
    0x31 ,
    0xC1 ,
    0x48 ,
    0x08 ,
    0x0F ,
    0x0C ,
    0x31 ,
    0x36 ,
    0x0F
};

void LCD_Data_SetInput(void);
void LCD_Data_SetOutput(void);

#define setReadDir()                        (LCD_Data_SetInput())
#define setWriteDir()                       (LCD_Data_SetOutput())

void delayMicroseconds(uint32_t t)
{
    CLK_SysTickDelay(t);
}

void LCD_Data_SetInput(void)
{
    /*
        #define LCD_D0                              (PA5)
        #define LCD_D1                              (PA4)
        #define LCD_D2                              (PC9)
        #define LCD_D3                              (PC10)
        #define LCD_D4                              (PC11)
        #define LCD_D5                              (PC12)
        #define LCD_D6                              (PE4)
        #define LCD_D7                              (PE5)
    */

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk );
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_GPIO | SYS_GPA_MFPL_PA5MFP_GPIO );    
    GPIO_SetMode(PA, BIT4|BIT5, GPIO_MODE_INPUT);

    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC9MFP_Msk |SYS_GPC_MFPH_PC10MFP_Msk | SYS_GPC_MFPH_PC11MFP_Msk | SYS_GPC_MFPH_PC12MFP_Msk );
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC9MFP_GPIO |SYS_GPC_MFPH_PC10MFP_GPIO | SYS_GPC_MFPH_PC11MFP_GPIO | SYS_GPC_MFPH_PC12MFP_GPIO );    
    GPIO_SetMode(PC, BIT9|BIT10|BIT11|BIT12, GPIO_MODE_INPUT);

    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE4MFP_Msk | SYS_GPE_MFPL_PE5MFP_Msk );
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE4MFP_GPIO | SYS_GPE_MFPL_PE5MFP_GPIO );    
    GPIO_SetMode(PE, BIT4|BIT5, GPIO_MODE_INPUT);    
}
void LCD_Data_SetOutput(void)
{
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA4MFP_Msk | SYS_GPA_MFPL_PA5MFP_Msk );
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA4MFP_GPIO | SYS_GPA_MFPL_PA5MFP_GPIO );    
    GPIO_SetMode(PA, BIT4|BIT5, GPIO_MODE_OUTPUT);

    SYS->GPC_MFPH &= ~(SYS_GPC_MFPH_PC9MFP_Msk |SYS_GPC_MFPH_PC10MFP_Msk | SYS_GPC_MFPH_PC11MFP_Msk | SYS_GPC_MFPH_PC12MFP_Msk );
    SYS->GPC_MFPH |= (SYS_GPC_MFPH_PC9MFP_GPIO |SYS_GPC_MFPH_PC10MFP_GPIO | SYS_GPC_MFPH_PC11MFP_GPIO | SYS_GPC_MFPH_PC12MFP_GPIO );    
    GPIO_SetMode(PC, BIT9|BIT10|BIT11|BIT12, GPIO_MODE_OUTPUT);

    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE4MFP_Msk | SYS_GPE_MFPL_PE5MFP_Msk );
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE4MFP_GPIO | SYS_GPE_MFPL_PE5MFP_GPIO );    
    GPIO_SetMode(PE, BIT4|BIT5, GPIO_MODE_OUTPUT); 
}

/*
                                        CS      WR      RD      D/CX(RS)
    Write command code.                 L       rising  H       L
    Read internal status.               L       H       rising  H 
    Write parameter or display data.    L       rising  H       H
    Reads parameter or display data.    L       H       rising  H


    WRITE cycle (RESET always HIGH)
    CS LOW , 
        D/CX(RS) LOW (if send addr)
                                                        D/CX(RS) set high after WR high
        WR set LOW 
                            WR set high when addr ready
        RD high
        DATA bus : addr

        D/CX(RS) high (if send data)
        WR set LOW
                            WR set high when data ready        
        RD high
        DATA bus : data


    READ cycle (RESET always HIGH)
    CS LOW , 
        D/CX(RS) LOW (if send addr)
                                                        D/CX(RS) set high after WR high
        WR set LOW 
                            WR set high when addr ready
        RD high
        DATA bus : addr

        D/CX(RS) high (if READ data)
        WR high    

        // FIRST DATA INVALID
        RD set LOW 
                            RD set high when data ready   
        DATA bus : data   

        // second DATA is VALID 
        RD set LOW 
                            RD set high when data ready   
        DATA bus : data               

*/

uint8_t read8(void)
{
    uint8_t temp = 0;
    #if defined (ENABLE_GPIO_EMULATE_EBI)       
    RD_ACTIVE;
    // delayMicroseconds(10);

    temp |= LCD_D0  << 0;
    temp |= LCD_D1  << 1;
    temp |= LCD_D2  << 2;
    temp |= LCD_D3  << 3;
    temp |= LCD_D4  << 4;
    temp |= LCD_D5  << 5;
    temp |= LCD_D6  << 6;
    temp |= LCD_D7  << 7;
    // delayMicroseconds(10);

    RD_IDLE;
    // delayMicroseconds(10);
    #else

    #endif

    return temp;
}

void write8(uint8_t c) 
{ 
    #if defined (ENABLE_GPIO_EMULATE_EBI)       
    /*Serial.print(" write8: "); Serial.print(c,HEX); Serial.write(',');*/
    WR_ACTIVE;
    LCD_D0 = (c&BIT0)?HIGH:LOW;
    LCD_D1 = (c&BIT1)?HIGH:LOW;
    LCD_D2 = (c&BIT2)?HIGH:LOW;
    LCD_D3 = (c&BIT3)?HIGH:LOW;
    LCD_D4 = (c&BIT4)?HIGH:LOW;
    LCD_D5 = (c&BIT5)?HIGH:LOW;
    LCD_D6 = (c&BIT6)?HIGH:LOW;
    LCD_D7 = (c&BIT7)?HIGH:LOW;
    // delayMicroseconds(10);

    WR_IDLE;
    // delayMicroseconds(10);    
    #else
    EBI0_WRITE_DATA8(0x00000000, cmd);

    #endif
}

void writeCommand8(uint8_t c)
{
    #if defined (ENABLE_GPIO_EMULATE_EBI)       
	CS_ACTIVE_CD_COMMAND;
	write8(c&0xFF);
    CD_DATA;
    #else    
    CD_COMMAND;
	write8(c&0xFF);
    CD_DATA;
    #endif
}

// void writeCommand16(uint16_t c)
// {
// 	CS_ACTIVE_CD_COMMAND;
// 	write8(c>>8);
// 	write8(c&0xFF);
//     CD_DATA;
// }

// void writeData16(uint16_t c)
// {
// 	write8(c>>8);
// 	write8(c&0xFF);
// }


/*-----------------------------------------------*/
// Write control registers of LCD module  
// 
/*-----------------------------------------------*/
void LCD_WR_REG(uint8_t cmd)   //void LCD_WR_REG(uint16_t cmd)
{
    writeCommand8(cmd);
}

uint16_t LCD_RD_REG(uint16_t r)
{
    uint16_t id;

    #if defined (ENABLE_GPIO_EMULATE_EBI)       
    uint8_t x;

    writeCommand8(r);
    setReadDir();  // Set up LCD data port(s) for READ operations
    delayMicroseconds(10);

    // first data is invalid
    x = read8();
    id = x;                 // Do not merge or otherwise simplify
    id <<= 8;               // these lines.  It's an unfortunate
    
    // second data is valid   
    id = 0; 
    x = read8();
    id |= x;        // shenanigans that are going on.
    CS_IDLE;
    setWriteDir();  // Restore LCD data port(s) to WRITE configuration

    //Serial.print("Read $"); Serial.print(r, HEX); 
    //Serial.print(":\t0x"); Serial.println(id, HEX);
    #else
    id = EBI0_READ_DATA8(r);
    #endif

    return id;    
}


/*-----------------------------------------------*/
// Write data to SRAM of LCD module  
// 
/*-----------------------------------------------*/
void LCD_WR_DATA(uint8_t dat)  //void LCD_WR_DATA(uint16_t dat)
{
    #if defined (ENABLE_GPIO_EMULATE_EBI)       
	write8(dat&0xFF);
    #else
	write8(dat&0xFF);
    #endif
}


/*-----------------------------------------------*/
// Read data from SRAM of LCD module 
// 
/*-----------------------------------------------*/
uint8_t LCD_RD_DATA(void)  //uint16_t LCD_RD_DATA(void)
{
    uint8_t id;    

    #if defined (ENABLE_GPIO_EMULATE_EBI)   
    uint8_t x;

    // first data is invalid
    x = read8();
    id = x;                 // Do not merge or otherwise simplify
    // id <<= 8;               // these lines.  It's an unfortunate
    
    // second data is valid   
    id = 0; 
    x = read8();
    id |= x;        // shenanigans that are going on.
    CS_IDLE;
    setWriteDir();  // Restore LCD data port(s) to WRITE configuration

    //Serial.print("Read $"); Serial.print(r, HEX); 
    //Serial.print(":\t0x"); Serial.println(id, HEX);
 
    #else

    #endif

    return id;       
}

/********************************************************************
*
*       LcdWriteDataMultiple
*
*   Function description:
*   Writes multiple values to a display register.
*/
void LcdWriteDataMultipleREG(uint8_t * pData, int NumItems)   //static void LcdWriteDataMultiple(uint16_t * pData, int NumItems)
{
    #if defined (ENABLE_GPIO_EMULATE_EBI)    
	CS_ACTIVE_CD_COMMAND;	
    while (NumItems--) 
    {
        write8(*pData++);
    }
    CD_DATA;    
    #else

    #endif
}

void LcdWriteDataMultipleData(uint8_t * pData, int NumItems)
{
    #if defined (ENABLE_GPIO_EMULATE_EBI)    
    while (NumItems--) 
    {
        write8(*pData++);
    }
    #else

    #endif
}

/********************************************************************
*
*       LcdReadDataMultiple
*
*   Function description:
*   Reads multiple values from a display register.
*/
void LcdReadDataMultiple(uint8_t * pData, int NumItems)    // static void LcdReadDataMultiple(uint16_t * pData, int NumItems)
{
    #if defined (ENABLE_GPIO_EMULATE_EBI)    
    uint8_t id;
    uint8_t x;

    // first data is invalid
    x = read8();
    id = x;                 // Do not merge or otherwise simplify
    // id <<= 8;               // these lines.  It's an unfortunate
    
    // second data is valid   
    id = 0; 
    while (NumItems--) 
    {
        *pData++ = read8();
    }

    CS_IDLE;
    setWriteDir();  // Restore LCD data port(s) to WRITE configuration    
    #else

    #endif
}

/*-----------------------------------------------*/
// Set cursor positon on the LCD screen 
// 
/*-----------------------------------------------*/
#if 0
void LCD_SetCursor(uint16_t x_s, uint16_t y_s)
{ 
    uint16_t x_e = x_s+1;
    uint16_t y_e = y_s+1;
        
    /* X range */
    LCD_WR_REG(0x2A);
    LCD_WR_DATA(x_s>>8);
    LCD_WR_DATA(x_s);
    LCD_WR_DATA(x_e>>8);
    LCD_WR_DATA(x_e);

    /* Y range */
    LCD_WR_REG(0x2B);
    LCD_WR_DATA(y_s>>8);
    LCD_WR_DATA(y_s);
    LCD_WR_DATA(y_e>>8);
    LCD_WR_DATA(y_e);
        
    /* Memory write */
    LCD_WR_REG(0x2C);
    
}
#endif

/*-----------------------------------------------*/
// Initial LIL9341 LCD driver chip 
// 
/*-----------------------------------------------*/
void tbl_Initial(uint8_t REG , uint8_t* Item)
{
    #if defined (ENABLE_GPIO_EMULATE_EBI)

    uint8_t len = COUNTOF(Item) - 1;
    uint8_t i = 0;

    writeCommand8(REG);

    for ( i = 0 ; i < len ; i++)
    {
        write8(Item[i]);
    }
    
    CS_IDLE;

    #else

    #endif
}


void ILI9341_Initial(void)
{
    uint16_t Reg = 0;
    
    /* Hardware reset */
    SET_RST;
    delayMicroseconds(5000);     // Delay 5ms
    
    CLR_RST;
    delayMicroseconds(20000);    // Delay 20ms
    
    SET_RST;
    delayMicroseconds(40000);    // Delay 40ms

    tbl_Initial(tlb_Power_Control_A_CBh[0],(uint8_t*)&tlb_Power_Control_A_CBh[1] );
    tbl_Initial(tlb_Power_Control_B_CFh[0],(uint8_t*)&tlb_Power_Control_B_CFh[1] );
    tbl_Initial(tlb_Driver_Timing_Control_A_E8h[0],(uint8_t*)&tlb_Driver_Timing_Control_A_E8h[1] );
    tbl_Initial(tlb_Driver_Timing_Control_B_EAh[0],(uint8_t*)&tlb_Driver_Timing_Control_B_EAh[1] );
    tbl_Initial(tlb_Power_On_Sequence_Control_EDh[0],(uint8_t*)&tlb_Power_On_Sequence_Control_EDh[1] );
    tbl_Initial(tlb_Pump_ratio_control_F7h[0],(uint8_t*)&tlb_Pump_ratio_control_F7h[1] );
    tbl_Initial(tlb_Power_Control_1_C0h[0],(uint8_t*)&tlb_Power_Control_1_C0h[1] );
    tbl_Initial(tlb_Power_Control_2_C1h[0],(uint8_t*)&tlb_Power_Control_2_C1h[1] );
    tbl_Initial(tlb_VCOM_Control_1_C5h[0],(uint8_t*)&tlb_VCOM_Control_1_C5h[1] );
    tbl_Initial(tlb_VCOM_Control_2_C7h[0],(uint8_t*)&tlb_VCOM_Control_2_C7h[1] );
    tbl_Initial(tlb_Memory_Access_Control_36h[0],(uint8_t*)&tlb_Memory_Access_Control_36h[1] );
    tbl_Initial(tlb_COLMOD_Pixel_Format_Set_3Ah[0],(uint8_t*)&tlb_COLMOD_Pixel_Format_Set_3Ah[1] );
    tbl_Initial(tlb_Frame_Rate_Control_In_Normal_Mode_Full_Colors_B1h[0],(uint8_t*)&tlb_Frame_Rate_Control_In_Normal_Mode_Full_Colors_B1h[1] );
    tbl_Initial(tlb_Display_Function_Control_B6h[0],(uint8_t*)&tlb_Display_Function_Control_B6h[1] );
    tbl_Initial(tlb_Enable_3G_F2h[0],(uint8_t*)&tlb_Enable_3G_F2h[1] );
    tbl_Initial(tlb_Gamma_Set_26h[0],(uint8_t*)&tlb_Gamma_Set_26h[1] );
    tbl_Initial(tlb_Positive_Gamma_Correction_E0h[0],(uint8_t*)&tlb_Positive_Gamma_Correction_E0h[1] );
    tbl_Initial(tlb_Negative_Gamma_Correction_E1h[0],(uint8_t*)&tlb_Negative_Gamma_Correction_E1h[1] );

    LCD_WR_REG(0x11);
    delayMicroseconds(200000);   // Delay 200ms
    
    LCD_WR_REG(0x29);           //Display on

    LCD_WR_REG(0x0A);
    Reg = LCD_RD_DATA();
    printf("0Ah = %02x.\n", Reg);
    
    LCD_WR_REG(0x0B);
    Reg = LCD_RD_DATA();
    printf("0Bh = %02x.\n", Reg);
    
    LCD_WR_REG(0x0C);
    Reg = LCD_RD_DATA();
    printf("0Ch = %02x.\n", Reg);

    printf("Initial ILI9341 LCD Module done.\n\n");

}

//void I2C2_Init(void)
//{
    /* Open I2C2 and set clock to 100k */
    //I2C_Open(I2C2, 100000);
//}

//void EINT7_IRQHandler(void)
//{
//    /* To check if PB.9 external interrupt occurred */
//    if(GPIO_GET_INT_FLAG(PB, BIT9)) {
//        GPIO_CLR_INT_FLAG(PB, BIT9);
//        printf("PB.9 EINT7 occurred.\n");
//    }

//}


void EBI_FuncPinInit(void)
{
    #if defined (ENABLE_GPIO_EMULATE_EBI)
    /*
        #define LCD_RST	                            (PB0)
        #define LCD_CS	                            (PB9)
        #define LCD_RS	                            (PB8)
        #define LCD_WR	                            (PB7)
        #define LCD_RD	                            (PB6)
    */

    LCD_Data_SetOutput();

    // SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
    // SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_GPIO | SYS_GPB_MFPL_PB1MFP_GPIO | SYS_GPB_MFPL_PB7MFP_GPIO);    
    // GPIO_SetMode(PB, BIT0|BIT1|BIT7, GPIO_MODE_OUTPUT);
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_GPIO | SYS_GPB_MFPL_PB6MFP_GPIO | SYS_GPB_MFPL_PB7MFP_GPIO);    
    GPIO_SetMode(PB, BIT0|BIT6|BIT7, GPIO_MODE_OUTPUT);    

    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB8MFP_Msk | SYS_GPB_MFPH_PB9MFP_Msk );
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB8MFP_GPIO | SYS_GPB_MFPH_PB9MFP_GPIO );    
    GPIO_SetMode(PB, BIT8|BIT9, GPIO_MODE_OUTPUT);

    CD_DATA;

    #else

    /*
        LCD_D0  PC0
        LCD_D1  PC1
        LCD_D2  PC2
        LCD_D3  PC3
        LCD_D4  PC4
        LCD_D5  PC5
        LCD_D6  PA6
        LCD_D7  PA7

        LCD_CS  PB7
        LCD_WR  PA10
        LCD_RD  PA11

        LCD_RST	/PB0   
        LCD_RS	/PB5 
    */

    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk  | SYS_GPC_MFPL_PC1MFP_Msk |
                       SYS_GPC_MFPL_PC2MFP_Msk | SYS_GPC_MFPL_PC3MFP_Msk |
                       SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_EBI_AD0  | SYS_GPC_MFPL_PC1MFP_EBI_AD1 |
                      SYS_GPC_MFPL_PC2MFP_EBI_AD2 | SYS_GPC_MFPL_PC3MFP_EBI_AD3 |
                      SYS_GPC_MFPL_PC4MFP_EBI_AD4 | SYS_GPC_MFPL_PC5MFP_EBI_AD5);

    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA6MFP_Msk | SYS_GPA_MFPL_PA7MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA6MFP_EBI_AD6 | SYS_GPA_MFPL_PA7MFP_EBI_AD7);


    /* EBI RD and WR pins on PE.4 and PE.5 */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA11MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA10MFP_EBI_nWR | SYS_GPA_MFPH_PA11MFP_EBI_nRD);

    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB7MFP_Msk;
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB7MFP_EBI_nCS0;


    /* Initialize EBI bank0 to access external LCD Module */
    EBI_Open(EBI_BANK0, EBI_BUSWIDTH_8BIT, EBI_TIMING_NORMAL, EBI_OPMODE_NORMAL, EBI_CS_ACTIVE_LOW);
    EBI->CTL0 |= EBI_CTL_CACCESS_Msk;
    EBI->TCTL0 |= (EBI_TCTL_WAHDOFF_Msk | EBI_TCTL_RAHDOFF_Msk);
    printf("\n[EBI CTL0:0x%08X, TCLT0:0x%08X]\n\n", EBI->CTL0, EBI->TCTL0);

    /*
        LCD_RST	/PB0
        LCD_RS	/PB5
    */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk );
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_GPIO | SYS_GPB_MFPL_PB5MFP_GPIO);    
    GPIO_SetMode(PB, BIT5 | BIT5, GPIO_MODE_OUTPUT);

    SET_RST;
    CD_DATA;

    #endif
}

/*********************************************************************
*
*       _InitController
*
* Purpose:
*   Initializes the display controller
*/
static void _InitController(void)
{
    static uint8_t s_InitOnce = 0;

    if (s_InitOnce == 0)
        s_InitOnce = 1;
    else
        return;

    /* Configure DC/RESET/LED pins */
    EBI_FuncPinInit();

    /* Init LCD Module */
    ILI9341_Initial();
    

}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
#if GUI_SUPPORT_TOUCH
extern int ts_phy2log(int *sumx, int *sumy);

void GUI_TOUCH_X_ActivateX(void) {
}

void GUI_TOUCH_X_ActivateY(void) {
}


 
int  GUI_TOUCH_X_MeasureX(void) {
  int sumx;
  int sumy;
	if (Read_TouchPanel(&sumx, &sumy))
	{
//		sysprintf("X = %d\n", sumx);
		ts_phy2log(&sumx, &sumy);		
    return sumx;
	}
	return -1;
}

int  GUI_TOUCH_X_MeasureY(void) {
  int sumx;
  int sumy;
	if ( Read_TouchPanel(&sumx, &sumy) )
	{
//		sysprintf("Y = %d\n", sumy);
		ts_phy2log(&sumx, &sumy);				
    return sumy;
	}
	return -1;
}
#endif
/*********************************************************************
*
*       LCD_X_Config
*
* Purpose:
*   Called during the initialization process in order to set up the
*   display driver configuration.
*
*/
void LCD_X_Config(void)
{
    GUI_DEVICE * pDevice;
    CONFIG_FLEXCOLOR Config = {0};
    GUI_PORT_API PortAPI = {0};
    //
    // Set display driver and color conversion
    //
    pDevice = GUI_DEVICE_CreateAndLink(DISPLAY_DRIVER, COLOR_CONVERSION, 0, 0);

    //
    // Display driver configuration
    //
    LCD_SetSizeEx (0, XSIZE_PHYS,   YSIZE_PHYS);
    LCD_SetVSizeEx(0, VXSIZE_PHYS,  VYSIZE_PHYS);
    //
    // Orientation
    //
    Config.Orientation = GUI_MIRROR_X | GUI_MIRROR_Y | GUI_SWAP_XY;
    GUIDRV_FlexColor_Config(pDevice, &Config);
    //
    // Set controller and operation mode
    //
    // PortAPI.pfWrite16_A0  = LCD_WR_REG;
    // PortAPI.pfWrite16_A1  = LCD_WR_DATA;
    // PortAPI.pfWriteM16_A0 = LcdWriteDataMultiple;
    // PortAPI.pfWriteM16_A1 = LcdWriteDataMultiple;
    // PortAPI.pfRead16_A0   = LCD_RD_DATA;
    // PortAPI.pfRead16_A1   = LCD_RD_DATA;
    // PortAPI.pfReadM16_A0  = LcdReadDataMultiple;
    // PortAPI.pfReadM16_A1  = LcdReadDataMultiple;

    PortAPI.pfWrite8_A0  = LCD_WR_REG;
    PortAPI.pfWrite8_A1  = LCD_WR_DATA;
    PortAPI.pfWriteM8_A0 = LcdWriteDataMultipleREG;
    PortAPI.pfWriteM8_A1 = LcdWriteDataMultipleData;
    PortAPI.pfRead8_A0   = LCD_RD_DATA;
    PortAPI.pfRead8_A1   = LCD_RD_DATA;
    PortAPI.pfReadM8_A0  = LcdReadDataMultiple;
    PortAPI.pfReadM8_A1  = LcdReadDataMultiple;


    GUIDRV_FlexColor_SetReadFunc66709_B16(pDevice, GUIDRV_FLEXCOLOR_READ_FUNC_III);
    GUIDRV_FlexColor_SetFunc(pDevice, &PortAPI, GUIDRV_FLEXCOLOR_F66709, GUIDRV_FLEXCOLOR_M16C0B8);

#if GUI_SUPPORT_TOUCH
// LCD calibration
//
// Calibrate touch screen
//
    GUI_TOUCH_Calibrate(GUI_COORD_X, 0, (__DEMO_TS_WIDTH__ - 1), 0, (__DEMO_TS_WIDTH__ - 1));
    GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, (__DEMO_TS_HEIGHT__-  1), 0, (__DEMO_TS_HEIGHT__-  1));
#endif
}
/*********************************************************************
*
*       LCD_X_DisplayDriver
*
* Purpose:
*   This function is called by the display driver for several purposes.
*   To support the according task the routine needs to be adapted to
*   the display controller. Please note that the commands marked with
*   'optional' are not cogently required and should only be adapted if
*   the display controller supports these features.
*
* Parameter:
*   LayerIndex - Index of layer to be configured
*   Cmd        - Please refer to the details in the switch statement below
*   pData      - Pointer to a LCD_X_DATA structure
*/
int LCD_X_DisplayDriver(unsigned LayerIndex, unsigned Cmd, void * pData)
{
    int r;

    GUI_USE_PARA(LayerIndex);
    GUI_USE_PARA(pData);
    switch (Cmd) {
        //
        // Required
        //
    case LCD_X_INITCONTROLLER: {
        //
        // Called during the initialization process in order to set up the
        // display controller and put it into operation. If the display
        // controller is not initialized by any external routine this needs
        // to be adapted by the customer...
        //
        _InitController();
            //printf("\n************ _InitController \n\n");
        return 0;
    }
    default:
        r = -1;
    }
    return r;
}

/*************************** End of file ****************************/
