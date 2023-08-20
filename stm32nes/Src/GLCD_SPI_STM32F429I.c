/*-----------------------------------------------------------------------------
 * Name:    GLCD_SPI_STM32F429I.c
 * Purpose: low level Graphic LCD (320x240 pixels) ILI9341
 *          with SPI interface
 *-----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2010-2014 KEIL - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "stm32f4xx.h"                  /* STM32F4xx Definitions              */
#include "GLCD.h"
#include "Font_6x8_h.h"
#include "Font_16x24_h.h"

/************************** Orientation  configuration ************************/

#ifndef LANDSCAPE
#define LANDSCAPE   1                   /* 1 for landscape, 0 for portrait    */
#endif
#ifndef ROTATE180
#define ROTATE180   0                   /* 1 to rotate the screen for 180 deg */
#endif

/*********************** Hardware specific configuration **********************/

/* SPI Interface: SPI5

   PINS:
   - CS     = PC2  (GPIO pin)
   - WRX    = PD13 (GPIO pin)
   - SCK    = PF7  (SPI5_SCK)
   - SDO    = PF8  (SPI5_MISO)
   - SDI    = PF9  (SPI5_MOSI)                                                */

#define PIN_CS      (1UL << 2)
#define PIN_WRX     (1UL <<13)

/*------------------------- Speed dependant settings -------------------------*/

/* If processor works on high frequency delay has to be increased, it can be
   increased by factor 2^N by this constant                                   */
#define DELAY_2N    14

/*---------------------- Graphic LCD size definitions ------------------------*/

#if (LANDSCAPE == 1)
#define WIDTH       320                 /* Screen Width (in pixels)           */
#define HEIGHT      240                 /* Screen Hight (in pixels)           */
#else
#define WIDTH       240                 /* Screen Width (in pixels)           */
#define HEIGHT      320                 /* Screen Hight (in pixels)           */
#endif
#define BPP         16                  /* Bits per pixel                     */
#define BYPP        ((BPP+7)/8)         /* Bytes per pixel                    */

/*--------------- Graphic LCD interface hardware definitions -----------------*/

/* Pin CS setting to 0 or 1                                                   */
#define LCD_CS(x)    ((x) ? (GPIOC->BSRRL = PIN_CS)  : (GPIOC->BSRRH = PIN_CS) );
#define LCD_WRX(x)   ((x) ? (GPIOD->BSRRL = PIN_WRX) : (GPIOD->BSRRH = PIN_WRX));

#define BG_COLOR  0                     /* Background color                   */
#define TXT_COLOR 1                     /* Text color                         */


/*---------------------------- Global variables ------------------------------*/

/******************************************************************************/
static volatile unsigned short Color[2] = {White, Black};

#if (LANDSCAPE == 0)
static unsigned int  scroll;
#endif

/************************ Local auxiliary functions ***************************/

/*******************************************************************************
* Delay in while loop cycles                                                   *
*   Parameter:    cnt:    number of while cycles to delay                      *
*   Return:                                                                    *
*******************************************************************************/

static void delay (int cnt) {
  cnt <<= DELAY_2N;
  while (cnt--) __NOP();
}


/*******************************************************************************
* Transfer 1 byte over the serial communication                                *
*   Parameter:    byte:   byte to be sent                                      *
*   Return:               byte read while sending                              *
*******************************************************************************/

static __inline unsigned char spi_tran (unsigned char byte) {

  SPI5->DR = byte;
  while (!(SPI5->SR & SPI_SR_RXNE));

  return (SPI5->DR);
}


/*******************************************************************************
* Write a command the LCD controller                                           *
*   Parameter:    cmd:    command to be written                                *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_cmd (unsigned char cmd) {

  LCD_WRX(0);                           /* set WRX to Command                 */
  LCD_CS(0);                            /* set ChipSelect to Active           */
  spi_tran(cmd);                        /* transmit command                   */
  LCD_CS(1);                            /* set ChipSelect to Inactive         */
}


/*******************************************************************************
* Write data to the LCD controller                                             *
*   Parameter:    dat:    data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat (unsigned short dat) {

  LCD_WRX(1);                           /* set WRX to Data                    */
  LCD_CS(0);                            /* set ChipSelect to Active           */
  spi_tran(dat);                        /* transmit data                      */
  LCD_CS(1);                            /* set ChipSelect to Inactive         */
}


/*******************************************************************************
* Start of data writing to the LCD controller                                  *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_start (void) {

  LCD_WRX(1);                           /* set WRX to Data                    */
  LCD_CS(0);                            /* set ChipSelect to Active           */
}


/*******************************************************************************
* Stop of data writing to the LCD controller                                   *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_stop (void) {

  LCD_CS(1);                            /* set ChipSelect to Inactive         */
}


/*******************************************************************************
* Data writing to the LCD controller                                           *
*   Parameter:    dat:    data to be written                                   *
*   Return:                                                                    *
*******************************************************************************/

static __inline void wr_dat_only (unsigned short dat) {

    spi_tran(((dat  ) >> 8) & 0xFF);
    spi_tran(((dat  )     ) & 0xFF);
}


/*******************************************************************************
* Read data from the LCD controller                                            *
*   Parameter:                                                                 *
*   Return:               read data                                            *
*******************************************************************************/
// not yet supported
//static __inline unsigned short rd_dat (void) {
//  unsigned short val = 0;
//
//  LCD_CS(0);
//  spi_tran(0);                                /* Dummy read 1                 */
//  val   = spi_tran(0);                        /* Read D8..D15                 */
//  val <<= 8;
//  val  |= spi_tran(0);                        /* Read D0..D7                  */
//  LCD_CS(1);
//
//  return (val);
//}


/*******************************************************************************
* Write a value to the to LCD register                                         *
*   Parameter:    reg:    register to be written                               *
*                 val:    value to write to the register                       *
*******************************************************************************/

static __inline void wr_reg (unsigned char reg, unsigned short val) {

  wr_cmd(reg);
  wr_dat(val);
}


/*******************************************************************************
* Read from the LCD register                                                   *
*   Parameter:    reg:    register to be read                                  *
*   Return:               value read from the register                         *
*******************************************************************************/
// not yet supported
//static unsigned short rd_reg (unsigned char reg) {

//  wr_cmd(reg);
//  return(rd_dat());
//}


/************************ Exported functions **********************************/

/*******************************************************************************
* Initialize the Graphic LCD controller (ILI9341)                              *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Initialize (void) {

#if (LANDSCAPE == 0)
  scroll = 0;
#endif


/*--------------------------------------------------------------
   Configures GLCD (ILI9341)  I/Os pins

        G2:  PA6      B2:  PD6      R2:  PC10     HSYNC:           PC6
        G3:  PG10     B3:  PG11     R3:  PB0      VSYNC:           PA4
        G4:  PB10     B4:  PG12     R4:  PA11     DE (Enable):     PF10
        G5:  PB11     B5:  PA3      R5:  PA12     CLK:             PG7
        G6:  PC7      B6:  PB8      R6:  PB1      WRX (CMD/DATA):  PD13
        G7:  PD3      B7:  PB9      R7:  PG6      SPI_CS:          PC2
  --------------------------------------------------------------*/

  /* configure WRX-Pin        */
  RCC->AHB1ENR  |= ((RCC_AHB1ENR_GPIODEN) );   /* Enable GPIOD clock          */
  GPIOD->MODER   &= ~(( 3UL << (2*13))  );
  GPIOD->MODER   |=  (( 1UL << (2*13))  );     /* General purpose output mode */
  GPIOD->OSPEEDR &= ~(( 3UL << (2*13))  );
  GPIOD->OSPEEDR |=  (( 2UL << (2*13))  );     /* 50 MHz Fast speed           */
  GPIOD->OTYPER  &= ~(( 1UL << (1*13))  );     /* Output push-pull            */
  GPIOD->PUPDR   &= ~(( 3UL << (2*13))  );     /* No pull-up, pull-down       */

  /* configure ChipSelect-Pin */
  RCC->AHB1ENR  |= ((RCC_AHB1ENR_GPIOCEN) );   /* Enable GPIOC clock          */
  GPIOC->MODER   &= ~(( 3UL << (2* 2))  );
  GPIOC->MODER   |=  (( 1UL << (2* 2))  );     /* General purpose output mode */
  GPIOC->OSPEEDR &= ~(( 3UL << (2* 2))  );
  GPIOC->OSPEEDR |=  (( 2UL << (2* 2))  );     /* 50 MHz Fast speed           */
  GPIOC->OTYPER  &= ~(( 1UL << (1* 2))  );     /* Output push-pull            */
  GPIOC->PUPDR   &= ~(( 3UL << (2* 2))  );     /* No pull-up, pull-down       */

  LCD_CS(1);

  /* configure GLCD pins (set to LOW (not used) */
  RCC->AHB1ENR  |= ((RCC_AHB1ENR_GPIOAEN) |    /* Enable GPIOA clock          */
                    (RCC_AHB1ENR_GPIOBEN) |    /* Enable GPIOA clock          */
                    (RCC_AHB1ENR_GPIOCEN) |    /* Enable GPIOC clock          */
                    (RCC_AHB1ENR_GPIODEN) |    /* Enable GPIOD clock          */
                    (RCC_AHB1ENR_GPIOFEN) |    /* Enable GPIOF clock          */
                    (RCC_AHB1ENR_GPIOGEN)  );  /* Enable GPIOG clock          */

  GPIOA->BSRRH    =  (( 1UL << (1* 3)) |
                      ( 1UL << (1* 4)) |
                      ( 1UL << (1* 6)) |
                      ( 1UL << (1*11)) |
                      ( 1UL << (1*12))  );     /* set Pins to low             */
  GPIOA->MODER   &= ~(( 3UL << (2* 3)) |
                      ( 3UL << (2* 4)) |
                      ( 3UL << (2* 6)) |
                      ( 3UL << (2*11)) |
                      ( 3UL << (2*12))  );
  GPIOA->MODER   |=  (( 1UL << (2* 3)) |
                      ( 1UL << (2* 4)) |
                      ( 1UL << (2* 6)) |
                      ( 1UL << (2*11)) |
                      ( 1UL << (2*12))  );     /* General purpose output mode */
  GPIOA->OSPEEDR &= ~(( 3UL << (2* 3)) |
                      ( 3UL << (2* 4)) |
                      ( 3UL << (2* 6)) |
                      ( 3UL << (2*11)) |
                      ( 3UL << (2*12))  );
  GPIOA->OSPEEDR |=  (( 2UL << (2* 3)) |
                      ( 2UL << (2* 4)) |
                      ( 2UL << (2* 6)) |
                      ( 2UL << (2*11)) |
                      ( 2UL << (2*12))  );     /* 50 MHz Fast speed           */
  GPIOA->OTYPER  &= ~(( 1UL << (1* 3)) |
                      ( 1UL << (1* 4)) |
                      ( 1UL << (1* 6)) |
                      ( 1UL << (1*11)) |
                      ( 1UL << (1*12))  );     /* Output push-pull            */
  GPIOA->PUPDR   &= ~(( 3UL << (2* 3)) |
                      ( 3UL << (2* 4)) |
                      ( 3UL << (2* 6)) |
                      ( 3UL << (2*11)) |
                      ( 3UL << (2*12))  );     /* No pull-up, pull-down       */

  GPIOB->BSRRH    =  (( 1UL << (1* 0)) |
                      ( 1UL << (1* 1)) |
                      ( 1UL << (1* 8)) |
                      ( 1UL << (1* 9)) |
                      ( 1UL << (1*10)) |
                      ( 1UL << (1*11))  );     /* set Pins to low             */
  GPIOB->MODER   &= ~(( 3UL << (2* 0)) |
                      ( 3UL << (2* 1)) |
                      ( 3UL << (2* 8)) |
                      ( 3UL << (2* 9)) |
                      ( 3UL << (2*10)) |
                      ( 3UL << (2*11))  );
  GPIOB->MODER   |=  (( 1UL << (2* 0)) |
                      ( 1UL << (2* 1)) |
                      ( 1UL << (2* 8)) |
                      ( 1UL << (2* 9)) |
                      ( 1UL << (2*10)) |
                      ( 1UL << (2*11))  );     /* General purpose output mode */
  GPIOB->OSPEEDR &= ~(( 3UL << (2* 0)) |
                      ( 3UL << (2* 1)) |
                      ( 3UL << (2* 8)) |
                      ( 3UL << (2* 9)) |
                      ( 3UL << (2*10)) |
                      ( 3UL << (2*11))  );
  GPIOB->OSPEEDR |=  (( 2UL << (2* 0)) |
                      ( 2UL << (2* 1)) |
                      ( 2UL << (2* 8)) |
                      ( 2UL << (2* 9)) |
                      ( 2UL << (2*10)) |
                      ( 2UL << (2*11))  );     /* 50 MHz Fast speed           */
  GPIOB->OTYPER  &= ~(( 1UL << (1* 0)) |
                      ( 1UL << (1* 1)) |
                      ( 1UL << (1* 8)) |
                      ( 1UL << (1* 9)) |
                      ( 1UL << (1*10)) |
                      ( 1UL << (1*11))  );     /* Output push-pull            */
  GPIOB->PUPDR   &= ~(( 3UL << (2* 0)) |
                      ( 3UL << (2* 1)) |
                      ( 3UL << (2* 8)) |
                      ( 3UL << (2* 9)) |
                      ( 3UL << (2*10)) |
                      ( 3UL << (2*11))  );     /* No pull-up, pull-down       */

  GPIOC->BSRRH    =  (( 1UL << (1* 6)) |
                      ( 1UL << (1* 7)) |
                      ( 1UL << (1*10))  );     /* set Pins to low             */
  GPIOC->MODER   &= ~(( 3UL << (2* 6)) |
                      ( 3UL << (2* 7)) |
                      ( 3UL << (2*10))  );
  GPIOC->MODER   |=  (( 1UL << (2* 6)) |
                      ( 1UL << (2* 7)) |
                      ( 1UL << (2*10))  );     /* General purpose output mode */
  GPIOC->OSPEEDR &= ~(( 3UL << (2* 6)) |
                      ( 3UL << (2* 7)) |
                      ( 3UL << (2*10))  );
  GPIOC->OSPEEDR |=  (( 2UL << (2* 6)) |
                      ( 2UL << (2* 7)) |
                      ( 2UL << (2*10))  );     /* 50 MHz Fast speed           */
  GPIOC->OTYPER  &= ~(( 1UL << (1* 6)) |
                      ( 1UL << (1* 7)) |
                      ( 1UL << (1*10))  );     /* Output push-pull            */
  GPIOC->PUPDR   &= ~(( 3UL << (2* 6)) |
                      ( 3UL << (2* 7)) |
                      ( 3UL << (2*10))  );     /* No pull-up, pull-down       */

  GPIOD->BSRRH    =  (( 1UL << (1* 3)) |
                      ( 1UL << (1* 6))  );     /* set Pins to low             */
  GPIOD->MODER   &= ~(( 3UL << (2* 3)) |
                      ( 3UL << (2* 6))  );
  GPIOD->MODER   |=  (( 1UL << (2* 3)) |
                      ( 1UL << (2* 6))  );     /* General purpose output mode */
  GPIOD->OSPEEDR &= ~(( 3UL << (2* 3)) |
                      ( 3UL << (2* 6))  );
  GPIOD->OSPEEDR |=  (( 2UL << (2* 3)) |
                      ( 2UL << (2* 6))  );     /* 50 MHz Fast speed           */
  GPIOD->OTYPER  &= ~(( 1UL << (1* 3)) |
                      ( 1UL << (1* 6))  );     /* Output push-pull            */
  GPIOD->PUPDR   &= ~(( 3UL << (2* 3)) |
                      ( 3UL << (2* 6))  );     /* No pull-up, pull-down       */

  GPIOF->BSRRH    =  (( 1UL << (1*10))  );     /* set Pins to low             */
  GPIOF->MODER   &= ~(( 3UL << (2*10))  );
  GPIOF->MODER   |=  (( 1UL << (2*10))  );     /* General purpose output mode */
  GPIOF->OSPEEDR &= ~(( 3UL << (2*10))  );
  GPIOF->OSPEEDR |=  (( 2UL << (2*10))  );     /* 50 MHz Fast speed           */
  GPIOF->OTYPER  &= ~(( 1UL << (1*10))  );     /* Output push-pull            */
  GPIOF->PUPDR   &= ~(( 3UL << (2*10))  );     /* No pull-up, pull-down       */

  GPIOG->BSRRH    =  (( 1UL << (1* 6)) |
                      ( 1UL << (1* 7)) |
                      ( 1UL << (1*10)) |
                      ( 1UL << (1*11)) |
                      ( 1UL << (1*12))  );     /* set Pins to low             */
  GPIOG->MODER   &= ~(( 3UL << (2* 6)) |
                      ( 3UL << (2* 7)) |
                      ( 3UL << (2*10)) |
                      ( 3UL << (2*11)) |
                      ( 3UL << (2*12))  );
  GPIOG->MODER   |=  (( 1UL << (2* 6)) |
                      ( 1UL << (2* 7)) |
                      ( 1UL << (2*10)) |
                      ( 1UL << (2*11)) |
                      ( 1UL << (2*12))  );     /* General purpose output mode */
  GPIOG->OSPEEDR &= ~(( 3UL << (2* 6)) |
                      ( 3UL << (2* 7)) |
                      ( 3UL << (2*10)) |
                      ( 3UL << (2*11)) |
                      ( 3UL << (2*12))  );
  GPIOG->OSPEEDR |=  (( 2UL << (2* 6)) |
                      ( 2UL << (2* 7)) |
                      ( 2UL << (2*10)) |
                      ( 2UL << (2*11)) |
                      ( 2UL << (2*12))  );     /* 50 MHz Fast speed           */
  GPIOG->OTYPER  &= ~(( 1UL << (1* 6)) |
                      ( 1UL << (1* 7)) |
                      ( 1UL << (1*10)) |
                      ( 1UL << (1*11)) |
                      ( 1UL << (1*12))  );     /* Output push-pull            */
  GPIOG->PUPDR   &= ~(( 3UL << (2* 6)) |
                      ( 3UL << (2* 7)) |
                      ( 3UL << (2*10)) |
                      ( 3UL << (2*11)) |
                      ( 3UL << (2*12))  );     /* No pull-up, pull-down       */




  // Configure SPI  pins
  RCC->AHB1ENR  |= ((RCC_AHB1ENR_GPIOFEN)  );  /* Enable GPIOF clock          */

  GPIOF->AFR[0]  &= ~((15UL << (4* 7)) );
  GPIOF->AFR[0]  |=  (( 5UL << (4* 7)) );      /* Alternate Function mode AF5 */
  GPIOF->AFR[1]  &= ~((15UL << (4* 0)) |
                      (15UL << (4* 1))  );
  GPIOF->AFR[1]  |=  (( 5UL << (4* 0)) |
                      ( 5UL << (4* 1))  );     /* Alternate Function mode AF5 */
  GPIOF->MODER   &= ~(( 3UL << (2* 7)) |
                      ( 3UL << (2* 8)) |
                      ( 3UL << (2* 9))  );
  GPIOF->MODER   |=  (( 2UL << (2* 7)) |
                      ( 2UL << (2* 8)) |
                      ( 2UL << (2* 9))  );     /* Alternate Function mode     */
  GPIOF->OSPEEDR &= ~(( 3UL << (2* 7)) |
                      ( 3UL << (2* 8)) |
                      ( 3UL << (2* 9))  );
  GPIOF->OSPEEDR |=  (( 3UL << (2* 7)) |
                      ( 3UL << (2* 8)) |
                      ( 3UL << (2* 9))  );     /* 100 MHz Fast speed          */
  GPIOF->OTYPER  &= ~(( 1UL << (1* 7)) |
                      ( 1UL << (1* 8)) |
                      ( 1UL << (1* 9))  );     /* Output push-pull            */
  GPIOF->PUPDR   &= ~(( 3UL << (2* 7)) |
                      ( 3UL << (2* 8)) |
                      ( 3UL << (2* 9))  );     /* No pull-up, pull-down       */

  // Configure SPI5
  RCC->APB2ENR |=  RCC_APB2ENR_SPI5EN;         /* Enable SPI Clock            */

  RCC->APB2RSTR |=  RCC_APB2RSTR_SPI5RST;      /* Reset SPI Peripheral        */
  __NOP(); __NOP(); __NOP(); __NOP();
  RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI5RST;

  SPI5->CR1  = (( 0UL <<  15) |          /* 2-line unidirectional data mode   */
	              ( 0UL <<  13) |          /* CRC calculation disabled          */
	              ( 0UL <<  11) |          /* 8-bit data frame format           */
	              ( 1UL <<   9) |          /* Software slave management enabled */
	              ( 1UL <<   8) |          /* Internal slave select             */
	              ( 0UL <<   7) |          /* MSB transmitted first             */
	              ( 2UL <<   3) |          /* Baud rate  5MHz @ 42MHz PCLK1     */
	              ( 1UL <<   2) |          /* Master configuration              */
	              ( 0UL <<   1) |          /* Clock polarity 0                  */
                ( 0UL <<   0)  );        /* Clock phase 0                     */
  SPI5->CR1 |=  SPI_CR1_SPE;             /* Enable SPI                        */

  wr_cmd(0x01);  // SW Reset
  delay(10);
  wr_cmd(0x28);  // Display OFF

  wr_cmd (0xCF); // Power control B (CFh)
  wr_dat(0x00);
  wr_dat(0xC1);
  wr_dat(0x30);

  wr_cmd (0xED); // Power on sequence control (EDh)
  wr_dat(0x64);
  wr_dat(0x03);
  wr_dat(0x12);
  wr_dat(0x81);

  wr_cmd (0xE8); // Driver timing control A (E8h)
  wr_dat(0x85);
  wr_dat(0x00);
  wr_dat(0x78);

  wr_cmd (0xCB); // Power control A (CBh)
  wr_dat(0x39);
  wr_dat(0x2C);
  wr_dat(0x00);
  wr_dat(0x34);
  wr_dat(0x02);

  wr_cmd (0xF7); // Pump ratio control (F7h)
  wr_dat(0x20);

  wr_cmd (0xEA); // Driver timing control B (EAh)
  wr_dat(0x00);
  wr_dat(0x00);

  wr_cmd (0xC0); // Power Control 1 (C0h)
  wr_dat(0x10);

  wr_cmd (0xC1); // Power Control 2 (C1h)
  wr_dat(0x10);

  wr_cmd (0xC5); // VCOM Control 1(C5h)
  wr_dat(0x45);
  wr_dat(0x15);

  wr_cmd (0xC7); // VCOM Control 2(C7h)
  wr_dat(0x90);

  wr_cmd (0x36); // Memory Access Control (36h)
#if (LANDSCAPE == 1)
  wr_dat(0x68);
#else
  wr_dat(0xC8);
#endif

  wr_cmd (0x3A); // Pixel Format Set (3Ah)
  wr_dat(0x55);

  wr_cmd (0xB1); // Frame Rate Control (B1h)
  wr_dat(0x00);
  wr_dat(0x1B);

  wr_cmd (0xF2); // Enable 3G (F2h)
  wr_dat(0x00);

  wr_cmd (0x26); // Gamma Set (26h)
  wr_dat(0x01);

  wr_cmd (0xE0); // Positive Gamma Correction (E0h)
  wr_dat(0x0F);
  wr_dat(0x29);
  wr_dat(0x24);
  wr_dat(0x0C);
  wr_dat(0x0E);
  wr_dat(0x09);
  wr_dat(0x4E);
  wr_dat(0x78);
  wr_dat(0x3C);
  wr_dat(0x09);
  wr_dat(0x13);
  wr_dat(0x05);
  wr_dat(0x17);
  wr_dat(0x11);
  wr_dat(0x00);

  wr_cmd (0xE1); // Negative Gamma Correction (E1h)
  wr_dat(0x00);
  wr_dat(0x16);
  wr_dat(0x1B);
  wr_dat(0x04);
  wr_dat(0x11);
  wr_dat(0x07);
  wr_dat(0x31);
  wr_dat(0x33);
  wr_dat(0x42);
  wr_dat(0x05);
  wr_dat(0x0C);
  wr_dat(0x0A);
  wr_dat(0x28);
  wr_dat(0x2F);
  wr_dat(0x0F);

  wr_cmd (0x2A); // Column Address Set (2Ah)
  wr_dat(0x00);
  wr_dat(0x00);
  wr_dat(0x00);
  wr_dat(0xEF);

  wr_cmd (0x2B); // Page Address Set (2Bh)
  wr_dat(0x00);
  wr_dat(0x00);
  wr_dat(0x01);
  wr_dat(0x3F);

  wr_cmd(0xB7);  // Entry Mode Set (B7h)
  wr_dat(0x07);

  wr_cmd (0xB6); // Display Function Control (B6h)
  wr_dat(0x0A);
  wr_dat(0xA7);
  wr_dat(0x27);
  wr_dat(0x04);

  wr_cmd (0x11); // Sleep Out (11h)
  delay(200);
  wr_cmd (0x29); // Display ON (29h)
  delay(200);

  wr_cmd (0x2C); // Memory Write (2Ch)

}


/*******************************************************************************
* Set draw window region                                                       *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        window width in pixel                            *
*                   h:        window height in pixels                          *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetWindow (unsigned int x, unsigned int y, unsigned int w, unsigned int h) {

  wr_cmd (0x2A); // Column Address Set (2Ah)
  wr_dat(((x    ) >> 8) & 0xFF);
  wr_dat(((x    )     ) & 0xFF);
  wr_dat(((x+w-1) >> 8) & 0xFF);
  wr_dat(((x+w-1)     ) & 0xFF);

  wr_cmd (0x2B); // Page Address Set (2Bh)
  wr_dat(((y    ) >> 8) & 0xFF);
  wr_dat(((y    )     ) & 0xFF);
  wr_dat(((y+h-1) >> 8) & 0xFF);
  wr_dat(((y+h-1)     ) & 0xFF);

  wr_cmd (0x2C); // Memory Write (2Ch)
}


/*******************************************************************************
* Set draw window region to whole screen                                       *
*   Parameter:                                                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_WindowMax (void) {
  GLCD_SetWindow (0, 0, WIDTH, HEIGHT);
}


/*******************************************************************************
* Draw a pixel in foreground color                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_PutPixel (unsigned int x, unsigned int y) {

  GLCD_SetWindow(x, y, 1, 1);
  wr_cmd(0x3C); // Write Memory Continue (3Ch)
  wr_dat_start();
  wr_dat_only(Color[TXT_COLOR]);
  wr_dat_stop();
}


/*******************************************************************************
* Set foreground color                                                         *
*   Parameter:      color:    foreground color                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetTextColor (unsigned short color) {

  Color[TXT_COLOR] = color;
}


/*******************************************************************************
* Set background color                                                         *
*   Parameter:      color:    background color                                 *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_SetBackColor (unsigned short color) {

  Color[BG_COLOR] = color;
}


/*******************************************************************************
* Clear display                                                                *
*   Parameter:      color:    display clearing color                           *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Clear (unsigned short color) {
  unsigned int i;

  GLCD_WindowMax();
  wr_cmd(0x3C); // Write Memory Continue (3Ch)
  wr_dat_start();
  for(i = 0; i < (WIDTH*HEIGHT); i++) {
    wr_dat_only(color);
	}
  wr_dat_stop();
}


/*******************************************************************************
* Draw character on given position                                             *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   cw:       character width in pixel                         *
*                   ch:       character height in pixels                       *
*                   c:        pointer to character bitmap                      *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DrawChar (unsigned int x, unsigned int y, unsigned int cw, unsigned int ch, unsigned char *c) {
  unsigned int i, j, k, pixs;

#if (LANDSCAPE == 0)
  y = (y + scroll) % HEIGHT;
#endif

  GLCD_SetWindow(x, y, cw, ch);
  wr_cmd(0x3C); // Write Memory Continue (3Ch)
  wr_dat_start();

  k  = (cw + 7)/8;

  if (k == 1) {
    for (j = 0; j < ch; j++) {
      pixs = *(unsigned char  *)c;
      c += 1;

      for (i = 0; i < cw; i++) {
        wr_dat_only (Color[(pixs >> i) & 1]);
      }
    }
  }
  else if (k == 2) {
    for (j = 0; j < ch; j++) {
      pixs = *(unsigned short *)c;
      c += 2;

      for (i = 0; i < cw; i++) {
        wr_dat_only (Color[(pixs >> i) & 1]);
      }
    }
  }
  wr_dat_stop();
}


/*******************************************************************************
* Disply character on given line                                               *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   fi:       font index (0 = 6x8, 1 = 16x24)                  *
*                   c:        ascii character                                  *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DisplayChar (unsigned int ln, unsigned int col, unsigned char fi, unsigned char c) {

  c -= 32;
  switch (fi) {
    case 0:  /* Font 6 x 8 */
      GLCD_DrawChar(col *  6, ln *  8,  6,  8, (unsigned char *)&Font_6x8_h  [c * 8]);
      break;
    case 1:  /* Font 16 x 24 */
      GLCD_DrawChar(col * 16, ln * 24, 16, 24, (unsigned char *)&Font_16x24_h[c * 24]);
      break;
  }
}


/*******************************************************************************
* Disply string on given line                                                  *
*   Parameter:      ln:       line number                                      *
*                   col:      column number                                    *
*                   fi:       font index (0 = 6x8, 1 = 16x24)                  *
*                   s:        pointer to string                                *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_DisplayString (unsigned int ln, unsigned int col, unsigned char fi, char *s) {

  while (*s) {
    GLCD_DisplayChar(ln, col++, fi, *s++);
  }
}


/*******************************************************************************
* Clear given line                                                             *
*   Parameter:      ln:       line number                                      *
*                   fi:       font index (0 = 6x8, 1 = 16x24)                  *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_ClearLn (unsigned int ln, unsigned char fi) {
  unsigned char i;
  char buf[60];

  GLCD_WindowMax();
  switch (fi) {
    case 0:  /* Font 6 x 8 */
      for (i = 0; i < (WIDTH+5)/6; i++)
        buf[i] = ' ';
      buf[i+1] = 0;
      break;
    case 1:  /* Font 16 x 24 */
      for (i = 0; i < (WIDTH+15)/16; i++)
        buf[i] = ' ';
      buf[i+1] = 0;
      break;
  }
  GLCD_DisplayString (ln, 0, fi, buf);
}

/*******************************************************************************
* Draw bargraph                                                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        maximum width of bargraph (in pixels)            *
*                   h:        bargraph height                                  *
*                   val:      value of active bargraph (in 1/1024)             *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bargraph (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int val) {
  int i,j;

  val = (val * w) >> 10;                /* Scale value                        */

  GLCD_SetWindow(x, y, w, h);
  wr_cmd(0x3C); // Write Memory Continue (3Ch)
  wr_dat_start();
  for (i = 0; i < h; i++) {
    for (j = 0; j <= w-1; j++) {
      if(j >= val) {
        wr_dat_only(Color[BG_COLOR]);
      } else {
        wr_dat_only(Color[TXT_COLOR]);
      }
    }
  }
  wr_dat_stop();
}


/*******************************************************************************
* Display graphical bitmap image at position x horizontally and y vertically   *
* (This function is optimized for 16 bits per pixel format, it has to be       *
*  adapted for any other bits per pixel format)                                *
*   Parameter:      x:        horizontal position                              *
*                   y:        vertical position                                *
*                   w:        width of bitmap                                  *
*                   h:        height of bitmap                                 *
*                   bitmap:   address at which the bitmap data resides         *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_Bitmap (unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned char *bitmap) {
  int i, j;
  unsigned short *bitmap_ptr = (unsigned short *)bitmap;

  GLCD_SetWindow (x, y, w, h);
  wr_cmd(0x3C); // Write Memory Continue (3Ch)
  wr_dat_start();
  for (i = (h-1)*w; i > -1; i -= w) {
    for (j = 0; j < w; j++) {
      wr_dat_only (bitmap_ptr[i+j]);
    }
  }
  wr_dat_stop();
}



/*******************************************************************************
* Scroll content of the whole display for dy pixels vertically                 *
*   Parameter:      dy:       number of pixels for vertical scroll             *
*   Return:                                                                    *
*******************************************************************************/

void GLCD_ScrollVertical (unsigned int dy) {

  // not yet supported
}


/*******************************************************************************
* Write a command to the LCD controller                                        *
*   Parameter:      cmd:      command to write to the LCD                      *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_WrCmd (unsigned char cmd) {
  wr_cmd (cmd);
}


/*******************************************************************************
* Write a value into LCD controller register                                   *
*   Parameter:      reg:      lcd register address                             *
*                   val:      value to write into reg                          *
*   Return:                                                                    *
*******************************************************************************/
void GLCD_WrReg (unsigned char reg, unsigned short val) {
  wr_reg (reg, val);
}
/******************************************************************************/
