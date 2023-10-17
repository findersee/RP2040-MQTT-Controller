/**
 * Copyright (c) 2023 Aki Kuusela
 *
 * <>
 */
#ifndef _DS3234_Driver_H_
#define _DS3234_Driver_H_

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
#define DS3234_Port spi1
#define DS3234_MOSI 11
#define DS3234_MISO 8
#define DS3234_SCK 10
#define DS3234_CS 9

//#define DS3234_MISO 14

#include <stdint.h>

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
typedef struct _TimeFormat
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    
    uint8_t date;
    uint8_t month;
    uint8_t year;

    uint8_t day;
} TimeFormat_t;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */

/*! \brief Set CS pin
 *  \ingroup DS3234_Driver
 *
 *  Set chip select pin of spi0 to low(Active low).
 *
 *  \param none
 */
static inline void DS3234_Select(void);

/*! \brief Set CS pin
 *  \ingroup DS3234_Driver
 *
 *  Set chip select pin of spi0 to high(Active low).
 *
 *  \param none
 */
static inline void DS3234_UnSelect(void);

/*! \brief Initialize DS3234 RTC
 *  \ingroup DS3234_Driver
 *
 *  Set GPIO to spi1.
 *  Puts the SPI into a known state, and enable it.
 *
 *  \param none
 */
void DS3234_init(void);

/*! \brief Read Time from RTC
 *  \ingroup DS3234_Driver
 *
 *  Reads RTC time registers and outpus to structure
 *
 *  \param time RTC Time structure
 */
void DS3234_ReadTime(TimeFormat_t *time);
/*! \brief Write Time to RTC
 *  \ingroup DS3234_Driver
 *
 *  Writes RTC time structure values to RTC
 *  to set time 
 * 
 *  \param time Pointer to the RTC Time structure
 */
void DS3234_WriteTime(TimeFormat_t *time);
/*! \brief Read a single register from RTC
 *  \ingroup DS3234_Driver
 * 
 *  Reads single register defined by reg param
 *  
 *
 *  \param time Pointer to the RTC Time structure
 * 
 * 
 */
uint8_t DS3234_ReadReg(uint8_t reg);
/*! \brief Read multiple registers from RTC
 *  \ingroup DS3234_Driver
 * 
 *  Reads multiple registers defined by reg param
 *  and len
 *  
 *
 *  \param reg Address of first register to read
 *  \param dst Destination pointer where to read
 *  \param len Number of bytes to read
 * 
 *  \return Amount of bytes read
 * 
 */
uint8_t DS3234_ReadRegs(uint8_t reg,uint8_t *dst,uint8_t len);
/*! \brief Write multiple registers to RTC
 *  \ingroup DS3234_Driver
 * 
 *  Writes multiple registers from buffer
 * 
 *  
 *
 *  \param buf Buffer to be written
 *  \param len Number of bytes to read
 * 
 *  \return Amount of bytes written
 * 
 */
uint8_t DS3234_WriteRegs(uint8_t *buf,uint8_t len);


//Time Registers
#define REGSECONDS 0x00
#define REGMINUTES 0x01
#define REGHOURS 0x02
#define REGDAY 0x03
#define REGDATE 0x04
#define REGMONTH 0x05
#define REGYEAR 0x06
//Alarm 1 settings
#define REGAL1SECONDS 0x07
#define REGAL1MINUTES 0x08
#define REGAL1HOUR 0x09
#define REGAL1DAY 0x0A
//Alarm 2 settings
#define REGAL2MINUTES 0x0B
#define REGAL2HOUR 0x0C
#define REGAL2DAY 0x0D
//Control and status registers
#define REGCTRL 0x0E
#define REGSTAT 0x0F
//


#endif