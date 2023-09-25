
#ifndef _DS3234_Driver_H_
#define _DS3234_Driver_H_

#define DS3234_Port spi1
#define DS3234_MOSI 11
#define DS3234_MISO 14
#define DS3234_SCK 13
#define DS3234_CS 12

#define DS3234_MISO 14

#include <stdint.h>


typedef struct _TimeFormat
{
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    
    uint8_t date;
    uint8_t month;
    uint8_t year;

    uint8_t day;


} TimeFormat;


static inline void DS3234_Select(void);

static inline void DS3234_UnSelect(void);


void DS3234_init(void);
void DS3234_ReadTime(TimeFormat *time);
void DS3234_WriteTime(TimeFormat *time);


//Time settings
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