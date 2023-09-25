#include "DS3234_Driver.h"
#include <stdio.h>
#include <FreeRTOS.h>

#include "port_common.h"


static inline void DS3234_Select(void)
{
    gpio_put(DS3234_CS, 0);
}

static inline void DS3234_UnSelect(void)
{
    gpio_put(DS3234_CS, 1);
}

void DS3234_Init(){

    spi_init(DS3234_Port, 2000 * 1000);

    gpio_set_function(DS3234_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(DS3234_MISO, GPIO_FUNC_SPI);
    gpio_set_function(DS3234_SCK, GPIO_FUNC_SPI);

    gpio_init(DS3234_CS);
    gpio_set_dir(DS3234_CS, GPIO_OUT);
    gpio_put(DS3234_CS, 1);
}

void DS3234_ReadTime(TimeFormat *time){

    uint8_t buf[7] = {0};

    DS3234_Select();

    spi_read_blocking(DS3234_Port,REGSECONDS,&buf,7);
    
    DS3234_UnSelect();

    time->seconds = (((buf[0]>>4)*10)+(buf[0]&0x0F));

    time->minutes = (((buf[1]>>4)*10)+(buf[1]&0x0F));

    time->hours = ((((buf[2]&0x30)>>4)*10)+(buf[2]&0x0F));

    time->day = (buf[3]&0x70);

    time->date = (((buf[4]&0x30)>>4)*10)+(buf[4]&0x0F);

    time->month = (((buf[5]&0x10)>>4)*10)+(buf[5]&0x0F);

    time->year = (((buf[6]>>4)*10)+(buf[6]&0x0F));

}

void DS3234_WriteTime(TimeFormat *time){

    uint8_t buf[7] = {0};
    buf[0] = (time->seconds/10)<<4|(time->seconds%10);
    buf[1] = (time->minutes/10)<<4|(time->minutes%10);
    buf[2] = (time->hours/10)<<4|(time->hours%10);
    buf[3] = (time->day);
    buf[4] = (time->date/10)<<4|(time->date%10);
    buf[5] = (time->month/10)<<4|(time->month%10);
    buf[5] = (time->month/10)<<4|(time->month%10);

    DS3234_Select();

    spi_write_blocking(DS3234_Port,REGSECONDS,1);
    spi_write_blocking(DS3234_Port,&buf,7);

    DS3234_UnSelect();
}