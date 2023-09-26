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

    bi_decl(bi_3pins_with_func(DS3234_MISO, DS3234_MOSI, DS3234_SCK, GPIO_FUNC_SPI));

    gpio_init(DS3234_CS);
    gpio_set_dir(DS3234_CS, GPIO_OUT);
    gpio_put(DS3234_CS, 1);

    bi_decl(bi_1pin_with_name(PICO_DEFAULT_SPI_CSN_PIN, "RTC CS"));
}

void DS3234_ReadTime(TimeFormat *time){

    uint8_t buf[7] = {0};

    DS3234_Select();
    //Write Address byte
    spi_write_blocking(DS3234_Port,REGSECONDS,1);    
    //Read bytes out
    spi_read_blocking(DS3234_Port,0,(uint8_t *)&buf,7);
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

    uint8_t buf[8] = {0};
    buf[0] = REGSECONDS|0x80;
    buf[1] = (time->seconds/10)<<4|(time->seconds%10);
    buf[2] = (time->minutes/10)<<4|(time->minutes%10);
    buf[3] = (time->hours/10)<<4|(time->hours%10);
    buf[4] = (time->day);
    buf[5] = (time->date/10)<<4|(time->date%10);
    buf[6] = (time->month/10)<<4|(time->month%10);
    buf[7] = (time->month/10)<<4|(time->month%10);

    DS3234_Select();

    //spi_write_blocking(DS3234_Port,REGSECONDS,1);
    spi_write_blocking(DS3234_Port,(uint8_t *)&buf,8);

    DS3234_UnSelect();
}

uint8_t DS3234_ReadRegs(uint8_t address, uint8_t *dst, uint8_t len){

    uint8_t buf = address;
    DS3234_Select();
    //Write Address byte
    spi_write_blocking(DS3234_Port,&buf,1);
    //Read bytes out
    uint8_t bytes = spi_read_blocking(DS3234_Port,0,dst,len);
    DS3234_UnSelect();

    return bytes;
}

uint8_t DS3234_ReadReg(uint8_t address){
    
    uint8_t data;
    DS3234_ReadRegs(address,(uint8_t *)&data,1);
    return data;
}

uint8_t DS3234_WriteRegs(uint8_t *buf,uint8_t len){

    DS3234_Select();
    //Write bytes
    uint8_t bytes = spi_write_blocking(DS3234_Port,buf,len);
    DS3234_UnSelect();

    return bytes;
}