/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "port_common.h"

#include "wizchip_conf.h"
#include "w5x00_spi.h"

#include "mqtt_interface.h"
#include "MQTTClient.h"

#include "timer.h"

#include "DS3234_Driver.h"

#include "mjson.h"

//#include "RP2040-MQTT.h"



/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
#define MQTT_TASK_STACK_SIZE 2048
#define MQTT_TASK_PRIORITY 10

#define YIELD_TASK_STACK_SIZE 512
#define YIELD_TASK_PRIORITY 8

#define UPTIME_TASK_STACK_SIZE 32
#define UPTIME_TASK_PRIORITY 1

#define RELAY_CONTROL_TASK_STACK_SIZE 512
#define RELAY_CONTROL_TASK_PRIORITY 6


/* Clock */
#define PLL_SYS_KHZ (125 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_MQTT 0

/* Port */
#define PORT_MQTT 1883

/* Timeout */
#define DEFAULT_TIMEOUT 1000 // 1 second

/* MQTT */
#define MQTT_CLIENT_ID "rpi-pico"
#define MQTT_USERNAME "RP2040"
#define MQTT_PASSWORD "0123456789"
#define MQTT_PUBLISH_TOPIC "RP2040_MQTT"
#define MQTT_RELAY_SUBSCRIBE_TOPIC "RP2040_MQTT_SET"
//#define MQTT_PUBLISH_PAYLOAD "Hello, World!"
#define MQTT_PUBLISH_PERIOD (1000 * 60) // 60 seconds
#define MQTT_TIME_SUBSCRIBE_TOPIC "RP2040_MQTT_TIME"
#define MQTT_KEEP_ALIVE 10 // 10 milliseconds


#define RTC_Interrupt 14
#define RELAY1 7
#define RELAY2 6
#define RELAY3 5


/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = {0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}, // MAC address
        .ip = {192, 168, 2, 11},                     // IP address
        .sn = {255, 255, 255, 0},                    // Subnet Mask
        .gw = {192, 168, 2, 66},                     // Gateway
        .dns = {8, 8, 8, 8},                         // DNS server
        .dhcp = NETINFO_STATIC                       // DHCP enable/disable
};

/* MQTT */
static uint8_t g_mqtt_send_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_mqtt_recv_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_mqtt_broker_ip[4] = {192, 168, 2, 66};
static Network g_mqtt_network;
//static Network g_sntp_network;
static MQTTClient g_mqtt_client;
static MQTTPacket_connectData g_mqtt_packet_connect_data = MQTTPacket_connectData_initializer;
static MQTTMessage g_mqtt_message;
static uint8_t g_mqtt_connect_flag = 0;

SemaphoreHandle_t AlarmInterruptSemaphore;


/* Timer  */
static volatile uint32_t g_msec_cnt = 0;

typedef struct _RelaySet
{
    uint8_t Relay_Set;
    uint8_t Time_Hour;
    uint8_t Time_Minute;
} RelaySet_t;

TimeFormat_t RTC_Time;

static volatile uint32_t uptime = 0;

static int JsonHours[24];
static int JsonMinutes[24];
static int JsonRelays[24];

static int TimeHours;
static int TimeMinutes;
static int TimeSeconds;
static int TimeYear;
static int TimeMonth;
static int TimeDate;
static int TimeDay;


int8_t RelaysCurrent[24];
int8_t RelaysNext[24];

static int  JSON_RELAY_SET;
static int  JSON_Count;   

const struct json_attr_t relays_attrs[] = {
    {"HOUR",	t_integer, .addr.integer = JsonHours},
    {"MINUTE",	t_integer, .addr.integer = JsonMinutes},
    {"RELAYS",	t_integer, .addr.integer = JsonRelays},
    {NULL},
};

static const struct json_attr_t time_attrs[] = {
    {"HOURS",	t_integer, .addr.integer = &TimeHours},
    {"MINUTES",	t_integer, .addr.integer = &TimeMinutes},
    {"SECONDS",	t_integer, .addr.integer = &TimeSeconds},
    {"YEAR",	t_integer, .addr.integer = &TimeYear},
    {"MONTH",	t_integer, .addr.integer = &TimeMonth},
    {"DATE",	t_integer, .addr.integer = &TimeDate},
    {"DAY",	t_integer, .addr.integer = &TimeDay},
    {NULL},
};


static const struct json_attr_t json_relay_set_attrs[] = {
    {"RelaySet", t_integer, .addr.integer = &JSON_RELAY_SET},
    {"Settings",   t_array, .addr.array.element_type = t_object,
		   	      .addr.array.arr.objects.subtype= relays_attrs,
			      .addr.array.maxlen = 24,
                  .addr.array.count = &JSON_Count},
    {NULL},
};


static uint32_t DevSerial;


/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
void mqtt_task(void *argument);
void SNTP_task(void *argument);
void RelayControl_task(void *argument);
void yield_task(void *argument);
void uptime_task(void *argument);

/* Clock */
static void set_clock_khz(void);

/* MQTT */
static void Relay_message_arrived(MessageData *msg_data);
static void Time_message_arrived(MessageData *msg_data);


/* Timer  */
static void repeating_timer_callback(void);

/* RTC Interrupt*/
void RTC_interrupt_callback(char *buf,uint32_t events);


uint32_t macCalc(uint8_t *buf);

/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */
int main(){
    /* Initialize */
    set_clock_khz();

   // sleep_ms(5000);


    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    timer_hw->dbgpause = 0;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
    stdio_init_all();

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    //sleep_ms(10000);
    
    uint8_t Serial[8];
    flash_get_unique_id((uint8_t *)&Serial);

    DevSerial = macCalc(&Serial);

    g_net_info.mac[5] = Serial[0];
    g_net_info.mac[4] = Serial[2];
    g_net_info.mac[3] = Serial[4];

    printf("Startup\n");
    DS3234_init();

    gpio_init(RELAY1);
    gpio_set_dir(RELAY1, GPIO_OUT);
    gpio_put(RELAY1, 0);

    gpio_init(RELAY2);
    gpio_set_dir(RELAY2, GPIO_OUT);
    gpio_put(RELAY2, 0);

    gpio_init(RELAY3);
    gpio_set_dir(RELAY3, GPIO_OUT);
    gpio_put(RELAY3, 0);    

    //Set internal pull-up on RTC interrupt pin
    gpio_set_pulls(RTC_Interrupt,true,false);



    DS3234_ReadTime(&RTC_Time);

    printf("Time is %d:%02d:%02d \n",RTC_Time.hours,RTC_Time.minutes,RTC_Time.seconds);

    //DS3234_ReadTime(&RTC_Time);

    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    wizchip_1ms_timer_initialize(repeating_timer_callback);


    AlarmInterruptSemaphore = xSemaphoreCreateBinary();

    xTaskCreate(mqtt_task, "MQTT_Task", MQTT_TASK_STACK_SIZE, NULL, MQTT_TASK_PRIORITY, NULL);
    xTaskCreate(yield_task, "YIELD_Task", YIELD_TASK_STACK_SIZE, NULL, YIELD_TASK_PRIORITY, NULL);
    xTaskCreate(uptime_task, "UPTIME_Task", UPTIME_TASK_STACK_SIZE, NULL, UPTIME_TASK_PRIORITY, NULL);
    xTaskCreate(RelayControl_task, "RELAY_TASK",RELAY_CONTROL_TASK_STACK_SIZE,NULL,RELAY_CONTROL_TASK_PRIORITY,NULL);
    vTaskStartScheduler();

    while (1)
    {
        ;
    }
}




/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Task */
void mqtt_task(void *argument){
    uint8_t retval;

    network_initialize(g_net_info);

    /* Get network information */
    print_network_information(g_net_info);
    uint8_t failCnt = 1;
    NewNetwork(&g_mqtt_network, SOCKET_MQTT);
    while (1){
        retval = ConnectNetwork(&g_mqtt_network, g_mqtt_broker_ip, PORT_MQTT);

        if (retval != 1)
        {
            printf(" Network connect failed, fail count: %d\n",failCnt);
            vTaskDelay(10 * 1000);
            failCnt ++;
            if(failCnt > 11){
                watchdog_reboot(0,0,0);    
            }
        }
        if (retval){
            printf(" Network connected\n");
            break;
        }
           

    }

    /* Initialize MQTT client */
    MQTTClientInit(&g_mqtt_client, &g_mqtt_network, DEFAULT_TIMEOUT, g_mqtt_send_buf, ETHERNET_BUF_MAX_SIZE, g_mqtt_recv_buf, ETHERNET_BUF_MAX_SIZE);

    /* Connect to the MQTT broker */
    g_mqtt_packet_connect_data.MQTTVersion = 3;
    g_mqtt_packet_connect_data.cleansession = 1;
    g_mqtt_packet_connect_data.willFlag = 0;
    g_mqtt_packet_connect_data.keepAliveInterval = MQTT_KEEP_ALIVE;
    g_mqtt_packet_connect_data.clientID.cstring = MQTT_CLIENT_ID;
    g_mqtt_packet_connect_data.username.cstring = MQTT_USERNAME;
    g_mqtt_packet_connect_data.password.cstring = MQTT_PASSWORD;

    retval = MQTTConnect(&g_mqtt_client, &g_mqtt_packet_connect_data);

    if (retval < 0)
    {
        printf(" MQTT connect failed : %d\n", retval);

        while (1)
        {
            watchdog_reboot(0,0,0); 

        }
    }

    printf(" MQTT connected\n");

    /* Configure publish message */
    g_mqtt_message.qos = QOS0;
    g_mqtt_message.retained = 0;
    g_mqtt_message.dup = 0;

    /* Subscribe */
    retval = MQTTSubscribe(&g_mqtt_client, MQTT_RELAY_SUBSCRIBE_TOPIC, QOS1, Relay_message_arrived);

    if (retval < 0)
    {
        printf(" Relay Subscribe failed : %d\n", retval);

        while (1)
        {
            watchdog_reboot(0,0,0);
        }
    }

    printf(" Subscribed Relay messages\n");

    retval = MQTTSubscribe(&g_mqtt_client, MQTT_TIME_SUBSCRIBE_TOPIC, QOS1, Time_message_arrived);

    if (retval < 0)
    {
        printf(" Time Subscribe failed : %d\n", retval);

        while (1)
        {
            watchdog_reboot(0,0,0);
        }
    }

    printf(" Subscribed Time messages\n");

    g_mqtt_connect_flag = 1;

    while (1)
    {
        uint16_t raw = adc_read();
        const float conversion_factor = 3.3f / (1<<12);
        float result = raw * conversion_factor;
        float temp = 27 - (result -0.706)/0.001721;

        char PayloadBuf[40];
        uint8_t PayloadLen = sprintf((char *)&PayloadBuf,"{\"UPTIME\":%u,\"TEMP\":%.2f}",uptime,temp);

        g_mqtt_message.payload = &PayloadBuf;
        g_mqtt_message.payloadlen = PayloadLen;

        /* Publish */
        retval = MQTTPublish(&g_mqtt_client, MQTT_PUBLISH_TOPIC, &g_mqtt_message);

        if (retval < 0)
        {
            printf(" Publish failed : %d\n", retval);
            /*
            while (1)
            {
                vTaskDelay(1000 * 1000);
            }

            */
        }

        //printf(" Published\n");

        vTaskDelay(MQTT_PUBLISH_PERIOD);
    }
}
/* NTP time task */
void SNTP_task(void *argument){



    
}

/* Relay control task */
void RelayControl_task(void *argument){

    TimeFormat_t RelayTime;

    //Set Alarm 1 to activate when minutes and seconds are 0 
    uint8_t spiBuf[10] = {0x87,0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x1D,0xC8};

    DS3234_WriteRegs((uint8_t *)&spiBuf,sizeof(spiBuf));

    //Activate interrupt after alarm has been initialized
    gpio_set_irq_enabled_with_callback(RTC_Interrupt, GPIO_IRQ_EDGE_FALL, true, (gpio_irq_callback_t)&RTC_interrupt_callback);    

    while (1){
        memset(&RelayTime,0,sizeof RelayTime);
        xSemaphoreTake(AlarmInterruptSemaphore, portMAX_DELAY);
        printf("Semaphore Taken \n");

        spiBuf[0] = 0x8F;
        spiBuf[1] = 0xC8;
        DS3234_WriteRegs((uint8_t *)&spiBuf,2);
        gpio_put(PICO_DEFAULT_LED_PIN, true);// led for alarm

        DS3234_ReadTime(&RelayTime);

        if((RelaysCurrent[RelayTime.hours]&0x01) == 0x01){
            gpio_put(RELAY1,(bool)(RelaysCurrent[RelayTime.hours]&0x02));
            gpio_put(RELAY2,(bool)(RelaysCurrent[RelayTime.hours]&0x04));
            gpio_put(RELAY3,(bool)(RelaysCurrent[RelayTime.hours]&0x08));
        }
        if(RelayTime.hours == 23)
            memcpy(&RelaysCurrent,&RelaysNext,sizeof(RelaysNext));
    }
}

/* Uptime task */
void uptime_task(void *argument){

    //bool ledState = false;
    //watchdog_enable(1200);
    while (1){

        uptime++;
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(200);
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        watchdog_update();
        vTaskDelay(800);
    }
}


void yield_task(void *argument){
    int retval;

    while (1)
    {
        if (g_mqtt_connect_flag == 1)
        {
            if ((retval = MQTTYield(&g_mqtt_client, g_mqtt_packet_connect_data.keepAliveInterval)) < 0)
            {
                printf(" Yield error : %d\n", retval);
                watchdog_reboot(0,0,0); 
                while (1)
                {
                    vTaskDelay(1000 * 1000);
                }
            }
        }

        vTaskDelay(10);
    }
}

/* Clock */
static void set_clock_khz(void){
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );

    // Enable 25 MHz clock output on GPIO21
    clock_gpio_init(23, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 5);

}

/* MQTT */
static void Relay_message_arrived(MessageData *msg_data){
    MQTTMessage *message = msg_data->message;

    json_read_object((const char *)message->payload,json_relay_set_attrs,NULL);


    switch (JSON_RELAY_SET)
    {
    case 0:
        for(uint8_t i=0;i < JSON_Count;i++){
            RelaysCurrent[JsonHours[i]] = JsonRelays[i];
        }
        break;
    case 1:
        for(uint8_t i=0;i < JSON_Count;i++){
            RelaysNext[JsonHours[i]] = JsonRelays[i];
        }
        break;
    default:
        break;
    }

    printf("%.*s \n", (uint32_t)message->payloadlen, (uint8_t *)message->payload);
}

static void Time_message_arrived(MessageData *msg_data){
    TimeFormat_t SetTime;
    MQTTMessage *message = msg_data->message;
    json_read_object((const char *)message->payload,time_attrs,NULL);

    SetTime.hours = TimeHours;
    SetTime.minutes = TimeMinutes;
    SetTime.seconds = TimeSeconds;
    SetTime.year = TimeYear;
    SetTime.month = TimeMonth;
    SetTime.date = TimeDate;
    SetTime.day = TimeDay;


    DS3234_WriteTime(&SetTime);

    memset(&SetTime,0,sizeof(SetTime));

    DS3234_ReadTime(&SetTime);

    printf("Time is %02d:%02d:%02d \n",SetTime.hours,SetTime.minutes,SetTime.seconds);

    //printf("%.*s \n", (uint32_t)message->payloadlen, (uint8_t *)message->payload);
}



/* Timer */
void repeating_timer_callback(void){
    MilliTimer_Handler();
}

/* RTC Interrupt */
void RTC_interrupt_callback(char *buf,uint32_t events){

    BaseType_t xHigherPriorityTaskWoken;

    /* Clear the interrupt. */

    xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR( AlarmInterruptSemaphore, (BaseType_t *)&xHigherPriorityTaskWoken );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

uint32_t macCalc(uint8_t *buf){

    uint32_t calcData = 0;

    for(uint8_t Loop = 0;Loop <= 8;Loop++){
        calcData ^= buf[Loop];
        calcData <<= 1;
        //printf(" CalcData %X, buffer %X \n",calcData,buf[Loop]);
    }
    calcData &= 0xFFFFFFFF;
    
    return calcData;
}