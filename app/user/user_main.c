/*
 *
 * Roomba Controller                                            HoWL  01-07/2015
 *
 * getestet auf ESP03
 *
 * LED (mit Vorwiderstand!) an GPIO14 gegen Masse zeigt WLAN Status an:
 * ein:     suche WLAN
 * aus:     mit WLAN verbunden
 * blinkt:  nicht mit WLAN verbunden, Accesspoint Mode
 *
 * kann auch gegen +3,3V geschaltet werden, zeigt den Status dann entgegengesetzt
 *
 * Hardware-Reset GPIO13 f�r 5 Sekunden gg. Masse
 *
 * 
 *
 * Roomba CMD siehe roomba.h
 *
 */
#include "c_types.h"
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"

#include "os_type.h"
#include "gpio.h"
#include "user_config.h"
#include "user_interface.h"
#include "roomba.h"

#include "at.h"

at_mdStateType mdState = m_unlink;

#define uint8_t unsigned char

uint8_t at_wifiMode;

struct station_config stationConf;
unsigned char set_ssid_wait;
unsigned char led_state;
unsigned char at_status;

extern uint8_t at_wifiMode;

char global_ssid[32] = {0};
char global_passwd[64] = {0};

static volatile os_timer_t my_timer;

volatile unsigned char mytimer_seconds;
volatile unsigned char mytimer_minutes;
volatile unsigned char mytimer_hours;
volatile unsigned int  mytimer_days;

volatile unsigned long mytimer_timestamp;

unsigned char buff[64];
unsigned char hash_avail;
unsigned char client_login_succesfull;

volatile unsigned char send_status;
extern volatile unsigned char ser_count;

// LED setzen
void setLED(unsigned char on_off)
{
    if (on_off == 0)
        gpio_output_set(0, BIT14, BIT14, 0); // LED auf 0 setzen
    else
        gpio_output_set(BIT14, 0, BIT14, 0); // LED auf 1 setzen

    led_state = on_off;
}

// LED toggeln
void toggleLED(void)
{
    if (led_state==off) setLED(on);
    else                setLED(off);
}

/*******************************************************************************
* Check auf Station Mode und gueltige IP Adresse*
******************************************************************************** */
check_ip(void)
{
    at_status   = wifi_station_get_connect_status();
    at_wifiMode = wifi_get_opmode();

    switch(at_wifiMode)
    {
        case STATION_MODE:
            if (at_status != STATION_GOT_IP && client_login_succesfull==0) wifi_set_opmode(STATIONAP_MODE);
            break;
        case STATIONAP_MODE:
            if (at_status == STATION_GOT_IP) wifi_set_opmode(STATION_MODE);
            if (client_login_succesfull==0) // beim ersten mal wegschreiben
            {
                spi_flash_read(0x3C000, (uint32 *)buff, 64);

                spi_flash_erase_sector(0x3C);
                buff[1] = 1;  // erfolgreicher Einlogversuch
                spi_flash_write(0x3C000, (uint32 *)buff, 64);

                client_login_succesfull=1;
            }
            break;
    }
    // nicht rauskuerzen!
    at_status   = wifi_station_get_connect_status();
    at_wifiMode = wifi_get_opmode();
}


// Sekundentimer
void my_timerfunc(void *arg)
{
    mytimer_timestamp++;
    mytimer_seconds++;
    if (mytimer_seconds > 59) { // uptime...
        mytimer_seconds = 0;
        mytimer_minutes++;
        if (mytimer_minutes > 59)  {
            mytimer_minutes = 0;
            mytimer_hours++;
            if (mytimer_hours > 23) {
                mytimer_hours = 0;
                mytimer_days++;
            }
        }
    }

    if (mytimer_seconds % 10 == 0) check_ip();     // IP pruefen

    if (mytimer_timestamp > 10)
    {
        if (at_wifiMode == STATIONAP_MODE || at_status != STATION_GOT_IP) toggleLED();   // im StationAP Mode blinken
        if (at_wifiMode == STATION_MODE   && at_status == STATION_GOT_IP) setLED(off);   // im Station Mode mit IP aus


        // Status aller 10 Sekunden anfordern
        if (mytimer_seconds % 10 == 0 && send_status==1) {
            ser_count = 0;
            uart_tx_one_char(UART0, SENSORS);
            uart_tx_one_char(UART0, SENSORS_POWER);
        }
    }

    //ntp_get_time();

    // ruecksetzen des Hashwertes fuer Passwort
    // GPIO 13 oder 02
    if (GPIO_INPUT_GET( 13 ) == 0 || GPIO_INPUT_GET( 2 ) == 0)
    {
        spi_flash_read(0x3C000, (uint32 *)buff, 64);
        spi_flash_erase_sector(0x3C);

        os_memcpy(&stationConf.ssid, "dummy", 32);
        os_memcpy(&stationConf.password, "dummy", 64);
        wifi_station_set_config(&stationConf);
        hash_avail = 0;
        client_login_succesfull = 0;
    }

    if (set_ssid_wait > 0) // nach setzen der SSID Parameter noch antworten, hier ueber den Timer dann erst setzen und reset
    {
        set_ssid_wait--;
        if (set_ssid_wait==0)
        {
            os_memcpy(&stationConf.ssid, global_ssid, 32);
            os_memcpy(&stationConf.password, global_passwd, 64);
            wifi_station_set_config(&stationConf);
            system_restart();
        }
    }
}

//##############################################################################
// user init
void user_init(void)
{
    unsigned int i;

    mytimer_timestamp =
    mytimer_seconds =
    mytimer_minutes =
    mytimer_hours   =
    mytimer_days    = 0;

    send_status = 1;

    set_ssid_wait = 0;
    client_login_succesfull = 0;

    //Set GPIO 14 to output mode (LED))
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
    setLED(on);

    //Set GPIO to output mode (Wakeup)
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2); // ESP01
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U , FUNC_GPIO12);  // ESP03

    // Set low
    // gpio_output_set(0, BIT2,  BIT2,  0); // ESP01
    gpio_output_set(0, BIT12, BIT12, 0);   // ESP03


    // Baudrate Roomba
    uart_init(BIT_RATE_115200, BIT_RATE_115200);
    roomba_init();
    roomba_init();


    for (i=0;i<500;i++)  // 5 Sekunden warten
    {
        WRITE_PERI_REG(0X60000914, 0x73); //WTD
        os_delay_us(10000); // 10ms
    }

    ser_init();

    at_setupCmdCipserver(80);   // Servermode, Port 80

    wifi_set_opmode( STATION_MODE );
    at_wifiMode = wifi_get_opmode();

    // Sekunden Timer
    os_timer_disarm(&my_timer);
    os_timer_setfn(&my_timer, (os_timer_func_t *)my_timerfunc, NULL);
    os_timer_arm(&my_timer, 1000, 1);

    spi_flash_read(0x3C000, (uint32 *)buff, 64);
    // Masterkennwort gesetzt ?
    if (buff[0] == 0xaa) hash_avail = 1; else hash_avail = 0;

    // mit der aktuellen Konfig schon einmal erfolgreich eingeloggt ?
    if (buff[1] == 0x01) client_login_succesfull = 1; else client_login_succesfull = 0;


    // ruecksetzen des Hash Wertes / WLAN Parameter fuer Wlan Parameter
    // GPIO13  oder GPIO2
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);    //set as gpio pin
    PIN_PULLDWN_DIS(PERIPHS_IO_MUX_MTCK_U);                //disable pulldown
    PIN_PULLUP_EN(PERIPHS_IO_MUX_MTCK_U);                //enable pull up R
    GPIO_DIS_OUTPUT( 13 );                              // disable output


    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U , FUNC_GPIO2);    //set as gpio pin
    PIN_PULLDWN_DIS(PERIPHS_IO_MUX_GPIO2_U );                //disable pulldown
    PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO2_U );                //enable pull up R
    GPIO_DIS_OUTPUT( 2 );                              // disable output
}
