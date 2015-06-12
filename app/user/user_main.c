/*
 * Roomba Controller                                            HoWL  01-06/2015
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
 *
 * Benutzung
 * WLAN einrichten  http://192.168.4.1/wlan=ssid,password
 * Webinterface:    http://192.168.0.100
 * Roomba CMD:      http://192.168.0.100/cmd=128:131:135:0:0
 * Version:         http://192.168.0.100/ver  
 * Update per WLAN: http://192.168.0.100/upd=192.168.0.70 
 * (192.168.0.70 Web-Server mit eingerichtetem Webordner /roomba/device/rom/ vorausgesetzt)
 *
 * 
 * Roomba CMD siehe roomba.h
 * 
 */

#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "at.h"
#include "os_type.h"
#include "gpio.h"
#include "user_config.h"
#include "user_interface.h"
#include "roomba.h"


unsigned char led_state;
extern volatile unsigned char ser[65];
extern volatile unsigned char ser_count;
extern uint8_t at_wifiMode;

static volatile os_timer_t my_timer;

volatile unsigned char mytimer_seconds;
volatile unsigned char mytimer_minutes;
volatile unsigned char mytimer_hours;
volatile unsigned int  mytimer_days;

volatile unsigned char send_status;

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
* Check auf Station Mode und gueltige IP Adresse, 
* fallback zu StationAP Mode fuer Eingabe WLAN Parameter
*
******************************************************************************** */ 
check_ip(void)
{
    static unsigned char status;
    
    status      = wifi_station_get_connect_status();
    at_wifiMode = wifi_get_opmode();

    switch(at_wifiMode)
    {
        case STATION_MODE:   if (status != STATION_GOT_IP) wifi_set_opmode(STATIONAP_MODE); break;
        case STATIONAP_MODE: if (status == STATION_GOT_IP) wifi_set_opmode(STATION_MODE);   break;
    }
    
    at_wifiMode = wifi_get_opmode(); // nicht rauskuerzen!
}


// Sekundentimer
void my_timerfunc(void *arg)
{  
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
    
    if (mytimer_seconds % 10 == 0) {
        check_ip();                                     // IP prüfen
        if (at_wifiMode == STATION_MODE) setLED(off);   // im Station Mode aus
    }
    
    if (at_wifiMode == STATIONAP_MODE) toggleLED();     // im StationAP Mode blinken    
    
    // Status aller 30 Sekunden anfordern
    if (mytimer_seconds % 30 == 0 && send_status == 1) {        
        ser_count = 0;       
        uart_tx_one_char(UART0, SENSORS);
        uart_tx_one_char(UART0, SENSORS_POWER);
    }
}

//##############################################################################
// user init
void user_init(void)
{     
    unsigned int i;
    
    mytimer_seconds = 
    mytimer_minutes = 
    mytimer_hours   = 
    mytimer_days    = 0;
    
    send_status     = 1; // Status senden einschalten

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

    at_init();        

    at_setupCmdE(15,"0");               // Echo aus
    at_setupCmdCipmux(15,"=1");         // mehrfache Verbindungen zulassen    
    at_setupCmdCipserver(15,"=1,80");   // Servermode, Port 80   
    at_setupCmdCipsto(15,"=15");        // Timeout 15 Sekunden  
    
    
    wifi_set_opmode( STATION_MODE );
    at_wifiMode = wifi_get_opmode();
        
           
    // Sekunden Timer 
    os_timer_disarm(&my_timer);
    os_timer_setfn(&my_timer, (os_timer_func_t *)my_timerfunc, NULL);
    os_timer_arm(&my_timer, 1000, 1);    
}