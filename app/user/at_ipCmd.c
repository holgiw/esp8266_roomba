#include "c_types.h"
#include "user_interface.h"
#include "at_version.h"
#include "espconn.h"
#include "mem.h"
#include "at.h"
#include "at_ipCmd.h"
#include "osapi.h"
#include "gpio.h"
#include "driver/uart.h"
#include <stdlib.h>
#include "roomba.h"
#include "string.h"


static char html_doctype[] = "<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\">"; 
static char html_head[] = "<head><meta http-equiv=\"content-type\" content=\"text/html; charset=utf-8\"><title>Roomba WiFi</title><style type=\"text/css\"> button { height:25px; } #links{ margin-left: 350px; } #rechts{ float: left; }</style></head>";
static char html_headline[] = "<h1>Roomba WiFi</h1>HoWL@gmx.de<br><hr>";
static char html_ok[] = "HTTP/1.0 200 OK\r\n\r\n";

       
extern unsigned char set_ssid_wait;
extern char global_ssid[32];
extern char global_passwd[64];

unsigned char buffer[128];
extern unsigned char buff[128];
extern unsigned char hash_avail;

signed int speed = default_speed;
static volatile os_timer_t roomba_timer_stop;

extern volatile unsigned long mytimer_timestamp;
extern volatile unsigned char mytimer_seconds;
extern volatile unsigned char mytimer_minutes;
extern volatile unsigned char mytimer_hours;
extern volatile unsigned int mytimer_days;

extern volatile unsigned char send_status;

void roomba_timer_stop_func(void *arg);
void roomba_wakeup(void);
void roomba_startup(void);
void roomba_stop(void);
void setTimerStop(unsigned int ms);
void roomba_dock(void);
void roomba_clean(void);
void roomba_goForward(void);
void roomba_goBackward(void);
void roomba_goDistanceForward(unsigned int cm);
void roomba_goDistanceBackward(unsigned int cm);
void roomba_spinRight(void);
void roomba_spinLeft(void);
void roomba_turnLeft(unsigned int degrees);
void roomba_turnRight(unsigned int degrees);
void roomba_timer_stop_func(void *arg);
void roomba_drive(unsigned int velocity, unsigned int radius);


extern at_mdStateType mdState;
BOOL specialAtState;

extern uint8_t at_wifiMode;
os_timer_t at_delayChack;
BOOL IPMODE;
uint8_t ipDataSendFlag = 0;

static BOOL disAllFlag = FALSE;

static at_linkConType pLink[at_linkMax];
static uint8_t sendingID;
static BOOL serverEn = FALSE;
static at_linkNum = 0;

static uint16_t server_timeover = 180;
static struct espconn *pTcpServer;

static void at_tcpclient_discon_cb(void *arg);


//Stupid li'l helper function that returns the value of a hex char.
static int httpdHexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

/*
 Parsed nach Argument und gibt, wenn vorhanden eine zugewiesene Variable in Buff zurÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¼ck
        // http://192.168.0.108/version             -->0
        // http://192.168.0.108/?version            -->0
        // http://192.168.0.108/?version=           -->0
        // http://192.168.0.108/?cmd=123:124:125    -->11    
 
 Rueckgabe:-2 Ergebnis passt nicht in BUFF
           -1 nicht gefunden oder keinen Suchstring uebergeben
            0 gefunden, aber keine Value
           >0 Laenge der gefundenen Value 
 */

int httpdFindArg(char *line, char *arg, char *buff, int buffLen)
{   
    char escval = 0;
    int count = 0;
    unsigned int mystrlen = os_strlen(arg);   
     
    *buff = 0; // auf 0 initialisieren   
    if (mystrlen == 0) return -1;
    
    buffLen--;  // eines weniger wg. NULL am Ende
        
    line = strstr(line, arg); // erstes Vorkommen suchen

    if (line == NULL) return -1; // nicht gefunden

    line += mystrlen;
    if (*line++ == '=') {
        while (1) {
            if (*line == 0 || *line == ' ' || *line == '&' || *line == '?') return count; // Stringende, Anzahl rueckmelden
            if (*line == '+') { // codiertes Leerzeichen             
                line++;
                *buff++ = ' ';
            } else if (*line == '%') // escaped Zeichenfolge z.b. %20
            {
                line++;
                escval  = httpdHexVal(*line++) << 4;
                escval += httpdHexVal(*line++);
                *buff++ = escval;
            } else
                *buff++ = *line++;
            count++;
            *buff = 0; // naechsten auf 0 setzen                    
            if (buffLen-- == 0) return -2; // Ueberlauf verhindern
        }                
    } 
    return 0; // String gefunden aber kein "="    
}

/**
 * @brief  Client received callback function.
 * @param  arg: contain the ip link information
 * @param  pdata: received data
 * @param  len: the lenght of received data
 * @retval None
 */
void ICACHE_FLASH_ATTR
at_tcpclient_recv(void *arg, char *pdata, unsigned short len) {

    struct espconn *pespconn = (struct espconn *) arg;
    at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;

    send_status = 0; // Sperre Status anfordern
    at_user_cmd(arg, pdata, len);
    send_status = 1;

    espconn_disconnect(pLink[linkTemp->linkId].pCon);

}

// Watchdog fuettern

void feedwdt(void) {
    WRITE_PERI_REG(0X60000914, 0x73);
}

// roomba bei neustart initialisieren

void roomba_init(void) {
    roomba_startup();
    uart_tx_one_char(UART0, 138); // Motoren / PWM aus
    uart_tx_one_char(UART0, 0);
    uart_tx_one_char(UART0, 144);
    uart_tx_one_char(UART0, 0);
    uart_tx_one_char(UART0, 0);
    uart_tx_one_char(UART0, 0);

    roomba_stop();
}

void roomba_wakeup(void) {
    unsigned char j;

    // Roomba ueber WakeUp Pin "wecken"
    roomba_startup();
    gpio_output_set(0, BIT2, BIT2, 0); // ESP01 io2  Low
    gpio_output_set(0, BIT12, BIT12, 0); // ESP03 io12 Low

    for (j = 0; j < 50; j++) // 0,5 Sekunden
    {
        feedwdt();
        os_delay_us(10000); // 10ms
    }
    gpio_output_set(BIT2, 0, BIT2, 0); // ESP01 io2  High
    gpio_output_set(BIT12, 0, BIT12, 0); // ESP03 io12 High

    for (j = 0; j < 100; j++) // 1 Sekunde
    {
        feedwdt();
        os_delay_us(10000); //10ms
    }
    roomba_startup();
}

void roomba_startup(void) {
    uart_tx_one_char(UART0, START);
    uart_tx_one_char(UART0, CONTROL);
    os_delay_us(30000);
}

void setTimerStop(unsigned int ms) {
    os_timer_disarm(&roomba_timer_stop);
    os_timer_setfn(&roomba_timer_stop, (os_timer_func_t *) roomba_timer_stop_func, NULL);
    os_timer_arm(&roomba_timer_stop, ms, 0);
}

void roomba_dock(void) {
    roomba_startup();
    uart_tx_one_char(UART0, DOCK);
}

void roomba_clean(void) {
    roomba_startup();
    uart_tx_one_char(UART0, CLEAN);
}

void roomba_goForward(void) {
    roomba_drive(speed, 0x8000);
} 

void roomba_goBackward(void) {
    roomba_drive(-speed, 0x8000);
} 

void roomba_spinRight(void) {
    roomba_drive(speed, 0xffff);
}

void roomba_spinLeft(void) {
    roomba_drive(speed, 1);
}

void roomba_stop(void) {
    roomba_drive(0, 0);
}

void roomba_timer_stop_func(void *arg) {
    roomba_stop();
}

void roomba_drive(unsigned int velocity, unsigned int radius) {
    roomba_startup();
    uart_tx_one_char(UART0, DRIVE);
    uart_tx_one_char(UART0, velocity / 256);
    uart_tx_one_char(UART0, velocity);
    uart_tx_one_char(UART0, radius / 256);
    uart_tx_one_char(UART0, radius);
}

void roomba_goDistanceForward(unsigned int cm) {
    if (cm > 0)
    {
        roomba_goForward();
        setTimerStop(cm * (int) 10000 / speed);
    }
}

void roomba_goDistanceBackward(unsigned int cm) {
    if (cm > 0)
    {
        roomba_goBackward();
        setTimerStop(cm * (int) 10000 / speed);
    }
}

void roomba_turnLeft(unsigned int degrees) {
    if (degrees > 0)
    {
        float ftime = (float) 10.3 * degrees;
        int itime = abs(ftime);
        roomba_spinLeft();
        setTimerStop(itime);
    }
}

void roomba_turnRight(unsigned int degrees) {
    if (degrees > 0)
    {
        float ftime = (float) 10.3 * degrees;
        int itime = abs(ftime);
        roomba_spinRight();
        setTimerStop(itime);
    }
}

/*
 SDBM Algorithmus fuer Hashwert MasterKennwort
 Dieser Algorithmus wird in Sleepycat's Datenbank BDB (Berkeley DataBase) verwendet.
 */
unsigned long sdbm(unsigned char *str) {
    unsigned long hash_ = 0;
    unsigned char c;

    while (c = *str++)
        hash_ = c + (hash_ << 6) + (hash_ << 16) - hash_;

    return hash_;
}

//******************************************************************************

void ICACHE_FLASH_ATTR
at_user_cmd(void *arg, char *pdata, unsigned short len) {
    struct espconn *pespconn = (struct espconn *) arg;
    at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;
    char temp[256];

    unsigned char i, count, command, anzahl;
    unsigned int value,value1;
    unsigned char ip[4];    
    unsigned int param[32];

    // HTML 200
    espconn_sent(pLink[linkTemp->linkId].pCon, html_ok, strlen(html_ok));
  

    specialAtState = FALSE;
    
    /***********************************************************************
     * 
     * Masterkey setzen
     *   
     *********************************************************************** */

    // Eingabe MasterKennwort
    if (httpdFindArg(pdata, "masterkey1", buffer, sizeof (buffer)) >= 0)
    {     
        char key[64] = {0};
        unsigned long hash;

        os_memcpy(&key, buffer, 32);

        if (strlen(key) == 0) {
            espconn_sent(pLink[linkTemp->linkId].pCon, html_doctype, strlen(html_doctype));
            
            os_sprintf(temp, "<html><head><meta http-equiv=\"refresh\" content=\"3; URL=/\"><head><body bgcolor=\"#e6e6f0\">");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            espconn_sent(pLink[linkTemp->linkId].pCon, html_headline, strlen(html_headline));

            os_sprintf(temp, "<br>Bitte ein neues MasterKennwort eingeben</body></html>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
            return;
        }


        hash = sdbm(key);

        spi_flash_read(0x3C000, (uint32 *) buff, 64);
        spi_flash_erase_sector(0x3C);

        buff[0] = 0xaa; // Kennzeichen fuer MasterKennwort gesetzt
        buff[1] = 0; // noch kein erfolgreicher Einlogversuch
        buff[2] = hash & 0xff;
        buff[3] = (hash >> 8) & 0xff;
        buff[4] = (hash >> 16) & 0xff;
        buff[5] = (hash >> 24) & 0xff;

        spi_flash_write(0x3C000, (uint32 *) buff, 64);
        hash_avail = 1;

        espconn_sent(pLink[linkTemp->linkId].pCon, html_doctype, strlen(html_doctype));
        
        os_sprintf(temp, "<html><head><meta http-equiv=\"refresh\" content=\"3; URL=/\"></head><body bgcolor=\"#e6e6f0\">");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

        espconn_sent(pLink[linkTemp->linkId].pCon, html_headline, strlen(html_headline));
                
        os_sprintf(temp, "<br>MasterKennwort wird gespeichert...</body></html>");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

        return;
    }

    // SSID, passwort setzen, Masterkey pruefen
    if (httpdFindArg(pdata, "wlanssid", buffer, sizeof (buffer)) >= 0) 
    {            
        char ssid[32] = {0};
        char password[64] = {0};
        char key[64] = {0};
        unsigned long hash;
             
        os_memcpy(&ssid, buffer, 32);
        
        httpdFindArg(pdata, "wlanpasswd", buffer, sizeof (buffer));
        os_memcpy(&password, buffer, 32);
                
        httpdFindArg(pdata, "masterkey2", buffer, sizeof (buffer));
        os_memcpy(&key, buffer, 32);
        
        // "Abbrechen" oder nichts gefuellt
        if (strlen(ssid) == 0 && strlen(password) == 0 && strlen(key) == 0) {
            espconn_sent(pLink[linkTemp->linkId].pCon, html_doctype, strlen(html_doctype));
            os_sprintf(temp, "<html><head><meta http-equiv=\"refresh\" content=\"3; URL=/\"></head></html>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));           
            return;
        }

        // Masterkennwort pruefen
        hash = sdbm(key);
        spi_flash_read(0x3C000, (uint32 *) buff, 64);

        if ((buff[0] == 0xaa) &&
                (buff[2] == (hash & 0xff)) &&
                (buff[3] == ((hash >> 8) & 0xff)) &&
                (buff[4] == ((hash >> 16) & 0xff)) &&
                (buff[5] == ((hash >> 24) & 0xff)))
        {
                    
            set_ssid_wait = 5; // setzen und Reset in 5 Sekunden ueber Timer

            os_memcpy(&global_ssid, ssid, 32);
            os_memcpy(&global_passwd, password, 64);

            espconn_sent(pLink[linkTemp->linkId].pCon, html_doctype, strlen(html_doctype));
            
            os_sprintf(temp, "<html>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            espconn_sent(pLink[linkTemp->linkId].pCon, html_head, strlen(html_head));
            
            os_sprintf(temp, "<body bgcolor=\"#e6e6f0\">");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            espconn_sent(pLink[linkTemp->linkId].pCon, html_headline, strlen(html_headline));
         
            os_sprintf(temp, "<br>WLan Parameter gespeichert.<br>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            os_sprintf(temp, "<br>Kann sich der Roomba mit Ihrem Netzwerk verbinden erh&auml;lt er von dort per DHCP eine neue IP-Adresse<br>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            os_sprintf(temp, "<br>Rufen Sie die Webseite mit dieser neuen IP auf, z.B. http://192.168.0.100<br>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            os_sprintf(temp, "</body></html>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            return;
        } else {
            espconn_sent(pLink[linkTemp->linkId].pCon, html_doctype, strlen(html_doctype));
            
            os_sprintf(temp, "<html><head><meta http-equiv=\"refresh\" content=\"3; URL=/\"></head><body bgcolor=\"#e6e6f0\">");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            espconn_sent(pLink[linkTemp->linkId].pCon, html_headline, strlen(html_headline));

            os_sprintf(temp, "<br>MasterKennwort falsch</body></html>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            return;
        }
    }


    // -------------------------------------------------------------------------
    // nur im Station Mode !!!
    if (at_wifiMode == STATION_MODE) {


        /**************************************************************************
         * 
         * Update von Server x.x.x.x
         *        
         *********************************************************************** */
        
        if (httpdFindArg(pdata, "update", buffer, sizeof (buffer)) > 0) {
            
            espconn_sent(pLink[linkTemp->linkId].pCon, html_doctype, strlen(html_doctype));
            os_sprintf(temp, "<html><head><meta http-equiv=\"refresh\" content=\"10; URL=/\"></head><body bgcolor=\"#e6e6f0\">");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                             
            espconn_sent(pLink[linkTemp->linkId].pCon, html_headline, strlen(html_headline));

            os_sprintf(temp, "<br>Update gestartet... Bitte warten...</body></html>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
        

            // IP Adresse auslesen aus buffer
            count = 0;
            for (i = 0; i < 4; i++) {
                ip[i] = (buffer[count++] - '0');
                while (buffer[count] >= '0' && buffer[count] <= '9') {
                    ip[i] *= 10;
                    ip[i] += buffer[count++] - '0';
                }
                count++;
            }
        
            setLED(on);
            at_exeCmdUpdateCustom(0, ip[0], ip[1], ip[2], ip[3]);

            return;
        }

        /**************************************************************************
         * Version
         *
         *           111111
         * 0123456789012345
         * GET /ver
         * GET /version
         *********************************************************************** */
     
        if (httpdFindArg(pdata, "version", buffer, sizeof (buffer)) >= 0) {
            os_sprintf(temp, "OK:VERSION:" __DATE__ " " __TIME__ "\r\n");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
            return;
        }

        /**************************************************************************
         * Uptime
         *
         * 
         * 
         *
         * return: sekunden_seit_start:tage:stunden:minuten:sekunden
         *********************************************************************** */
        if (httpdFindArg(pdata, "uptime", buffer, sizeof (buffer)) >= 0) {
            os_sprintf(temp, "OK:UPTIME:%d:%d:%d:%d:%d", mytimer_timestamp,
                    mytimer_days, mytimer_hours, mytimer_minutes, mytimer_seconds);
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            return;
        }

        /**************************************************************************
         * 
         * Roomba CMD
         *         
         *********************************************************************** */
        if (httpdFindArg(pdata, "cmd", buffer, sizeof (buffer)) > 0) {
            // Uebergebene Parameter einlesen
            count = 0; 
            for (i = 0; i < 32; i++) {
                param[i] = 0; // wird benoetigt !
                while (buffer[count] >= '0' && buffer[count] <= '9') {
                    param[i] *= 10;
                    param[i] += buffer[count++] - '0';
                }
                anzahl++;
                if (buffer[count] != ':') break; // auf Trennzeichen ":" pruefen, ansonsten raus
                count++;
            }

            feedwdt();
            command = param[0];
            value = param[1]; // 16 Bit
            
            // bei Handeingabe
            if      (command==201 && httpdFindArg(pdata, "dist_v", buffer, sizeof (buffer)) > 0) value = atoi(buffer);
            else if (command==202 && httpdFindArg(pdata, "dist_r", buffer, sizeof (buffer)) > 0) value = atoi(buffer);
            else if (command==203 && httpdFindArg(pdata, "rot_r",  buffer, sizeof (buffer)) > 0) value = atoi(buffer);
            else if (command==204 && httpdFindArg(pdata, "rot_l",  buffer, sizeof (buffer)) > 0) value = atoi(buffer);
            
            // Command als Text            
            if      (httpdFindArg(pdata, "cmd=clean", buffer, sizeof (buffer)) == 0) command = myCLEAN;
            else if (httpdFindArg(pdata, "cmd=stop",  buffer, sizeof (buffer)) == 0) command = mySTOP;
            else if (httpdFindArg(pdata, "cmd=dock",  buffer, sizeof (buffer)) == 0) command = myDOCK;            
            
            
            switch (command) {
                case myReset: system_restart();
                    break;
                case WAKEUP: roomba_wakeup();
                    break;
                case mySTOP: roomba_stop();
                    break;
                case myDOCK: roomba_dock();
                    break;
                case myCLEAN: roomba_clean();
                    break;
                case goDistanceForward: roomba_goDistanceForward(value);
                    break;
                case goDistanceBackward: roomba_goDistanceBackward(value);
                    break;
                case turnRight: roomba_turnRight(value);
                    break;
                case turnLeft: roomba_turnLeft(value);
                    break;
             
                case SENSORS: break; // hier nur abfangen

                    // alle uebergebenen Parameter ausgeben an Roomba
                default:
                    for (i = 0; i < anzahl; i++) {
                        uart_tx_one_char(UART0, param[i]);
                        feedwdt();
                    }
                    break;
            }

            if (command == SENSORS) {
                os_sprintf(temp, "OK:ROOMBA:%d:", param[1]); // Gruppe
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                switch (param[1]) {
                    case SENSORS_POWER:
                        os_sprintf(temp, "%d:", ser[0]); // Status
                        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                        os_sprintf(temp, "%d:", ser[SENSORS_POWER_VOLTAGE_HI] * 256L + ser[SENSORS_POWER_VOLTAGE_LO]);
                        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                        value = ser[SENSORS_POWER_CURRENT_HI] * 256L + ser[SENSORS_POWER_CURRENT_LO];
                        if (value > 32768) {
                            value = (65536 - value);
                            os_sprintf(temp, "-%d:", value);
                        } else os_sprintf(temp, "%d:", value);
                        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                        os_sprintf(temp, "%d:", ser[SENSORS_POWER_TEMPERATURE]);
                        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                        os_sprintf(temp, "%d:", ser[SENSORS_POWER_CHARGE_HI] * 256L + ser[SENSORS_POWER_CHARGE_LO]);
                        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                        os_sprintf(temp, "%d:", ser[SENSORS_POWER_CAPACITY_HI] * 256L + ser[SENSORS_POWER_CAPACITY_LO]);
                        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                        break;
                }
            } else {
                os_sprintf(temp, "OK:ROOMBA:CMD");
               // espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
            }

            specialAtState = FALSE;

            feedwdt();
           
            if (httpdFindArg(pdata, "/web", buffer, sizeof (buffer)) == -1)
            {
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                return;
            }
            
        }
    }
    /**************************************************************************
     * 
     * Roomba Webseite
     *     
     * ********************************************************************* */
            // Doctype
            espconn_sent(pLink[linkTemp->linkId].pCon, html_doctype, strlen(html_doctype));

            os_sprintf(temp, "<html>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
            
            // Header
            espconn_sent(pLink[linkTemp->linkId].pCon, html_head, strlen(html_head));
                      
            os_sprintf(temp, "<body bgcolor=\"#e6e6f0\">");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
     
            // Kopfzeile
            espconn_sent(pLink[linkTemp->linkId].pCon, html_headline, strlen(html_headline));           

            if (hash_avail == 0) // noch kein Masterkennwort eingegeben                
            {
                os_sprintf(temp, "Bitte MasterKennwort eingeben<br><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                os_sprintf(temp, "(keine Sonderzeichen, keine Leerzeichen, nur Zahlen und Buchstaben)<br><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<form action=\"/web\" method=\"post\" autocomplete=\"off\">");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<table>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<tr><td>Masterkennwort:</td><td><input type=\"text\" name=\"masterkey1\" maxlength=\"30\" size=\"35\"></td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<tr><td colspan=2 align=\"right\">  <input type=\"submit\" value=\"senden\"></td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "</table></form>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                 
            }   // Hash vorhanden, aber im Stationmode, Eingabe von SSID, Passwort und MasterKennwort erforderlich
            //else if (at_wifiMode == STATIONAP_MODE || httpdFindArg(pdata, "config", buffer, sizeof (buffer)) >= 0) {
                
                // Hash vorhanden, aber keine Station IP bekommen, Eingabe von SSID, Passwort und MasterKennwort erforderlich
            else if (wifi_station_get_connect_status() != STATION_GOT_IP || httpdFindArg(pdata, "config", buffer, sizeof (buffer)) >= 0) {  
                        
                os_sprintf(temp, "<form action=\"/web\" method=\"post\" autocomplete=\"off\">");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<table>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<tr><td colspan=\"2\">&nbsp;</td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                os_sprintf(temp, "<tr><td>MasterKennwort:</td><td><input type=\"text\" name=\"masterkey2\" maxlength=\"32\" size=\"35\"></td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                os_sprintf(temp, "<tr><td colspan=\"2\">&nbsp;</td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                os_sprintf(temp, "<tr><td>WLan SSID:</td><td><input type=\"text\" name=\"wlanssid\" maxlength=\"64\" size=\"35\"></td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<tr><td>WLan Passwort:</td><td><input type=\"text\" name=\"wlanpasswd\" maxlength=\"32\" size=\"35\"></td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<tr><td colspan=\"2\">&nbsp;</td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));


                os_sprintf(temp, "<tr><td align=\"left\">  <input type=\"submit\" value=\"abbrechen\"></td>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                os_sprintf(temp, "<td align=\"right\">  <input type=\"submit\" value=\"senden\"></td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "</table></form>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
               
            } else {
              
                os_sprintf(temp, "Version: " __DATE__ " " __TIME__ "<br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
            
            
                os_sprintf(temp, "Uptime: %d Tag(e), %d Stunde(n), %d Minute(n), %d Sekunde(n)<br><br>",
                        mytimer_days, mytimer_hours, mytimer_minutes, mytimer_seconds);
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                                               
                os_sprintf(temp, "<form method=\"post\" action=\"web\">");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                os_sprintf(temp, "<button style=\"width:250px;\" name=\"update\" type=\"submit\" value=\"192.168.0.115\">Update !</button><br><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                
                
                os_sprintf(temp, "<button style=\"width:250px;\" name=\"config\" type=\"submit\" value=\"0\">Konfiguration WLan</button><br><br><hr><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                os_sprintf(temp, "<div id=\"rechts\"><b>Reinigungsfunktionen</b><br><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                
                
                os_sprintf(temp, "<button style=\"width:120px;\" name=\"cmd\" type=\"submit\" value=\"207\">Clean</button><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
 
                os_sprintf(temp, "<button style=\"width:120px;\" name=\"cmd\" type=\"submit\" value=\"206\">Dock</button><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                os_sprintf(temp, "<button style=\"width:120px;\" name=\"cmd\" type=\"submit\" value=\"205\">Stop</button>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
               
                
                os_sprintf(temp, "</div><div id=\"links\"><b>Achtung: folgende Funktionen arbeiten<br>ohne Auswertung der Sensoren !</b><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                
                os_sprintf(temp, "<button style=\"width:120px;\" name=\"cmd\" type=\"submit\" value=\"201\">vorw&auml;rts</button>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "&nbsp;<input type=\"text\" name=\"dist_v\" maxlength=\"5\" size=\"6\">&nbsp;cm<br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                os_sprintf(temp, "<button style=\"width:120px;\" name=\"cmd\" type=\"submit\" value=\"202\">r&uuml;ckw&auml;rts</button>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "&nbsp;<input type=\"text\" name=\"dist_r\" maxlength=\"5\" size=\"6\">&nbsp;cm<br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                os_sprintf(temp, "<button style=\"width:120px;\" name=\"cmd\" type=\"submit\" value=\"203\">Drehung rechts</button>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "&nbsp;<input type=\"text\" name=\"rot_r\" maxlength=\"3\" size=\"6\">&nbsp;Grad<br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                os_sprintf(temp, "<button style=\"width:120px;\" name=\"cmd\" type=\"submit\" value=\"204\">Drehung links</button>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "&nbsp;<input type=\"text\" name=\"rot_l\" maxlength=\"3\" size=\"6\">&nbsp;Grad<br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                              
                os_sprintf(temp, "</div><br><hr><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                // os_sprintf(temp, "<button style=\"width:100px;\" name=\"c\" type=\"submit\" value=\"w\">WakeUp</button><br>");
                // espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                os_sprintf(temp, "<button style=\"width:200px;\" name=\"c\" type=\"submit\" value=\"142:3\">Sensoren aktualisieren</button><br>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                os_sprintf(temp, "<table border=0>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

              
                os_sprintf(temp, "<tr><td>Status    </td><td align=\"right\">%d</td><td align=\"left\">", ser[SENSORS_POWER_CHARGINGSTATE]);
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                switch (ser[SENSORS_POWER_CHARGINGSTATE])
                {
                    case 0: os_sprintf(temp, "keine Ladung"); break;
                    case 1: os_sprintf(temp, "auffrischen"); break;
                    case 2: os_sprintf(temp, "laden"); break;
                    case 3: os_sprintf(temp, "Erhaltungsladung"); break;
                    case 4: os_sprintf(temp, "warten"); break;
                    case 5: os_sprintf(temp, "Fehler beim Laden"); break;
                }
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "</td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));                               
                
                os_sprintf(temp, "<tr><td>Spannung  </td><td align=\"right\">%d</td><td align=\"left\">mV </td></tr>", ser[SENSORS_POWER_VOLTAGE_HI] *256L + ser[SENSORS_POWER_VOLTAGE_LO]);
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<tr><td>Temperatur</td><td align=\"right\">%d</td><td align=\"left\">&deg;C</td></tr>", ser[SENSORS_POWER_TEMPERATURE]);
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<tr><td>Kapazitaet</td><td align=\"right\">%d</td><td align=\"left\">mAh</td></tr>", ser[SENSORS_POWER_CAPACITY_HI] *256L + ser[SENSORS_POWER_CAPACITY_LO]);
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                os_sprintf(temp, "<tr><td>Ladung    </td><td align=\"right\">%d</td><td align=\"left\">mAh", ser[SENSORS_POWER_CHARGE_HI] *256L + ser[SENSORS_POWER_CHARGE_LO]);
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                
                value = ser[SENSORS_POWER_CHARGE_HI]   *256L + ser[SENSORS_POWER_CHARGE_LO];
                value1 =ser[SENSORS_POWER_CAPACITY_HI] *256L + ser[SENSORS_POWER_CAPACITY_LO];
               
                if (value1 > 0)    {
                    os_sprintf(temp, "&nbsp;&nbsp;=&nbsp;%d %",  value * 100 / value1 );
                    espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                }
                os_sprintf(temp, "</td></tr>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
                    
                value = ser[SENSORS_POWER_CURRENT_HI] * 256L + ser[SENSORS_POWER_CURRENT_LO];
                if (value > 32768) {
                    value = (65536 - value);
                    os_sprintf(temp, "<tr><td>Strom</td><td align=\"right\">-%d</td><td align=\"left\">mA</td></tr>", value);
                } else
                    os_sprintf(temp, "<tr><td>Strom</td><td align=\"right\">%d</td><td align=\"left\">mA</td></tr>", value);
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

                os_sprintf(temp, "</table></form>");
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
            }

            os_sprintf(temp, "</body></html>");
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            specialAtState = FALSE;

            feedwdt();
            return;
}

/**
 * @brief  Client send over callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
static void ICACHE_FLASH_ATTR
at_tcpclient_sent_cb(void *arg) {
    //	os_free(at_dataLine);
    //  // ("send_cb\r\n");
    if (IPMODE == TRUE) {
        ipDataSendFlag = 0;
        os_timer_disarm(&at_delayChack);
        os_timer_arm(&at_delayChack, 20, 0);
        system_os_post(at_recvTaskPrio, 0, 0); ////
        ETS_UART_INTR_ENABLE();
        return;
    }
    //uart0_sendStr("\r\nSEND OK\r\n");
    specialAtState = TRUE;
}

/**
 * @brief  Tcp client connect success callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
static void ICACHE_FLASH_ATTR
at_tcpclient_connect_cb(void *arg) {
    struct espconn *pespconn = (struct espconn *) arg;
    at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;

    //// ("tcp client connect\r\n");
    //// ("pespconn %p\r\n", pespconn);

    linkTemp->linkEn = TRUE;
    linkTemp->teType = teClient;
    linkTemp->repeaTime = 0;
    espconn_regist_disconcb(pespconn, at_tcpclient_discon_cb);
    espconn_regist_recvcb(pespconn, at_tcpclient_recv); ////////
    espconn_regist_sentcb(pespconn, at_tcpclient_sent_cb); ///////

    mdState = m_linked;
    //  at_linkNum++;
    //at_backOk;
    //uart0_sendStr("Linked\r\n");
    specialAtState = TRUE;
}

/**
 * @brief  Tcp client connect repeat callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
static void ICACHE_FLASH_ATTR
at_tcpclient_recon_cb(void *arg, sint8 errType) {
    struct espconn *pespconn = (struct espconn *) arg;
    at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;
    struct ip_info ipconfig;
    os_timer_t sta_timer;

    // ("at_tcpclient_recon_cb %p\r\n", arg);

    if (linkTemp->teToff == TRUE) {
        linkTemp->teToff = FALSE;
        linkTemp->repeaTime = 0;
        if (pespconn->proto.tcp != NULL) {
            os_free(pespconn->proto.tcp);
        }
        os_free(pespconn);
        linkTemp->linkEn = false;
        at_linkNum--;
        if (at_linkNum == 0) {
            //at_backOk;
            mdState = m_unlink; //////////////////////
            // uart0_sendStr("Unlink\r\n");
            disAllFlag = false;
            specialAtState = TRUE;
        }
    } else {
        linkTemp->repeaTime++;
        if (linkTemp->repeaTime >= 1) {
            // ("repeat over %d\r\n", linkTemp->repeaTime);
            //      specialAtState = TRUE;
            //      at_state = at_statIdle;
            linkTemp->repeaTime = 0;
            //      // ("err %d\r\n", errType);
            if (errType == ESPCONN_CLSD) {
                //at_backOk;
            } else {
                // at_backError;
            }
            if (pespconn->proto.tcp != NULL) {
                os_free(pespconn->proto.tcp);
            }
            os_free(pespconn);
            linkTemp->linkEn = false;
            // ("disconnect\r\n");
            //  // ("con EN? %d\r\n", pLink[0].linkEn);
            at_linkNum--;
            if (at_linkNum == 0) {
                mdState = m_unlink; //////////////////////

                //     uart0_sendStr("Unlink\r\n");
                //    specialAtState = true;
                //    at_state = at_statIdle;
                disAllFlag = false;
                ETS_UART_INTR_ENABLE(); //exception disconnect
                //    specialAtState = true;
                //    at_state = at_statIdle;
                //    return;
            }
            specialAtState = true;
            return;
        }
        // ("link repeat %d\r\n", linkTemp->repeaTime);
        pespconn->proto.tcp->local_port = espconn_port();
        espconn_connect(pespconn);
    }
}


static ip_addr_t host_ip;

/******************************************************************************
 * FunctionName : user_esp_platform_dns_found
 * Description  : dns found callback
 * Parameters   : name -- pointer to the name that was looked up.
 *                ipaddr -- pointer to an ip_addr_t containing the IP address of
 *                the hostname, or NULL if the name could not be found (or on any
 *                other error).
 *                callback_arg -- a user-specified callback argument passed to
 *                dns_gethostbyname
 * Returns      : none
 *******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
at_dns_found(const char *name, ip_addr_t *ipaddr, void *arg) {
    struct espconn *pespconn = (struct espconn *) arg;
    at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;

    if (ipaddr == NULL) {
        linkTemp->linkEn = FALSE;
        //uart0_sendStr("DNS Fail\r\n");
        specialAtState = TRUE;
        //    device_status = DEVICE_CONNECT_SERVER_FAIL;
        return;
    }

    // ("DNS found: %d.%d.%d.%d\n",
    //        *((uint8 *) & ipaddr->addr),
    //        *((uint8 *) & ipaddr->addr + 1),
    //        *((uint8 *) & ipaddr->addr + 2),
    //        *((uint8 *) & ipaddr->addr + 3));

    if (host_ip.addr == 0 && ipaddr->addr != 0) {
        if (pespconn->type == ESPCONN_TCP) {
            os_memcpy(pespconn->proto.tcp->remote_ip, &ipaddr->addr, 4);
            espconn_connect(pespconn);
            at_linkNum++;
        } else {
            os_memcpy(pespconn->proto.udp->remote_ip, &ipaddr->addr, 4);
            espconn_connect(pespconn);
            specialAtState = TRUE;
            at_linkNum++;
            // at_backOk;
        }
    }
}

/**
 * @brief  Tcp client disconnect success callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
static void ICACHE_FLASH_ATTR
at_tcpclient_discon_cb(void *arg) {
    struct espconn *pespconn = (struct espconn *) arg;
    at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;
    uint8_t idTemp;

    if (pespconn == NULL) {
        return;
    }
    if (pespconn->proto.tcp != NULL) {
        os_free(pespconn->proto.tcp);
    }
    os_free(pespconn);

    linkTemp->linkEn = FALSE;
    // ("disconnect\r\n");
    //  // ("con EN? %d\r\n", pLink[0].linkEn);
    at_linkNum--;

    if (disAllFlag == FALSE) {
        //at_backOk;
    }
    if (at_linkNum == 0) {
        mdState = m_unlink; //////////////////////
        if (disAllFlag) {
            //at_backOk;
        }
        //    uart0_sendStr("Unlink\r\n");
        ETS_UART_INTR_ENABLE(); /////transparent is over
        //    specialAtState = TRUE;
        //    at_state = at_statIdle;
        disAllFlag = FALSE;
        //    specialAtState = TRUE;
        //    at_state = at_statIdle;
        //    return;
    }

    if (disAllFlag) {
        idTemp = linkTemp->linkId + 1;
        for (; idTemp < at_linkMax; idTemp++) {
            if (pLink[idTemp].linkEn) {
                if (pLink[idTemp].teType == teServer) {
                    continue;
                }
                if (pLink[idTemp].pCon->type == ESPCONN_TCP) {
                    specialAtState = FALSE;
                    espconn_disconnect(pLink[idTemp].pCon);
                    break;
                } else {
                    pLink[idTemp].linkEn = FALSE;
                    espconn_delete(pLink[idTemp].pCon);
                    os_free(pLink[idTemp].pCon->proto.udp);
                    os_free(pLink[idTemp].pCon);
                    at_linkNum--;
                    if (at_linkNum == 0) {
                        mdState = m_unlink;
                        //at_backOk;
                        //        uart0_sendStr("Unlink\r\n");
                        disAllFlag = FALSE;
                        //            specialAtState = TRUE;
                        //            at_state = at_statIdle;
                        //            return;
                    }
                }
            }
        }
    }
    //  IPMODE = FALSE;
    specialAtState = TRUE;
}


/**
 * @brief  Tcp server disconnect success callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
static void ICACHE_FLASH_ATTR
at_tcpserver_discon_cb(void *arg) {
    struct espconn *pespconn = (struct espconn *) arg;
    at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;

    // ("S conect C: %p\r\n", arg);

    if (pespconn == NULL) {
        return;
    }

    linkTemp->linkEn = FALSE;
    linkTemp->pCon = NULL;
    // ("con EN? %d\r\n", linkTemp->linkId);
    if (linkTemp->teToff == TRUE) {
        linkTemp->teToff = FALSE;
        specialAtState = true;
        //at_backOk;
    }
    at_linkNum--;
    if (at_linkNum == 0) {
        mdState = m_unlink;
        //uart0_sendStr("Unlink\r\n");
        disAllFlag = false;
    }
}

/**
 * @brief  Tcp server connect repeat callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
static void ICACHE_FLASH_ATTR
at_tcpserver_recon_cb(void *arg, sint8 errType) {
    struct espconn *pespconn = (struct espconn *) arg;
    at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;

    // ("S conect C: %p\r\n", arg);

    if (pespconn == NULL) {
        return;
    }

    linkTemp->linkEn = false;
    linkTemp->pCon = NULL;
    // ("con EN? %d\r\n", linkTemp->linkId);
    at_linkNum--;
    if (at_linkNum == 0) {
        mdState = m_unlink; //////////////////////

        //  uart0_sendStr("Unlink\r\n");
        disAllFlag = false;
    }
    if (linkTemp->teToff == TRUE) {
        linkTemp->teToff = FALSE;
        specialAtState = true;
        //at_backOk;
    }
}

/**
 * @brief  Tcp server listend callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
LOCAL void ICACHE_FLASH_ATTR
at_tcpserver_listen(void *arg) {
    struct espconn *pespconn = (struct espconn *) arg;
    uint8_t i;

    // ("get tcpClient:\r\n");
    for (i = 0; i < at_linkMax; i++) {
        if (pLink[i].linkEn == FALSE) {
            pLink[i].linkEn = TRUE;
            break;
        }
    }
    if (i >= 5) {
        return;
    }
    pLink[i].teToff = FALSE;
    pLink[i].linkId = i;
    pLink[i].teType = teServer;
    pLink[i].repeaTime = 0;
    pLink[i].pCon = pespconn;
    mdState = m_linked;
    at_linkNum++;
    pespconn->reverse = &pLink[i];
    espconn_regist_recvcb(pespconn, at_tcpclient_recv);
    espconn_regist_reconcb(pespconn, at_tcpserver_recon_cb);
    espconn_regist_disconcb(pespconn, at_tcpserver_discon_cb);
    espconn_regist_sentcb(pespconn, at_tcpclient_sent_cb); ///////
    // uart0_sendStr("Link\r\n");
}

/**
 * @brief  Udp server receive data callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
LOCAL void ICACHE_FLASH_ATTR
at_udpserver_recv(void *arg, char *pusrdata, unsigned short len) {
    struct espconn *pespconn = (struct espconn *) arg;
    at_linkConType *linkTemp;
    char temp[32];
    uint8_t i;

    // ("get udpClient:\r\n");

    if (pespconn->reverse == NULL) {
        for (i = 0; i < at_linkMax; i++) {
            if (pLink[i].linkEn == FALSE) {
                pLink[i].linkEn = TRUE;
                break;
            }
        }
        if (i >= 5) {
            return;
        }
        pLink[i].teToff = FALSE;
        pLink[i].linkId = i;
        pLink[i].teType = teServer;
        pLink[i].repeaTime = 0;
        pLink[i].pCon = pespconn;
        espconn_regist_sentcb(pLink[i].pCon, at_tcpclient_sent_cb);
        mdState = m_linked;
        at_linkNum++;
        pespconn->reverse = &pLink[i];
        // uart0_sendStr("Link\r\n");
    }
    linkTemp = (at_linkConType *) pespconn->reverse;
    if (pusrdata == NULL) {
        return;
    }
    //os_sprintf(temp, "\r\n+IPD,%d,%d:",
    //        linkTemp->linkId, len);
    uart0_sendStr(temp);
    uart0_tx_buffer(pusrdata, len);
    //at_backOk;
}

/**
 * @brief  Setup commad of module as server.
 * @param  id: commad id number
 * @param  pPara: AT input param
 * @retval None
 */
void ICACHE_FLASH_ATTR
at_setupCmdCipserver(uint8_t port) {
   
   
        pTcpServer = (struct espconn *) os_zalloc(sizeof (struct espconn));
        if (pTcpServer == NULL) {
            //uart0_sendStr("TcpServer Failure\r\n");
            return;
        }
        pTcpServer->type = ESPCONN_TCP;
        pTcpServer->state = ESPCONN_NONE;
        pTcpServer->proto.tcp = (esp_tcp *) os_zalloc(sizeof (esp_tcp));
        pTcpServer->proto.tcp->local_port = port;
        espconn_regist_connectcb(pTcpServer, at_tcpserver_listen);
        espconn_accept(pTcpServer);
        espconn_regist_time(pTcpServer, 15, 0);
       
    
    serverEn = 1;
    //at_backOk;
}


#define ESP_PARAM_SAVE_SEC_0    1
#define ESP_PARAM_SAVE_SEC_1    2
#define ESP_PARAM_SEC_FLAG      3
#define UPGRADE_FRAME  "{\"path\": \"/v1/messages/\", \"method\": \"POST\", \"meta\": {\"Authorization\": \"token %s\"},\
\"get\":{\"action\":\"%s\"},\"body\":{\"pre_rom_version\":\"%s\",\"rom_version\":\"%s\"}}\n"

#define pheadbuffer "Connection: keep-alive\r\n\
Cache-Control: no-cache\r\n\
User-Agent: Mozilla/5.0 (Windows NT 5.1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/30.0.1599.101 Safari/537.36 \r\n\
Accept: */*\r\n\
Authorization: token %s\r\n\
Accept-Encoding: gzip,deflate,sdch\r\n\
Accept-Language: zh-CN,zh;q=0.8\r\n\r\n"

//#define test
#ifdef test
#define KEY "39cdfe29a1863489e788efc339f514d78b78f0de"
#else
#define KEY "4ec90c1abbd5ffc0b339f34560a2eb8d71733861"
#endif


struct espconn *pespconn;
struct upgrade_server_info *upServer = NULL;

struct esp_platform_saved_param {
    uint8 devkey[40];
    uint8 token[40];
    uint8 activeflag;
    uint8 pad[3];
};

struct esp_platform_sec_flag_param {
    uint8 flag;
    uint8 pad[3];
};

/******************************************************************************
 * FunctionName : user_esp_platform_upgrade_cb
 * Description  : Processing the downloaded data from the server
 * Parameters   : pespconn -- the espconn used to connetion with the host
 * Returns      : none
 *******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
at_upDate_rsp(void *arg) {
    struct upgrade_server_info *server = arg;
   
    if (server->upgrade_flag == true) system_upgrade_reboot();
       
    // ("device_upgrade_success\r\n");  
    //} else // ("device_upgrade_failed\r\n");

    os_free(server->url);
    server->url = NULL;
    os_free(server);
    server = NULL;

    specialAtState = TRUE;
}


/**
 * @brief  Tcp client disconnect success callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
static void ICACHE_FLASH_ATTR
at_upDate_discon_cb(void *arg) {
    struct espconn *pespconn = (struct espconn *) arg;
    uint8_t idTemp;

   
    if (pespconn->proto.tcp != NULL) {
        os_free(pespconn->proto.tcp);
    }
    if (pespconn != NULL) {
        os_free(pespconn);
    }

    // ("disconnect\r\n");

    if (system_upgrade_start(upServer) == false) {
        //    uart0_sendStr("+CIPUPDATE:0/r/n");
        //at_backError;
        specialAtState = TRUE;
    } else {
        //uart0_sendStr("+CIPUPDATE:4\r\n");
    }
}

/**
 * @brief  Udp server receive data callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
LOCAL void ICACHE_FLASH_ATTR
at_upDate_recv(void *arg, char *pusrdata, unsigned short len) {
    struct espconn *pespconn = (struct espconn *) arg;
    char temp[32];
    char *pTemp;
    uint8_t user_bin[9] = {0};  
    uint8_t i;

    os_timer_disarm(&at_delayChack);
    //  // ("get upRom:\r\n");
    //uart0_sendStr("+CIPUPDATE:3\r\n");

    //  // ("%s",pusrdata);
    pTemp = (char *) os_strstr(pusrdata, "rom_version\": ");
    if (pTemp == NULL) {
        //uart0_sendStr("+CIPUPDATE: error\r\n");
        return;
    }
    pTemp += sizeof ("rom_version\": ");

    //  user_esp_platform_load_param(&esp_param);

    upServer = (struct upgrade_server_info *) os_zalloc(sizeof (struct upgrade_server_info));
    os_memcpy(upServer->upgrade_version, pTemp, 5);
    upServer->upgrade_version[5] = '\0';
    os_sprintf(upServer->pre_version, "v%d.%d", AT_VERSION_main, AT_VERSION_sub);

    upServer->pespconn = pespconn;
    
    os_memcpy(upServer->ip, pespconn->proto.tcp->remote_ip, 4);

    upServer->port = 80;

    upServer->check_cb = at_upDate_rsp;
    upServer->check_times = 60000;

    if (upServer->url == NULL) {
        upServer->url = (uint8 *) os_zalloc(512);
    }

    if (system_upgrade_userbin_check() == UPGRADE_FW_BIN1) {
        //uart0_sendStr("+CIPUPDATE: user2.bin\r\n");
        os_memcpy(user_bin, "user2.bin", 10);

    } else if (system_upgrade_userbin_check() == UPGRADE_FW_BIN2) {
        //uart0_sendStr("+CIPUPDATE: user1.bin\r\n");
        os_memcpy(user_bin, "user1.bin", 10);
    }


    os_sprintf(upServer->url,
            "GET /roomba/device/rom/?action=download_rom&version=%s&filename=%s HTTP/1.1\r\nHost: "IPSTR":%d\r\n"pheadbuffer"",
            upServer->upgrade_version, user_bin, IP2STR(upServer->ip),
            upServer->port, KEY);

    espconn_disconnect(pespconn);
        
}

LOCAL void ICACHE_FLASH_ATTR
at_upDate_wait(void *arg) {
    struct espconn *pespconn = arg;
    os_timer_disarm(&at_delayChack);
    if (pespconn != NULL) {
        espconn_disconnect(pespconn);
    } else {
        //at_backError;
        specialAtState = TRUE;
    }
}

/******************************************************************************
 * FunctionName : user_esp_platform_sent_cb
 * Description  : Data has been sent successfully and acknowledged by the remote host.
 * Parameters   : arg -- Additional argument to pass to the callback function
 * Returns      : none
 *******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
at_upDate_sent_cb(void *arg) {
    struct espconn *pespconn = arg;
    os_timer_disarm(&at_delayChack);
    os_timer_setfn(&at_delayChack, (os_timer_func_t *) at_upDate_wait, pespconn);
    os_timer_arm(&at_delayChack, 5000, 0);
    // ("at_upDate_sent_cb\r\n");
}

/**
 * @brief  Tcp client connect success callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
static void ICACHE_FLASH_ATTR
at_upDate_connect_cb(void *arg) {
    struct espconn *pespconn = (struct espconn *) arg;
    uint8_t user_bin[9] = {0};
    //  uint8_t devkey[41] = {0};
    char *temp;

    //uart0_sendStr("+CIPUPDATE:2\r\n");
   
    espconn_regist_disconcb(pespconn, at_upDate_discon_cb);
    espconn_regist_recvcb(pespconn, at_upDate_recv); ////////
    espconn_regist_sentcb(pespconn, at_upDate_sent_cb);
   
    temp = (uint8 *) os_zalloc(512);

    os_sprintf(temp, "GET /roomba/device/rom/?is_format_simple=true HTTP/1.0\r\nHost: "IPSTR":%d\r\n"pheadbuffer"",
            IP2STR(pespconn->proto.tcp->remote_ip),
            80, KEY);

    espconn_sent(pespconn, temp, os_strlen(temp));
    os_free(temp);
    /////////////////////////
}

/**
 * @brief  Tcp client connect repeat callback function.
 * @param  arg: contain the ip link information
 * @retval None
 */
static void ICACHE_FLASH_ATTR
at_upDate_recon_cb(void *arg, sint8 errType) {
    struct espconn *pespconn = (struct espconn *) arg;
   
    if (pespconn->proto.tcp != NULL) {
        os_free(pespconn->proto.tcp);
    }
    os_free(pespconn);
    // ("disconnect\r\n");

    if (upServer != NULL) {
        os_free(upServer);
        upServer = NULL;
    }
    //at_backError;

    specialAtState = TRUE;
   
}

/******************************************************************************
 * FunctionName : upServer_dns_found
 * Description  : dns found callback
 * Parameters   : name -- pointer to the name that was looked up.
 *                ipaddr -- pointer to an ip_addr_t containing the IP address of
 *                the hostname, or NULL if the name could not be found (or on any
 *                other error).
 *                callback_arg -- a user-specified callback argument passed to
 *                dns_gethostbyname
 * Returns      : none
 *******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
upServer_dns_found(const char *name, ip_addr_t *ipaddr, void *arg) {
    struct espconn *pespconn = (struct espconn *) arg;
    //  char temp[32];

    if (ipaddr == NULL) {
        //at_backError;
        //    os_sprintf(temp,at_backTeError,2);
        //    uart0_sendStr(at_backTeError"2\r\n");
        specialAtState = TRUE;
        //    device_status = DEVICE_CONNECT_SERVER_FAIL;
        return;
    }
    //uart0_sendStr("+CIPUPDATE:1\r\n");

    //  // ("DNS found: %d.%d.%d.%d\n",
    //            *((uint8 *) &ipaddr->addr),
    //            *((uint8 *) &ipaddr->addr + 1),
    //            *((uint8 *) &ipaddr->addr + 2),
    //            *((uint8 *) &ipaddr->addr + 3));

    if (host_ip.addr == 0 && ipaddr->addr != 0) {
        if (pespconn->type == ESPCONN_TCP) {
            os_memcpy(pespconn->proto.tcp->remote_ip, &ipaddr->addr, 4);
            espconn_regist_connectcb(pespconn, at_upDate_connect_cb);
            espconn_regist_reconcb(pespconn, at_upDate_recon_cb);
            espconn_connect(pespconn);

            //      at_upDate_connect_cb(pespconn);
        }
    }
}

void ICACHE_FLASH_ATTR
at_exeCmdUpdate(uint8_t id) {
    pespconn = (struct espconn *) os_zalloc(sizeof (struct espconn));
    pespconn->type = ESPCONN_TCP;
    pespconn->state = ESPCONN_NONE;
    pespconn->proto.tcp = (esp_tcp *) os_zalloc(sizeof (esp_tcp));
    pespconn->proto.tcp->local_port = espconn_port();
    pespconn->proto.tcp->remote_port = 80;

    specialAtState = FALSE;
    //espconn_gethostbyname(pespconn, "iot.espressif.cn", &host_ip, upServer_dns_found);

    //uart0_sendStr("+CIPUPDATE:1b\r\n");
    if (host_ip.addr == 0) {
        *((uint8 *) & pespconn->proto.tcp->remote_ip) = 192; //Host IP
        *((uint8 *) & pespconn->proto.tcp->remote_ip + 1) = 168;
        *((uint8 *) & pespconn->proto.tcp->remote_ip + 2) = 0;
        *((uint8 *) & pespconn->proto.tcp->remote_ip + 3) = 205;

        espconn_regist_connectcb(pespconn, at_upDate_connect_cb);
        espconn_regist_reconcb(pespconn, at_upDate_recon_cb);
        espconn_connect(pespconn);
    }
}

void at_exeCmdUpdateCustom(uint8_t id, uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3) {
    pespconn = (struct espconn *) os_zalloc(sizeof (struct espconn));
    pespconn->type = ESPCONN_TCP;
    pespconn->state = ESPCONN_NONE;
    pespconn->proto.tcp = (esp_tcp *) os_zalloc(sizeof (esp_tcp));
    pespconn->proto.tcp->local_port = espconn_port();
    pespconn->proto.tcp->remote_port = 80;

    specialAtState = FALSE;
    //espconn_gethostbyname(pespconn, "iot.espressif.cn", &host_ip, upServer_dns_found);

    //uart0_sendStr("+CIPUPDATE:1b\r\n");

    if (host_ip.addr == 0) {
        *((uint8 *) & pespconn->proto.tcp->remote_ip) = ip0; //Host IP
        *((uint8 *) & pespconn->proto.tcp->remote_ip + 1) = ip1;
        *((uint8 *) & pespconn->proto.tcp->remote_ip + 2) = ip2;
        *((uint8 *) & pespconn->proto.tcp->remote_ip + 3) = ip3;

        espconn_regist_connectcb(pespconn, at_upDate_connect_cb);
        espconn_regist_reconcb(pespconn, at_upDate_recon_cb);
        espconn_connect(pespconn);
    }
}
