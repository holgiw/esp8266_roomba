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

signed int speed = default_speed;
static volatile os_timer_t roomba_timer_stop;

extern volatile unsigned char mytimer_seconds;
extern volatile unsigned char mytimer_minutes;
extern volatile unsigned char mytimer_hours;
extern volatile unsigned int  mytimer_days;

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
void roomba_goStraightAt(signed int velocity );

extern at_mdStateType mdState;
extern BOOL specialAtState;
extern at_stateType at_state;
extern at_funcationType at_fun[];
extern uint8_t *pDataLine;
extern uint8_t at_dataLine[];///
//extern uint8_t *at_dataLine;
//extern UartDevice UartDev;
extern uint8_t at_wifiMode;
extern int8_t at_dataStrCpy(void *pDest, const void *pSrc, int8_t maxLen);

uint16_t at_sendLen; //now is 256
uint16_t at_tranLen; //now is 256
os_timer_t at_delayChack;
BOOL IPMODE;
uint8_t ipDataSendFlag = 0;

static BOOL at_ipMux = TRUE;
static BOOL disAllFlag = FALSE;


static at_linkConType pLink[at_linkMax];
static uint8_t sendingID;
static BOOL serverEn = FALSE;
static at_linkNum = 0;

//static uint8_t repeat_time = 0;
static uint16_t server_timeover = 180;
static struct espconn *pTcpServer;
static struct espconn *pUdpServer;

static void at_tcpclient_discon_cb(void *arg);

/**
  * @brief  Test commad of get module ip.
  * @param  id: commad id number
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_testCmdCifsr(uint8_t id)
{
  at_backOk;
}

void ICACHE_FLASH_ATTR
at_setupCmdCifsr(uint8_t id, char *pPara)
{
  struct ip_info pTempIp;
  int8_t len;
  char ipTemp[64];
//  char temp[64];

  if(at_wifiMode == STATION_MODE)
  {
    at_backError;
    return;
  }
  pPara = strchr(pPara, '\"');
  len = at_dataStrCpy(ipTemp, pPara, 32);
  if(len == -1)
  {
    uart0_sendStr("IP ERROR\r\n");
    return;
  }

  wifi_get_ip_info(0x01, &pTempIp);
  pTempIp.ip.addr = ipaddr_addr(ipTemp);

  os_printf("%d.%d.%d.%d\r\n",
                 IP2STR(&pTempIp.ip));

  if(!wifi_set_ip_info(0x01, &pTempIp))
  {
    at_backError;
    return;
  }
  at_backOk;
  return;
}

/**
  * @brief  Execution commad of get module ip.
  * @param  id: commad id number
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_exeCmdCifsr(uint8_t id)//add get station ip and ap ip
{
  struct ip_info pTempIp;
  char temp[64];

  if((at_wifiMode == SOFTAP_MODE)||(at_wifiMode == STATIONAP_MODE))
  {
    wifi_get_ip_info(0x01, &pTempIp);
    os_sprintf(temp, "%d.%d.%d.%d\r\n",
               IP2STR(&pTempIp.ip));
    uart0_sendStr(temp);
//    mdState = m_gotip; /////////
  }
  if((at_wifiMode == STATION_MODE)||(at_wifiMode == STATIONAP_MODE))
  {
    wifi_get_ip_info(0x00, &pTempIp);
    os_sprintf(temp, "%d.%d.%d.%d\r\n",
               IP2STR(&pTempIp.ip));
    uart0_sendStr(temp);
//    mdState = m_gotip; /////////
  }
  mdState = m_gotip;
  at_backOk;
}

/**
  * @brief  Test commad of get link status.
  * @param  id: commad id number
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_testCmdCipstatus(uint8_t id)
{
  at_backOk;
}

/**
  * @brief  Execution commad of get link status.
  * @param  id: commad id number
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_exeCmdCipstatus(uint8_t id)
{
  char temp[64];
  uint8_t i;

  os_sprintf(temp, "STATUS:%d\r\n",
             mdState);
  uart0_sendStr(temp);
  if(serverEn)
  {

  }
  for(i=0; i<at_linkMax; i++)
  {
    if(pLink[i].linkEn)
    {
      if(pLink[i].pCon->type == ESPCONN_TCP)
      {
        os_sprintf(temp, "%s:%d,\"TCP\",\"%d.%d.%d.%d\",%d,%d\r\n",
                   at_fun[id].at_cmdName,
                   pLink[i].linkId,
                   IP2STR(pLink[i].pCon->proto.tcp->remote_ip),
                   pLink[i].pCon->proto.tcp->remote_port,
                   pLink[i].teType);
        uart0_sendStr(temp);
      }
      else
      {
        os_sprintf(temp, "%s:%d,\"UDP\",\"%d.%d.%d.%d\",%d,%d\r\n",
                   at_fun[id].at_cmdName,
                   pLink[i].linkId,
                   IP2STR(pLink[i].pCon->proto.udp->remote_ip),
                   pLink[i].pCon->proto.udp->remote_port,
                   pLink[i].teType);
        uart0_sendStr(temp);
      }
    }
  }
  at_backOk;
}

/**
  * @brief  Test commad of start client.
  * @param  id: commad id number
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_testCmdCipstart(uint8_t id)
{
  char temp[64];

  if(at_ipMux)
  {
    os_sprintf(temp, "%s:(\"type\"),(\"ip address\"),(port)\r\n",
               at_fun[id].at_cmdName);
    uart0_sendStr(temp);
    os_sprintf(temp, "%s:(\"type\"),(\"domain name\"),(port)\r\n",
               at_fun[id].at_cmdName);
    uart0_sendStr(temp);
  }
  else
  {
    os_sprintf(temp, "%s:(id)(\"type\"),(\"ip address\"),(port)\r\n",
               at_fun[id].at_cmdName);
    uart0_sendStr(temp);
    os_sprintf(temp, "%s:((id)\"type\"),(\"domain name\"),(port)\r\n",
               at_fun[id].at_cmdName);
    uart0_sendStr(temp);
  }
  at_backOk;
}

/**
  * @brief  Client received callback function.
  * @param  arg: contain the ip link information
  * @param  pdata: received data
  * @param  len: the lenght of received data
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_tcpclient_recv(void *arg, char *pdata, unsigned short len)
{   
    
    struct espconn *pespconn = (struct espconn *)arg;
    at_linkConType *linkTemp = (at_linkConType *)pespconn->reverse;
    
    send_status = 0;        // Sperre Status senden
    at_user_cmd(arg,pdata,len);
    send_status = 1;
    
    espconn_disconnect(pLink[linkTemp->linkId].pCon);  
             
    at_backOk;
}

// Watchdog fuettern
void feedwdt(void)  { WRITE_PERI_REG(0X60000914, 0x73); }

void roomba_init(void)
{
    roomba_startup(); 
    uart_tx_one_char(UART0, 138); // Motoren / PWM aus
    uart_tx_one_char(UART0, 0);
    uart_tx_one_char(UART0, 144);
    uart_tx_one_char(UART0, 0);
    uart_tx_one_char(UART0, 0);
    uart_tx_one_char(UART0, 0);
    
    roomba_stop();
}

void roomba_wakeup(void)
{
    unsigned char j;
    
    // Roomba start
    roomba_startup();
    gpio_output_set(0, BIT2, BIT2,  0); // ESP01 io2  Low
    gpio_output_set(0, BIT12,BIT12, 0); // ESP03 io12 Low
    
    for (j=0;j<50;j++)   // 0,5 Sekunden
    {   
        feedwdt();
        os_delay_us(10000);    // 10ms
    }
    gpio_output_set(BIT2,  0, BIT2,  0); // ESP01 io2  High
    gpio_output_set(BIT12, 0, BIT12, 0); // ESP03 io12 High
    
    for (j=0;j<100;j++)   // 1 Sekunde
    {
        feedwdt();
        os_delay_us(10000); //10ms
    }    
    roomba_startup();
}

void roomba_startup(void)
{   
    uart_tx_one_char(UART0, START);
    uart_tx_one_char(UART0, CONTROL);
    os_delay_us(30000);
}

void setTimerStop(unsigned int ms)
{
    os_timer_disarm(&roomba_timer_stop);  
    os_timer_setfn(&roomba_timer_stop, (os_timer_func_t *)roomba_timer_stop_func, NULL);
    os_timer_arm(&roomba_timer_stop, ms, 0);
}

void roomba_dock(void)      { roomba_startup(); uart_tx_one_char(UART0, DOCK);     }
void roomba_clean(void)     { roomba_startup(); uart_tx_one_char(UART0, CLEAN);    }
void roomba_goForward(void) { roomba_drive( speed,0x8000);       } // START, CONTROL, CONTROL, DRIVE, 0, 0xc8, 0x80, 0 };
void roomba_goBackward(void){ roomba_drive(-speed,0x8000);       } // START, CONTROL, CONTROL, DRIVE, 0xff, 0x38, 0x80, 0 };    
void roomba_spinRight(void) { roomba_drive(speed, 0xffff);       }
void roomba_spinLeft(void)  { roomba_drive(speed, 1);            }
void roomba_stop(void)      { roomba_drive(0, 0);                }

void roomba_timer_stop_func(void *arg)  { roomba_stop(); }


void roomba_drive(unsigned int velocity, unsigned int radius)
{
    roomba_startup(); 
    uart_tx_one_char(UART0, DRIVE);
    uart_tx_one_char(UART0, velocity / 256);
    uart_tx_one_char(UART0, velocity);
    uart_tx_one_char(UART0, radius / 256);
    uart_tx_one_char(UART0, radius);
}

void roomba_goDistanceForward(unsigned int cm)
{
    roomba_goForward();    
    setTimerStop(cm * (int)10000/speed);
}

void roomba_goDistanceBackward(unsigned int cm)
{    
    roomba_goBackward();
    setTimerStop(cm * (int)10000/speed);
}


void roomba_turnLeft(unsigned int degrees)
{          
    float ftime = (float)10.3 * degrees;
    int itime =abs(ftime);
    roomba_spinLeft();  
    setTimerStop(itime);    
}

void roomba_turnRight(unsigned int degrees)
{
    float ftime = (float)10.3 * degrees;
    int itime =abs(ftime);      
    roomba_spinRight();
    setTimerStop(itime);      
}




//******************************************************************************
void ICACHE_FLASH_ATTR
at_user_cmd(void *arg, char *pdata, unsigned short len)
{    
    struct espconn *pespconn = (struct espconn *)arg;
    at_linkConType *linkTemp = (at_linkConType *)pespconn->reverse;
    char temp[256];
       
    unsigned char i,count,command,anzahl;
    unsigned int j,value;          
    unsigned char ip[4], wert;
    unsigned char pdata_up[32];
    unsigned char param[32];
           
    
    os_sprintf(temp, "HTTP/1.0 200 OK\r\n\r\n");        
    espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

    // erste 32 Zeichen in Grossbuchstaben fuer Vergleich in extra Buffer ablegen
    for (i=0;i<32;i++) {        
        if (pdata[i] == 0) break;
        wert = pdata[i];
        if (wert >= 'a' && wert <='z') wert = wert - 32;
        pdata_up[i] = wert;
    }
    
    specialAtState = FALSE;
    
    
    // als erstes stehen lassen !
    /**************************************************************************     
     * WLAN Parameter setzen
     * Reset wird ausgeloest
     *           111111
     * 0123456789012345  
     * GET /wlan=ssid,passw      
     *********************************************************************** */ 
    
    if (os_memcmp(pdata_up+5,"WLAN=",5) == 0)
    {        
        unsigned char i, j, wert;
        char ssid[32] = {0}; 
        char password[64] = {0}; 
        struct station_config stationConf;
        
        j = 10;      // Start im String  
        for (i=0;i<32;i++)
        {
            wert = pdata[j++];
            if (wert == ',' || wert == 0 || wert == ' ') break;
            ssid[i] = wert;
        }
       
        for (i=0;i<64;i++)
        {
            wert = pdata[j++];
            if (wert == ',' || wert == 0 || wert == ' ') break;
            password[i] = wert;
        }            
    
        // todo: Ausgaben werden nicht mehr angezeigt...      
        
        // Set ap settings
        os_memcpy(&stationConf.ssid, ssid, 32);
        os_memcpy(&stationConf.password, password, 64);
        wifi_station_set_config(&stationConf);
        
        os_sprintf(temp, "saved, restart now...");        
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));        
        
        espconn_disconnect(pLink[linkTemp->linkId].pCon);  
        
        os_delay_us(1*1000*1000);
        wifi_set_opmode(STATION_MODE);
        system_restart();
        //return;    
    }
    
    
    // -------------------------------------------------------------------------
    // nicht weiter im Station Mode (Sicherheit !!!)   
    if (at_wifiMode == STATIONAP_MODE) return;
    // -------------------------------------------------------------------------
    
    
   /**************************************************************************     
     * Version
     * 
     *           111111
     * 0123456789012345  
     * GET /ver
     * GET /version
     *********************************************************************** */     
    if (os_memcmp(pdata_up+5,"VER",3) == 0)
    {                 
        os_sprintf(temp,"Version vom: " __DATE__ " "  __TIME__ "\r\n");            
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));  
        return;
    }
    
    /**************************************************************************     
     * Update von Server x.x.x.x
     * 
     *           111111
     * 0123456789012345  
     * GET /upd=192.168.0.205
     *********************************************************************** */     
    if (os_memcmp(pdata_up+5,"UPD=",4) == 0)
    {        
        os_sprintf(temp, "update started\r\n");        
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

        // IP Adresse auslesen ab Stelle 9
        count = 9;
        for (i=0;i<4;i++)
        {
            ip[i] = (pdata[count++] - '0');
            while (pdata[count] >= '0' && pdata[count] <= '9')
            {
                ip[i] *=10;
                ip[i] += pdata[count++] - '0';
            }
            count++;
        }
        
        os_sprintf(temp,"ServerIP: %d.%d.%d.%d\r\n",ip[0],ip[1],ip[2],ip[3]);            
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));              
            
        at_exeCmdUpdateCustom(0,ip[0],ip[1],ip[2],ip[3]);
        
        return;
    }

    
            
    /**************************************************************************     
     * Roomba CMD
     * 
     *           111111
     * 0123456789012345  
     * GET /cmd=128:131:135:0:0    
     *********************************************************************** */     
      
    if (os_memcmp(pdata_up+5,"CMD=",4) == 0)
    {                             
        // Uebergebene Parameter einlesen
        count = 9; // ab Position
        for (i=0;i<32;i++) 
        {
            param[i] = 0; // wird benoetigt !
            while (pdata[count] >= '0' && pdata[count] <= '9')
            {
                param[i] *=10;
                param[i] += pdata[count++] - '0';                
            }         
            anzahl++;
            if (pdata[count] != ':') break; // auf Trennzeichen ":" pruefen, ansonsten raus            
            count++;
        }       
                
        feedwdt();
        command = param[0];
        value = param[1] * 256L + param[2];  // 16 Bit
        switch (command)
        {
            case myReset:               system_restart();
            
            case WAKEUP:                roomba_wakeup();                    break;                            
            case mySTOP:                roomba_stop();                      break;                
            case myDOCK:                roomba_dock();                      break;                            
            case myCLEAN:               roomba_clean();                     break;                            
            case goDistanceForward:     roomba_goDistanceForward(value);    break;
            case goDistanceBackward:    roomba_goDistanceBackward(value);   break;   
            case turnRight:             roomba_turnRight(value);            break;
            case turnLeft:              roomba_turnLeft(value);             break;
            
               
            case SENSORS:   break;      // hier nur abfangen
                
            // alle uebergebenen Parameter ausgeben an Roomba
            default: 
                for (i=0;i<anzahl;i++)
                {                              
                    uart_tx_one_char(UART0, param[i]);                       
                    feedwdt();
                }                        
                break;
        }
             
        if (command == SENSORS)
        {
            switch (param[1])
            {
                case SENSORS_ALL:   j = 26; break;
                case SENSORS_POWER: j = 10; break;
                
                default: j = 0;
            }
                   
            os_sprintf(temp, "OK:ROOMBA:%d:",ser[64]); // Anzahl
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));

            for (i=0;i<j;i++)
            {
                feedwdt();
                os_sprintf(temp, "%d:",ser[i]);
                espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
            }   
        }
        else
        {
            os_sprintf(temp, "OK:ROOMBA:CMD");        
            espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
        }
        
        specialAtState = FALSE;
        
        feedwdt();
        
        return;      
    }
    
    /**************************************************************************     
     * Roomba webseite
     * 
     *           111111
     * 0123456789012345 
       GET / 
     * GET /web 
     * GET /web?c=1
     *********************************************************************** */    
    
    // http://192.168.0.213    
    // http://192.168.0.213/web
    
    
    if (os_memcmp(pdata_up+5,"WEB",3) == 0  || pdata[5] == ' ')
    {        
        os_sprintf(temp, "<html><body><h1>Roomba</h1>");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
        
        os_sprintf(temp, "Version: " __DATE__ " " __TIME__ "<br/>");        
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
        
        os_sprintf(temp, "Uptime: %d Tag(e), %d Stunde(n), %d Minute(n), %d Sekunde(n)<br/><br/>",                
                    mytimer_days, mytimer_hours, mytimer_minutes, mytimer_seconds);     
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
        
        os_sprintf(temp, "<form method=\"get\" action=\"web\">");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));        
                
        os_sprintf(temp, "<button style=\"width:100px; height:25px\" name=\"c\" type=\"submit\" value=\"7\">Clean</button><br>");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
        
        os_sprintf(temp, "<button style=\"width:100px; height:25px\" name=\"c\" type=\"submit\" value=\"6\">Dock</button><br>");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
        
        os_sprintf(temp, "<button style=\"width:100px; height:25px\" name=\"c\" type=\"submit\" value=\"5\">Stop</button><br>");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
        
        os_sprintf(temp, "<button style=\"width:100px; height:25px\" name=\"c\" type=\"submit\" value=\"9\">Wartung</button><br>");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp)); 
        
        os_sprintf(temp, "<button style=\"width:100px; height:25px\" name=\"c\" type=\"submit\" value=\"w\">WakeUp</button><br>");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp)); 
        
        os_sprintf(temp, "<br><button style=\"width:200px; height:25px\" name=\"c\" type=\"submit\" value=\"8\">Sensoren aktualisieren</button><br>");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp)); 
                
        os_sprintf(temp, "<table border=0>");  espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));  
                
        os_sprintf(temp, "<tr><td>Status    </td><td align=right>%d</td><td align=left>   </td></tr>" ,ser[SENSORS_POWER_CHARGINGSTATE]);                                       espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));  
        os_sprintf(temp, "<tr><td>Spannung  </td><td align=right>%d</td><td align=left>mV </td></tr>" ,ser[SENSORS_POWER_VOLTAGE_HI] *256L + ser[SENSORS_POWER_VOLTAGE_LO]);    espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));           
        os_sprintf(temp, "<tr><td>Temperatur</td><td align=right>%d</td><td align=left>C  </td></tr>" ,ser[SENSORS_POWER_TEMPERATURE]);                                         espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));  
        os_sprintf(temp, "<tr><td>Kapazitaet</td><td align=right>%d</td><td align=left>mAh</td></tr>" ,ser[SENSORS_POWER_CAPACITY_HI] *256L + ser[SENSORS_POWER_CAPACITY_LO]);  espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));  
        os_sprintf(temp, "<tr><td>Ladung    </td><td align=right>%d</td><td align=left>mAh</td></tr>" ,ser[SENSORS_POWER_CHARGE_HI] *256L + ser[SENSORS_POWER_CHARGE_LO]);      espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));  
        
        
        value = ser[SENSORS_POWER_CURRENT_HI] * 256L + ser[SENSORS_POWER_CURRENT_LO];  
        if (value > 32768) 
        {
            value = (65536 - value);
            os_sprintf(temp, "<tr><td>Strom</td><td align=right>-%d</td><td align=left>mA</td></tr>"     ,value);
        } else
            os_sprintf(temp, "<tr><td>Strom</td><td align=right>%d</td><td align=left>mA</td></tr>"      ,value);
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));             
        
        
        os_sprintf(temp, "</form></body></html>\r\n");
        espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));
        
        // 000000000011111
        // 012345678901234
        // GET /web?c=7
        
        // sofort ausführen
        if (pdata[9] == 'c')
        {
            roomba_startup();
            command = pdata[11];
            switch (command)
            {
                case '7': roomba_clean(); break;
                case '6': roomba_dock();  break;  
                case '5': roomba_stop();  break;   
                case '9': roomba_goDistanceBackward(130);   break;     // 130cm rueckwaerts fahren         
                case 'w': roomba_wakeup;  break;
            }
        }
                
        specialAtState = FALSE;
        
        feedwdt();
        return;
    }
    
    
    //--------------------------------------------------------------------------
    // letzter Eintrag, nicht gefunden, 404 zurueckgeben
    // todo: gibt noch keinen echten 404 Error zurueck
    os_sprintf(temp, "HTTP/1.0 404 Not Found\r\n\r\n404 Not Found\r\n");  
    espconn_sent(pLink[linkTemp->linkId].pCon, temp, strlen(temp));        
}

/**
  * @brief  Client send over callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
static void ICACHE_FLASH_ATTR
at_tcpclient_sent_cb(void *arg)
{
//	os_free(at_dataLine);
//  os_printf("send_cb\r\n");
  if(IPMODE == TRUE)
  {
    ipDataSendFlag = 0;
  	os_timer_disarm(&at_delayChack);
  	os_timer_arm(&at_delayChack, 20, 0);
  	system_os_post(at_recvTaskPrio, 0, 0); ////
    ETS_UART_INTR_ENABLE();
    return;
  }
	uart0_sendStr("\r\nSEND OK\r\n");
  specialAtState = TRUE;
  at_state = at_statIdle;
}

/**
  * @brief  Tcp client connect success callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
static void ICACHE_FLASH_ATTR
at_tcpclient_connect_cb(void *arg)
{
  struct espconn *pespconn = (struct espconn *)arg;
  at_linkConType *linkTemp = (at_linkConType *)pespconn->reverse;

  os_printf("tcp client connect\r\n");
  os_printf("pespconn %p\r\n", pespconn);

  linkTemp->linkEn = TRUE;
  linkTemp->teType = teClient;
  linkTemp->repeaTime = 0;
  espconn_regist_disconcb(pespconn, at_tcpclient_discon_cb);
  espconn_regist_recvcb(pespconn, at_tcpclient_recv);////////
  espconn_regist_sentcb(pespconn, at_tcpclient_sent_cb);///////

  mdState = m_linked;
//  at_linkNum++;
  at_backOk;
  uart0_sendStr("Linked\r\n");
  specialAtState = TRUE;
  at_state = at_statIdle;
}

/**
  * @brief  Tcp client connect repeat callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
static void ICACHE_FLASH_ATTR
at_tcpclient_recon_cb(void *arg, sint8 errType)
{
  struct espconn *pespconn = (struct espconn *)arg;
  at_linkConType *linkTemp = (at_linkConType *)pespconn->reverse;
  struct ip_info ipconfig;
  os_timer_t sta_timer;

  os_printf("at_tcpclient_recon_cb %p\r\n", arg);

  if(linkTemp->teToff == TRUE)
  {
    linkTemp->teToff = FALSE;
    linkTemp->repeaTime = 0;
    if(pespconn->proto.tcp != NULL)
    {
      os_free(pespconn->proto.tcp);
    }
    os_free(pespconn);
    linkTemp->linkEn = false;
    at_linkNum--;
    if(at_linkNum == 0)
    {
      at_backOk;
      mdState = m_unlink; //////////////////////
     // uart0_sendStr("Unlink\r\n");
      disAllFlag = false;
      specialAtState = TRUE;
      at_state = at_statIdle;
    }
  }
  else
  {
    linkTemp->repeaTime++;
    if(linkTemp->repeaTime >= 1)
    {
      os_printf("repeat over %d\r\n", linkTemp->repeaTime);
//      specialAtState = TRUE;
//      at_state = at_statIdle;
      linkTemp->repeaTime = 0;
//      os_printf("err %d\r\n", errType);
      if(errType == ESPCONN_CLSD)
      {
        at_backOk;
      }
      else
      {
        at_backError;
      }
      if(pespconn->proto.tcp != NULL)
      {
        os_free(pespconn->proto.tcp);
      }
      os_free(pespconn);
      linkTemp->linkEn = false;
      os_printf("disconnect\r\n");
      //  os_printf("con EN? %d\r\n", pLink[0].linkEn);
      at_linkNum--;
      if (at_linkNum == 0)
      {
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
      at_state = at_statIdle;
      return;
    }
    os_printf("link repeat %d\r\n", linkTemp->repeaTime);
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
at_dns_found(const char *name, ip_addr_t *ipaddr, void *arg)
{
  struct espconn *pespconn = (struct espconn *) arg;
  at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;

  if(ipaddr == NULL)
  {
    linkTemp->linkEn = FALSE;
    uart0_sendStr("DNS Fail\r\n");
    specialAtState = TRUE;
    at_state = at_statIdle;
//    device_status = DEVICE_CONNECT_SERVER_FAIL;
    return;
  }

  os_printf("DNS found: %d.%d.%d.%d\n",
            *((uint8 *) &ipaddr->addr),
            *((uint8 *) &ipaddr->addr + 1),
            *((uint8 *) &ipaddr->addr + 2),
            *((uint8 *) &ipaddr->addr + 3));

  if(host_ip.addr == 0 && ipaddr->addr != 0)
  {
    if(pespconn->type == ESPCONN_TCP)
    {
      os_memcpy(pespconn->proto.tcp->remote_ip, &ipaddr->addr, 4);
      espconn_connect(pespconn);
      at_linkNum++;
    }
    else
    {
      os_memcpy(pespconn->proto.udp->remote_ip, &ipaddr->addr, 4);
      espconn_connect(pespconn);
      specialAtState = TRUE;
      at_state = at_statIdle;
      at_linkNum++;
      at_backOk;
    }
  }
}

/**
  * @brief  Setup commad of start client.
  * @param  id: commad id number
  * @param  pPara: AT input param
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_setupCmdCipstart(uint8_t id, char *pPara)
{
  char temp[64];
//  enum espconn_type linkType;
  int8_t len;
  enum espconn_type linkType = ESPCONN_INVALID;
  uint32_t ip = 0;
  char ipTemp[128];
  int32_t port;
  uint8_t linkID;

//  if(mdState != m_unlink)
//  {
//    uart0_sendStr("no ip\r\n");
//    return;
//  }
  if(at_wifiMode == 1)
  {
    if(wifi_station_get_connect_status() != STATION_GOT_IP)
    {
      uart0_sendStr("no ip\r\n");
      return;
    }
  }
  pPara++;
  if(at_ipMux)
  {
    linkID = atoi(pPara);
    pPara++;
    pPara = strchr(pPara, '\"');
  }
  else
  {
    linkID = 0;
  }
  if(linkID >= at_linkMax)
  {
    uart0_sendStr("ID ERROR\r\n");
    return;
  }
  len = at_dataStrCpy(temp, pPara, 6);
  if(len == -1)
  {
    uart0_sendStr("Link typ ERROR\r\n");
    return;
  }
  if(os_strcmp(temp, "TCP") == 0)
  {
    linkType = ESPCONN_TCP;
  }
  else if(os_strcmp(temp, "UDP") == 0)
  {
    linkType = ESPCONN_UDP;
  }
  else
  {
    uart0_sendStr("Link typ ERROR\r\n");
    return;
  }
  pPara += (len+3);
  len = at_dataStrCpy(ipTemp, pPara, 64);
  if(len == -1)
  {
    uart0_sendStr("IP ERROR\r\n");
    return;
  }
  pPara += (len+2);
  if(*pPara != ',')
  {
    uart0_sendStr("ENTRY ERROR\r\n");
    return;
  }
  pPara += (1);
  port = atoi(pPara);

  if(pLink[linkID].linkEn)
  {
    uart0_sendStr("ALREADY CONNECT\r\n");
    return;
  }
  pLink[linkID].pCon = (struct espconn *)os_zalloc(sizeof(struct espconn));
  if (pLink[linkID].pCon == NULL)
  {
    uart0_sendStr("CONNECT FAIL\r\n");
    return;
  }
  pLink[linkID].pCon->type = linkType;
  pLink[linkID].pCon->state = ESPCONN_NONE;
  pLink[linkID].linkId = linkID;
  ip = ipaddr_addr(ipTemp);

  switch(linkType)
  {
  case ESPCONN_TCP:
    pLink[linkID].pCon->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
    pLink[linkID].pCon->proto.tcp->local_port = espconn_port();
    pLink[linkID].pCon->proto.tcp->remote_port = port;

    os_memcpy(pLink[linkID].pCon->proto.tcp->remote_ip, &ip, 4);

    pLink[linkID].pCon->reverse = &pLink[linkID];

    espconn_regist_connectcb(pLink[linkID].pCon, at_tcpclient_connect_cb);
    espconn_regist_reconcb(pLink[linkID].pCon, at_tcpclient_recon_cb);
    specialAtState = FALSE;
    if((ip == 0xffffffff) && (os_memcmp(ipTemp,"255.255.255.255",16) != 0))
    {
      espconn_gethostbyname(pLink[linkID].pCon, ipTemp, &host_ip, at_dns_found);
    }
    else
    {
      espconn_connect(pLink[linkID].pCon);
      at_linkNum++;
    }
    break;

  case ESPCONN_UDP:
    pLink[linkID].pCon->proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));
    pLink[linkID].pCon->proto.udp->local_port = espconn_port();
    pLink[linkID].pCon->proto.udp->remote_port = port;
    os_memcpy(pLink[linkID].pCon->proto.udp->remote_ip, &ip, 4);

    pLink[linkID].pCon->reverse = &pLink[linkID];
//    os_printf("%d\r\n",pLink[linkID].pCon->proto.udp->local_port);///

    pLink[linkID].linkId = linkID;
    pLink[linkID].linkEn = TRUE;
    pLink[linkID].teType = teClient;
    espconn_regist_recvcb(pLink[linkID].pCon, at_tcpclient_recv);
    espconn_regist_sentcb(pLink[linkID].pCon, at_tcpclient_sent_cb);
    if((ip == 0xffffffff) && (os_memcmp(ipTemp,"255.255.255.255",16) != 0))
    {
      specialAtState = FALSE;
      espconn_gethostbyname(pLink[linkID].pCon, ipTemp, &host_ip, at_dns_found);
    }
    else
    {
      espconn_create(pLink[linkID].pCon);
      at_linkNum++;
      at_backOk;
    }
    break;

  default:
    break;
  }
//  os_sprintf(temp, "%d.%d.%d.%d:%d\r\n",/////
//             IP2STR(&ip), port);/////
//  uart0_sendStr(temp);////
//  at_backOk;/////
}

/**
  * @brief  Tcp client disconnect success callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
static void ICACHE_FLASH_ATTR
at_tcpclient_discon_cb(void *arg)
{
  struct espconn *pespconn = (struct espconn *)arg;
  at_linkConType *linkTemp = (at_linkConType *)pespconn->reverse;
  uint8_t idTemp;

  if(pespconn == NULL)
  {
    return;
  }
  if(pespconn->proto.tcp != NULL)
  {
    os_free(pespconn->proto.tcp);
  }
  os_free(pespconn);

  linkTemp->linkEn = FALSE;
  os_printf("disconnect\r\n");
//  os_printf("con EN? %d\r\n", pLink[0].linkEn);
  at_linkNum--;

  if(disAllFlag == FALSE)
  {
    at_backOk;
  }
  if(at_linkNum == 0)
  {
    mdState = m_unlink;//////////////////////
    if(disAllFlag)
    {
      at_backOk;
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

  if(disAllFlag)
  {
    idTemp = linkTemp->linkId + 1;
    for(; idTemp<at_linkMax; idTemp++)
    {
      if(pLink[idTemp].linkEn)
      {
        if(pLink[idTemp].teType == teServer)
        {
          continue;
        }
        if(pLink[idTemp].pCon->type == ESPCONN_TCP)
        {
        	specialAtState = FALSE;
          espconn_disconnect(pLink[idTemp].pCon);
        	break;
        }
        else
        {
          pLink[idTemp].linkEn = FALSE;
          espconn_delete(pLink[idTemp].pCon);
          os_free(pLink[idTemp].pCon->proto.udp);
          os_free(pLink[idTemp].pCon);
          at_linkNum--;
          if(at_linkNum == 0)
          {
            mdState = m_unlink;
            at_backOk;
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
  at_state = at_statIdle;
}

/**
  * @brief  Test commad of close ip link.
  * @param  id: commad id number
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_testCmdCipclose(uint8_t id)
{
  at_backOk;
}

/**
  * @brief  Setup commad of close ip link.
  * @param  id: commad id number
  * @param  pPara: AT input param
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_setupCmdCipclose(uint8_t id, char *pPara)
{
  char temp[64];
  uint8_t linkID;
  uint8_t i;

//  if(mdState == m_linked)
//  {
//
//  }
  pPara++;
  if(at_ipMux == 0)
  {
    uart0_sendStr("MUX=0\r\n");
    return;
  }
  linkID = atoi(pPara);
  if(linkID > at_linkMax)
  {
    at_backError;
    return;
  }
  if(linkID == at_linkMax)
  {
    if(serverEn)
    {
      /* restart */
      uart0_sendStr("we must restart\r\n");
      return;
    }
    for(linkID=0; linkID<at_linkMax; linkID++)
    {
      if(pLink[linkID].linkEn)
      {
        if(pLink[linkID].pCon->type == ESPCONN_TCP)
        {
          pLink[linkID].teToff = TRUE;
          specialAtState = FALSE;
//          pLink[linkID].linkEn = FALSE;
          espconn_disconnect(pLink[linkID].pCon);
          disAllFlag = TRUE;
//          os_free(pLink[linkID].pCon);
//          at_linkNum--;
//          if(at_linkNum == 0)
//          {
//            mdState = m_unlink;
//            uart0_sendStr("Unlink\r\n");
//          }
          break;
        }
        else
        {
          pLink[linkID].linkEn = FALSE;
          espconn_delete(pLink[linkID].pCon);
          os_free(pLink[linkID].pCon->proto.udp);///
          os_free(pLink[linkID].pCon);
          at_linkNum--;
          if(at_linkNum == 0)
          {
            mdState = m_unlink;
            at_backOk;
         //   uart0_sendStr("Unlink\r\n");
          }
        }
      }
    }
  }
  else
  {
    if(pLink[linkID].linkEn == FALSE)
    {
      uart0_sendStr("link is not\r\n");
      return;
    }
    if(pLink[linkID].teType == teServer)
    {
      if(pLink[linkID].pCon->type == ESPCONN_TCP)
      {
        pLink[linkID].teToff = TRUE;
        specialAtState = FALSE;
        espconn_disconnect(pLink[linkID].pCon);
      }
      else
      {
        pLink[linkID].linkEn = FALSE;
        espconn_delete(pLink[linkID].pCon);
        at_linkNum--;
        at_backOk;
        if(at_linkNum == 0)
        {
          mdState = m_unlink;
        //  uart0_sendStr("Unlink\r\n");
        }
      }
    }
    else
    {
      if(pLink[linkID].pCon->type == ESPCONN_TCP)
      {
        pLink[linkID].teToff = TRUE;
        specialAtState = FALSE;
        espconn_disconnect(pLink[linkID].pCon);
      }
      else
      {
        pLink[linkID].linkEn = FALSE;
        espconn_delete(pLink[linkID].pCon);
        os_free(pLink[linkID].pCon->proto.udp);
        os_free(pLink[linkID].pCon);
        at_linkNum--;
        at_backOk;
        if(at_linkNum == 0)
        {
          mdState = m_unlink;
    //      uart0_sendStr("Unlink\r\n");
        }
      }
    }
  }
//    if(pLink[linkID].pCon->type == ESPCONN_TCP)
//    {
//      specialAtState = FALSE;
//      espconn_disconnect(pLink[linkID].pCon);
//    }
//    else
//    {
//      pLink[linkID].linkEn = FALSE;
//      espconn_disconnect(pLink[linkID].pCon);
//      os_free(pLink[linkID].pCon->proto.udp);
//      os_free(pLink[linkID].pCon);
//      at_linkNum--;
//      at_backOk;
//      if(at_linkNum == 0)
//      {
//        mdState = m_unlink;
//        uart0_sendStr("Unlink\r\n");
//      }
//    }

//  specialAtState = FALSE;
}

/**
  * @brief  Execution commad of close ip link.
  * @param  id: commad id number
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_exeCmdCipclose(uint8_t id)
{
  char temp[64];

//  if(mdState == m_linked)
//  {
//
//  }
  if(at_ipMux)
  {
    uart0_sendStr("MUX=1\r\n");
    return;
  }
  if(pLink[0].linkEn)
  {
    if(serverEn)
    {
      /* restart */
      uart0_sendStr("we must restart\r\n");
      return;
    }
    else
    {
      if(pLink[0].pCon->type == ESPCONN_TCP)
      {
        specialAtState = FALSE;
        espconn_disconnect(pLink[0].pCon);
      }
      else
      {
        pLink[0].linkEn = FALSE;
        espconn_delete(pLink[0].pCon);
        os_free(pLink[0].pCon->proto.udp);
        os_free(pLink[0].pCon);
        at_linkNum--;
        if(at_linkNum == 0)
        {
          mdState = m_unlink;
          at_backOk;
      //    uart0_sendStr("Unlink\r\n");
        }
      }
    }
  }
  else
  {
    at_backError;
  }
}

/**
  * @brief  Test commad of send ip data.
  * @param  id: commad id number
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_testCmdCipsend(uint8_t id)
{
  at_backOk;
}

char * ICACHE_FLASH_ATTR
at_checkLastNum(char *pPara, uint8_t maxLen)
{
  int8_t ret = -1;
  char *pTemp;
  uint8_t i;

  pTemp = pPara;
  for(i=0;i<maxLen;i++)
  {
    if((*pTemp > '9')||(*pTemp < '0'))
    {
      break;
    }
    pTemp++;
  }
  if(i == maxLen)
  {
    return NULL;
  }
  else
  {
    return pTemp;
  }
}
/**
  * @brief  Setup commad of send ip data.
  * @param  id: commad id number
  * @param  pPara: AT input param
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_setupCmdCipsend(uint8_t id, char *pPara)
{
//  char temp[64];
//  uint8_t len;

//  if(mdState != m_linked)
//  {
//    uart0_sendStr("link is not\r\n");
//    return;
//  }
  if(IPMODE == TRUE)
  {
    uart0_sendStr("IPMODE=1\r\n");
    at_backError;
    return;
  }
  pPara++;
  if(at_ipMux)
  {
    sendingID = atoi(pPara);
    if(sendingID >= at_linkMax)
    {
      at_backError;
      return;
    }
    pPara++;
    if(*pPara != ',') //ID must less 10
    {
      at_backError;
      return;
    }
    pPara++;
  }
  else
  {
    sendingID = 0;
  }
  if(pLink[sendingID].linkEn == FALSE)
  {
    uart0_sendStr("link is not\r\n");
    return;
  }
  at_sendLen = atoi(pPara);
  if(at_sendLen > 2048)
  {
    uart0_sendStr("too long\r\n");
    return;
  }
  pPara = at_checkLastNum(pPara, 5);
  if((pPara == NULL)||(*pPara != '\r'))
  {
    uart0_sendStr("type error\r\n");
    return;
  }
//  at_dataLine = (uint8_t *)os_zalloc(sizeof(uint8_t)*at_sendLen);
//  if(at_dataLine == NULL)
//  {
//  	at_backError;
//  	return;
//  }
  pDataLine = at_dataLine;
//  pDataLine = UartDev.rcv_buff.pRcvMsgBuff;
  specialAtState = FALSE;
  at_state = at_statIpSending;
  uart0_sendStr("> "); //uart0_sendStr("\r\n>");
}

/**
  * @brief  Send data through ip.
  * @param  pAtRcvData: point to data
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_ipDataSending(uint8_t *pAtRcvData)
{
  espconn_sent(pLink[sendingID].pCon, pAtRcvData, at_sendLen);
  //bug if udp,send is ok
//  if(pLink[sendingID].pCon->type == ESPCONN_UDP)
//  {
//    uart0_sendStr("\r\nSEND OK\r\n");
//    specialAtState = TRUE;
//    at_state = at_statIdle;
//  }
}

/**
  * @brief  Transparent data through ip.
  * @param  arg: no used
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_ipDataTransparent(void *arg)
{
	if(at_state != at_statIpTraning)
	{
		return;
	}
//	if(ipDataSendFlag)
//	{
//	  return;
//	}
//	ETS_UART_INTR_DISABLE(); //
	os_timer_disarm(&at_delayChack);
	if((at_tranLen == 3) && (os_memcmp(at_dataLine, "+++", 3) == 0)) //UartDev.rcv_buff.pRcvMsgBuff
	{
//	  ETS_UART_INTR_DISABLE(); //
		specialAtState = TRUE;
    at_state = at_statIdle;
//		ETS_UART_INTR_ENABLE();
//		IPMODE = FALSE;
		return;
	}
	else if(at_tranLen)
	{
	  ETS_UART_INTR_DISABLE(); //
    espconn_sent(pLink[0].pCon, at_dataLine, at_tranLen); //UartDev.rcv_buff.pRcvMsgBuff ////
    ipDataSendFlag = 1;
//    pDataLine = UartDev.rcv_buff.pRcvMsgBuff;
    pDataLine = at_dataLine;
  	at_tranLen = 0;
  	return;
  }
  os_timer_arm(&at_delayChack, 20, 0);
//  system_os_post(at_recvTaskPrio, 0, 0); ////
//  ETS_UART_INTR_ENABLE();
}

/**
  * @brief  Send data through ip.
  * @param  pAtRcvData: point to data
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_ipDataSendNow(void)
{
//  if(ipDataSendFlag)
//  {
//    return;
//  }
  espconn_sent(pLink[0].pCon, at_dataLine, at_tranLen); //UartDev.rcv_buff.pRcvMsgBuff ////
  ipDataSendFlag = 1;
  pDataLine = at_dataLine;
  at_tranLen = 0;
}

/**
  * @brief  Setup commad of send ip data.
  * @param  id: commad id number
  * @param  pPara: AT input param
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_exeCmdCipsend(uint8_t id)
{
	if((serverEn) || (IPMODE == FALSE))
	{
            
		at_backError;
		return;
	}
	if(pLink[0].linkEn == FALSE)
  {
          
	  at_backError;
	  return;
  }
	pDataLine = at_dataLine;//UartDev.rcv_buff.pRcvMsgBuff;
	at_tranLen = 0;
  specialAtState = FALSE;
  at_state = at_statIpTraning;
  os_timer_disarm(&at_delayChack);
  os_timer_setfn(&at_delayChack, (os_timer_func_t *)at_ipDataTransparent, NULL);
  os_timer_arm(&at_delayChack, 20, 0);
//  IPMODE = TRUE;
  uart0_sendStr("\r\n>");
}

/**
  * @brief  Query commad of set multilink mode.
  * @param  id: commad id number
  * @param  pPara: AT input param
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_queryCmdCipmux(uint8_t id)
{
  char temp[32];
  os_sprintf(temp, "%s:%d\r\n",
             at_fun[id].at_cmdName, at_ipMux);
  uart0_sendStr(temp);
  at_backOk;
}

/**
  * @brief  Setup commad of set multilink mode.
  * @param  id: commad id number
  * @param  pPara: AT input param
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_setupCmdCipmux(uint8_t id, char *pPara)
{
  uint8_t muxTemp;
  if(mdState == m_linked)
  {
    uart0_sendStr("link is builded\r\n");
    return;
  }
  pPara++;
  muxTemp = atoi(pPara);
  if(muxTemp == 1)
  {
    at_ipMux = TRUE;
  }
  else if(muxTemp == 0)
  {
    at_ipMux = FALSE;
  }
  else
  {
    at_backError;
    return;
  }
  at_backOk;
}

//static void ICACHE_FLASH_ATTR
//user_tcp_discon_cb(void *arg)
//{
//  struct espconn *pespconn = (struct espconn *)arg;
//
//  if(pespconn == NULL)
//  {
//    return;
//  }
////  if(pespconn->proto.tcp != NULL)
////  {
////    os_free(pespconn->proto.tcp);
////  }
////  os_free(pespconn);
////  pServerCon = NULL;
//  os_printf("disconnect\r\n");
//  os_printf("pespconn %p\r\n", pespconn);
//}

/**
  * @brief  Tcp server disconnect success callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
static void ICACHE_FLASH_ATTR
at_tcpserver_discon_cb(void *arg)
{
  struct espconn *pespconn = (struct espconn *) arg;
  at_linkConType *linkTemp = (at_linkConType *) pespconn->reverse;

  os_printf("S conect C: %p\r\n", arg);

  if (pespconn == NULL)
  {
    return;
  }

  linkTemp->linkEn = FALSE;
  linkTemp->pCon = NULL;
  os_printf("con EN? %d\r\n", linkTemp->linkId);
  if(linkTemp->teToff == TRUE)
  {
    linkTemp->teToff = FALSE;
    specialAtState = true;
    at_state = at_statIdle;
    at_backOk;
  }
  at_linkNum--;
  if (at_linkNum == 0)
  {
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
at_tcpserver_recon_cb(void *arg, sint8 errType)
{
  struct espconn *pespconn = (struct espconn *)arg;
  at_linkConType *linkTemp = (at_linkConType *)pespconn->reverse;

  os_printf("S conect C: %p\r\n", arg);

  if(pespconn == NULL)
  {
    return;
  }

  linkTemp->linkEn = false;
  linkTemp->pCon = NULL;
  os_printf("con EN? %d\r\n", linkTemp->linkId);
  at_linkNum--;
  if (at_linkNum == 0)
  {
    mdState = m_unlink; //////////////////////

  //  uart0_sendStr("Unlink\r\n");
    disAllFlag = false;
  }
  if(linkTemp->teToff == TRUE)
  {
    linkTemp->teToff = FALSE;
    specialAtState = true;
    at_state = at_statIdle;
    at_backOk;
  }
}

/**
  * @brief  Tcp server listend callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
LOCAL void ICACHE_FLASH_ATTR
at_tcpserver_listen(void *arg)
{
  struct espconn *pespconn = (struct espconn *)arg;
  uint8_t i;

  os_printf("get tcpClient:\r\n");
  for(i=0;i<at_linkMax;i++)
  {
    if(pLink[i].linkEn == FALSE)
    {
      pLink[i].linkEn = TRUE;
      break;
    }
  }
  if(i>=5)
  {
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
  espconn_regist_sentcb(pespconn, at_tcpclient_sent_cb);///////
 // uart0_sendStr("Link\r\n");
}

/**
  * @brief  Udp server receive data callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
LOCAL void ICACHE_FLASH_ATTR
at_udpserver_recv(void *arg, char *pusrdata, unsigned short len)
{
  struct espconn *pespconn = (struct espconn *)arg;
  at_linkConType *linkTemp;
  char temp[32];
  uint8_t i;

  os_printf("get udpClient:\r\n");

  if(pespconn->reverse == NULL)
  {
    for(i = 0;i < at_linkMax;i++)
    {
      if(pLink[i].linkEn == FALSE)
      {
        pLink[i].linkEn = TRUE;
        break;
      }
    }
    if(i >= 5)
    {
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
  linkTemp = (at_linkConType *)pespconn->reverse;
  if(pusrdata == NULL)
  {
    return;
  }
  os_sprintf(temp, "\r\n+IPD,%d,%d:",
             linkTemp->linkId, len);
  uart0_sendStr(temp);
  uart0_tx_buffer(pusrdata, len);
  at_backOk;
}

/**
  * @brief  Setup commad of module as server.
  * @param  id: commad id number
  * @param  pPara: AT input param
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_setupCmdCipserver(uint8_t id, char *pPara)
{
  BOOL serverEnTemp;
  int32_t port;
  char temp[32];

  if(at_ipMux == FALSE)
  {
    at_backError;
    return ;
  }
  pPara++;
  serverEnTemp = atoi(pPara);
  pPara++;
  if(serverEnTemp == 0)
  {
    if(*pPara != '\r')
    {
      at_backError;
      return ;
    }
  }
  else if(serverEnTemp == 1)
  {
    if(*pPara == ',')
    {
      pPara++;
      port = atoi(pPara);
    }
    else
    {
      port = 333;
    }
  }
  else
  {
    at_backError;
    return ;
  }
  if(serverEnTemp == serverEn)
  {
    uart0_sendStr("no change\r\n");
    return;
  }

  if(serverEnTemp)
  {
    pTcpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
    if (pTcpServer == NULL)
    {
      uart0_sendStr("TcpServer Failure\r\n");
      return ;
    }
    pTcpServer->type = ESPCONN_TCP;
    pTcpServer->state = ESPCONN_NONE;
    pTcpServer->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
    pTcpServer->proto.tcp->local_port = port;
    espconn_regist_connectcb(pTcpServer, at_tcpserver_listen);
    espconn_accept(pTcpServer);
    espconn_regist_time(pTcpServer, server_timeover, 0);

    pUdpServer = (struct espconn *)os_zalloc(sizeof(struct espconn));
    if (pUdpServer == NULL)
    {
      uart0_sendStr("UdpServer Failure\r\n");
      return;
    }
    pUdpServer->type = ESPCONN_UDP;
    pUdpServer->state = ESPCONN_NONE;
    pUdpServer->proto.udp = (esp_udp *)os_zalloc(sizeof(esp_udp));
    pUdpServer->proto.udp->local_port = port;
    pUdpServer->reverse = NULL;
    espconn_regist_recvcb(pUdpServer, at_udpserver_recv);
    espconn_create(pUdpServer);

//    if(pLink[0].linkEn)
//    {
//      uart0_sendStr("Link is builded\r\n");
//      return;
//    }
//    pLink[0].pCon = (struct espconn *)os_zalloc(sizeof(struct espconn));
//    if (pLink[0].pCon == NULL)
//    {
//      uart0_sendStr("Link buile Failure\r\n");
//      return;
//    }
//    pLink[0].pCon->type = ESPCONN_TCP;
//    pLink[0].pCon->state = ESPCONN_NONE;
//    pLink[0].linkId = 0;
//    pLink[0].linkEn = TRUE;
//
//    pLink[0].pCon->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
//    pLink[0].pCon->proto.tcp->local_port = port;
//
//    pLink[0].pCon->reverse = &pLink[0];
//
//    espconn_regist_connectcb(pLink[0].pCon, user_test_tcpserver_listen);
//    espconn_accept(pLink[0].pCon);
//    at_linkNum++;
  }
  else
  {
    /* restart */
    uart0_sendStr("we must restart\r\n");
    return ;
  }
  serverEn = serverEnTemp;
  at_backOk;
}

/**
  * @brief  Query commad of set transparent mode.
  * @param  id: commad id number
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_queryCmdCipmode(uint8_t id)
{
	char temp[32];

  os_sprintf(temp, "%s:%d\r\n", at_fun[id].at_cmdName, IPMODE);
  uart0_sendStr(temp);
  at_backOk;
}

/**
  * @brief  Setup commad of transparent.
  * @param  id: commad id number
  * @param  pPara: AT input param
  * @retval None
  */
void ICACHE_FLASH_ATTR
at_setupCmdCipmode(uint8_t id, char *pPara)
{
	uint8_t mode;
  char temp[32];

  pPara++;
  if((at_ipMux) || (serverEn))
  {
  	at_backError;
  	return;
  }
  mode = atoi(pPara);
  if(mode > 1)
  {
  	at_backError;
  	return;
  }
  IPMODE = mode;
  at_backOk;
}

void ICACHE_FLASH_ATTR
at_queryCmdCipsto(uint8_t id)
{
  char temp[32];
  os_sprintf(temp, "%s:%d\r\n",
             at_fun[id].at_cmdName, server_timeover);
  uart0_sendStr(temp);
  at_backOk;
}

void ICACHE_FLASH_ATTR
at_setupCmdCipsto(uint8_t id, char *pPara)
{
  char temp[64];
  uint16_t timeOver;

  if(serverEn == FALSE)
  {
    at_backError;
    return;
  }
  pPara++;
  timeOver = atoi(pPara);
  if(timeOver>28800)
  {
    at_backError;
    return;
  }
  if(timeOver != server_timeover)
  {
    server_timeover = timeOver;
    espconn_regist_time(pTcpServer, server_timeover, 0);
  }
  at_backOk;
  return;
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
//static struct esp_platform_saved_param esp_param;

/******************************************************************************
 * FunctionName : user_esp_platform_upgrade_cb
 * Description  : Processing the downloaded data from the server
 * Parameters   : pespconn -- the espconn used to connetion with the host
 * Returns      : none
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
at_upDate_rsp(void *arg)
{
  struct upgrade_server_info *server = arg;
//  struct espconn *pespconn = server->pespconn;
//  uint8 *pbuf = NULL;
//  char *action = NULL;
//  char temp[32];

//  if(pespconn->proto.tcp != NULL)
//  {
//    os_free(pespconn->proto.tcp);
//  }
//  os_free(pespconn);

  if(server->upgrade_flag == true)
  {
//    pbuf = (char *) os_zalloc(2048);
//        ESP_DBG("user_esp_platform_upgarde_successfully\n");
    os_printf("device_upgrade_success\r\n");
//    action = "device_upgrade_success";
    at_backOk;
    system_upgrade_reboot();
//    os_sprintf(pbuf, UPGRADE_FRAME,
//               devkey, action,
//               server->pre_version,
//               server->upgrade_version);
//        ESP_DBG(pbuf);

//    espconn_sent(pespconn, pbuf, os_strlen(pbuf));

//    if(pbuf != NULL)
//    {
//      os_free(pbuf);
//      pbuf = NULL;
//    }
  }
  else
  {
//        ESP_DBG("user_esp_platform_upgrade_failed\n");
    os_printf("device_upgrade_failed\r\n");
//    action = "device_upgrade_failed";
//    os_sprintf(pbuf, UPGRADE_FRAME, devkey, action);
//        ESP_DBG(pbuf);
//    os_sprintf(temp, at_backTeError, 1);
//    uart0_sendStr(at_backTeError"1\r\n");
    at_backError;
//    espconn_sent(pespconn, pbuf, os_strlen(pbuf));

//    if(pbuf != NULL)
//    {
//      os_free(pbuf);
//      pbuf = NULL;
//    }
  }

  os_free(server->url);
  server->url = NULL;
  os_free(server);
  server = NULL;

//  espconn_disconnect(pespconn);
  specialAtState = TRUE;
  at_state = at_statIdle;
}

///******************************************************************************
// * FunctionName : user_esp_platform_load_param
// * Description  : load parameter from flash, toggle use two sector by flag value.
// * Parameters   : param--the parame point which write the flash
// * Returns      : none
//*******************************************************************************/
//void ICACHE_FLASH_ATTR
//user_esp_platform_load_param(struct esp_platform_saved_param *param)
//{
//    struct esp_platform_sec_flag_param flag;
//
//    load_user_param(ESP_PARAM_SEC_FLAG, 0, &flag, sizeof(struct esp_platform_sec_flag_param));
//
//    if (flag.flag == 0) {
//        load_user_param(ESP_PARAM_SAVE_SEC_0, 0, param, sizeof(struct esp_platform_saved_param));
//    } else {
//        load_user_param(ESP_PARAM_SAVE_SEC_1, 0, param, sizeof(struct esp_platform_saved_param));
//    }
//}

/**
  * @brief  Tcp client disconnect success callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
static void ICACHE_FLASH_ATTR
at_upDate_discon_cb(void *arg)
{
  struct espconn *pespconn = (struct espconn *)arg;
  uint8_t idTemp;

//  if(pespconn == NULL)
//  {
//    return;
//  }
  if(pespconn->proto.tcp != NULL)
  {
    os_free(pespconn->proto.tcp);
  }
  if(pespconn != NULL)
  {
    os_free(pespconn);
  }

  os_printf("disconnect\r\n");

  if(system_upgrade_start(upServer) == false)
  {
//    uart0_sendStr("+CIPUPDATE:0/r/n");
    at_backError;
    specialAtState = TRUE;
    at_state = at_statIdle;
  }
  else
  {
    uart0_sendStr("+CIPUPDATE:4\r\n");
  }
}

/**
  * @brief  Udp server receive data callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
LOCAL void ICACHE_FLASH_ATTR
at_upDate_recv(void *arg, char *pusrdata, unsigned short len)
{
  struct espconn *pespconn = (struct espconn *)arg;
  char temp[32];
  char *pTemp;
  uint8_t user_bin[9] = {0};
//  uint8_t devkey[41] = {0};
  uint8_t i;

  os_timer_disarm(&at_delayChack);
//  os_printf("get upRom:\r\n");
  uart0_sendStr("+CIPUPDATE:3\r\n");
  
//  os_printf("%s",pusrdata);
  pTemp = (char *)os_strstr(pusrdata,"rom_version\": ");
  if(pTemp == NULL)
  {    
    uart0_sendStr("+CIPUPDATE: error\r\n");
    return;
  }
  pTemp += sizeof("rom_version\": ");
  
//  user_esp_platform_load_param(&esp_param);

  upServer = (struct upgrade_server_info *)os_zalloc(sizeof(struct upgrade_server_info));
  os_memcpy(upServer->upgrade_version, pTemp, 5);
  upServer->upgrade_version[5] = '\0';
  os_sprintf(upServer->pre_version, "v%d.%d", AT_VERSION_main, AT_VERSION_sub);

  upServer->pespconn = pespconn;

//  os_memcpy(devkey, esp_param.devkey, 40);
  os_memcpy(upServer->ip, pespconn->proto.tcp->remote_ip, 4);

  upServer->port = 80;

  upServer->check_cb = at_upDate_rsp;
  upServer->check_times = 60000;

  if(upServer->url == NULL)
  {
    upServer->url = (uint8 *) os_zalloc(512);
  }

  if(system_upgrade_userbin_check() == UPGRADE_FW_BIN1)
  {
      uart0_sendStr("+CIPUPDATE: user2.bin\r\n");
      os_memcpy(user_bin, "user2.bin", 10);
    
  }
  else if(system_upgrade_userbin_check() == UPGRADE_FW_BIN2)
  { 
      uart0_sendStr("+CIPUPDATE: user1.bin\r\n");    
      os_memcpy(user_bin, "user1.bin", 10);
  }
  
  
    os_sprintf(upServer->url,
        "GET /roomba/device/rom/?action=download_rom&version=%s&filename=%s HTTP/1.1\r\nHost: "IPSTR":%d\r\n"pheadbuffer"",
        upServer->upgrade_version, user_bin, IP2STR(upServer->ip),
        upServer->port, KEY);

  

  //  ESP_DBG(upServer->url);

  // HoWL freigegeben  Update own cloud
  espconn_disconnect(pespconn);
}

LOCAL void ICACHE_FLASH_ATTR
at_upDate_wait(void *arg)
{
  struct espconn *pespconn = arg;
  os_timer_disarm(&at_delayChack);
  if(pespconn != NULL)
  {
    espconn_disconnect(pespconn);
  }
  else
  {
    at_backError;
    specialAtState = TRUE;
    at_state = at_statIdle;
  }
}

/******************************************************************************
 * FunctionName : user_esp_platform_sent_cb
 * Description  : Data has been sent successfully and acknowledged by the remote host.
 * Parameters   : arg -- Additional argument to pass to the callback function
 * Returns      : none
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
at_upDate_sent_cb(void *arg)
{
  struct espconn *pespconn = arg;
  os_timer_disarm(&at_delayChack);
  os_timer_setfn(&at_delayChack, (os_timer_func_t *)at_upDate_wait, pespconn);
  os_timer_arm(&at_delayChack, 5000, 0);
  os_printf("at_upDate_sent_cb\r\n");
}

/**
  * @brief  Tcp client connect success callback function.
  * @param  arg: contain the ip link information
  * @retval None
  */
static void ICACHE_FLASH_ATTR
at_upDate_connect_cb(void *arg)
{
  struct espconn *pespconn = (struct espconn *)arg;
  uint8_t user_bin[9] = {0};
//  uint8_t devkey[41] = {0};
  char *temp;

  uart0_sendStr("+CIPUPDATE:2\r\n");

//  user_esp_platform_load_param(&esp_param);
//  os_memcpy(devkey, esp_param.devkey, 40);

  espconn_regist_disconcb(pespconn, at_upDate_discon_cb);
  espconn_regist_recvcb(pespconn, at_upDate_recv);////////
  espconn_regist_sentcb(pespconn, at_upDate_sent_cb);

//  os_printf("at_upDate_connect_cb %p\r\n", arg);

  temp = (uint8 *) os_zalloc(512);

  os_sprintf(temp,"GET /roomba/device/rom/?is_format_simple=true HTTP/1.0\r\nHost: "IPSTR":%d\r\n"pheadbuffer"",
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
at_upDate_recon_cb(void *arg, sint8 errType)
{
  struct espconn *pespconn = (struct espconn *)arg;
//  os_timer_t sta_timer;
//  static uint8_t repeaTime = 0;
//  char temp[32];

//  os_printf("at_upDate_recon_cb %p\r\n", arg);

//  repeaTime++;
//  if(repeaTime >= 3)
//  {
//    os_printf("repeat over %d\r\n", repeaTime);
//    repeaTime = 0;
    at_backError;
    if(pespconn->proto.tcp != NULL)
    {
      os_free(pespconn->proto.tcp);
    }
    os_free(pespconn);
    os_printf("disconnect\r\n");

    if(upServer != NULL)
    {
      os_free(upServer);
      upServer = NULL;
    }
    at_backError;

    specialAtState = TRUE;
    at_state = at_statIdle;
//    return;
//  }
//  os_printf("link repeat %d\r\n", repeaTime);
//  pespconn->proto.tcp->local_port = espconn_port();
//  espconn_connect(pespconn);
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
upServer_dns_found(const char *name, ip_addr_t *ipaddr, void *arg)
{
  struct espconn *pespconn = (struct espconn *) arg;
//  char temp[32];

  if(ipaddr == NULL)
  {
    at_backError;
//    os_sprintf(temp,at_backTeError,2);
//    uart0_sendStr(at_backTeError"2\r\n");
    specialAtState = TRUE;
    at_state = at_statIdle;
//    device_status = DEVICE_CONNECT_SERVER_FAIL;
    return;
  }
  uart0_sendStr("+CIPUPDATE:1\r\n");

//  os_printf("DNS found: %d.%d.%d.%d\n",
//            *((uint8 *) &ipaddr->addr),
//            *((uint8 *) &ipaddr->addr + 1),
//            *((uint8 *) &ipaddr->addr + 2),
//            *((uint8 *) &ipaddr->addr + 3));

  if(host_ip.addr == 0 && ipaddr->addr != 0)
  {
    if(pespconn->type == ESPCONN_TCP)
    {
      os_memcpy(pespconn->proto.tcp->remote_ip, &ipaddr->addr, 4);
      espconn_regist_connectcb(pespconn, at_upDate_connect_cb);
      espconn_regist_reconcb(pespconn, at_upDate_recon_cb);
      espconn_connect(pespconn);

//      at_upDate_connect_cb(pespconn);
    }
  }
}

void ICACHE_FLASH_ATTR
at_exeCmdUpdate(uint8_t id)
{
  pespconn = (struct espconn *)os_zalloc(sizeof(struct espconn));
  pespconn->type = ESPCONN_TCP;
  pespconn->state = ESPCONN_NONE;
  pespconn->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
  pespconn->proto.tcp->local_port = espconn_port();
  pespconn->proto.tcp->remote_port = 80;

  specialAtState = FALSE;
  //espconn_gethostbyname(pespconn, "iot.espressif.cn", &host_ip, upServer_dns_found);
  
  uart0_sendStr("+CIPUPDATE:1b\r\n");
  if(host_ip.addr == 0 )
  {
    *((uint8 *) &pespconn->proto.tcp->remote_ip)     = 192; //Host IP
    *((uint8 *) &pespconn->proto.tcp->remote_ip + 1) = 168;
    *((uint8 *) &pespconn->proto.tcp->remote_ip + 2) = 0;
    *((uint8 *) &pespconn->proto.tcp->remote_ip + 3) = 205;

    espconn_regist_connectcb(pespconn, at_upDate_connect_cb);
    espconn_regist_reconcb(pespconn, at_upDate_recon_cb);
    espconn_connect(pespconn);
  }
}

void at_exeCmdUpdateCustom(uint8_t id,uint8_t ip0,uint8_t ip1,uint8_t ip2,uint8_t ip3)
{    
    pespconn = (struct espconn *)os_zalloc(sizeof(struct espconn));
    pespconn->type = ESPCONN_TCP;
    pespconn->state = ESPCONN_NONE;
    pespconn->proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));
    pespconn->proto.tcp->local_port = espconn_port();
    pespconn->proto.tcp->remote_port = 80;

    specialAtState = FALSE;
    //espconn_gethostbyname(pespconn, "iot.espressif.cn", &host_ip, upServer_dns_found);

    uart0_sendStr("+CIPUPDATE:1b\r\n");

    if(host_ip.addr == 0 )
    {
        *((uint8 *) &pespconn->proto.tcp->remote_ip)     = ip0; //Host IP
        *((uint8 *) &pespconn->proto.tcp->remote_ip + 1) = ip1;
        *((uint8 *) &pespconn->proto.tcp->remote_ip + 2) = ip2;
        *((uint8 *) &pespconn->proto.tcp->remote_ip + 3) = ip3;

        espconn_regist_connectcb(pespconn, at_upDate_connect_cb);
        espconn_regist_reconcb(pespconn, at_upDate_recon_cb);
        espconn_connect(pespconn);
    }
}

/**
  * @}
  */


/*

http://tech.scargill.net/esp8266-and-the-dallas-ds18b20-and-ds18b20p/

* Adaptation of Paul StoffregenÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢s One wire library to the ESP8266 and
* NecromantÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢s Frankenstein firmware by Erland Lewin <erland@lewin.nu>
*
* PaulÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢s original library site:
*   http://www.pjrc.com/teensy/td_libs_OneWire.html
*
* See also http://playground.arduino.cc/Learning/OneWire
*
* Stripped down to bare minimum by Peter Scargill for single DS18B20 or DS18B20P integer read

ds_reset();
ds_write(0xcc,1);
ds_write(0xbe,1);

temperature=(int)ds_read();
temperature=temperature+(int)ds_read()*256;
temperature/=16;
if (temperature>100) temperature-=4096;
 
ds_reset();
ds_write(0xcc,1);
ds_write(0x44,1);

*/
static int gpioPin;

void ICACHE_FLASH_ATTR ds_init(unsigned int gpio)
{            
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5);    //set gpio2 as gpio pin    
    PIN_PULLDWN_DIS(PERIPHS_IO_MUX_GPIO5_U);                //disable pulldown    
    PIN_PULLUP_EN(PERIPHS_IO_MUX_GPIO5_U);                  //enable pull up R
        
    GPIO_DIS_OUTPUT( gpio );
    gpioPin = gpio;
}

// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesnÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢t then it is broken or shorted
// and we return;
void ICACHE_FLASH_ATTR ds_reset(void)
{
    uint8_t retries = 125;
        
    GPIO_DIS_OUTPUT( gpioPin );
    // wait until the wire is highÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ just in case
    do {
        if (--retries == 0) return;
        os_delay_us(2);
    } while ( !GPIO_INPUT_GET( gpioPin ));
    
    GPIO_OUTPUT_SET( gpioPin, 0 );
    
    os_delay_us(480);
    
    GPIO_DIS_OUTPUT( gpioPin ); // allow it to float
        
    os_delay_us(15);
    
    // low presence Impuls vom DS
    
    os_delay_us(480);
}


// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.

static inline void ds_write_bit( int v )
{
    GPIO_OUTPUT_SET( gpioPin, 0 );          // drive output low
    if( v ) {        
        os_delay_us(10);
        GPIO_OUTPUT_SET( gpioPin, 1 );      // drive output high        
        os_delay_us(55);
    } else {        
        os_delay_us(65);
        GPIO_OUTPUT_SET( gpioPin, 1 );      // drive output high        
        os_delay_us(5);
    }
}


// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.

static inline int ds_read_bit(void)
{    
    int r;
    
    GPIO_OUTPUT_SET( gpioPin, 0 );
    os_delay_us(3);
    GPIO_DIS_OUTPUT( gpioPin );    // let pin float, pull up will raise    
    os_delay_us(10);    
    r = GPIO_INPUT_GET( gpioPin );    
    os_delay_us(53);
    return r;
}

// Write a byte
void ICACHE_FLASH_ATTR  ds_write( uint8_t v ) {
    uint8_t bitMask;
    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
        ds_write_bit( (bitMask & v)?1:0);
    }    
    GPIO_DIS_OUTPUT( gpioPin );
    GPIO_OUTPUT_SET( gpioPin, 0 );      
}

// Read a byte
uint8_t ICACHE_FLASH_ATTR ds_read() {
    uint8_t bitMask;
    uint8_t r = 0;
    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
        if ( ds_read_bit()) r |= bitMask;
    }
    return r;
} 