#include "at.h"
#include "user_interface.h"
#include "osapi.h"
#include "driver/uart.h"


volatile unsigned char ser[65];
volatile unsigned char ser_count=1;
 
extern void at_ipDataSending(uint8_t *pAtRcvData);
extern void at_ipDataSendNow(void);


os_event_t    at_recvTaskQueue[at_recvTaskQueueLen];


static void at_recvTask(os_event_t *events);

/**
  * @brief  Uart receive task.
  * @param  events: contain the uart receive data
  * @retval None
  */
static void ICACHE_FLASH_ATTR ///////
at_recvTask(os_event_t *events)
{
  
  uint8_t temp;

  while(READ_PERI_REG(UART_STATUS(UART0)) & (UART_RXFIFO_CNT << UART_RXFIFO_CNT_S))
  {
    
    temp = READ_PERI_REG(UART_FIFO(UART0)) & 0xFF;     
      
    if (ser_count>64) ser_count = 0;
    ser[ser_count] = temp;
    ser_count++;
    ser[64] = ser_count;   
    feedwdt();
  }
  
  if(UART_RXFIFO_FULL_INT_ST == (READ_PERI_REG(UART_INT_ST(UART0)) & UART_RXFIFO_FULL_INT_ST))
  {
    WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_FULL_INT_CLR);
  }
  else if(UART_RXFIFO_TOUT_INT_ST == (READ_PERI_REG(UART_INT_ST(UART0)) & UART_RXFIFO_TOUT_INT_ST))
  {
    WRITE_PERI_REG(UART_INT_CLR(UART0), UART_RXFIFO_TOUT_INT_CLR);
  }
  ETS_UART_INTR_ENABLE();
}


/**
  * @brief  Initializes Empfang.
  * @param  None
  * @retval None
  */
void ICACHE_FLASH_ATTR
ser_init(void)
{
  system_os_task(at_recvTask, at_recvTaskPrio, at_recvTaskQueue, at_recvTaskQueueLen);  
}
 
