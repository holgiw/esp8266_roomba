#ifndef __AT_IPCMD_H
#define __AT_IPCMD_H

#define at_linkMax 5

#include "c_types.h"
extern volatile unsigned char ser[65];
extern volatile unsigned char ser_count;


typedef enum
{
  teClient,
  teServer
}teType;

typedef struct
{
	BOOL linkEn;
  BOOL teToff;
	uint8_t linkId;
	teType teType;
	uint8_t repeaTime;
	struct espconn *pCon;
}at_linkConType;

void at_user_cmd(void *arg, char *pdata, unsigned short len);

void at_exeCmdUpdate(uint8_t id);
void at_exeCmdUpdateCustom(uint8_t id,uint8_t ip0,uint8_t ip1,uint8_t ip2,uint8_t ip3);
 
#endif
