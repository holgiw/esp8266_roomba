#ifndef __AT_H
#define __AT_H

#include "c_types.h"

#define at_recvTaskPrio        0
#define at_recvTaskQueueLen    64


typedef enum{
  m_init,
  m_wact,
  m_gotip,
  m_linked,
  m_unlink,
  m_wdact
}at_mdStateType;


void ser_init(void);

#endif
