#ifndef _RECORD_TASK_H_
#define _RECORD_TASK_H_
/**********************************************************************
   Title                      : record_task.h         
                                                                         
   Module Description         : This file is the task for data records
	
   Author                     : 
   
 *********************************************************************/

typedef enum record_event_tag
{
    RECORD_EVT_NOP,
    RECORD_EVT_FIX,
    RECORD_EVT_WAKETIME,
    
    RECORD_NUM_EVENTS
}record_event_t;

/*********************************************************************/
/* interface for modules                                             */
/*********************************************************************/

extern void Record_Task(void* pvParameters);
extern uint8_t device_unmount_status(void);

/*=======================================================================================*\
 * File Revision History
 *=======================================================================================
 * ----------  ------   ---------------------------------------------
 *
\*=======================================================================================*/
#endif //_GPS_H_
