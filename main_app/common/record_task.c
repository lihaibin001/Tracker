/*----------------------------------------------------------------------------/
 *  (C)Dedao, 2016
 *-----------------------------------------------------------------------------/
 *
 * Copyright (C) 2016, Dedao, all right reserved.
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this condition and the following disclaimer.
 *
 * This software is provided by the copyright holder and contributors "AS IS"
 * and any warranties related to this software are DISCLAIMED.
 * The copyright owner or contributors be NOT LIABLE for any damages caused
 * by use of this software.
 *----------------------------------------------------------------------------*/

/**********************************************************************
   Title                      : record_task.c         
                                                                         
   Module Description         : Handle timing/distance related record tasks.

   Author                     : 
   
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "standard.h"
#include "record_task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gps.h"
#include "GPRS.h"
#include "ATProtocol.h"

#define USE_DEBUG
#include "Debug.h"

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define DEVICE_UNMOUNT_LIMIT (1)
#define AD_TEMP 0 /* temperature channel */
#define GPS_UPLOAD_INTERVAL (85)

// 45s timer for upload first time
#define GPS_UPLOAD_TIME_1 (45000)
// 10s timer for upload second time
#define GPS_UPLOAD_TIME_2 (10000)

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/

/**********************************************************************
 * Function Definitions
 *********************************************************************/
static void prvRecord_evt_nop(int16_t data);
static void prvRecord_evt_fix(int16_t data);
static void prvRecord_evt_waketime(int16_t data);

static uint32_t gps_record_interval(void);
static uint32_t gps_upload_interval(void);
#if 0
static void test_rtc(void);
#endif
static void record_check(void);
//static void vDev_Go_Sleep(void);

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/

static void_int16_fptr record_event_handler[]=
{
    prvRecord_evt_nop,					// EVT_NOP
    prvRecord_evt_fix,                  // EVT_FIX
    prvRecord_evt_waketime,
};
/*********************************************************************/
/* User file include                                                 */
/*********************************************************************/


/*******************************************************************************
*    Function:  Record_Task
*
*  Parameters:  None
*     Returns:  None
* Description:  TASK to handle timing/distance records.
*******************************************************************************/
extern void Record_Task(void* pvParameters)
{
    Data_Message_T msg;
    uint32_t csq_time = OS_Time();

    #ifdef USE_DEBUG
    DEBUG_PRINT0( DEBUG_MEDIUM, "[Record]:Record TASK Started!\r\n");
    #endif

    while(PS_Running())
    {
        if(E_OK == OS_Wait_Message(OS_RECORD_TASK, &msg.all, MSec_To_Ticks(500)))
        {
            if(msg.parts.msg < RECORD_NUM_EVENTS)
            {
                if(NULL != record_event_handler[msg.parts.msg])
                {
                    (*record_event_handler[msg.parts.msg])(msg.parts.data);
                }
            }
        }
        if ((csq_time + MSec_To_Ticks(300)) < OS_Time())
        {
            csq_time = OS_Time();
            // Check messages in flash
            if ((Get_GPS_Data_Total_Number()>0) && (0!=GPRS_server_connected()))
            {
                OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BACKUP_GPS_UP, 1));
            }
        }
    }
    OS_Terminate_Task();
}

static void prvRecord_evt_nop(int16_t data)
{
}


static void record_save_current_pos(void)
{
    static gps_data_t gpsInfo;
    GPS_log_t gps_log;
    vGps_Get_Gps_Info(&gpsInfo);
    Switch_GPS_Sturcture_data_to_log(gpsInfo,&gps_log);
    ATProt_Get_Loc(gps_log.structData.lac,gps_log.structData.cell_id);
#if 0
    if (!gpsInfo.valid)
    {
        uint32_t timestamp=0;
        uint8_t clk_tmp[14];
        if (0!=ATProt_Get_Clock(clk_tmp))
        {
            timestamp=sys_get_sec_offset(clk_tmp);
        }
        gps_log.structData.postion_time[0]=(timestamp>>24) & 0xff;
        gps_log.structData.postion_time[1]=(timestamp>>16) & 0xff;
        gps_log.structData.postion_time[2]=(timestamp>>8) & 0xff;
        gps_log.structData.postion_time[3]=(timestamp) & 0xff;
    }
#endif
    Write_GPS_Data(&gps_log);
}

static void record_check_again(void)
{
    if (0==GPRS_server_connected())
    {
        // save data to flash
        record_save_current_pos();
    }
    else
    {
        if (0 < Get_GPS_Data_Total_Number())
        {
            record_save_current_pos();
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BACKUP_GPS_UP, 1));
        }
        else
        {
            // For now upload GPS data anyway
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_GPS_UPLOAD, 1));
        }
    }
    TMR_Start_Timer(GPS_UPLOAD_TIMER, gps_record_interval(), record_check_again);

//    }
}

static void prvRecord_evt_fix(int16_t data)
{
    if (rl_is_tracking())
    {
        return;
    }
    TMR_Stop_Timer(GPS_UPLOAD_TIMER);
    if (0==GPRS_server_connected())
    {
        // save data to flash
        record_save_current_pos();
    }
    else
    {
        if (0 < Get_GPS_Data_Total_Number())
        {
            record_save_current_pos();
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BACKUP_GPS_UP, 1));
        }
        else
        {
            // For now upload GPS data anyway
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_GPS_UPLOAD, 1));
        }
    }
}

static void prvRecord_evt_waketime(int16_t data)
{
    if (rl_is_tracking())
    {
        TMR_Start_Timer(GPS_UPLOAD_TIMER, GPS_UPLOAD_TIME_1, record_check);
    }
    else
    {
        // check once before sleep
        uint32_t wake_time=rl_get_wake_duration();
        if (wake_time < 60000)
        {
            wake_time=60000;
        }
        else
        {
            wake_time-=10000;
        }
        TMR_Start_Timer(GPS_UPLOAD_TIMER, wake_time, record_check);
    }
}

static void record_check(void)
{
    if (rl_is_tracking())
    {
        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_GPS_UPLOAD, 1));
        TMR_Start_Timer(GPS_UPLOAD_TIMER, gps_record_interval(), record_check_again);
        return;
    }
    if (0==GPRS_server_connected())
    {
        // Start new timer to send data
        record_save_current_pos();
    }
    else
    {
        if (0 < Get_GPS_Data_Total_Number())
        {
            record_save_current_pos();
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BACKUP_GPS_UP, 1));
        }
        else
        {
            OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_GPS_UPLOAD, 1));
        }
    }
}

static uint32_t gps_upload_interval(void)
{
    Config_t cfg;
    Get_config_data(&cfg);
    if (cfg.structData.GPS_upload_interval <= 2)
    {
        return 20000;
    }
    else
    {
        return (cfg.structData.GPS_upload_interval*1000);
    }
}

static uint32_t gps_record_interval(void)
{
    Config_t cfg;
    Get_config_data(&cfg);
    if (cfg.structData.GPS_record_interval <=1)
    {
        return 10000;
    }
    else
    {
        return (cfg.structData.GPS_record_interval*1000);
    }
//    return 10000;
}

/*====================================================================================*\
 * File Revision History
 *====================================================================================
 *
 * Date        userid  (Description on following lines:)
 * ----------- ------  ---------------------------------------------
 *
  ====================================================================================*/

