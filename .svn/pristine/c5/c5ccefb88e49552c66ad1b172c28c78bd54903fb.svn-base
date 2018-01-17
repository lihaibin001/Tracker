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
   Title                      : Diag_Task.C

   Module Description         : 

   Author                     : 

   Created                    : 2016-08-29

 **********************************************************************/

/*********************************************************************/
/* Include header files                                              */
/*********************************************************************/
/*********************************************************************/
#include "standard.h"
#include "gps.h"
#include "ATProtocol.h"

#define USE_DEBUG
#include "Debug.h"

/*********************************************************************/
/* File level pragmas                                                */
/*********************************************************************/

/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
/*USER DEFINITION*/

#define  diag_current_state()       (diag_current_state)
//#define AD_TEMP (1)
#define AD_TEMP (0)
#define TEMP_AD_REF_VOL (330)

#define LOW_VOLTAGE_ALARM_THRESHOLD (260)

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/


/*********************************************************************/
/* Function Prototypes for Private Functions with File Level Scope   */
/*********************************************************************/

/*event handler*/
static void diag_evt_nop(int16_t data);
static void diag_evt_connected(int16_t data);

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/

// Definition of the event handler function pointer array.
static const void_int16_fptr event_handler[DIAG_NUM_EVENTS] = 
{
	diag_evt_nop,			   		   
    diag_evt_connected,
};

/*********************************************************************/
/* Global and Const Variable Defining Definitions / Initializations  */
/*********************************************************************/
static const int32_t themal_res[25][2]={
    {31913,-40},
    {31663,-35},
    {31352,-30},
    {30971,-25},
    {30510,-20},
    {29960,-15},
    {29308,-10},
    {28552,-5},
    {27686,0},
    {26709,5},
    {25626,10},
    {24445,15},
    {23609,20},
    {21840,25},
    {20453,30},
    {19041,35},
    {17635,40},
    {16231,45},
    {14878,50},
    {13580,55},
    {12336,60},
    {11179,65},
    {10104,70},
    {9089,75},
    {8202,80}
};

static Self_Diag_T diag_result;

/*********************************************************************/
/* ROM Const Variables With File Level Scope                         */
/*********************************************************************/

/*********************************************************************/
/* Add User defined functions                                        */
/*********************************************************************/
// Test repeatly
static void diag_self_test(void);
static void diag_test_voltage(void);
static void diag_test_temp(void);
static void diag_test_light_sensor(void);
static void diag_test_gps(void);
static void diag_test_gprs(void);
static void diag_test_csq(void);
static void diag_data_init(void);

/**********************************************************************
 *    Function: Diag_Task
 *  Parameters: None
 *     Returns: None
 * Description: Main routine called by the operating system
 *********************************************************************/
void Diag_Task(void *pvParameters)
{
    Data_Message_T msg;          // Holds message received from the system
    uint32_t csq_time = 0;
    csq_time = OS_Time();

    while(PS_Running())
    {
        if(E_OK == OS_Wait_Message(OS_DIAG_TASK,&msg.all,100))
        {
            if((msg.parts.msg > 0) && ((msg.parts.msg) < DIAG_NUM_EVENTS))
            {
                (*event_handler[msg.parts.msg])(msg.parts.data);       // Run event handler routine
            }
        }
        if ((csq_time + MSec_To_Ticks(3000)) < OS_Time())
        {
            diag_self_test();
            csq_time = OS_Time();
        }
    }

    OS_Terminate_Task();
}


/**********************************************************************
 * Description: Do nothing event handler
 * Parameters: message data
 *     Returns: None
 *********************************************************************/
static void diag_evt_nop(int16_t data)
{
	
}

static void diag_evt_connected(int16_t data)
{
    diag_result.network_connected=1;
    DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Server connected\n\r");
}

static void diag_test_voltage(void)
{
    uint16_t int_vol=Pwr_Fail_Get_Voltage();
    diag_result.voltage=int_vol;
    DEBUG_PRINT1(DEBUG_MEDIUM,"[DIAG]Battery=%d\n\r",int_vol);
    if ((int_vol < LOW_VOLTAGE_ALARM_THRESHOLD) && (0 == Get_Low_Voltage_Uploaded()))
    {
        OS_Send_Message(OS_IOT_TASK,Build_Message(TM_EVT_BATTERY_FAULT, 1));
    }
}

static void diag_test_temp(void)
{
    int32_t temp=AD_Read(AD_TEMP);
    uint8_t i=0;
    if (diag_result.voltage>0)
    {
        if (temp>=((themal_res[0][0]*TEMP_AD_REF_VOL)/diag_result.voltage))
        {
            DEBUG_PRINT1(DEBUG_MEDIUM,"[DIAG]temperature =%d\n\r",themal_res[0][1]);
            diag_result.temp=themal_res[0][1];
            return;
        }
        for (i=1;i<24;i++)
        {
            if(temp>=((themal_res[i][0]*TEMP_AD_REF_VOL)/diag_result.voltage))
            {
                break;
            }
        }
        DEBUG_PRINT1(DEBUG_MEDIUM,"[DIAG]temperature=%d\n\r",themal_res[i-1][1]);
    }
    diag_result.temp=themal_res[i-1][1];
}

static void diag_test_light_sensor(void)
{
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
    {
        diag_result.light_sensor=1;
        Set_Wkup_Enable(0);
        DEBUG_PRINT0(DEBUG_HIGH,"[DIAG]Light Sensor 1\n\r");
    }
    else
    {
        diag_result.light_sensor=0;
        Set_Wkup_Enable(1);
        DEBUG_PRINT0(DEBUG_HIGH,"[DIAG]Light Sensor 0\n\r");
    }
}

static void diag_test_gps(void)
{
    if (vGps_Get_Gps_Status())
    {
        diag_result.gps_fixed=1;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]GPS fixed\n\r");
    }
    else
    {
        diag_result.gps_fixed=0;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]GPS NOT fixed\n\r");
    }
}

static void diag_test_gprs(void)
{
#if 0
    if (0!=GPRS_server_connected())
    {
        diag_result.network_connected=1;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Server connected\n\r");
    }
    else
    {
        diag_result.network_connected=0;
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Server NOT connected\n\r");
    }
#endif
}

static void diag_test_csq(void)
{
    uint8_t csq_data=ATProt_getRSSI();
    if (0!=GPRS_server_connected())
    {
        DEBUG_PRINT1(DEBUG_MEDIUM,"[DIAG]CSQ data %d\n\r",csq_data);
        if (diag_result.csq_max<csq_data)
        {
            diag_result.csq_max=csq_data;
        }
        if (diag_result.csq_min>csq_data)
        {
            diag_result.csq_min=csq_data;
        }
        DEBUG_PRINT1(DEBUG_MEDIUM,"[DIAG]CSQ MAX %d\n\r",diag_result.csq_max);
        DEBUG_PRINT1(DEBUG_MEDIUM,"[DIAG]CSQ MIN %d\n\r",diag_result.csq_min);
    }
}

static void diag_self_test(void)
{
    // Check GPS
    // Check GPRS
    diag_test_voltage();
    diag_test_temp();
    diag_test_light_sensor();
    diag_test_gps();
    diag_test_gprs();
}

void diag_set_flash_result(uint8_t result)
{
    diag_result.flash_ok = result;
    if (result == 0)
    {
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Flash test fail\n\r");
    }
    else
    {
        DEBUG_PRINT0(DEBUG_MEDIUM,"[DIAG]Flash test OK\n\r");
    }
}

void diag_get_result(Self_Diag_T *result)
{
    memcpy(result, &diag_result, sizeof(Self_Diag_T));
}

static void diag_data_init(void)
{
    diag_result.csq_min=31;
    diag_result.csq_max=0;
}

/********************************************************************** 
 *                                                             
 * REVISION RECORDS                                            
 *                                                             
*********************************************************************/
/* $HISTROY$

 *********************************************************************/
