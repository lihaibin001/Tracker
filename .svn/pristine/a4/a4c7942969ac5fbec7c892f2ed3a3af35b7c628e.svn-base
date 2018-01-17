/* $Header:   relays.h    $*/
/*************************************************************************
   Title                    : relays.h

   Module Description       : This file has all of the standard global defines
                              for module relays

   Author                   : 

   Created                  : 

   Configuration ID         : 

*************************************************************************/

/*----------------------------------------------------------------------
*   Instructions for using this module if any:
*
*---------------------------------------------------------------------*/
#ifndef  RELAYS_H
#define  RELAYS_H

typedef enum rl_mode_tag
{
   RL_IDLE_MODE,
   RL_AWAKE_MODE,
   RL_IGNITION_MODE,
   RL_PRE_PLAY_MODE,
   RL_PLAY_MODE,
   NO_RL_MODE
} rl_mode_type;


/*********************************************************************/
/* Include files                                                     */
/*********************************************************************/
#include "standard.h"        /* include standard includes */

/*********************************************************************/
/* Standard C Library                                                */
/*********************************************************************/

/*********************************************************************/
/* Application specific Header files                                 */
/*********************************************************************/
#include "psync.h"
#include "regulator.h"

/*********************************************************************/
/* relays header file                                              */
/*********************************************************************/

/*********************************************************************/
/* Constant and Macro Definitions using #define                      */
/*********************************************************************/
#define STBY_OFF_WAIT_TICKS               (100)

//#define AWAKE_TO_IDLE_INCREMENT_TICKS     (MSec_To_Ticks(100))
#define AWAKE_TO_IDLE_INCREMENT_TICKS     (MSec_To_Ticks(50))//fixme:scb
#define NUM_INCREMENTS_TO_RESEND_IDLE_MSG 5


#define NUM_INCREMENTS_TO_FORCE_IDLE      10  //fixme:scb

/* max requeue time for Relays_Task */
#define MAX_RL_WAIT_TIME                  (MSec_To_Ticks(100))

/* after enable IIC-controlled powe always wait min. 25ms to let voltage stabilize */
#define REG_ON_TIME                       (MSec_To_Ticks(25))

#define RL_GPRS_COLD_START_WAIT_TIME       (MSec_To_Ticks(30000))//30S

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/

/*********************************************************************/
/* Global Variable extern Declarations                               */
/*********************************************************************/

/*********************************************************************/
/* Function Prototypes                                               */
/*********************************************************************/
extern bool rl_full_awake_on(void);
extern bool RL_In_Idle(void);
extern bool RL_In_Play (void);
extern void RL_Begin_Awake_Sequence(void);
extern void RL_Finish_Awake_Sequence(void);
extern void RL_Enter_Powerfail(bool low_voltage);
extern rl_mode_type  RL_Mode(void);
extern bool RL_Voltage_Check_Abnormal(void);

extern void RL_Set_Pwr_Fail_Detected(bool status);//for pwr fail

void rl_delay_without_schedule(Tick_Type ms);
void RL_Start_GPRS(void);
void RL_Start_GPS(void);
void RL_Force_Sleep(void);

extern void rl_rtc_disable(void);
extern void rl_rtc_config(void);
extern void rl_exti_config(void);
extern uint32_t rl_get_rtc_timeout(void);
extern void rl_set_rtc_timeout(uint32_t value);
extern uint16_t rtc_get_tick(void);
extern bool rl_get_can_interrupt_state(void);
extern void rl_set_can_interrupt_state(bool value);
extern uint8_t rl_get_low_batt_sleep(void);
extern void rl_set_low_batt_sleep(uint8_t value);
extern void rl_reset(void);
extern void rl_exit_tracking(void);
extern bool rl_is_tracking(void);
extern uint32_t rl_get_wake_duration(void);
#endif
/**********************************************************************
*                                                                     *
* REVISION RECORDS                                                    *
*                                                                     *
**********************************************************************/
/*********************************************************************/
/* 
 *
 *********************************************************************/
