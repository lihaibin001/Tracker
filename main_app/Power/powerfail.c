/*===========================================================================*/
/**
 * @file powerfail.c
 *
 *
 */
/*==========================================================================*/

/*===========================================================================*
 * Header Files
 *===========================================================================*/ 
#include    "standard.h"
#define USE_DEBUG
#include "Debug.h"
#include "TelmProtocol.h"

/*===========================================================================*
 * Local Preprocessor #define MACROS
 *===========================================================================*/
#define write_timer(x, y)           (timer[x] = y)
#define read_timer(x)               (timer[x])
#define timer_running(x)            (timer[x] > OS_Time())

#define AD_LVW_THRESHOLD            voltage_limits[V_LOW].lower_voltage
#define AD_RECOVERY_THRESHOLD            voltage_limits[V_LOW].upper_voltage

#define AD_BATTERY 9 /* battery voltage channel */
//#define AD_GAIN() 2156 //for HW 134.78 =(65520*(40.2/(330+40.2)))/(3.3) ,theoretical value
#define AD_GAIN() 65016 //for HW 134.78 =(65520*(43/(100+43)))*(3.3) ,theoretical value


#define MAX_DEBOUNCE_WARNING_COUNT          12  

/*===========================================================================*
 * Local Type Declarations
 *===========================================================================*/
enum
{
    MUTE_DELAY_TIMER,
    NUM_PF_TIMERS
};

enum
{
    V_LOW,
    MUTE_LOW,
    MUTE_HIGH,
    V_HIGH,
    V_IGN_PULSE,  
    NUM_MAX_VOLTAGE_TRESHOLD
};

typedef struct PF_Voltage_Treshold_Limits_Tag
{
    uint16_t  lower_voltage;
    uint16_t  upper_voltage;
} PF_Voltage_Treshold_Limits_Type;

typedef struct PF_Voltage_Treshold_Data_Tag
{
    uint8_t   count;
    bool      lower_voltage_is;
} PF_Voltage_Treshold_Data_Type;

/*===========================================================================*
 * Local Object Definitions
 *===========================================================================*/
static const PF_Voltage_Treshold_Limits_Type voltage_limits[NUM_MAX_VOLTAGE_TRESHOLD] =
{
//    { 600,  650 }, /* V_LOW          RESET threshold    */
//    { 1150,  1180 }, /* MUTE_LOW    WARNING  threshold  */
//    { 1600, 1650}, /* MUTE_HIGH  WARNING threshold */
//    { 2000, 2050}  /* V_HIGH       RESET threshold */
    { 200,  250 }, /* V_LOW          RESET threshold    */
    { 250,  260 }, /* MUTE_LOW    WARNING  threshold  */
    { 500, 650}, /* MUTE_HIGH  WARNING threshold */
    { 2000, 2050}  /* V_HIGH       RESET threshold */
};

static PF_Voltage_Treshold_Data_Type voltage_data[NUM_MAX_VOLTAGE_TRESHOLD] =
{
    {MAX_DEBOUNCE_COUNT, false},
    {12, false},
    {MAX_DEBOUNCE_COUNT, true },
    {MAX_DEBOUNCE_COUNT, true }
};

static bool pwr_fail_mute_is;
static bool pwr_fail_reset_is;
static Tick_Type timer[NUM_PF_TIMERS];
//static uint8_t pwr_warning_debounce = 30;

static uint32_t   batt_volt_avg;
static uint16_t   batt_volt_avg_last;
/*===========================================================================*
 * Local Function Prototypes
 *===========================================================================*/
static void Pwr_Fail_Voltage_Hysteresis(uint16_t voltage, uint8_t index);
bool Pwr_Fail_Is_Mute_Condition(void);
bool Pwr_Fail_Is_Reset_Condition (void);

/*===========================================================================*
 * Function Definitions
 *===========================================================================*/

/**********************************************************************
*
*    Function: Pwr_Fail_Initialize
*
*  Parameters: none
*
*     Returns: none
*
* Description: initializes powerfail monitor module
*
**********************************************************************/
void Pwr_Fail_Initialize(void)
{
    /* configure AD-LVW interrupt */
    //AD_Interrupt_Configure();
    //AD_Interrupt_Enable();
}

/**********************************************************************
*
*    Function: Pwr_Fail_Shutdown
*
*  Parameters: none
*
*     Returns: none
*
* Description: This function configures powerfail module for idle mode.
*
*********************************************************************/
void Pwr_Fail_Shutdown(void)
{
    /* AD conversion interrupt does not work in STOP! */
    //AD_Interrupt_Disable();
}

/**********************************************************************
*
*    Function: Pwr_Fail_Monitor
*
*  Parameters: none
*
*     Returns: none
*
* Description: This function handles all powerfail cases.
*
*********************************************************************/
void Pwr_Fail_Monitor(void)
{
    Pwr_Fail_Check_Voltage();
    // Check RTC counter
    // Read flash to see if full
    // If not full, save current RTC count and voltage into flash
    // If RTC count greater than 1387584000, save count
    // Save voltage with 4 interval(in seconds): 300, 3600, 7200, 14400
    // Only save data when acc off
    /* check powerfail-mute: voltage range that allows mute and stops devices */
    if (pwr_fail_mute_is)
    {
        /* powerfail mute still active? */
        if (Pwr_Fail_Is_Mute_Condition()== false)
        {
            /* reenable powerfail mute monitoring */
            pwr_fail_mute_is = false;

            /* inform RELAYS */
            RL_Set_Pwr_Fail_Detected(pwr_fail_mute_is);
        }
        else
        {
            /* retrigger MUTE_DELAY_TIMER */
            write_timer(MUTE_DELAY_TIMER, OS_Time() + MUTE_OFF_WAIT_TICKS);
        }
    }
    else
    {
        if (Pwr_Fail_Is_Mute_Condition())
        {
            if((!PS_Full_System())
		      &&(Sys_Is_RTC_Deep_Wakeup()))
            {
                Sys_Clear_Wakeup_Src_Flags();
                Sys_Set_Low_Batt_Wakeup_Flag();
                SY_Warm_Start();
                pwr_fail_mute_is = true; 
            }
            /* trigger MUTE_DELAY_TIMER , delay 500ms unmute operation after recover from power fail*/
            write_timer(MUTE_DELAY_TIMER, OS_Time() + MUTE_OFF_WAIT_TICKS);

            /* inform RELAYS */
            RL_Set_Pwr_Fail_Detected(pwr_fail_mute_is);
        }
        else
        {
            if (TMR_Is_Timer_Active(TELM_BATT_CHECK_TIMER))
            {
                TMR_Stop_Timer(TELM_BATT_CHECK_TIMER);
            }
        }
    }

    /* check powerfail-reset: voltage range that allows reset (warm start) */
    if (pwr_fail_reset_is)
    {

    }
    else
    {
        if (Pwr_Fail_Is_Reset_Condition())
        {
            pwr_fail_reset_is = true;
            /* inform RELAYS */
            RL_Set_Pwr_Fail_Detected(pwr_fail_reset_is);
        }
    }
}

/**********************************************************************
*
*    Function: Pwr_Fail_AD_get_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: get  the AD convertion result and convert to the value fo voltage.
*              example:1200 means 12.00v
**********************************************************************/
uint16_t Pwr_Fail_AD_get_Voltage(void)
{
    uint16_t batt_voltage;

    batt_voltage = AD_Read(AD_BATTERY);
//    DEBUG_PRINT1 (DEBUG_MEDIUM,"\r\n[SYSTEM]:AD=%d \r\n", batt_voltage);
    if (batt_voltage==0)
    {
        return 0;
    }
    else
    {
//        batt_voltage = (((uint32_t)(batt_voltage) * 100)/AD_GAIN())+VOLTAGE_VAL_COPENSTATION;
        batt_voltage = (AD_GAIN() * 100)/((uint32_t)(batt_voltage));
        return batt_voltage;
    }
}
/**********************************************************************
*
*    Function: Pwr_Fail_Get_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: It is the average value of battery voltage in 20ms
**********************************************************************/
uint16_t Pwr_Fail_Get_Voltage(void)
{
    return batt_volt_avg_last;
}

/**********************************************************************
*
*    Function: Pwr_Fail_Check_Voltage
*
*  Parameters: none
*
*     Returns: void
*
* Description: Determine's if the voltage is above/below a certain votage.
*
**********************************************************************/
void Pwr_Fail_Check_Voltage(void)
{
    static uint32_t period_slice = 0;
    uint16_t batt_voltage;

    batt_voltage = Pwr_Fail_AD_get_Voltage();
    batt_volt_avg += batt_voltage;
    period_slice ++;
    if(period_slice%4 == 0)//20ms
    {
        batt_volt_avg_last = (batt_volt_avg >> 2);
        batt_volt_avg = 0;
        if((!PS_Full_System())
          &&(Sys_Is_RTC_Deep_Wakeup()))
        {
//            batt_volt_nv_check_write_timer();
        }
    }

    /* first check all LOW thresholds on AN_VLOW line */
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, V_LOW   );
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, MUTE_LOW);

    /* then check all HIGH thresholds on AN_BATTERY line */
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, MUTE_HIGH);
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, V_HIGH   );

    /* then check ign thresholds on AN_BATTERY line */
    Pwr_Fail_Voltage_Hysteresis(batt_voltage, V_IGN_PULSE   );
}

/**********************************************************************
*
*    Function: Pwr_Fail_Voltage_Hysteresis
*
*  Parameters: uint16_t voltage - voltage to process
*              uint8_t index    - index to voltage threshold table
*
*     Returns: none
*
* Description: runs the voltage hysteresis
*
**********************************************************************/
static void Pwr_Fail_Voltage_Hysteresis (uint16_t voltage, uint8_t index)
{
    uint8 debounce = 0;

    if((index == (uint8_t)MUTE_LOW)||(index == (uint8_t)V_IGN_PULSE))
        debounce = MAX_DEBOUNCE_WARNING_COUNT;
    else
        debounce = MAX_DEBOUNCE_COUNT;

    if (voltage_data[index].lower_voltage_is == false)
    {
        if (voltage < voltage_limits[index].lower_voltage)
        {/*count down if voltage below */
            if (!voltage_data[index].count--)
            {
                voltage_data[index].count = debounce;
                voltage_data[index].lower_voltage_is = true;
            }
        }
        else if (voltage_data[index].count < debounce)
        {/* count up if voltage above */
            voltage_data[index].count++;
        }
    }
    else
    {
        if (voltage > voltage_limits[index].upper_voltage)
        {/*count down if voltage above */
            if (!voltage_data[index].count--)
            {
                voltage_data[index].count = debounce;
                voltage_data[index].lower_voltage_is = false;
            }
        }
        else if (voltage_data[index].count < debounce)
        {/*count up if voltage below */
            voltage_data[index].count++;
        }
    }
}

/**********************************************************************
*
*    Function: Pwr_Fail_Is_Mute_Condition
*
*  Parameters: none
*
*     Returns: true/false
*
* Description: returns powerfail mute condition
*
**********************************************************************/
bool Pwr_Fail_Is_Mute_Condition(void)
{
    //return((voltage_data[MUTE_LOW].lower_voltage_is  == true) ||
    //       (voltage_data[MUTE_HIGH].lower_voltage_is == false));
    return(voltage_data[MUTE_LOW].lower_voltage_is  == true) ;
}

/**********************************************************************
*
*    Function: Pwr_Fail_Is_Reset_Condition
*
*  Parameters: none
*
*     Returns: true/false
*
* Description: returns powerfail reset condition
*
**********************************************************************/
bool Pwr_Fail_Is_Reset_Condition (void)
{
    return((voltage_data[V_LOW].lower_voltage_is  == true) ||
          (voltage_data[V_HIGH].lower_voltage_is == false));
}
/**********************************************************************
*
*    Function: Pwr_Fail_Is_Voltage_Good
*
*  Parameters: None
*
*     Returns: true/false
*
* Description: returns if the voltage is over a working threshold
*
*********************************************************************/
bool Pwr_Fail_Is_Voltage_Good (void)
{
    return(!Pwr_Fail_Is_Mute_Condition() && !timer_running(MUTE_DELAY_TIMER));
}

/**********************************************************************
*
*    Function: Pwr_Fail_AD_ISR
*
*  Parameters: -
*
*     Returns: -
*
* Description: Interrupt Service Routine for Low Voltage Warning.
*              This interrupt is activated when the battery sense line
*              goes low.
*              A flag is set and the relay resoure is released.
*              The A/D interrupt be disabled to reduce the system load.
*              It will be reenabled if the voltage is in an allowed range.
*
**********************************************************************/
void Pwr_Fail_AD_ISR(void)
{
    AD_Interrupt_Disable();

    /* low voltage detected */
    pwr_fail_reset_is = true;
    pwr_fail_mute_is  = true;

    /* set hysteresis state machine to start checking if U > 6.5V ! */
    voltage_data[V_LOW].count = MAX_DEBOUNCE_COUNT;
    voltage_data[V_LOW].lower_voltage_is = true;

    voltage_data[MUTE_LOW].count = MAX_DEBOUNCE_COUNT;
    voltage_data[MUTE_LOW].lower_voltage_is = true;

    /* inform RELAYS */
    /* trigger RELAYS task when powerfail status is updated */
    OS_Release_Resource(RES_RELAYS);
}


/**********************************************************************
 *
 * Revision History
 *
 *********************************************************************
 *
 *********************************************************************/
