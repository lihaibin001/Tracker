/**
  ******************************************************************************
  * @file    stm32f10x_rtc.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file provides all the RTC firmware functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_rtc.h"
//#define USE_DEBUG
#include "Debug.h"

#include "standard.h"

/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup RTC 
  * @brief RTC driver modules
  * @{
  */

/** @defgroup RTC_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */

/** @defgroup RTC_Private_Defines
  * @{
  */
#define RTC_LSB_MASK     ((uint32_t)0x0000FFFF)  /*!< RTC LSB Mask */
#define PRLH_MSB_MASK    ((uint32_t)0x000F0000)  /*!< RTC Prescaler MSB Mask */
#define REF_RTC_BASE     ((uint32_t)10000000) /* divide base for RTC calibration */

//static uint8_t Is_Leap_Year(uint16_t year);

/**
  * @}
  */

/** @defgroup RTC_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup RTC_Private_Variables
  * @{
  */

//时间结构体
typedef struct
{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;                 
    uint16_t w_year;
    uint8_t  w_month;
    uint8_t  w_date;
    uint8_t  week;            
}tm;

tm rtc_timer;

static const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};

// RTC calibration reference
static uint32_t ref_start = 0;
static int32_t ref_cal = 0;

/**
  * @}
  */

/** @defgroup RTC_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup RTC_Private_Functions
  * @{
  */

/**
  * @brief  Enables or disables the specified RTC interrupts.
  * @param  RTC_IT: specifies the RTC interrupts sources to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_IT_OW: Overflow interrupt
  *     @arg RTC_IT_ALR: Alarm interrupt
  *     @arg RTC_IT_SEC: Second interrupt
  * @param  NewState: new state of the specified RTC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void RTC_ITConfig(uint16_t RTC_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RTC_IT(RTC_IT));  
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    RTC->CRH |= RTC_IT;
  }
  else
  {
    RTC->CRH &= (uint16_t)~RTC_IT;
  }
}

/**
  * @brief  Enters the RTC configuration mode.
  * @param  None
  * @retval None
  */
void RTC_EnterConfigMode(void)
{
  /* Set the CNF flag to enter in the Configuration Mode */
  RTC->CRL |= RTC_CRL_CNF;
}

/**
  * @brief  Exits from the RTC configuration mode.
  * @param  None
  * @retval None
  */
void RTC_ExitConfigMode(void)
{
  /* Reset the CNF flag to exit from the Configuration Mode */
  RTC->CRL &= (uint16_t)~((uint16_t)RTC_CRL_CNF); 
}

/**
  * @brief  Gets the RTC counter value.
  * @param  None
  * @retval RTC counter value.
  */
uint32_t RTC_GetCounter(void)
{
  uint16_t tmp = 0;
  uint32_t rt;
  uint32_t delta = 0;
  tmp = RTC->CNTL;
  rt = (((uint32_t)RTC->CNTH << 16 ) | tmp) ;
/*  if (ref_start < rt)
  {
      uint32_t high = ref_cal/1000;
      uint32_t low = ref_cal%1000;
      uint32_t delta_high = ((rt - ref_start)*(high)) / (REF_RTC_BASE/1000);
      uint32_t delta_low = ((rt - ref_start)*(low)) / (REF_RTC_BASE);
      delta = delta_high+delta_low;
      rt -= delta;
  }*/
  return rt;
}

/**
  * @brief  Sets the RTC counter value.
  * @param  CounterValue: RTC counter new value.
  * @retval None
  */
void RTC_SetCounter(uint32_t CounterValue)
{ 
  RTC_EnterConfigMode();
  /* Set RTC COUNTER MSB word */
  RTC->CNTH = CounterValue >> 16;
  /* Set RTC COUNTER LSB word */
  RTC->CNTL = (CounterValue & RTC_LSB_MASK);
  RTC_ExitConfigMode();
}

/**
  * @brief  Sets the RTC prescaler value.
  * @param  PrescalerValue: RTC prescaler new value.
  * @retval None
  */
void RTC_SetPrescaler(uint32_t PrescalerValue)
{
  /* Check the parameters */
  assert_param(IS_RTC_PRESCALER(PrescalerValue));
  
  RTC_EnterConfigMode();
  /* Set RTC PRESCALER MSB word */
  RTC->PRLH = (PrescalerValue & PRLH_MSB_MASK) >> 16;
  /* Set RTC PRESCALER LSB word */
  RTC->PRLL = (PrescalerValue & RTC_LSB_MASK);
  RTC_ExitConfigMode();
}

/**
  * @brief  Sets the RTC alarm value.
  * @param  AlarmValue: RTC alarm new value.
  * @retval None
  */
void RTC_SetAlarm(uint32_t AlarmValue)
{  
  RTC_EnterConfigMode();
  /* Set the ALARM MSB word */
  RTC->ALRH = AlarmValue >> 16;
  /* Set the ALARM LSB word */
  RTC->ALRL = (AlarmValue & RTC_LSB_MASK);
  RTC_ExitConfigMode();
}

/**
  * @brief  Gets the RTC divider value.
  * @param  None
  * @retval RTC Divider value.
  */
uint32_t RTC_GetDivider(void)
{
  uint32_t tmp = 0x00;
  tmp = ((uint32_t)RTC->DIVH & (uint32_t)0x000F) << 16;
  tmp |= RTC->DIVL;
  return tmp;
}

/**
  * @brief  Waits until last write operation on RTC registers has finished.
  * @note   This function must be called before any write to RTC registers.
  * @param  None
  * @retval None
  */
void RTC_WaitForLastTask(void)
{
  /* Loop until RTOFF flag is set */
  while ((RTC->CRL & RTC_FLAG_RTOFF) == (uint16_t)RESET)
  {
  }
}

/**
  * @brief  Waits until the RTC registers (RTC_CNT, RTC_ALR and RTC_PRL)
  *   are synchronized with RTC APB clock.
  * @note   This function must be called before any read operation after an APB reset
  *   or an APB clock stop.
  * @param  None
  * @retval None
  */
void RTC_WaitForSynchro(void)
{
  /* Clear RSF flag */
  RTC->CRL &= (uint16_t)~RTC_FLAG_RSF;
  /* Loop until RSF flag is set */
  while ((RTC->CRL & RTC_FLAG_RSF) == (uint16_t)RESET)
  {
  }
}

/**
  * @brief  Checks whether the specified RTC flag is set or not.
  * @param  RTC_FLAG: specifies the flag to check.
  *   This parameter can be one the following values:
  *     @arg RTC_FLAG_RTOFF: RTC Operation OFF flag
  *     @arg RTC_FLAG_RSF: Registers Synchronized flag
  *     @arg RTC_FLAG_OW: Overflow flag
  *     @arg RTC_FLAG_ALR: Alarm flag
  *     @arg RTC_FLAG_SEC: Second flag
  * @retval The new state of RTC_FLAG (SET or RESET).
  */
FlagStatus RTC_GetFlagStatus(uint16_t RTC_FLAG)
{
  FlagStatus bitstatus = RESET;
  
  /* Check the parameters */
  assert_param(IS_RTC_GET_FLAG(RTC_FLAG)); 
  
  if ((RTC->CRL & RTC_FLAG) != (uint16_t)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the RTC's pending flags.
  * @param  RTC_FLAG: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_FLAG_RSF: Registers Synchronized flag. This flag is cleared only after
  *                        an APB reset or an APB Clock stop.
  *     @arg RTC_FLAG_OW: Overflow flag
  *     @arg RTC_FLAG_ALR: Alarm flag
  *     @arg RTC_FLAG_SEC: Second flag
  * @retval None
  */
void RTC_ClearFlag(uint16_t RTC_FLAG)
{
  /* Check the parameters */
  assert_param(IS_RTC_CLEAR_FLAG(RTC_FLAG)); 
    
  /* Clear the corresponding RTC flag */
  RTC->CRL &= (uint16_t)~RTC_FLAG;
}

/**
  * @brief  Checks whether the specified RTC interrupt has occurred or not.
  * @param  RTC_IT: specifies the RTC interrupts sources to check.
  *   This parameter can be one of the following values:
  *     @arg RTC_IT_OW: Overflow interrupt
  *     @arg RTC_IT_ALR: Alarm interrupt
  *     @arg RTC_IT_SEC: Second interrupt
  * @retval The new state of the RTC_IT (SET or RESET).
  */
ITStatus RTC_GetITStatus(uint16_t RTC_IT)
{
  ITStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_RTC_GET_IT(RTC_IT)); 
  
  bitstatus = (ITStatus)(RTC->CRL & RTC_IT);
  if (((RTC->CRH & RTC_IT) != (uint16_t)RESET) && (bitstatus != (uint16_t)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/**
  * @brief  Clears the RTC's interrupt pending bits.
  * @param  RTC_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_IT_OW: Overflow interrupt
  *     @arg RTC_IT_ALR: Alarm interrupt
  *     @arg RTC_IT_SEC: Second interrupt
  * @retval None
  */
void RTC_ClearITPendingBit(uint16_t RTC_IT)
{
  /* Check the parameters */
  assert_param(IS_RTC_IT(RTC_IT));  
  
  /* Clear the corresponding RTC pending bit */
  RTC->CRL &= (uint16_t)~RTC_IT;
}

/**
  * @brief  RTC configuration.
  */
void RTC_Config(void)
{
    uint16_t u16_WaitForOscSource;
    if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
    {
        /* Clear Wake Up flag */
        PWR_ClearFlag(PWR_FLAG_WU);
    }
    RTC_ClearITPendingBit(RTC_IT_ALR);
    RTC_ClearITPendingBit(RTC_IT_SEC);

    if (BKP_ReadBackupRegister(BKP_DR1) != 0x5A5A)
    {

        /* Enable PWR and BKP clocks */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

        /* Allow access to BKP Domain */
        PWR_BackupAccessCmd(ENABLE);

        /* Reset Backup Domain */
        BKP_DeInit();

        /* Enable the LSI OSC */
//        RCC_LSICmd(ENABLE);
        RCC_LSEConfig(RCC_LSE_ON);
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
        {}
        /* Select the RTC Clock Source */
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
        /* Select LSE as RTC Clock Source */
        /* Enable RTC Clock */
        RCC_RTCCLKCmd(ENABLE);

        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();

        /* Enable the RTC Second */
        RTC_ITConfig(RTC_IT_SEC, ENABLE);

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();

        /* Set RTC prescaler: set RTC period to 1sec */
        RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();

        BKP_WriteBackupRegister(BKP_DR1, 0x5A5A);
        RTC_Set(1970,01,01,0,0,0);//

        RCC_ClearFlag();
//        batt_volt_nv_erase();
    }
    else
    {

        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
        for(u16_WaitForOscSource=0;u16_WaitForOscSource<5000;u16_WaitForOscSource++);
//        RCC_LSICmd(ENABLE);
        RCC_LSEConfig(RCC_LSE_ON);
        /* Wait till LSI is ready */
        while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
//        while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
        {}
        /* Select the RTC Clock Source */
        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
//        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
        /* Select LSE as RTC Clock Source */
        /* Enable RTC Clock */
        RCC_RTCCLKCmd(ENABLE);
        /* Wait for RTC registers synchronization */
        RTC_WaitForSynchro();
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
//        RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
        if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
        {
            DEBUG_PRINT0(DEBUG_MEDIUM,"[RTC]:power on!\r\n");
            RTC_Set(1970,01,01,0,0,0);//
        }
        else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
        {
            DEBUG_PRINT0(DEBUG_MEDIUM,"[RTC]:external reset!\r\n");
        }

        RTC_ITConfig(RTC_IT_SEC, ENABLE);

        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
        RCC_ClearFlag();
    }

    return;
}

uint8_t RTC_Set(uint16_t syear, uint8_t smon, uint8_t sday, uint8_t hour, uint8_t min, uint8_t sec)
{
    uint16_t t;
    uint32_t seccount=0;
    if(syear<2000||syear>2099)
        return 1;     
    for(t=1970;t<syear;t++)
    {
        if(Is_Leap_Year(t))
            seccount+=31622400;
        else
            seccount+=31536000;
    }
    smon-=1;
    for(t=0;t<smon;t++)
    {
        seccount+=(u32)mon_table[t]*86400;
        if(Is_Leap_Year(syear)&&t==1)seccount+=86400;        
    }
    seccount+=(u32)(sday-1)*86400;
    seccount+=(u32)hour*3600;
    seccount+=(u32)min*60;
    seccount+=sec;                                                                                    

//    RTC_Set_Ref(seccount,224358);
    if ((seccount > 1424000000) && (ref_start > 1424000000))
    {
        uint32_t rtc_now = RTC_GetCounter();
        if (rtc_now > rtc_now)
        {
            if (ref_cal != 0)
            {
                ref_cal += ((rtc_now - seccount)*5000000) / (rtc_now - ref_start);
            }
            else
            {
                ref_cal = ((rtc_now - seccount)*10000000) / (rtc_now - ref_start);
            }
        }
    }

    PWR_BackupAccessCmd(ENABLE);
    RTC_WaitForLastTask();
    RTC_SetCounter(seccount);
    RTC_WaitForLastTask();
    return 0;     
}


uint8_t RTC_Get(uint8_t *data)
{
    static u16 daycnt=0;
    u32 timecount=0;
    u32 temp=0;
    u16 temp1=0;

    timecount=RTC_GetCounter();

    temp=timecount/86400;
    if(daycnt!=temp)
    {
        daycnt=temp;
        temp1=1970;
        while(temp>=365)
        {
            if(Is_Leap_Year(temp1))
            {
                if(temp>=366) {
                    temp-=366;
                } else {
                    temp1++;
                    break;
                }
            }
            else temp-=365;
            temp1++;
        }
        rtc_timer.w_year=temp1;
        temp1=0;
        while(temp>=28)
        {
            if(Is_Leap_Year(rtc_timer.w_year)&&temp1==1)
            {
                if(temp>=29)temp-=29;
                else break;
            }
            else
            {
                if(temp>=mon_table[temp1])temp-=mon_table[temp1];
                else break;
            }
            temp1++; 
        }
        rtc_timer.w_month=temp1+1;
        rtc_timer.w_date=temp+1;
    }
    else if (temp==0)
    {
        rtc_timer.w_year=1970;
        rtc_timer.w_month=1;
        rtc_timer.w_date=1;
    }
    temp=timecount%86400;    
    rtc_timer.hour=temp/3600;
    rtc_timer.min=(temp%3600)/60;     
    rtc_timer.sec=(temp%3600)%60;
    if (data != NULL)
    {
	    sprintf((char *)data,"%d",rtc_timer.w_year);
	    *(data+4) = (rtc_timer.w_month/10) + '0';
	    *(data+5) = (rtc_timer.w_month%10) + '0';
	    *(data+6) = (rtc_timer.w_date/10) + '0';
	    *(data+7) = (rtc_timer.w_date%10) + '0';
	    *(data+8) = (rtc_timer.hour/10) + '0';
	    *(data+9) = (rtc_timer.hour%10) + '0';
	    *(data+10) = (rtc_timer.min/10) + '0';
	    *(data+11) = (rtc_timer.min%10) + '0';
	    *(data+12) = (rtc_timer.sec/10) + '0';
	    *(data+13) = (rtc_timer.sec%10) + '0';
	    //DEBUG_PRINT1(DEBUG_MEDIUM,"[RTC]%s\r\n",data);
    }
    return 0;
}

void RTC_Set_Ref(u32 ref_rtc, int32_t cal)
{
    if ((ref_rtc > 0) && (ref_rtc != 0xffffffff))
        ref_start = ref_rtc;
    if ((cal > -100000000) && (cal < 100000000))
        ref_cal = cal;
}

uint32_t RTC_Get_Cal()
{
    return ref_cal;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
