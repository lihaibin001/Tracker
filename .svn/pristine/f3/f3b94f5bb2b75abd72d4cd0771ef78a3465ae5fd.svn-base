/**********************************************************************
   Title                      : Sensors.c         
                                                                         
   Module Description         : Sensors Module.

   Author                     : 
   
 *********************************************************************/

/**********************************************************************
 * Include files                                                       
 *********************************************************************/
#include "standard.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#define USE_DEBUG
#include "Debug.h"
#include "Sensors.h"
//#include "gps.h"

/**********************************************************************
 * Constant and Macro Definitions using #define
 *********************************************************************/
#define SPI_REG_ADDR_BUFFER_LEN         (2)
#define SPI_WRITE_BIT_MASK              (0x80)

/* registor data define */
/* PM2-PM0 */
#define POWER_MODE_DOWN			(0x00)
#define POWER_MODE_NORMAL		(0x01)
#define POWER_MODE_LOW_HALF		(0x02)
#define POWER_MODE_LOW_1		(0x03)
#define POWER_MODE_LOW_2		(0x04)
#define POWER_MODE_LOW_5		(0x05)
#define POWER_MODE_LOW_10		(0x06)

/*Sensor scale*/
//#define FULL_SCALE_2G_MODE   (0x00)
#define FULL_SCALE_4G_MODE   (0x02)
//#define FULL_SCALE_8G_MODE   (0x03)

/*Interrupt threshhold*/
#define STATIC_THRESH 10    //mg

#define INT_THRESH_OVER_TIMES 3
#define INT_THRESH_CHECK_TIME 5 //5ms per times

#define INT_THRESH_MOVE_TIMES 30
#define INT_THRESH_STATIC_TIME 200 //5ms per times

/*Interrupt mode*/
#define INT_OR_COMB 0
#define INT_6D_MOVE 1
#define INT_AND_COMB 2
#define INT_6D_POST 3

#define G_CAL_COUNT_MAX (10)
#define G_CAL_SPEED_LIMIE (20)
#define G_TURN_LIMIT (200)

//#define GSENSOR_ALERT_TRIGGER (100000)
#define GSENSOR_ALERT_TRIGGER (97500)
//Filter, G shall not higher than 2.8g
//#define G_COUNT_START (640000)
#define G_COUNT_START (102500)
#define G_LIMIT (7840000)
//#define G_X_ACC_LIMIT (10000)
#define G_X_ACC_LIMIT (100)
#define G_TOTAL_COUNT_LIMIT (24)
#define G_HIGH_COUNT_LIMIT (16)
#define G_HIGH_COUNT_DATA_SIZE (48)//16*3

#define G_REF_DELTA_HIGH (60900)
#define G_REF_DELTA_LOW (59100)

/*VECM threshold config*/
/*  The resolution is equal to the selected accelerometer resolution set in XYZ_DATA_CFG[fs] */
//set  A_VECM_THS_MSB, LSB = 0x8032 => 204 *0.122mg = ~25mg
//set  A_VECM_THS_MSB, LSB = 0x80CC => 204 *0.488mg = ~100mg
//set  A_VECM_THS_MSB, LSB = 0x812C => 300 *0.488mg = ~146mg
#define G_SENSOR_LEVEL_1_VECM_THRESHOLD (0x8032)
#define G_SENSOR_LEVEL_2_VECM_THRESHOLD (0x80CC)

/*********************************************************************/
/* Enumerations and Structures and Typedefs                          */
/*********************************************************************/
typedef struct
{
	uint8_t	active		:1;
	uint8_t	f_read		:1;
	uint8_t	lnoise		:1;
	uint8_t	ODR			:3;	
	uint8_t	aslp_rate	:2;
}CtrlReg1;

typedef struct
{
	uint8_t	modes		:2;
	uint8_t	aslpe		:1;
	uint8_t	smods		:2;
	uint8_t   reserved		:1;
	uint8_t	rst_en		:1;	
	uint8_t	selftest		:1;
}CtrlReg2;

typedef struct
{
	uint8_t	src_drdy	:1;  //Data-ready interrupt status bit.
	uint8_t	src_a_vecm	:1;  //Accelerometer vector-magnitude interrupt status bit
	uint8_t	src_ffmt		:1;  //Freefall/motion interrupt status bit
	uint8_t	src_pulse	:1;  //Pulse interrupt status bit
	uint8_t	src_lndprt	:1;  //Landscape/Portrait orientation interrupt status bit
	uint8_t	src_trans	:1;  //Transient interrupt status bit
	uint8_t	src_fifo		:1;  //FIFO interrupt status bit
	uint8_t	src_aslp		:1;  //Auto-Sleep/Wake interrupt status bit:
}IntSrc;

/**********************************************************************
 * Function Prototypes for Private Functions with File Level Scope
 *********************************************************************/
static uint8_t prvSensors_readRegs(uint8_t regAddr, uint8_t* data, uint8_t len);
static void prvSensors_writeRegs(uint8_t regAddr, uint8_t data, uint8_t NumByteToWrite);

//static void FXLS1_FIFO_DataRead(void);
static void Sensors_LowLevel_Init(void);
static void Sensors_FIFO_Fuction_Disable(void);
static void Sensors_VECM_Theshold_Config(uint16_t threshold);

#if 0
static void Sensors_FFMT_fuction_init(void);
static void Sensors_TRANS_fuction_init(void);
static void Sensors_CLICK_fuction_init(void);
#endif
/**********************************************************************
 * Global and Const Variable Defining Definitions / Initializations
 *********************************************************************/
//#define GSENS_BUF_NUM 30
bool Gsensor_move_flag;
bool Gsensor_static_flag;
bool car_move_detected;
int16_t xAccel_mg;
int16_t yAccel_mg;
int16_t zAccel_mg;

int16_t pre_xAccel_mg;
int16_t pre_yAccel_mg;
int16_t pre_zAccel_mg;

int16_t xAccel;                      	
int16_t yAccel;                      	
int16_t zAccel; 

int16_t static_xAccel;               	
int16_t static_yAccel;      	
int16_t static_zAccel; 

#ifdef CONFIG_G_SENSOR_UPLOAD
uint8_t g_count = 0;
uint8_t g_high_count = 0;
uint8_t g_total_count = 0;
int32_t g_high_data[G_HIGH_COUNT_LIMIT][2];
int16_t g_acc_data[G_HIGH_COUNT_LIMIT][3];
int16_t g_acc_stored[G_HIGH_COUNT_DATA_SIZE];
uint8_t speed_stored = 0;
#endif
static uint16_t crank_count = 0; 
static uint16_t shake_count = 0;

static int16_t g_stop_acc[3];
static int16_t g_run_acc[3];

FXLS1_sensorData_t FIFO_sensorDate[32];

/*********************************************************************/
/* Static Variables and Const Variables With File Level Scope        */
/*********************************************************************/
#define AWAKE_MODE 2
#define SLEEP_MODE   1
#define STANDBY_MODE 0

/*********************************************************************/
/* User file include                                                 */
/*********************************************************************/

/**********************************************************************
 * Function Definitions
 *********************************************************************/
/*******************************************************************************
*    Function: Sensors_init
*
*  Parameters:  uint8_t regAddr
*     Returns:  None
* Description:  read register of Gsensor through IIC bus
*******************************************************************************/
void Sensors_init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    uint8_t deviceWhoAmI = 0;
    uint8_t sysmod = 0;
    uint8_t readdata;

    Sensors_LowLevel_Init();

    /*!< SPI configuration */

    GSENSOR_CS_HIGH();/*!< Deselect the GSENSOR: Chip Select high */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;

    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(GSENSOR_SPI, &SPI_InitStructure);
    /*!< Enable the GSENSOR_SPI  */
    SPI_Cmd(GSENSOR_SPI, ENABLE);

    if(Cold_Start())
    {
        /*Verify device is there*/
        prvSensors_readRegs(WHO_AM_I_REG,&deviceWhoAmI,1);
        if (deviceWhoAmI != WHO_AM_I_VAL)
        {
            DEBUG_PRINT0(DEBUG_HIGH,"[sensor]:Failed to get ID of Gsensor \r\n");
        }

        /*!!!Modification of control register can only occur in Standby mode */ 
        prvSensors_readRegs(CTRL_REG1,&readdata,1);
        prvSensors_writeRegs(CTRL_REG1, readdata & ~ACTIVE_MASK,1);

        uDelay(200);//delay 200us 
        /*Check if we are in STANDBY mode using SYSMOD*/
        prvSensors_readRegs(SYSMOD_REG,&sysmod,1);
        if ((sysmod & SYSMOD_MASK) != SYSMOD_STANDBY)
        {
             DEBUG_PRINT1 (DEBUG_HIGH,"[sensor]:Entering stby mode failed, current mode is %x\r\n",sysmod);
        }

        /*Enable low power in sleep state, high resolution in active state*/
        prvSensors_writeRegs(CTRL_REG2, SMOD_LOW_POWER|MOD_HIGH_RES,1);

        /*Set full-scale range to +- 4G, ¡À0.488 mg/LSB.*/
        prvSensors_writeRegs(XYZ_DATA_CFG_REG,FULL_SCALE_4G,1);

        /*VECM interrupt Threshold level 1,~25mg */
        Sensors_VECM_Theshold_Config(G_SENSOR_LEVEL_1_VECM_THRESHOLD);

        /*Enable A_VECM detection interrupts*/
        prvSensors_writeRegs(CTRL_REG4, INT_EN_A_VECM_MASK,1);

        /*Route A_VECM interrupts to INT1,INT2 is unused*/
        prvSensors_writeRegs(CTRL_REG5, INT_CFG_A_VECM_MASK,1);

        /*Enable A_VECM as a wake-up source*/
        //prvGSensor_writeRegs(CTRL_REG3, WAKE_A_VECM_MASK,1);

        /*Finally activate device with ODR=100Hz, FSR=4g*/
        prvSensors_writeRegs(CTRL_REG1, DATA_RATE_100HZ|ACTIVE_MASK,1);

        uDelay(200);//delay 200us 
        /* Check if we are in ACTIVE mode using SYSMOD*/
        prvSensors_readRegs(SYSMOD_REG,&sysmod,1);
        sysmod = sysmod & SYSMOD_MASK;
        if (sysmod != SYSMOD_WAKE)
        {
            DEBUG_PRINT1 (DEBUG_HIGH,"[sensor]:Entering active mode failed, current mode is %x\r\n",sysmod);
        }
    }
}

/*******************************************************************************
*    Function: Sensors_FIFO_Fuction_Disable
*
*  Parameters: None
*     Returns:  None
* Description:  disable FIFO function init
*******************************************************************************/
static void Sensors_FIFO_Fuction_Disable(void)
{
    uint8_t readdata;
    uint8_t sysmod = 0;
    /*!!!Modification of control register can only occur in Standby mode */ 
    prvSensors_readRegs(CTRL_REG1,&readdata,1);
    prvSensors_writeRegs(CTRL_REG1, readdata & ~ACTIVE_MASK,1);

    uDelay(200);//delay 200us 
    /*Check if we are in STANDBY mode using SYSMOD*/
    prvSensors_readRegs(SYSMOD_REG,&sysmod,1);
    if ((sysmod & SYSMOD_MASK) != SYSMOD_STANDBY)
    {
        DEBUG_PRINT1 (DEBUG_HIGH,"[sensor]:Entering stby mode failed, current mode is %x\r\n",sysmod);
    }

    /*Disable FIFO*/
    prvSensors_writeRegs(F_SETUP_REG,0x00,1); 

    /*VECM interrupt Threshold level 1,~25mg */
    Sensors_VECM_Theshold_Config(G_SENSOR_LEVEL_1_VECM_THRESHOLD);   
	
    /*Enable A_VECM detection interrupts*/
    prvSensors_writeRegs (CTRL_REG4, INT_EN_A_VECM_MASK,1);
    /*Route A_VECM interrupts to INT1,INT2 is unused.*/
    prvSensors_writeRegs (CTRL_REG5, INT_CFG_A_VECM_MASK,1);
    /*Enable  A_VECM as a wake-up source*/
    prvSensors_writeRegs (CTRL_REG3, WAKE_A_VECM_MASK,1);

    /*Finally activate device with ODR=100Hz, FSR=4g*/
    prvSensors_writeRegs (CTRL_REG1, DATA_RATE_100HZ|ACTIVE_MASK,1);

    uDelay(200);//delay 200us 

    /* Check if we are in ACTIVE mode using SYSMOD*/
    prvSensors_readRegs(SYSMOD_REG,&sysmod,1);
    sysmod = sysmod & SYSMOD_MASK;
    if (sysmod != SYSMOD_WAKE)
    {
        DEBUG_PRINT1 (DEBUG_HIGH,"[sensor]:Entering active mode failed, current mode is %x\r\n",sysmod);
    }
}
/*******************************************************************************
*    Function: Sensors_VECM_fuction_Config
*
*  Parameters: None
*     Returns:  None
* Description:  //this threshold is for vibrate detection to check if engine is running
*******************************************************************************/
static void Sensors_VECM_Theshold_Config(uint16_t threshold)
{
    #if (DEBUG_LVL == DEBUG_MEDIUM)
    uint8_t readdata;
    #endif
    prvSensors_writeRegs (A_VECM_THS_MSB, (uint8_t)(threshold>>8),1); 
    prvSensors_writeRegs (A_VECM_THS_LSB, (uint8_t)(threshold),1); 
    // set debounce to 2, A_VECM_CNT = 2 /400hz = 5ms
    prvSensors_writeRegs (A_VECM_CNT, 0x02,1);
    // A_VECM_CFG 
    //	0110 1000 = 0x68
    //   !!!! !---------- a_vecm_en = 1 => enable acceleration vector magnitude detection feature
    //	!!!! ----------- a_vecm_updm = 0 => update initial reference
    //	!!!- ----------- a_vecm_initm = 0 => the function uses the current x/y/z accelerometer output data at the time when the vector magnitude function is enabled.
    //	!!-------------- a_vecm_ele = 1 => event latching enabled
    prvSensors_writeRegs (A_VECM_CFG, 0x48,1);

    #if (DEBUG_LVL == DEBUG_MEDIUM)
    prvSensors_readRegs(A_VECM_THS_MSB,&readdata,1);
    DEBUG_PRINT1 (DEBUG_MEDIUM,"[sensor]:VECM MSB = %x\r\n",readdata);
    prvSensors_readRegs(A_VECM_THS_LSB,&readdata,1);
    DEBUG_PRINT1 (DEBUG_MEDIUM,"[sensor]:VECM LSB = %x\r\n",readdata);
    #endif
}
/*******************************************************************************
*    Function: Sensors_gotoSleep
*
*  Parameters:  uint8_t regAddr
*     Returns:  None
* Description:  read register of sensors
*******************************************************************************/
void Sensors_gotoSleep(void)
{
    uint8_t readdata;
    uint8_t sysmod = 0;
    uint8_t intSource;

    /*!!!Modification of control register can only occur in Standby mode */
    prvSensors_readRegs(CTRL_REG1,&readdata,1);
    prvSensors_writeRegs(CTRL_REG1, readdata & ~ACTIVE_MASK,1);

    uDelay(200);//delay 200us 
    /*Check if we are in STANDBY mode using SYSMOD*/
    prvSensors_readRegs(SYSMOD_REG,&sysmod,1);
    if ((sysmod & SYSMOD_MASK) != SYSMOD_STANDBY)
    {
        DEBUG_PRINT1 (DEBUG_HIGH,"[sensor]:Entering stby mode failed, current mode is %x\r\n",sysmod);
    }

    /*Enable low power mode in sleep state, high res in normal state*/
    prvSensors_writeRegs (CTRL_REG2, SMOD_LOW_POWER|MOD_HIGH_RES,1);

    /*Set full-scale range to +- 4G, ¡À0.488 mg/LSB.*/
    prvSensors_writeRegs (XYZ_DATA_CFG_REG,FULL_SCALE_4G,1);  

    /*Config the FIFO*/
    /*First set the F_SETUP_REG value as 0 before modification*/
    prvSensors_writeRegs(F_SETUP_REG, 0x00,1); 
    /*Set the FIFO to trigger mode,and the watermark value is 10*/
    prvSensors_writeRegs(F_SETUP_REG, F_MODE_TRIGGER|0x0A,1); 
    /*Set the FIFO interrupt Trigger for VECM Detection*/
    prvSensors_writeRegs(TRIG_CFG_REG, TRIG_A_VECM_MASK,1); 

    /*Config the VECM trigger thresold in sleep mode*/
    Sensors_VECM_Theshold_Config(G_SENSOR_LEVEL_2_VECM_THRESHOLD); 

    /*Enable FIFO interrupt detection interrupts*/
    prvSensors_writeRegs (CTRL_REG4, INT_EN_FIFO_MASK,1);

    /*route FIFO interrupt to INT1;INT2 is unused*/
    prvSensors_writeRegs (CTRL_REG5, (INT_CFG_FIFO_MASK),1);

    /*Enable A_VECM as a wake-up source*/
    //prvSensor_writeRegs (CTRL_REG3, FIFO_GATE_MASK|WAKE_A_VECM_MASK,1);

    /*Finally activate device with ODR=100Hz, FSR=4g*/
    prvSensors_writeRegs (CTRL_REG1, DATA_RATE_100HZ|ACTIVE_MASK,1);

    uDelay(200);//delay 200us 

    /*Check if we are in ACTIVE mode using SYSMOD*/
    prvSensors_readRegs(SYSMOD_REG,&sysmod,1);
    sysmod = sysmod & SYSMOD_MASK;
    if (sysmod != SYSMOD_WAKE)
    {
        DEBUG_PRINT1 (DEBUG_HIGH,"[Gsensor]:Entering active mode failed, current mode is %x\r\n",sysmod);
    }
    /*read INT_SOURCE to clear interrupt flag*/
    prvSensors_readRegs(INT_SOURCE_REG,&intSource,1); 
    if (GSENSOR_INT1_ACTIVE () != 0X01)
    {
        DEBUG_PRINT0 (DEBUG_HIGH,"[sensor]:Failed to config Gsensor before sleep\r\n");
    }
}

/*******************************************************************************
*    Function: Sensors_Clear_Crank_Count
*
*  Parameters:  
*     Returns:  None
* Description:  
*******************************************************************************/
/*void Sensors_Clear_Crank_Count(void)
{
	crank_count = 0;
}
*/
/*******************************************************************************
*    Function: Sensors_Get_Crank_Count
*
*  Parameters:  
*     Returns:  None
* Description:  
*******************************************************************************/
/*uint16_t Sensors_Get_Crank_Count(void)
{
	return crank_count;
}
*/
/*******************************************************************************
*    Function: Sensors_Clear_Shake_Count
*
*  Parameters:  
*     Returns:  None
* Description:  
*******************************************************************************/
/*void Sensors_Clear_Shake_Count(void)
{
	shake_count = 0;
}
*/
/*******************************************************************************
*    Function: Sensors_Get_Shake_Count
*
*  Parameters:  
*     Returns:  None
* Description:  
*******************************************************************************/
/*uint16_t Sensors_Get_Shake_Count(void)
{
    return shake_count;
}
*/
#ifdef CONFIG_G_SENSOR_UPLOAD
/*******************************************************************************
*    Function: Sensors_check_data
*
*  Parameters:  
*     Returns:  None
* Description:  
*******************************************************************************/
void Sensors_check_data(uint8_t source)
{
    int32_t total_g = (xAccel_mg*xAccel_mg)+(yAccel_mg*yAccel_mg)+(zAccel_mg*zAccel_mg);
    if ((xAccel_mg == yAccel_mg) || (zAccel_mg == yAccel_mg))
    {
        return;
    }
    if (((total_g - 1000000) > G_COUNT_START) && (total_g > 1000000) && (total_g < G_LIMIT))
    {
        //DEBUG_PRINT1(DEBUG_MEDIUM,"g too large[%d]\n\r",(total_g - 1000000));
        if (g_count == 0)
        {
            g_count = 1;
        }
        else
        {
            if (g_high_count < G_HIGH_COUNT_LIMIT)
            {
                g_high_data[g_high_count][0] = total_g - 1000000;
//                g_high_data[g_high_count][1] = IF_VehSpeed_B;
                g_acc_data[g_high_count][0] = xAccel_mg;
                g_acc_data[g_high_count][1] = yAccel_mg;
                g_acc_data[g_high_count][2] = zAccel_mg;
            }
            g_high_count++;
        }
    }
    else if (((1000000 - total_g) > GSENSOR_ALERT_TRIGGER) && (total_g < 1000000))
    {
        //DEBUG_PRINT1(DEBUG_MEDIUM,"g too low[%d]\n\r",(1000000-total_g));
        if (g_count == 0)
        {
            g_count = 1;
        }
        else
        {
            if (g_high_count < G_HIGH_COUNT_LIMIT)
            {
                g_high_data[g_high_count][0] = 1000000 - total_g;
//                g_high_data[g_high_count][1] = IF_VehSpeed_B;
                g_acc_data[g_high_count][0] = xAccel_mg;
                g_acc_data[g_high_count][1] = yAccel_mg;
                g_acc_data[g_high_count][2] = zAccel_mg;
            }
            g_high_count++;
        }
    }
    else if ((((1000000 - total_g) < G_REF_DELTA_LOW) && (total_g < 1000000)) ||
      (((total_g - 1000000) < G_REF_DELTA_HIGH) && (total_g > 1000000)))
    {
//        if (0 == IF_VehSpeed_B)
        {
            if ((g_stop_acc[0] == 0) && (g_stop_acc[1] == 0) && (g_stop_acc[2] == 0))
            {
                g_stop_acc[0] = xAccel_mg;
                g_stop_acc[1] = yAccel_mg;
                g_stop_acc[2] = zAccel_mg;
            }
            else
            {
                g_stop_acc[0] = ((49*g_stop_acc[0]) + xAccel_mg)/50;
                g_stop_acc[1] = ((49*g_stop_acc[1]) + yAccel_mg)/50;
                g_stop_acc[2] = ((49*g_stop_acc[2]) + zAccel_mg)/50;
            }
        }
/*        else if (0 < IF_VehSpeed_B)
        {
            if ((g_run_acc[0] == 0) && (g_run_acc[1] == 0) && (g_run_acc[2] == 0))
            {
                g_run_acc[0] = xAccel_mg;
                g_run_acc[1] = yAccel_mg;
                g_run_acc[2] = zAccel_mg;
            }
            else
            {
                g_run_acc[0] = ((49*g_run_acc[0]) + xAccel_mg)/50;
                g_run_acc[1] = ((49*g_run_acc[1]) + yAccel_mg)/50;
                g_run_acc[2] = ((49*g_run_acc[2]) + zAccel_mg)/50;
            }
        }*/
    }
    if (g_count == 1)
    {
        g_total_count++;
        if ((g_total_count > G_TOTAL_COUNT_LIMIT) || (g_high_count >= G_HIGH_COUNT_LIMIT))
        {
            if(g_high_count < G_HIGH_COUNT_LIMIT)
            {
            }
            else
            {
                uint8_t i = 0;
                uint8_t j = 0;
                for (i=1;i<G_HIGH_COUNT_LIMIT;i++)
                {
                    int32_t abs_delta = ((g_acc_data[i][0]-g_acc_data[i-1][0])*
                                    (g_acc_data[i][0]-g_acc_data[i-1][0])) + 
                                    ((g_acc_data[i][1]-g_acc_data[i-1][1])*
                                    (g_acc_data[i][1]-g_acc_data[i-1][1])) + 
                                    ((g_acc_data[i][2]-g_acc_data[i-1][2])*
                                    (g_acc_data[i][2]-g_acc_data[i-1][2])); 
                    if (abs_delta > G_X_ACC_LIMIT)
                        j++;
//                    DEBUG_PRINT2(DEBUG_MEDIUM,"g exceed limit[%d,%d]\n\r",g_high_data[i][0],g_high_data[i][1]);
//                    DEBUG_PRINT3(DEBUG_MEDIUM,"acc:[%d,%d,%d]\n\r",g_acc_data[i][0],g_acc_data[i][1],g_acc_data[i][2]);
                }
                if (j > 1)
//                gps_data_t gps_info;
//                vGps_Get_Gps_Info(&gps_info);
                {
                    // Send message to server
//                    uint16_t cog = GPS_Parse_Cog(gps_info.cog);
                    g_acc_data[G_HIGH_COUNT_LIMIT -3][0] = g_stop_acc[0];
                    g_acc_data[G_HIGH_COUNT_LIMIT -3][1] = g_stop_acc[1];
                    g_acc_data[G_HIGH_COUNT_LIMIT -3][2] = g_stop_acc[2];
                    g_acc_data[G_HIGH_COUNT_LIMIT -2][0] = g_run_acc[0];
                    g_acc_data[G_HIGH_COUNT_LIMIT -2][1] = g_run_acc[1];
                    g_acc_data[G_HIGH_COUNT_LIMIT -2][2] = g_run_acc[2];
                    memcpy(g_acc_stored,g_acc_data,G_HIGH_COUNT_DATA_SIZE*2);
//                    g_acc_stored[0] = cog;
//                    speed_stored = IF_VehSpeed_B;
//                    if (speed_stored > 0)
//                        OS_Send_Message(OS_IOT_TASK, Build_Message(TM_EVT_G_HIGH_REP,0x60));
                }
            }
            g_count = 0;
            g_high_count = 0;
            g_total_count = 0;
        }
    }
}
#endif
/*******************************************************************************
*    Function: GSensor_Read_Xyz_Value
*
*  Parameters:  
*     Returns:  None
* Description:  
*******************************************************************************/
/*void Sensors_Read_Xyz_Value(uint8_t source)
{
	FXLS1_sensorData_t sensorData;
	// read accel registers
	prvSensors_readRegs(OUT_X_MSB_REG, (uint8_t*) &sensorData,6);
	// process and align data
	xAccel = (int16_t)((sensorData.accelXMSB<<8) | (sensorData.accelXLSB));
	xAccel = xAccel >> 2;
	yAccel = (int16_t)((sensorData.accelYMSB<<8) | (sensorData.accelYLSB));
	yAccel = yAccel >> 2;
	zAccel = (int16_t)((sensorData.accelZMSB<<8) | (sensorData.accelZLSB));
	zAccel = zAccel >> 2;
	xAccel_mg=(int32_t)xAccel*488/1000;  //0x488mg
	yAccel_mg=(int32_t)yAccel*488/1000;
	zAccel_mg=(int32_t)zAccel*488/1000;	
//        DEBUG_PRINT3(DEBUG_MEDIUM,"[Sensor]: [X %d,Y %d,Z %d]\r\n",xAccel_mg,yAccel_mg,zAccel_mg);
}
*/

/*******************************************************************************
*    Function: FXLS1_FIFO_DataRead
*
*  Parameters:  
*     Returns:  None
* Description:  
*******************************************************************************/
static void FXLS1_FIFO_DataRead(void)
{
    uint8_t readvalue;
    #if (DEBUG_LVL == DEBUG_LOW)
    uint8_t i;
    #endif
    //Read the Status Register to clear interrupt flag
    prvSensors_readRegs(F_STATUS_REG, (uint8_t*) &readvalue,1);
    if(( readvalue & 0xE0) >= 0x60) // FIFO trigger event is dectected and all 32 samples ready?
    {
        /*read FIFO buffer*/
        prvSensors_readRegs(OUT_X_MSB_REG, (uint8_t*) FIFO_sensorDate,192);
        #if (DEBUG_LVL == DEBUG_LOW)
        // process and align data
        for(i=0;i<32;i++)
        {
            xAccel = (int16_t)((FIFO_sensorDate[i].accelXMSB<<8) | (FIFO_sensorDate[i].accelXLSB));
            xAccel = xAccel >> 2;
            yAccel = (int16_t)((FIFO_sensorDate[i].accelYMSB<<8) | (FIFO_sensorDate[i].accelYLSB));
            yAccel = yAccel >> 2;
            zAccel = (int16_t)((FIFO_sensorDate[i].accelZMSB<<8) | (FIFO_sensorDate[i].accelZLSB));
            zAccel = zAccel >> 2;
            xAccel_mg=(int32_t)xAccel*488/1000;  //0x488mg
            yAccel_mg=(int32_t)yAccel*488/1000;
            zAccel_mg=(int32_t)zAccel*488/1000;
            DEBUG_PRINT4(DEBUG_LOW,"[%d]: xA =%dmg  yA =%dmg zA =%dmg \r\n",i ,xAccel_mg,yAccel_mg,zAccel_mg);
        }
        #endif
    }
    else
    {
        DEBUG_PRINT1 (DEBUG_HIGH,"[Gsensor]:FIFO trigger event error, F_STATUS = %x\r\n",readvalue);
    }
    Sensors_FIFO_Fuction_Disable();
}

void Sensors_getFIFO(uint8_t source, uint8_t *data)
{
    memcpy(data,FIFO_sensorDate,192);
}

#ifdef CONFIG_G_SENSOR_UPLOAD
void Sensors_getHighG(uint8_t source, uint8_t *data)
{
    uint8_t i=0;
    //memcpy(data+1,g_acc_stored,96);
    for (i=0;i<G_HIGH_COUNT_DATA_SIZE;i++)
    {
        *(data+9+(i*2)) = (*(g_acc_stored+i) >> 8) & 0xff;
        *(data+10+(i*2)) = (*(g_acc_stored+i)) & 0xff;
    }
    *(data) = speed_stored;
}
#endif
/*******************************************************************************
*    Function: Sensors_Interrupt_Handler
*
*  Parameters:None
*     Returns:  None
* Description: 
*******************************************************************************/
#if 0
void Sensors_Interrupt_Handler(void)
{
    uint8_t readbyte;
    if (GSENSOR_INT1_ACTIVE () == 0x00)//gsensor interrupt occur
    {
        /*Read interrupt source to clear flag*/
        prvSensors_readRegs(INT_SOURCE_REG,&readbyte,1);
//        DEBUG_PRINT1 (DEBUG_MEDIUM,"[Gsensor]:Interrupt source = %x\r\n",readbyte);
        /*Is a VECM interrupt?*/	   
        if ((readbyte & SRC_A_VECM_MASK) == SRC_A_VECM_MASK)
        {
            crank_count ++;
            if (!PS_Eng_On())
            {
                if (shake_count < 0xFFFF)
                {
                    shake_count++;//for vehicle state detection
                }
            }

            if(crank_count > 250)
            {
                //vehicle must be in motion, check if engine on
                crank_count = 0;
            }	
            DEBUG_PRINT1 (DEBUG_LOW,"[sensor]:Crank Count = %d\r\n",crank_count);
        }
        else if ((readbyte & SRC_FIFO_MASK) == SRC_FIFO_MASK)/*Is a FIFO trigger interrupt?*/  
        {
            DEBUG_PRINT0 (DEBUG_MEDIUM,"[Gsensor]:FIFO trigger interrupt\r\n");
            FXLS1_FIFO_DataRead();
        }
    }
}
#endif

/*******************************************************************************
*    Function:  Sensor_Task
*
*  Parameters:  None
*     Returns:  None
* Description:  TASK to handle Sensor data read.
*******************************************************************************/
extern void Sensor_Task(void* pvParameters)
{
    Data_Message_T msg;
    uint8_t data;

    #ifdef USE_DEBUG
    DEBUG_PRINT0( DEBUG_MEDIUM, "[Sensor]:Sensor TASK Started!\r\n");
    #endif
    
    while(PS_Running()&&(Mdg_SW_Upgrage_mode==false))
    {
//    	prvGPS_checkAntenna();
//        if(E_OK == OS_Wait_Message(OS_SENSOR_TASK, &msg.all, MSec_To_Ticks(500)))
        {
            #if 0
            if(msg.parts.msg < GPS_NUM_EVENTS)
            {
                if(NULL != gps_event_handler[msg.parts.msg])
                {
                     (*gps_event_handler[msg.parts.msg])(msg.parts.data);
                }
            }
            #endif 
        }
//        OS_Wait_Resource(RES_UART_0, requeue_delay);
    }
    OS_Terminate_Task();
}

static uint8_t sGsensor_SendByte(uint8_t byte)
{
    /*!< Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(GSENSOR_SPI, SPI_I2S_FLAG_TXE) == RESET);

    /*!< Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(GSENSOR_SPI, byte);

    /*!< Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(GSENSOR_SPI, SPI_I2S_FLAG_RXNE) == RESET);

    /*!< Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(GSENSOR_SPI);
}
/*******************************************************************************
*    Function: prvSensors_readRegs
*
*  Parameters:  uint8_t regAddr
*     Returns:  result of reading:
* Description:  read regitor of sensors
*******************************************************************************/
static uint8_t prvSensors_readRegs(uint8_t regAddr, uint8_t* data, uint8_t len)
{
    uint8_t vol;
    /*!< Select the Gsensor: Chip Select low */
    GSENSOR_CS_LOW();
    /* send address byte0  */
    sGsensor_SendByte((~SPI_WRITE_BIT_MASK & regAddr));
    /* send address byte1  */
    sGsensor_SendByte((~SPI_WRITE_BIT_MASK | regAddr));

    while (len--)
    {
        *data=sGsensor_SendByte(0x0000);
        data++;
    }
    vol=*data;
    /*!< Select the Gsensor: Chip Select high */
    GSENSOR_CS_HIGH();
    return vol;
}

/*******************************************************************************
*    Function: prvSensors_writeRegs
*
*  Parameters:  uint8_t regAddr
*     Returns:  None
* Description:  write regitor of sensors
*******************************************************************************/
//static bool 
static void prvSensors_writeRegs(uint8_t regAddr, uint8_t data, uint8_t NumByteToWrite)
{
    /*!< Select the Gsensor: Chip Select low */
    GSENSOR_CS_LOW();
    /* send address byte0  */
    sGsensor_SendByte((SPI_WRITE_BIT_MASK | regAddr));
    /* send address byte1  */
    sGsensor_SendByte((SPI_WRITE_BIT_MASK & regAddr));
    /*!< while there is data to be written on the FLASH */
    while (NumByteToWrite--)
    {
        /*!< Send the current byte */
        sGsensor_SendByte(data);
        /*!< Point on the next byte to be written */
        data++;
    }
    /*!< Select the Gsensor: Chip Select high */
    GSENSOR_CS_HIGH();  //Width CS High time must be about 100ns
}
/*******************************************************************************
*    Function: GSensor_LowLevel_Init
*
*  Parameters:  None
*     Returns:  None
* Description:  Initializes the peripherals used by the SPI GSENSOR driver.
*******************************************************************************/
static void Sensors_LowLevel_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    #if 0 // already done in main
    /*!< GSENSOR_SPI_CS_GPIO, GSENSOR_SPI_MOSI_GPIO, GSENSOR_SPI_MISO_GPIO 
         and GSENSOR_SPI_SCK_GPIO Periph clock enable */
    RCC_APB2PeriphClockCmd(GSENSOR_CS_GPIO_CLK | GSENSOR_SPI_MOSI_GPIO_CLK | GSENSOR_SPI_MISO_GPIO_CLK |
                           GSENSOR_SPI_SCK_GPIO_CLK, ENABLE);

    /*!< GSENSOR_SPI Periph clock enable */
    RCC_APB2PeriphClockCmd(GSENSOR_SPI_CLK, ENABLE);
    #endif
    /*!< Configure GSENSOR_SPI pins: SCK */
    GPIO_InitStructure.GPIO_Pin = GSENSOR_SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GSENSOR_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure GSENSOR_SPI pins: MOSI */
    GPIO_InitStructure.GPIO_Pin = GSENSOR_SPI_MOSI_PIN;
    GPIO_Init(GSENSOR_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure GSENSOR_SPI pins: MISO */
    GPIO_InitStructure.GPIO_Pin = GSENSOR_SPI_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GSENSOR_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure GSENSOR_CS_PIN pin: GSENSOR CS pin */
    GPIO_InitStructure.GPIO_Pin = GSENSOR_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GSENSOR_CS_GPIO_PORT, &GPIO_InitStructure);

    /*!< Configure GSENSOR_INT1_PIN pin: GSENSOR INT1 pin */
    GPIO_InitStructure.GPIO_Pin = GSENSOR_INT1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GSENSOR_CS_GPIO_PORT, &GPIO_InitStructure);
}
/*====================================================================================*\
 * File Revision History
 *====================================================================================
 *
 * Date        userid  (Description on following lines:)
 * ----------- ------  ---------------------------------------------
 *
  ====================================================================================*/

