/*********************************************************************************************************
 * FileName       : PRJ_CONFIG.H
 * Author         : 
 * Description    : Contains switchs, function list, special define and others which depense on project
 * Version        : 
 * Function List  : 
 * Config ID      : 
**********************************************************************/
#ifndef _PRJ_CONFIG_H_
#define _PRJ_CONFIG_H_

#include "gps.h"
/**********************************************************************
 * DEBUG Switches
 *********************************************************************/

/**********************************************************************
 * BENCH Switches
 *********************************************************************/
//#define SUPPORT_SENSORS
#define NORMAL_POWER_MODE 0
#define LOW_POWER_MODE 1
#define FULL_POWER_MODE 2

// temporary data setting for environment tests
//#define EV_TEST_DATA
//#define USE_DEFAULT_CONFIG
//#define TELM_TEST_PHONE
#define BOARD_VER (2)
#define DEVICE_DEFAULT_SN "300100200300400f"

/*
								Flash division
	--------------------------------------------------------------------------
				AREA								SIZE
	--------------------------------------------------------------------------
		Manufacture setting		 					4K
	--------------------------------------------------------------------------
				Config					  			4k
	--------------------------------------------------------------------------
				GPS Info							4k
	--------------------------------------------------------------------------
				GPS Log								512k
	--------------------------------------------------------------------------
				Battery Log							4k
	--------------------------------------------------------------------------
				OTA Info							4k
	--------------------------------------------------------------------------
				OTA Data							512k
	--------------------------------------------------------------------------
				Sys log								8k
	--------------------------------------------------------------------------
				Reserved							-
	--------------------------------------------------------------------------

*/

/**********************************************************************
 * FUNCTION Switches
 *********************************************************************/

/**********************************************************************
* DEBUG MACRO
**********************************************************************/

/*********************************************************************
* Project Flash Config
**********************************************************************/
/* Flash definitions */
#define FLASH_MANUFACTURE_SETTING_OFFSET		0
#define FLASH_CONFIG_OFFSET						(4 * 1024)
#define FLASH_GPS_INFO_OFFSET					(8 * 1024)
#define FLASH_GPS_LOG_OFFSET					(12 * 1024)
#define FLASH_BATTERY_LOG_OFFSET				(524 * 1024)
#define FLASH_OTA_INFO_OFFSET					(528 * 1024)
#define FLASH_OTA_DATA_OFFSET					(532 * 1024)
#define FLASH_SYS_LOG_OFFSET					(1044 * 1024)
#define FLASH_DEV_START_OFFSET					(1052 * 1024)
#define FLASH_RESERVED_OFFSET					(1056 * 1024)

/*
#define FLASH_MANUFACTURE_SETTING_SIZE			(FLASH_CONFIG_OFFSET - FLASH_MANUFACTURE_SETTING_OFFSET)
#define FLASH_CONFIG_SIZE						(FLASH_GPS_INFO_OFFSET - FLASH_CONFIG_OFFSET)
#define FLASH_GPS_INFO_SIZE						(FLASH_GPS_LOG_OFFSET - FLASH_GPS_INFO_OFFSET)
#define FLASH_GPS_LOG_SIZE						(FLASH_BATTERY_LOG_OFFSET - FLASH_GPS_LOG_OFFSET)
#define FLASH_BATTERY_LOG_SIZE					(FLASH_OTA_INFO_OFFSET - FLASH_BATTERY_LOG_OFFSET)
#define FLASH_OTA_INFO_SIZE						(FLASH_OTA_DATA_OFFSET - FLASH_OTA_INFO_OFFSET)
#define FLASH_OTA_DATA_SIZE						(FLASH_SYS_LOG_OFFSET - FLASH_OTA_DATA_OFFSET)
#define FLASH_SYS_LOG_SIZE						(FLASH_RESERVED_OFFSET - FLASH_SYS_LOG_OFFSET)
#define FLASH_RESERVED_SIZE						(2 * 1024 * 1024 - FLASH_RESERVED_OFFSET)
*/

#define INITIALIZE_DATA							(0xa5a5)
#define START_OTA_FLAG 							(0x55AA)
#define DEV_START_FLAG 							(0x55AA)
#define INIT_FLAG_DATA 							INITIALIZE_DATA

#define GPS_BACKUP_ADDR							FLASH_GPS_LOG_OFFSET
#define DEVICE_INFO_ADDR 						(FLASH_MANUFACTURE_SETTING_OFFSET)
#define OTA_LIVE_HEADER_ADDR					FLASH_OTA_INFO_OFFSET
#define DEVICE_CONFIG_ADDR						FLASH_CONFIG_OFFSET
#define OTA_DATA_ADDR							FLASH_OTA_DATA_OFFSET
#define OTA_HEADER_ADDR							FLASH_OTA_INFO_OFFSET
#define INIT_FLAG_ADDR							FLASH_MANUFACTURE_SETTING_OFFSET

#define DEVICE_INFO_LENGTH (14)

/* **********
 * 2017/2/9 LiHaibin modify. Extend CONFIG_DATA_LEN from 69 to 72
 * **********/
#define CONFIG_DATA_LEN (72)
#define GPS_INFO_LENGTH (10)
#define GPS_LOG_LENGTH (26)
#define BATTERY_LOG_LENGTH (7)
#define OTA_HEADER_LENGTH (36)

#define OTA_FLAG_START (0x01)
#define OTA_FLAG_GOING (0x02)
#define OTA_FLAG_END (0x03)

#define DEVICE_MODE_NA           (0x00)
#define DEVICE_MODE_LOGISTIC     (0x01)
#define DEVICE_MODE_SLEEP        (0x02)
#define DEVICE_MODE_TRACKING     (0x03)
#define DEVICE_MODE_ACTIVATED    (0x04)

/* *****************
 * LiHaibin Modify
 * 2017/2/9
 * *****************/
/***************************************************/
#define RL_WAKE_UP_TYPE_GENERAL     (uint8_t)(0)
#define RL_WAKE_UP_TYPE_DAY         (uint8_t)(1)
#define RL_WAKE_UP_TYPE_HOUR        (uint8_t)(2)
#define RL_WAKE_UP_TYPE_HALFHOUR    (uint8_t)(3)

/***************************************************/
#define DEV_SN_LENGTH (8)
#define BOARD_VERSION_LENGTH (4)

typedef union DevInfo_Sct_tag
{
    uint8_t byte[DEVICE_INFO_LENGTH];
    struct
    {
        uint8_t initialized[2];
        uint8_t dev_sn[DEV_SN_LENGTH];
        uint8_t mb_ver[BOARD_VERSION_LENGTH];
        uint8_t activation_status ;
        uint8_t sw_ver[4];
        uint8_t dev_type;
		uint8_t crc[2] ;
    }structData;
}DevInfo_Sct_t;

/* ****************
 * 2017/2/9 LiHaibin Modify.
 * Add 'wakeup_type' and 'wakeup_time'.
 * ****************/
typedef union Config_tag{
	uint8_t byte[CONFIG_DATA_LEN] ;
    struct
    {
        uint8_t config_ver[2];
        // Ignition detection parameters
        uint8_t ignition_on_threshold_low;
        uint8_t ignition_on_threshold_high;
        uint8_t ignition_on_down_max;
        uint8_t ignition_on_rise_max;
        uint8_t ignition_off_down_max;
        uint8_t ignition_on_absolute_voltage;
        uint8_t ignition_off_absolute_voltage;
        // Voltage alarm parameters
        uint8_t external_low_voltage_threshold;
        uint8_t internal_low_voltage_threshold;
        // Working parameters
        uint8_t wakeup_lasting_time[2];	/* unit: second */
        uint8_t data_resend_times;
        // Sleep parameters
        uint8_t sleep_time[2];	/* unit: minute */
        uint8_t network_fail_sleep_time;
        // GPS upload setting
        uint8_t GPS_upload_interval;
        uint8_t GPS_record_mileage[2];
        uint8_t GPS_record_interval;
        uint8_t GPS_record_turning;
        // tracking mode
        uint8_t tracking_mode_on_time[2]; /* unit: minute */
        uint8_t tracking_mode_sleep_time[2];
        // Log bytes
        uint8_t log_max_upload_bytes[2];
        // Network config
		uint8_t network_connection_retain_time;
		uint8_t network_connection_port[2];
        uint8_t network_connection_IP[16];
        uint8_t network_connection_APN[20];
        // crash
        // driving
        // theft

        uint8_t wakeup_type;
        uint8_t wakeup_timeHour;
        uint8_t wakeup_timeMinute;

        uint8_t tracking_on;
        uint8_t next_tracking_ts[4];

        uint8_t single_wake_num;
        uint8_t single_wake_ts[5][4];
        uint8_t crc[2];
    }structData ;
}Config_t;

typedef union GPS_Info_tag{
	uint8_t byte[GPS_INFO_LENGTH] ;
    struct
    {
        uint8_t log_total_number[2] ;
        uint8_t log_sector_read_pointer[2] ;
        uint8_t log_read_sector[2] ;
        uint8_t log_sector_write_pointer[2] ;
        uint8_t log_write_sector[2] ;
    }structData;
}GPS_Info_t ;

typedef union GPS_log_tag{
	uint8_t byte[GPS_LOG_LENGTH];
    struct
    {
        uint8_t postion_time[4];
        uint8_t latitude[4];
        uint8_t longitude[4];
        uint8_t altitude[4];
        uint8_t fix_flag;
        uint8_t cog[2];
        uint8_t speed;
        uint8_t used_satellite_number;
        uint8_t pdop;
        uint8_t lac[2];
        uint8_t cell_id[2];
    }structData;
}GPS_log_t;

typedef union Battery_log_tag{
    uint8_t byte[BATTERY_LOG_LENGTH];
    struct
    {
        uint8_t battery_total_number[2];
        uint8_t battery_voltage_log;
        uint8_t battery_log_time[4];
    }structData;
}Battery_log_t;

/*
typedef union DevCfg_Sct_tag
{
    uint8_t byte[CONFIG_DATA_LEN];
    struct
    {
        uint8_t ota_start;
        uint8_t server_ip[4];
        uint8_t server_port[2];
        uint8_t con_retry;
        uint8_t resend_retry;
        uint8_t expt_data_num;
        uint8_t drive_param[14];
        uint8_t factory_test;
        uint8_t version;
        uint8_t g_log_enable;
        uint8_t data_mining_enable;
    }structData;
}DevCfg_Sct_t;
*/

typedef union OTAHdr_Sct_tag{
	uint8_t byte[OTA_HEADER_LENGTH] ;
    struct
    {
        uint8_t ota_ctrl;
        uint8_t ota_target_ver[4];
        uint8_t ota_total_bytes[4];
		uint8_t ota_total_package[2];
		uint8_t ota_current_package_index[2];
		uint8_t ota_downloaded_bytes[4];
    }structData;
}OTAHdr_Sct_t ;

/* struct to store last valid GPS data */
typedef struct Last_GPS_Sct_tag
{
    uint8_t longitude[4];
    uint8_t latitude[4];
    uint8_t valid;
}Last_GPS_Sct_t;

/**********************************************************************
 *
 * REVISION RECORDS
 *
 *********************************************************************/
extern void Save_Param(void);
extern bool Load_Param(void);
extern void test_flash(uint32_t address, uint8_t *test_flash_data, uint16_t test_flash_len);

extern bool Get_Config(Config_t *config);
extern bool Set_Config(Config_t config);
extern bool Get_Config_Version(uint8_t *cfv);
extern uint8_t Get_IP_config(uint8_t *ip, uint8_t *port);
extern void Get_config_data(Config_t *config);
extern void Reset_default_config(void);

extern bool Get_Manufacture_Setting(DevInfo_Sct_t *g_devinfo_sct_t);
extern bool Get_Hardware_Version(uint8_t *hv);
extern void Set_Activation_Status(uint8_t status);
extern uint8_t Get_Activation_Status(void);
extern bool Get_Firmware_Version(uint8_t* fv);
extern bool Get_Device_Type(uint8_t *type);

extern bool Set_OTA_Info(OTAHdr_Sct_t g_ota_info_t);
extern bool Get_OTA_Target_Ver(uint16_t *ver);
extern bool Get_OTA_Info(OTAHdr_Sct_t *g_ota_info_t);
extern bool read_ota_data(uint8_t *data);
extern bool WriteOTAData(uint8_t *data , uint16_t len);
extern bool Start_Write_OTA_Data(uint32_t total_bytes);
extern bool Write_OTA_Data(uint8_t *data , uint16_t len);
extern bool End_Write_OTA_Data(uint16_t crc);
extern bool Read_OTA_Data(uint8_t *data , uint16_t len);

extern bool WriteBatteryData(Battery_log_t g_battery_log_t);
extern uint16_t Get_Battery_Log_Number(void);
extern void Set_Battery_Log_Number(uint16_t);

extern bool Get_GPS_Log_Write_Pointer(uint16_t *write_pointer);
bool Switch_GPS_Sturcture_data_to_log(gps_data_t gps_data, GPS_log_t *gps_log);
extern bool Set_GPS_Data_Next_Read_Pointer(uint8_t num);
extern uint8_t Read_GPS_Data(uint8_t *gps_log_t, uint8_t num);
extern bool Write_GPS_Data(GPS_log_t *g_gps_log_t);
extern uint16_t Get_GPS_Data_Total_Number(void);
extern uint8_t Get_Last_GPS_uploaded(void);
void Set_Last_GPS_uploader(uint8_t status);
extern uint32_t sys_get_cur_sec_offset(void);
extern uint32_t sys_get_sec_offset(uint8_t *time_buffer);

extern uint8_t Get_Low_Voltage_Uploaded(void);
extern void Set_Low_Voltage_Uploaded(uint8_t status);

/********************************************************************
 *
********************************************************************/
#endif //_PRJ_CONFIG_H
