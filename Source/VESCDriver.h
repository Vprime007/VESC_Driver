#ifndef _VESC_DRIVER_H
#define _VESC_DRIVER_H

#include <stdint.h>

#include "datatypes.h"
#include "VESCDriver_cfg.h"

/******************************************************************************
*   Public Definitions
*******************************************************************************/


/******************************************************************************
*   Public Macros
*******************************************************************************/


/******************************************************************************
*   Public Data Types
*******************************************************************************/
typedef uint8_t (VESC_Handle_t);

typedef void(*VESC_SendOutDataFunc)(uint8_t *pData, uint16_t length);
typedef void(*VESC_CmdReturnCallback)(COMM_PACKET_ID command_id, 
                                      uint8_t *pData, 
                                      uint16_t length);

typedef struct VESC_Rtn_Values_s{
    int16_t fet_temperature;
    int16_t motor_temperature;
    int32_t avg_motor_current;
    int32_t avg_input_current;
    int32_t avg_id;
    int32_t avg_iq;
    int16_t duty_cycle_now;
    int32_t rpm;
    int16_t input_voltage;
    int32_t amp_hour;
    int32_t amp_hour_charged;
    int32_t watt_hour;
    int32_t watt_hour_charged;
    int32_t tach;
    int32_t tach_abs;
    uint8_t fault;
    int32_t pid_pos_now;
    uint8_t ctrl_id;
    int16_t ntc_temp_mos1;
    int16_t ntc_temp_mos2;
    int16_t ntc_temp_mos3;
    int32_t avg_vd;
    int32_t avg_vq;
    uint8_t comm_status;
}VESC_Rtn_Values_t;

typedef struct VESC_Command_s{
    COMM_PACKET_ID command_id;
    uint8_t *pData;
    uint16_t length;
}VESC_Command_t;

typedef enum VESC_Ret_e{
    VESC_STATUS_ERROR,
    VESC_STATUS_OK,
}VESC_Ret_t;

/******************************************************************************
*   Public Variables
*******************************************************************************/


/******************************************************************************
*   Error Check
*******************************************************************************/


/******************************************************************************
*   Public Functions
*******************************************************************************/
/***************************************************************************//*!
*  \brief VESC Driver initialization.
*
*   This function is used to init the VESC Driver module.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_InitDriver(void);

/***************************************************************************//*!
*  \brief Add VESC driver instance.
*
*   This function is used to add a VESC driver instance to the module.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[in]   data_out_func      Function used to send byte out to VESC.
*   \param[in]   rtn_callback       Function call when data is return from the VESC.
*   \param[out]  pHandle            VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_AddDriver(VESC_SendOutDataFunc data_out_func, 
                          VESC_CmdReturnCallback rtn_callback,
                          VESC_Handle_t *pHandle);

/***************************************************************************//*!
*  \brief Remove VESC driver instance.
*
*   This function is used to remove a VESC driver instance to the module.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[in]  handle              VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_RemoveDriver(VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Send command to VESC instance
*
*   This function is used to send a command to a VESC Driver instance.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    command            Command to send.
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_SendCmd(VESC_Command_t command, VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Pass incoming bytes to VESC instance
*
*   This function is used to pass incoming bytes to a VESC Driver instance.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    pBytes             Pointer to incoming bytes.
*   \param[in]    length             Length of incoming bytes.
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_PassIncomingBytes(uint8_t *pBytes, 
                                  uint16_t length, 
                                  VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Send COMM_SET_DUTY command.
*
*   This function is used to send a COMM_SET_DUTY command to 
*   a VESC Driver instance.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    duty_cycle         Duty cycle value (100% -> 100000).
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_SetDutyCycle(int32_t duty_cycle, VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Send COMM_SET_CURRENT command.
*
*   This function is used to send a COMM_SET_CURRENT command to 
*   a VESC Driver instance.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    current_ma         Current in ma.
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_SetCurrent(int32_t current_ma, VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Send COMM_SET_CURRENT_BRAKE command.
*
*   This function is used to send a COMM_SET_CURRENT_BRAKE command to 
*   a VESC Driver instance.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    current_ma         Brake current in ma.
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_SetCurrentBrake(int32_t current_ma, VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Send COMM_SET_RPM command.
*
*   This function is used to send a COMM_SET_RPM command to 
*   a VESC Driver instance.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    rpm                Motor RPM.
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_SetRPM(int32_t rpm, VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Set Motor relative current.
*
*   This function send a COMM_SET_CURRENT_REL command to a VESC Driver instance.
*   The motor current will be set relatively to it's maximum current.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    current_rel        Relative current (100% max current -> 100000)
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_SetCurrentRel(int32_t current_rel, VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Send comm keep alive.
*
*   This function send a COMM_ALIVE command to a VESC Driver instance to 
*   prevent a communication timeout.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_SendKeepAlive(VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Send device reboot.
*
*   This function send a COMM_REEBOT command to a VESC Driver instance to 
*   trigger a device reboot.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_SendReboot(VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Get Vesc Driver values.
*
*   This function is used to get the VESC driver values. The returned payload
*   is passed through the returned callback and can be extracted with
*   VESC_ExtractReturnedValues() function.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_GetValues(VESC_Handle_t handle);

/***************************************************************************//*!
*  \brief Extract returned values.
*
*   This function is used to extract values from the payload of the respond
*   from the VESC_GetValues command.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]      pRaw_data           Raw data buffer.
*   \param[in]      length              Buffer length.
*   \param[out]     pReturned_value     Pointer to store extracted values.     
*
*   \return         operation status
*
*******************************************************************************/
VESC_Ret_t VESC_ExtractReturnedValues(uint8_t *pRaw_data, 
                                      uint16_t length, 
                                      VESC_Rtn_Values_t *pReturned_value);

#endif//_VESC_DRIVER_H
