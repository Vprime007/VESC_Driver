#ifndef _VESC_DRIVER_H
#define _VESC_DRIVER_H

#include <stdint.h>
#include "datatypes.h"

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
typedef void(*VESC_CmdReturnCallback)(COMM_PACKET_ID command_id, 
                                      uint8_t *pData, 
                                      uint16_t length);

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
VESC_Ret_t VESC_InitDriver(void);

VESC_Ret_t VESC_AddDriver(VESC_Handle_t *pHandle);

VESC_Ret_t VESC_RemoveDriver(VESC_Handle_t handle);

VESC_Ret_t VESC_SendCmd(VESC_Command_t command, 
                        VESC_CmdReturnCallback callback, 
                        VESC_Handle_t handle);


#endif//_VESC_DRIVER_H
