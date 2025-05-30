/******************************************************************************
*   Includes
*******************************************************************************/
#include <stdbool.h>
#include <stdlib.h>
#include "VESCDriver.h"

/******************************************************************************
*   Private Definitions
*******************************************************************************/
#define MAX_PACKET_BUFFER_SIZE              (512 + 8)
#define INVALID_DRIVER_INDEX                (0xFF)

/******************************************************************************
*   Private Macros
*******************************************************************************/


/******************************************************************************
*   Private Data Types
*******************************************************************************/
typedef enum VESC_FSM_State_e{
    VESC_FSM_WAIT_START,
    VESC_FSM_WAIT_LEN_HIGH,
    VESC_FSM_WAIT_LEN_LOW,
    VESC_FSM_WAIT_PAYLOAD,
    VESC_FSM_WAIT_CRC_HIGH,
    VESC_FSM_WAIT_CRC_LOW,
    VESC_FSM_WAIT_END,

    VESC_FSM_INVALID,
}VESC_FSM_State_t;

typedef struct VESC_Driver_s{
    bool active_flag;
    VESC_FSM_State_t fsm_state;
    uint8_t rx_buffer[MAX_PACKET_BUFFER_SIZE];
    uint8_t tx_buffer[MAX_PACKET_BUFFER_SIZE];
    uint16_t rx_cptr;
    VESC_CmdReturnCallback rtn_callback;
    VESC_SendOutDataFunc data_out_func;
}VESC_Driver_t;

/******************************************************************************
*   Private Functions Declaration
*******************************************************************************/
static bool isDriverTableFull(void);
static uint8_t getFirstAvailableDriverIndex(void);
static void processIncomingByte(uint8_t byte, VESC_Handle_t handle); 

/******************************************************************************
*   Public Variables
*******************************************************************************/


/******************************************************************************
*   Private Variables
*******************************************************************************/
static VESC_Driver_t driver_table[VESC_DRIVER_MAX_INSTANCE] = {0};

/******************************************************************************
*   Private Functions Definitions
*******************************************************************************/
/***************************************************************************//*!
*  \brief Is driver table full
*
*   This function return True if Driver table is full and False otherwise.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \return     (True -> Full / False -> Not full)
*
*******************************************************************************/
static bool isDriverTableFull(void){

    for(uint8_t i=0; i<VESC_DRIVER_MAX_INSTANCE; i++){

        if(driver_table[i].active_flag == false){
            return false;
        }
    }

    return true;
}

/***************************************************************************//*!
*  \brief Get first available index in table
*
*   This function return the first available index in the driver table.
*   If no index are available -> return INVALID_DRIVER_INDEX.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \return     Table index
*
*******************************************************************************/
static uint8_t getFirstAvailableDriverIndex(void){

    for(uint8_t i=0; i<VESC_DRIVER_MAX_INSTANCE; i++){

        if(driver_table[i].active_flag == false){
            return i;
        }
    }

    return INVALID_DRIVER_INDEX;
}

/***************************************************************************//*!
*  \brief Process incoming byte for driver instance
*
*   This function process an incoming byte througt the FSM and package the 
*   bytes into a valid frame.  
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*   \param[in]  byte        Incoming byte
*   \param[in]  handle      Driver instance handle
*
*******************************************************************************/
static void processIncomingByte(uint8_t byte, VESC_Handle_t handle){

    switch(driver_table[handle].fsm_state){

        case VESC_FSM_WAIT_START:
        {
            if(byte == 2){
                //Payload <= 256 bytes
                driver_table[handle].fsm_state = VESC_FSM_WAIT_LEN_LOW;

                //Store new byte in rx buffer
                driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
                driver_table[handle].rx_cptr++;
            }
            else if(byte == 3){
                //Payload > 256 bytes
                driver_table[handle].fsm_state = VESC_FSM_WAIT_LEN_HIGH;

                //Store new byte in rx buffer
                driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
                driver_table[handle].rx_cptr++;
            }
            else{
                //Do nothing until we get a frame start byte...
            }
        }
        break;

        case VESC_FSM_WAIT_LEN_HIGH:
        {   
            driver_table[handle].fsm_state = VESC_FSM_WAIT_LEN_LOW;

            //Store new byte in rx buffer
            driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
            driver_table[handle].rx_cptr++;
        }
        break;

        case VESC_FSM_WAIT_LEN_LOW:
        {
            driver_table[handle].fsm_state = VESC_FSM_WAIT_PAYLOAD;

            //Store new byte in rx buffer
            driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
            driver_table[handle].rx_cptr++;
        }
        break;

        case VESC_FSM_WAIT_PAYLOAD:
        {

        }
        break;

        case VESC_FSM_WAIT_CRC_HIGH:
        {
            driver_table[handle].fsm_state = VESC_FSM_WAIT_CRC_LOW;

            //Store new byte in rx buffer
            driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
            driver_table[handle].rx_cptr++;
        }
        break;

        case VESC_FSM_WAIT_CRC_LOW:
        {
            driver_table[handle].fsm_state = VESC_FSM_WAIT_END;

            //Store new byte in rx buffer
            driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
            driver_table[handle].rx_cptr++;
        }
        break;

        case VESC_FSM_WAIT_END:
        {
            //Compare received CRC to calculated CRC

            
        }
        break;

        case VESC_FSM_INVALID:
        default:
        {
            //Do nothing... and contemplate the void...
        }
        break;
    }
}

/******************************************************************************
*   Public Functions Definitions
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
VESC_Ret_t VESC_InitDriver(void){

    for(uint8_t i=0; i<VESC_DRIVER_MAX_INSTANCE; i++){
        driver_table[i].active_flag = false;
        driver_table[i].fsm_state = VESC_FSM_INVALID;
        driver_table[i].rtn_callback = NULL;
        driver_table[i].data_out_func = NULL;
        driver_table[i].rx_cptr = 0;

        for(uint16_t j=0; j<MAX_PACKET_BUFFER_SIZE; j++){
            driver_table[i].rx_buffer[j] = 0;
            driver_table[i].tx_buffer[j] = 0;
        }
    }

    return VESC_STATUS_OK;
}
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
*   \param[out]  pHandle            VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_AddDriver(VESC_SendOutDataFunc data_out_func, VESC_Handle_t *pHandle){

    //Check if params are valid
    if((pHandle == NULL) || (data_out_func == NULL)){
        return VESC_STATUS_ERROR;
    }

    //Check if the driver table is full
    if(isDriverTableFull()){
        return VESC_STATUS_ERROR;
    }

    //Get the first available index in the driver table
    uint8_t index = getFirstAvailableDriverIndex();
    if(index == INVALID_DRIVER_INDEX){
        return VESC_STATUS_ERROR;
    }

    //Store new instance param in table
    driver_table[index].fsm_state = VESC_FSM_WAIT_START;
    driver_table[index].data_out_func = data_out_func;
    driver_table[index].rtn_callback = NULL;
    driver_table[index].rx_cptr = 0;

    for(uint16_t i=0; i<MAX_PACKET_BUFFER_SIZE; i++){
        driver_table[index].rx_buffer[i] = 0;
        driver_table[index].tx_buffer[i] = 0;
    }

    //Activate new instance
    driver_table[index].active_flag = true;
    *pHandle = index;

    return VESC_STATUS_OK;
}

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
VESC_Ret_t VESC_RemoveDriver(VESC_Handle_t handle){

    if(handle >= VESC_DRIVER_MAX_INSTANCE){
        return VESC_STATUS_ERROR;
    }

    driver_table[handle].active_flag = false;

    return VESC_STATUS_OK;
}

/***************************************************************************//*!
*  \brief Send command to VESC instance
*
*   This function is used to send a command to a VESC Driver instance.
*
*   If you want to get a return answer from the VESC, use the VESC_CmdReturnCallback.
*
*   Preconditions: Instance is active
*
*   Side Effects: None.
*
*   \param[in]    command            Command to send.
*   \param[in]    callback           Command return callback.
*   \param[in]    handle             VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_SendCmd(VESC_Command_t command, 
                        VESC_CmdReturnCallback callback, 
                        VESC_Handle_t handle){


    if(handle >= VESC_DRIVER_MAX_INSTANCE){
        return VESC_STATUS_ERROR;
    }

    return VESC_STATUS_OK;
}

/******************************************************************************
*   Interrupts
*******************************************************************************/


