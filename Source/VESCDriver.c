/******************************************************************************
*   Includes
*******************************************************************************/
#include <stdbool.h>
#include <stdlib.h>

#include "VESCDriver.h"
#include "buffer.h"
#include "crc.h"

/******************************************************************************
*   Private Definitions
*******************************************************************************/
#define MAX_PACKET_BUFFER_SIZE              (512 + 8)
#define INVALID_DRIVER_INDEX                (0xFF)

#define UART_FRAME_START_OFFSET             (0)
#define UART_FRAME_START_SIZE               (1)//Byte

#define UART_SHORT_FRAME_ID                 (2)
#define UART_LONG_FRAME_ID                  (3)
#define UART_FRAME_END                      (3)

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

    static uint16_t payload_index = 0;
    static uint16_t payload_len = 0;

    switch(driver_table[handle].fsm_state){

        case VESC_FSM_WAIT_START:
        {
            payload_index = 0;
            payload_len = 0;

            if(byte == UART_SHORT_FRAME_ID){
                //Payload <= 256 bytes
                driver_table[handle].fsm_state = VESC_FSM_WAIT_LEN_LOW;

                payload_index = 2; //Payload start at index 2 when short frame

                //Store new byte in rx buffer
                driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
                driver_table[handle].rx_cptr++;
            }
            else if(byte == UART_LONG_FRAME_ID){

                payload_index = 3; //Payload start at index 3 when long frame

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
            payload_len = (byte << 8); //Store high byte of payload length

            driver_table[handle].fsm_state = VESC_FSM_WAIT_LEN_LOW;

            //Store new byte in rx buffer
            driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
            driver_table[handle].rx_cptr++;
        }
        break;

        case VESC_FSM_WAIT_LEN_LOW:
        {
            payload_len += byte; //Add low byte of payload length

            driver_table[handle].fsm_state = VESC_FSM_WAIT_PAYLOAD;

            //Store new byte in rx buffer
            driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
            driver_table[handle].rx_cptr++;
        }
        break;

        case VESC_FSM_WAIT_PAYLOAD:
        {
            //Store new byte in rx buffer
            driver_table[handle].rx_buffer[driver_table[handle].rx_cptr] = byte;
            driver_table[handle].rx_cptr++;

            if((driver_table[handle].rx_cptr >= (payload_index + payload_len))){
                //We have received the full payload, now we can wait for CRC
                driver_table[handle].fsm_state = VESC_FSM_WAIT_CRC_HIGH;
            }
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
            //Check if end byte is valid
            if(byte != UART_FRAME_END){
                //Clean rx_buffer and cptr 
                for(uint16_t i=0; i<MAX_PACKET_BUFFER_SIZE; i++){
                    driver_table[handle].rx_buffer[i] = 0;
                }
                driver_table[handle].rx_cptr = 0;

                //Restart frame processing
                driver_table[handle].fsm_state = VESC_FSM_WAIT_START;
                return;
            }

            if(driver_table[handle].rtn_callback == NULL){
                //No need to process further... the driver doesn't
                //give a shit about returned data...

                //Clean rx_buffer and cptr 
                for(uint16_t i=0; i<MAX_PACKET_BUFFER_SIZE; i++){
                    driver_table[handle].rx_buffer[i] = 0;
                }
                driver_table[handle].rx_cptr = 0;

                //Restart frame processing
                driver_table[handle].fsm_state = VESC_FSM_WAIT_START;
                return;
            }

            uint16_t buffer_crc = 0;

            if(driver_table[handle].rx_buffer[0] == UART_LONG_FRAME_ID){
                
                payload_index = 3;//Payload start at index 3 when long frame
                
                //Extract payload len from buffer
                payload_len = (driver_table[handle].rx_buffer[1] << 8);
                payload_len += (driver_table[handle].rx_buffer[2]);
            }
            else{

                payload_index = 2;//Payload start at index 2 when short frame

                //Extract payload len from buffer
                payload_len = driver_table[handle].rx_buffer[1];
            }

            //Extract CRC from buffer
            buffer_crc = (driver_table[handle].rx_buffer[payload_index + payload_len] << 8);
            buffer_crc += driver_table[handle].rx_buffer[payload_index + payload_len + 1];

            //Compare received CRC to calculated CRC
            uint16_t calculated_crc = crc16(&(driver_table[handle].rx_buffer[payload_index]), payload_len);
            if(buffer_crc != calculated_crc){
                //Clean rx_buffer and cptr 
                for(uint16_t i=0; i<MAX_PACKET_BUFFER_SIZE; i++){
                    driver_table[handle].rx_buffer[i] = 0;
                }
                driver_table[handle].rx_cptr = 0;

                //Restart frame processing
                driver_table[handle].fsm_state = VESC_FSM_WAIT_START;
                return;
            }

            //Pass data to the return callback of the driver
            driver_table[handle].rtn_callback(driver_table[handle].rx_buffer[payload_index],
                                              &(driver_table[handle].rx_buffer[payload_index+1]),
                                              payload_len);
            
            //Flush data and wait for new frame
            //Clean rx_buffer and cptr 
            for(uint16_t i=0; i<MAX_PACKET_BUFFER_SIZE; i++){
                driver_table[handle].rx_buffer[i] = 0;
            }
            driver_table[handle].rx_cptr = 0;

            //Restart frame processing
            driver_table[handle].fsm_state = VESC_FSM_WAIT_START;
            return;                           
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
*   \param[in]   rtn_callback       Function call when data is return from the VESC.
*   \param[out]  pHandle            VESC Driver instance handle.
*
*   \return     operation status
*
*******************************************************************************/
VESC_Ret_t VESC_AddDriver(VESC_SendOutDataFunc data_out_func, 
                          VESC_CmdReturnCallback rtn_callback,
                          VESC_Handle_t *pHandle){

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
    driver_table[index].rtn_callback = rtn_callback;
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
VESC_Ret_t VESC_SendCmd(VESC_Command_t command, VESC_Handle_t handle){

    //Check if params are valid
    if(handle >= VESC_DRIVER_MAX_INSTANCE){
        return VESC_STATUS_ERROR;
    }

    //Check if VESC Driver is active
    if(driver_table[handle].active_flag == false){
        return VESC_STATUS_ERROR;
    }

    uint16_t total_data_size = 0;
    command.length += 1;//Add one byte for the CMD ID

    //Fill tx buffer with command data
    if(command.length > 256){
        driver_table[handle].tx_buffer[0] = UART_LONG_FRAME_ID;
        driver_table[handle].tx_buffer[1] = (command.length >> 8) & 0xFF; //Length high byte
        driver_table[handle].tx_buffer[2] = command.length & 0xFF; //Length low byte
    
        driver_table[handle].tx_buffer[3] = command.command_id; //Command ID
        for(uint16_t i=0; i<command.length; i++){
            driver_table[handle].tx_buffer[i + 4] = command.pData[i]; //Payload
        }

        //Calculate CRC
        uint16_t crc = crc16(&(driver_table[handle].tx_buffer[3]), command.length + 1);
        driver_table[handle].tx_buffer[command.length + 4] = (crc >> 8) & 0xFF; //CRC high byte
        driver_table[handle].tx_buffer[command.length + 5] = crc & 0xFF; //CRC low byte

        //Add end byte
        driver_table[handle].tx_buffer[command.length + 6] = UART_FRAME_END;

        total_data_size = command.length + 6; //3 bytes for header + 2 bytes for CRC + 1 byte for end byte
    }
    else{
        driver_table[handle].tx_buffer[0] = UART_SHORT_FRAME_ID;
        driver_table[handle].tx_buffer[1] = command.length; //Length

        driver_table[handle].tx_buffer[2] = command.command_id; //Command ID
        for(uint16_t i=0; i<command.length; i++){
            driver_table[handle].tx_buffer[i + 3] = command.pData[i]; //Payload
        }

        //Calculate CRC
        uint16_t crc = crc16(&(driver_table[handle].tx_buffer[2]), command.length + 1);
        driver_table[handle].tx_buffer[command.length + 3] = (crc >> 8) & 0xFF; //CRC high byte
        driver_table[handle].tx_buffer[command.length + 4] = crc & 0xFF; //CRC low byte

        //Add end byte
        driver_table[handle].tx_buffer[command.length + 5] = UART_FRAME_END;

        total_data_size = command.length + 5; //2 bytes for header + 2 bytes for CRC + 1 byte for end byte
    }

    if(driver_table[handle].data_out_func != NULL){
        //Send data out using the provided function
        driver_table[handle].data_out_func(driver_table[handle].tx_buffer, total_data_size);
    }
    else{
        //No data out function provided, return error
        return VESC_STATUS_ERROR;
    }

    return VESC_STATUS_OK;
}

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
VESC_Ret_t VESC_SetDutyCycle(int32_t duty_cycle, VESC_Handle_t handle){

    //Check if params are valid
    if(handle >= VESC_DRIVER_MAX_INSTANCE){
        return VESC_STATUS_ERROR;
    }

    //Check if VESC Driver is active
    if(driver_table[handle].active_flag == false){
        return VESC_STATUS_ERROR;
    }

    //Re-format duty_cycle value
    uint8_t data_to_send[4] = {0};
    buffer_append_int32(data_to_send, duty_cycle, data_to_send);

    VESC_Command_t cmd = {
        .command_id = COMM_SET_DUTY,
        .length = sizeof(duty_cycle),
        .pData = data_to_send,
    };

    return VESC_SendCmd(cmd, handle);
}

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
VESC_Ret_t VESC_SetCurrent(int32_t current_ma, VESC_Handle_t handle){

    //Check if params are valid
    if(handle >= VESC_DRIVER_MAX_INSTANCE){
        return VESC_STATUS_ERROR;
    }

    //Check if VESC Driver is active
    if(driver_table[handle].active_flag == false){
        return VESC_STATUS_ERROR;
    }

    //Re-format current_ma value
    uint8_t data_to_send[4] = {0};
    buffer_append_int32(data_to_send, current_ma, data_to_send);

    VESC_Command_t cmd = {
        .command_id = COMM_SET_CURRENT,
        .length = sizeof(current_ma),
        .pData = data_to_send,
    };

    return VESC_SendCmd(cmd, handle);
}

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
VESC_Ret_t VESC_SetCurrentBrake(int32_t current_ma, VESC_Handle_t handle){

    //Check if params are valid
    if(handle >= VESC_DRIVER_MAX_INSTANCE){
        return VESC_STATUS_ERROR;
    }

    //Check if VESC Driver is active
    if(driver_table[handle].active_flag == false){
        return VESC_STATUS_ERROR;
    }

    //Re-format current_brake_ma value
    uint8_t data_to_send[4] = {0};
    buffer_append_int32(data_to_send, current_ma, data_to_send);

    VESC_Command_t cmd = {
        .command_id = COMM_SET_CURRENT_BRAKE,
        .length = sizeof(current_ma),
        .pData = data_to_send,
    };

    return VESC_SendCmd(cmd, handle);
}

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
VESC_Ret_t VESC_SetRPM(int32_t rpm, VESC_Handle_t handle){

    //Check if params are valid
    if(handle >= VESC_DRIVER_MAX_INSTANCE){
        return VESC_STATUS_ERROR;
    }

    //Check if VESC Driver is active
    if(driver_table[handle].active_flag == false){
        return VESC_STATUS_ERROR;
    }

    //Re-format rpm value
    uint8_t data_to_send[4] = {0};
    buffer_append_int32(data_to_send, rpm, data_to_send);

    VESC_Command_t cmd = {
        .command_id = COMM_SET_RPM,
        .length = sizeof(rpm),
        .pData = data_to_send,
    };

    return VESC_SendCmd(cmd, handle);
}

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
                                  VESC_Handle_t handle){

    //Check if params are valid
    if((pBytes == NULL) || (length == 0) || 
       (length > MAX_PACKET_BUFFER_SIZE) || 
       (handle >= VESC_DRIVER_MAX_INSTANCE)){

        return VESC_STATUS_ERROR;
    }

    //Check if VESC Driver is active
    if(!driver_table[handle].active_flag){
        return VESC_STATUS_ERROR;
    }

    //Procees each incoming byte
    for(uint16_t i=0; i<length; i++){
        processIncomingByte(pBytes[i], handle);
    }

    return VESC_STATUS_OK;
}

/******************************************************************************
*   Interrupts
*******************************************************************************/

