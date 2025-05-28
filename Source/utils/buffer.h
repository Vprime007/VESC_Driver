#ifndef _BUFFER_H
#define _BUFEFR_H

#include <stdint.h>

/******************************************************************************
*   Public Definitions
*******************************************************************************/


/******************************************************************************
*   Public Macros
*******************************************************************************/


/******************************************************************************
*   Public Data Types
*******************************************************************************/


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
*  \brief Buffer append int16_t
*
*   Append a int16_t variable to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append uint16_t
*
*   Append a uint16_t variable to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append int32_t
*
*   Append a int32_t variable to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append uint32_t
*
*   Append a uint32_t variable to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append int64_t
*
*   Append a int64_t variable to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_int64(uint8_t* buffer, int64_t number, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append uint64_t
*
*   Append a uint64_t variable to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_uint64(uint8_t* buffer, uint64_t number, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append float 16bits
*
*   Append a float 16bits variable to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append float 32bits
*
*   Append a float 32bits variable to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append double 64bits
*
*   Append a double 64bits variable to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_double64(uint8_t* buffer, double number, double scale, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append float 32bits with auto-scale.
*
*   Append a float 32bits variable with auto-scale to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_float32_auto(uint8_t* buffer, float number, int32_t *index);

/***************************************************************************//*!
*  \brief Buffer append float 64bits with auto-scale.
*
*   Append a float 64bits variable with auto-scale to a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
void buffer_append_float64_auto(uint8_t* buffer, double number, int32_t *index);

/***************************************************************************//*!
*  \brief Get int16_t from buffer.
*
*   Return a int16_t variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);

/***************************************************************************//*!
*  \brief Get uint16_t from buffer.
*
*   Return a uint16_t variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);

/***************************************************************************//*!
*  \brief Get int32_t from buffer.
*
*   Return a int32_t variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);

/***************************************************************************//*!
*  \brief Get uint32_t from buffer.
*
*   Return a uint32_t variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);

/***************************************************************************//*!
*  \brief Get int64_t from buffer.
*
*   Return a int64_t variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
int64_t buffer_get_int64(const uint8_t *buffer, int32_t *index);

/***************************************************************************//*!
*  \brief Get uint64_t from buffer.
*
*   Return a uint64_t variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
uint64_t buffer_get_uint64(const uint8_t *buffer, int32_t *index);

/***************************************************************************//*!
*  \brief Get float 16bits from buffer.
*
*   Return a float 16bits variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);

/***************************************************************************//*!
*  \brief Get float 32bits from buffer.
*
*   Return a float 32bits variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);

/***************************************************************************//*!
*  \brief Get double 64bits from buffer.
*
*   Return a double 64bits variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
double buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index);

/***************************************************************************//*!
*  \brief Get float 32bits with auto-scale from buffer.
*
*   Return a float 32bits with auto-scale variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index);

/***************************************************************************//*!
*  \brief Get float 64bits with auto-scale from buffer.
*
*   Return a float 164bits with auto-scale variable from a buffer.
*   
*   Preconditions: None.
*
*   Side Effects: None.
*
*******************************************************************************/
double buffer_get_float64_auto(const uint8_t *buffer, int32_t *index);

#endif//_BUFFER_H