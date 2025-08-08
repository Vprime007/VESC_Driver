/******************************************************************************
*   Includes
*******************************************************************************/
#include <stdbool.h>
#include <math.h>

#include "buffer.h"

/******************************************************************************
*   Private Definitions
*******************************************************************************/


/******************************************************************************
*   Private Macros
*******************************************************************************/


/******************************************************************************
*   Private Data Types
*******************************************************************************/


/******************************************************************************
*   Private Functions Declaration
*******************************************************************************/


/******************************************************************************
*   Public Variables
*******************************************************************************/


/******************************************************************************
*   Private Variables
*******************************************************************************/


/******************************************************************************
*   Private Functions Definitions
*******************************************************************************/


/******************************************************************************
*   Public Functions Definitions
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
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index){

	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

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
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index){

	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

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
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index){

	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

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
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index){

	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

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
void buffer_append_int64(uint8_t* buffer, int64_t number, int32_t *index){

	buffer[(*index)++] = number >> 56;
	buffer[(*index)++] = number >> 48;
	buffer[(*index)++] = number >> 40;
	buffer[(*index)++] = number >> 32;
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

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
void buffer_append_uint64(uint8_t* buffer, uint64_t number, int32_t *index){

	buffer[(*index)++] = number >> 56;
	buffer[(*index)++] = number >> 48;
	buffer[(*index)++] = number >> 40;
	buffer[(*index)++] = number >> 32;
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

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
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index){

    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

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
void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index){

    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

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
void buffer_append_double64(uint8_t* buffer, double number, double scale, int32_t *index){

	buffer_append_int64(buffer, (int64_t)(number * scale), index);
}

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
void buffer_append_float32_auto(uint8_t* buffer, float number, int32_t *index){

	if (fabsf(number) < 1.5e-38) {
		number = 0.0;
	}

	int e = 0;
	float sig = frexpf(number, &e);
	float sig_abs = fabsf(sig);
	uint32_t sig_i = 0;

	if (sig_abs >= 0.5) {
		sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
		e += 126;
	}

	uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
	if (sig < 0) {
		res |= 1U << 31;
	}

	buffer_append_uint32(buffer, res, index);
}

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
void buffer_append_float64_auto(uint8_t* buffer, double number, int32_t *index){

	float n = number;
	float err = (float)(number - (double)n);
	buffer_append_float32_auto(buffer, n, index);
	buffer_append_float32_auto(buffer, err, index);
}

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
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index){

	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

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
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index){

	uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

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
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index){

	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

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
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index){

	uint32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

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
int64_t buffer_get_int64(const uint8_t *buffer, int32_t *index){

	int64_t res = ((uint64_t) buffer[*index]) << 56 |
				  ((uint64_t) buffer[*index + 1]) << 48 |
				  ((uint64_t) buffer[*index + 2]) << 40 |
				  ((uint64_t) buffer[*index + 3]) << 32 |
				  ((uint64_t) buffer[*index + 4]) << 24 |
				  ((uint64_t) buffer[*index + 5]) << 16 |
				  ((uint64_t) buffer[*index + 6]) << 8 |
				  ((uint64_t) buffer[*index + 7]);
	*index += 8;
	return res;
}

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
uint64_t buffer_get_uint64(const uint8_t *buffer, int32_t *index){

	uint64_t res = ((uint64_t) buffer[*index]) << 56 |
				   ((uint64_t) buffer[*index + 1]) << 48 |
				   ((uint64_t) buffer[*index + 2]) << 40 |
				   ((uint64_t) buffer[*index + 3]) << 32 |
				   ((uint64_t) buffer[*index + 4]) << 24 |
				   ((uint64_t) buffer[*index + 5]) << 16 |
				   ((uint64_t) buffer[*index + 6]) << 8 |
				   ((uint64_t) buffer[*index + 7]);
	*index += 8;
	return res;
}

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
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index){

    return (float)buffer_get_int16(buffer, index) / scale;
}

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
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index){

    return (float)buffer_get_int32(buffer, index) / scale;
}

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
double buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index){
    
    return (double)buffer_get_int64(buffer, index) / scale;
}

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
float buffer_get_float32_auto(const uint8_t *buffer, int32_t *index){

	uint32_t res = buffer_get_uint32(buffer, index);

	int e = (res >> 23) & 0xFF;
	uint32_t sig_i = res & 0x7FFFFF;
	bool neg = res & (1U << 31);

	float sig = 0.0;
	if (e != 0 || sig_i != 0) {
		sig = (float)sig_i / (8388608.0 * 2.0) + 0.5;
		e -= 126;
	}

	if (neg) {
		sig = -sig;
	}

	return ldexpf(sig, e);
}

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
double buffer_get_float64_auto(const uint8_t *buffer, int32_t *index){

	double n = buffer_get_float32_auto(buffer, index);
	double err = buffer_get_float32_auto(buffer, index);
	return n + err;    
}

/******************************************************************************
*   Interrupts
*******************************************************************************/


