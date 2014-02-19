/* 
This file contains definitions for all constants in the specification of the IMC protocol.

(C) 2014, Ben Weiss & Matthew Sorensen
 */
#ifndef imc_protocol_constants_h
#define imc_protocol_constants_h

typedef enum __attribute__ ((__packed__)) {
	IMC_RET_SUCCESS = 0,		// first four lines are imc_response_type.
	IMC_RET_SLAVE_UNKNOWN,
	IMC_RET_SLAVE_ERROR,
	IMC_RET_SLAVE_QUEUEFULL,
	IMC_RET_COMM_ERROR,
	IMC_RET_PARAM_ERROR,
	IMC_RET_OTHER_ERROR
} imc_return_type;

// constants in this enum need to match indices into the imc_message_length[] and imc_resp_length[] arrays in message_structs.h
typedef enum __attribute__ ((__packed__)) {
  IMC_MSG_INITIALIZE = 1,
  IMC_MSG_STATUS = 2,
  IMC_MSG_HOME = 3,
  IMC_MSG_QUEUEMOVE = 4,
  IMC_MSG_GETPARAM = 5,
  IMC_MSG_SETPARAM = 6,
} imc_message_type;


typedef enum  __attribute__ ((__packed__)) {
  IMC_RSP_OK = 0,		// this enum should be the first entries in imc_return_type
  IMC_RSP_UNKNOWN,
  IMC_RSP_ERROR,
  IMC_RSP_QUEUEFULL
} imc_response_type;

typedef enum __attribute__ ((__packed__)) {
  IMC_PARAM_ERROR_INFO1,
  IMC_PARAM_ERROR_INFO2,
  IMC_PARAM_FLIP_AXIS,
  IMC_PARAM_HOME_DIR,
  IMC_PARAM_MIN_SOFTWARE_ENDSTOPS,
  IMC_PARAM_MAX_SOFTWARE_ENDSTOPS,
  IMC_PARAM_MIN_LIMIT_EN,
  IMC_PARAM_MIN_LIMIT_INV,
  IMC_PARAM_MIN_LIMIT_PULLUP,
  IMC_PARAM_MAX_LIMIT_EN,
  IMC_PARAM_MAX_LIMIT_INV,
  IMC_PARAM_MAX_LIMIT_PULLUP,
  IMC_PARAM_MIN_POS,
  IMC_PARAM_MAX_POS,
  IMC_PARAM_HOME_POS,
  IMC_PARAM_HOMING_FEEDRATE,
  IMC_PARAM_MOTOR_ON,
  IMC_PARAM_MOTOR_IDLE_TIMEOUT,
  IMC_PARAM_SLOWDOWN
} imc_axis_parameter;

typedef enum __attribute__ ((__packed__)) {
  IMC_ERR_NONE,
  IMC_ERR_CONTROL,
  IMC_ERR_ELECTRICAL,
  IMC_ERR_MECHANICAL,
  IMC_ERR_TIMEOUT
} imc_axis_error;

#endif
