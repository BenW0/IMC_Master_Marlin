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
  IMC_PARAM_ERROR_INFO1,			// 0
  IMC_PARAM_ERROR_INFO2,			// 1
  IMC_PARAM_FLIP_AXIS,				// 2
  IMC_PARAM_HOME_DIR,				// 3
  IMC_PARAM_MIN_SOFTWARE_ENDSTOPS,	// 4
  IMC_PARAM_MAX_SOFTWARE_ENDSTOPS,	// 5
  IMC_PARAM_MIN_LIMIT_EN,			// 6
  IMC_PARAM_MIN_LIMIT_INV,			// 7
  IMC_PARAM_MIN_LIMIT_PULLUP,		// 8
  IMC_PARAM_MAX_LIMIT_EN,			// 9
  IMC_PARAM_MAX_LIMIT_INV,			// 10
  IMC_PARAM_MAX_LIMIT_PULLUP,		// 11
  IMC_PARAM_MIN_POS,				// 12
  IMC_PARAM_MAX_POS,				// 13
  IMC_PARAM_HOME_POS,				// 14
  IMC_PARAM_HOMING_FEEDRATE,		// 15
  IMC_PARAM_MOTOR_ON,				// 16
  IMC_PARAM_MOTOR_IDLE_TIMEOUT,		// 17
  IMC_PARAM_SLOWDOWN				// 18
} imc_axis_parameter;

#define IMC_PARAM_COUNT   19		// number of parameters

typedef enum __attribute__ ((__packed__)) {
  IMC_PARAM_TYPE_UINT,
  IMC_PARAM_TYPE_INT,
  IMC_PARAM_TYPE_FLOAT
} imc_param_type;

// constant used when interpreting the type of axis parameters, esp for printing results to console.
#define IMC_PARAM_TYPES { \
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_INT, \
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_INT, \
	IMC_PARAM_TYPE_INT, \
	IMC_PARAM_TYPE_INT, \
	IMC_PARAM_TYPE_INT, \
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT,\
	IMC_PARAM_TYPE_UINT	\
}

typedef enum __attribute__ ((__packed__)) {
  IMC_NO_PULL,
  IMC_PULLUP,
  IMC_PULLDOWN
} imc_pullup_values;
  

typedef enum __attribute__ ((__packed__)) {
  IMC_ERR_NONE,
  IMC_ERR_CONTROL,
  IMC_ERR_ELECTRICAL,
  IMC_ERR_MECHANICAL,
  IMC_ERR_TIMEOUT
} imc_axis_error;

#endif
