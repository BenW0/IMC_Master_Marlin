/*
  imc_i2c.h - wraps the i2c interface used as an interface for the
  discrete Intelligent Motor Controllers.

  Matthew Sorensen & Ben Weiss, University of Washington
*/

#ifndef __IMC_I2C_H
#define __IMC_I2C_H


#include <stdint.h>
#include "Configuration.h"
#include "imc_i2c_message_structs.h"
#include "imc_i2c_constants.h"

#ifdef IMC_ENABLED

// check to make sure we have a valid motherboard
#ifndef IMC_SYNC_PIN
#error "Please select a motherboard for which IMC_SYNC_PIN is defined"
#endif	// IMC_SYNC_PIN

// initializes i2c
uint8_t imc_init(void);

// slave/motor connectivity queries
bool imc_is_slave_connected(uint8_t motor_id);

// synchronization pin commands
void imc_sync_set();
bool imc_sync_release();
bool imc_sync_check();

// Sends the Initialize message to all slaves.
imc_return_type imc_send_init_all(uint16_t slave_hw_vers[IMC_MAX_MOTORS], uint16_t slave_fw_vers[IMC_MAX_MOTORS], 
	uint16_t slave_queue_depths[IMC_MAX_MOTORS], uint8_t retries = 3);
imc_return_type imc_send_init_all(const msg_initialize_t *params, rsp_initialize_t resps[IMC_MAX_MOTORS], uint8_t retries = 3);
imc_return_type imc_send_init_one(uint8_t motor_id, uint16_t *slave_hw_ver, uint16_t *slave_fw_ver, 
	uint16_t *slave_queue_depth, uint8_t retries = 3);
imc_return_type imc_send_init_one(uint8_t motor_id, const msg_initialize_t *params, rsp_initialize_t *resp, uint8_t retries = 3);

// Send the Status message
//imc_return_type imc_send_status_all(int32_t locations[IMC_MAX_MOTORS], uint32_t sync_errors[IMC_MAX_MOTORS], 
//	imc_axis_error statuss[IMC_MAX_MOTORS], uint8_t queued_moves[IMC_MAX_MOTORS], uint8_t retries = 3);
imc_return_type imc_send_status_all(rsp_status_t resps[IMC_MAX_MOTORS], uint8_t retries = 3);
//imc_return_type imc_send_status_one(uint8_t motor_id, int32_t *location, uint32_t *sync_error, imc_axis_error *status, uint8_t *queued_moves, uint8_t retries = 3);
imc_return_type imc_send_status_one(uint8_t motor_id, rsp_status_t *resp, uint8_t retries = 3);

// Send Home message
//imc_return_type imc_send_home_all(int32_t old_positions[IMC_MAX_MOTORS], uint8_t retries = 3);
imc_return_type imc_send_home_all(rsp_home_t resps[IMC_MAX_MOTORS], uint8_t retries = 3);
//imc_return_type imc_send_home_one(uint8_t motor_id, int32_t *old_position, uint8_t retries = 3);
imc_return_type imc_send_home_one(uint8_t motor_id, rsp_home_t *resp, uint8_t retries = 3);

// Send Queue Move message
imc_return_type imc_send_queue_all(const msg_queue_move_t params[IMC_MAX_MOTORS], uint8_t retries = 3);
//imc_return_type imc_send_queue_one(uint8_t motor_id, int32_t length, uint32_t total_length, uint32_t initial_rate, uint32_t final_rate, 
//	uint32_t acceleration, uint32_t stop_accelerating, uint32_t start_decelerating, uint8_t retries = 3);
imc_return_type imc_send_queue_one(uint8_t motor_id, const msg_queue_move_t *params, uint8_t retries = 3);

// Send Get Parameter message
imc_return_type imc_send_get_param_all(imc_axis_parameter param_id, uint32_t values[IMC_MAX_MOTORS], uint8_t retries = 3);
imc_return_type imc_send_get_param_one(uint8_t motor_id, imc_axis_parameter param_id, uint32_t *value, uint8_t retries = 3);

// Send Set Parameter message
imc_return_type imc_send_set_param_all(imc_axis_parameter param_id, uint32_t value, uint8_t retries = 3);
imc_return_type imc_send_set_param_one(uint8_t motor_id, imc_axis_parameter param_id, uint32_t value, uint8_t retries = 3);

// global variables
extern const imc_param_type imc_param_types[];

#endif	// IMC_ENABLED
#endif // __IMC_I2C_H