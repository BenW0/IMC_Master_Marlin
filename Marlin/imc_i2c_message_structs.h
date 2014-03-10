/* 
This file contains definitions for all messages in the specification of the IMC protocol.

(C) 2014, Ben Weiss & Matthew Sorensen
 */
#ifndef message_structs_h
#define message_structs_h

#include <stdint.h>
#include "imc_i2c_constants.h"

////////////////////// Message Packet Structures /////////////////////////////////
// All
typedef struct {
  uint16_t host_revision;
  uint8_t  reserved[6];
} __attribute__ ((packed)) msg_initialize_t;

//typedef struct {
//  ;  // no fields in message
//} __attribute__ ((packed)) msg_status_t;

//typedef struct {
//  ;  // no fields in message
//} __attribute__ ((packed)) msg_home_t;

typedef struct { // Assume we don't have to pad this on 32-bit systems
  int32_t length;
  uint32_t total_length;
  uint32_t initial_rate;
  uint32_t nominal_rate;
  uint32_t final_rate;
  uint32_t acceleration;
  uint32_t stop_accelerating;
  uint32_t start_decelerating;
} __attribute__ ((packed)) msg_queue_move_t;

typedef struct {
  uint8_t param_id;
} __attribute__ ((packed)) msg_get_param_t;

typedef struct {
  uint32_t param_value;
  uint8_t param_id;
} __attribute__ ((packed)) msg_set_param_t;


///////////////////////// Response Packet Structures /////////////////////////////////
// All response packets begin with a single byte response character, not included in the
// structs below because it would mess with efficient packing on 32-bit microcontrollers.

typedef struct {
  uint16_t slave_hw_ver;
  uint16_t slave_fw_ver;
  uint16_t queue_depth;
} __attribute__ ((packed)) rsp_initialize_t;

typedef struct {
  uint16_t queued_moves;
  imc_axis_error status;
} __attribute__ ((packed)) rsp_status_t;


//typedef struct {
//} __attribute__ ((packed)) rsp_queue_move_t;

typedef struct {
  uint32_t value;
} __attribute__ ((packed)) rsp_get_param_t;

//typedef struct {
//} __attribute__ ((packed)) rsp_set_param_t;
  
  
const uint8_t imc_message_type_count = 6;    // number of message types defined.

//const uint8_t imc_message_length[imc_message_type_count + 1] = {0, sizeof(msg_initialize_t), 
//                              0/*sizeof(msg_status_t)*/, 0/*sizeof(msg_home_t)*/, sizeof(msg_queue_move_t), 
//                              sizeof(msg_get_param_t), sizeof(msg_set_param_t)};

//const uint8_t imc_resp_length[imc_message_type_count + 1] = {0, sizeof(rsp_initialize_t), 
//                              sizeof(rsp_status_t), sizeof(rsp_home_t), 0/*sizeof(rsp_queue_move_t)*/,
//                              sizeof(rsp_get_param_t), 0/*sizeof(rsp_set_param_t)*/};
#endif


