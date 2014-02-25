/*
  imc_i2c.cpp - wraps the i2c interface used as an interface for the
  discrete Intelligent Motor Controllers.

  Matthew Sorensen & Ben Weiss, University of Washington

  TODO: Replace all tabs by two spaces to conform with the rest of Marlin
  TODO: Sync release on build start? On queue move?
  TODO: x_enable() needs to be rewritten.
  TODO: Figure out how to control endstops - presence vs. enabling.
  TODO: Fill out imc_quick_stop

  Questions to answer:
  * Will slaves wait for a sync before homing, or home immediately after message send (my preference)?
*/

#include <Wire.h>
#include "imc_i2c.h"
#include "marlin.h"
#include "Configuration.h"
#include "planner.h"
#include "imc_i2c_message_structs.h"
#include "language.h"
#include "pins.h"
#include "temperature.h"
#include "ultralcd.h"

#ifdef IMC_ENABLED

// Queue Balancer shaping constants
// if the slave queue is less than IMC_QUQUE_LOW_THRESH blocks, extra blocks are taken off the planner queue to prevent buffer underrun.
#define IMC_QUEUE_LOW_THRESH      8


// Global Variables===================================================================
const imc_param_type imc_param_types[IMC_PARAM_COUNT] = IMC_PARAM_TYPES;
uint16_t imc_queue_depth;				// minimum slave queue depth

// Local Variables ===================================================================
bool slave_exists[IMC_MAX_MOTORS];		// is the slave connected?
uint16_t queue_depth[IMC_MAX_MOTORS];	// What is each slave's queue depth?
uint16_t moves_queued_guess = 0;            // Set by imc_check_status, incremented when imc_send_queue_move_* is called, and cleared when imc_drain_queue or imc_flush_queue is called.

// Local Function Predeclares ========================================================
uint16_t imc_push_blocks(uint16_t blocks_to_push);
imc_return_type do_txrx(uint8_t motor, imc_message_type msg_type, const uint8_t *payload, uint8_t payload_len, uint8_t *resp, uint8_t resp_len, uint8_t retries);
uint8_t checksum(const uint8_t *data, uint8_t len, uint8_t startval = 0);

// Function Implementations =========================================================

// initializes i2c. Returns the number of motors successfully initialized.
uint8_t imc_init(void)
{
	uint16_t slave_hw_ver, slave_fw_ver;
	uint8_t i;
	imc_return_type ret;
	Wire.begin();
	// deactivate internal pullups. There are already hardware pullups in place.
	digitalWrite(SDA, 0);
	digitalWrite(SCL, 0);

	// configure the sync line to keep motion from happening until we're ready.
	imc_sync_set();
  moves_queued_guess = 0;


	// Query each of the slaves here and get queue depth, presence information
	msg_initialize_t params = {IMC_HOST_REVISION, {0, 0, 0, 0, 0, 0}};
	rsp_initialize_t resp;
	imc_queue_depth = 0xffff;
	uint8_t num_worked = 0;
	for(i = 0; i < IMC_MAX_MOTORS; ++i)
	{
		slave_exists[i] = true;		// if false, the function call will fail.
		ret = imc_send_init_one(i, &params, &resp, 5);
		if(IMC_RET_SUCCESS == ret)
		{
			num_worked++;
			queue_depth[i] = resp.queue_depth;
			imc_queue_depth = min(min_queue_depth, queue_depth[i]);
			#ifdef IMC_DEBUG_MODE
				SERIAL_ECHO("IMC Axis ");
				SERIAL_ECHO((int)i);
				SERIAL_ECHOLN(" worked!");
			#endif
		}
		else
		{
			#ifdef IMC_DEBUG_MODE
				SERIAL_ECHO("IMC Axis ");
				SERIAL_ECHO((int)i);
				SERIAL_ECHO(" returned ");
				SERIAL_ECHOLN((int)ret);
			#endif
			// slave did not respond or there was another error. Do not add to the list
			slave_exists[i] = false;
			queue_depth[i] = 0xffff;
		}
	}


	// upload limit switch and other functional information to each slave controlling a limited axis...
	// //||\\ TODO Some of these settings are probably stored in soft memory. Need to find the actual values not just global defaults.

	uint8_t min_limit_en = 1, max_limit_en = 1;
	#ifdef DISABLE_MIN_ENDSTOPS
		min_limit_en = 0;
	#endif
	#ifdef DISABLE_MAX_ENDSTOPS
		max_limit_en = 0;
	#endif

	// X axis
	if(slave_exists[0] && !DISABLE_X)
	{
		i = 0;
		ret = IMC_RET_SUCCESS;
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_FLIP_AXIS, INVERT_X_DIR));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOME_DIR, X_HOME_DIR));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_SOFTWARE_ENDSTOPS, min_software_endstops));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_SOFTWARE_ENDSTOPS, max_software_endstops));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_EN, min_limit_en));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_INV, X_MIN_ENDSTOP_INVERTING));
		#ifdef ENDSTOPPULLUP_XMIN
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_PULLUP, 1));
		#else
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_PULLUP, 0));
		#endif
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_EN, max_limit_en));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_INV, X_MAX_ENDSTOP_INVERTING));
		#ifdef ENDSTOPPULLUP_XMAX
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_PULLUP, 1));
		#else
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_PULLUP, 0));
		#endif
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_POS, X_MIN_POS));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_POS, X_MAX_POS));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOME_POS, X_HOME_POS));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOMING_FEEDRATE, (uint32_t)((float)homing_feedrate[i] * axis_steps_per_unit[i])));
		if( IMC_RET_SUCCESS != ret )
		{
			SERIAL_ECHOPGM(MSG_IMC_INIT_ERROR);
			SERIAL_ECHO("X");
			num_worked--;
			slave_exists[0] = false;
		}

	}
	else
		slave_exists[0] = false;
	
	// Y axis
	if(slave_exists[1] && !DISABLE_Y)
	{
		i = 1;
		ret = IMC_RET_SUCCESS;
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_FLIP_AXIS, INVERT_Y_DIR));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOME_DIR, Y_HOME_DIR));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_SOFTWARE_ENDSTOPS, min_software_endstops));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_SOFTWARE_ENDSTOPS, max_software_endstops));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_EN, min_limit_en));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_INV, Y_MIN_ENDSTOP_INVERTING));
		#ifdef ENDSTOPPULLUP_YMIN
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_PULLUP, 1));
		#else
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_PULLUP, 0));
		#endif
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_EN, max_limit_en));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_INV, Y_MAX_ENDSTOP_INVERTING));
		#ifdef ENDSTOPPULLUP_YMAX
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_PULLUP, 1));
		#else
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_PULLUP, 0));
		#endif
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_POS, Y_MIN_POS));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_POS, Y_MAX_POS));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOME_POS, Y_HOME_POS));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOMING_FEEDRATE, (uint32_t)((float)homing_feedrate[i] * axis_steps_per_unit[i])));
		if( IMC_RET_SUCCESS != ret )
		{
			SERIAL_ECHOPGM(MSG_IMC_INIT_ERROR);
			SERIAL_ECHO("Y");
			num_worked--;
			slave_exists[1] = false;
		}
	}
	else
		slave_exists[1] = false;
	
	// Z axis
	if(slave_exists[2] && !DISABLE_Z)
	{
		i = 2;
		ret = IMC_RET_SUCCESS;
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_FLIP_AXIS, INVERT_Z_DIR));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOME_DIR, Z_HOME_DIR));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_SOFTWARE_ENDSTOPS, min_software_endstops));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_SOFTWARE_ENDSTOPS, max_software_endstops));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_EN, min_limit_en));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_INV, Z_MIN_ENDSTOP_INVERTING));
		#ifdef ENDSTOPPULLUP_ZMIN
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_PULLUP, 1));
		#else
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_PULLUP, 0));
		#endif
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_EN, max_limit_en));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_INV, Z_MAX_ENDSTOP_INVERTING));
		#ifdef ENDSTOPPULLUP_ZMAX
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_PULLUP, 1));
		#else
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_PULLUP, 0));
		#endif
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_POS, Z_MIN_POS));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_POS, Z_MAX_POS));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOME_POS, Z_HOME_POS));
		ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOMING_FEEDRATE, (uint32_t)((float)homing_feedrate[i] * axis_steps_per_unit[i])));
		if( IMC_RET_SUCCESS != ret )
		{
			SERIAL_ECHOPGM(MSG_IMC_INIT_ERROR);
			SERIAL_ECHO("Z");
			num_worked--;
			slave_exists[2] = false;
		}
	}
	else
		slave_exists[2] = false;
	
	// Extruder axes
	for(i = 3; i < IMC_MAX_MOTORS; ++i)
	{
		if(slave_exists[i] && !DISABLE_E)
		{
			ret = IMC_RET_SUCCESS;
			if(3 == i)
				ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_FLIP_AXIS, INVERT_E0_DIR));
			else if(4 == i)
				ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_FLIP_AXIS, INVERT_E1_DIR));
			else if(5 == i)
				ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_FLIP_AXIS, INVERT_E2_DIR));

			//ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOME_DIR, Z_HOME_DIR));
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_SOFTWARE_ENDSTOPS, 0));
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_SOFTWARE_ENDSTOPS, 0));
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_EN, 0));
			//ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_INV, Z_MIN_ENDSTOP_INVERTING));
			//ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_LIMIT_PULLUP, ENDSTOPPULLUP_ZMIN));
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_EN, 0));
			//ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_INV, Z_MAX_ENDSTOP_INVERTING));
			//ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_LIMIT_PULLUP, ENDSTOPPULLUP_ZMAX));
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MIN_POS, 0));
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_MAX_POS, 0));
			//ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOME_POS, 0));
			ret = max(ret, imc_send_set_param_one(i, IMC_PARAM_HOMING_FEEDRATE, 0));
			if( IMC_RET_SUCCESS != ret )
			{
				SERIAL_ECHOPGM(MSG_IMC_INIT_ERROR);
				SERIAL_ECHO('A' + (char)(i - 3));
				num_worked--;
				slave_exists[i] = false;
			}
		}
		else
			slave_exists[i] = false;
	}

	return num_worked;
		
}


// Slave status queries

// imc_is_slave_connected - queries the imc_i2c module to find out if a slave motor was discovered at startup.
//  this function does not actively interrogate the slave, but relies upon information learned at startup.
// Params:
//   Motor ID: The ID of the motor (0 = X, 1 = Y, 2 = Z, 3 = A, 4 = B, etc...)
bool imc_is_slave_connected(uint8_t motor_id)
{
	if(motor_id < IMC_MAX_MOTORS)
		return slave_exists[motor_id];
	return false;
}

// imc_check_status
// checks with slaves and determines the queue state, if it isn't already known. Returns the maximum number of queued moves.
// This function applies to *all* active slaves
// Returns:
//   Function return - 0 = success, > 0 = error.
//   axis_errors - errors reported by each axis. Set to NULL to ignore. If any axis has errors, the return will be nonzero.
//   queued_moves - set by the function to the maximum number of queued moves. Set to NULL to ignore.
//   queue_disagreement - set by the function to True if some slaves report a different number of queued moves. This might happen if the query
//       just spans a move transition. Set to NULL to ignore.
imc_return_type imc_check_status(imc_axis_error axis_errors[IMC_MAX_MOTORS], uint16_t *queued_moves, bool *queue_disagreement)
{
  imc_return_type ret = IMC_RET_SUCCESS;
  rsp_status_t rsp;
  uint16_t min_queue = 0xffff, max_queue = 0;;
  for(uint8_t i = 0; i < IMC_MAX_MOTORS; ++i)
  {
    if(slave_exists[i])
    {
      ret = max(ret, imc_send_status_one(i, &rsp);
      if(NULL != axis_errors)
        axis_errors[i] = rsp.status;
      if(!ret)
      {
        min_queue = min(min_queue, rsp.queued_moves);
        max_queue = max(max_queue, rsp.queued_moves);
      }
    }
    if(NULL != axis_errors)
      axis_errors[i] = IMC_RET_SUCCESS;
  }
  if(NULL != queued_moves)
    *queued_moves = max_queue;
  if(NULL != queue_disagreement && 0xffff != min_queue)
    *queue_disagreement = (max_queue != min_queue);
  moves_queued_guess = max_queue;
  return ret;
}


// synchronization pin commands
// imc_sync_set - sets the synchronization line low
void imc_sync_set()
{
	pinMode(IMC_SYNC_PIN, OUTPUT);
	digitalWrite(IMC_SYNC_PIN, LOW);
}

// imc_sync_release - tristates the sync pin and returns the value read.
bool imc_sync_release()
{
	pinMode(IMC_SYNC_PIN, INPUT);
	return (bool)digitalRead(IMC_SYNC_PIN);
}

// returns the state of the sync pin (true -> HIGH, false -> LOW)
bool imc_sync_check()
{
	return (bool)digitalRead(IMC_SYNC_PIN);
}



// Queue management commands

// imc_balance_queues
// Balances the queues between the planner and the slaves. The routine tries to keep the number of queued moves in the planner
// and on the slaves roughly equal, unless the slaves are under IMC_QUEUE_LOW_THRESH, in which case moves are transferred to
// the slaves as fast as possible until the queue level rises above IMC_QUEUE_LOW_THRESH.
// Params: None
// Returns: 0 = success, >0 = error
imc_return_type imc_balance_queues(void)
{
  uint16_t slave_blocks = 0;
  uint16_t planner_blocks = 0;
  uint16_t total_blocks;
  bool slave_out_of_sync = false;
  imc_return_type ret = IMC_RET_SUCCESS;

  // first, get the number of blocks in the slave queue
  ret = imc_check_status(NULL, &slave_blocks, &slave_out_of_sync);
  if(ret)   // error state!
  {
    // check for more errors and report to console.
    imc_axis_error errs[IMC_MAX_MOTORS];
    imc_check_status(errs);
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("IMC Error! ");
    SERIAL_ERRORLN(ret);
    const char axis_codes[] = "XYZABC";
    for(uint8_t i = 0; i < IMC_MAX_MOTORS)
    {
      SERIAL_ERROR(axis_codes[i]);
      SERIAL_ERRORLN(errs[i]);
    }
    return ret;
  }
#ifdef IMC_DEBUG_MODE
  if(slave_out_of_sync)
  {
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Slaves out of sync!");
  }
#endif
  moves_queued_guess = slave_blocks;

  // now, get the # of blocks in the planner queue
  planner_blocks = movesplanned();

  // if we have no moves presently planned, set the sync pin so slaves don't start when we queue new moves.
  if(slave_blocks == 0)
    imc_sync_set();

  // is the slave buffer near empty?
  if(slave_blocks < IMC_QUEUE_LOW_THRESH)
  {
    // push blocks to the slave as fast as possible!
    moves_queued_guess += imc_push_blocks(IMC_QUEUE_LOW_THRESH - slave_blocks);
    
  }
  else
  {
    // balance the blocks evenly
    total_blocks = planner_blocks + slave_blocks;
    if(planner_blocks > total_blocks / 2)
    {
      if(slave_blocks + (planner_blocks - total_blocks/2) < imc_queue_depth)
        moves_queued_guess += imc_push_blocks(planner_blocks - total_blocks / 2);
      else
        moves_queued_guess += imc_push_blocks(imc_queue_depth - slave_blocks);
    }
  }

  // if there are blocks to be moved now but were't before, start the movement
  if(moves_queued_guess > 0 && 0 == slave_blocks)
    imc_sync_release();
}

// imc_push_blocks (local)
// If all slaves have queue space available, pops a move off of the local planner queue and sends it to the slaves.
// Params: blocks_to_push - number of blocks to pop off the local planner and send to the slaves.
// Returns: # of blocks pushed.
uint16_t imc_push_blocks(uint16_t blocks_to_push)
{
  uint16_t blocks_pushed = 0;
  if(0 == blocks_to_push)   // nothing to do
    return 0;

  msg_queue_move_t move_data[IMC_MAX_MOTORS];

  for(uint16_t block = 0; block < blocks_to_push; ++block)
  {
    // get the current block
    current_block = plan_get_current_block();
    if(NULL != current_block)
    {
      current_block->busy = true;

      // fill the move data structure for sending to the slaves.
      for(uint8_t k = 0; k < IMC_MAX_MOTORS; k++)
      {
        // the IMC motor controllers work in steps/min or steps/min^2
        move_data[k].acceleration = current_block->acceleration_st * 60 * 60;   // steps/min^2
        move_data[k].final_rate = current_block->final_rate;               // steps/min
        move_data[k].initial_rate = current_block->initial_rate;         // steps/min
        move_data[k].nominal_rate = current_block->nominal_rate;         // steps/min
        move_data[k].start_decelerating = current_block->decelerate_after;
        move_data[k].stop_accelerating = current_block->accelerate_until;
        move_data[k].total_length = current_block->step_event_count;
        move_data[k].length = 0;
      }
      move_data[0].length = current_block->steps_x;
      move_data[1].length = current_block->steps_y;
      move_data[2].length = current_block->steps_z;
      move_data[3].length = current_block->steps_e;

      // push this block to the slaves
      imc_send_queue_all(move_data);
      blocks_pushed++;

      // finish with the current block
      plan_discard_current_block();
    }
  }
  return blocks_pushed;
}

// imc_drain_queues
// Blocks until all slaves have emptied their queues by moving (does NOT erase queue blocks). This WILL start motion if queues are full
// but execution is manually paused using imc_sync_set() (though this condition should be very rare). This will drain both the 
// slave queues and the planner queue.
// 
// Params: none
// Returns: 0=success, >0 = error
imc_return_type imc_drain_queues(void)
{
  imc_return_type ret = IMC_RET_SUCCESS;
  uint16_t queued_moves = 1;
  // check - are queues already empty?
  if(moves_queued_guess)
  {
    ret = imc_check_status(NULL, &queued_moves, 10);
    if(!ret)    // continue only if no errors
	  {
      if(queued_moves == 0)
      {
        // nothing to be done. We're already drained.
        moves_queued_guess = 0;
        return ret;
      }
	  }
	  else
		  // error condition
		  return ret;
  }
  else
    // we already know the queue is empty
    return IMC_RET_SUCCESS;

  // make sure the sync line is released --> slaves can move
  imc_sync_release();

  // idle waiting for queues to empty.
  while(queued_moves)
  {
	  ret = imc_check_status(NULL, &queued_moves, 10);
    if(ret)   // error condition
      return ret;
    moves_queued_guess = queued_moves;
	  manage_heater();
	  manage_inactivity();
	  lcd_update();
  }

  moves_queued_guess = 0;
}

// imc_quick_stop
// Forces motion stop and deletes all entries in build queues. Used when cancelling a build. This function shadows stepper.cpp/quickStop
void imc_quick_stop(void)
{
  // TODO: fill this if used!
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
  //moves_queued_guess = 0;
  imc_drain_queues();
}

// imc_last_queue_state
// returns the queue state as of the last status check, WITHOUT querying slaves. The number returned should be regarded as an upper bound
// on the number of moves queued. Generally, use this to quickly check if no moves are queued.
uint16_t imc_last_queue_state(void)
{
  return moves_queued_guess;
}



// Slave location commands




// Message functions

// Sends the Initialize message to all slaves. Returns success, or an error code if any of the active slaves have an error.
// Params:
//   host_revision - host revision to send to the slave. Should usually be Configuration.h/IMC_HOST_REVISION.
//   retries - number of times to retry sending the packet before failing.
// Returns: Passing NULL as the pointer to slave_hw_vers will disable all return values and just send the message.
//   slave_hw_vers - array of the hardware versions of slaves. Only populated for active slave motors. Use imc_is_slave_connected() to 
//      figure out which ones are.
//   slave_fw_vers - array of the software versions reported by the slaves.
//   slave_queue_depths - array of the queue depths reported by the slaves.
imc_return_type imc_send_init_all(uint16_t slave_hw_vers[IMC_MAX_MOTORS], uint16_t slave_fw_vers[IMC_MAX_MOTORS], 
	uint16_t slave_queue_depths[IMC_MAX_MOTORS], uint8_t retries)
{
	msg_initialize_t params = {IMC_HOST_REVISION, {0, 0, 0, 0, 0, 0}};
	rsp_initialize_t resps[IMC_MAX_MOTORS];
	imc_return_type ret = imc_send_init_all(&params, resps, retries);

	if( NULL == slave_hw_vers ) return ret;
	for(uint8_t i = 0; i < IMC_MAX_MOTORS; ++i)
	{
		slave_hw_vers[i] = resps[i].slave_hw_ver;
		slave_fw_vers[i] = resps[i].slave_fw_ver;
		slave_queue_depths[i] = resps[i].queue_depth;
	}
	return ret;
}

// Sends the initialization message to all slaves. Returns success, or an error code if the active slaves have an error.
// Params:
//   params - msg_initialize_t structure with parameters to send to slaves.
//   retries - number of times to retry sending the packet before failing.
// Returns:
//   resps - list of responses received from the slaves. Only populated for online slaves (see imc_is_slave_connected()).
imc_return_type imc_send_init_all(const msg_initialize_t *params, rsp_initialize_t resps[IMC_MAX_MOTORS], uint8_t retries )
{
	imc_return_type ret = IMC_RET_SUCCESS;
	for( uint8_t i = 0; i < IMC_MAX_MOTORS; ++i )
		if( slave_exists[i] )
			ret = max(ret, imc_send_init_one(i, params, &resps[i], retries));
  moves_queued_guess = 0;
	return ret;
}

// pass NULL to slave_hw_ver to ignore/not set slave_hw_ver, slave_fw_ver, and slave_queue_depth.
imc_return_type imc_send_init_one(uint8_t motor_id, uint16_t *slave_hw_ver, uint16_t *slave_fw_ver, 
	uint16_t *slave_queue_depth, uint8_t retries )
{
	msg_initialize_t params = {IMC_HOST_REVISION, {0, 0, 0, 0, 0, 0}};
	rsp_initialize_t resp;
	imc_return_type ret = imc_send_init_one(motor_id, &params, &resp, retries);

	if( NULL == slave_hw_ver ) return ret;
	*slave_hw_ver = resp.slave_hw_ver;
	*slave_fw_ver = resp.slave_fw_ver;
	*slave_queue_depth = resp.queue_depth;
	return ret;
}

imc_return_type imc_send_init_one(uint8_t motor_id, const msg_initialize_t *params, rsp_initialize_t *resp, uint8_t retries )
{
	return do_txrx( motor_id, IMC_MSG_INITIALIZE, (const uint8_t *)params, sizeof(msg_initialize_t), (uint8_t*)resp, 
			sizeof(rsp_initialize_t), retries);
}

// Send the Status message
// Requests status from all slaves and returns a list of responses <resps>.
// Params:
//   retries - number of packet send/receive retries before fail.
// Returns:
//   function return - success(0)/error code
//   resps - filled by the function with the response structures of the motors (only for motors that are enabled).
imc_return_type imc_send_status_all(rsp_status_t resps[IMC_MAX_MOTORS], uint8_t retries )
{
	imc_return_type ret = IMC_RET_SUCCESS;
	for( uint8_t i = 0; i < IMC_MAX_MOTORS; ++i )
		if( slave_exists[i] )
			ret = max(ret, imc_send_status_one(i, &resps[i], retries));
	return ret;
}
// Same as above, but sending to only one motor.
imc_return_type imc_send_status_one(uint8_t motor_id, rsp_status_t *resp, uint8_t retries )
{
	return do_txrx( motor_id, IMC_MSG_STATUS, (const uint8_t *)NULL, 0, (uint8_t*)resp, 
			sizeof(rsp_status_t), retries);
}


// Send Home message
// Sends the homeing message to the slave. Returns immediately (i.e. does not wait for homeing to complete)
// Params:
//   retries - number of packet send/receive tries before fail.
// Returns:
//   function return - success(0)/error code
//   resps - filled by the function with the response structures of the motors (only for motors that are enabled).
imc_return_type imc_send_home_all(rsp_home_t resps[IMC_MAX_MOTORS], uint8_t retries )
{
	imc_return_type ret = IMC_RET_SUCCESS;
	for( uint8_t i = 0; i < IMC_MAX_MOTORS; ++i )
		if( slave_exists[i] )
			ret = max(ret, imc_send_home_one(i, &resps[i], retries));
	return ret;
}
// Same as above, but only one motor. pass NULL to old_position to ignore it.
imc_return_type imc_send_home_one(uint8_t motor_id, int32_t *old_position, uint8_t retries)
{
	rsp_home_t resp;
  imc_return_type ret;
  ret = imc_send_home_one(motor_id, &resp, retries);
  if(NULL != old_position)
    *old_position = resp.old_position;
  return ret;
}
// Same as above, but only one motor.
imc_return_type imc_send_home_one(uint8_t motor_id, rsp_home_t *resp, uint8_t retries )
{
	return do_txrx( motor_id, IMC_MSG_HOME, (const uint8_t *)NULL, 0, (uint8_t*)resp, 
			sizeof(rsp_home_t), retries);
}

// Send Queue Move message
// Sends the queue move packet to all motors.
// Params:
//   params - the parameters to send to each motor.
//   retries - the number of times to retry sending the packet.
// Returns:
//   function return - success(0)/error code
imc_return_type imc_send_queue_all(const msg_queue_move_t params[IMC_MAX_MOTORS], uint8_t retries )
{
	imc_return_type ret = IMC_RET_SUCCESS;
	for( uint8_t i = 0; i < IMC_MAX_MOTORS; ++i )
		if( slave_exists[i] )
			ret = max(ret, imc_send_queue_one(i, &params[i], retries));
  moves_queued_guess++;   // signal that the queue may be full now...
	return ret;
}
// Same as above, but for only one motor. DO NOT USE this function for normal operation, or estimates of queue capacity will
// be affected. It is made public only for debug and special code M453.
imc_return_type imc_send_queue_one(uint8_t motor_id, const msg_queue_move_t *params, uint8_t retries )
{
	return do_txrx( motor_id, IMC_MSG_QUEUEMOVE, (const uint8_t *)params, sizeof(msg_queue_move_t), (uint8_t*)NULL, 
			0, retries);
}

// Send Get Parameter message
// Sends the "Get Parameter" message to all slaves
// Params:
//   param_id - parameter ID (from the imc_axis_parameter list) to retrieve
//   retries - number of times to retry sending the packet
// Returns:
//   function return - success(0)/error code
//   values - set by the function with the response reported by each slave (only for slaves that are online)
imc_return_type imc_send_get_param_all(imc_axis_parameter param_id, uint32_t values[IMC_MAX_MOTORS], uint8_t retries )
{
	imc_return_type ret = IMC_RET_SUCCESS;
	for( uint8_t i = 0; i < IMC_MAX_MOTORS; ++i )
		if( slave_exists[i] )
			ret = max(ret, imc_send_get_param_one(i, param_id, &values[i], retries));
	return ret;
}
// Same as above, but for just one motor.
imc_return_type imc_send_get_param_one(uint8_t motor_id, imc_axis_parameter param_id, uint32_t *value, uint8_t retries )
{
	return do_txrx( motor_id, IMC_MSG_GETPARAM, (const uint8_t *)&param_id, sizeof(uint8_t), (uint8_t*)value, 
			sizeof(uint32_t), retries);
}

// Send Set Parameter message
// Sends the "Set Parameter" message to all slaves, setting all slaves parameters to the same value.
// Params:
//   param_id - parameter ID (from the imc_axis_parameter list) to retrieve
//   retries - number of times to retry sending the packet
//   value - value to assign for the parameter (reinterpret-cast all int/float parameters into uint32 for transmission)
// Returns:
//   function return - success(0)/error code
imc_return_type imc_send_set_param_all(imc_axis_parameter param_id, uint32_t value, uint8_t retries )
{
	imc_return_type ret = IMC_RET_SUCCESS;
	for( uint8_t i = 0; i < IMC_MAX_MOTORS; ++i )
		if( slave_exists[i] )
			ret = max(ret, imc_send_set_param_one(i, param_id, value, retries));
	return ret;
}
// Same as above, but for one motor
imc_return_type imc_send_set_param_one(uint8_t motor_id, imc_axis_parameter param_id, uint32_t value, uint8_t retries )
{
	msg_set_param_t msg;
	msg.param_id = param_id;
	msg.param_value = value;
	return do_txrx( motor_id, IMC_MSG_SETPARAM, (const uint8_t *)&msg, sizeof(msg_set_param_t), (uint8_t*)NULL, 
			0, retries);
}



// Do the transmit/receive of a packet (local function). The first byte of the response from the slave indicates the
// slave's acknowledgement of the packet and is returned in imc_return_type. If an IMC_RSP_UNKNOWN or IMC_RSP_ERROR response
// is received, the packet is re-transmitted up to <retries> times before exiting. This function does nothing if the specified
// <motor> is not _true_ in slave_exists[].
// 
// Params: 
//   motor - motor id to use. Must be 0<motor<IMC_MAX_MOTORS
//   msg_type - message type byte. Must be from the enum imc_message_type.
//   payload - the message payload (from one of the msg_* structures in imc_i2c_message_structs.h)
//   payload_len - length of payload, bytes.
//   resp_len - number of bytes to request in response from the slave.
//   retries - number of times to retry the packet if IMC_RSP_UNKNOWN or IMC_RSP_ERROR are returned by the slave.
//
// Returns:
//   function return - returns 0 for success, or an enum of errors if now, incorporating and expanding the first byte of 
//       response from the slave.
//   *resp - handle to an array of length <resp_len> in which to put the slave's response. Probably a handle to a rsp_* structure.
imc_return_type do_txrx(uint8_t motor, imc_message_type msg_type, const uint8_t *payload, uint8_t payload_len, uint8_t *resp, 
	uint8_t resp_len, uint8_t retries)
{
    uint8_t slave_addr = IMC_I2C_BASE_ADDRESS + motor;
	imc_return_type ret = IMC_RET_OTHER_ERROR;
	uint8_t checkval, respcode, respcheck;

	// sanity check - motor id valid?
	if( motor > IMC_MAX_MOTORS || !slave_exists[motor] )
		return IMC_RET_PARAM_ERROR;

	// create the packet checksum
	checkval = checksum((uint8_t*)&msg_type, 1);
	checkval = checksum(payload, payload_len, checkval);

	// send packet
	for( uint8_t i = 0; i < retries && ret != IMC_RET_SUCCESS; ++i )
	{
		Wire.beginTransmission(slave_addr);
		Wire.write(msg_type);			// send the header
		Wire.write(payload, payload_len);	// send the payload
		Wire.write(checkval);			// send the checksum
		switch(Wire.endTransmission())    // done transmitting.
		{
		case 0:				// success
			break;
		case 1: case 4:		// issues with the parameters. Won't be able to send this packet
			return IMC_RET_PARAM_ERROR;
			break;
		default:			// 2 and 3 are communication issues. Try again.
			ret = IMC_RET_COMM_ERROR;
			continue;		// retry (go to top. do not pass go. do not collect $200).
		}
		
		// read response. Will try <retries> times until we get a clean transmission.
		ret = IMC_RET_COMM_ERROR;
		for( uint8_t j = 0; j < retries; ++j )
		{
			// read back the response - we will need resp_len + 2 because of the added response byte and checksum.
			Wire.requestFrom(slave_addr, (uint8_t)(resp_len + 2));
        

			// did we get enough bytes?
			if( Wire.available() < resp_len + 2 )
			{
				ret = IMC_RET_COMM_ERROR;
				continue;
			}

			// read back the data
			respcode = Wire.read();
			Wire.readBytes((char*)resp, resp_len);
			respcheck = Wire.read();
		        
			// check for message integrity
			checkval = checksum(&respcode, 1);
			checkval = checksum(resp, resp_len, checkval);
			if( respcheck != checkval )
			{
				ret = IMC_RET_COMM_ERROR;
				continue;
			}

			ret = (imc_return_type)respcode;

			break;	// successful read. Finish loop.
		}

		// did we succeed at the read?
		if( IMC_RET_COMM_ERROR == ret )
			return ret;					// errored out of read; quit.

		// check for response byte value.
		switch(respcode)
		{
		case IMC_RSP_OK: case IMC_RSP_QUEUEFULL:
			// not a transmission error condition; transmission is complete.
			return ret;
			break;
		default :		// transmission error occurred. Retransmit original packet.
			continue;
		}
		
		// transmission completed successfully if execution reaches here.
		break;
	}

	return ret;
}

// Checksum()
// computes a simple xor checksum.
// Params:
//    data - array of data to perform the checksum on
//    len - length of data array (bytes)
//    startval - value to initialize the xor collector to. Used when combining different variables into one stream and checksumming together.
// Returns:
//    returns the 8-bit xor checksum.
uint8_t checksum(const uint8_t *data, uint8_t len, uint8_t startval)
{
	uint8_t XOR = startval;
	for(uint8_t i = 0; i < len; ++i)
	{
		XOR ^= data[i];
	}
	return XOR;
}


#endif // IMC_ENABLED