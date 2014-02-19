# Changes needed to Marlin

## Phase 1 - all motors are run on slave drivers

### Init

* <done>Scan to see which slaves are present; enable/disable motors accordingly
* <done>Send Initialization packets to all motors; upload limit switch configuration information
* <done>Find all the other places Wire.begin() is used (mostly panels) and figure out a way of making sure it's only called once.
* FUTURE: Implement a timing sync routine that measures finish time spread on a move and adjusts clocks accordingly.

### Parser

* Implement custom M-codes to enable direct communication with the motor controllers

### Planner

* Moves get popped off the buffer by the I2C interface module; planner thinks the most recently sent move is "executing"

### Motor Drivers

* Disable

### Pinnout

* Add constants for I2C and synchronization pins

### I2C Interface

* Implement send/receive with slaves
* Only talk with slaves we found during init (save bandwidth; timeouts)
Print Start:
* Pull down on the sync signal line
* Clear every slave's queue
* Fill queue to smallest slave queue depth
* Release (tri-state) the sync line. The sync line will remain tri-stated by the Master for the remainder of the print unless a pause is needed.
Idle Loop:
* Query slaves' status, determine the slave with the fewest free queue spots, check for errors
* Fire off that number of moves if we have them queued in the planner
Print Pause:
* **Look into this**

### Homing

* Change Marlin to send the homing packets to each axis

### Jogging/Build Plate Levelling

* **Look into this**

### Misc

* Translate new constants in language.h into the various languages


## Phase 2 - mix of local and remote motor drivers allowed

### Init

* Scan to see which slaves are present; enable/disable local motors accordingly.

### Motor Drivers

* Sync motor driver execution with bus execution (don't process next move until sync line goes hi)
* Pull sync line low after starting a move