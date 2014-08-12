Marlin_IMC makes substantial changes to the Marlin codebase to enable it to interface over I2C with a network of IMC nodes instead of driving stepper motors directly. The code that runs on an IMC node, as well as schematics, are available at https://github.com/BenW0/IMC-Axis-Simple

Changes made by Matthew Sorensen and Ben Weiss, 2014.

# Summary of major changes to Marlin

## Init

* Scan to see which slaves are present; enable/disable motors accordingly
* Send Initialization packets to all motors; upload limit switch configuration information
* FUTURE: Implement a timing sync routine that measures finish time spread on a move and adjusts clocks accordingly.

## Parser

* Implements custom M-codes (M450-M459) to enable direct communication with the motor controllers, primarily for debugging purposes

## Planner

* Moves get popped off the buffer by the I2C interface module; planner thinks the most recently sent move is "executing"

## Motor Drivers

* Disabled. All routines in stepper.cpp are replaced by links to the new functions in the I2C interface module.

## Pinnout

* Added constants for I2C and synchronization pins

## I2C Interface

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
FUTURE: Handle print pause

## Homing

* Change Marlin to send the homing packets to each axis

## Jogging/Build Plate Levelling

* Not yet modified.

## Misc

* FUTURE: Translate new constants in language.h into the various languages
