#file for storing all arduino serial commands
COMMAND_MOVE_UPPER = 0x01
COMMAND_MOVE_LOWER = 0x02
COMMAND_FIRE = 0x03
COMMAND_LIDAR = 0x04

#Amount degrees are shifted for float to integer conversion before sent over comms (this is inverse)
DEG_DECIMAL_SHIFT = 0.01