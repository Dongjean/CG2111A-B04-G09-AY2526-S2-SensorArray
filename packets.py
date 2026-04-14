"""
packets.py
Shared TPacket protocol constants for the Sensor Mini-Project.
"""

# ----------------------------------------------------------------
# TPACKET CONSTANTS
# ----------------------------------------------------------------

PACKET_TYPE_COMMAND  = 0
PACKET_TYPE_RESPONSE = 1
PACKET_TYPE_MESSAGE  = 2

COMMAND_ESTOP  = 0
COMMAND_COLOUR = 1
COMMAND_GO     = 2
COMMAND_CW     = 3
COMMAND_CCW    = 4
COMMAND_BACK   = 5
COMMAND_STOP   = 6
COMMAND_FASTER = 7
COMMAND_SLOWER = 8
COMMAND_ARM_MOVE = 10

# Define Servo IDs
SERVO_BASE     = 0
SERVO_SHOULDER = 1
SERVO_ELBOW    = 2
SERVO_GRIPPER  = 3
SERVO_SPEED = 4

RESP_OK     = 0
RESP_STATUS = 1
RESP_COLOUR = 2

STATE_RUNNING = 0
STATE_STOPPED = 1

MAX_STR_LEN  = 32
PARAMS_COUNT = 16

TPACKET_SIZE = 1 + 1 + 2 + MAX_STR_LEN + (PARAMS_COUNT * 4)  # = 100
TPACKET_FMT  = f'<BB2x{MAX_STR_LEN}s{PARAMS_COUNT}I'

# ----------------------------------------------------------------
# RELIABLE FRAMING: magic number + XOR checksum
# ----------------------------------------------------------------

MAGIC = b'\xDE\xAD'          # 2-byte magic number (0xDEAD)
FRAME_SIZE = len(MAGIC) + TPACKET_SIZE + 1   # 2 + 100 + 1 = 103
