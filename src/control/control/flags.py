# Flags
POS_X = 1
NEG_X = 2
POS_Y = 4
NEG_Y = 8
POS_Z = 16
NEG_Z = 32
POS_ROLL = 64
NEG_ROLL = 128
POS_PITCH = 256
NEG_PITCH = 512
POS_YAW = 1024
NEG_YAW = 2048

flags = [2**i for i in range(16)]
