"""
Store every configs like L1, L2
"""

L1 = 0.115
L2 = 0.115

MAX_POS = L1 + L2
MIN_POS = abs(L1 - L2)
ERROR = 0.001 # the small error possible in the measurement
MAX_INTERATION = 1000
INITIAL_GUESS = (0, 90) # initial guess for IK newton method, (10deg, 30deg)

# Motor ratios
MOTOR1_RATIO = 1.0
MOTOR2_RATIO = 5 / 3

FILE_NAME = "log.txt"
