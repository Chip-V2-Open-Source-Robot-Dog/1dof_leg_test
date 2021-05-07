'''
This class will allow us to store default parameters of the system!
'''

class DEFAULTS():
    #these guys might yet be useful
    Y_PID_LIMIT = 0.25
    X_PID_LIMIT = 0.15
    Z_PID_LIMIT = 0.1

    #origin defaults
    PLATFORM_ORIGIN = [0.29, 0.15] #this is the locatin of the IMU everything is in ref to that.

    #NT Defaults
    NT_DEFAULT_VAL = -1000000.0

    #IP Addresses
    RIO_IP = "10.20.20.2"
    JETSON_IP = "10.20.20.12"

    #epsilons for == testing & Defaults to NOT CHANGE
    EPSILON = 1.0
    #joint limits
    JOINT_LIMIT = 50.0
    JL_L = 5.0
    #zeroing and terrain offsets
    TERRAIN_OFFSET = 0.0 
    ZERO_OFFSET = -8.0
    #configurations to note
    STARTING_CONFIG = [8, 0, 8, 8, 0, 8, 8, 0, 8, 8, 0, 8]
    