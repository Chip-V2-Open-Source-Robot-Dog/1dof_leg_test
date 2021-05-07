#!/usr/bin/env python

'''
This node is both a publisher and a subscriber that will eventually 
read information from the ROS system and send it to the RIO as 
well as read information from the RIO and send it to the jetson. 

PUBLISHED TOPICS:
POS = Float64[] ARRAY OF RAW ENCODER POSITIONS S, H, K FOR EACH LEG
CURRENTS = Float64[] ARRAY OF OUTPUT CURRENTS FOR EACH LEG S, H, K
VBUS = Float64 AVERAGE BUS VOLTAGE OF THE SYSTEM
'''

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from CHIP_ROS.msg import ChipBodyControl #custom message definition for both pos + vel joint control
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from networktables import NetworkTables as nt
from DEFAULTS import DEFAULTS

'''some networkin stuff'''
import socket    
hostname = socket.gethostname()    
IPAddr = socket.gethostbyname(hostname)

'''
SOME INITIALIZATION CODE
'''
#start network tables server
nt.initialize(server=DEFAULTS.RIO_IP)
#initialize tables
readout=nt.getTable("rio_readout")

'''
DEFINE SOME CLASS VARIABLES
'''
POS_IN = []
VEL_IN = []
CURRENTS_IN = []
VBUS = 0.0
arr = []
connection = "Initializing"
mtype = 2

'''
DEFINING SOME CONSTANTS
'''
DEFAULT_VAL = DEFAULTS.NT_DEFAULT_VAL
PUB_RATE = 60 #60hz

#____________________________________________________________________________________________________________________________________________________________________
# THESE METHODS ARE NON-ROS ROBORIO SPECIFIC FUNCTIIONS FOR COMMUNICATIONS
#____________________________________________________________________________________________________________________________________________________________________

'''
This method gets the information from the RIO and organizes it in the arrays/nested arrays
'''
def read_rio():
    #GET GLOBAL VARS
    global VBUS
    global POS_IN
    global VEL_IN
    global CURRENTS_IN

    '''
    FOOT POSITIONS GETTING
    '''
    POS_IN = [readout.getNumber("fl_s", DEFAULT_VAL), readout.getNumber("fl_h", DEFAULT_VAL), readout.getNumber("fl_k", DEFAULT_VAL), readout.getNumber("fr_s", DEFAULT_VAL), readout.getNumber("fr_h", DEFAULT_VAL), readout.getNumber("fr_k", DEFAULT_VAL), readout.getNumber("bl_s", DEFAULT_VAL), readout.getNumber("bl_h", DEFAULT_VAL), readout.getNumber("bl_k", DEFAULT_VAL), readout.getNumber("br_s", DEFAULT_VAL), readout.getNumber("br_h", DEFAULT_VAL), readout.getNumber("br_k", DEFAULT_VAL)]
    VEL_IN = [readout.getNumber("fl_sv", DEFAULT_VAL), readout.getNumber("fl_hv", DEFAULT_VAL), readout.getNumber("fl_kv", DEFAULT_VAL), readout.getNumber("fr_sv", DEFAULT_VAL), readout.getNumber("fr_hv", DEFAULT_VAL), readout.getNumber("fr_kv", DEFAULT_VAL), readout.getNumber("bl_sv", DEFAULT_VAL), readout.getNumber("bl_hv", DEFAULT_VAL), readout.getNumber("bl_kv", DEFAULT_VAL), readout.getNumber("br_sv", DEFAULT_VAL), readout.getNumber("br_hv", DEFAULT_VAL), readout.getNumber("br_kv", DEFAULT_VAL)]

    '''
    GETTING CURRENTS
    '''
    #just get and add because it's total currents
    CURRENTS_IN = [readout.getNumber("fl_s_current", DEFAULT_VAL), readout.getNumber("fl_h_current", DEFAULT_VAL), readout.getNumber("fl_k_current", DEFAULT_VAL), readout.getNumber("fr_s_current", DEFAULT_VAL), readout.getNumber("fr_h_current", DEFAULT_VAL), readout.getNumber("fr_k_current", DEFAULT_VAL), readout.getNumber("bl_s_current", DEFAULT_VAL), readout.getNumber("bl_h_current", DEFAULT_VAL), readout.getNumber("bl_k_current", DEFAULT_VAL), readout.getNumber("br_s_current", DEFAULT_VAL), readout.getNumber("br_h_current", DEFAULT_VAL), readout.getNumber("br_k_current", DEFAULT_VAL)]

    '''
    GETTING VBUS
    '''
    VBUS = readout.getNumber("VBUS", DEFAULT_VAL)

#____________________________________________________________________________________________________________________________________________________________________
# HERE'S THE ACTUAL ROS NODE METHODS AND ETC.
#____________________________________________________________________________________________________________________________________________________________________
'''
created the diagnostic message to publish
'''
def create_diag_msg():
    global arr

    '''determine data'''
    if (VBUS==DEFAULTS.NT_DEFAULT_VAL):
        global connection
        global mtype
        connection = "Not Connected to RIO... retrying. Looking for RIO at " + DEFAULTS.RIO_IP
        mtype = 2
    else:
        global connection
        global mtype
        connection = "Connected to RIO at " + DEFAULTS.RIO_IP
        mtype = 0

    arr = DiagnosticArray()
    arr.status = [
        DiagnosticStatus(name='RoboRIO Actuation Controller', message=connection, level=mtype),
        DiagnosticStatus(name='Jetson TX1 Connection IP', message=DEFAULTS.JETSON_IP)
    ]

'''
This is a ROS SPECIFIC node that publishes the data at a certain rate
'''
def publisher():
    #CREATE THE PUBLISHERS
    POS_PUBLISHER = rospy.Publisher('POS', ChipBodyControl, queue_size=0) #default queue size is 0
    CURRENTS_PUBLISHER = rospy.Publisher('CURRENTS', Float64MultiArray, queue_size=0) #default queue size is 0
    VOLTAGE_PUBLISHER = rospy.Publisher('VBUS', Float64, queue_size=0) #default queue size is 0
    DIAGNOSTICS = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=1)

    #initilize the node
    rospy.init_node("RIO_READER")
    #set the publishing and reading rate
    rate = rospy.Rate(PUB_RATE) #in Hz

    while not rospy.is_shutdown():
        #NOW WE READ DATA FROM THE RIO
        read_rio()
        create_diag_msg()
        msg = ChipBodyControl()
        msg.pos = POS_IN
        msg.vel = VEL_IN
        #WRITE DATA NOW THAT read_rio has modified it
        POS_PUBLISHER.publish(msg)
        CURRENTS_PUBLISHER.publish(Float64MultiArray(data=CURRENTS_IN))
        VOLTAGE_PUBLISHER.publish(Float64(data=VBUS))
        DIAGNOSTICS.publish(arr)

        #sleep for the pub rate
        rate.sleep()

'''
not specific to ROS, this is the MAIN method, run this
'''
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

