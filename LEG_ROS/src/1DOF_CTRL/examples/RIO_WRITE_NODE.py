#!/usr/bin/env python

'''
This node is both a publisher and a subscriber that will eventually 
read information from the ROS system and send it to the RIO as 
well as read information from the RIO and send it to the jetson. 

PUBLISHED TOPICS:
FOOT_POSITIONS - Float64[] ARRAY OF FOOT POSITIONS XYZ
THETAS = Float64[] ARRAY OF THETAS FOR EACH LEG
LEG_CURRENTS = Float64[] ARRAY OF TOTAL CURRENT IN EACH LEG
DEAFULT_VALUE Float64 that tells the system what values means 
there is no communication with RIO

MIGHT WANT TO THINK ABOUT SPLITITNG THIS INTO TWO NODES, A PUBLISHER AND A SUBSCRIBER
'''

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from CHIP_ROS.msg import ChipBodyControl #added custom message definition
from networktables import NetworkTables as nt
from DEFAULTS import DEFAULTS

'''
SOME INITIALIZATION CODE
'''
#start network tables server
nt.initialize(server=DEFAULTS.RIO_IP)
#initialize tables
sender=nt.getTable("rio_input")

#____________________________________________________________________________________________________________________________________________________________________
# THESE METHODS ARE NON-ROS ROBORIO SPECIFIC FUNCTIIONS FOR COMMUNICATIONS
#____________________________________________________________________________________________________________________________________________________________________

'''
takes in four arrays of XYZ foot positions to send to the rio 
'''
def write(FL, FR, BL, BR):
    #data from FL
    sender.putNumber("fl_s", FL[0])
    sender.putNumber("fl_h", FL[1])
    sender.putNumber("fl_k", FL[2])

    #data from FR
    sender.putNumber("fr_s", FR[0])
    sender.putNumber("fr_h", FR[1])
    sender.putNumber("fr_k", FR[2])

    #data from BL
    sender.putNumber("bl_s", BL[0])
    sender.putNumber("bl_h", BL[1])
    sender.putNumber("bl_k", BL[2])

    #data from BR
    sender.putNumber("br_s", BR[0])
    sender.putNumber("br_h", BR[1])
    sender.putNumber("br_k", BR[2])


#____________________________________________________________________________________________________________________________________________________________________
# HERE'S THE ACTUAL ROS NODE METHODS AND ETC.
#____________________________________________________________________________________________________________________________________________________________________

'''
This is a ROS SPECIFIC node that runs upon recipt of a message FOR SUBSCRIBER ONLY
'''
def callback(data):
    #basically this method will READ the message we subscribed to, and send those values to the legs
    FLL=[data.pos[0], data.pos[1], data.pos[2]]
    FRL=[data.pos[3], data.pos[4], data.pos[5]]
    BLL=[data.pos[6], data.pos[7], data.pos[8]]
    BRL=[data.pos[9], data.pos[10], data.pos[11]]
    #NOTE: Add support for velocities writing later
    write(FLL, FRL, BLL, BRL)

'''
This is a ROS SPECIFIC node that publishes the data at a certain rate
'''
def subscriber():
    #initilize the node
    rospy.init_node("RIO_WRITER")
    #read the topic we're subscribing to and execute CALLBACK
    rospy.Subscriber("CMDS", ChipBodyControl, callback)

    #keeps node alive
    rospy.spin()

'''
not specific to ROS, this is the MAIN method, run this
'''
if __name__ == '__main__':
    subscriber()

