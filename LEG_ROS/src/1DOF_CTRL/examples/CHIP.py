#!/usr/bin/env python

'''
This node is both a publisher and a subscriber. It is the most
complicated node in the ROS system, it takes in joystick input,
the model, and messages from MAVROS and sends commands to the 
platform.
'''

import rospy
from std_msgs.msg import Float64, Float64MultiArray, Bool, Int16
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from CHIP_ROS.msg import ChipBodyControl #custom message definition for both pos + vel joint control
import math
from CONTROL_MODES import CONTROL_MODE
from DEFAULTS import DEFAULTS
from TRAJECTORY import TRAJECTORY
from LEG_MODEL import MODEL as legModel
from pyfirmata import Arduino, util
import csv
from datetime import datetime as dt
import os

'''
SETUP CSV LOGGING
'''
#create the CSV file for the LOG
filename = "../../logs/log_"+str(dt.now())+".csv" 
script_dir = os.path.dirname(__file__)
full_path = os.path.join(script_dir, filename)
csv_file = open(full_path, mode='w')
#setup the CSV file
fieldnames = ['time', 'body_pos', 'fll_pos', 'frl_pos', 'bll_pos', 'brl_pos', 'orientation', 'reset']
writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
writer.writeheader()
t0 = 0.0
#the reset flag is set to true when the operator pushes the reset button (helpful for debugging)

'''
SETUP THE SIMULATION
'''
#__________________________________
# ALL SIMULATION CODE HERE
#__________________________________
import pybullet as p
import time
import pybullet_data
from libIK import *

physicsClient = p.connect(p.DIRECT)#or p.GUI for graphical version
#physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

startOrientation = p.getQuaternionFromEuler([0,0,0])
urdfFlags = p.URDF_USE_SELF_COLLISION
#import the leg
leg = p.loadURDF("/home/chip-core/chip_v2_new/src/CHIP_ROS/src/scripts/chiplegleft.urdf", [0,0,0], startOrientation, flags=urdfFlags, useFixedBase=True)
robot = p.loadURDF("/home/chip-core/chip_v2_new/src/CHIP_ROS/src/scripts/chipv2.urdf", [0,0,0], startOrientation, flags=urdfFlags, useFixedBase=True)

'''Now setup the joint control'''
jointIds=[]
paramIds=[]
jointAngles = [0,0,0,0,0,0,0,0,0,0,0,0]

#add the joint control parameters here
activeJoint=0
for j in range (p.getNumJoints(robot)):
    info = p.getJointInfo(robot,j)
    #print(info)
    jointName = info[1]
    #print(jointName)
    jointType = info[2]
    if (jointType==p.JOINT_PRISMATIC or jointType==p.JOINT_REVOLUTE):
        jointIds.append(j)
        #paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"),-4,4,jointAngles[activeJoint]))
        p.resetJointState(robot, j, jointAngles[activeJoint])
        activeJoint+=1

'''
DEIFNE GLOBAL PLATFORM CONTROL MODE, DEFAULTS TO STAND_SIT
'''
MODE = CONTROL_MODE.STAND_SIT
trajRunner = TRAJECTORY()
POS = []
BODY_POS = [0.0, 0.0, 0.0]
SIM_COMMAND = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
ORI = [0.0, 0.0, 0.0]

'''
DEFIINE VARIABLES TO PUBLISH
'''
LAST_PUB = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #THIS IS THE DEFAULT "HOME" we should change this to the current foot pose

'''
creating publisher up here so on callback we can REPUBLISH
'''
#SET_PUB = rospy.Publisher('SETPOINT_UNSTABLE', Float64MultiArray, queue_size=0) #default queue size is 0
CMD_PUB = rospy.Publisher('CMDS', ChipBodyControl, queue_size=0) #default queue size is 0
MODE_PUB = rospy.Publisher('CONTROL_MODE', Int16, queue_size=0)

#____________________________________________________________________________________________________________________________________________________________________
# THESE METHODS ARE HELPER METHODS
#____________________________________________________________________________________________________________________________________________________________________

"""Bounds the value input between lower and upper, known limits of the system, think of them as virtual hardstops"""
def clip(value, lower, upper):
    if (value>=upper):
        return upper
    if (value<=lower):
        return lower
    return value

"""calcualte the commands to send"""
def calculateCMDS(thetas):
    return [thetas[0] * 100.0/(2*math.pi), thetas[1] * 100.0/(2*math.pi), thetas[2] * 100.0/(2*math.pi)]

"""
adds an xyz,rpy point to the trajectory
RETURNS COMMANDS IN SIM FRAME
"""
def addBodyIKPoint(pos, roll, pitch, yaw):
    #perform inverse kinematics
    COMMANDS_SIM = IK(p, leg, pos, pitch, roll, yaw)
    #manipulate command with drivers 1 and 2
    COMMANDS = driver_001(COMMANDS_SIM)
    COMMANDS = driver_002(COMMANDS)
    #add to the trajectory
    trajRunner.addWaypoint(COMMANDS)
    return COMMANDS_SIM

"""
directly publishes an xyz,rpy point to the platform
RETURNS COMMANDS IN SIM FRAME
"""
def pubBodyIKPoint(pos, roll, pitch, yaw):
    #perform inverse kinematics
    COMMANDS_SIM = IK(p, leg, pos, pitch, roll, yaw)
    #manipulate command with drivers 1 and 2
    COMMANDS = driver_001(COMMANDS_SIM)
    COMMANDS = driver_002(COMMANDS)
    #add to the trajectory
    publish_CMDS(COMMANDS)
    return COMMANDS_SIM

#____________________________________________________________________________________________________________________________________________________________________
# HERE'S THE ACTUAL ROS NODE METHODS AND ETC.
#____________________________________________________________________________________________________________________________________________________________________

#publishes the commands to the actuators to the NETWORK accounting for the zeroing-offsets. 
def publish_CMDS(CMDS):
    #add offsets and limit the outputs 
    ONE = clip(CMDS[0]+DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    TWO = clip(CMDS[1], -DEFAULTS.JL_L, DEFAULTS.JOINT_LIMIT)
    THREE = clip(CMDS[2]+DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    FOUR = clip(CMDS[3]+DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    FIVE = clip(CMDS[4], -DEFAULTS.JL_L, DEFAULTS.JOINT_LIMIT)
    SIX = clip(CMDS[5]+DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    SEVEN = clip(CMDS[6]+DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    EIGHT = clip(CMDS[7], -DEFAULTS.JL_L, DEFAULTS.JOINT_LIMIT)
    NINE = clip(CMDS[8]+DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    TEN = clip(CMDS[9]+DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    ELEVEN = clip(CMDS[10], -DEFAULTS.JL_L, DEFAULTS.JOINT_LIMIT)
    TWELVE = clip(CMDS[11]+DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET, -DEFAULTS.JOINT_LIMIT, DEFAULTS.JOINT_LIMIT)
    #populate and publish the vector
    CMDS_NEW = [ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE]
    #create the custom message
    msg = ChipBodyControl()
    msg.pos = CMDS_NEW
    msg.vel = []
    CMD_PUB.publish(msg)
    #update global variable
    global LAST_PUB
    LAST_PUB = CMDS

#handles getting the current actual platform RPY
def imu_callback(data):
    #now actually update the orientation variable
    global ORI
    #reset simulation body orientation
    ORI = data.data
    ori = p.getQuaternionFromEuler(ORI[:3])
    p.resetBasePositionAndOrientation(robot, [0.0,0.0,0.0], ori)
    #p.stepSimulation()

#handles getting the current actual actuator positions IN THE USER COORD FRAME/not RAW FRAME
def pos_callback(data):
    #convert coordinate frames
    ONE = data.pos[0]-DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET
    TWO = data.pos[1]
    THREE = data.pos[2]-DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET
    FOUR = data.pos[3]-DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET
    FIVE = data.pos[4]
    SIX = data.pos[5]-DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET
    SEVEN = data.pos[6]-DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET
    EIGHT = data.pos[7]
    NINE = data.pos[8]-DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET
    TEN = data.pos[9]-DEFAULTS.ZERO_OFFSET+DEFAULTS.TERRAIN_OFFSET
    ELEVEN = data.pos[10]
    TWELVE = data.pos[11]-DEFAULTS.ZERO_OFFSET-DEFAULTS.TERRAIN_OFFSET
    #populate the vector
    POS_NEW = [ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, TEN, ELEVEN, TWELVE]
    #set the current foot_pos
    global POS
    POS = POS_NEW
    #update simulation
    global SIM_COMMAND
    SIM_COMMAND = driver_003(POS_NEW)
    for i in range(len(jointIds)):
        p.resetJointState(robot, jointIds[i], SIM_COMMAND[i])
    #p.stepSimulation()

    #NOW DEAL WITH THE TRAJAECTORY RUNNER
    if(not(MODE==CONTROL_MODE.STAND_SIT)):
        publish_CMDS(trajRunner.tick(POS))

#handles most of the main processes of the node
def joy_callback(data):
    #GET THE VALUES FROM THE JOYSTICK
    #1.0 is PRESSED
    #AXIS are all on a scale of -1.0 to 1.0
    RX = data.axes[2] #INVERTED X, THIS CONTROL WHEELS
    RY = data.axes[3] #THIS CONTROL WHEELS
    LX = data.axes[0] #INVERTED X
    LY = data.axes[1]
    '''these axis will send change parameters'''
    RT = data.axes[4]
    LT = data.axes[5]
    '''these buttons will send specific commands'''
    POVX = data.axes[6]
    POVY = data.axes[7]
    LB = data.buttons[4]
    RB = data.buttons[5]
    '''these buttons will switch the platform into diff modes'''
    A = data.buttons[0]
    B = data.buttons[1]
    X = data.buttons[2]
    Y = data.buttons[3]

    if(A==1.0):
        global MODE
        MODE = CONTROL_MODE.WALK
    if(B==1.0):
        global MODE
        MODE = CONTROL_MODE.STAND_SIT
    if(X==1.0):
        global MODE
        MODE = CONTROL_MODE.DANCE
    if(Y==1.0):
        global MODE
        MODE = CONTROL_MODE.TEST

    #only if the mode is stand_sit mode
    if(MODE==CONTROL_MODE.STAND_SIT):
        #just as a stand-sit test
        pass
        '''NOTE: RESET THE BODY POSITION TO THE HOME AND THEN SET THE RESET FLAG TO TRUE + WRITE'''
    
    if(MODE==CONTROL_MODE.WALK):
        pass
        #things we have to do in here
        #(1) implement the stand/sit
        #(2) implement the x and y shifts for ALL 
        #(3) implement the roll-pitch-yaw for ALL z heights
        #implement some sort of detection of which foot is off the ground - implement a switch where depending we can control different things

        #can we write a logger where we just press a button if tthe robot is falling over and it logs that foot pos and that it fell or didnt --> big lookup table???
        #ORRR JUST ADD GENERAL CSV LOGGING!!!

    if(MODE==CONTROL_MODE.DANCE):
        pass
        #implement txt or csv files that contain the dance moves (have a trajectory libary)

    if(MODE==CONTROL_MODE.TEST):
        #let's set the  sticks
        lift = RY*0.02 #use joystick to raise and lower the platform, tune till it feels right
        xpos = LY*0.005
        ypos = round(RX*0.15,3)
        yaw = round(LX*0.4, 2)

        #get current positions
        zc = BODY_POS[2]
        xc = BODY_POS[0]

        #add desired positions
        #NOTE: WE ADDED SOME PROTECTION HERE, ONLY WORKS IF YOU HOLD THE RT TRIGGER IN X AND Z
        x = xc
        z = zc
        if RT <= 0.5:
            x = round(clip(xc+xpos, -0.15, 0.15), 3)
            z = round(clip(zc+lift, 0.0, 0.842), 2) #lowest z-height
        #SAFEGUARD
        if zc <= 0.1: x = 0.0
        pos = [x, ypos, z]

        #publish to platform
        global BODY_POS
        pubBodyIKPoint(pos, 0.0, 0.0, yaw)
        BODY_POS = pos

    #get the foot positions
    local_fps = footPos(p, robot, ORI[0], ORI[1], ORI[2]) #FL FR BL BR
    z_hat = height_filter(p, robot)

    #wheels are always active in all control modes
    #actuate_wheels(RX, RY) #NOTE: Removed the module for now
    #actually publish the setpoint
    MODE_PUB.publish(Int16(data=MODE))

    t = rospy.Time.now()-t0
    vector = {'time': t, 'body_pos': BODY_POS, 'fll_pos': local_fps[0], 'frl_pos': local_fps[1], 'bll_pos': local_fps[2], 'brl_pos': local_fps[3], 'orientation': ORI, 'reset': 0}
    if(MODE==CONTROL_MODE.STAND_SIT):
        vector['reset'] = 1 #change the RESET flag
    writer.writerow(vector)
    #print(vector)


'''
This does the actual subscribing and calling the callback
'''
def publisher_subscriber(): 
    #initilize the node
    rospy.init_node("CHIP_MASTER")
    #read the topic we're subscribing to and execute CALLBACK
    rospy.Subscriber("POS", ChipBodyControl, pos_callback)
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.Subscriber("ORIENTATION", Float64MultiArray, imu_callback)

    #set the global initial time
    global t0
    t0 = rospy.Time.now()

    #keeps node alive
    rospy.spin()


'''
not specific to ROS, this is the MAIN method, run this
'''
if __name__ == '__main__':
    try:
        publisher_subscriber()
    except rospy.ROSInterruptException:
        pass
