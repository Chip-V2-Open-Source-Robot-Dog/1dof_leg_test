#!/usr/bin/env python

#imports for CSV logging and such
import csv
from datetime import datetime as dt
import os
import time
#import network tables
from networktables import NetworkTables as nt
#defaults class
from DEFAULTS import DEFAULTS

'''
SETUP CSV LOGGING
'''
#create the CSV file for the LOG
filename = "../../logs/log_"+str(dt.now())+".csv" 
script_dir = os.path.dirname(__file__)
full_path = os.path.join(script_dir, filename)
csv_file = open(full_path, mode='w')
#setup the CSV file
fieldnames = ["t", "VBUS",
            "m1_p", "m2_p", "m1_current", "m2_current", 
            "m1_v", "m2_v",
            "m1_tau", "m2_tau"]
writer = csv.writer(csv_file)
writer.writerow(fieldnames)
t0 = time.time()

'''
SOME INITIALIZATION CODE
'''
#start network tables server
nt.initialize(server=DEFAULTS.RIO_IP)
nt.setUpdateRate(0.01)
#initialize tables
sender=nt.getTable("rio_input")
readout=nt.getTable("rio_readout")

'''
SETUP MODES DICT
'''
MODES = {1: 'VOL', 2: 'CUR', 3: 'VEL', 4: 'POS'}
M1_DATA = [DEFAULTS.NT_DEFAULT_VAL]*4
M2_DATA = [DEFAULTS.NT_DEFAULT_VAL]*4
VBUS = DEFAULTS.NT_DEFAULT_VAL

def write(command):
    #data from M1
    sender.putNumber("m1_p", command[0])
    sender.putNumber("m1_pd", command[1])
    sender.putNumber("m1_tau", command[2])
    sender.putNumber("m1_mode", command[3])
    #data from M2
    sender.putNumber("m2_p", command[4])
    sender.putNumber("m2_pd", command[5])
    sender.putNumber("m2_tau", command[6])
    sender.putNumber("m2_mode", command[7])

def read(data):
    #data from M1
    M1_DATA = [readout.getNumber("m1_p", DEFAULTS.NT_DEFAULT_VAL), readout.getNumber("m1_v", DEFAULTS.NT_DEFAULT_VAL), readout.getNumber("m1_tau", DEFAULTS.NT_DEFAULT_VAL), readout.getNumber("m1_current", DEFAULTS.NT_DEFAULT_VAL)]
    #data from M2
    M2_DATA = [readout.getNumber("m2_p", DEFAULTS.NT_DEFAULT_VAL), readout.getNumber("m2_v", DEFAULTS.NT_DEFAULT_VAL), readout.getNumber("m2_tau", DEFAULTS.NT_DEFAULT_VAL), readout.getNumber("m2_current", DEFAULTS.NT_DEFAULT_VAL)]
    #VBUS
    VBUS = readout.getNumber("VBUS", DEFAULTS.NT_DEFAULT_VAL)
    
    #LOG THESE VALUES
    t = time.time()-t0
    DATA_VECTOR = [t]+[VBUS]+M1_DATA+M2_DATA
    writer.writerow(DATA_VECTOR)

#run the main code
if __name__=='__main__':
    pass









'''
//DO NETWORKING SETUP
networking = Networking.getInstance();
networking.addReadouts(
    //BUS VOLTAGE
    "VBUS",
    //GENERAL MOTOR PARAMS
    "m1_p", "m2_p", "m1_current", "m2_current", 
    //ADDING IN VELOCITIES FOR P-PDOT CONTROL
    "m1_v", "m2_v",
    //ADDING IN TORQUES
    "m1_tau", "m2_tau"
);
networking.addInputs(
    "m1_p",
    "m2_p",
    "m1_pd",
    "m2_pd",
    "m1_tau",
    "m2_tau",
    "m1_mode",
    "m2_mode"
);
'''

'''
COMMAND:
[m1_p, m1_pd, m1_tau, m1_mode, m2_p, m2_pd, m2_tau, m2_mode]

MODE --> control mode (VEL, POS, CUR, VOL)

RETURN DATA:
[m1_p, m1_v, m1_tau, m1_current...
'''