
#imports for CSV logging and such
import csv
from datetime import datetime as dt
import os
#import network tables
from networktables import NetworkTables as nt

'''
SETUP CSV LOGGING
'''
#create the CSV file for the LOG
filename = "../../logs/log_"+str(dt.now())+".csv" 
script_dir = os.path.dirname(__file__)
full_path = os.path.join(script_dir, filename)
csv_file = open(full_path, mode='w')
#setup the CSV file
fieldnames = ['time', 'body_pos', 'fll_pos', 'frl_pos', 'bll_pos', 'brl_pos', 'orientation', 'reset'] # UPDATE THIS LATER
writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
writer.writeheader()
t0 = 0.0

'''
SOME INITIALIZATION CODE
'''
#start network tables server
nt.initialize(server=DEFAULTS.RIO_IP)
#initialize tables
sender=nt.getTable("rio_input")
readout=nt.getTable("rio_readout")

def write(command):
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

'''