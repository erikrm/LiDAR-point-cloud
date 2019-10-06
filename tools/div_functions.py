import os
import socket
import sys
import numpy as np
import utm
import logging

#For file input
from tkinter import Tk
from tkinter.filedialog import askopenfilename, askdirectory
from tkinter import messagebox

def get_input_file_from_dialog(title, init_file_path, file_type, exit_if_not_exist=False):
    Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    file_name = askopenfilename(initialdir=init_file_path, title = title, filetypes = ((file_type + " files" , "*." + file_type),("all files" , "*.*"))) # show an "Open" dialog box and return the path to the selected file

    if not os.path.isfile(file_name):
        if exit_if_not_exist:
            sys.exit("file name: " + file_name + " not found, exiting")
        else:
            print('"{}" does not exist'.format(file_name), file=sys.stderr) 
            return -1
    
    return file_name


def socket_setup(udp_ip, udp_port):
    sock = socket.socket(socket.AF_INET, #Internet
                            socket.SOCK_DGRAM, socket.IPPROTO_UDP) #UDP
    sock.bind(('', udp_port))
    sock.connect((udp_ip,udp_port))

    #sock.setsockopt(socket.IPPROTO_IP, socket.IP_HDRINCL, 1)
    return sock

def seconds_to_time_str(seconds):
    seconds_since_midnight = seconds % 86400 #60sec *60min *24hours
    hours = int(seconds_since_midnight / 3600)
    minutes = int((seconds_since_midnight % (3600*hours))/60)
    seconds = int(seconds_since_midnight % (hours*3600+minutes*60))
    
    if seconds < 10:
        string = str(hours) + ":"  + str(minutes) + ":0" + str(seconds)
    else:
        string = str(hours) + ":"  + str(minutes) + ":" + str(seconds)
    return string


def process_ins_to_utm(ins_file):
    dt_ins_raw = [('gps_time','f8'),('longitude','f8'),('latitude','f8'),('z','f8'),('roll','f8'),('pitch','f8'),('yaw','f8')] #The idea is to convert from dt_ins_gps to a local frame and save it in dt_ins_local. First I will just simulate the values in dt_ins_local
    ins_raw_data = np.genfromtxt(ins_file, dtype=dt_ins_raw, delimiter='\t') #We are loosing the 9th decimal of the ins file
    
    dt_ins = [('gps_time','f8'),('x','f8'),('y','f8'),('z','f8'),('roll','f8'),('pitch','f8'),('yaw','f8')] #The idea is to convert from dt_ins_gps to a local frame and save it in dt_ins_local. First I will just simulate the values in dt_ins_local
    ins_data = np.zeros(ins_raw_data.size, dtype=dt_ins)

    logging.info('Loaded INS file: ' +str(ins_file))

    zone_temp = ''
    letter_temp = ''

    for i in range(ins_raw_data.size):
        ins_data['x'][i], ins_data['y'][i], zone, letter = utm.conversion.from_latlon(ins_raw_data['latitude'][i], ins_raw_data['longitude'][i])
        if zone_temp != zone:
            zone_temp = zone
        elif letter_temp != letter:
            letter_temp = letter

    logging.info('Latlon -> utm conversion finished, zone: ' + str(zone) + ' letter: ' + str(letter))

    ins_data['gps_time'] = ins_raw_data['gps_time'] % (3600 * 24) #Remove all full days 
    ins_data['z'] = ins_raw_data['z']
    ins_data['roll'] = ins_raw_data['roll']
    ins_data['pitch'] = ins_raw_data['pitch']
    ins_data['yaw'] = ins_raw_data['yaw']
    
    logging.info("Time span INS data: " + seconds_to_time_str(ins_data['gps_time'][0]) + " -> " + seconds_to_time_str(ins_data['gps_time'][-1]))

    return ins_data