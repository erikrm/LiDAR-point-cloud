# How to deal with pcap:
# https://vnetman.github.io/pcap/python/pyshark/scapy/libpcap/2018/10/25/analyzing-packet-captures-with-python-part-1.html

import os
import sys
from scapy.utils import RawPcapReader
from time import perf_counter_ns as time_ns
import numpy as np
import tools.udp_unpack as udp_unpack
#import tools.udp_unpack_position as udp_unpack_position
import laspy
import pynmea2
import datetime
import utm 

#For fileinput
from tkinter import Tk
from tkinter.filedialog import askopenfilename

from tools.lidar_values_and_settings import lidar_info
from tools.lidar_values_and_settings import udp_info
from tools.lidar_values_and_settings import dt_measurement
from tools.lidar_values_and_settings import dt_position
from tools.lidar_values_and_settings import angle_offset_per_laser as offset
from tools import write_to_las





def process_pcap(outfile_location, file_name, ins_file, num_frames, packet_devisor): 
    """Loads the file and returns the data sorted into frames where on frame is a the field of view. Zero indexed, it has to work through all the data from start to last_fram_nr whatever you write in first_frame_nr"""
    print('Opening {}...'.format(file_name))

    frame_nr = 0
    data_packet_nr = 0
    last_azimuth_block = -1    

    dt_ins_raw = [('gps_time','f8'),('longitude','f8'),('latitude','f8'),('z','f8'),('roll','f8'),('pitch','f8'),('yaw','f8')] #The idea is to convert from dt_ins_gps to a local frame and save it in dt_ins_local. First I will just simulate the values in dt_ins_local
    ins_raw_data = np.genfromtxt(ins_file, dtype=dt_ins_raw, delimiter='\t') #We are loosing the 9th decimal of the ins file
    
    #np.savetxt('ins_raw_data.csv',ins_raw_data)
    #print("dt_ins_raw", dt_ins_raw)
    dt_ins = [('gps_time','f8'),('x','f8'),('y','f8'),('z','f8'),('roll','f8'),('pitch','f8'),('yaw','f8')] #The idea is to convert from dt_ins_gps to a local frame and save it in dt_ins_local. First I will just simulate the values in dt_ins_local
    ins_data = np.zeros(ins_raw_data.size, dtype=dt_ins)

    
    print('INS file', ins_file, 'loaded')

    zone_temp = ''
    letter_temp = ''

    for i in range(ins_raw_data.size):
        ins_data['x'][i], ins_data['y'][i], zone, letter = utm.conversion.from_latlon(ins_raw_data['latitude'][i], ins_raw_data['longitude'][i])
        if zone_temp != zone:
            zone_temp = zone
            print("Zone set to:", zone)
        elif letter_temp != letter:
            letter_temp = letter
            print("Letter set to:", letter)

    ins_data['gps_time'] = ins_raw_data['gps_time'] % (3600 * 24) #Remove all full days 
    ins_data['z'] = ins_raw_data['z']
    ins_data['roll'] = ins_raw_data['roll']
    ins_data['pitch'] = ins_raw_data['pitch']
    ins_data['yaw'] = ins_raw_data['yaw']
    
    #print("dt_ins", dt_ins)
    #np.savetxt('ins_data.csv',ins_data)
    
    print('Latlon -> utm conversion finished, zone:', zone, 'letter:', letter)
    
    flight_date = datetime.date.fromtimestamp(0) #Just to initilize

    data_packet_length = lidar_info['num_firings_per_packet']
    num_data_packets = lidar_info['expected_number_of_packets_per_rotation']
    data_length_fov = data_packet_length * (num_data_packets + 1)

    data_position_length = lidar_info['position_packets_per_second']
    data = np.zeros(data_length_fov,dtype=dt_measurement)
    #data_position = np.zeros(data_position_length,dtype=dt_position)
    data_position_nr = 0

    last_nmea_message = ''
    last_gps_time_toh = 0
    
    current_gps_time_toh = -1
    current_gps_time = -1

    #write_to_las.write_data_into_files()
    
    index = -1
    
    print("First ins_time", ins_data['gps_time'][0], "last ins_time", ins_data['gps_time'][-1])
    printed_first_lidar_time = False
    
    for (pkt_data, pkt_metadata,) in RawPcapReader(file_name): #There should be fdesc and magic values, but I have not found out what they should be. It seems to be stable without them
        index += 1
        
        flight_date = datetime.date.fromtimestamp(pkt_metadata.sec) #We need the date to get the gps time
        
        if pkt_metadata.wirelen == 1206 + udp_info['header_size'] and index % packet_devisor == 0:
            if not printed_first_lidar_time:
                print("First lidar time", current_gps_time, "first toh", current_gps_time_toh)
                printed_first_lidar_time = True

            if current_gps_time >= ins_data['gps_time'][0] and current_gps_time <= ins_data['gps_time'][-1]:
                
                data_packet, new_azimuth_block = udp_unpack.unpack_measurement(pkt_data[udp_info['header_size']:],frame_nr,last_azimuth_block,data_packet_nr) #The socket function removes the header, while RawPcapReader does not 
                
                #if not_set:
                #    txyzrpy_delta_txyzrpy[0] = data_packet['timestamp'][0] - 1000 #Set one time, otherwise you have to update the constants in txyzrpy too
                #    not_set = False

                """xyz is calculated with the geometry given in the user manual in section 9.2"""
                
                data[data_packet_nr*data_packet_length : (data_packet_nr+1)*data_packet_length][:] = data_packet
                data_packet_nr += 1

                azimuth_gap_block = new_azimuth_block - last_azimuth_block
                last_azimuth_block = new_azimuth_block
                if azimuth_gap_block < 0:
                    azimuth_gap_block += 360

                if azimuth_gap_block > 360 - lidar_info['fov_size']*2: #This solution saves the first batch of each frame at the old frame right? That is not perfect
                    #print(current_gps_time_toh)
                    if frame_nr < 10: #Easier sorting, works up to 9999
                        str_frame_nr = '000' + str(frame_nr)
                    elif frame_nr < 100:
                        str_frame_nr = '00' + str(frame_nr)
                    elif frame_nr < 1000:
                        str_frame_nr = '0' + str(frame_nr)
                    else: str_frame_nr = str(frame_nr)

                    file_name = outfile_location + 'frame_' + str_frame_nr + '.las'
                    data_trimmed = data[np.nonzero(data)]
                    data.fill(0)
                    #data_trimmed = data_trimmed[data_trimmed['laser_id'] == 5]

                    udp_unpack.interpolate_ins_2(data_trimmed, ins_data, current_gps_time_toh) #Will this give 1 sec wrong every hour change on one batch?

                    udp_unpack.transform_to_map(data_trimmed)
                    
                    #if data_trimmed['xyz'][0,2] < 2000:
                    #    print(data_trimmed)

                    #mask = np.isin(data['laser_id'], [0, 5, 10])
                    
                    with open('timestamp.csv','a') as fd:
                        np.savetxt(fd,np.divide(data_trimmed['timestamp'], pow(10,9)) + current_gps_time_toh + (flight_date.weekday()+1)*24*3600,delimiter='\t')

                    """
                    with open('test.csv','a') as fd:
                        array = np.append(data_trimmed['xyz'], data_trimmed['xyz_lu'], axis=1)
                        array2 = np.append(data_trimmed['timestamp'], np.transpose(array), axis=1)
                        np.savetxt(fd,array2,delimiter=',')
                    """
                    
                    write_to_las.write_one_frame_to_las(file_name,None,data_trimmed)
                    
                    data_packet_nr = 0
                    data_position_nr = 0

                    frame_nr += 1
                    print('Processed frames: ' + str(frame_nr) + ' lidar time: ' + str(current_gps_time), end='\r')
                    if frame_nr == num_frames:
                        print('Processed frames: ' + str(frame_nr) + ' lidar time: ' + str(current_gps_time), end='\n')
                        break
                    
            

        if pkt_metadata.wirelen == 554:
            data_position = udp_unpack.unpack_position(pkt_data[udp_info['header_size']:])
            
            nmea_str = str(np.squeeze(data_position['nmea']))
            if 'GPGGA' in nmea_str:
                last_nmea_message = pynmea2.parse(nmea_str)
                
                current_gps_time_toh =  last_nmea_message.timestamp.hour*3600 #((flight_date.weekday()+1)*24 +

                #current_gps_time_toh += 1000

                current_gps_time = current_gps_time_toh + last_nmea_message.timestamp.minute*60 + last_nmea_message.timestamp.second
            #last_gps_time_toh = last_nmea_message.timestamp
            #print(last_nmea_message.timestamp)
            data_position_nr += 1
        



if __name__ == '__main__':       
    location_frame_files = "./files_from_pcap/"
    packet_devisor = 1 #Will only process every n packets
    num_frames = 20 #If set to negative value it will finish all
    outfile_directory = './collected_las_files_pcap/'
    num_frames_per_las_file = 200 * packet_devisor # Possible to divide into different batches to limit data stored in memory
    file_name_input_default = 'wireshark_flight_1'
    file_input_folder = './flight_scans/'

    ins_file_default = './flight_ins/export_flight01_20.08.19_all.txt'

    file_name_input = input("Write file name to read from (the flight must be in the " + file_input_folder + " folder)\n")
    if not file_name_input:
        file_name_input = file_name_input_default
    
    file_name_input_full_path = file_input_folder + file_name_input

    file_name = "./" + file_name_input_full_path + ".pcap"
    if not os.path.isfile(file_name):
        print('"{}" does not exist'.format(file_name), file=sys.stderr)
        sys.exit(-1)
    
    outfile_file_name = input("Write file name to read to\n")
    if not outfile_file_name:
        outfile_file_name = file_name_input
    

    time_start = time_ns()
    process_pcap(location_frame_files, file_name, ins_file_default, num_frames, packet_devisor)
    print("Execution time from pcap:",(time_ns() - time_start)/pow(10,9))
    
    #sys.exit(0)
    input("Continue and collect the files?")
    
    batch_num = 0
    
    time_start = time_ns()
    while batch_num*num_frames_per_las_file <= num_frames or num_frames < 0: 
        time_top = time_ns()
        data, offset = write_to_las.load_las_files_in_directory_with_offset(location_frame_files, num_frames_per_las_file, delete = True)
        if isinstance(data, int):
            if not data:
                print("Finished all the files")
                sys.exit(0)

        write_to_las.write_data_into_files_based_on_user_data_with_offset(outfile_directory + outfile_file_name + '_batch_' + str(batch_num), None, data, offset)    
        # still_contain_las_files = write_to_las.delete_las_files_in_directory(location_frame_files,num_frames_per_las_file)
        batch_num += 1
        print("Total execution time collecting:",(time_ns() - time_start)/pow(10,9),'Execution time this batch:',(time_ns() - time_top)/pow(10,9),'Batch num:', batch_num)

    sys.exit(0) 

    #Not important: 
    #animate it: https://stackoverflow.com/questions/41602588/matplotlib-3d-scatter-animations 
    #TODO: animate it with intensity change: https://pythonmatplotlibtips.blogspot.com/2018/11/animation-3d-surface-plot-funcanimation-matplotlib.html 