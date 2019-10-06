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
import logging 

#For file input
from tkinter import Tk
from tkinter.filedialog import askopenfilename, askdirectory
from tkinter import messagebox

from tools.lidar_values_and_settings import lidar_info
from tools.lidar_values_and_settings import udp_info
from tools.lidar_values_and_settings import dt_measurement
from tools.lidar_values_and_settings import dt_position
from tools.lidar_values_and_settings import angle_offset_per_laser as offset
from tools.lidar_values_and_settings import sheet_lidar_settings
from tools.lidar_values_and_settings import excel_setting_file_name

from tools import write_to_las
from tools.div_functions import seconds_to_time_str
from tools.div_functions import process_ins_to_utm
from tools.div_functions import get_input_file_from_dialog

def process_pcap(out_file_location, file_name, ins_data, num_frames, packet_devisor): 
    """Loads the file and returns the data sorted into frames where on frame is a the field of view. Zero indexed, it has to work through all the data from start to last_fram_nr whatever you write in first_frame_nr"""
    

    frame_nr = 0
    data_packet_nr = 0
    last_azimuth_block = -1    

    
    #flight_date = datetime.date.fromtimestamp(0) #Just to initialize

    data_packet_length = lidar_info['num_firings_per_packet']
    num_data_packets = lidar_info['expected_number_of_packets_per_rotation']
    data_length_fov = data_packet_length * (num_data_packets + 1)

    #data_position_length = lidar_info['position_packets_per_second']
    data = np.zeros(data_length_fov,dtype=dt_measurement)
    #data_position = np.zeros(data_position_length,dtype=dt_position)
    data_position_nr = 0

    last_nmea_message = ''
    #last_gps_time_toh = 0
    
    current_gps_time_toh = -1
    current_gps_time = -1

    #write_to_las.write_data_into_files()
    
    index = -1
    
    logging.info("First ins_time: " + seconds_to_time_str(ins_data['gps_time'][0]) + " last ins_time: " + seconds_to_time_str(ins_data['gps_time'][-1]))
    printed_first_lidar_time = False
    
    logging.info("Opening " + file_name)
    for (pkt_data, pkt_metadata,) in RawPcapReader(file_name): #There should be fdesc and magic values to define the pcap file, but I have not found out what they should be. It seems to be stable without them
        index += 1

        #flight_date = datetime.date.fromtimestamp(pkt_metadata.sec) #We need the date to get the gps time
        
        if pkt_metadata.wirelen == 1206 + udp_info['header_size'] and index % packet_devisor == 0:
            if not printed_first_lidar_time:
                logging.info("First lidar time: " + seconds_to_time_str(current_gps_time))
                first_lidar_time = current_gps_time
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
                    if frame_nr < 10: #Easier sorting, works up to 9999
                        str_frame_nr = '000' + str(frame_nr)
                    elif frame_nr < 100:
                        str_frame_nr = '00' + str(frame_nr)
                    elif frame_nr < 1000:
                        str_frame_nr = '0' + str(frame_nr)
                    else: str_frame_nr = str(frame_nr)

                    file_name = out_file_location + 'frame_' + str_frame_nr + '.las'
                    data_trimmed = data[np.nonzero(data)]
                    data.fill(0)
                    #data_trimmed = data_trimmed[data_trimmed['laser_id'] == 5]

                    udp_unpack.interpolate_ins_2(data_trimmed, ins_data, current_gps_time_toh) #Will this give 1 sec wrong every hour change on one batch
                    udp_unpack.transform_to_map(data_trimmed)
                    
                    write_to_las.write_one_frame_to_las(file_name,None,data_trimmed)
                    
                    data_packet_nr = 0
                    data_position_nr = 0

                    frame_nr += 1
                    print('Processed frames: ' + str(frame_nr) + ' lidar time: ' + seconds_to_time_str(current_gps_time), end='\r')
                    if frame_nr == num_frames:
                        print('Processed frames: ' + str(frame_nr) + ' last lidar time: ' + seconds_to_time_str(current_gps_time), end='\n')

                        logging.info('Time span pcap data: ' + seconds_to_time_str(first_lidar_time) + ' -> ' + seconds_to_time_str(current_gps_time))

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


def get_the_four_paths():
    #Find input file locations and output folder:
    if os.path.isdir(sheet_lidar_settings['B49'].value):
        temp_location_frame_files = sheet_lidar_settings['B49'].value
    else: 
        temp_location_frame_files = "./files_from_pcap/"

    if os.path.isfile(sheet_lidar_settings['B47'].value) :
        file_path_pcap = sheet_lidar_settings['B47'].value
    else:   
        file_path_pcap = get_input_file_from_dialog("Choose PCAP file", "./flight_scans", "pcap", True)
        if file_path_pcap == -1:
            sys.exit("File path pcap doesn't exist")
    
    if os.path.isfile(sheet_lidar_settings['B48'].value) :
        file_path_ins = sheet_lidar_settings['B48'].value
    else:   
        file_path_ins = get_input_file_from_dialog("Choose INS file", "./flight_ins", "txt", True)

    if os.path.isdir(sheet_lidar_settings['B56'].value) :
        out_file_directory = sheet_lidar_settings['B56'].value
    else:   
        out_file_directory = askdirectory(initialdir="./processed_las", title="Choose folder for the output las-files")

    return temp_location_frame_files, file_path_pcap, file_path_ins, out_file_directory


if __name__ == '__main__':     
    if sheet_lidar_settings != -1:
        
        packet_devisor = sheet_lidar_settings['B51'].value #Will only process every n packets
        num_frames = sheet_lidar_settings['B52'].value #If set to negative value it will finish all
        num_frames_per_las_file = sheet_lidar_settings['B55'].value # Possible to divide into different batches to limit data stored in memory
        
        temp_location_frame_files, file_path_pcap, file_path_ins, out_file_directory = get_the_four_paths()

    else: #Default values:
        temp_location_frame_files = "./files_from_pcap/"
        packet_devisor = 1 #Will only process every n packets
        num_frames = 200 #If set to negative value it will finish all
        num_frames_per_las_file = 200 * packet_devisor # Possible to divide into different batches to limit data stored in memory
        
        #Find input file locations and output folder:
        file_path_pcap = get_input_file_from_dialog("Choose PCAP file", "./flight_scans", "pcap", True)
        file_path_ins = get_input_file_from_dialog("Choose INS file", "./flight_ins", "txt", True)
        out_file_directory = askdirectory(initialdir="./processed_las", title="Choose folder for the output las-files")

    #Using a log file to save info about the processing
    logging.basicConfig(filename=out_file_directory + '/log.txt',
                        filemode='a',
                        format='%(asctime)s,%(msecs)d %(name)s %(levelname)s %(message)s',
                        datefmt='%H:%M:%S',
                        level=logging.DEBUG)

    time_start = time_ns()
    
    logging.info("Processing started, " + str(datetime.datetime.now()))
    logging.info("Settings file: " + str(excel_setting_file_name))
    logging.info("Parameters: packet_devisor=" + str(packet_devisor) + " num_frames=" + str(num_frames) + " num_frames_per_las_file=" + str(num_frames_per_las_file))
    logging.info("File paths: pcap=" + str(file_path_pcap) + " ins=" + str(file_path_ins) + " out_file_directory=" + str(out_file_directory))

    ins_data = process_ins_to_utm(file_path_ins)

    process_pcap(temp_location_frame_files, file_path_pcap, ins_data, num_frames, packet_devisor)
    logging.info("Execution time from pcap: " + str((time_ns() - time_start)/pow(10,9)))
    
    batch_num = 0
    
    time_start = time_ns()
    while batch_num*num_frames_per_las_file <= num_frames or num_frames < 0: 
        time_top = time_ns()
        data, offset, loaded_frame_files = write_to_las.load_las_files_in_directory_with_offset(temp_location_frame_files, num_frames_per_las_file)
        if isinstance(data, int):
            logging.info("Finished all the files")
            break

        if num_frames_per_las_file >= num_frames:
            out_file_name = ''
        else: out_file_name = 'batch_' + str(batch_num) + '_'

        write_to_las.write_data_into_files_based_on_user_data_with_offset(out_file_directory + "/" + out_file_name, None, data, offset)    

        for frame_file_name in loaded_frame_files:
            os.remove(frame_file_name)
        
        logging.info("Batch: " + str(batch_num) + " of frame files deleted")

        # still_contain_las_files = write_to_las.delete_las_files_in_directory(temp_location_frame_files,num_frames_per_las_file)
        
        logging.info("Total execution time collecting: " + str((time_ns() - time_start)/pow(10,9)) + ' Execution time this batch: ' + str((time_ns() - time_top)/pow(10,9)) + ' Batch num: ' + str(batch_num))
        batch_num += 1
        


    sys.exit(0) 
