""" 
Library for unpacking LiDAR data packets, calculating xyz, precise azimuth and precise timestamp for each measurement
This code is loosely based on: https://github.com/ShuiXinYun/Multi_Sensor/blob/master/Lidar/velodyne/Velodyne-VLP-32C_driver.py 
"""

#Base
import math
import numpy as np
import utm 

from tools.lidar_values_and_settings import lidar_info
from tools.lidar_values_and_settings import angle_offset_per_laser as offset
from tools.lidar_values_and_settings import dt_measurement
from tools.lidar_values_and_settings import dt_position
from tools.lidar_values_and_settings import position_info_distribution_ints
from tools.lidar_values_and_settings import lidar_lever_arm
from tools.lidar_values_and_settings import gps_leap_seconds

def unpack_measurement(recv_Data,frame_nr,last_azimuth_block_of_previous_data_packet,data_packet_nr):
    """Unpacks the recieved measurement data and returns it as a structured numpy array of size (384,13)""" 

    num_measurements = lidar_info['num_firings_per_packet']
    num_bytes_sequence = lidar_info['num_fire_sequence_bytes']
    num_data_blocks = lidar_info['fire_sequences_per_packet']
    num_lasers = lidar_info['num_lasers']
    
    #Variables
    num_returns = 1
    azimuth_gap = []

    data_packet = np.zeros(num_measurements,dtype=dt_measurement)

    #The different data types that comes in the udp_packet
    dt_azimuth = np.dtype('u2')
    dt_azimuth = dt_azimuth.newbyteorder('<') #little endian

    dt_timestamp = np.dtype('u4')
    dt_timestamp = dt_timestamp.newbyteorder('<') #little endian
    
    dt_distance_reflectivity = np.dtype([('distance','u2'),('reflectivity','u1')])
    dt_distance_reflectivity = dt_distance_reflectivity.newbyteorder('<') #little endian


    last_azimuth = -1
    for i in range(num_data_blocks):
        current_azimuth = np.frombuffer(recv_Data, dtype=dt_azimuth,count=1,offset=(i*num_bytes_sequence + 2)) /100 #The LiDAR gives the value in percentage of degree
        data_packet['azimuth_centered'][i*num_lasers] = current_azimuth

        if last_azimuth == -1:
            data_packet['return_num'][i*num_lasers:(i+1)*num_lasers] = 1 
        elif current_azimuth == last_azimuth:
            data_packet['return_num'][i*num_lasers:(i+1)*num_lasers] = 2
            if num_returns == 1:
                num_returns = 2
        else: 
            data_packet['return_num'][i*num_lasers:(i+1)*num_lasers] = 1    

            azimuth_gap_temp = current_azimuth - last_azimuth
            if azimuth_gap_temp < 0:
                azimuth_gap_temp += 360
            if azimuth_gap_temp < lidar_info['max_azimuth_gap']*2: #Don't want any strange azimuth values
                azimuth_gap.append(azimuth_gap_temp)
        last_azimuth = current_azimuth
        
        data_packet[['distance','reflectivity']][i*num_lasers:(i+1)*num_lasers] = np.frombuffer(recv_Data, dtype=dt_distance_reflectivity,count=num_lasers,offset=(4+i*num_bytes_sequence))
        data_packet['laser_id'][i*num_lasers:(i+1)*num_lasers] = np.arange(num_lasers)
        data_packet['data_block'][i*num_lasers:(i+1)*num_lasers] = i


    data_packet['num_returns'] = num_returns #The whole data_packet has the same num_return, really the whole session should have it
    data_packet['distance'] *= 4 #Distance has a granularity of 4mm, i.e. each bit represents four mm
    data_packet['frame_nr'] = frame_nr
    data_packet['data_packet_nr'] = data_packet_nr

    """Azimuth gap, this value is used to calculate the position of the lasers at the moment of fire by calibrating for the movement of the laser"""
    azimuth_gap_mean = np.mean(azimuth_gap)

    """azimuth_centered gives the center of the laser array at the moment of fire"""
    data_packet['azimuth_centered'] = data_packet['azimuth_centered'][ data_packet['data_block']*num_lasers ] + np.floor(data_packet['laser_id']/2) * azimuth_gap_mean * lidar_info['single_fire_length']/lidar_info['fire_sequence_length'] #The lasers fires in pairs

    """azimuth gives the exact direction of the laser at the moment of fire"""
    data_packet['azimuth'] = data_packet['azimuth_centered'] + offset['azimuth'][data_packet['laser_id']]

    """The timestamps comes in microseconds, converted to nanoseconds and is calculated from the timing of the firing sequence, the timestamp recieved with any packet gives the first firing of the first block, or for dual mode the first firing of the first two blocks"""
    data_packet['timestamp'][0] = np.frombuffer(recv_Data, dtype=dt_timestamp,count=1,offset=(num_data_blocks*num_bytes_sequence))
    data_packet['timestamp'][0] *= 1000
    if num_returns == 2: 
        data_packet['timestamp'][num_lasers] = data_packet['timestamp'][0] #if dual mode, both returns will have the same timestamp

    data_packet['timestamp'] = data_packet['timestamp'][0] + np.floor(data_packet['data_block']/num_returns)*lidar_info['fire_sequence_length'] + np.floor(data_packet['laser_id']/2)*lidar_info['single_fire_length']
    
    #print(data_packet[::64])
    return data_packet,last_azimuth

nmea_sentence_address = [0xCE,128]

udp_header = 42


def position_get_info_integers(recv_data, position_packet):
    for description, address_offset in position_info_distribution_ints.items(): 
        #address_offset[0] += udp_header
        data_snippet = recv_data[address_offset[0]:address_offset[0] + address_offset[1]]
        value = 0
        for i in range(address_offset[1]):
            value += int(data_snippet[i])*pow(256,i)
        position_packet[description] = value
    return position_packet


def unpack_position(recv_data):

    position_packet = np.zeros(1,dtype=dt_position)

    position_get_info_integers(recv_data, position_packet)
    position_packet['timestamp'] *= 1000 #Convert to ns
    nmea_sentence = recv_data[nmea_sentence_address[0]:nmea_sentence_address[0] + nmea_sentence_address[1]]
    nmea_sentence_trimmed ='no nmea message' #At least no properly structured nmea message
    for i in range(len(nmea_sentence)-1):
        if nmea_sentence[i:i+2] == b'\r\n':
            nmea_sentence_trimmed = nmea_sentence[:i]
            break
    position_packet['nmea'] = nmea_sentence_trimmed


    return position_packet


def rotation_matrix(gamma,beta,alpha):
    """returnes a rotation matrix for use on a xyz-matrix, rotates yaw, then pitch, then roll, 
    roll: rotates gamma degrees counterclockwise around the x-axis,
    pitch: rotates beta degrees counterclocwise around the y-axis,
    yaw (heading): rotates alpha degrees counterclockwise around the z-axis
    src: http://planning.cs.uiuc.edu/node102.html
    """

    alpha = np.deg2rad(alpha)
    beta = np.deg2rad(beta)
    gamma = np.deg2rad(gamma)

    R = np.array([[np.cos(alpha)*np.cos(beta), np.cos(alpha)*np.sin(beta)*np.sin(gamma) - np.sin(alpha)*np.cos(gamma), np.cos(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(alpha)*np.sin(gamma)], 
                [np.sin(alpha)*np.cos(beta), np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.cos(alpha)*np.sin(gamma)], 
                [-np.sin(beta), np.cos(beta)*np.sin(gamma), np.cos(beta)*np.cos(gamma)]])

    """Old rotation matrix:
     R = np.array([[np.cos(alpha)*np.cos(beta), np.cos(alpha)*np.sin(alpha)*np.sin(gamma) - np.sin(alpha)*np.cos(gamma), np.cos(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(alpha)*np.sin(gamma)], 
                [np.sin(alpha)*np.cos(beta), np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.cos(alpha)*np.sin(gamma)], 
                [-np.sin(beta), np.cos(beta)*np.sin(gamma), np.cos(beta)*np.cos(gamma)]])
    """
    return R

def rotation_matrix_array(rpy):
    rpy = np.deg2rad(rpy)

    gamma = rpy[:,0]
    beta = rpy[:,1]
    alpha = rpy[:,2]

    R = np.zeros((gamma.size,3,3),dtype='f8')
    R[:] = np.array([
            np.cos(alpha)*np.cos(beta), np.cos(alpha)*np.sin(beta)*np.sin(gamma) - np.sin(alpha)*np.cos(gamma), np.cos(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(alpha)*np.sin(gamma), 
            np.sin(alpha)*np.cos(beta), np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.cos(alpha)*np.sin(gamma), 
            -np.sin(beta), np.cos(beta)*np.sin(gamma), np.cos(beta)*np.cos(gamma)
            ]).flatten(order='F').reshape(gamma.size,3,3,order='C') #Transform from Fortran-style order to C-style order with one rotation matrix for each set of angles

    return R

def interpolate_ins(measurement_data, ins_data, current_gps_time_toh):

    gps_times = measurement_data['timestamp'] + current_gps_time_toh*pow(10,9) + gps_leap_seconds*pow(10,9)
    
    if gps_times[0] < ins_data['gps_time'][0]*pow(10,9) or gps_times[-1] > ins_data['gps_time'][-1]*pow(10,9):
        print("Gps time out of bounds: " + str(gps_times[0]) + " < " + str(ins_data['gps_time'][-1]) + " or " + str(gps_times[-1]) + " > " + str(ins_data['gps_time'][-1]))

    measurement_data['ins_xyz'][:,0] = np.interp(gps_times, ins_data['gps_time']*pow(10,9), ins_data['x']*1000)
    measurement_data['ins_xyz'][:,1] = np.interp(gps_times, ins_data['gps_time']*pow(10,9), ins_data['y']*1000)
    measurement_data['ins_xyz'][:,2] = np.interp(gps_times, ins_data['gps_time']*pow(10,9), ins_data['z']*1000)
    measurement_data['ins_rpy'][:,0] = np.interp(gps_times, ins_data['gps_time']*pow(10,9), 360 - ins_data['roll'])
    measurement_data['ins_rpy'][:,1] = np.interp(gps_times, ins_data['gps_time']*pow(10,9), 360 - ins_data['pitch'])
    measurement_data['ins_rpy'][:,2] = np.interp(gps_times, ins_data['gps_time']*pow(10,9), 360 - ins_data['yaw'])


def calculate_xyz_lu(measurement_data): #lu : lidar unit
    return np.transpose(np.array([
        measurement_data['distance'] * np.cos(np.deg2rad(offset['pitch'][measurement_data['laser_id']])) * np.sin(np.deg2rad(measurement_data['azimuth'])),
        measurement_data['distance'] * np.cos(np.deg2rad(offset['pitch'][measurement_data['laser_id']])) * np.cos(np.deg2rad(measurement_data['azimuth'])), 
        measurement_data['distance'] * np.sin(np.deg2rad(offset['pitch'][measurement_data['laser_id']]))
                    ]))


def transform_to_map(measurement_data):
    """ 
    Maps the LiDAR sentered measurements onto real world map through INS and lever arm. 
    lu -> laser unit
    la -> lever arm
    map -> Ground based coordinate
    R -> rotation matrix
    r -> displacement vector
    xyz -> coordinates 

    xyz_map = r_gps + R_gps * r_la + R_gps * R_la * xyz_lu
    """ 

    xyz_lu = calculate_xyz_lu(measurement_data)


    measurement_data['xyz_lu'] = xyz_lu
    xyz_lu = xyz_lu[:,:, np.newaxis]

    R_la = rotation_matrix(lidar_lever_arm['roll'], lidar_lever_arm['pitch'], lidar_lever_arm['yaw'])

    r_la = np.array([lidar_lever_arm['x'], lidar_lever_arm['y'], lidar_lever_arm['z']])
    r_la = r_la[np.newaxis,:,np.newaxis]

    R_gps = rotation_matrix_array(measurement_data['ins_rpy'])

    r_gps = measurement_data['ins_xyz']

    measurement_data['xyz'] =  r_gps + np.squeeze(np.matmul(np.matmul(R_gps,R_la), xyz_lu + r_la))

    """
    #This is a "manual" way of rotating the main degrees, probably faster. 
    temp = np.array(xyz_lu + r_la)
    changed_coordinate = np.array(temp[:,[0,2,1],:])
    changed_coordinate[:,:,:] *= -1
    measurement_data['xyz'] = r_gps + np.squeeze(np.matmul(R_gps, changed_coordinate))
    """
    
    
    