"""Setting and constants used by the program set, all times are given in nanoseconds, 
important to set serial_settings, udp_info and lidar_settings"""
import numpy as np
from openpyxl import load_workbook
from .div_functions import get_input_file_from_dialog

#Constants:
lidar_constants = {
    'num_lasers': 32,
    'baudrate': 9600,
    'fire_sequence_length': 55296,
    'single_fire_length': 2304, 
    'fire_sequences_per_packet': 12,
    'num_fire_sequence_bytes': 100
}

excel_setting_file_name = get_input_file_from_dialog("Excel LiDAR settings", "./", "xlsx")

# Sets settings equal to the users preferences if the user chooses a suitable file, otherwise the default setting are applied
if excel_setting_file_name != -1:
    wb = load_workbook(filename = excel_setting_file_name)
    sheet_lidar_settings = wb['Settings']

    lidar_settings = {
        'fov_size': sheet_lidar_settings['B6'].value,
        'rpm': sheet_lidar_settings['B7'].value, #Must be divisible by 60 and in the range [300, 1200], The LiDAR won't accept anything else. 
        'num_returns': sheet_lidar_settings['B8'].value,  #Dual mode = 2, single mode = 1. The programs understands the number of returns themself, this is used for testing
        'timeout_between_packets':sheet_lidar_settings['B9'].value, #How long to wait before deciding that all packets in a FOV has been recieved
        'num_lasers_in_use': 32
    }

    udp_info = {
    'ip': sheet_lidar_settings['B12'].value,
    'port_measurement_packet': sheet_lidar_settings['B13'].value,
    'port_position_packet': sheet_lidar_settings['B14'].value,
    }

    # xyz in mm, roll, pitch, yaw in degrees. Given in LiDAR perspective with coordinate system as defined in user manual 9.2
    lidar_lever_arm = {
        'x': sheet_lidar_settings['B17'].value,
        'y': sheet_lidar_settings['B18'].value, 
        'z': sheet_lidar_settings['B19'].value,
        'roll': sheet_lidar_settings['B20'].value,
        'pitch': sheet_lidar_settings['B21'].value, 
        'yaw': sheet_lidar_settings['B22'].value
    }

    gps_leap_seconds = sheet_lidar_settings['B25'].values

else: #Default settings if the provided sheet is wrong
    lidar_settings = {
        'fov_size': 30,
        'rpm': 300, #Must be divisible by 60 and in the range [300, 1200], The LiDAR won't accept anything else. 
        'num_returns': 2,  #Dual mode = 2, single mode = 1. The programs understands the number of returns themself, this is used for testing
        'timeout_between_packets':10*pow(10,6), #How long to wait before deciding that all packets in a FOV has been recieved
        'num_lasers_in_use': 32
    }

    # xyz in mm, roll, pitch, yaw in degrees. Given in LiDAR perspective with coordinate system as defined in user manual 9.2
    lidar_lever_arm = {
        'x': 0,
        'y': 0, 
        'z': 0,
        'roll': 270,
        'pitch': 180, 
        'yaw': 0
    }

    # Socket: 
    udp_info = {
        'ip':'192.168.168.201',
        'port_measurement_packet': 2368,
        'port_position_packet': 8308,
    }

    gps_leap_seconds = 18


lidar_info = {
    'rotation_frequency':int(lidar_settings['rpm']/60), # 5Hz
    'rotation_period':int(60/lidar_settings['rpm']*pow(10,9)),  #200 ms
    'time_in_fov':int(lidar_settings['fov_size']/360 * 60/lidar_settings['rpm'] * pow(10,9)), #13.888 ms for fov 25, rpm 300
    'expected_number_of_packets_per_rotation': lidar_settings['num_returns'] + int(lidar_settings['fov_size']/360 * 60/lidar_settings['rpm'] * pow(10,9) * lidar_settings['num_returns'] /(lidar_constants['fire_sequences_per_packet']*lidar_constants['fire_sequence_length'])), #44. We receive num_returns packets more than the calculations say, the LiDAR extends a little outside FOVs, that is the added constant
    'num_firings_per_packet': lidar_constants['fire_sequences_per_packet'] * lidar_constants['num_lasers'], # 384
    'max_azimuth_gap': lidar_constants['fire_sequence_length']*lidar_settings['rpm']*360/60/pow(10,9), #max gap between azimuth angles of successive data_blocks with out a jump between FOV
    'position_packets_per_second': 1507 #From user manual
}

udp_info.update({'packet_read_time': lidar_info['time_in_fov'] + 5*pow(10,6), # adding 5ms buffer to be sure to get all packets)
                 'header_size': 42 #bytes
                }) 

lidar_info.update(lidar_constants)
lidar_info.update(lidar_settings)

#Serial
serial_settings = {
    'port' : 'COM14',
    'baudrate': 9600,
    'bytesize': 8,
    'timeout': 10,
    'stopbits': 1
}


#Laser offset angles
# The lasers are not pointed horizontaly outwards
# Omega compensation (vertical)
pitch_deg_degree = [-25.0, -1.0, -1.667, -15.639, -11.31, 0.0, -0.667, -8.843,
             -7.254, 0.333, -0.333, -6.148, -5.333, 1.333, 0.667, -4.0,
             -4.667, 1.667, 1.0, -3.667, -3.333, 3.333, 2.333, -2.667,
             -3.0, 7.0, 4.667, -2.333, -2.0, 15.0, 10.333, -1.333]

#pitch_deg_degree = [0]*32
# Azmiuth compensation (horizontal) for each laser

azimuth_offset_degree = [1.4, -4.2, 1.4, -1.4, 1.4, -1.4, 4.2, -1.4,
             1.4, -4.1, 1.5, -1.4, 4.2, -1.4, 4.2, -1.4,
             1.4, -4.2, 1.4, -4.2, 4.2, -1.4, 1.4, -1.4,
             1.4, -1.4, 1.4, -4.2, 4.2, -1.4, 1.4, -1.4]

#azimuth_offset_degree = [0]*32

angle_offset_per_laser = np.zeros(len(pitch_deg_degree),dtype=[('pitch','f8'),('azimuth','f8')])
angle_offset_per_laser['pitch'] = pitch_deg_degree
angle_offset_per_laser['azimuth'] = azimuth_offset_degree

#Numpy array
#data types:
dt_measurement = [('frame_nr','u4'),('data_packet_nr','u4'),('data_block','u2'),('laser_id','u2'),('gps_time', 'u8'),('timestamp','u8'),('azimuth_centered','f4'),('distance','f4'),('reflectivity','u1'),('azimuth','f4'),('return_num','u1'),('num_returns','u1'),('xyz','f8',(3)),('xyz_lu','f8',(3)),('ins_xyz','f8',(3)),('ins_rpy','f8',(3))]
dt_position = [('timestamp','u8'),('nmea','U256'),('pps','u1'),('temp_top_board','u1'),('temp_bottom_board','u1'),('temp_when_adc_calibration_last_ran','u1'),('temp_change_since_last_adc_calibration','i2'),('reason_last_adc_calibration','u1'),('status_adc_calibration','u1'),('thermal_status','u1'),('last_shutdown_temp','u1'),('temp_unit_power_up','u1')]
dt_ins_gps = [('timestamp','u8'),('latitude','f8'),('longitude','f8'),('altitude','f8'),('north_velocity','f4'),('east_velocity','f4'),('down_velocity','f4'),('roll','f8'),('pitch','f8'),('heading','f8')]
dt_ins_local = [('timestamp','u8'),('x','f16'),('y','f16'),('z','f16'),('roll','f8'),('pitch','f8'),('yaw','f8')] #The idea is to convert from dt_ins_gps to a local frame and save it in dt_ins_local. First I will just simulate the values in dt_ins_local

#Position packet information distribution for integers:
#Table 9-3 page 59
position_info_distribution_ints = {
    'temp_top_board': [0xBB,1],
    'temp_bottom_board': [0xBC,1],
    'temp_when_adc_calibration_last_ran': [0xBD,1],
    'temp_change_since_last_adc_calibration': [0xBE,2],
    'reason_last_adc_calibration': [0xC4,1],
    'status_adc_calibration': [0xC5,1],
    'timestamp': [0xC6,4],
    'pps': [0xCA,1],
    'thermal_status': [0xCB,1],
    'last_shutdown_temp': [0xCC,1],
    'temp_unit_power_up': [0xCD,1]
}
