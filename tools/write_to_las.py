import laspy
from tools.lidar_values_and_settings import dt_measurement
from tools.lidar_values_and_settings import lidar_info
from tools.lidar_values_and_settings import las_file_settings

import numpy as np
import os
#import udp_listen

def write_one_frame_to_las(file_name, header, data_meas):
    """Saves one frame to las, applies the offset and scale found in lidar_values_and_settings"""
    x_offset = 0
    y_offset = 0
    z_offset = 0

    if header == None: 
        header = laspy.header.Header(file_version=1.0, point_format=las_file_settings['point_format'])
    
    #print('Writing frame to {}...'.format(file_name))
    outfile = laspy.file.File(file_name,mode="w",header=header)
    
    scale = [las_file_settings['scale_x'], las_file_settings['scale_y'], las_file_settings['scale_z']]

    x_min = np.amin(data_meas['xyz'][:,0])
    x_max = np.amax(data_meas['xyz'][:,0])
    
    y_min = np.amin(data_meas['xyz'][:,1])
    y_max = np.amax(data_meas['xyz'][:,1])
    
    z_min = np.amin(data_meas['xyz'][:,2])
    z_max = np.amax(data_meas['xyz'][:,2])

    if y_min < 0 or x_min < 0:
        print("NEGATIVE!!? xy_min", x_min, y_min, "xy_max", x_max, y_max)

    if x_max > las_file_settings['max_x']:
        x_offset = int(x_min*scale[0]) #In meters

    if y_max > las_file_settings['max_y']:
        y_offset = int(y_min*scale[1]) #In meters

    if z_max > las_file_settings['max_z']:
        z_offset = int(z_min*scale[2])

    outfile.X = data_meas['xyz'][:,0] - x_offset/scale[0] #In measurement in mm, offset in m, have to divide by scale
    outfile.Y = data_meas['xyz'][:,1] - y_offset/scale[1] 
    outfile.Z = data_meas['xyz'][:,2] - z_offset/scale[2] #Shouldn't be necessary with offset for z

    outfile.intensity = data_meas['reflectivity'] # This mismatch in name convention can lead to confusion, the LiDAR user manual uses reflectivity while las and the VeloView software uses intensity
    outfile.gps_time = data_meas['timestamp'] #Must add seconds until TOH, this is only from TOH
    #outfile.scan_angle = data_meas['azimuth'] # 
    outfile.num_returns = data_meas['num_returns'] #Don't really need to find it for each measurement as the whole frame has the same num_return
    outfile.return_num = data_meas['return_num'] #Don't really need to find it for each measurement as the whole frame has the same return_num
    '''
    outfile.flag_byte = 
    '''
    outfile.user_data = data_meas['laser_id']
    
    #All colors should be normalized to 16 bit values (including NIR)
    #outfile.blue = (data_meas['distance'] == 0) * 65535 # Laspy saves this as green for some reason...
    #outfile.green(0)
    #outfile.red = 0
    #outfile.set_nir(0) #NIR (Near infrared) channel value 

    

    #outfile.blue = (data_meas['laser_id'] + 1) *  2048 - 1#Color laser 0-31 in gradients of green (laspy mixes colors..)
    #outfile.green = (data_meas['laser_id'] == 5) * 65535 # Set the horizontal laser to blue (and a little green)
    #outfile.blue = (data_meas['laser_id'] - 31) * 2048 -1

    outfile.header.set_offset([x_offset, y_offset, z_offset])
    outfile.header.set_scale(scale) #precision mm precision, multiply with 0.001 due to already operating in mm
    #outfile.header.set_wkt()
    outfile.close()

def write_points_to_las_with_offset(file_name, header, data, offset):
    if header == None:
        header = laspy.header.Header(file_version=1.0, point_format=las_file_settings['point_format'])
    
    outfile = laspy.file.File(file_name, mode="w", header=header)

    outfile.points = data
    outfile.header.set_scale([las_file_settings['scale_x'],las_file_settings['scale_y'],las_file_settings['scale_z']]) 
    outfile.header.set_offset(offset)
    outfile.close()


def write_data_into_files(file_name, header, data, offset):
    if file_name.endswith(".las"):
        file_name_temp = file_name
    else: 
        file_name_temp = file_name + ".las"
    
    write_points_to_las_with_offset(file_name_temp, header, data, offset)
    print(file_name_temp)


def write_data_into_files_based_on_user_data_with_offset(file_name, header, data, offset):
    for i in range(lidar_info['num_lasers']):      
        if file_name.endswith(".las"):
            file_name_temp = file_name.replace(".las", "laser_id_" + str(i) + ".las")
        else: 
            file_name_temp = file_name + "laser_id_" + str(i) + ".las"

        if np.any(data['point']['user_data'] == i):
            write_points_to_las_with_offset(file_name_temp, header, data[data['point']['user_data'] == i], offset)
            print(file_name_temp)


def load_las_files_in_directory_with_offset(directory_path, num_files):
    """Loads all las files in directory into one numpy array, returns -1 if no files"""
    if num_files > len(os.listdir(directory_path)):
        num_files = len(os.listdir(directory_path)) # Load all files

    num_lasers_in_use = lidar_info['num_lasers_in_use']
    fire_seq_per_packet = 12
    num_packets = 53 #Usually 52, need som margin apparently 
    data_len_file = num_lasers_in_use*fire_seq_per_packet*num_packets
    data_len_total = num_files * data_len_file
    
    data_table_exists = False
    
    idx_to_write = 0
    idx_file_num = 0
    
    #Find general information:
    offset_min = [0,0,0]

    loaded_frame_files = []

    idx = 0
    for file in os.listdir(directory_path): # Go through all the files to find the smallest offset
        if file.endswith(".las"):
            file_name = os.path.join(directory_path, file)
            infile = laspy.file.File(file_name, mode="r")

            for i in range(len(offset_min)):
                if infile.header.offset[i] < offset_min[i] or offset_min[i] == 0: #The offset should never be negative since we are using UTM
                    offset_min[i] = infile.header.offset[i]
            infile.close()

            idx += 1
            if idx == num_files:
                break
            

    print("offset_min", offset_min)

    

    for file in os.listdir(directory_path):
        if file.endswith(".las"):
            file_name = os.path.join(directory_path, file)
            infile = laspy.file.File(file_name, mode="r")

            if (not data_table_exists):
                data = np.zeros(data_len_total,dtype=infile.points.dtype)
                data_table_exists = True


            current_offset = infile.header.get_offset()
            current_scale = infile.header.get_scale()

            data_temp = np.copy(infile.points)
            data_temp['point']['X'] += int((current_offset[0] - offset_min[0])/current_scale[0]) #Adding the extra offset
            data_temp['point']['Y'] += int((current_offset[1] - offset_min[1])/current_scale[1])



            #import sys
            #sys.exit(0)
            #X = infile[:][0][0][0] * offset[0]/scale[0] #X
            #Y = infile[:][0][0][1] * offset[1]/scale[1] #Y

            data[idx_to_write : idx_to_write + infile.points.size] = data_temp  

            idx_to_write += infile.points.size
        
            infile.close()
            
            loaded_frame_files.append(file_name)

            idx_file_num += 1
            if num_files == idx_file_num:
                break

    if not data_table_exists:
        print("No data in directory path", directory_path)
        return False, offset_min, loaded_frame_files
    else: 
        print("Expected measurements: "+ str(data_len_total) + " Received measurements: " + str(idx_to_write) + " Data loss: " + str(data_len_total - idx_to_write))
        return data[0:idx_to_write], offset_min, loaded_frame_files


def delete_las_files_in_directory(directory_path, num_files):
    idx_file_num = 0
    still_contain_files = False 
    for file in os.listdir(directory_path):
        if file.endswith(".las"):
            file_name = os.path.join(directory_path, file)
            os.remove(file_name)
            idx_file_num += 1
            if idx_file_num == num_files:
                still_contain_files = True
                break

    return still_contain_files #If 
