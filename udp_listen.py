#!python3
import socket
import os, sys
from select import select as select
import numpy as np
import datetime
from time import sleep
import tools.udp_unpack as udp_unpack
import tools.write_to_las as write_to_las
from time import perf_counter_ns as time_ns
from tools.lidar_values_and_settings import lidar_info
from tools.lidar_values_and_settings import dt_measurement
from tools.lidar_values_and_settings import dt_position
from tools.lidar_values_and_settings import dt_ins_local
from tools.lidar_values_and_settings import udp_info

from ximc.examples.testpython import testpython as stage_control

from tools import serial_communication

import tools.networking as networking


def retrieve_packet_batch(timeout_between_packets, timeout_total):
    time_top = time_ns()
    

    data_measurements = []
    data_position = []

    sock_measurement_packet = networking.socket_setup(udp_info['ip'],udp_info['port_measurement_packet'])
    sock_position_packet = networking.socket_setup(udp_info['ip'],udp_info['port_position_packet'])

    time_of_last_measurement_packet = 0
    received_any_measurement_packets = False

    while not received_any_measurement_packets or (time_ns() - time_of_last_measurement_packet) < timeout_between_packets: #The timeout here is to stop reading after the stream of packets has stopped
       
        available_measurement = select([sock_measurement_packet],[],[],0)

        if available_measurement[0]:           
            data_measurements.append(sock_measurement_packet.recv(1248))
            received_any_measurement_packets = True 
            time_of_last_measurement_packet = time_ns()

        available_position = select([sock_position_packet],[],[],0)
        if available_position[0]:
            data_position.append(sock_position_packet.recv(554))

        if time_ns() - time_top > timeout_total and not received_any_measurement_packets: #The timeout here is in case there is no connection
            break

    sock_measurement_packet.shutdown(socket.SHUT_RD)
    sock_position_packet.shutdown(socket.SHUT_RD)
    sock_measurement_packet.close()
    sock_position_packet.close()

    return data_measurements, data_position


def retrieve_unpack_store(txyzrpy_delta_txyzrpy_shared, lock_txyzrpy, lock_socket, frame_nr_shared, file_location, header, laser_id_save_list, process_nr):
    data_measurements = []

    last_azimuth_block = -1    

    data_packet_length = lidar_info['num_firings_per_packet']
    num_data_packets = lidar_info['expected_number_of_packets_per_rotation']
    timeout_between_packets = lidar_info['timeout_between_packets'] 
    timeout_total = lidar_info['rotation_period'] # Read for one rotation.
    

    data_length_fov = data_packet_length * num_data_packets

    data_position_length = int(data_length_fov/14)
    data = np.zeros(data_length_fov,dtype=dt_measurement)
    data_position = np.zeros(data_position_length,dtype=dt_position)

    while True:
        lock_socket.acquire() #Only one process at the time can read from the socket 
        data_measurements, data_position = retrieve_packet_batch(timeout_between_packets, timeout_total) #5 ms timeout
        lock_socket.release()
            
        
        if data_measurements:
            time_top = time_ns()
            #print("N data meas:",len(data_measurements),"Process", process_nr)
            
            for i_packet in range(len(data_measurements)):
                data_packet,last_azimuth_block = udp_unpack.unpack_measurement(data_measurements[i_packet],frame_nr_shared.value,last_azimuth_block,i_packet)
                data[i_packet*data_packet_length : (i_packet+1)*data_packet_length][:] = data_packet

            if data_position:
                for i_position_packet in range(len(data_position)):
                    data_position[i_position_packet] = udp_unpack.unpack_position(data_position[i_position_packet])
            
            lock_txyzrpy.acquire()
            delta_t_exists = udp_unpack.interpolate_ins(data,txyzrpy_delta_txyzrpy_shared) 
            lock_txyzrpy.release()
            
            if delta_t_exists:
                udp_unpack.transform_to_map(data)
                file_name = file_location + 'frame_' + str(frame_nr_shared.value) + '.las'
                frame_nr_shared.value += 1
                
                if laser_id_save_list:
                    mask = np.isin(data['laser_id'], laser_id_save_list)
                    write_to_las.write_one_frame_to_las(file_name,header,data[mask]) #TODO remove where distance is 0 or timestamp is 0?
                else:
                    write_to_las.write_one_frame_to_las(file_name,header,data) #TODO remove where distance is 0 or timestamp is 0?
                #data2 = data
                #array = np.append(data2['xyz'], data2['xyz_lu'], axis=1)
                #np.savetxt('./frame_files_csv/frame_' + str(frame_nr_shared.value) + '.csv',array,delimiter=',')
                #np.savetxt('./frame_files_csv/frame_' + str(frame_nr_shared.value) + '_lu.csv',data2['xyz_lu'],delimiter=',')
            
            if len(data_measurements) != num_data_packets: #Print if packet loss
                print("txyzrpy_delta_txyzrpy",txyzrpy_delta_txyzrpy_shared[:])
                print("Frame nr:", frame_nr_shared.value, "N data meas:",len(data_measurements),"Process", process_nr, "Process time usage",(time_ns()-time_top)/pow(10,9))
            
            print("Frame nr: " + str(frame_nr_shared.value) + " Process time usage " + str((time_ns()-time_top)/pow(10,9)), end='\r')
            data_measurements = []
            data.fill(0)


def calculate_delta(new_ins_local, old_ins_local):
    txyzrpy_delta_txyzrpy = [0]*14

    new_t = new_ins_local['timestamp']
    old_t = old_ins_local['timestamp']

    new_t = datetime.timedelta(hours=int(new_t/10000),minutes=int(new_t/100)%100, seconds=new_t%100)
    old_t = datetime.timedelta(hours=int(old_t/10000),minutes=int(old_t/100)%100, seconds=old_t%100)
    
    delta_t = new_t - old_t
    delta_t = delta_t.seconds

    if delta_t > 10:
        delta_t = 0 #Unreasonable as it should be 1 second or 200 ms, definitely not 10 seconds

    txyzrpy_delta_txyzrpy[0] = new_t.seconds %3600 #Seconds since top of hour
    txyzrpy_delta_txyzrpy[1] = new_ins_local['x']
    txyzrpy_delta_txyzrpy[2] = new_ins_local['y']
    txyzrpy_delta_txyzrpy[3] = new_ins_local['z']
    txyzrpy_delta_txyzrpy[4] = new_ins_local['roll']
    txyzrpy_delta_txyzrpy[5] = new_ins_local['pitch']
    txyzrpy_delta_txyzrpy[6] = new_ins_local['yaw']
    txyzrpy_delta_txyzrpy[7] = delta_t
    txyzrpy_delta_txyzrpy[8] = new_ins_local['x'] - old_ins_local['x']
    txyzrpy_delta_txyzrpy[9] = new_ins_local['y'] - old_ins_local['y']
    txyzrpy_delta_txyzrpy[10] = new_ins_local['z'] - old_ins_local['z']
    txyzrpy_delta_txyzrpy[11] = new_ins_local['roll'] - old_ins_local['roll']
    txyzrpy_delta_txyzrpy[12] = new_ins_local['pitch'] - old_ins_local['pitch']
    txyzrpy_delta_txyzrpy[13] = new_ins_local['yaw'] - old_ins_local['yaw']
    return txyzrpy_delta_txyzrpy

def simulate_movement_one_direction(axis, steps_per_second, new_ins_local, old_ins_local):
    if movement_axis in ['x', 'y', 'z']:
        delta_distance =  0.0125 * steps_per_second #6.25 mm/s for stage speed 500 
    elif movement_axis in ['roll', 'pitch', 'yaw']:
        delta_distance =  steps_per_second/100 #100 steps equals one degree 
    else: 
        print("Wrong axis name, must be x,y,z,roll,pitch or yaw")

    new_ins_local[axis] = delta_distance + old_ins_local[axis]
    return new_ins_local

def ins_simulator(txyzrpy_delta_txyzrpy_shared, lock_txyzrpy, ser, processes_to_start, steps_per_second, movement_axis):
    ins_local_length = 500
    ins_local = np.zeros(ins_local_length,dtype=dt_ins_local) #Some buffer
    ins_local_idx = 0

    started = False

    #Need two baseline measurement to do the first delta from
    gps_timestamp = serial_communication.gps_sentence_receive_and_pass_on(ser,None) 
    print(gps_timestamp)
    ins_local['timestamp'][ins_local_idx] = gps_timestamp
    ins_local_idx += 1

    gps_timestamp = serial_communication.gps_sentence_receive_and_pass_on(ser,None) 
    ins_local['timestamp'][ins_local_idx] = gps_timestamp
    ins_local_idx += 1

    while(not exit_program):
        gps_timestamp = serial_communication.gps_sentence_receive_and_pass_on(ser,None)
        ins_local['timestamp'][ins_local_idx] = gps_timestamp
        
        ins_local[ins_local_idx] = simulate_movement_one_direction(movement_axis,steps_per_second,ins_local[ins_local_idx],ins_local[ins_local_idx-1])

        lock_txyzrpy.acquire()
        txyzrpy_delta_txyzrpy_shared[:] = calculate_delta(ins_local[ins_local_idx], ins_local[ins_local_idx-1])
        lock_txyzrpy.release()

        if not started and txyzrpy_delta_txyzrpy_shared[7] > 0: #Wait until we have enough gps data before starting reading from sockets
            processes_to_start[-1].start() #Start moving the stage
            sleep(0.5) #Wait until any packets from non top speed is reached
            for i in range(len(processes_to_start)-1):
                processes_to_start[i].start()
                
            started = True
          
        ins_local_idx += 1

        if ins_local_idx == ins_local_length:
            #Save ins_local to file?
            ins_local_idx = 0

def stage_scan_to_top_move_to_bottom(lib, device_id, stage_start, stage_end, lock_socket, STEPS_PER_SECOND_MAX):
    stage_control.test_move(lib, device_id, stage_end, 0)

    while(stage_control.get_position(lib, device_id)[0] < stage_end - 10):
        pass

    lock_socket.acquire() #Only want to film upwards one time
    stage_control.test_wait_for_stop(lib, device_id, interval)
    stage_control.test_set_speed(lib, device_id, STEPS_PER_SECOND_MAX) #GOing home, and going home fast
    stage_control.test_move(lib, device_id, stage_start, 0)
    stage_control.test_wait_for_stop(lib,device_id,interval)



if __name__ == "__main__":
    import multiprocessing 
    import threading

    #Serial setup
    ser = serial_communication.initialize_serial()

    #Stage setup
    lib, device_id = stage_control.connection_setup()
    interval = 150

    translation_true_rotation_false = False
    if translation_true_rotation_false:
        #Translation stage
        STAGE_MIN = 11100
        STAGE_MAX = STAGE_MIN + 29500
    else: 
        #Rotation stage
        STAGE_MIN = 16000
        STAGE_MAX = 22000


    STEPS_PER_SECOND_MAX = 1500
    STEPS_PER_SECOND = 100
  
    stage_start = STAGE_MIN
    stage_end = STAGE_MAX

    movement_axis = 'yaw'
    direction = 1 # 1 for up, -1 for down. Anything else means trouble, I don't check for anything.
    
    stage_control.test_set_speed(lib, device_id, STEPS_PER_SECOND_MAX)    
    stage_control.test_move(lib, device_id, stage_start, 0)
    stage_control.test_wait_for_stop(lib, device_id, interval)
    stage_control.test_set_speed(lib, device_id, STEPS_PER_SECOND)  

    #Inital variables
    frame_nr = 0
    txyzrpy_delta_txyzrpy_initial = [0]*14
    processes = []
    global exit_program
    exit_program = False

    #Shared variables
    txyzrpy_delta_txyzrpy_shared = multiprocessing.Array('f',txyzrpy_delta_txyzrpy_initial) #timestamp_start + delta timestamp,x,y,z,roll,pitch,yaw
    frame_nr_shared = multiprocessing.Value('i',0)
    lock_socket = multiprocessing.Lock()
    lock_txyzrpy = multiprocessing.Lock()
    
    #Save settings
    laser_id_save_list = [] #The closest horizontal lasers
    frame_files_nr = 1
    time_now = datetime.datetime.now()
    description = "test_new_position"
    directory_name = str(time_now.year) + "_" + str(time_now.month) + "_" + str(time_now.day) + "_" + str(frame_files_nr) + "_" + description
    directory_path = "./frame_files/frame_files_"
    file_location = directory_path + directory_name + "/"
    os.mkdir(file_location)

    header = None #If none the program creates a default one
    
    processes.append(multiprocessing.Process(target=retrieve_unpack_store, args=(txyzrpy_delta_txyzrpy_shared, lock_txyzrpy, lock_socket, frame_nr_shared, file_location, header, laser_id_save_list, 0)))
    processes.append(multiprocessing.Process(target=retrieve_unpack_store, args=(txyzrpy_delta_txyzrpy_shared, lock_txyzrpy, lock_socket, frame_nr_shared, file_location, header, laser_id_save_list, 1)))
    processes.append(multiprocessing.Process(target=retrieve_unpack_store, args=(txyzrpy_delta_txyzrpy_shared, lock_txyzrpy, lock_socket, frame_nr_shared, file_location, header, laser_id_save_list, 2)))
    #processes.append(threading.Thread(target=retrieve_unpack_store, daemon=True, args=(txyzrpy_delta_txyzrpy_shared, lock_txyzrpy, lock_socket, frame_nr_shared, file_location, header, laser_id_save_list, 1)))

    processes.append(threading.Thread(target=stage_scan_to_top_move_to_bottom, daemon=False, args=(lib,device_id, stage_start, stage_end, lock_socket, STEPS_PER_SECOND_MAX)))
    pser = threading.Thread(target=ins_simulator, args=(txyzrpy_delta_txyzrpy_shared, lock_txyzrpy, ser,processes, STEPS_PER_SECOND*direction, movement_axis))

    pser.start()
    
    try: 
        key = input("Enter to quit\n")
        exit_program = True

    except KeyboardInterrupt:
        exit_program = True



    for i in range(len(processes)-1):
        processes[i].terminate()
        processes[i].join()
        processes[i].close()


    # Collecting all files and deviding them based on color instead of frames

    directory = directory_path + directory_name
    
    classifier_array = ['user_data']
    
    data = write_to_las.load_all_las_files_in_directory(directory)
    
    outfile_directory = "./collected_las_files/"
    outfile_file_name = directory_name
    write_to_las.write_data_into_files_based_on_classifier(outfile_directory + directory_name, None, data, classifier_array)
    
    ser.close()

