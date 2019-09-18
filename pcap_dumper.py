from scapy.utils import PcapWriter
from select import select as select
from time import perf_counter_ns as time_ns

import tools.networking as networking
from tools.lidar_values_and_settings import udp_info
from tools.lidar_values_and_settings import lidar_info




if __name__ == "__main__":
    sock_measurement_packet = networking.socket_setup(udp_info['ip'],udp_info['port_measurement_packet'])
    sock_position_packet = networking.socket_setup(udp_info['ip'],udp_info['port_position_packet'])
    
    file_name = input("Write file name (without .pcap)\n")

    packetdump = PcapWriter(file_name + ".pcap", append=True, sync=True)

    #These are just copies of the headers the LiDAR sends out while recording. I searched a long time for how to receive the UDP-header, which should be dead simple, but alas, I give up now.
    udp_header_measurement = b'\xff\xff\xff\xff\xff\xff\x60\x76\x88\x34\x37\x56\x08\x00\x45\x00\x04\xd2\x00\x00\x40\x00\xff\x11\xb4\x81\xc0\xa8\x01\xf1\xff\xff\xff\xff\x09\x40\x09\x40\x04\xbe\x00\x00'
    udp_header_position = b'\xff\xff\xff\xff\xff\xff\x60\x76\x88\x34\x37\x56\x08\x00\x45\x00\x02\x1c\x58\xd3\x00\x00\x40\x11\x5d\x65\xc0\xa8\x01\xf1\xff\xff\xff\xff\x20\x74\x20\x74\x02\x08\x00\x00'

    measurement_packets_saved = 0
    position_packets_saved = 0

    time_start = time_ns()
    seconds_passed = 0

    num_expected_measurement_packets_per_second = lidar_info['expected_number_of_packets_per_rotation'] * lidar_info['rotation_frequency']

    print("Expected number of measurement packets per second: " + str(num_expected_measurement_packets_per_second))

    while(True):   
        available_measurement = select([sock_measurement_packet],[], [], 0)
        if available_measurement[0]:
            data, addr =  sock_measurement_packet.recvfrom(1248)      
            packetdump.write(udp_header_measurement + data) #TODO: Should create a custom header with source and receiver port, data length and checksum (The checksum is currently set to 00 00, meaning no checking)
            measurement_packets_saved += 1
        else: 
            if (time_ns()-time_start)/1000000000 > seconds_passed: #Update every second
                seconds_passed += 1
                num_expected_measurement_packets = int(num_expected_measurement_packets_per_second * (time_ns() - time_start) /pow(10,9))
                print("Time since start[s]: " + str(seconds_passed) + " Measurement packets: " + str(measurement_packets_saved) + " Expected (AVG): " + str(num_expected_measurement_packets) + " Loss (AVG): " + str(num_expected_measurement_packets - measurement_packets_saved) + " Position packets: " + str(position_packets_saved), end='\r')

            available_position = select([sock_position_packet],[],[],0)
            if available_position[0]:
                packetdump.write(udp_header_position + sock_position_packet.recv(554))
                position_packets_saved += 1
        
        
        
"""
Header:
ff ff ff ff ff ff 60 76  88 34 37 56 08 00 45 00
04 d2 00 00 40 00 ff 11  b4 81 c0 a8 01 f1 ff ff 
ff ff 09 40 09 40 04 be  00 00


b'\xff\xff\xff\xff\xff\xff\x60\x76\x88\x34\x37\x56\x08\x00\x45\x00
\x04\xd2\x00\x00\x40\x00\xff\x11\xb4\x81\xc0\xa8\x01\xf1\xff\xff 
\xff\xff\x09\x40\x09\x40\x04\xbe\x00\x00'

#Measurement packet:
b'\xff\xff\xff\xff\xff\xff\x60\x76\x88\x34\x37\x56\x08\x00\x45\x00\x04\xd2\x00\x00\x40\x00\xff\x11\xb4\x81\xc0\xa8\x01\xf1\xff\xff\xff\xff\x09\x40\x09\x40\x04\xbe\x00\x00'

ff ff ff ff ff ff 60 76  88 34 37 56 08 00 45 00
02 1c 58 d3 00 00 40 11  5d 65 c0 a8 01 f1 ff ff 
ff ff 20 74 20 74 02 08  f9 75

ff ff ff ff ff ff 60 76  88 34 37 56 08 00 45 00
02 1c 58 d3 00 00 40 11  5d 65 c0 a8 01 f1 ff ff 
ff ff 20 74 20 74 02 08  00 00

b'\xff\xff\xff\xff\xff\xff\x60\x76\x88\x34\x37\x56\x08\x00\x45\x00\x02\x1c\x58\xd3\x00\x00\x40\x11\x5d\x65\xc0\xa8\x01\xf1\xff\xff\xff\xff\x20\x74\x20\x74\x02\x08\x00\x00
#Position packet:
b'
"""