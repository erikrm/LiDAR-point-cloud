import serial #https://pyserial.readthedocs.io/en/latest/pyserial_api.html 
from time import perf_counter_ns as time_ns
from time import sleep

from tools.lidar_values_and_settings import serial_settings as serial_settings

b = 3

def write_and_print_answer(ser,gps_code,description):
    """Sends gps_code with setings and prints the response"""
    ser.write(gps_code)
    for i in range(3): #Checking 3 times for the sentence, I have only encountered having to wait ones
        msg = ser.read_until(b'\r\n')
        if msg[:6] == gps_code[:6]:
            print(description,gps_code[1:7].decode('utf-8'),'settings:',msg)
            break
        if i == 2:
            print('Transmit or receive not connected, or message is wrong')

def initialize_gps(ser):
    """Initilizes the gps to use GPRMC, boadrate 9600 and the rest of the settings recommended in VLP-32C user manual"""
    # Source: https://static.garmincdn.com/pumac/GPS_18x_Tech_Specs.pdf
    #Section: 4.1.3 - 4.1.5

    PGRMCE = b'$PGRMCE\r\n'
    PGRMC1E = b'$PGRMC1E\r\n'
    PGRMC = b'$PGRMC,A,00140.2,28,,,,,,A,4,,2,4,30\r\n' #Sensor configuration 
    PGRMC1 = b'$PGRMC1,1,1,2,,,,2,W,N,,,,1,2,2\r\n'  #Sensor configuration extra
    PGRMO_disable_all = b'$PGRMO,,2\r\n' #Disables all output sentences
    PGRMO_enable_GPRMC = b'$PGRMO,GPRMC,1\r\n' #Enable GPRMC

    write_and_print_answer(ser,PGRMCE,'Old')
    write_and_print_answer(ser,PGRMC1E,'Old')
    write_and_print_answer(ser,PGRMC,'New')
    write_and_print_answer(ser,PGRMC1,'New')
    write_and_print_answer(ser,PGRMO_disable_all,'New')
    write_and_print_answer(ser,PGRMO_enable_GPRMC,'New')
    
def send_pps_pulse_once(ser):
    """Sends one pulse, uses 150 ms"""
    ser.setRTS(False) #HIGH
    sleep(0.1) #Default pps-width is 100 ms
    ser.setRTS(True) #LOW
    sleep(0.05) #Must wait minimum 50 ms after setting pps low

def wait_for_pps(ser,timeout):
    """Waits until pps signal goes high"""
    time_start_listening_cts = time_ns()
    while ser.getCTS(): #Waits until the pps signal goes HIGH -> False
        if time_ns() - time_start_listening_cts > timeout:
            print("pps has been True for", time_ns() - time_start_listening_cts, "ns")
            return
    return time_ns()

def gps_sentence_receive_and_pass_on(ser,new_timestamp):
    """Waits until it recieves any message then passes the message on with a new timestamp, send timestamp = None to not change the message"""
    msg_from_gps = ser.read_until(b'\r\n')
    #print("Time of gps read:",(time_ns() - time_top)/pow(10,6))

    if new_timestamp:
        msg_manipulated = msg_from_gps[0:7] + new_timestamp + msg_from_gps[13:] 
    else: msg_manipulated = msg_from_gps

    ser.write(msg_manipulated)

    #timestamp = msg_from_gps[7:13]
    try:
        timestamp = int(msg_from_gps[7:13])
    except ValueError:
        print("Gps timestamp raised an value error")
        timestamp = 0

    return timestamp #timestamp
    #print("Time of LiDAR write:",(time_ns() - time_top)/pow(10,6))

def initialize_serial():
    ser = serial.Serial(port=serial_settings['port'], baudrate=serial_settings['baudrate'],bytesize=serial_settings['bytesize'],timeout=serial_settings['timeout'],stopbits=serial_settings['stopbits'])
    ser.close() # Due to always quiting with keyboard interrupt the ser.close() is not reached on the bottom
    ser.open()
    return ser

if __name__ == "__main__":
    time_baseline = time_ns()
    ser = initialize_serial()
    GPRMC_example_sentence = b'$GPRMC,172028,A,5959.1639,N,01101.6761,E,000.0,229.1,110719,002.6,E,A*1C\r\n'
    timeout = 2*pow(10,9) #2 sec
    while True:
        time_top = wait_for_pps(ser,timeout)
        gps_sentence_receive_and_pass_on(ser,None)
        
        

    ser.close()


"""
b'[\xae\xb5U+\x9d\xfb\x8b\xf6\xb6\xd6\x96v\xad\x9d\xabkV\xd6l\xadv\x16vv\x1b\x9d\xfb\xeb\xd6\xf6\xd66W6\xb6\x96v\xad\x9d\xfb\xfb\xf66\xedv\xeb\x9666\xedv\xad\xd6\xf6\x16\xd6\xd6\xec\xed\xf6\xb66mv\xab\xad\x8b\xdb\x95\xeb\xb7q_qq}\xa7\x9f\x91\x9f\x9b\x9d\x99\xa7\x95\x8d\x95\x8d\xa3\x9d\x97\x91\x97\xa7c\xa7\x9f\x9d\x9d\x9f\x9d\xa3\x95\x93\x9b\x99\xa7u\xa7\x9d\xa7\x9d\x9f\xa7\x9f\xa3\x8f\xa7\x9d\x97\x9d\xa3\x9d\xa7e\xa7\x99\x95\xa3\x8f\xa7e\xa7\xa7\xab\x97\x9b\xe5\xeb\xb7q_qY}\xa7}\xa7\x99\xa7\x9f\x9b\xa7\x9f\x99\xa7\x9f\x93\xa7\x9d\x9b\xa7\x9d\x97\xa7\x9b\x97\xa7\x9b\x95\xa7\x9b\x8d\xa7\x99\x9d\xa7\x99\x9b\xa7\xa7\xa7\x9d\xa3\x93\xa7\x9f\xa3\x8f\xa7\x9d\xa3\x97\xab\x99\x9f\xe5\xeb\xb7q_qYS\xa7\x99\xa7\x9d\xa7\x9d\x9b\xa7\x9f\x9b\xa7\x99\x8d\xa7\x9d\x9f\x9f\xa7\x9d\x91\xa7\x9f\x99\xa7\x9f\x8d\xa7\x99\x97\x9b\xa7\x99\x95\xa7\x9f\x93\xa7\x9b\x8d\xa7\x9f\x97\x8d\xa7\x9b\x9d\xa7\x9d\x9b\xa7\x95\x91\xa7\x9f\x8d\x8d\xa7\x9b\x95\xab\x91{\xe5\xeb\xb7q_qYS\xa7\x99\xa7\x9b\xa7\x9d\x9b\xa7\x9d\x97\xa7\x99\x97\xa7\x9b\x91\x97\xa7\x99\x91\xa7\x9b\x97\xa7\x9d\x99\xa7\x9d\x95\x95\xa7\x9b\x99\xa7\x9b\x95\xa7\x91\x91\xa7\x9b\x97\x93\xa7\x99\x95\xa7\x9b\x8d\xa7\x99\x93\xa7\x9b\x9f\x95\xa7\x9d\x93\xab\x91\x91\xe5\xeb\xb7q_qYS\xa7\x99\xa7\x99\xa7\x9d\x9b\xa7\x99\x9d\xa7\x99\x95\xa7\x99\x9f\x9f\xa7\x97\x97\xa7\x99\x9b\xa7\x9b\x93\xa7\x9b\x95\x9f\xa7\x99\x8d\xa7\x9d\x8d\xa7\x9f\x95\xa7\x9f\x97\x93\xa7\x9f\x9f\xa7\x9b\x9b\xa7\x9f\x9d\xa7\x99\x9b\x93\xa7\x9f\x9f\xab\x91y\xe5\xeb\x00'
"""