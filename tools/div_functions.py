import os
import socket
import sys

#For file input
from tkinter import Tk
from tkinter.filedialog import askopenfilename, askdirectory
from tkinter import messagebox

def get_input_file_from_dialog(title, init_file_path, file_type):
    Tk().withdraw() # we don't want a full GUI, so keep the root window from appearing
    file_name = askopenfilename(initialdir=init_file_path, title = title, filetypes = ((file_type + " files" , "*." + file_type),("all files" , "*.*"))) # show an "Open" dialog box and return the path to the selected file

    if not os.path.isfile(file_name):
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