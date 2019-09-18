import socket

def socket_setup(udp_ip, udp_port):
    sock = socket.socket(socket.AF_INET, #Internet
                            socket.SOCK_DGRAM, socket.IPPROTO_UDP) #UDP
    sock.bind(('', udp_port))
    sock.connect((udp_ip,udp_port))

    #sock.setsockopt(socket.IPPROTO_IP, socket.IP_HDRINCL, 1)
    return sock