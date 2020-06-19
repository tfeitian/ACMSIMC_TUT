import socket
import sys
import struct


SEND_BUF_SIZE = 256

RECV_BUF_SIZE = 256

Communication_Count: int = 0

receive_count: int = 0


def start_tcp_server(ip, port):
    # create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (ip, port)

    # bind port
    print("starting listen on ip %s, port %s" % server_address)
    sock.bind(server_address)

    while True:
        data, addr = sock.recvfrom(1024)
        print("Receive from %s:%s" % addr + str(data))

    print(" close client connect ")


if __name__ == '__main__':
    start_tcp_server('127.0.0.1', 8888)
