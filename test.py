import socket

def main():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    local_addr = ("", 3838)
    udp_socket.bind(local_addr)

    recv_data = udp_socket.recvfrom(1024)

    result_recv_data=recv_data[0].hex()

    print(result_recv_data)#
    udp_socket.close()

if __name__ == "__main__":
    main()