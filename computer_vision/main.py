import socket
import struct
import object_detection as od

BUFFER_SIZE = 8
DEMUX_SIG = 5       # number of signals from Simulink to de-multiplex


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind socket to port
server_address = ('localhost', 10100)
print(f'Starting at {server_address[0]}:{server_address[1]}')
sock.bind(server_address)

# Put socket in server mode and listen for incoming connections
sock.listen(1)
od.start_cv()

while True:
    # Blocking wait for connection
    print('Waiting for connection')
    connection, client_address = sock.accept()

    # when connection received
    try:
        print(f'Connection from {client_address}')

        for i in range(1, 5000):
            coord_newest = [0]*DEMUX_SIG
            # note: must send data first, or Simulink fails
            # send data to Simulink
            msg1 = struct.pack('>d', i)
            msg2 = struct.pack('>d', i/10)
            connection.send(msg1 + msg2)
            # print('sent data: ', i, 'and', i/10)

            # receive data from Simulink
            for j in range(DEMUX_SIG):
                data = connection.recv(BUFFER_SIZE)
                if (len(data)) == 8:
                    converted_data = struct.unpack('>d', data)[0]
                    coord_newest[j] = converted_data
                    print(j, ': received data:', converted_data)

            # update CV module with newest data
            od.set_coord(coord_newest)

    # close socket when done
    finally:
        sock.close()

