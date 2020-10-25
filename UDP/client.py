import socket
 
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(('',6666))
print("UDP waiting")
 
while True:
	dataFromClient,address = server_socket.recvfrom(256)
	print(dataFromClient)
