from socket import *
HOST = "192.168.1.2"
PORT = 12000
s = socket(AF_INET, SOCK_STREAM)
s.connect((HOST, PORT))
messagetoserver = input("Enter Message for Server: ")
s.send(messagetoserver)
replyfromserver = s.recv(1024)
print ("Reply Message from Server: ", replyfromserver)
s.close()