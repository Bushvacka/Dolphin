from socket import *
serverName = "192.168.1.2"
serverPort = 12000
clientSocket = socket(AF_INET, SOCK_STREAM)
clientSocket.connect((serverName, serverPort))
messagetoserver = input("Enter Message for Server: ")
clientSocket.send(messagetoserver)
replyfromserver = clientSocket.recv(1024)
print ("Reply Message from Server: ", replyfromserver)
clientSocket.close()