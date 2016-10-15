import socket               # Import socket module

s = socket.socket()         # Create a socket object
port = 80                # Reserve a port for your service.

s.connect(('192.168.1.41', port))
print s.recv(1024)
s.send("hello you are connected to an intel edison")
s.close                     # Close the socket when done
