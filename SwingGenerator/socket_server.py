import socket               # Import socket module

s = socket.socket()         # Create a socket object
port = 80                # Reserve a port for your service.
s.bind(('', port))        # Bind to the port

s.listen(5)                 # Now wait for client connection.
while True:
   c, addr = s.accept()     # Establish connection with client.
   print 'Got connection from', addr
   print c.recv(1024)
   c.send('Thank you for connecting')
   c.close()                # Close the connection
