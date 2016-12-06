import threading
import socket
import time
 
def fetch_local_ipv6_address(port=3000):
    # try to detect whether IPv6 is supported at the present system and
    # fetch the IPv6 address of localhost.
    if not socket.has_ipv6:
        raise Exception("the local machine has no IPv6 support enabled")
 
    addrs = socket.getaddrinfo('aaaa::212:7402:2:202', port, socket.AF_INET6, 0, socket.SOL_TCP)
    # example output: [(23, 0, 6, '', ('::1', 10008, 0, 0))]
 
    if len(addrs) == 0:
        raise Exception("there is no IPv6 address configured for localhost")
 
    entry0 = addrs[0]
    sockaddr = entry0[-1]
    return sockaddr
 
 
def ipv6_echo_server(sockaddr):
    # Echo server program
 
    s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
    s.bind(sockaddr)
    s.listen(1)
    print ("server opened socket connection:", s, ", address: '%s'" % sockaddr[0])
    conn, addr = s.accept()
 
    time.sleep(1)
    print ('Server: Connected by', addr)
    if True: # answer a single request
        data = conn.recv(1024)
        conn.send(data)
    conn.close()
 
 
def ipv6_echo_client(sockaddr):
    # Echo client program
    # use hostname or port number or use 'sockaddr' to open the connection
 
    #HOST = 'localhost'
    #PORT = 10008              # The same port as used by the server
    s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
    #s.connect((HOST, PORT))
    s.connect(sockaddr)
 
    print ("client opened socket connection:", s.getsockname())
    data = 'Hello, world! -> via IPv6 :-)'
    print ('Client is sending:', repr(data))
 
    s.send(data.encode())
    data = s.recv(1024).decode()
    s.close()
    print ('Client received response:', repr(data))
 
 
try:
    # fetch the local IPv6 address
    local_ipv6_addr = fetch_local_ipv6_address()
    t = threading.Thread(target=ipv6_echo_server, args=(local_ipv6_addr,))
    t.start()
 
    time.sleep(1)
    ipv6_echo_client(local_ipv6_addr)
 
except Exception as e:
    print ("Error occurred: ", e)
 
print ("bye.")
