import network
import time
import socket

# --- Wi-Fi Connection ---
ap_if = network.WLAN(network.AP_IF)
ap_if.active(False)

wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect("Frederik - iPhone 15", "123456789")

# Wait for Wi-Fi connection
while not wifi.isconnected():
    time.sleep(1)

# --- HTTP Request ---
host = "www.httpforever.com"
port = 80

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(5) # Setting a timeout here is good practice, even in "bones" code

# The correctly formatted HTTP GET request
request = b"GET / HTTP/1.1\r\nHost: " + host.encode() + "\r\n\r\n"

s.connect((host, port))
s.send(request)

response = s.recv(4096)
print(response)

s.close()