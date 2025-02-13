import os
import ssl
import socket
import time

ip_address = 'hermes.westpoint.edu'
port = 8089
cert_path = '~/catkin_ws/src/atak_bridge/src/user2.pem'

# Resolve the certificate path
cert_path = os.path.expanduser(cert_path)
print(__name__ + " Resolved cert_path: " + cert_path)

# Check if the certificate file exists
if not os.path.isfile(cert_path):
    print(f"Certificate file not found: {cert_path}")
    exit(1)

# Creating a standard socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print(__name__ + " socket created")

# Wrapping socket with SSL
context = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
context.load_verify_locations(cert_path)

# Allow self-signed certificates
context.check_hostname = False
context.verify_mode = ssl.CERT_NONE

sock = context.wrap_socket(sock, server_hostname=ip_address)
print("Loaded cert_path " + cert_path)

print(f"{__name__} Opening Secure Socket")
try:
    sock.connect((ip_address, port))
    print(f"{__name__} Connected successfully to {ip_address}:{port}")
except ssl.SSLCertVerificationError as e:
    print(f"Cert validation failed: {str(e)}")
    exit(1)
except Exception as e:
    print(f"Cannot connect to {ip_address}:{port}. Error: {str(e)}")
    exit(1)

# Function to send data
def send(sock, cotdata, sleeptime=.075):
    print("\nSENDING THE COT DATA!!!!!!!!!!!!!!!!!!!!!!\n")
    print(cotdata)
    try:
        print(__name__ + " Socket fileno: " + str(sock.fileno()))
        if sock.fileno() == -1:
            print(__name__ + " Socket Closed")
            raise Exception(__name__ + "Socket Closed")
    except Exception as e:
        print(__name__ + " Could not get socket status: " + str(e))
        raise Exception(__name__ + " could not get socket status: " + str(e))
    
    sentdata = ""
    try:
        sock.settimeout(0.5)  # 0 is non-blocking
        if isinstance(cotdata, str):
            cotdata = cotdata.encode('utf-8')  # Ensure data is encoded if it's a string
        print("\nSending the cot data\n")
        print("\nsentdata is: " + str(sentdata))
        print("\nlen(cotdata) is: " + str(len(cotdata)))
        print("\ncotdata type is: " + str(type(cotdata)))
        sentdata = sock.send(cotdata)
        if sentdata != len(cotdata):
            print(__name__ + " Socket Send mismatch " + str(sentdata) + " " + str(len(cotdata)))
            raise Exception(__name__ + " Socket Send mismatch " + str(sentdata) + " " + str(len(cotdata)))
        print("Data sent successfully")
    except socket.timeout as e:
        print(__name__ + " Socket Timeout: " + str(e))
        raise Exception(__name__ + " Socket Timeout: " + str(e))
    except ssl.SSLZeroReturnError as e:
        print(__name__ + " SSL connection has been closed (EOF): " + str(e))
        raise Exception(__name__ + " SSL connection has been closed (EOF): " + str(e))
    except Exception as e:
        print(__name__ + " Send data failed: " + str(e))
        raise Exception(__name__ + " Send Failed: " + str(e))

    # Set a minimum delay so the server does not get overrun
    time.sleep(sleeptime)
    return sentdata

# Test sending data
cotdata = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><event version="2.0" uid="ugv_rrc-de1-5bc9c538d51f" time="2025-01-30T15:56:21Z" start="2025-01-30T15:56:21Z" stale="2025-01-30T15:56:27Z" how="m-g" type="a-f-G-M-F-Q-f-X"><point lat="41.3911139" lon="-73.9530166" hae="9999999" ce="9999999.0" le="9999999.0" /><detail><contact endpoint="*:-1:stcp" callsign="husky" /><precisionlocation altsrc="GPS" geopointsrc="GPS" /><__group role="Team Member" name="Cyan" /><takv os="1" platform="takpak.mkcot" version="1.1.0" /><color argb="-1" /></detail></event>'

cotdata = cotdata.encode('utf-8')
send(sock, cotdata)