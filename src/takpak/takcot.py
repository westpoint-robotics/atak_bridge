# takcot.py
__author__ = 'Alan Barrow <traveler@pinztrek.com>'
__copyright__ = 'Copyright 2020 Alan Barrow'
__license__ = 'GPL, Version 3+'
import ssl
import OpenSSL.crypto
import os
import sys
#from time import sleep,gmtime,strftime
import time

import logging
import socket

class Error(Exception):
    """Base class for exceptions in this module."""
    pass
    
# TODO investigate why this class exists, python socket module should be able to do this
class SocketError(Error):

    """Exception raised for errors in the input.

    Attributes:
        expression -- input expression in which the error occurred
        message -- explanation of the error
    """
    # TODO DML made below changes to fix broken exception handling in the code
    #def __init__(self, expression, message):
    #    self.expression = expression        
    def __init__(self, message):
        self.message = message

class takcot():
    """
    Connects, Sends and receives properly formed CoT's to TAK servers
    Tested on FTS, but should work on TAK
    """

    def __init__(self, logger=None):
        # use existing logger
        self.logger = logger or logging.getLogger(__name__)

        self.logger.debug(__name__ + " self.logger logging started")

        self.sock = None
        
    #def open(self, ip_address, port=8087):# use_ssl=False, cert_file='user2.p12', cert_password=None): #8087
    def open(self, ip_address, port=8089, cert_path='~/catkin_ws/src/atak_bridge/src/user2.p12'):
        self.logger.info(__name__ + " Opening: " + ip_address + ":" + str(port))
        print("\n Starting open, doing the SSL stuff here!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n")
        try:

            cert_path = os.path.expanduser(cert_path)
            print(__name__ + " Resolved cert_path: " + cert_path)

            # Check if the certificate file exists
            if not os.path.isfile(cert_path):
                print(f"Certificate file not found: {cert_path}")
                return None

            ##cryptology_functions.convert_cert(cert_path, cert_password)


            # Creating a standard socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print(__name__ + " socket created")

            # Wrapping socket with SSL
            #context.load_cert_chain(certfile='path/to/client.p12', password='your_password')
            context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
            
            context.load_verify_locations(cert_path)
            # Allow self-signed certificates
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
            self.sock = context.wrap_socket(self.sock, server_hostname=ip_address)
            print("Loaded cert_path " + cert_path)

            print(f"{__name__} Opening Secure Socket")
            #-------------------------------------------------------------------------
            self.sock.connect((ip_address, port))
            print(f"{__name__} Connected successfully to {ip_address}:{port}")
        except ssl.SSLCertVerificationError as e:
            print(f"Cert validation failed: {str(e)}")
            self.sock = None
        except Exception as e:
            print(f"Cannot connect to {ip_address}:{port}. Error: {str(e)}")
            self.sock = None
        return self.sock
            
        
    def close(self):
        try:
            #closereturn = self.sock.shutdown(1)
            #time.sleep(0.2)
            closereturn = self.sock.close()
            self.logger.debug(__name__ + "takserver connection closed")
        except:
            closereturn = 0
            self.logger.warning(__name__ + " Socket Close failed")
        return closereturn

    def send(self, cotdata, sleeptime=.075 ):
        print("\nSENDING THE COT DATA!!!!!!!!!!!!!!!!!!!!!!\n")
        print(cotdata)
        
        try:
            print(__name__ + " Socket fileno: " + str(self.sock.fileno()))
            if self.sock.fileno() == -1:
                print(__name__ + " Socket Closed")
                raise SocketError(__name__ + "Socket Closed")
        except Exception as e:
            print(__name__ + " Could not get socket status: " + str(e))
            raise SocketError(__name__ + " could not get socket status: " + str(e))
        sentdata=""
        try:
            self.sock.settimeout(0.5) # 0 is non-blocking
            if isinstance(cotdata, str):
                cotdata = cotdata.encode('utf-8')  # Ensure data is encoded if it's a string
            print("\nSending the cot data\n")
            print("\nsentdata is: " + str(sentdata))
            print("\nlen(cotdata) is: " + str(len(cotdata)))
            print("\ncotdata type is: " + str(type(cotdata)))
            sentdata = self.sock.send(cotdata)
            
            if sentdata != len(cotdata):
                print(__name__ + " Socket Send mismatch " + str(sentdata) + " " + str(len(cotdata)))
                raise SocketError(__name__ + " Socket Send mismatch " + str(sentdata) + " " + str(len(cotdata)))
            self.logger.debug("Data sent successfully")
        except socket.timeout as e:
            print(__name__ + " Socket Timeout: " + str(e))
            raise SocketError(__name__ + " Socket Timeout: " + str(e))
        except ssl.SSLZeroReturnError as e:
            print(__name__ + " SSL connection has been closed (EOF): " + str(e))
            raise SocketError(__name__ + " SSL connection has been closed (EOF): " + str(e))
        except Exception as e:
            self.logger.warning(__name__ + " Send data failed: " + str(e))
            print(__name__ + " Send data failed: " + str(e))
            raise SocketError(__name__ + " Send Failed: " + str(e))

        time.sleep(sleeptime)
        return sentdata
        # Now read what was sent
        #try:
        #    self.sock.settimeout(1)
        #    rcvdata = self.sock.recv(2048)
        #    self.logger.debug("pushTCP Rcv Data:" + str(rcvdata))
        #except:
        #    self.logger.warning("push_tcp: Rcv data failed")
        #    return 0
        # Set a minimum delay so the server does not get overrun    
        #self.time.sleep(self.sleeptime) 


    # Get rid of any old messages, really should not be needed with unique UID's
    def flush(self, readtimeout=0.5):
        self.sock.settimeout(readtimeout)

        #response = 'start some reading'
        #Flush any pending server responses
        while True:
            #print("Read attempt: " + str(i))
            try:
                response = self.sock.recv(2048)
                #print("flushit response is:")
                #print(response)
                pass

            except:
                #print("flushit read empty")
                # Flushed, now return
                break
        return 0 

    def read(self, readattempts=5, readtimeout=0.5):
        self.sock.settimeout(readtimeout)

        response = ''
        for i in range(readattempts):
            # print("Read attempt: " + str(i))
            response = ''
            try:
                while True:
                    response += self.sock.recv(2049)

            except socket.timeout:
#                print("readit response is: ====================",len(response))
#                print(response)
                return response

            except KeyboardInterrupt:
                self.logger.debug("Kbd Interrupt during read")
                raise
            
            except:
                print(f"Takcot socket read failed: {str(e)}")
                print("Takcot socket read failed: %s" % (sys.exc_info()[0]))
                raise

    def readcot(self, readtimeout=10, frag=""):
        try:
            cotbuff = self.read(readtimeout=readtimeout, readattempts=1)
        except Exception as e:
            self.logger.error("Takcot socket read failed: %s" % str(e))
            return "", frag

        if cotbuff:
            try:
                cotbuff = frag + cotbuff.decode('utf-8')
            except Exception as e:
                self.logger.error("Failed to decode cotbuff: %s" % str(e))
                return "", frag
        else:
            cotbuff = frag

        cotbuff = cotbuff.replace("\n", "")

        try:
            cots = cotbuff.split("/event>", 1)
        except Exception as e:
            self.logger.error("Failed to split cotbuff: %s" % str(e))
            return "", frag

        if len(cots) > 1:
            cot_xml = cots[0] + "/event>"
            frag = cots[1]

            if cot_xml.startswith("<?xml") or cot_xml.startswith("<event"):
                return cot_xml, frag
            else:
                self.logger.warning(__name__ + " Not a valid CoT")
                self.logger.warning(cot_xml)
                return "", frag
        else:
            return "", cotbuff
        """# Read a buff
        #print("readcot passed frag= " + frag)
        #print("readtimeout= " + str(readtimeout))
        try:
             
            cotbuff = self.read(readtimeout=readtimeout,readattempts=1)
            #print("readcot successful read")
        except:
            # Nothing read in timeout
            #print("Nothing read in timeout2, but now process any frag")
            print("readcot failed: %s" % (sys.exc_info()[0]))
            #return "",frag

        # print("cotbuff length is: " + str(len(cotbuff)))
        # print("raw cotbuff:")
        # print(cotbuff)

        # OK, we read something, now prepend the frag and clean it up
        if cotbuff:
            #cotbuff = bytes(frag,'utf-8') + cotbuff
            cotbuff = frag + cotbuff.decode('utf-8')
        else:
            #cotbuff = bytes(frag,'utf-8')
            cotbuff = frag
            #print("No Data")

        # from here out cotbuff is a string

        cotbuff=cotbuff.replace("\n","")

        #print("cleaned cotbuff is:")
        #print(cotbuff)
        #print(type(cotbuff))
        #print()

        #cots="nothing"
        count = 1

        #while len(cots)>1:
        #print("cots length is: " + str(len(cots)))
        try:
            # split the buff using the closing event tag
            #print("splitting cotbuff")
            cots=cotbuff.split("/event>",1)

        except:
            # really should never get here
            print("cotbuff split failed")


        if len(cots) > 1:
            # OK, we have a cot it appears
            # The first part should be an xml hopefully
            cot_xml=cots[0] + "/event>"

            #print("function cot_xml: " + cot_xml)

            # The 2nd part is a frag if at all
            frag=cots[1]

            #print("function frag: " + frag)

            # FTS is not always sending the XML block with FTS version 1.0
            # so accept one beginning with the event block as valid
            if cot_xml.startswith("<?xml") or cot_xml.startswith("<event"):
                #print("Looks valid")
                #return cot_xml, frag
                return cot_xml, cots[1]
            else:
                # Must have had an incomplete cot fragment
                # Ignore the invalid CoT
                self.logger.warning(__name__ + " Not a valid CoT")
                self.logger.warning(cot_xml)
                return "", frag

        else:
            #print("Incomplete readcot:")
            #print(cotbuff)
            #print()
            #print(cots)
            #print("cots length " + str(len(cots)))
            # read buff did not have a "/event" in it, no Cot yet 
            cot_xml=""
            return "", cots[0]

        print("readcot Should never get here")

"""
