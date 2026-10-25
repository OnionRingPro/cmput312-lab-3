#!/usr/bin/python3       
# RUN ON BRICK
    
import socket
import os
import time
import math

# This class handles the client side of communication. It has a set of predefined messages to send to the server as well as functionality to poll and decode data.
class Client:
    def __init__(self, host, port):
        # We need to use the ipv4 address that shows up in ipconfig in the computer for the USB. Ethernet adapter handling the connection to the EV3
        print("Setting up client\nAddress: " + host + "\nPort: " + str(port))
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.s.connect((host, port))                               
        
    # Block until a message from the server is received. When the message is received it will be decoded and returned as a string.
    # Output: UTF-8 decoded string containing the instructions from server.
    def pollData(self):
        print("Waiting for Data")
        data = self.s.recv(128).decode("UTF-8")
        print("Data Received")
        return data
    
    # Sends a message to the server letting it know that the movement of the motors was executed without any inconvenience.
    def sendDone(self):
        self.s.send("DONE".encode("UTF-8"))

    # Sends a message to the server letting it know that there was an isse during the execution of the movement (obstacle avoided) and that the initial jacobian should be recomputed (Visual servoing started from scratch)
    def sendReset(self):
        self.s.send("RESET".encode("UTF-8"))

    def sendCommand(self, command: str):
        self.s.send(command.encode("UTF-8"))

    def request_end_effector_pos(self):
        # pos[0]=x, pos[1]=y, pos[2]=circle radius
        self.sendCommand(command="GET_EEPOS_CAM")
        end_effector_pos_str = self.pollData()
        end_effector_pos = (float(s) for s in end_effector_pos_str.split(","))
        return end_effector_pos

    def request_positions(self):
        """Return ((ee_u, ee_v), (target_u, target_v)), or None if invalid."""
        self.sendCommand("GET_POSITIONS_CAM")
        # A TCP reply can arrive in several pieces; read through its newline.
        data = b""
        while not data.endswith(b"\n"):
            chunk = self.s.recv(128)
            if not chunk:
                raise ConnectionError("Camera server disconnected.")
            data += chunk
        reply = data.decode("UTF-8").strip()
        if reply == "NONE":
            return None
        values = tuple(float(value) for value in reply.split(","))
        if (len(values) != 6 or not all(math.isfinite(value) for value in values)
                or values[2] <= 0 or values[5] <= 0):
            return None
        return values[:2], values[3:5]

client = None

def setup_client():
    global client
    host = "192.168.0.2"
    port = 9999
    client = Client(host, port)
    return client

# while True:
#     # The robot should receive new angle data
#     print(client.pollData())
#     time.sleep(1)
#     client.sendDone()

# Left for backwards compat but please use class methods instead
def movement_is_done():
    assert client is not None, "Client is not set up"
    client.sendDone()

def reset_jacobian():
    assert client is not None, "Client is not set up"
    client.sendReset()

def request_end_effector_pos():
    assert client is not None, "Client is not set up"
    client.request_end_effector_pos()
