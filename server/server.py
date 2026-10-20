#!/usr/bin/python
# RUN ON LAPTOP USING PYTHON 3.6

import socket
import time
from queue import Queue
from server.camera_helpers.color_tracking import setup_tracker, get_tracker_point

# This class handles the Server side of the comunication between the laptop and the brick.
class Server:
    def __init__(self, host, port):
       # setup server socket
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        # We need to use the ip address that shows up in ipconfig for the usb ethernet adapter that handles the comunication between the PC and the brick
        print("Setting up Server\nAddress: " + host + "\nPort: " + str(port))
        
        self.serversocket.bind((host, port))
        # queue up to 5 requests
        self.serversocket.listen(5)
        self.wait_for_new_conn()


    # Sends set of angles to the brick via TCP.
    # Input: base_angle [Float]: The angle by which we want the base to move
    #        joint_angle [Float]: The angle by which we want to joint to move
    #        queue [Thread-safe Queue]: Mutable data structure to store (and return) the messages received from the client
    def sendAngles(self, base_angle, joint_angle):
        # Format in which the client expects the data: "angle1,angle2"
        data = str(base_angle) + "," + str(joint_angle)
        self.sendData(data)
        arm_is_moving = True

    def sendData(self, data: str):
        print("Sending Data: (" + data + ") to robot.")
        self.cs.send(data.encode("UTF-8"))

    def waitForClientRequest(self, queue):
        reply = self.cs.recv(128).decode("UTF-8")
        if not reply:
            print("Closing client...")
            self.cs.close()
            print("Client closed")
            return False
        queue.put(reply)
        return True

    # Sends a termination message to the client. This will cause the client to exit "cleanly", after stopping the motors.
    def sendTermination(self):
        self.cs.send("EXIT".encode("UTF-8"))

    # Lets the client know that it should enable safety mode on its end
    def sendEnableSafetyMode(self):
        self.cs.send("SAFETY_ON".encode("UTF-8"))
    
    # Lets the client know that it should disable safety mode on its end
    def sendDisableSafetyMode(self):
        self.cs.send("SAFETY_OFF".encode("UTF-8"))

    def wait_for_new_conn(self):
        self.cs, addr = self.serversocket.accept()
        print ("Connected to: " + str(addr))
        self.arm_is_moving = False

# Set up server
host = "192.168.0.2"
port = 9999
server = Server(host, port)
queue = Queue()

# Set up color tracking
setup_tracker()

def get_server():
    return server

def send_angles(theta1: float, theta2: float):
    server.sendAngles(theta1, theta2)

def get_tracker_pos():
    pos = get_tracker_point()
    return f"TrackerPos:{pos[0]},{pos[1]},{pos[2]}"

def main():
    # Continuously wait for a client to send a message
    while (True):
        client_still_open = server.waitForClientRequest(queue)

        # If the client closes, empty the queue and wait for an incoming client request
        if (not client_still_open):
            while queue.qsize() > 0:
                queue.get_nowait()
            server.wait_for_new_conn()

        # If the client does not close, read the incoming message
        elif (queue.qsize() > 0):
            request_item = queue.get()
            if (request_item == "GET_EEPOS_CAM"):
                server.sendData(get_tracker_pos())

if __name__ == "__main__":
    main()