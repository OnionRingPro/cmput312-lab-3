#################################################################################################################################
### This file implements color specific sphere tracking. It can potentially be used for tracking recovery by object detection ###
#################################################################################################################################

#Requires refactoring. This is just a "proof of concept" implementation

import cv2
import numpy as np
import threading
import time

#####HSV Colour Ranges#################
#If the ball is red (0-10) or (170-180)
redLowMask = (0,50,50)
redHighMask = (10, 255, 255)

#If the ball is blue
blueLowMask = (100, 44, 90)
blueHighMask = (120, 255, 255)

#If the ball is orange
orangeLowMask = (5, 50, 50)
orangeHighMask = (20, 255, 255)

#If the ball is yellow
yellowLowMask = (30, 83, 141)
yellowHighMask = (38, 255, 255)

#If the ball is green
greenLowMask= (58, 67, 15)
greenHighMask= (78, 255, 255)
########################################

class Tracker:

    def __init__(self, pointColor, goalColor):
        self.point = (0,0,0)
        self.goal = (0,0,0)
        self.positions = None
        print("Starting tracker thread")
        thread = threading.Thread(target=self.TrackerThread, args=(pointColor, goalColor), daemon=True)
        thread.start()

    def TrackerThread(self, pointColor, goalColor):
        print("Tracker Started")
        # Get the camera
        vc = cv2.VideoCapture(0) # regular camera
        # if using firewire, 
        # vc = cv2.VideoCapture(0, cv2.CAP_DC1394)
        if vc.isOpened(): # try to get the first frame
            rval, original_frame = vc.read()
        else:
            rval = False

        if pointColor == 'g':
            p_hul, p_sal, p_val = greenLowMask
            p_huh, p_sah, p_vah = greenHighMask
            p_color_mask = (0, 255, 0)
        elif pointColor == 'y':
            p_hul, p_sal, p_val = yellowLowMask
            p_huh, p_sah, p_vah = yellowHighMask
            p_color_mask = (0, 255, 255)
        elif pointColor == 'b':
            p_hul, p_sal, p_val = blueLowMask
            p_huh, p_sah, p_vah = blueHighMask
            p_color_mask = (255, 0, 0)
        
        if goalColor == 'g':
            g_hul, g_sal, g_val = greenLowMask
            g_huh, g_sah, g_vah = greenHighMask
            g_color_mask = (0, 255, 0)
        elif goalColor == 'y':
            g_hul, g_sal, g_val = yellowLowMask
            g_huh, g_sah, g_vah = yellowHighMask
            g_color_mask = (0, 255, 255)
        elif goalColor == 'b':
            g_hul, g_sal, g_val = blueLowMask
            g_huh, g_sah, g_vah = blueHighMask
            g_color_mask = (255, 0, 0)

        while rval:
            # Handle current frame
            frame_started_at = time.monotonic()
            rval, original_frame = vc.read()
            if not rval:
                break

            frame = original_frame
            circlesPoint = self.GetLocation(frame, pointColor)
            circlesGoal = self.GetLocation(frame, goalColor)
            self.DrawCircles(frame, circlesPoint, (255, 0, 0))
            self.DrawCircles(frame, circlesGoal, (0, 0, 255))

            if circlesPoint is not None:
                self.point = circlesPoint[0, 0]
            
            if circlesGoal is not None:
                self.goal = circlesGoal[0, 0]

            # Publish both markers from the same frame, or mark it invalid.
            if circlesPoint is not None and circlesGoal is not None:
                self.positions = (tuple(self.point), tuple(self.goal), frame_started_at)
            else:
                self.positions = None

            # Shows the original image with the detected circles drawn.
            cv2.imshow("Result", frame)

            # Mask window:

            # make array for final values
            P_HSVLOW = np.array([p_hul, p_sal, p_val])
            P_HSVHIGH = np.array([p_huh, p_sah, p_vah])
            G_HSVLOW = np.array([g_hul, g_sal, g_val])
            G_HSVHIGH = np.array([g_huh, g_sah, g_vah])
            hsv = cv2.cvtColor(original_frame, cv2.COLOR_BGR2HSV)

            kernel = np.ones((5, 5), np.uint8)

            pmask = cv2.inRange(hsv, P_HSVLOW, P_HSVHIGH)
            pmask = cv2.erode(pmask, kernel, iterations=1)
            pmask = cv2.dilate(pmask, kernel, iterations=5)

            gmask = cv2.inRange(hsv, G_HSVLOW, G_HSVHIGH)
            gmask = cv2.erode(gmask, kernel, iterations=1)
            gmask = cv2.dilate(gmask, kernel, iterations=5)

            # combinedMask = cv2.bitwise_or(pmask, gmask)

            # Optional cleanup on the combined binary mask
            combinedMask = np.zeros_like(original_frame)
            combinedMask[pmask > 0] = p_color_mask
            combinedMask[gmask > 0] = g_color_mask
            cv2.imshow("Combined Mask", combinedMask)

            # check if esc key pressed
            key = cv2.waitKey(20)
            if key == 27:
                break
        
        self.positions = None
        vc.release()
        cv2.destroyAllWindows()
        print("Tracker Ended")

    def GetLocation(self, frame, color):
        # Uncomment for gaussian blur
        #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        blurred = cv2.medianBlur(frame,11)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == 'r':
            # Red Tracking
            mask = cv2.inRange(hsv, redLowMask, redHighMask)
        if color == 'o':
            # Orange Tracking
            mask = cv2.inRange(hsv, orangeLowMask, orangeHighMask)
        if color == 'y':
            # Yellow Tracking
            mask = cv2.inRange(hsv, yellowLowMask, yellowHighMask)
        if color == 'b':
            # Blue Tracking
            mask = cv2.inRange(hsv, blueLowMask, blueHighMask)
        if color == 'g':
            # Green Tracking
            mask = cv2.inRange(hsv, greenLowMask, greenHighMask)
        # Perform erosion and dilation in the image (in 11x11 pixels squares) in order to reduce the "blips" on the mask
        mask = cv2.erode(mask, np.ones((5, 5),np.uint8), iterations=1)
        mask = cv2.dilate(mask, np.ones((5, 5),np.uint8), iterations=5)
        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv2.bitwise_and(blurred,blurred, mask= mask)
        # masked_blurred = cv2.bitwise_and(frame,frame, mask= mask)
        # Convert the masked image to gray scale (Required by HoughCircles routine)
        result = cv2.cvtColor(masked_blurred, cv2.COLOR_BGR2GRAY)

        # Detect circles in the image using Canny edge and Hough transform
        circles = cv2.HoughCircles(result, cv2.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=10, minRadius=20, maxRadius=45)
        return circles
            
    def DrawCircles(self, frame, circles, dotColor):
        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                #print("Circle: " + "("+str(x)+","+str(y)+")")
                # draw the circle in the output image, then draw a rectangle corresponding to the center of the circle
                # The circles and rectangles are drawn on the original image.
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), dotColor, -1)

    def get_tracker_point(self):
        return self.point

    def get_tracker_goal(self):
        return self.goal

    def get_tracker_positions(self, after=None):
        """Return valid markers captured after the cutoff and under one second old."""
        positions = self.positions
        if (positions is None or time.monotonic() - positions[2] > 1.0
                or (after is not None and positions[2] <= after)):
            return None
        return positions[:2]
        


def setup_tracker():
    global tracker
    print("Tracker Setup")

    # Green is the end effector, blue is the goal
    tracker = Tracker('g', 'b')
    print("Moving on")
    return tracker
