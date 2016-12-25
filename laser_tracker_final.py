#! /usr/bin/env python
import argparse
import cv2
import sys
import time
import datetime
import imutils
from collections import deque
import numpy as np
import serial

ser = serial.Serial('COM18', 9600)    #initializing serial communication for Zigbee

cam_device = 1
laser = (0,0)
maxlen=10
pts = deque(maxlen=10)
detected = ""
radius= 0
circle_pos = (0,0)
boundary_x=(200,220)   #left and down
boundary_y=(400,180)   #right and up
laser_position = ""
X = ""
Y = ""
stop_trigger=False
send_trigger=""
border_trigger = False
x_border_trig = False
y_border_trig = False
center = None    

class LaserTracker(object):

    def __init__(self, cam_width=640, cam_height=480, hue_min=20, hue_max=160,
                 sat_min=100, sat_max=255, val_min=200, val_max=256,
                 display_thresholds=True):
        """
        * ``cam_width`` x ``cam_height`` -- This should be the size of the
        image coming from the camera. Default is 640x480.

        HSV color space Threshold values for a RED laser pointer are determined
        by:

        * ``hue_min``, ``hue_max`` -- Min/Max allowed Hue values
        * ``sat_min``, ``sat_max`` -- Min/Max allowed Saturation values
        * ``val_min``, ``val_max`` -- Min/Max allowed pixel values

        If the dot from the laser pointer doesn't fall within these values, it
        will be ignored.

        * ``display_thresholds`` -- if True, additional windows will display
          values for threshold image channels.

        """

        self.cam_width = cam_width
        self.cam_height = cam_height
        self.hue_min = hue_min
        self.hue_max = hue_max
        self.sat_min = sat_min
        self.sat_max = sat_max
        self.val_min = val_min
        self.val_max = val_max
        self.display_thresholds = display_thresholds

        self.capture = None  # camera capture device
        self.channels = {
            'hue': None,
            'saturation': None,
            'value': None,
            'laser': None,
        }

    def create_and_position_window(self, name, xpos, ypos):
        """Creates a named widow placing it on the screen at (xpos, ypos)."""
        # Create a window
        cv2.namedWindow(name)
        # Resize it to the size of the camera image
        cv2.resizeWindow(name, self.cam_width, self.cam_height)
        # Move to (xpos,ypos) on the screen
        cv2.moveWindow(name, xpos, ypos)

    def setup_camera_capture(self, device_num):
        """Perform camera setup for the device number (default device = 0).
        Returns a reference to the camera Capture object.

        """
        try:
            pass
            #device = int(device_num)
            #sys.stdout.write("Using Camera Device: {0}\n".format(device_num))
        except (IndexError, ValueError):
            # assume we want the 1st device
            device_num = ""
            #sys.stderr.write("Invalid Device. Using default device 0\n")

        # Try to start capturing frames
        self.capture = cv2.VideoCapture(device_num)
        if not self.capture.isOpened():
            sys.stderr.write("Faled to Open Capture device. Quitting.\n")
            sys.exit(1)

        # set the wanted image size from the camera
        self.capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.cam_width
        )
        self.capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.cam_height
        )
        return self.capture

    def handle_quit(self, delay=10):
        """Quit the program if the user presses "Esc" or "q"."""
        key = cv2.waitKey(delay)
        c = chr(key & 255)
        if c in ['q', 'Q', chr(27)]:
            sys.exit(0)

    def threshold_image(self, channel):
        if channel == "hue":
            minimum = self.hue_min
            maximum = self.hue_max
        elif channel == "saturation":
            minimum = self.sat_min
            maximum = self.sat_max
        elif channel == "value":
            minimum = self.val_min
            maximum = self.val_max

        (t, tmp) = cv2.threshold(
            self.channels[channel], # src
            maximum, # threshold value
            0, # we dont care because of the selected type
            cv2.THRESH_TOZERO_INV #t type
        )

        (t, self.channels[channel]) = cv2.threshold(
            tmp, # src
            minimum, # threshold value
            255, # maxvalue
            cv2.THRESH_BINARY # type
        )

        if channel == 'hue':
            # only works for filtering red color because the range for the hue is split
            self.channels['hue'] = cv2.bitwise_not(self.channels['hue'])


    def detect(self, frame):
        global laser
        global pts
        global detected
        global maxlen
        global radius
        global circle_pos
        global laser_position
        global boundary_x
        global boundary_y
        global X, Y
        global center
        global ser
        global stop_trigger, send_trigger
        global y_border_trig         
        global x_border_trig 
        global border_trigger
        #Point pt;
        #pt.x = 10;
        #pt.y = 8;
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


        # split the video frame into color channels
        h, s, v = cv2.split(hsv_img)
        self.channels['hue'] = h
        self.channels['saturation'] = s
        self.channels['value'] = v

        # Threshold ranges of HSV components; storing the results in place
        self.threshold_image("hue")
        self.threshold_image("saturation")
        self.threshold_image("value")

        # Perform an AND on HSV components to identify the laser!
        self.channels['laser'] = cv2.bitwise_and(
            self.channels['hue'],
            self.channels['value']
        )
        self.channels['laser'] = cv2.bitwise_and(
            self.channels['saturation'],
            self.channels['laser']
        )

        # Merge the HSV components back together.
        hsv_image = cv2.merge([self.channels['hue'],self.channels['saturation'],self.channels['value'],])
        
       
        (_, cnts, _) = cv2.findContours(self.channels['laser'].copy(), cv2.RETR_EXTERNAL,
         cv2.CHAIN_APPROX_SIMPLE)
        
#            
      

    	   # only proceed if at least one contour was found
        if len(cnts) > 0:
    		# find the largest contour in the mask, then use
    		# it to compute the minimum enclosing circle and
    		# centroid
    		# print len(cnts) 
    		
    		c = max(cnts, key=cv2.contourArea)
    		(circle_pos, radius) = cv2.minEnclosingCircle(c)
    		M = cv2.moments(c)
      
          #checking the size of laser pointer. 
    		if (M["m10"] != 0) or (M["m00"] != 0) or (M["m01"] != 0):  #find the centre of circle only if the laser pointer is big enough
    			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    			laser = center
		else:   #else if the laser is too small (far away), then don't find the centre of it
    		     laser = circle_pos
    		     #msg = "laser too far"
           
    		# only proceed if the radius meets a minimum size
    		if True:#radius > 5:
    			# draw the circle and centroid on the frame,
    			# then update the list of tracked points
    			cv2.circle(frame, (int(laser[0]), int(laser[1])), int(radius),
    				(0, 255, 255), 2)
    			cv2.circle(frame, center, 5, (0, 0, 255), -1) 
    			laser = center
    			detected = "Yes"
    			cv2.line(frame,(300,230),laser,(0,255,0),5)                
    		#else:
    			#detected = "No"
    			#area = ""

           #boundary_x=(left,down) 
           #boundary_y=(right,up) 
    		if laser != None and boundary_x != None:
    		  #stop_trigger = True 
    		  if x_border_trig == True or y_border_trig == True:
    			print 'outside border'
    			stop_trigger = False
    			ser.write( X + "-" + Y + "'\n")
    		  elif border_trigger == False or y_border_trig == False:
    			if stop_trigger == False:
    			   print 'inside border'
    			   ser.write( "---'\n")
    			   stop_trigger = True

       
    		  if laser[1] > boundary_x[1]: 
    			X = "D"
    			x_border_trig = True
    			cv2.line(frame,(300,230),laser,(150,0,255),5)        
       
    		  if laser[1] < boundary_y[1]:
    			X = "U"
    			x_border_trig = True
    			cv2.line(frame,(300,230),laser,(0,0,255),5) 
       
    		  if laser[1] > boundary_y[1] and laser[1] < boundary_x[1]: #inside square
    			X = "-"       
    			x_border_trig = False
        
       ################################################################
       
    		  if laser[0] >  boundary_x[0]:
    			Y = "-"             
    			y_border_trig = False 
       
    		  if laser[0] > boundary_y[0]:
    			Y = "R" 
    			y_border_trig = True
    			cv2.line(frame,(300,230),laser,(0,0,255),5) 
       
    		  if laser[0] < boundary_y[0] and laser[0] < boundary_x[0]:
    			Y = "L" 
    			y_border_trig = True 
       
       
#    		  else:  #laser pointer outside the square
#    			send_trigger  = "-"
#    			print "inside square"             
#    			if stop_trigger == True:
#    			   ser.write("S-S\n")
#    			   stop_trigger = False                       
        else:
           detected = "No"

           #laser_boundary= "-"
           
        laser_position = X + ":" + Y #+ ":" + L + ":" + R   
        #################################################   
#        pts.appendleft(center)
#        	# loop over the set of tracked points
#        for i in xrange(1, len(pts)):
#		# if either of the tracked points are None, ignore
#		# them
#		if pts[i - 1] is None or pts[i] is None:
#			continue
# 
#		# otherwise, compute the thickness of the line and
#		# draw the connecting lines
#		thickness = int(np.sqrt(maxlen / float(i + 1)) * 2.5)
#		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
#
        #################################################
        cv2.putText(frame, "contour area: {}".format(laser), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, "laser: {}".format(detected), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        #cv2.putText(frame, datetime.datetime.now().strftime("%A %d %B %Y %I:%M:%S%p"),(10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        cv2.putText(frame, "Radius: {}".format(radius), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.putText(frame, "{}".format(laser_position), (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # update the points queue
                             #left#down #right#up  
        cv2.rectangle(frame, boundary_x, boundary_y, 150,  thickness=3, lineType=3, shift=0)
        cv2.imshow('mask', self.channels['laser'])
        cv2.imshow('RGB_VideoFrame', frame)
        ################################################
        return hsv_image

    def display(self, img, frame):
        """Display the combined image and (optionally) all other image channels
        NOTE: default color space in OpenCV is BGR.
        """
        #cv2.imshow('RGB_VideoFrame', frame)
        #cv2.imshow('LaserPointer', self.channels['laser'])
        #if self.display_thresholds:
        #    cv2.imshow('Thresholded_HSV_Image', img)
        #    cv2.imshow('Hue', self.channels['hue'])
        #    cv2.imshow('Saturation', self.channels['saturation'])
        #    cv2.imshow('Value', self.channels['value'])


    def setup_windows(self):
        sys.stdout.write("Using OpenCV version: {0}\n".format(cv2.__version__))

        # create output windows
        self.create_and_position_window('LaserPointer', 0, 0)
        self.create_and_position_window('RGB_VideoFrame',
            10 + self.cam_width, 0)
        if self.display_thresholds:
            self.create_and_position_window('Thresholded_HSV_Image', 10, 10)
            self.create_and_position_window('Hue', 20, 20)
            self.create_and_position_window('Saturation', 30, 30)
            self.create_and_position_window('Value', 40, 40)


    def run(self):
        # Set up window positions
        #self.setup_windows()
        # Set up the camera capture
        global cam_device
        self.setup_camera_capture(cam_device)

        while True:
            # 1. capture the current image
            success, frame = self.capture.read()
            if not success: # no image captured... end the processing
                #sys.stderr.write("Could not read camera frame. Quitting\n")
                #sys.exit(1)
                #time.sleep(1)
                #self.setup_windows()
                self.setup_camera_capture(1)
                success, frame = self.capture.read()

            hsv_image = self.detect(frame)
            self.display(hsv_image, frame)
            self.handle_quit()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run the Laser Tracker')
    parser.add_argument('-W', '--width',
        default=640,
        type=int,
        help='Camera Width'
    )
    parser.add_argument('-H', '--height',
        default=480,
        type=int,
        help='Camera Height'
    )
    parser.add_argument('-u', '--huemin',
        default=20,
        type=int,
        help='Hue Minimum Threshold'
    )
    parser.add_argument('-U', '--huemax',
        default=160,
        type=int,
        help='Hue Maximum Threshold'
    )
    parser.add_argument('-s', '--satmin',
        default=100,
        type=int,
        help='Saturation Minimum Threshold'
    )
    parser.add_argument('-S', '--satmax',
        default=255,
        type=int,
        help='Saturation Maximum Threshold'
    )
    parser.add_argument('-v', '--valmin',
        default=200,
        type=int,
        help='Value Minimum Threshold'
    )
    parser.add_argument('-V', '--valmax',
        default=255,
        type=int,
        help='Value Maximum Threshold'
    )
    parser.add_argument('-d', '--display',
        action='store_true',
        help='Display Threshold Windows'
    )
    params = parser.parse_args()

    tracker = LaserTracker(
        cam_width=params.width,
        cam_height=params.height,
        hue_min=params.huemin,
        hue_max=params.huemax,
        sat_min=params.satmin,
        sat_max=params.satmax,
        val_min=params.valmin,
        val_max=params.valmax,
        display_thresholds=False
    )
    tracker.run()
