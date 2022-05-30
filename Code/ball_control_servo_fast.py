import configparser
import time
from multiprocessing import Process
import threading

import cv2
import cvzone
import numpy as np
import serial
from numpy import arcsin, cos, pi, sin, sqrt

# Serial/arduino setup
port_id = 'COM5'

# Define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0

# Camera
# IDs should be from 0 and up. Internal webcam is probably 0, USB webcam will then be 1
cap_id = 1
cam_exposure = -8 # Set as low as possible to get 30fps
IMG_W = 1280
IMG_H = 720

x, y = 0, 0 # Mouse click event coords
plat_c, plat_r = [0,0], 0 # Platform definition
col_mask = {'hmin': 0, 'smin': 0, 'vmin': 0, 'hmax': 255, 'smax': 255, 'vmax': 255}

plat_actual_r = 0.175 # Actual radius in m

# Regulator parameters
reg_b = [4.8521423282,0.1963758629,-4.6557664653]
reg_a = [1.0000000000,-0.5030443975,0.1419604340]

# Config file handling
config_file = configparser.ConfigParser()
config_file.read("config.ini")

def save_configfile():
    # Write parameters to config file
    global config_file, plat_c, plat_r, col_mask
    with open("config.ini","w") as file_object:
        for key in col_mask:
            config_file["ColMask"][key] = str(col_mask[key])
        config_file["Platform"]["plat_c_x"] = str(plat_c[0])
        config_file["Platform"]["plat_c_y"] = str(plat_c[1])
        config_file["Platform"]["plat_r"] = str(plat_r)
        config_file.write(file_object)

def read_configfile():
    # Get parameters from config file
    global config_file, plat_c, plat_r, col_mask
    for key in col_mask:
        col_mask[key] = int(config_file["ColMask"][key])
    plat_c[0] = float(config_file["Platform"]["plat_c_x"])
    plat_c[1] = float(config_file["Platform"]["plat_c_y"])
    plat_r = float(config_file["Platform"]["plat_r"])
    print("Settings loaded from config.ini")

class FreshestFrame(threading.Thread):
    # From https://gist.github.com/crackwitz/15c3910f243a42dcd9d4a40fcdb24e40
    # Ensure returned image frame is the most recent
	def __init__(self, capture, name='FreshestFrame'):
		self.capture = capture
		assert self.capture.isOpened()

		# this lets the read() method block until there's a new frame
		self.cond = threading.Condition()

		# this allows us to stop the thread gracefully
		self.running = False

		# keeping the newest frame around
		self.frame = None

		# passing a sequence number allows read() to NOT block
		# if the currently available one is exactly the one you ask for
		self.latestnum = 0

		# this is just for demo purposes		
		self.callback = None
		
		super().__init__(name=name)
		self.start()

	def start(self):
		self.running = True
		super().start()

	def release(self, timeout=None):
		self.running = False
		self.join(timeout=timeout)
		self.capture.release()

	def run(self):
		counter = 0
		while self.running:
			# block for fresh frame
			(rv, img) = self.capture.read()
			assert rv
			counter += 1

			# publish the frame
			with self.cond: # lock the condition for this operation
				self.frame = img if rv else None
				self.latestnum = counter
				self.cond.notify_all()

			if self.callback:
				self.callback(img)

	def read(self, wait=True, seqnumber=None, timeout=None):
		# with no arguments (wait=True), it always blocks for a fresh frame
		# with wait=False it returns the current frame immediately (polling)
		# with a seqnumber, it blocks until that frame is available (or no wait at all)
		# with timeout argument, may return an earlier frame;
		#   may even be (0,None) if nothing received yet

		with self.cond:
			if wait:
				if seqnumber is None:
					seqnumber = self.latestnum+1
				if seqnumber < 1:
					seqnumber = 1
				
				rv = self.cond.wait_for(lambda: self.latestnum >= seqnumber, timeout=timeout)
				if not rv:
					return (self.latestnum, self.frame)

			return (self.latestnum, self.frame)

class ObserverControllerFilter():
    # IIR filter
    def __init__(self, b, a):
        self.a = np.array(a)
        self.b = np.array(b)
        self.x_prev = np.zeros(len(b)-1)
        self.y_prev = np.zeros(len(b)-1)

    def run(self, x):
        # Calculate filter step
        y = self.b[0] * x
        for i in range(len(self.x_prev)):
            y += self.b[i+1] * self.x_prev[i]
            y -= self.a[i+1] * self.y_prev[i]
        y /= self.a[0]
        self.x_prev = np.roll(self.x_prev, 1)
        self.x_prev[0] = x
        self.y_prev = np.roll(self.y_prev, 1)
        self.y_prev[0] = y
        return y

class ServoControl(Process):
    def __init__(self, serial_port, cap_id, baudrate="250000", timeout=0.1):
        # Platform kinematics parameters
        self.platform_R = 0.040
        self.platform_L = 0.225
        global reg_b, reg_a, col_mask
        # Color mask
        self.col_mask = col_mask
        # Mask out area outside platform circle
        self.platform_mask = np.zeros((IMG_H, IMG_W), np.uint8)
        self.platform_mask = cv2.circle(self.platform_mask, center=(round(plat_c[0]), round(plat_c[1])), radius=round(plat_r * 0.9), thickness=-1, color=255)
        # Regulators
        self.xctrl = ObserverControllerFilter(b=reg_b, a=reg_a)
        self.yctrl = ObserverControllerFilter(b=reg_b, a=reg_a)

        self.cap_id = cap_id
        # Begin
        super(ServoControl, self).__init__(target=self.arduino_init, args=(serial_port, baudrate, timeout))

    def arduino_init(self, serial_port, baudrate, timeout):
        # Initialize serial interface
        self.arduino = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)
        
        # Camera capture
        self.cap = cv2.VideoCapture(self.cap_id, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_EXPOSURE, cam_exposure)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)

        # Class to get newest possible camera frame
        self.fresh = FreshestFrame(self.cap)
        # Begin servo control loop
        self.loop()

    def get_coord(self):
        # Get current position of ball from webcam
        _, img = self.fresh.read()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        min = np.array([self.col_mask["hmin"], self.col_mask["smin"], self.col_mask["vmin"]])
        max = np.array([self.col_mask["hmax"], self.col_mask["smax"], self.col_mask["vmax"]])
        mask = cv2.inRange(hsv, min, max)
        mask = cv2.bitwise_and(mask, self.platform_mask)
        res = cv2.bitwise_and(img, img, mask=mask)
        _, contours = cvzone.findContours(img, mask)
        ball_pos_abs = (0,0)
        if contours:
            if contours[0]['area'] > 50:
                # Ball found 
                ball_pos_abs = ((contours[0]['center'][0]), (contours[0]['center'][1]))
        return ball_pos_abs

    def inv_kine(self, p, r):
        # Calculate servo angles from roll and pitch angles
        L = self.platform_L
        R = self.platform_R
        z = np.array([  sqrt(3)*L/6 * sin(p)*cos(r) - L/2*sin(r),
                        sqrt(3)*L/6 * sin(p)*cos(r) + L/2*sin(r),
                        -sqrt(3)*L/3 * sin(p)*cos(r)])
        # Constrain input to arcsin function (must be in [-1, 1])
        zR = np.zeros(3)
        for i in range(3):
            zR[i] = self.constrain(-1, z[i]/R, 1)
        Va = arcsin(zR) * 180/pi # Radians to degrees
        return Va

    def constrain(self, lower, val, upper):
        # Limit val between lower and upper
        return max(lower, min(val, upper))

    def write_angles(self, ang1, ang2, ang3):
        # Write servo angles to arduino over the serial port
        angles: tuple = (round(-ang1, 1), # Flip angles as servo rotations are reversed
                         round(-ang2, 1),
                         round(-ang3, 1))
        self.arduino.write(bytes(str(angles), "utf-8"))
    
    def loop(self):
        # Regulator main loop
        print("ServoControl started")
        global plat_c, plat_r
        i = 0
        t0 = time.time()
        tim = 0
        coord_info = (0,0)
        while True:
            coord_info = self.get_coord()
            tim += time.time() - t0
            i += 1
            t0 = time.time()
            if i >= 100:
                print("AVG. FPS:", i / tim)
                tim = 0
                i = 0
            if coord_info == (0,0):
                print("Ball not found")
                pass
            else:
                t = time.time()
                pr = 0.08 * np.array([sin(2*pi*0.4*t), sin(2*pi*0.4*t)*cos(2*pi*0.4*t)])
                #pr = [0, 0]
                #pr = [0.05*(1-2*(t%10<2.5 + t%10>7.5)), 0.05*(1-2*(2.5 < t%10 < 7.5))]

                global plat_actual_r
                pos_m = np.array([coord_info[0]-plat_c[0], coord_info[1]-plat_c[1]]) / plat_r * plat_actual_r # Convert pixels to meters
                # Calculate desired angles
                p = self.xctrl.run(pr[0] - pos_m[1])
                r = self.yctrl.run(pr[1] - pos_m[0])
                print("P:{%.2f} R:{%.2f}"%(p*180/pi, r*180/pi))
                # Constrain
                angle_max = 12.5 * pi/180
                p = max(-angle_max, min(angle_max, p))
                r = max(-angle_max, min(angle_max, r))
                Va = self.inv_kine(p, r)
                self.write_angles(Va[0], Va[1], Va[2])


read_configfile()

if __name__ == "__main__":
    p1 = ServoControl(port_id, cap_id)
    p1.start()

    input()
    p1.kill()