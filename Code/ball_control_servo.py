from concurrent.futures import process
import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import numpy as np


# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""
# Camera setup
camera_port = 0
cap = cv2.VideoCapture(camera_port)
cap.set(3, 1280)
cap.set(4, 720)
get, img = cap.read()
h, w, _ = img.shape

# Serial/arduino setup
port_id = '/dev/cu.usbmodem1411101'
# initialise serial interface
arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)

# Define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

# Maximum allowable rotation angle for the servos
servo_angle_limit = [[ -90, 90 ]    # Servo 1
                     [ -90, 90 ]    # Servo 2
                     [ -90, 90 ]]   # Servo 3

def ball_track(key1, coord_queue, image_queue):

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 0, 'smin': 65, 'vmin': 219, 'hmax': 179, 'smax': 255, 'vmax': 255}

    center_point = [626, 337, 2210] # center point of the plate, calibrated


    while True:
        get, img = cap.read()
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask)

        if countours:

            data = round((countours[0]['center'][0] - center_point[0]) / 10), \
                   round((h - countours[0]['center'][1] - center_point[1]) / 10), \
                   round(int(countours[0]['area'] - center_point[2])/100)

            coord_queue.put(data)
            #print("The got coordinates for the ball are :", data)
        else:
            data = 'nil' # returns nil if we cant find the ball
            coord_queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        # cv2.imshow("Image", imgStack)
        image_queue.put(imgStack)
        cv2.waitKey(1)


def servo_control(key2, coord_queue):
    if key2:
        print('Servo controls are initiated')


    def all_angle_assign(angle_passed1,angle_passed2,angle_passed3):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = np.radians(float(angle_passed1))
        servo2_angle = np.radians(float(angle_passed2))
        servo3_angle = np.radians(float(angle_passed3))
        write_servo()

    def in_range(val, limits):
        return limits[0] < val < limits[1]

    def writeCoord():
        """
        Here in this function we get both coordinate and servo control, it is an ideal place to implement the controller
        """
        coord_info = coord_queue.get()

        if coord_info == 'nil': # Checks if the output is nil
            print("Can't find the ball :(")
        else:
            print('The position of the ball : ', coord_info[2])

            if in_range(coord_info[0], servo_angle_limit[0]) and in_range(coord_info[1], servo_angle_limit[1]) and in_range(coord_info[2], servo_angle_limit[2]):
                all_angle_assign(coord_info[0],coord_info[1],coord_info[2])
            else:
                all_angle_assign(0,0,0)

    def write_arduino(data):
        print('The angles send to the arduino : ', data)

        arduino.write(bytes(data, 'utf-8'))

    def write_servo():
        ang1 = servo1_angle
        ang2 = servo2_angle
        ang3 = servo3_angle

        angles: tuple = (round(np.degrees(ang1), 1),
                         round(np.degrees(ang2), 1),
                         round(np.degrees(ang3), 1))

        write_arduino(str(angles))


    

    while key2:
        writeCoord()


def run_gui(image_queue):
    

if __name__ == '__main__':

    coord_queue = Queue() # Communication between the two processes - ball position coordinates
    image_queue = Queue() # Captured image with contour overlay
    key1 = 1 # just two dummy arguments passed for the processes
    key2 = 2
    processes = []
    processes[0] = mp.Process(target= ball_track, args=(key1, coord_queue)) # initiate ball tracking process
    processes[1] = mp.Process(target=servo_control,args=(key2, coord_queue)) # initiate servo controls
    processes[2] = mp.Process(target=run_gui,args=(image_queue)) # Initiate GUI
    
    for p in processes: # Start all processes
        p.start()
    for p in processes: # Wait for all processes to finish
        p.join()