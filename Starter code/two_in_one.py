
import multiprocessing as mp
from multiprocessing import Queue
import cvzone
from cvzone.ColorModule import ColorFinder
import cv2
import serial
import math
from tkinter import *

# -------------------------------------------Both programs(Servo Control and Ball Tracker) in one -------------------------------------------
"""
For running both programs simultaneously we can use multithreading or multiprocessing
"""

camera_port = 0
cap = cv2.VideoCapture(0)
cap.set(3, 1280)
cap.set(4, 720)

get, img = cap.read()
h, w, _ = img.shape


port_id = '/dev/cu.usbmodem1411101'
# initialise serial interface
arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)

# define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0
all_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = 90
servo1_angle_limit_negative = -90

servo2_angle_limit_positive = 90
servo2_angle_limit_negative = -90

servo3_angle_limit_positive = 90
servo3_angle_limit_negative = -90


def ball_track(key1, queue):

    if key1:
        print('Ball tracking is initiated')

    myColorFinder = ColorFinder(False)  # if you want to find the color and calibrate the program we use this *(Debugging)
    hsvVals = {'hmin': 0, 'smin': 65, 'vmin': 219, 'hmax': 179, 'smax': 255, 'vmax': 255}

    center_point = [626, 337, 2210]

    while True:
        get, img = cap.read()
        imgColor, mask = myColorFinder.update(img, hsvVals)
        imgContour, countours = cvzone.findContours(img, mask)

        if countours:

            data = round((countours[0]['center'][0] - center_point[0]) / 10), \
                   round((h - countours[0]['center'][1] - center_point[1]) / 10), \
                   int(countours[0]['area'] - center_point[2])

            queue.put(data)
            #print("The got coordinates for the ball are :", data)
        else:
            data = 'nil'
            queue.put(data)

        imgStack = cvzone.stackImages([imgContour], 1, 1)
        # imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
        cv2.imshow("Image", imgStack)
        cv2.waitKey(1)


def servo_control(key2, queue):
    if key2:
        print('Servo controls are initiated')


    def servo1_angle_assign(servo1_angle_passed):
        global servo1_angle
        servo1_angle = math.radians(float(servo1_angle_passed))
        write_servo()

    def servo2_angle_assign(servo2_angle_passed):
        global servo2_angle
        servo2_angle = math.radians(float(servo2_angle_passed))
        write_servo()

    def servo3_angle_assign(servo3_angle_passed):
        global servo3_angle
        servo3_angle = math.radians(float(servo3_angle_passed))
        write_servo()

    def all_angle_assign(all_angle_passed):
        global servo1_angle, servo2_angle, servo3_angle
        servo1_angle = math.radians(float(all_angle_passed))
        servo2_angle = math.radians(float(all_angle_passed))
        servo3_angle = math.radians(float(all_angle_passed))
        write_servo()

    root = Tk()
    root.resizable(0, 0)

    w2 = Label(root, justify=LEFT, text="Simple Servo Controller")
    w2.config(font=("Elephant", 30))
    w2.grid(row=3, column=0, columnspan=2, padx=100)

    S1Lb = Label(root, text="Servo 1 Angle")
    S1Lb.config(font=("Elephant", 15))
    S1Lb.grid(row=5, column=0, pady=10)

    S2Lb = Label(root, text="Servo 2 Angle")
    S2Lb.config(font=("Elephant", 15))
    S2Lb.grid(row=10, column=0, pady=10)

    S3Lb = Label(root, text="Servo 3 Angle")
    S3Lb.config(font=("Elephant", 15))
    S3Lb.grid(row=15, column=0, pady=10)

    S4Lb = Label(root, text="All Servos at once")
    S4Lb.config(font=("Elephant", 15))
    S4Lb.grid(row=20, column=0, pady=10)

    servo1 = Scale(root, from_=servo1_angle_limit_negative, to=servo1_angle_limit_positive, orient=HORIZONTAL,
                   resolution=1.0, length=400, command=servo1_angle_assign)
    servo1.grid(row=5, column=1)

    servo2 = Scale(root, from_=servo1_angle_limit_negative, to=servo1_angle_limit_positive, orient=HORIZONTAL,
                   resolution=1.0, length=400, command=servo2_angle_assign)
    servo2.grid(row=10, column=1)

    servo3 = Scale(root, from_=servo1_angle_limit_negative, to=servo1_angle_limit_positive, orient=HORIZONTAL,
                   resolution=1.0, length=400, command=servo3_angle_assign)
    servo3.grid(row=15, column=1)

    all_at_once = Scale(root, from_=servo1_angle_limit_negative, to=servo1_angle_limit_positive, orient=HORIZONTAL,
                   resolution=1.0, length=400, command=all_angle_assign)
    all_at_once.grid(row=20, column=1)

    def write_arduino(data):
        print('The angles send to the arduino : ', data)
        print('The position of the ball : ', queue.get())
        arduino.write(bytes(data, 'utf-8'))

    def write_servo():
        ang1 = servo1_angle
        ang2 = servo2_angle
        ang3 = servo3_angle

        angles: tuple = (round(math.degrees(ang1), 1),
                         round(math.degrees(ang2), 1),
                         round(math.degrees(ang3), 1))

        write_arduino(str(angles))

    root.mainloop()  # running loop

if __name__ == '__main__':

    queue = Queue() # The queue is done inorder for the communication between the two processes.
    key1 = 1 # just two dummy arguments passed for the processes
    key2 = 2
    p1 = mp.Process(target= ball_track, args=(key1, queue)) # initiate ball tracking process
    p2 = mp.Process(target=servo_control,args=(key2, queue)) # initiate servo controls
    p1.start()
    p2.start()
    p1.join()
    p2.join()