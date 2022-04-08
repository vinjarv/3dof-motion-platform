import serial
import math
from tkinter import *


"""
This code can be used for controlling servos by using a simple controller. 
"""

#define the port of the arduino. each system may have its own port id
#NOTE: This can be found from going to arduino ide --> Tools --> Ports
port_id = '/dev/cu.usbmodem1411101'

#initialise serial interface
arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)

#define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
servo1_angle_limit_positive = 90
servo1_angle_limit_negative = -90

servo2_angle_limit_positive = 90
servo2_angle_limit_negative = -90

servo3_angle_limit_positive = 90
servo3_angle_limit_negative = -90


# -------------------------------------------angle assignment functions-------------------------------------------
"""
This functions assign angles to the respective servos and call write_servo() function to create a tuple to pass to the arduino.
 
I have put three of this angle assignment separately to make it easy to understand.
"""


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



root = Tk()
root.resizable(0,0)

# ------------------------------------------- Simple controller GUI definition-------------------------------------------

w2 = Label(root, justify= LEFT, text="Simple Servo Controller")
w2.config(font=("Elephant",30))
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

servo1=Scale(root, from_=servo1_angle_limit_negative, to=servo1_angle_limit_positive, orient=HORIZONTAL,resolution=0.1,length = 400, command =servo1_angle_assign)
servo1.grid(row=5, column = 1)

servo2=Scale(root, from_=servo1_angle_limit_negative, to=servo1_angle_limit_positive, orient=HORIZONTAL,resolution=0.1,length = 400, command =servo2_angle_assign)
servo2.grid(row=10, column = 1)

servo3=Scale(root, from_=servo1_angle_limit_negative, to=servo1_angle_limit_positive, orient=HORIZONTAL,resolution=0.1,length = 400, command =servo3_angle_assign)
servo3.grid(row=15, column = 1)

def write_arduino(data):
    print('The angles send to the arduino : ',data)
    arduino.write(bytes(data, 'utf-8'))

# write function that will write the angles into a single tuple and call write_arduino function to pass it to arduino
def write_servo():
    ang1 = servo1_angle
    ang2 = servo2_angle
    ang3 = servo3_angle

    angles: tuple = (round(math.degrees(ang1), 1),
                     round(math.degrees(ang2), 1),
                     round(math.degrees(ang3), 1))

    write_arduino(str(angles))
root.mainloop() # running loop