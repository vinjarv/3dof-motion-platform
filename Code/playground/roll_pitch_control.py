import serial
import math
from tkinter import *
import numpy as np
from numpy import pi, cos, sin, arcsin, sqrt


"""
This code can be used for controlling servos by using a simple controller. 
"""

#define the port of the arduino. each system may have its own port id
#NOTE: This can be found from going to arduino ide --> Tools --> Ports
port_id = 'COM5'

#initialise serial interface
arduino = serial.Serial(port=port_id, baudrate=250000, timeout=0.1)

#define servo angles and set a value
servo1_angle = 0
servo2_angle = 0
servo3_angle = 0

# Set a limit to upto which you want to rotate the servos (You can do it according to your needs)
angle_min = -15
angle_max = 15

def constrain(lower, val, upper):
    return max(lower, max(val, upper))

def ikine(p, r):
    R = 40
    L = 225
    p = p * pi/180
    r = r * pi/180
    z = np.array([  sqrt(3)*L/6 * sin(p)*cos(r) - L/2*sin(r),
                    sqrt(3)*L/6 * sin(p)*cos(r) + L/2*sin(r),
                   -sqrt(3)*L/6 * sin(p)*cos(r)])
    zR = np.zeros(3)
    for i in range(3):
        zR[i] = constrain(-1, z[i]/R, 1)
    Va = arcsin(z/R) * 180/pi
    return Va

# -------------------------------------------angle assignment functions-------------------------------------------
"""
This functions assign angles to the respective servos and call write_servo() function to create a tuple to pass to the arduino.
 
I have put three of this angle assignment separately to make it easy to understand.
"""


def angle_assign(_):
    global servo1_angle, servo2_angle, servo3_angle
    global roll, pitch
    r = roll.get()
    p = pitch.get()
    Va = ikine(p, r)
    servo1_angle = Va[0]
    servo2_angle = Va[1]
    servo3_angle = Va[2]
    write_servo()




root = Tk()
root.resizable(0,0)

# ------------------------------------------- Simple controller GUI definition-------------------------------------------

w2 = Label(root, justify= LEFT, text="Simple Servo Controller")
w2.config(font=("Elephant",30))
w2.grid(row=3, column=0, columnspan=2, padx=100)

RLb = Label(root, text="Roll Angle")
RLb.config(font=("Elephant", 15))
RLb.grid(row=5, column=0, pady=10)

PLb = Label(root, text="Pitch Angle")
PLb.config(font=("Elephant", 15))
PLb.grid(row=10, column=0, pady=10)

roll=Scale(root, from_=angle_min, to=angle_max, orient=HORIZONTAL,resolution=0.05,length = 400, command =angle_assign)
roll.grid(row=5, column = 1)

pitch=Scale(root, from_=angle_min, to=angle_max, orient=HORIZONTAL,resolution=0.05,length = 400, command =angle_assign)
pitch.grid(row=10, column = 1)


def write_arduino(data):
    print('The angles send to the arduino : ',data)
    arduino.write(bytes(data, 'utf-8'))

# write function that will write the angles into a single tuple and call write_arduino function to pass it to arduino
def write_servo():
    ang1 = -servo1_angle # Negative, as all servos are reversed from the mathematics
    ang2 = -servo2_angle
    ang3 = -servo3_angle

    angles: tuple = (round(ang1, 1),
                     round(ang2, 1),
                     round(ang3, 1))

    write_arduino(str(angles))
root.mainloop() # running loop