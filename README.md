# 3dof-motion-platform

1.	simple_ball_tracker.py : This code just tracks the ball and gives the coordinates.
2.	simple_servo_control.py : This code provides a GUI that can be used as a controller for the servos. The code initiates a serial communication with the servos with angle input. 
3.	two_in_one.py : This code combines both the tracker and controller by using multiprocessing. 
4.	ball_control_servo.py : This code does a simple mapping between the tracker and servos, like I just put x,y,z coordinate of the balls as input to angles on the servos, so x-coordinate control servo 1, y-coordinate control servo 2 and z-control servo 3. 

Python setup:

Run in shell
```console
python -m venv Code\venv
Code\venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```