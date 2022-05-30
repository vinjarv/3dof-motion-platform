# 3dof-motion-platform

This project was a part of the subject AIS2102 Dynamical Systems at NTNU Ålesund, Spring 2022. The task given was to control the position of a ball on a platform. This included building the platform, designing a regulator, and implementing it in code.

## Folders:
- Code - Arduino and python code for running the project
- Code/playground - test code for various functions
- Matlab - files for generation regulator parameters and simulating the response
- Parts for 3D printing - STL files used in the project
- Parts for laser cutting - DXF files
- Starter code - supplied code examples from project 

## Python programs:
1.	Code/config_gui.py : GUI for selecting platform position and ball color mask
2.	Code/ball_control_servo_fast.py : Controls ball position on platform. Main project code


## Python setup / installation:

Navigate to folder and run in shell:
```console
cd Code
python -m venv venv
venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```