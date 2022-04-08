import cvzone
from cvzone.ColorModule import ColorFinder
import cv2

"""
This code can be used for computer vision based ball tracking. 
"""


camera_port = 0  # this is the camera port number (This can vary from, 0 - 10 from pc to pc)
cap = cv2.VideoCapture(camera_port) # Set the camera capture device

# set width and height of the camera capture area
cap.set(3,1280)
cap.set(4, 720)

get, img = cap.read()
h,w,_ = img.shape

myColorFinder = ColorFinder(False) # if you want to find the color and calibrate the program we use this *(Debugging)
hsvVals = {'hmin': 0, 'smin': 65, 'vmin': 219, 'hmax': 179, 'smax': 255, 'vmax': 255}  # this is hsv values for orange color


center_point = [626,337,2210] # this center point is found by placing the ball at the center of the plate and calibrating it.

while True:
    get, img = cap.read()
    imgColor, mask = myColorFinder.update(img,hsvVals)
    imgContour, countours = cvzone.findContours(img,mask)

    if countours:
        data = round((countours[0]['center'][0] - center_point[0])/10), \
               round((h-countours[0]['center'][1]  - center_point[1])/10),\
               round(int(countours[0]['area'] - center_point[2])/1000)

        print("The got coordinates for the ball are :",data)

    imgStack = cvzone.stackImages([imgContour], 1, 1)
    #imgStack = cvzone.stackImages([img,imgColor, mask, imgContour],2,0.5) #use for calibration and correction
    cv2.imshow("Image", imgStack)
    cv2.waitKey(1)
