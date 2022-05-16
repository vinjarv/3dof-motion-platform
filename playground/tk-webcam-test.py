from glob import glob
from hashlib import sha1
import tkinter as tk
from turtle import color
from PIL import Image, ImageTk
import cvzone
import cv2
import time
import numpy as np

IMG_W = 1280
IMG_H = 720

root = tk.Tk()
rcanvas = tk.Canvas(root, width=IMG_W, height=IMG_H, borderwidth=0, highlightthickness=0)

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)

x, y = 0, 0 # Mouse click event coords
plat_c, plat_r = [0,0], 0 # Platform definition

col_mask = {'hmin': 0, 'smin': 0, 'vmin': 0, 'hmax': 255, 'smax': 255, 'vmax': 255}

def getImage():
    # Fetch camera frame, return both cv2 and tkinter images
    retval, img  = cap.read()
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(img_rgb)
    img_tk = ImageTk.PhotoImage(image=img_pil)
    return img, img_tk

def cv2_to_tk(img):
    # Convert cv2 image format to tkinter
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_pil = Image.fromarray(img_rgb)
    img_tk = ImageTk.PhotoImage(image=img_pil)
    return img_tk

def mask_img(img, col_mask):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    min = np.array([col_mask["hmin"], col_mask["smin"], col_mask["vmin"]])
    max = np.array([col_mask["hmax"], col_mask["smax"], col_mask["vmax"]])
    mask = cv2.inRange(hsv, min, max)
    res = cv2.bitwise_and(img, img, mask=mask)
    return mask, res

def imagePopup():
    # Show still image in new window
    popup = tk.Toplevel()
    popup.wm_title("Image")

    cv2Img, tkImg  = getImage()
    lbl = tk.Label(popup, image=tkImg)
    lbl.pack()
    while popup.winfo_exists():
        popup.update_idletasks()
        popup.update()
        time.sleep(0.01)

def center_calib_popup():
    # Popup window to pick 3 points, calculate platform center and radius, and store results
    popup = tk.Toplevel()
    canvas = tk.Canvas(popup, width=IMG_W, height=IMG_H, borderwidth=0, highlightthickness=0)
    popup.wm_title("Platform center calibration")
    popup.bind("<Button 1>",getorigin)
    global x, y
    global plat_defined
    plat_defined = False
    calibCoords = []

    cv2Img, tkImg  = getImage()
    canvas.create_image((0, 0), image=tkImg, anchor="nw")
    canvas.pack()
    while popup.winfo_exists():
        if x > 0:
            # Click detected, append coords to list and draw
            calibCoords.append([x, y])
            marker_r = 4
            canvas.create_oval(x-marker_r, y-marker_r, x+marker_r, y+marker_r, fill="red")
            x, y = 0, 0
        if len(calibCoords) >= 3:
            # All markers placed
            global plat_c, plat_r
            plat_c, plat_r = find_center(calibCoords[0][0], calibCoords[0][1], calibCoords[1][0], calibCoords[1][1], calibCoords[2][0], calibCoords[2][1])
            print("Center at X:{:.2f}; Y:{:.2f}\nRadius:{:.2f}".format(plat_c[0], plat_c[1], plat_r))
            canvas.create_oval(plat_c[0]-plat_r, plat_c[1]-plat_r, plat_c[0]+plat_r, plat_c[1]+plat_r, fill="", outline="blue", width=4)
            popup.update()
            time.sleep(1)
            popup.destroy()

        popup.update_idletasks()
        popup.update()
        time.sleep(0.01)

def getorigin(eventorigin):
    # Gets mouse position
    global x,y
    x = eventorigin.x
    y = eventorigin.y

def find_center(x1, y1, x2, y2, x3, y3):
    # Find circle parameters from 3 points on its circumference
    A = 2 * np.array([[(x2-x1), (y2-y1)],
                      [(x3-x2), (y3-y2)]])
    b = np.array([[x2**2 + y2**2 - x1**2 - y1**2], 
                  [x3**2 + y3**2 - x2**2 - y2**2]])
    center = np.linalg.solve(A, b)
    center = np.squeeze(center)
    r = np.sqrt((x1-center[0])**2 + (y1-center[1])**2)
    return center, r # center[0] = x, center [1] = y

def color_calib_popup():
    # Popup window to calibrate color mask values
    popup = tk.Toplevel()
    canvas = tk.Canvas(popup, width=IMG_W, height=IMG_H, borderwidth=0, highlightthickness=0)
    popup.wm_title("Color calibration")

    cv2Img, tkImg  = getImage()
    canv_img = canvas.create_image((0, 0), image=tkImg, anchor="nw")
    canvas.pack(side="left")
    scale_len = 200
    control_frame = tk.Frame(popup); control_frame.pack(side="bottom")
    hue_box = tk.LabelFrame(control_frame, text="Hue")
    sat_box = tk.LabelFrame(control_frame, text="Sat")
    val_box = tk.LabelFrame(control_frame, text="Val")

    global col_mask
    local_mask = {}

    hl = tk.Scale(hue_box, from_=0, to=255, length=scale_len); hl.set(col_mask["hmin"]); hl.pack(side="left")
    hh = tk.Scale(hue_box, from_=0, to=255, length=scale_len); hh.set(col_mask["hmax"]); hh.pack(side="left")
    sl = tk.Scale(sat_box, from_=0, to=255, length=scale_len); sl.set(col_mask["smin"]); sl.pack(side="left")
    sh = tk.Scale(sat_box, from_=0, to=255, length=scale_len); sh.set(col_mask["smax"]); sh.pack(side="left")
    vl = tk.Scale(val_box, from_=0, to=255, length=scale_len); vl.set(col_mask["vmin"]); vl.pack(side="left")
    vh = tk.Scale(val_box, from_=0, to=255, length=scale_len); vh.set(col_mask["vmax"]); vh.pack(side="left")
    
    def read_mask_vals():
        local_mask["hmin"] = hl.get()
        local_mask["hmax"] = hh.get()
        local_mask["smin"] = sl.get()
        local_mask["smax"] = sh.get()
        local_mask["vmin"] = vl.get()
        local_mask["vmax"] = vh.get()

    def save_mask_vals():
        col_mask["hmin"] = hl.get()
        col_mask["hmax"] = hh.get()
        col_mask["smin"] = sl.get()
        col_mask["smax"] = sh.get()
        col_mask["vmin"] = vl.get()
        col_mask["vmax"] = vh.get()
        print(col_mask)
        popup.destroy()
        global color_calibrated
        color_calibrated = True

    confirm_btn = tk.Button(control_frame, text="Confirm", command=save_mask_vals)
    confirm_btn.pack(side="bottom")
    mask_enabled = tk.BooleanVar(control_frame)
    mask_check = tk.Checkbutton(control_frame, text="Show mask", variable=mask_enabled, offvalue=False, onvalue=True); mask_check.pack(side="bottom")
    val_box.pack(side="bottom"); sat_box.pack(side="bottom"); hue_box.pack(side="bottom")

    while 1:
        cv2Img, tkImg = getImage()
        read_mask_vals()
        mask, masked_img = mask_img(cv2Img, local_mask)
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if mask_enabled.get():
            print("Masked")
            tkImg = cv2_to_tk(mask_bgr)
        else:
            tkImg = cv2_to_tk(masked_img)
        canvas.itemconfig(canv_img, image=tkImg)
        popup.update_idletasks()
        popup.update()
        time.sleep(0.01)

#  Initialize image
cv2Img, tkImg = getImage()
canv_img = rcanvas.create_image((0, 0), image=tkImg, anchor="nw")
rcanvas.pack(side="left")

btnStill = tk.Button(root, text="Get still image", command=imagePopup).pack(side="bottom")
btnCalibrateCenter = tk.Button(root, text="Calibrate center", command=center_calib_popup).pack(side="bottom")
btnCalibrateColor = tk.Button(root, text="Calibrate color", command=color_calib_popup).pack(side="bottom")

plat_defined = False
color_calibrated = False

while 1:
    # Draw new image
    cv2Img, tkImg  = getImage()
    rcanvas.itemconfig(canv_img, image=tkImg)

    # Draw platform home pos. 
    if plat_r > 0 and not plat_defined:
        if "xcoordhelper" in locals():
            rcanvas.delete(xcoordhelper)
            rcanvas.delete(ycoordhelper)
            rcanvas.delete(platcircle)
        xcoordhelper = rcanvas.create_line(plat_c[0], plat_c[1], plat_c[0] + 20, plat_c[1], fill="red", width=2)
        ycoordhelper = rcanvas.create_line(plat_c[0], plat_c[1], plat_c[0], plat_c[1] + 20, fill="blue", width=2)
        platcircle = rcanvas.create_oval(plat_c[0]-plat_r, plat_c[1]-plat_r, plat_c[0]+plat_r, plat_c[1]+plat_r, fill="", outline="blue", width=1)
        plat_defined = True

    # Detect contours and draw outline
    if color_calibrated:
        mask, masked_image = mask_img(cv2Img, col_mask)
        imgContour, contours = cvzone.findContours(cv2Img, mask)
        if contours:
            ball_pos_abs =  ((contours[0]['center'][0]), (contours[0]['center'][1]))
            ball_area =     contours[0]['area']
        if "center_circle" in locals():
            rcanvas.delete(center_circle)
        rect_radius = np.sqrt(ball_area/np.pi)
        center_circle = rcanvas.create_oval(ball_pos_abs[0]-rect_radius, ball_pos_abs[1]-rect_radius, ball_pos_abs[0]+rect_radius, ball_pos_abs[1]+rect_radius, fill="", outline="green", width=4)
    
    # Update GUI
    root.update_idletasks()
    root.update()
    time.sleep(0.05)