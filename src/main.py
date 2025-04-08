import sys
import cv2
import math
import time
import threading
import numpy as np

import hiwonder.PID as PID
import hiwonder.Misc as Misc
import hiwonder.Board as Board
import hiwonder.Camera as Camera
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

debug = False

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = ('green',)
def setTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, (), 'SetTargetTrackingColor')

def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours: 
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 10:
                area_max_contour = c

    return area_max_contour, contour_area_max

lab_data = None
servo_data = None
def load_config():
    global lab_data, servo_data
    
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)

load_config()

x_dis = servo_data['servo2']
y_dis = 1500

def initMove():
    Board.setPWMServoPulse(1, y_dis, 500)
    Board.setPWMServoPulse(2, x_dis, 500)

x_pid = PID.PID(P=0.2, I=0.002, D=0.02)
y_pid = PID.PID(P=0.2, I=0.002, D=0.02)

def reset():
    global x_dis, y_dis
    global __target_color
       
    x_dis = servo_data['servo2']
    y_dis = 1500
    x_pid.clear()
    y_pid.clear()
    __target_color = ()
    initMove()

def init():
    print("ColorTrack Init")
    load_config()
    reset()

__isRunning = False
def start():
    global __isRunning
    __isRunning = True
    print("ColorTrack Start")

def stop():
    global __isRunning
    __isRunning = False
    reset()
    print("ColorTrack Stop")

def exit():
    global __isRunning
    __isRunning = False
    AGC.runActionGroup('stand_slow')
    print("ColorTrack Exit")

def hisEqulColor(img):
    ycrcb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split(ycrcb)
    cv2.equalizeHist(channels[0], channels[0])
    cv2.merge(channels, ycrcb)
    img_eq = cv2.cvtColor(ycrcb, cv2.COLOR_YCR_CB2BGR)
    return img_eq

size = (320, 240)

mode = 0
bufferCount = 0;
move = ''

def moveThread():
    global bufferCount

    while True:
        if (len(move) > 0):
            AGC.runActionGroup(move)
            if (bufferCount < 0):
                bufferCount -= 1

checkingRight = True
timeWait = 0

def processInfo(frameInfo, img):
    global x_dis, y_dis, move, mode, checkingRight, timeWait
    # Is looking around for the plant
    if mode == 0:
        if frameInfo[0]:
            # Moves the camera in the suggested direction
            y_dis += frameInfo[3]
            y_dis = 1000 if y_dis < 1000 else y_dis
            y_dis = 2000 if y_dis > 2000 else y_dis    

            x_dis += frameInfo[2]
            x_dis = 500 if x_dis < 500 else x_dis          
            x_dis = 2500 if x_dis > 2500 else x_dis

            Board.setPWMServoPulse(1, y_dis, frameInfo[4]*1000)
            Board.setPWMServoPulse(2, x_dis, frameInfo[4]*1000)
            time.sleep(frameInfo[4])

            if x_dis > 1400:
                move = 'turn_left'
            elif x_dis < 1600:
                move = 'turn_right'
            else:
                move = ''
                mode = 1
        else:
            if checkingRight:
                x_dis += 20
                x_dis = 2500 if x_dis > 2500 else x_dis
                Board.setPWMServoPulse(2, x_dis, 0.001)
                time.sleep(0.001)

                if x_dis == 2500:
                    checkingRight = False
            else:
                x_dis -= 20
                x_dis = 500 if x_dis < 500 else x_dis
                Board.setPWMServoPulse(2, x_dis, 0.001)
                time.sleep(0.001)

    # Moving Toward the plant
    elif mode == 1:
        # We sort of assume that the camera can still see the plant at this point
        y_dis += frameInfo[3]
        y_dis = 1000 if y_dis < 1000 else y_dis
        y_dis = 2000 if y_dis > 2000 else y_dis    

        x_dis += frameInfo[2]
        x_dis = 500 if x_dis < 500 else x_dis          
        x_dis = 2500 if x_dis > 2500 else x_dis

        Board.setPWMServoPulse(1, y_dis, frameInfo[4]*1000)
        Board.setPWMServoPulse(2, x_dis, frameInfo[4]*1000)
        time.sleep(frameInfo[4])

        if x_dis > 1400:
            move = 'turn_left'
        elif x_dis < 1600:
            move = 'turn_right'
        else:
            move = 'go_forward'

        if frameInfo[1] > 100:
            mode = 2
            timeWait = 500

    # Taking a picture of the plant
    elif mode == 2:
        if timeWait > 0:
            timeWait -= 1
        else:
            cv2.imwrite('plant.jpg', img)
            mode = -1

# Gets the image and analysis all the colors and color groups available
def run(img):
    global x_dis, y_dis
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning or __target_color == ():
        return img

    cv2.line(img, (int(img_w/2 - 10), int(img_h/2)), (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w/2), int(img_h/2 - 10)), (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (5, 5), 5)   
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
    
    area_max = 0
    areaMaxContour = 0 for i in lab_data:
        if i in __target_color:
            detect_color = i
            frame_mask = cv2.inRange(frame_lab,
                                         (lab_data[i]['min'][0],
                                          lab_data[i]['min'][1],
                                          lab_data[i]['min'][2]),
                                         (lab_data[i]['max'][0],
                                          lab_data[i]['max'][1],
                                          lab_data[i]['max'][2]))
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            if debug:
                cv2.imshow(i, dilated)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)

    colorPresent = area_max > 20
    dx = 0
    dy = 0
    centerX = 0
    centerY = 0
    use_time = 0
    # Draws a circle on the screen around the color and moves the camera
    if colorPresent:
        # Draws the circle
        (centerX, centerY), radius = cv2.minEnclosingCircle(areaMaxContour)
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))
        cv2.circle(img, (int(centerX), int(centerY)), int(radius), range_rgb[detect_color], 2)
        
        # Moves the camera to the next position
        if abs(centerX - img_w/2) > 15:
            x_pid.SetPoint = img_w/2
        else:
            x_pid.SetPoint = centerX 
        x_pid.update(centerX)
        dx = int(x_pid.output)
        use_time = abs(dx*0.00025)
        # x_dis += dx
        
        # x_dis = 500 if x_dis < 500 else x_dis          
        # x_dis = 2500 if x_dis > 2500 else x_dis
            
        if abs(centerY - img_h/2) > 15:  
            y_pid.SetPoint = img_h/2
        else:
            y_pid.SetPoint = centerY      
        y_pid.update(centerY)
        dy = int(y_pid.output)
        use_time = round(max(use_time, abs(dy*0.00025)), 5)
        # y_dis += dy
        
        # y_dis = 1000 if y_dis < 1000 else y_dis
        # y_dis = 2000 if y_dis > 2000 else y_dis    
        
        if not debug:
            # Board.setPWMServoPulse(1, y_dis, use_time*1000)
            # Board.setPWMServoPulse(2, x_dis, use_time*1000)
            # time.sleep(use_time)
            
    # Returns true if the color is present and suggested movement to for the
    # camera to center on the green object
    return (colorPresent, area_max, dx, dy, use_time, centerX, centerY)

def main():
    # This is a hard coded path that I need to get
    # TODO replace this with the hard coded paths it provides
    param_data = np.load('/home/pi/TonyPi/Functions/CameraCalibration/calibration_param.npz')

    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)
    
    debug = False
    if debug:
        print('Debug Mode')
    
    init()
    start()
    __target_color = ('green',)

    # Initializing Camera
    open_once = yaml_handle.get_yaml_data('/boot/camera_setting.yaml')['open_once']
    if open_once:
        my_camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
    else:
        my_camera = Camera.Camera()
        my_camera.camera_open()        
    AGC.runActionGroup('stand')

    # Loop for Video
    while True:
        ret, img = my_camera.read()
        if ret:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
            nextMovement = run(frame)           
            processInfo(nextMovement, frame)
            cv2.imshow('Frame', frame)
            key = cv2.waitKey(1)
            # This is the q key
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

