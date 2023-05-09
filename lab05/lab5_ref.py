import cv2
# import tello
import time
import math
import numpy as np
from pyimagesearch.pid import PID
from djitellopy import Tello

def keyboard(self, key):
    #global is_flying
    # is_flying = False
    print("key:", key)
    fb_speed = 40
    lf_speed = 40
    ud_speed = 50
    degree = 30
    if key == ord('1'):
        self.takeoff()
        #is_flying = True
    if key == ord('2'):
        self.land()
        #is_flying = False
    if key == ord('q'):
        self.sned_rc_control(0,0,0,0)
        is_keyboard = False
        print("let it autopilot!")
    if key == ord('3'):
        self.send_rc_control(0, 0, 0, 0)
        is_keyboard = True
        print("stop!!!!")
    if key == ord('w'):
        self.send_rc_control(0, fb_speed, 0, 0)
        is_keyboard = True
        print("forward!!!!")
    if key == ord('s'):
        self.send_rc_control(0, (-1) * fb_speed, 0, 0)
        is_keyboard = True
        print("backward!!!!")
    if key == ord('a'):
        self.send_rc_control((-1) * lf_speed, 0, 0, 0)
        is_keyboard = True
        print("left!!!!")
    if key == ord('d'):
        self.send_rc_control(lf_speed, 0, 0, 0)
        is_keyboard = True
        print("right!!!!")
    if key == ord('z'):
        self.send_rc_control(0, 0, ud_speed, 0)
        is_keyboard = True
        print("down!!!!")
    if key == ord('x'):
        self.send_rc_control(0, 0, (-1) *ud_speed, 0)
        is_keyboard = True
        print("up!!!!")
    if key == ord('c'):
        self.send_rc_control(0, 0, 0, degree)
        is_keyboard = True
        print("rotate!!!!")
    if key == ord('v'):
        self.send_rc_control(0, 0, 0, (-1) *degree)
        is_keyboard = True
        print("counter rotate!!!!")
    if key == ord('5'):
        height = self.get_height()
        print(height)
    if key == ord('6'):
        battery = self.get_battery()
        print (battery)
# drone = tello.Tello('', 8889)
# cap = cv2.VideoCapture(0)
drone = Tello()
drone.connect()
def camera_calibration():
    # drone = tello.Tello('', 8889)
    time.sleep(5)

    width, height = 9, 6
    object_point = np.zeros((width * height,3), np.float32)
    object_point[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1,2)
    object_point = object_point * 23 #2.3cm
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    num = 0

    object_points = []
    img_points = []

    while(True):
        # frame = drone.read()
        # ret, frame = cap.read()
        drone.streamon()
        frame = drone.get_frame_read()
        frame = frame.frame
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imshow('frame', frame)
        key = cv2.waitKey(100)

        if key == ord('q') or key == ord('Q') or key == 27: # Esc
            print('break')
            break
        elif key == 13: # Enter
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            #cv2.imshow("image", gray)
            ret, corners = cv2.findChessboardCorners(gray, (width, height), None)
            if ret == True:
                cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                object_points.append(object_point)
                img_points.append(corners)
                num += 1
                print('successful')
            else:
                print('failed')
        if num >= 10:
            break
    cv2.destroyAllWindows()  

    ret_1, intrinsic, distortion, rvecs, tvecs = cv2.calibrateCamera(object_points, img_points, gray.shape[::-1], None, None)
    f = cv2.FileStorage('camera_calibration.txt', cv2.FILE_STORAGE_WRITE)
    f.write("intrinsic", intrinsic)
    f.write("distortion", distortion)
    f.release()

def control():
    frame = np.zeros((123,123,3),np.uint8)
    while(True):
        if key != -1:
            keyboard(drone,key)
        cv2.imshow("test",frame)
        key = cv2.waitKey(33)


def angle(v1, v2):
    dx1 = v1[2] - v1[0]
    dy1 = v1[3] - v1[1]
    dx2 = v2[2] - v2[0]
    dy2 = v2[3] - v2[1]
    angle1 = math.atan2(dy1, dx1)
    angle1 = int(angle1 * 180/math.pi)
    # print(angle1)
    angle2 = math.atan2(dy2, dx2)
    angle2 = int(angle2 * 180/math.pi)
    # print(angle2)
    if angle1*angle2 >= 0:
        included_angle = abs(angle1-angle2)
    else:
        included_angle = abs(angle1) + abs(angle2)
        if included_angle > 180:
            included_angle = 360 - included_angle
    return included_angle

def main():
    # camera_calibration()

    f = cv2.FileStorage('camera_calibration.txt', cv2.FILE_STORAGE_READ)
    intrinsic, distortion = f.getNode("intrinsic").mat(), f.getNode("distortion").mat()
    z_pid = PID(kP=0.7,kI=0.0001,kD=0.1)            
    y_pid = PID(kP=0.7,kI=0.005,kD=0.1)            
    # x_pid = PID(kP=0.7, kI=0.0001, kD=0.1)
    yaw_pid = PID(kP=0.8,kI=0.005,kD=0.2)

    yaw_pid.initialize()
    z_pid.initialize()
    y_pid.initialize()            
    # x_pid.initialize()
    print(intrinsic.shape, distortion.shape)
    font = cv2.FONT_HERSHEY_SIMPLEX 

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()
    is_flying = False
    while(True):
        if not is_flying:
            drone.send_rc_control(0, 0, 0, 0)
        # frame = drone.read()
        # ret, frame = cap.read()
        drone.streamon()
        frame = drone.get_frame_read()
        frame = frame.frame
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    
        if markerIds is not None:
            # frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
            rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.15, intrinsic, distortion)
            # frame = cv2.putText(frame,"x: %.3f y: %.3f z:%.3f" %(tvec[0][0][0],tvec[0][0][1],tvec[0][0][2]),(00,185),cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0),2,cv2.LINE_AA)
            # print(tvec)            
            speed = 40

            # x_update = tvec[0][0][0] * 100 - 100
            # print("org_x: " + str(x_update))
            # x_update = x_pid.update(x_update,sleep=0)
            # print("pid_x: " + str(x_update))
            # if x_update > speed:
            #     x_update = speed
            # elif x_update < -speed:
            #     x_update = -speed
            print("tevvv")
            print(tvec[0][0])
            y_update = tvec[0][0][1]  - 10
            print("org_y: " + str(y_update))
            y_update = y_pid.update(y_update,sleep=0)
            print("pid_y: "+str(y_update))
            if y_update > speed:
                y_update = speed
            elif y_update < -speed:
                y_update = -speed

            z_update = tvec[0][0][2]* 100  - 50
            print("org_z: " + str(z_update))
            z_update = z_pid.update(z_update,sleep=0)
            print("pid_z: "+str(z_update))
            if z_update > speed:
                z_update = speed
            elif z_update < -speed:
                z_update = -speed

            dst = cv2.Rodrigues(rvec[0][0])
            print(dst)
            z = np.array([dst[0][0][2],dst[0][1][2],dst[0][2][2]])
            z = np.array([z[0],0,z[2]])
            degree = math.atan2(z[0],z[2])
            degree = math.degrees(degree)
            yaw_update = degree - 90
            print("org_yaw: " + str(yaw_update))
            yaw_update = yaw_pid.update(yaw_update,sleep=0)
            print("pid_yaw: " + str(yaw_update))
            # yaw_update = math.degrees(yaw_update)
            if yaw_update > 20:
                yaw_update = 20
            elif yaw_update < -20:
                yaw_update=-20

                
            is_flying = True
            drone.send_rc_control(0, int(z_update//2), int(y_update//2), -int(yaw_update//2))
            # frame = cv2.aruco.drawAxis(frame, intrinsic, distortion, rvec, tvec, 0.1)
        else:
            is_flying = False
        cv2.imshow('frame', frame)
        key = cv2.waitKey(33)
        keyboard(drone,key)
        if is_keyboard:
            while True:
                keyboard(drone,key)
                if not is_keyboard:
                    break
        # if key != -1:
        #     keyboard(drone,key)
        # if key == ord('q') or key == ord('Q') or key == 27: # Esc
        #     print('break')
        #     break
    cv2.destroyAllWindows() 

if __name__ == "__main__":
    time.sleep(5)
    global is_flying
    is_flying = False
    global is_keyboard
    is_keyboard = False
    
    main()
    # control()

