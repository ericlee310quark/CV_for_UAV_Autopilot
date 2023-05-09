<<<<<<< HEAD
import os.path as path
import numpy as np
import cv2
import time
import math
#import tello
from pyimagesearch.pid import PID
from djitellopy import Tello

#####################################################################
#       Const. Definition
#####################################################################
MAX_SPEED_THRESHOLD = 40
Z_BASE = np.array([[0],[0],[1]])
RC_update_para_x = 1
RC_update_para_y = 1
RC_update_para_z = 2
RC_update_para_yaw = 1

CORRECT_THRESHOD_X = 5
CORRECT_THRESHOD_Y = 5
CORRECT_THRESHOD_Z = 5
#####################################################################
#       State flags
#####################################################################

current_state = 0

step_1_correct = 1
step_1_down = 2
step_1_finish = 3

step_2_follow = 4
step_2_finish = 5

step_3_1st_correct = 6
step_3_1st_ready = 7        #stop and fly right
step_3_2nd_correct = 8
step_3_2nd_ready = 9        #stop and fly left and forward

step_4_correct = 10
step_4_finish = 11 #!!!!!! No need?

#####################################################################
#       Func. Definition
#####################################################################

def keyboard(self, key):
    #global is_flying
    print("key:", key)
    fb_speed = 40
    lf_speed = 40
    ud_speed = 50
    degree = 30
    if key == ord('1'):
        self.takeoff()
        is_flying = True
        print("Take off!!!")
    if key == ord('2'):
        self.land()
        is_flying = False
        print("Landed!!!")
    if key == ord('3'):
        self.send_rc_control(0, 0, 0, 0)
        print("stop!!!!")
    if key == ord('w'):
        self.send_rc_control(0, fb_speed, 0, 0)
        print("forward!!!!")
    if key == ord('s'):
        self.send_rc_control(0, (-1) * fb_speed, 0, 0)
        print("backward!!!!")
    if key == ord('a'):
        self.send_rc_control((-1) * lf_speed, 0, 0, 0)
        print("left!!!!")
    if key == ord('d'):
        self.send_rc_control(lf_speed, 0, 0, 0)
        print("right!!!!")
    if key == ord('z'):
        self.send_rc_control(0, 0, ud_speed, 0)
        print("down!!!!")
    if key == ord('x'):
        self.send_rc_control(0, 0, (-1) *ud_speed, 0)
        print("up!!!!")
    if key == ord('c'):
        self.send_rc_control(0, 0, 0, degree)
        print("rotate!!!!")
    if key == ord('v'):
        self.send_rc_control(0, 0, 0, (-1) *degree)
        print("counter rotate!!!!")
    if key == ord('5'):
        height = self.get_height()
        print(height)
    if key == ord('6'):
        battery = self.get_battery()
        print (battery)

def intrinsic_parameter():
    f = cv2.FileStorage('in_coeffs.xml', cv2.FILE_STORAGE_READ)
    intr = f.getNode("intrinsic").mat()
    dist = f.getNode("distortion").mat()
    
    print("intrinsic: {}".format(intr))
    print("distortion: {}".format(dist))

    f.release()
    return intr, dist

###################################
#       Battery display
###################################

def battery_dis_per30s():
    curr_time = time.time()
    if (curr_time-prev_time)>30:
        prev_time = curr_time
        battery = drone.get_battery()
        print("Now battery: {}".format(battery))

def MAX_threshold(value):
    if value > MAX_SPEED_THRESHOLD:
        print("fixed to {}".format(str(MAX_SPEED_THRESHOLD)))
        return MAX_SPEED_THRESHOLD
    elif value < -1* MAX_SPEED_THRESHOLD:
        print("fixed to {}".format(str(-1* MAX_SPEED_THRESHOLD)))
        return -1* MAX_SPEED_THRESHOLD
    else:
        return value

###################################
#       find ARUCO
###################################
counter = 0     #for waiting the drone be stable
counter_2 = 0   #to wait the drone seeing the aruco for a countinuous time(use in Step2 -> Step3)

def find_id(markerIds, id)->int:
    try:
        find_target = int(markerIds.index(id))
    except:
        find_target = -1
    return find_target

def correct_v2(rvec, tvec, i, dist_diff, countable= True)->bool:
    PID_state = {}
    rvec_3x3,_ = cv2.Rodrigues(rvec[i])
    rvec_zbase = rvec_3x3.dot(Z_BASE)
    rx_project = rvec_zbase[0]
    rz_project = rvec_zbase[2]
    angle_diff= math.atan2(float(rz_project), float(rx_project))*180/math.pi + 90  #from -90 to 90
    # When angle_diff -> +90:
    #   turn counterclockwise
    # When angle_diff -> -90:
    #   turn clockwise
    #######################
    #       Z-PID
    #######################
    z_update = tvec[i,0,2] - dist_diff #70 --> 85
    PID_state["org_z"] = str(z_update)
    z_update = z_pid.update(z_update, sleep=0)
    PID_state["pid_z"] = str(z_update)

    z_update = MAX_threshold(z_update)
    
    #######################
    #       X-PID
    #######################
    x_update = tvec[i,0,0]
    PID_state["org_x"] = str(x_update)
    x_update = x_pid.update(x_update, sleep=0)
    PID_state["pid_x"] = str(x_update)
    
    x_update = MAX_threshold(x_update)
    
    #######################
    #       Y-PID
    #######################
    y_update = tvec[i,0,1]*(-1)
    PID_state["org_y"] = str(y_update)
    y_update = y_pid.update(y_update, sleep=0)
    PID_state["pid_y"] = str(y_update)

    y_update = MAX_threshold(y_update)
    
    #######################
    #       YAW-PID
    #######################
    yaw_update = (-1)* angle_diff
    PID_state["org_yaw"] = str(yaw_update)
    yaw_update = yaw_pid.update(yaw_update, sleep=0)
    PID_state["pid_yaw"] = str(yaw_update)

    yaw_update = MAX_threshold(yaw_update)
    
    #######################
    #   Motion Response
    #######################
    drone.send_rc_control(int(x_update//RC_update_para_x), int(z_update//RC_update_para_z), int(y_update//RC_update_para_y), int(yaw_update//RC_update_para_yaw))
    print("--------------------------------------------")
    #now = time.ctime()
    #print("{}: PID state".format(now))
    print("MarkerIDs: {}".format(i))
    print("tvec: {}||{}||{}||{}".format(tvec[i,0,0], tvec[i,0,1], tvec[i,0,2], angle_diff))
    print("org: {}||{}||{}||{}".format(PID_state["org_x"],PID_state["org_y"],PID_state["org_z"],PID_state["org_yaw"]))
    print("PID: {}||{}||{}||{}".format(PID_state["pid_x"],PID_state["pid_y"],PID_state["pid_z"],PID_state["pid_yaw"]))
    print("--------------------------------------------")
    text ="ID:{}|x,y,z||angle = {},{},{}||{}".format(len(markerIds), tvec[i,0,0], tvec[i,0,1], tvec[i,0,2], angle_diff) 
    #print(tvec)
    #cv2.putText(frame, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
    if(tvec[i,0,0]<=CORRECT_THRESHOD_X and tvec[i,0,0]>=(-1)*CORRECT_THRESHOD_X\
         and tvec[i,0,1]<=CORRECT_THRESHOD_Y and tvec[i,0,1]>=(-1)*CORRECT_THRESHOD_Y\
              and tvec[i,0,2]<=(dist_diff+CORRECT_THRESHOD_Z) and tvec[i,0,2]>= (dist_diff-CORRECT_THRESHOD_Z) and countable):
        counter +=1
        if counter >500:
            counter =0
            return True
    else:
        counter = 0
        return False

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#TODO fly_around:  if drone can't see aruco first, just fly in some patterns to guess the postiion of aruco
fly_around_pattern = 0
time_diff = 0
time_diff_2 = 0
temp_fly = 1
time_diff_thred = 1
def fly_around():
    curr_time = time.time()
    time_diff = curr_time - prev_time
    global temp_fly, time_diff_thred
    if time_diff> time_diff_thred:
        prev_time = curr_time
        if fly_around_pattern == 0:
            drone.send_rc_control(int(10), int(0), int(0), int(0))        #left
            fly_around_pattern = 9
            temp_fly = 1
            time_diff_thred = 2
        elif fly_around_pattern == 1:
            drone.send_rc_control(int(-10), int(0), int(0), int(0))     #right
            fly_around_pattern = 9
            temp_fly = 2
            time_diff_thred = 1.414
        elif fly_around_pattern == 2:
            drone.send_rc_control(int(10), int(0), int(10), int(0))   #\
            fly_around_pattern = 9
            temp_fly = 3
        elif fly_around_pattern == 3:
            drone.send_rc_control(int(0), int(0), int(-10), int(0))        #down
            fly_around_pattern = 9
            temp_fly = 4
            time_diff_thred = 2
        elif fly_around_pattern == 4:
            drone.send_rc_control(int(0), int(-5), int(0), int(0))          #back
            fly_around_pattern = 9
            temp_fly = 5
            time_diff_thred = 1
        elif fly_around_pattern == 5:
            drone.send_rc_control(int(20), int(0), int(0), int(0))          #
            fly_around_pattern = 9
            temp_fly = 6
        elif fly_around_pattern == 6:
            drone.send_rc_control(int(20), int(0), int(0), int(0))
            fly_around_pattern = 9
            temp_fly = 7
        elif fly_around_pattern == 7:
            drone.send_rc_control(int(20), int(0), int(0), int(0))
            fly_around_pattern = 9
            temp_fly = 8
        elif fly_around_pattern == 8:
            drone.send_rc_control(int(20), int(0), int(0), int(0))
            fly_around_pattern = 9
            temp_fly = 0
        elif fly_around_pattern == 9:
            drone.send_rc_control(int(0), int(0), int(0), int(0))
            fly_around_pattern = temp_fly
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

####################################################################################
#           Main Block
####################################################################################
    
if __name__ == '__main__':
    #############################################
    #           SETUP and Initialization
    #############################################
    global is_flying, prev_time, curr_time
    is_flying = False
    #drone = tello.Tello('', 8889)
    cali_intr, cali_dist = intrinsic_parameter()    #fetch the calibration data
    drone = Tello()
    drone.connect()
    #cap = cv2.VideoCapture(1)
    time.sleep(10)

    x_pid = PID(kP=0.8, kI=0.00005, kD=0.28)  # Use tvec_x (tvec[i,0,0]) ----> control left and right
    z_pid = PID(kP=0.8, kI=0.0005, kD=0.2)  # Use tvec_z (tvec[i,0,2])----> control forward and backward
    y_pid = PID(kP=0.8, kI=0.0001, kD=0.15)  # Use tvec_y (tvec[i,0,1])----> control upward and downward
    yaw_pid = PID(kP=0.8,kI=0.0001, kD=0.15)
    x_pid.initialize()
    z_pid.initialize()
    y_pid.initialize()
    yaw_pid.initialize()

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()

    ###################################
    #       Battery display
    ###################################
    prev_time = time.time()
    #battery = drone.get_battery()
    #print("Now battery: {}".format(battery))
    #curr_time = time.time()

    ####################################################################################
    #   Frame Loop
    ####################################################################################
    key = -1

    try:
        while True: 
            drone.streamon()
            #!frame = drone.read()
            #ret, frame = cap.read()
            frame = drone.get_frame_read()
            frame = frame.frame
            #!frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            markerIds = []
            markerCorners, markerIds, rejectedCandidates =cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            
            correct_ready = False

            if markerIds is not None:
                
                frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
                rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, cali_intr, cali_dist)
                for i in range(len(markerIds)):
                    frame = cv2.aruco.drawAxis(frame, cali_intr, cali_dist, rvec[i], tvec[i], 7.5)

            ########################################
            #  Highest control for keyboard control
            ########################################
            if key!=-1:
                #drone.send_rc_control(0, 0, 0, 0)
                keyboard(drone,key)
            ##########################################################################
            #       FSM states
            ##########################################################################
            else:
                #=======================================
                #       Forward 125 cm
                #=======================================
                if (current_state == 0 and is_flying):
                    print("state: {}".format(current_state))
                    drone.move("forward",125)
                    current_state = step_1_correct
                #=======================================
                #       correct to id 1
                #=======================================
                elif (current_state == step_1_correct):
                    print("state: {}".format(current_state))

                    target_idex = find_id(markerIds, 1)
                    if target_idex != -1:
                        correct_ready = correct_v2(rvec, tvec, target_idex, 70)
                    else:
                        #TODO
                        #fly_around()
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")

                    if (correct_ready == True):
                        current_state = step_1_down
                #=======================================
                #       connect to Step 2 & follow id 0
                #=======================================
                elif (current_state == step_1_down):
                    print("state: {}".format(current_state))
                    drone.move("down", 40)
                    drone.move("forward",65)
                    drone.move("up", 40)
                    current_state = step_2_follow

                elif (current_state == step_2_follow):
                    print("state: {}".format(current_state))

                    target_idex = find_id(markerIds, 0)
                    if target_idex != -1:
                        correct_v2(rvec, tvec, target_idex, 70, False)
                    else:
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")

                    target_idex_2 = find_id(markerIds, 5)
                    if target_idex_2 != -1:
                        counter_2 += 1
                    if ((counter_2 >= 100) and (target_idex == -1)):
                        current_state = step_2_finish
                        counter_2 = 0
                #=======================================
                #       connect to Step 3
                #=======================================
                elif (current_state == step_2_finish):
                    print("state: {}".format(current_state))
                    target_idex_2 = find_id(markerIds, 5)
                    if target_idex_2 != -1:
                        current_state = step_3_1st_correct    
                    else:
                        drone.move("forward", 25)
                        """
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")
                        """

                        """
                        drone.send_rc_control(0, -10, 0, 0)
                        
                        """
                #=======================================
                #       correct to id 5
                #=======================================
                elif (current_state == step_3_1st_correct):
                    print("state: {}".format(current_state))
                    target_idex = find_id(markerIds, 5)
                    if target_idex != -1:
                        correct_ready = correct_v2(rvec, tvec, target_idex, 70)
                    else:
                        #TODO
                        #fly_around()
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")

                    if (correct_ready == True):
                        current_state = step_3_1st_ready
                #=======================================
                #       dodge and go to find id 3
                #=======================================
                elif (current_state == step_3_1st_ready):
                    print("state: {}".format(current_state))
                    drone.move("left",70)
                    current_state = step_3_2nd_correct
                #=======================================
                #       correct to id 3
                #=======================================
                elif(current_state == step_3_2nd_correct):
                    print("state: {}".format(current_state))
                    target_idex = find_id(markerIds, 3)
                    if target_idex != -1:
                        correct_ready = correct_v2(rvec, tvec, target_idex, 70)
                    else:
                        #TODO
                        #fly_around()
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")

                    if (correct_ready == True):
                        current_state = step_3_2nd_ready
                #=======================================
                #       dodge and go to Step 4
                #=======================================
                elif (current_state == step_3_2nd_ready):
                    print("state: {}".format(current_state))
                    drone.move("right",125)
                    drone.rotate_counter_clockwise(30)
                    drone.move("forward",70)
                    current_state = step_4_correct
                #=======================================
                #       correct to id 4 and land
                #=======================================   
                elif (current_state == step_4_correct):

                    print("state: {}".format(current_state))

                    target_idex = find_id(markerIds, 4)
                    if target_idex != -1:
                        correct_ready = correct_v2(rvec, tvec, target_idex, 85)
                    else:
                        #TODO
                        #fly_around()
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")
                
                    if (correct_ready == True):
                        drone.land()
                else:
                    print("state: out of state")
                    ################################
                    #   Floating if no instructions
                    ################################
                    if is_flying == True:
                        drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                    #now = time.ctime()
                    print("No instructions")
                
            cv2.imshow('frame', frame)
            key = cv2.waitKey(1)
            ########################################
            #   Display Battery
            ########################################    
            #battery_dis_per30s()
        else:
            print("fail to open film")

    except KeyboardInterrupt:
        #cap.release()
=======
import os.path as path
import numpy as np
import cv2
import time
import math
#import tello
from pyimagesearch.pid import PID
from djitellopy import Tello

#####################################################################
#       Const. Definition
#####################################################################
MAX_SPEED_THRESHOLD = 40
Z_BASE = np.array([[0],[0],[1]])
RC_update_para_x = 1
RC_update_para_y = 1
RC_update_para_z = 2
RC_update_para_yaw = 1

CORRECT_THRESHOD_X = 5
CORRECT_THRESHOD_Y = 5
CORRECT_THRESHOD_Z = 5
#####################################################################
#       State flags
#####################################################################

current_state = 0

step_1_correct = 1
step_1_down = 2
step_1_finish = 3

step_2_follow = 4
step_2_finish = 5

step_3_1st_correct = 6
step_3_1st_ready = 7        #stop and fly right
step_3_2nd_correct = 8
step_3_2nd_ready = 9        #stop and fly left and forward

step_4_correct = 10
step_4_finish = 11 #!!!!!! No need?

#####################################################################
#       Func. Definition
#####################################################################

def keyboard(self, key):
    #global is_flying
    print("key:", key)
    fb_speed = 40
    lf_speed = 40
    ud_speed = 50
    degree = 30
    if key == ord('1'):
        self.takeoff()
        is_flying = True
        print("Take off!!!")
    if key == ord('2'):
        self.land()
        is_flying = False
        print("Landed!!!")
    if key == ord('3'):
        self.send_rc_control(0, 0, 0, 0)
        print("stop!!!!")
    if key == ord('w'):
        self.send_rc_control(0, fb_speed, 0, 0)
        print("forward!!!!")
    if key == ord('s'):
        self.send_rc_control(0, (-1) * fb_speed, 0, 0)
        print("backward!!!!")
    if key == ord('a'):
        self.send_rc_control((-1) * lf_speed, 0, 0, 0)
        print("left!!!!")
    if key == ord('d'):
        self.send_rc_control(lf_speed, 0, 0, 0)
        print("right!!!!")
    if key == ord('z'):
        self.send_rc_control(0, 0, ud_speed, 0)
        print("down!!!!")
    if key == ord('x'):
        self.send_rc_control(0, 0, (-1) *ud_speed, 0)
        print("up!!!!")
    if key == ord('c'):
        self.send_rc_control(0, 0, 0, degree)
        print("rotate!!!!")
    if key == ord('v'):
        self.send_rc_control(0, 0, 0, (-1) *degree)
        print("counter rotate!!!!")
    if key == ord('5'):
        height = self.get_height()
        print(height)
    if key == ord('6'):
        battery = self.get_battery()
        print (battery)

def intrinsic_parameter():
    f = cv2.FileStorage('in_coeffs.xml', cv2.FILE_STORAGE_READ)
    intr = f.getNode("intrinsic").mat()
    dist = f.getNode("distortion").mat()
    
    print("intrinsic: {}".format(intr))
    print("distortion: {}".format(dist))

    f.release()
    return intr, dist

###################################
#       Battery display
###################################

def battery_dis_per30s():
    curr_time = time.time()
    if (curr_time-prev_time)>30:
        prev_time = curr_time
        battery = drone.get_battery()
        print("Now battery: {}".format(battery))

def MAX_threshold(value):
    if value > MAX_SPEED_THRESHOLD:
        print("fixed to {}".format(str(MAX_SPEED_THRESHOLD)))
        return MAX_SPEED_THRESHOLD
    elif value < -1* MAX_SPEED_THRESHOLD:
        print("fixed to {}".format(str(-1* MAX_SPEED_THRESHOLD)))
        return -1* MAX_SPEED_THRESHOLD
    else:
        return value

###################################
#       find ARUCO
###################################
counter = 0     #for waiting the drone be stable
counter_2 = 0   #to wait the drone seeing the aruco for a countinuous time(use in Step2 -> Step3)

def find_id(markerIds, id)->int:
    try:
        find_target = int(markerIds.index(id))
    except:
        find_target = -1
    return find_target

def correct_v2(rvec, tvec, i, dist_diff, countable= True)->bool:
    PID_state = {}
    rvec_3x3,_ = cv2.Rodrigues(rvec[i])
    rvec_zbase = rvec_3x3.dot(Z_BASE)
    rx_project = rvec_zbase[0]
    rz_project = rvec_zbase[2]
    angle_diff= math.atan2(float(rz_project), float(rx_project))*180/math.pi + 90  #from -90 to 90
    # When angle_diff -> +90:
    #   turn counterclockwise
    # When angle_diff -> -90:
    #   turn clockwise
    #######################
    #       Z-PID
    #######################
    z_update = tvec[i,0,2] - dist_diff #70 --> 85
    PID_state["org_z"] = str(z_update)
    z_update = z_pid.update(z_update, sleep=0)
    PID_state["pid_z"] = str(z_update)

    z_update = MAX_threshold(z_update)
    
    #######################
    #       X-PID
    #######################
    x_update = tvec[i,0,0]
    PID_state["org_x"] = str(x_update)
    x_update = x_pid.update(x_update, sleep=0)
    PID_state["pid_x"] = str(x_update)
    
    x_update = MAX_threshold(x_update)
    
    #######################
    #       Y-PID
    #######################
    y_update = tvec[i,0,1]*(-1)
    PID_state["org_y"] = str(y_update)
    y_update = y_pid.update(y_update, sleep=0)
    PID_state["pid_y"] = str(y_update)

    y_update = MAX_threshold(y_update)
    
    #######################
    #       YAW-PID
    #######################
    yaw_update = (-1)* angle_diff
    PID_state["org_yaw"] = str(yaw_update)
    yaw_update = yaw_pid.update(yaw_update, sleep=0)
    PID_state["pid_yaw"] = str(yaw_update)

    yaw_update = MAX_threshold(yaw_update)
    
    #######################
    #   Motion Response
    #######################
    drone.send_rc_control(int(x_update//RC_update_para_x), int(z_update//RC_update_para_z), int(y_update//RC_update_para_y), int(yaw_update//RC_update_para_yaw))
    print("--------------------------------------------")
    #now = time.ctime()
    #print("{}: PID state".format(now))
    print("MarkerIDs: {}".format(i))
    print("tvec: {}||{}||{}||{}".format(tvec[i,0,0], tvec[i,0,1], tvec[i,0,2], angle_diff))
    print("org: {}||{}||{}||{}".format(PID_state["org_x"],PID_state["org_y"],PID_state["org_z"],PID_state["org_yaw"]))
    print("PID: {}||{}||{}||{}".format(PID_state["pid_x"],PID_state["pid_y"],PID_state["pid_z"],PID_state["pid_yaw"]))
    print("--------------------------------------------")
    text ="ID:{}|x,y,z||angle = {},{},{}||{}".format(len(markerIds), tvec[i,0,0], tvec[i,0,1], tvec[i,0,2], angle_diff) 
    #print(tvec)
    #cv2.putText(frame, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
    if(tvec[i,0,0]<=CORRECT_THRESHOD_X and tvec[i,0,0]>=(-1)*CORRECT_THRESHOD_X\
         and tvec[i,0,1]<=CORRECT_THRESHOD_Y and tvec[i,0,1]>=(-1)*CORRECT_THRESHOD_Y\
              and tvec[i,0,2]<=(dist_diff+CORRECT_THRESHOD_Z) and tvec[i,0,2]>= (dist_diff-CORRECT_THRESHOD_Z) and countable):
        counter +=1
        if counter >500:
            counter =0
            return True
    else:
        counter = 0
        return False

#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#TODO fly_around:  if drone can't see aruco first, just fly in some patterns to guess the postiion of aruco
fly_around_pattern = 0
time_diff = 0
time_diff_2 = 0
temp_fly = 1
time_diff_thred = 1
def fly_around():
    curr_time = time.time()
    time_diff = curr_time - prev_time
    global temp_fly, time_diff_thred
    if time_diff> time_diff_thred:
        prev_time = curr_time
        if fly_around_pattern == 0:
            drone.send_rc_control(int(10), int(0), int(0), int(0))        #left
            fly_around_pattern = 9
            temp_fly = 1
            time_diff_thred = 2
        elif fly_around_pattern == 1:
            drone.send_rc_control(int(-10), int(0), int(0), int(0))     #right
            fly_around_pattern = 9
            temp_fly = 2
            time_diff_thred = 1.414
        elif fly_around_pattern == 2:
            drone.send_rc_control(int(10), int(0), int(10), int(0))   #\
            fly_around_pattern = 9
            temp_fly = 3
        elif fly_around_pattern == 3:
            drone.send_rc_control(int(0), int(0), int(-10), int(0))        #down
            fly_around_pattern = 9
            temp_fly = 4
            time_diff_thred = 2
        elif fly_around_pattern == 4:
            drone.send_rc_control(int(0), int(-5), int(0), int(0))          #back
            fly_around_pattern = 9
            temp_fly = 5
            time_diff_thred = 1
        elif fly_around_pattern == 5:
            drone.send_rc_control(int(20), int(0), int(0), int(0))          #
            fly_around_pattern = 9
            temp_fly = 6
        elif fly_around_pattern == 6:
            drone.send_rc_control(int(20), int(0), int(0), int(0))
            fly_around_pattern = 9
            temp_fly = 7
        elif fly_around_pattern == 7:
            drone.send_rc_control(int(20), int(0), int(0), int(0))
            fly_around_pattern = 9
            temp_fly = 8
        elif fly_around_pattern == 8:
            drone.send_rc_control(int(20), int(0), int(0), int(0))
            fly_around_pattern = 9
            temp_fly = 0
        elif fly_around_pattern == 9:
            drone.send_rc_control(int(0), int(0), int(0), int(0))
            fly_around_pattern = temp_fly
#!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

####################################################################################
#           Main Block
####################################################################################
    
if __name__ == '__main__':
    #############################################
    #           SETUP and Initialization
    #############################################
    global is_flying, prev_time, curr_time
    is_flying = False
    #drone = tello.Tello('', 8889)
    cali_intr, cali_dist = intrinsic_parameter()    #fetch the calibration data
    drone = Tello()
    drone.connect()
    #cap = cv2.VideoCapture(1)
    time.sleep(10)

    x_pid = PID(kP=0.8, kI=0.00005, kD=0.28)  # Use tvec_x (tvec[i,0,0]) ----> control left and right
    z_pid = PID(kP=0.8, kI=0.0005, kD=0.2)  # Use tvec_z (tvec[i,0,2])----> control forward and backward
    y_pid = PID(kP=0.8, kI=0.0001, kD=0.15)  # Use tvec_y (tvec[i,0,1])----> control upward and downward
    yaw_pid = PID(kP=0.8,kI=0.0001, kD=0.15)
    x_pid.initialize()
    z_pid.initialize()
    y_pid.initialize()
    yaw_pid.initialize()

    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()

    ###################################
    #       Battery display
    ###################################
    prev_time = time.time()
    #battery = drone.get_battery()
    #print("Now battery: {}".format(battery))
    #curr_time = time.time()

    ####################################################################################
    #   Frame Loop
    ####################################################################################
    key = -1

    try:
        while True: 
            drone.streamon()
            #!frame = drone.read()
            #ret, frame = cap.read()
            frame = drone.get_frame_read()
            frame = frame.frame
            #!frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            markerIds = []
            markerCorners, markerIds, rejectedCandidates =cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
            
            correct_ready = False

            if markerIds is not None:
                
                frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)
                rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, cali_intr, cali_dist)
                for i in range(len(markerIds)):
                    frame = cv2.aruco.drawAxis(frame, cali_intr, cali_dist, rvec[i], tvec[i], 7.5)

            ########################################
            #  Highest control for keyboard control
            ########################################
            if key!=-1:
                #drone.send_rc_control(0, 0, 0, 0)
                keyboard(drone,key)
            ##########################################################################
            #       FSM states
            ##########################################################################
            else:
                #=======================================
                #       Forward 125 cm
                #=======================================
                if (current_state == 0 and is_flying):
                    print("state: {}".format(current_state))
                    drone.move("forward",125)
                    current_state = step_1_correct
                #=======================================
                #       correct to id 1
                #=======================================
                elif (current_state == step_1_correct):
                    print("state: {}".format(current_state))

                    target_idex = find_id(markerIds, 1)
                    if target_idex != -1:
                        correct_ready = correct_v2(rvec, tvec, target_idex, 70)
                    else:
                        #TODO
                        #fly_around()
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")

                    if (correct_ready == True):
                        current_state = step_1_down
                #=======================================
                #       connect to Step 2 & follow id 0
                #=======================================
                elif (current_state == step_1_down):
                    print("state: {}".format(current_state))
                    drone.move("down", 40)
                    drone.move("forward",65)
                    drone.move("up", 40)
                    current_state = step_2_follow

                elif (current_state == step_2_follow):
                    print("state: {}".format(current_state))

                    target_idex = find_id(markerIds, 0)
                    if target_idex != -1:
                        correct_v2(rvec, tvec, target_idex, 70, False)
                    else:
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")

                    target_idex_2 = find_id(markerIds, 5)
                    if target_idex_2 != -1:
                        counter_2 += 1
                    if ((counter_2 >= 100) and (target_idex == -1)):
                        current_state = step_2_finish
                        counter_2 = 0
                #=======================================
                #       connect to Step 3
                #=======================================
                elif (current_state == step_2_finish):
                    print("state: {}".format(current_state))
                    target_idex_2 = find_id(markerIds, 5)
                    if target_idex_2 != -1:
                        current_state = step_3_1st_correct    
                    else:
                        drone.move("forward", 25)
                        """
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")
                        """

                        """
                        drone.send_rc_control(0, -10, 0, 0)
                        
                        """
                #=======================================
                #       correct to id 5
                #=======================================
                elif (current_state == step_3_1st_correct):
                    print("state: {}".format(current_state))
                    target_idex = find_id(markerIds, 5)
                    if target_idex != -1:
                        correct_ready = correct_v2(rvec, tvec, target_idex, 70)
                    else:
                        #TODO
                        #fly_around()
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")

                    if (correct_ready == True):
                        current_state = step_3_1st_ready
                #=======================================
                #       dodge and go to find id 3
                #=======================================
                elif (current_state == step_3_1st_ready):
                    print("state: {}".format(current_state))
                    drone.move("left",70)
                    current_state = step_3_2nd_correct
                #=======================================
                #       correct to id 3
                #=======================================
                elif(current_state == step_3_2nd_correct):
                    print("state: {}".format(current_state))
                    target_idex = find_id(markerIds, 3)
                    if target_idex != -1:
                        correct_ready = correct_v2(rvec, tvec, target_idex, 70)
                    else:
                        #TODO
                        #fly_around()
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")

                    if (correct_ready == True):
                        current_state = step_3_2nd_ready
                #=======================================
                #       dodge and go to Step 4
                #=======================================
                elif (current_state == step_3_2nd_ready):
                    print("state: {}".format(current_state))
                    drone.move("right",125)
                    drone.rotate_counter_clockwise(30)
                    drone.move("forward",70)
                    current_state = step_4_correct
                #=======================================
                #       correct to id 4 and land
                #=======================================   
                elif (current_state == step_4_correct):

                    print("state: {}".format(current_state))

                    target_idex = find_id(markerIds, 4)
                    if target_idex != -1:
                        correct_ready = correct_v2(rvec, tvec, target_idex, 85)
                    else:
                        #TODO
                        #fly_around()
                        ################################
                        #   Floating if no instructions
                        ################################
                        if is_flying == True:
                            drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                        #now = time.ctime()
                        print("No instructions")
                
                    if (correct_ready == True):
                        drone.land()
                else:
                    print("state: out of state")
                    ################################
                    #   Floating if no instructions
                    ################################
                    if is_flying == True:
                        drone.send_rc_control(0, 0, 0, 0)      #Stop in the air
                    #now = time.ctime()
                    print("No instructions")
                
            cv2.imshow('frame', frame)
            key = cv2.waitKey(1)
            ########################################
            #   Display Battery
            ########################################    
            #battery_dis_per30s()
        else:
            print("fail to open film")

    except KeyboardInterrupt:
        #cap.release()
>>>>>>> d0c1458f300b017373d341987228b77d3c073f9e
        cv2.destroyAllWindows()