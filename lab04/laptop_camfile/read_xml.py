import os.path as path
import numpy as np
import cv2
import glob
import time
import math
#import tello






#refMarkerArray={0: [0.0, 0.0, 0.0]}






if __name__ == '__main__':
    #drone = tello.Tello('', 8889)
    #time.sleep(10)
    t1= 0
    cap = cv2.VideoCapture(0)
    time.sleep(20)
    f = cv2.FileStorage('in_coeffs.xml', cv2.FILE_STORAGE_READ)
    intrinsic = f.getNode("intrinsic").mat()
    distortion = f.getNode("distortion").mat()
    
    print("intrinsic: {}".format(intrinsic))
    print("distortion: {}".format(distortion))
    
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters_create()    
    try:
        if cap.isOpened():
            print("cap start")
            while True:

                ret, frame = cap.read()
                
                #frame = drone.read()
                #frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                markerIds = []
                markerCorners, markerIds, rejectedCandidates =cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)
                if (markerIds!=None):
                
                    frame = cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIds)

                    rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 15, intrinsic, distortion) 
                
                    #print("rvec shape {}".format(rvec.shape))
                    #print("tvec shape {}".format(tvec.shape))
                    
                    #rvec, _ = cv2.Rodrigues(rvec)
                    

                    #distance = detectTarget(intrinsic, distortion, rvec, tvec, [0], markerCorners, markerIds,zWorld = 0.0)
                    #text ="{} x,y,z = {}".format(len(markerIds), distance)

                    #cv2.putText(frame, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
               
                    
                    for i in range(len(markerIds)):
                        frame = cv2.aruco.drawAxis(frame, intrinsic, distortion, rvec[i], tvec[i], 7.5)
                        #tvec_ = tvec[0][0]
                        #tvec_[0] *= 10
                        #tvec_[1] *= 10
                        

                        rvec_3x3,_ = cv2.Rodrigues(rvec[i])
                        #z_base = np.array([[0],[0],[1]])
                        """
                        z = np.array([rvec_3x3[0][2],rvec_3x3[1][2],rvec_3x3[2][2]])
                        z = np.array([z[0],0,z[2]])
                        degree = math.atan2(z[0],z[2])
                        degree = math.degrees(degree)
                        yaw_update = degree - 90
                        yaw_update *=-1
                        """
                        

                       
                        rvec,_ = cv2.Rodrigues(rvec)
                        z_base = np.array([[0],[0],[1]])
                        rvec_zbase = rvec.dot(z_base)

                        x_project = rvec_zbase[0]
                        z_project = rvec_zbase[2]
                        angle_diff= math.atan2(float(z_project), float(x_project))*180/math.pi + 90
                        text ="{} x,y,z = {}".format(len(markerIds), angle_diff) 
                        
                        print(angle_diff)
                        
                        cv2.putText(frame, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)

                
                cv2.imshow('frame', frame)
                cv2.waitKey(33)
                key = cv2.waitKey(1)

                #if key!=-1:
                #    drone.keyboard(key)
        else:
            print("fail to open film")
    except KeyboardInterrupt:
        cap.release()
        cv2.destroyAllWindows()
    f.release()