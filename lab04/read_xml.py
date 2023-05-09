import os.path as path
import numpy as np
import cv2
import glob
import time
#import tello






#refMarkerArray={0: [0.0, 0.0, 0.0]}






if __name__ == '__main__':
    #drone = tello.Tello('', 8889)
    #time.sleep(10)
    
    cap = cv2.VideoCapture(1)
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
                        frame = cv2.aruco.drawAxis(frame, intrinsic, distortion, rvec[i], tvec[i], 0.1)
                        #tvec_ = tvec[0][0]
                        #tvec_[0] *= 10
                        #tvec_[1] *= 10
                        
                        text ="{} x,y,z = {}".format(len(markerIds), tvec) 
                        
                        print(tvec)
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