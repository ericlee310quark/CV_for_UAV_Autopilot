import os.path as path
import numpy as np
import cv2
import glob
import time
#import tello


def detectTarget(cameraMatrix, dist, rMatrix, tvec, targetMarker, corners, markerIDs,zWorld = 0.0):
    '''
    測算目標marker中心在世界坐標系中的位置
    輸入:

    輸出:
        與markerIDs長度相等的列表,包含位置確定的目標坐標,未檢測到填None,例如[None,[x2,y2,z2]]
    '''
    if rMatrix==[]:
        return
    targets_count=len(targetMarker)
    if targets_count == 0:
        raise Exception('targets empty, areyou dou?')

    #創建與targetMarker相同尺寸的列表,用於存儲解算所得到目標的世界坐標
    targetsWorldPoint=[None] * targets_count

    for i in range(len(markerIDs)): #遍歷探測到的marker ID,
        markerIDThisIterate = markerIDs[i][0]
        if markerIDThisIterate in targetMarker: #如果是目標marker的ID
            #獲得當前處理的marker在targetMarker中的下標,用於填充targetsWorldPoint
            targetIndex = targetMarker.index(markerIDThisIterate)
        else:
            continue
        #計算marker中心的圖像坐標
        markerCenter = corners[i][0].sum(0)/4.0
        #畸變較正,轉換到相機坐標系,得到(u,v,1)
        #https://stackoverflow.com/questions/39394785/opencv-get-3d-coordinates-from-2d
        markerCenterIdeal=cv2.undistortPoints(markerCenter.reshape([1,-1,2]),cameraMatrix,dist)
        markerCameraCoodinate=np.append(markerCenterIdeal[0][0],[1])
        print('++++++++markerCameraCoodinate')
        print(markerCameraCoodinate)

        #marker的坐標從相機轉換到世界坐標
        markerWorldCoodinate = np.linalg.inv(rMatrix).dot((markerCameraCoodinate-tvec.reshape(3)) )
        print('++++++++markerworldCoodinate')
        print(markerWorldCoodinate)
        
        
        """    
        #將相機的坐標原點轉換到世界坐標系
        originWorldCoodinate = np.linalg.inv(rMatrix).dot((np.array([0, 0, 0.0])-tvec.reshape(3)) )
        #兩點確定了一條直線 (x-x0)/(x0-x1) = (y-y0)/(y0-y1) = (z-z0)/(z0-z1) 
        #當z=0時,算得x,y
        delta = originWorldCoodinate-markerWorldCoodinate
        #zWorld = 0.0
        xWorld = (zWorld-originWorldCoodinate[2])/delta[2] * delta[0] + originWorldCoodinate[0]
        yWorld = (zWorld-originWorldCoodinate[2])/delta[2] * delta[1] + originWorldCoodinate[1]
        targetsWorldPoint[targetIndex]=[xWorld,yWorld,zWorld]
        
        print('-=-=-=\n Target Position '+ str(targetsWorldPoint[targetIndex]) )
        pass

        """

    return markerWorldCoodinate





refMarkerArray={0: [0.0, 0.0, 0.0]}






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
                
                    print("rvec shape {}".format(rvec.shape))
                    print("tvec shape {}".format(tvec.shape))
                    
                    rvec, _ = cv2.Rodrigues(rvec)
                    

                    distance = detectTarget(intrinsic, distortion, rvec, tvec, [0], markerCorners, markerIds,zWorld = 0.0)
                    text ="{} x,y,z = {}".format(len(markerIds), distance)

                    cv2.putText(frame, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
               
                    """
                    for i in range(len(markerIds)):
                        frame = cv2.aruco.drawAxis(frame, intrinsic, distortion, rvec[i], tvec[i], 0.1)
                        tvec_ = tvec[0][0]
                        tvec_[0] *= 10
                        tvec_[1] *= 10
                        
                        text ="{} x,y,z = {}".format(len(markerIds), tvec_) 
                        
                        print(tvec_)
                        cv2.putText(frame, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)
                    """
                
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