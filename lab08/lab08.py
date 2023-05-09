import os.path as path
import numpy as np
import cv2
import glob
import time
import math
import dlib

refHuman =[[0.0,193,0.0],[91.0,193.0,0.0],[91.0,0.0,0.0],[0.0,0.0,0.0]]
refHuman = np.array(refHuman,dtype=np.float32)
refFace =[[0.0,14.0,0.0],[12.0,14.0,0.0],[12.0,0.0,0.0],[0.0,0.0,0.0]]
refFace = np.array(refFace,dtype=np.float32)



def createHOGDescriptor(frame):
    """Function creates the HOG Descriptor for human detection and returns the detections (rects)."""
    # Initialize OpenCV's HOG Descriptor and SVM classifier
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # Detect people in the image and return the bounding rectangles. Altering the parameters
    # in detectMultiScale() can affect the accuracy of detections. winStride refers to the
    # number of steps the sliding window moves in the x and y directions; the sliding window
    # is padded to improve accuracy; a smaller scale value will increase detection accuracy,
    # but also increase processing time
    #rects, weights = hog.detectMultiScale(frame, winStride=(4, 4),padding=(8, 8), scale=1.1)
    rects, weights = hog.detectMultiScale(frame, winStride=(4, 4),padding=(8, 8), scale=1.275)
    # For each of the rects detected in an image, add the values for the corners
    # of the rect to an array
    rects = np.array([[x, y, x + width, y + height] for (x, y, width, height) in rects])
    return rects


if __name__ == '__main__':
    #drone = tello.Tello('', 8889)
    #time.sleep(10)
    
    cap = cv2.VideoCapture(0)
    time.sleep(20)
    f = cv2.FileStorage('in_coeffs.xml', cv2.FILE_STORAGE_READ)
    intrinsic = f.getNode("intrinsic").mat()
    distortion = f.getNode("distortion").mat()
    
    print("intrinsic: {}".format(intrinsic))
    print("distortion: {}".format(distortion))
    
    #dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    #parameters = cv2.aruco.DetectorParameters_create()    
    try:
        if cap.isOpened():
            print("cap start")
            while True:
                
                FACE_BOX_COLOR= (0, 255, 0)
                ret, frame = cap.read()
                MUL = 1.25
                frame_output = frame.copy()
                
                frame2 =cv2.resize(frame, dsize=None, fx=1/MUL, fy=1/MUL)
                detector = dlib.get_frontal_face_detector()
                face_rects = detector(frame2, 0)
                
                for i, d in enumerate(face_rects):
                    x1 = round(int(d.left())*MUL)
                    y1 = round(int(d.top())*MUL)
                    x2 = round(int(d.right())*MUL)
                    y2 =  round(int(d.bottom())*MUL)

                    face_point = []
                    face_point.append([x1,y1])
                    face_point.append([x2,y1])
                    face_point.append([x2,y2])
                    face_point.append([x1,y2])
                    face_point2 = np.array(face_point,dtype=np.float64)

                    ret_val_f, rvec_f, tvec_f = cv2.solvePnP(refFace,face_point2,intrinsic,distortion)
                    
                    text ="face:{} cm".format(tvec_f[2])
                    print(text)
                    
                    if (y2-10<= 0):
                        text_y = 0
                    else:
                        text_y = y2
                    cv2.putText(frame_output, text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, FACE_BOX_COLOR, 1, cv2.LINE_AA)

                    frame_output = cv2.rectangle(frame_output, (x1, y1), (x2, y2), FACE_BOX_COLOR, 2)
                
                HUMAN_BOX_COLOR= (0, 0, 255)
                MUL = 1
                frame2 =cv2.resize(frame, dsize=None, fx=1/MUL, fy=1/MUL)
                rects = createHOGDescriptor(frame2)
                for (x1, y1, x2, y2) in rects:
                    x1 =round(x1*MUL)
                    y1 =round(y1*MUL)
                    x2 =round(x2*MUL)
                    y2 =round(y2*MUL)

                    human_point = []
                    human_point.append([x1,y1])
                    human_point.append([x2,y1])
                    human_point.append([x2,y2])
                    human_point.append([x1,y2])
                    human_point2 = np.array(human_point,dtype=np.float64)

                    ret_val_f, rvec_f, tvec_f = cv2.solvePnP(refHuman,human_point2,intrinsic,distortion)
                    text ="Human:{} cm".format(tvec_f[2])
                    print(text)

                    if (y2-10<= 0):
                        text_y = 0
                    else:
                        text_y = y2
                    cv2.putText(frame_output, text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, HUMAN_BOX_COLOR, 1, cv2.LINE_AA)


                    frame_output = cv2.rectangle(frame_output, (x1, y1), (x2, y2), HUMAN_BOX_COLOR, 2)
                
                cv2.imshow('frame_output', frame_output)
                #cv2.waitKey(33)
                key = cv2.waitKey(1)

        else:
            print("fail to open film")
    except KeyboardInterrupt:
        cap.release()
        cv2.destroyAllWindows()
    f.release()
