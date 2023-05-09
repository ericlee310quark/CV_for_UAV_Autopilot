import os.path as path
import numpy as np
import cv2


if __name__ == '__main__':
    #cap = cv2.VideoCapture(1)
    cap = cv2.VideoCapture(0)
    if cap.isOpened():

        text = []
        for i in range(20):
            str1 = str(i)+'.jpg'
            text.append(str1)

        ind = 0
        while True:

            ret, frame = cap.read()
            corner =[]
            grey_f = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
           

            cv2.imshow('frame', grey_f)

            cv2.waitKey(33)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.imwrite(text[ind], grey_f)
                ind+=1
           
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("fail to open film")
