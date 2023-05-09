import os.path as path
import numpy as np
import cv2


if __name__ == '__main__':
    cap = cv2.VideoCapture(path.join('vtest.mp4'))
    backSub = cv2.createBackgroundSubtractorMOG2()
    #ret, frame = cap.read()
    if cap.isOpened():
        while True:
            ret, frame = cap.read()
            
            
            fgmask = backSub.apply(frame)
            
            
            
            shadowval = backSub.getShadowValue()
            print(shadowval)
            ret, nmask = cv2.threshold(fgmask, shadowval, 255, cv2.THRESH_BINARY)
            
            n1 = median = cv2.bilateralFilter(nmask,9,75,75)#cv2.medianBlur(nmask,1)#cv2.GaussianBlur(nmask,(3,3),1)
            cv2.imshow("frame", nmask)
            cv2.imshow("n1", n1)
            
            cv2.waitKey(33)



            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("fail to open film")
