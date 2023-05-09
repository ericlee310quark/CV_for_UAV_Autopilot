import os.path as path
import numpy as np
import cv2
import glob
w =9
h =6
objp = np.zeros((w*h,3),np.float32)
print(objp.shape)
objp[:,:2] = np.mgrid[0:w,0:h].T.reshape(-1,2)
print(objp.shape)
objpoints=[]
imgpoints= []
im =0
if __name__ == '__main__':
    imgs = glob.glob('*.jpg')
    for fimg in imgs:
        img = cv2.imread(fimg)
        gray_f = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_f,(w,h),None)
        if ret ==True:
            cv2.cornerSubPix(gray_f,corners,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
            objpoints.append(objp)
            imgpoints.append(corners)
            cv2.drawChessboardCorners(gray_f,(w,h),corners,ret)
            cv2.imshow('frame', gray_f)

            cv2.waitKey(33)
        im = gray_f.shape
    ret, cameraMatrix, distCoeffs, rvecs, tvecs =cv2.calibrateCamera(objpoints, imgpoints, im[::-1], None,None)
    
    
    print(cameraMatrix)
    print(distCoeffs)
    
    f = cv2.FileStorage('in_coeffs.xml', cv2.FILE_STORAGE_WRITE)
    f.write("intrinsic", cameraMatrix)
    f.write("distortion", distCoeffs)
    f.release()



    """
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        while True:

            

            ret, frame = cap.read()
            corner =[]
            grey_f = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret_, corner = cv2.findChessboardCorners(grey_f, (9,6),None)

            print(ret_)
            print(corner)
            if ret_:
                cv2.cornerSubPix(grey_f,corner,(11,11),(-1,-1),(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
                for i in range(len(corner)):
                    cv2.circle(grey_f,corner[i], 5, (0, 255, 0), 2)

            #ret, cameraMatrix, distCoeffs, rvecs, tvecs =
            
            #for i in range(len(corner)):
             #   cv2.circle(grey_f,corner[i], 5, cv2.Scalar(0, 255, 0), 2, 8, 0)

            cv2.imshow('frame', grey_f)

            cv2.waitKey(33)

            
           
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("fail to open film")
    """