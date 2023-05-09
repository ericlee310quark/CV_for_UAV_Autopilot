import os.path as path
import numpy as np
import cv2
import glob
import time

def intrinsic_parameter():
    f = cv2.FileStorage('in_coeffs.xml', cv2.FILE_STORAGE_READ)
    intr = f.getNode("intrinsic").mat()
    dist = f.getNode("distortion").mat()
    f.release()
    return intr, dist


if __name__ == '__main__':
    #drone = tello.Tello('', 8889)
    
  
    #cap = cv2.VideoCapture(1)
    intr, dist = intrinsic_parameter()

    print("intrinsic: {}".format(intr))
    print("distortion: {}".format(dist))
    