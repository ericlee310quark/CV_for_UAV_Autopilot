import os.path as path
import numpy as np
import math
import cv2

SCALE = 3
def NN_interpolation(img, dstH, dstW):
    scrH, scrW =img.shape[:2]
    retimg = np.zeros((dstH, dstW, 3), dtype=np.uint8)
    for y in range(dstH-1):
        for x in range(dstW-1):
            select_x = round(x*(scrW/dstW))
            select_y = round(y*(scrH/dstH))
            retimg[y, x] = img[select_y, select_x]
    return retimg

def BiLinear_interpolation(img,dstH,dstW):
    scrH, scrW = img.shape[:2]
    img = np.pad(img,((0,1),(0,1),(0,0)),'edge')
    print(img.shape)
    retimg=np.zeros((dstH,dstW,3),dtype=np.uint8)
    for i in range(dstH):
        for j in range(dstW):
            scrx=(i+1)*(scrH/dstH)-1
            scry=(j+1)*(scrW/dstW)-1
            x=math.floor(scrx)
            y=math.floor(scry)
            u=scrx-x
            v=scry-y
            retimg[i,j]=(1-u)*(1-v)*img[x,y]+u*(1-v)*img[x+1,y]+(1-u)*v*img[x,y+1]+u*v*img[x+1,y+1]
    
    return retimg
if __name__ == '__main__':
    image = cv2.imread(path.join('lisa.jpeg'))
    image2 = NN_interpolation(image,image.shape[0]*SCALE, image.shape[1]*SCALE)
    image3 = BiLinear_interpolation(image,image.shape[0]*SCALE,image.shape[1]*SCALE)
    cv2.imshow('Original_Lisa', image)
    cv2.imshow('NN_interpolation', image2)
    cv2.imshow('Bilinear_Interpolation', image3)
    
    cv2.imwrite("./pic/NN_inter_by_Lab2a.jpg", image2)
    cv2.imwrite("./pic/NN_bilinear_by_Lab2a.jpg", image3)
    cv2.waitKey()
    cv2.destroyAllWindows()