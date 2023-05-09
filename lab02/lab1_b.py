import os.path as path
import numpy as np
import cv2

if __name__ == '__main__':
    img = cv2.imread(path.join('kifune.jpg'))
    
    height, width = img.shape[:2]

    pixels = height * width
    img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #cv2.imshow('oringinal1', img_HSV)
    img2 = img_HSV.copy()
    
    #img2 = np.zeros((3*height, 3*width, 3), dtype=np.uint8)
    
    intensity = np.zeros((256, 4))
    

    for y in range(height):
        for x in range(width):
            intensity[int(img_HSV[y,x,2]), 0]+=1
            
            
    for index in range(256):
        if index == 0:
            intensity[0, 1] = intensity[0,0]
            
        else:
            intensity[index, 1] = intensity[index,0] + intensity[index-1,1]

    
    for index in range(256):
        intensity[index, 2] = int(intensity[index, 1]/pixels*256)
    
    
    for y in range(height):
        for x in range(width):
            img2[y,x,2] = intensity[int(img_HSV[y,x,2]),2]
            
            
    img2 = cv2.cvtColor(img2, cv2.COLOR_HSV2BGR)



    height_n, width_n = img2.shape[:2]

    print(height, width)

    print(height_n, width_n)


    cv2.imshow('oringinal', img)
    cv2.imshow('new img2', img2)
    cv2.imwrite('./pic/HE_HSV_img.jpg',img2)
    cv2.waitKey()
    cv2.destroyAllWindows()

            


            
    