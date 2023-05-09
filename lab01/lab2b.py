import os.path as path
import numpy as np
import cv2

if __name__ == '__main__':
    img = cv2.imread(path.join('lisa.jpeg'))
    
    height, width = img.shape[:2]

    times = input()
    times = int(times)
    img2 = np.zeros((times*height, times*width, 3), dtype=np.uint8)
    height_n, width_n = img2.shape[:2]

    print(height, width)

    print(height_n, width_n)

    for y in range(times*height-1):
        for x in range(times*width-1):
            y_select = 0
            x_select = 0
            x_select = int(x/times)
            y_select = int(y/times)
            xtimes = (x%times)/times
            ytimes = (y%times)/times

            if((y_select != height-1) and (x_select != width-1)):
                img2[y,x,:] = (1-ytimes)*(1-xtimes)*img[y_select, x_select,:] + (ytimes)*(1-xtimes)*img[y_select+1, x_select,:] + (1-ytimes)*(xtimes)*img[y_select, x_select+1,:] + (ytimes)*(xtimes)*img[y_select+1, x_select+1,:]
            elif((y_select != height-1) and (x_select == width-1)):
                img2[y,x,:] = (1-ytimes)*img[y_select, x_select,:] + (ytimes)*img[y_select+1, x_select,:]
            elif((y_select == height-1) and (x_select != width-1)):
                img2[y,x,:] = (1-xtimes)*img[y_select, x_select,:] + (xtimes)*img[y_select, x_select+1,:]
            elif((y_select == height-1) and (x_select == width-1)):
                img2[y,x,:] = img[y_select, x_select,:]
            
    cv2.imshow('new img', img)
    cv2.imshow('new img2', img2)
    cv2.waitKey()
    cv2.destroyAllWindows()
    cv2.imwrite('img4.jpg', img2)