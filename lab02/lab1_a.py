import os.path as path
import numpy as np
import cv2

if __name__ == '__main__':
    img = cv2.imread(path.join('kifune.jpg'))
    
    height, width = img.shape[:2]

    pixels = height * width
    img2 = img.copy()
    #img2 = np.zeros((3*height, 3*width, 3), dtype=np.uint8)

    dia_blue = np.zeros((256, 4))
    dia_red = np.zeros((256, 4))
    dia_green = np.zeros((256, 4))

    for y in range(height):
        for x in range(width):
            dia_blue[int(img[y,x,0]), 0]+=1
            dia_red[int(img[y,x,1]), 0]+=1
            dia_green[int(img[y,x,2]), 0]+=1
            
    for index in range(256):
        if index == 0:
            dia_blue[0, 1] = dia_blue[0,0]
            dia_red[0, 1] = dia_red[0,0]
            dia_green[0, 1] = dia_green[0,0]
        else:
            dia_blue[index, 1] = dia_blue[index,0]+ dia_blue[index-1,1]
            dia_red[index, 1] = dia_red[index,0]+dia_red[index-1,1]
            dia_green[index, 1] = dia_green[index,0]+dia_green[index-1,1]
        
    for index in range(256):
        dia_blue[index, 2] = int(dia_blue[index, 1]/pixels*255)
        dia_red[index, 2] = int(dia_red[index,1]/pixels*255)
        dia_green[index, 2] = int(dia_green[index,1]/pixels*255)
    
    
    
    for y in range(height):
        for x in range(width):
            img2[y,x,0] = dia_blue[int(img2[y,x,0]),2]
            img2[y,x,1] = dia_red[int(img2[y,x,1]),2]
            img2[y,x,2] = dia_green[int(img2[y,x,2]),2]
            



    height_n, width_n = img2.shape[:2]

    print(height, width)

    print(height_n, width_n)


    cv2.imshow('oringinal', img)
    cv2.imshow('new img2', img2)
    cv2.imwrite('./pic/HE_RGB_img.jpg',img2)

    cv2.waitKey()
    cv2.destroyAllWindows()

            


            
    