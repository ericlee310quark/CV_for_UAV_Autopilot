import os.path as path

import cv2

if __name__ == '__main__':
    img = cv2.imread(path.join('nctu_flag.jpg'))
   #123
    img2 = img.copy()
    height, width = img.shape[:2]
    print(height, width)
    # coefficient: https://medium.com/@s12121296simon/%E5%BD%B1%E5%83%8F%E8%99%95%E7%90%86-%E4%BA%8C-visual-c-clr-%E5%BD%A9%E8%89%B2%E5%9C%96%E7%89%87%E8%BD%89%E7%81%B0%E9%9A%8E-f6f9208277cb
    ALPHA = 1   #0.299 coefficient for Red
    BETA =  1   #0.587 coefficient for Green
    GAMMA = 1   #0.114 coefficient for Blue


    for y in range(height):
        for x in range(width):
            
            Blue, Green, Red = img[y,x,:]
            Blue = int(Blue)
            Green = int(Green)
            Red = int(Red)
            pixel = 0
            if (Blue>60 and ((Blue*0.77>Green) and (Blue*0.8>Red))):
                pixel = 0
                continue
            pixel = (Red*ALPHA+Green*BETA+Blue*GAMMA)/3
            for i in range(3):
                img2[y,x,i] = pixel

    cv2.imshow('new img', img)
    cv2.imshow('new img2', img2)
    cv2.imwrite('./pic/filtered_img.jpg', img2)
    cv2.waitKey()
    cv2.destroyAllWindows()