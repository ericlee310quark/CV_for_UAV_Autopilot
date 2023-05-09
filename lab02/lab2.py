import os.path as path
import numpy as np
import cv2

if __name__ == '__main__':
    
    img = cv2.imread(path.join('input.jpg'))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #print(img)
    height, width = img.shape[:2]
   
    #cv2.imshow('oringinal1', img_HSV)
    img2 = img.copy()
 

    intensity = np.zeros(256)
    

    for y in range(height):
        for x in range(width):
            intensity[img[y,x]]+=1
    intensity_norm = intensity.ravel()/intensity.sum()
    Q = intensity_norm.cumsum()
    bins = np.arange(256)
    Min_Var =np.inf
    thresh = -1
    #print(intensity)
    for index in range(1,256):
        p1, p2 = np.hsplit(intensity_norm,[index])
        q1, q2 = Q[index], Q[255]-Q[index]
        if q1<1.e-6 or q2<1.e-6:
            continue
        b1, b2 = np.hsplit(bins,[index])
        m1, m2 = np.sum(p1*b1)/q1, np.sum(p2*b2)/q2
        v1, v2 = np.sum(((b1-m1)**2)*p1)/q1, np.sum(((b2-m2)**2)*p2)/q2
        fn = v1*q1 +v2*q2

        if(fn<Min_Var):
            Min_Var =fn
            thresh = index

    for y in range(height):
        for x in range(width):
            if(int(img[y,x])<thresh):
                img2[y,x]= 0
            else:
                img2[y,x] =255

    
    img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    height_n, width_n = img2.shape[:2]

    print(height, width)

    print(height_n, width_n)

    #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('oringinal', img)
    cv2.imshow('new img2', img2)
    cv2.imwrite('./pic/Otsu_threshold.jpg',img2)
    cv2.waitKey()
    cv2.destroyAllWindows()

            


            
    