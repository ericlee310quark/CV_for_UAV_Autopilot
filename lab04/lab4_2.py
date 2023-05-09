import os.path as path
import numpy as np
import math
import cv2
"""

def to_mtx(img):
    H,V,C = img.shape
    mtr = np.zeros((V,H,C), dtype='int')
    for i in range(img.shape[0]):
        mtr[:,i] = img[i]
    
    return mtr


def to_img(mtr):
    V,H,C = mtr.shape
    img = np.zeros((H,V,C), dtype='int')
    for i in range(mtr.shape[0]):
        img[:,i] = mtr[i]
        
    return img
"""
def BiLinear_interpolation(img,dstH,dstW):
    scrH, scrW = img.shape[:2]
    img = np.pad(img,((0,1),(0,1),(0,0)),'edge')
    #print(img.shape)
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



def MywarpPerspective(img, M, dsize):
    width,height = dsize
    img_h, img_w = img.shape[:2]
    warp_pic = np.zeros((height,width, 3), dtype=np.uint8)
    
    for y in range(img_h):
        for x in range(img_w):
            p_y = int((M[1,0]*x+M[1,1]*y+M[1,2])/(M[2,0]*x+M[2,1]*y+M[2,2]))
            p_x = int((M[0,0]*x+M[0,1]*y+M[0,2])/(M[2,0]*x+M[2,1]*y+M[2,2]))

            warp_pic[p_y,p_x,:] = img[y,x,:]
    warp_pic =  BiLinear_interpolation(warp_pic,math.floor(img_h*2.7),math.floor(img_w*2.7))

    return warp_pic
    """
    mtr = to_mtx(img)
    R,C = dsize
    dst = np.zeros((R,C,mtr.shape[2]))
    for i in range(mtr.shape[0]):
        for j in range(mtr.shape[1]):
            res = np.dot(M, [i,j,1])
            i2,j2,_ = (res / res[2] + 0.5).astype(int)
            if i2 >= 0 and i2 < R:
                if j2 >= 0 and j2 < C:
                    dst[i2,j2] = mtr[i,j]
    
    return to_img(dst)
    """

if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    bgimg = cv2.imread(path.join('broadway.jpg'))
    img_corner =np.float32([[420,205],[640,90],[670,420],[423,430]])
    #w, h = bgimg.shape[:2]
    h, w = bgimg.shape[:2]

    print(h)
    #cap_corner=np.float32([[0,0],[w-1,0],[w-1,h/2-1],[0,h/2-1]])
    cap_corner=np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])

    pro_matrix = cv2.getPerspectiveTransform(cap_corner,img_corner)

    map_list = []
    
    for y in range(90, 431):
        for x in range(420,671):
            if(((x-670)/(670-430)-(y-420)/(420-430))>0):
                continue
            if(((x-423)/(430-420)-(y-430)/(430-205))<0):
                continue
            if(((x-420)/(640-420)-(y-205)/(90-205))<0):
                continue
            if(((x-640)/(670-640)-(y-90)/(420-90))>0):
                continue
            map_list.append([y,x])




    print(pro_matrix)
    try:
        if cap.isOpened():
            while True:

                ret, frame = cap.read()
                
                processed = MywarpPerspective(frame,pro_matrix,(w, h))
                """
                for y in range(h):
                    for x in range(w):
                        if not ((processed[y,x,:]==[0,0,0]).all()):
                            bgimg[y,x,:] = processed[y,x,:]
            
                """
                for point in map_list:
                    bgimg[point[0],point[1],:] = processed[point[0]+158+32,point[1]+272+54,:]

                #cv2.imshow('frame', frame)
                cv2.imshow('pic', bgimg)
                #cv2.imshow("processed",processed)
                cv2.waitKey(33)
   
        else:
            print("fail to open film")
    except KeyboardInterrupt:
        cap.release()
        cv2.destroyAllWindows()
 