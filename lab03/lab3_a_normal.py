import os.path as path
import numpy as np
import cv2

def equal(n, parent):
    i = int(n)
    while(parent[i]!=0):
        i = int(parent[i])
    return i

def union_label(x, y, parent):
    i = int(x)
    j = int(y)
   # print ((parent[i]))
    while(parent[i]!=0):
        i = int(parent[i])
    while(parent[j]!=0):
        j = int(parent[j])
    if(i!=j):
        parent[i] = j

class detection():
    def __init__(self, id, y, x):
        self.id = int(id)
        self.left = int(x)
        self.right = int(x)
        self.up = int(y)
        self.bottom = int(y)
        self.area = 1

    def update(self, y, x):
        if int(x) < self.left:
            self.left = int(x)
        if int(x) > self.right:
            self.right = int(x)
        if int(y) > self.bottom:
            self.bottom = int(y)
        if int(y) < self.up:
            self.up = int(y)
        self.area += 1



if __name__ == '__main__':
    cap = cv2.VideoCapture('vtest.mp4')
    backSub = cv2.createBackgroundSubtractorMOG2()
    
    while cap.isOpened():
        ret, frame = cap.read()
        fgmask = backSub.apply(frame)
        shadowval = backSub.getShadowValue()
        #print(shadowval)
        ret, nmask = cv2.threshold(fgmask, shadowval, 255, cv2.THRESH_BINARY)
        #gmask = cv2.GaussianBlur(fgmask,(5,5),0)
        height, width = nmask.shape[:2]
        #nmask=cv2.GaussianBlur(nmask,(3,3),1)
        #nmask=cv2.medianBlur(nmask,3)
        #print(height, width) 576, 720
        label = 1
        #img = nmask.copy()
        a1 = np.zeros((576,720),dtype = int) #建構一個跟圖片一樣像素數目的array
        #record_position = []
        flag = False    #用來更新label的flag
        la = np.zeros(5000,dtype = int) #假設label 2 = label 4, la[4] = 2
        for h in range(height):        #1Pass
            for w in range(width):
                if(nmask[h][w] == 255):
                    flag = True
                    if((h-1)>=0 and a1[h-1][w]>0): #檢查上面
                        a1[h][w] = a1[h-1][w]         #優先跟上面一樣
                        '''
                        if((w-1)>=0):
                            if(a1[h][w-1]>0):              #左邊有衝突
                                union_label(a1[h][w-1], a1[h][w], la)   #記錄關係
                        '''
                        if((w-1)>=0 and a1[h][w-1]>0):      #左邊有衝突
                            union_label(a1[h][w-1], a1[h][w], la)   #記錄關係
                        
                    else:
                        a1[h][w] = label
                
                elif(nmask[h][w]==0 and flag == True):  #進入黑色區塊(每次從白色出去後，第一次進去時，更新label)
                    label += 1
                    flag = False
            flag = False #換行不連續
            label += 1
        
        roots =[]
        for h in range(height):         #2pass
            for w in range(width):
                if (a1[h][w]!=0):
                    a1[h][w] = equal(a1[h][w], la)
                    find = False
                    index_record = 0
                    for index in range(len(roots)):
                        if (roots[index].id == int(a1[h][w])):
                            index_record =index                          
                            find = True
                            break
                    if find:
                        roots[index_record].update(int(h),int(w))
                    else:
                        det = detection(int(a1[h][w]), int(h),int(w))
                        roots.append(det)
        print("roots: {}".format(len(roots)))
        Max_area = 0
        for ind in range(len(roots)):           #劃出bounding box
            if int(roots[ind].area)>Max_area:
                Max_area = roots[ind].area
                

            #print("area: {}".format(roots[ind].area))
            if int(roots[ind].area) >=350 :#and int(roots[ind].area)<11000:
                cv2.rectangle(frame, (int(roots[ind].left),int(roots[ind].up)), (int(roots[ind].right), int(roots[ind].bottom)), (255, 0, 255), 1)
        roots.clear()
        print("Max area: {}".format(Max_area)) 

        """
                if(nmask[height][width] == 255): #如果是白色的
                    if((height - 1 < 0) and (width - 1 < 0)):
                        a1[height][width] = label
                        label += 1
                    elif((height - 1 >= 0) and (width - 1 < 0)):
                        if(a1[height-1][width] != 0):
                            a1[height][width] = a1[height-1][width]
                        else:
                            a1[height][width] = label
                            label += 1
                    elif((height - 1 < 0) and (width - 1 >= 0)):
                        if(a1[height][width-1] != 0):
                            a1[height][width] = a1[height][width-1]
                        else:
                            a1[height][width] = label
                            label += 1
                    else:
                        if(a1[height][width-1] < a1[height-1][width]):
                            if(a1[height][width-1] != 0):
                                a1[height][width] = a1[height][width-1]
                                temp = a1[height-1][width]
                                la[int(temp)] = a1[height][width-1]
                            else:
                                a1[height][width] = a1[height-1][width]
                        elif(a1[height][width-1] > a1[height-1][width]):
                            if(a1[height-1][width] != 0):
                                a1[height][width] = a1[height-1][width]
                                temp = a1[height][width-1]
                                la[int(temp)] = a1[height-1][width]
                            else:
                                a1[height][width] = a1[height][width-1]
                        elif(a1[height][width-1] == a1[height-1][width]):
                            a1[height][width] = a1[height-1][width]
                        else:    
                            a1[height][width] = label
                            label +=1
        
        for height in range(h):
            for width in range(w):
                a1[height][width] = a1[height][width] *50
        """
        cv2.imshow("frame", frame)
        cv2.imshow("nmask", nmask)
        cv2.waitKey(33)
        if(cv2.waitKey(1) == ord('q')):
            break
        
    cap.release()
    cv2.destroyAllWindows()
