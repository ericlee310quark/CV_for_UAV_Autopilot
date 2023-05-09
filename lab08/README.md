# Lab08
## HOG(Histogram of Oriented Gradient)
* use openCV lib
```
# initialize the HOG descriptor/person detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv.HOGDescriptor_getDefaultPeopleDetector())
rects, weights = hog.detectMultiScale(src, #輸入圖/
                                    winStride, #在圖上抓取特徵時窗口的移動大小/
                                    scale, #抓取不同scale (越小就要做越多/useMeanshiftGrouping = False)
```

## Dlib Face Detection
* python >=3.7
* ```pip instal cmake```
* ```pip install dlib```
```
import dlib
detector = dlib.get_frontal_face_detector()
face_rects = dectector(img, 0)
#Fetch all results
for i, d in enumerate(face_rects):
    x1 = d.left()
    y1 = d.top()
    x2 = d.right()
    y2 = d.bottom()
```
* Draw a rectangle
* ``` image = cv2.rectangle(img, start_point, end_point, color, thickness)```
* Depth prediction
    + Implement by ```cv2.solvePnP()```

## Run Together
* run ```python lab08.py```