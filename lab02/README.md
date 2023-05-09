# Lab02
## 1. Histogram Equalization
- Before
    ![demo1_ori](./kifune.jpg)
- HE on RGB field
    ![HE_RGB](./pic/HE_RGB_img.jpg)
```
%python lab1_a.py
```
- HE on HSV field
    ![HE_HSV](./pic/HE_HSV_img.jpg)

## 2. Otsu thereshould
- Before
    ![demo2_ori](./input.jpg)
- After
    ![Otsu_thre](./pic/Otsu_threshold.jpg)

I calculated the threshold by writing tool script.
```
%python lab2_tool.py
```
Apply Otsu threshold:
```
%python lab2.py 
```