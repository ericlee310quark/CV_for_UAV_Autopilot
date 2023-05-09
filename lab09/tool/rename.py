import os
import time
from xxlimited import new
addr=r'D:\py_env\UAV\Yolo_mark-master\x64\Release\data\img2_2'

file_list = os.listdir(addr)
base_index = 950
for i in range(220,590):

    select_jpg = addr + '\\'+ str(i)+'.jpg'
    select_txt = addr + '\\'+ str(i)+'.txt'
    
    new_index = base_index + i

    new_jpg_name = addr +'\\'+str(new_index)+'.jpg'

    new_txt_name = addr +'\\'+str(new_index)+'.txt'
    os.rename(select_jpg,new_jpg_name)
    print('now processed: {}--->{}'.format(select_jpg,new_jpg_name))
    time.sleep(0.01)
    os.rename(select_txt,new_txt_name)
    print('now processed: {}--->{}'.format(select_txt, new_txt_name))
    time.sleep(0.01)
print('all down')