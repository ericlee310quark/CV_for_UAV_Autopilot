#to split film into orb image folder format
#use "python orb_film_split_v2.py -h" to see what arguments it need
#if you type "python orb_film_split_v2.py -t abc.mp4 --tfsr=20",
#it will create a folder:
#   --abc_sr_20
#       |--rgb
#           |images00001.jpg
#           |images00002.jpg
#           |images00003.jpg
#            ...
#       |rgb.txt

import cv2
import pathlib
import os
import numpy as np
import argparse
from argparse import RawTextHelpFormatter

def parse_args():
    parser = argparse.ArgumentParser(formatter_class=RawTextHelpFormatter, description="ORB test Film Split Program\n ex: python orb_film_split.py -t abc.mp4 --tfsr=20")
    parser.add_argument('-t', '--testfilm', type=str, required=True, help='test film name')
    parser.add_argument('--tfsr', default='5', type=int, help='Enter test film frame sample rate. Take once per {n} frames. Default is 5. (ex: \'--tfsr=5\')')
    return parser.parse_args()

    
def get_info_from_video(folder_addr, video_name, time_F):
    video_info = []
    vc = cv2.VideoCapture(video_name)
    c = 1
    time_unit = 1.0/30.0
    time_stamp = 0.0
    img_index = 1   
    folder_addr = folder_addr + '/'+folder_name
    if not os.path.isdir(folder_addr):
        print("find no folder: {}".format(folder_addr))
        os.mkdir(folder_addr)
        print("create a new folder: {}".format(folder_addr))

    if vc.isOpened():
        rval, video_frame = vc.read()
        time_stamp += time_unit
    else:
        rval = False
        print("error when starting")

    while rval:
        rval, video_frame = vc.read()
        if not rval:
            print("error or the end of the video")
            break
        if(c % time_F == 0): #每隔幾幀進行擷取
            print("select new frame to save as .jpg imgs: {}".format(img_index))
            img_name = ("images%05d.jpg" % (img_index))
            img_addr = folder_addr+'/'+img_name
            cv2.imwrite(img_addr, video_frame)
            timestamp_str = ('%.6f'%time_stamp)
            info = timestamp_str +' '+img_addr.split('/')[-2]+'/'+img_name+'\n'
            video_info.append(info)            
            img_index += 1
        time_stamp += time_unit
        c += 1
    vc.release()
    
    return video_info


if __name__ == '__main__':
    args = parse_args()
    current_addr = str(pathlib.Path(__file__).parent.absolute())
    time_F = args.tfsr
    video_name =  args.testfilm 
    folder_name = video_name.split('.')[-2].strip('/')+'_sr_'+str(time_F)
    folder_addr = current_addr+'/'+folder_name

    if not os.path.isdir(folder_addr):
        print("find no folder: {}".format(folder_addr))
        os.mkdir(folder_addr)
        print("create a new folder: {}".format(folder_addr))


    text_name = current_addr+'/'+folder_name+'/'+'rgb.txt'
    f1 = open(text_name,'w+')
    f1.write('# color images\n')
    line = '# file: \''+folder_name+'.bag\'\n'
    f1.write(line)
    f1.write('# timestamp filename\n')

    video_info = get_info_from_video(folder_addr, video_name, time_F)
    print("All films are splilted into .png")
    print('img num: {}'.format(len(video_info)))
    print('start writing .txt')
    for line in video_info:
        f1.write(line)
    print('Finish writing')
    f1.close
