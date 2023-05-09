#to split film into colmap image folder format
#You can type "python film_split -h" to see how to use it
import cv2
import pathlib
import os
import argparse
from argparse import RawTextHelpFormatter
def parse_args():
    parser = argparse.ArgumentParser(formatter_class=RawTextHelpFormatter, description="Film Split Program\n\
    Require place the films in the same folder of this python script.\n\
    usage: \"python film_split.py -s -g abc.mp4 --gfsr=30 -t edf.mp4 --tfsr=10\"\nThis instruction will create a folder structure:\n\
    |-abc_gfsr_30\n\t|-ground\n\t\t|-ground00001.jpg\n\t\t| ground00002.jpg\n\t\t| ground00003.jpg\n\t\t|...\n\
    \t|-edf_tfsr_10\n\t\t|-images00001.jpg\n\t\t| images00002.jpg\n\t\t| images00003.jpg\n\t\t|...\n\
    usage2: \"python film_split.py -f gfh_gfsr_40 -g abc.mp4 --gfsr=30\"\nThis instruction will create a folder structure:\n\
    |-gfh_gfsr_40\n\t|-ground\n\t\t|-ground00001.jpg\n\t\t| ground00002.jpg\n\t\t| ground00003.jpg\n\t\t|...\n\
    usage3: \"python film_split.py -f xyz_gfsr_50 -t efg.mp4 --tfsr=20\"\nThis instruction will create a folder structure:\n\
    |-xyz_gfsr_50\n\t|-efg_tfsr_20\n\t\t|-images00001.jpg\n\t\t| images00002.jpg\n\t\t| images00003.jpg\n\t\t|...\n\
    ")
    parser.add_argument('-f', '--folder', type=str, help='img folder name')
    parser.add_argument('-s', '--ensamefolder', action='store_true', help='Booleen type. If it\'s True,\
 all file will be create in the same-ground-film-name folder, and the \'-f\' argument will not work.\nDefault is False.  Enter \'-s\' to enable')
    parser.add_argument('-g', '--gf', type=str, help='Enter ground film file name. (ex: \'-g xxx.mp4\')')
    parser.add_argument('--gfsr', default='20', type=int, help='Enter ground film frame sample rate. Take once per {n} frames. Default is 20. (ex: \'--gfsr=20\')')
    parser.add_argument('-t', '--tf', type=str, help='Enter test film file name. (ex: \'-t xxx.mp4\')')
    parser.add_argument('--tfsr', default='5', type=int, help='Enter test film frame sample rate. Take once per {n} frames. Default is 5. (ex: \'--tfsr=5\')')
    return parser.parse_args()

def get_images_from_video(imgs_folder, video_name, time_F, mode):
    video_images = []
    vc = cv2.VideoCapture(video_name)
    c = 1
    img_index = 1   #img index start from 000000.jpg
    if vc.isOpened(): #判斷是否開啟影片
        rval, video_frame = vc.read()
    else:
        rval = False
        print("error when starting video")

    while rval:   #擷取視頻至結束
        rval, video_frame = vc.read()
        if not rval:
            print("end or error occur")
            break
        if(c % time_F == 0): #每隔幾幀進行擷取
            print("select new frame to save as .jpg imgs: {}".format(img_index))
            width, height,_ = video_frame.shape
            video_frame = cv2.resize(video_frame,(int(height/2),int(width/2)))
            video_images.append(video_frame)
            if (mode=='ground'):
                
                img_name = ("ground%05d.jpg" % (img_index))

            elif(mode=='test'):
                img_name = ("images%05d.jpg" % (img_index))
            img_addr = imgs_folder+'\\'+img_name
            cv2.imwrite(img_addr, video_frame)
            img_index += 1       
        c += 1
    vc.release()
    
    return video_images


if __name__ == '__main__':
    args = parse_args()
    if args.ensamefolder:
        time_F = args.gfsr#time_F越小，取樣張數越多
        video_name = args.gf #影片名稱
        file_name = video_name.split('.')[0]+'_gfsr_'+str(time_F)
        op_addr = str(pathlib.Path(__file__).parent.absolute())+'\\'+file_name
    else:
        try:
            op_addr = str(pathlib.Path(__file__).parent.absolute())+'\\'+args.folder
        except:
            print("require set a foler name for ouput: (ex: use\"-f abc\" will create a \"abc\" folder to store the images)")

    if not os.path.isdir(op_addr):
            print("find no folder: {}".format(op_addr))
            os.mkdir(op_addr)
            print("create a new folder: {}".format(op_addr))

    if args.gf is not None:

        time_F = args.gfsr#time_F越小，取樣張數越多
        video_name = args.gf #影片名稱
        ground_imgs_addr = op_addr+'\\ground'
        if not os.path.isdir(ground_imgs_addr):
            print("find no folder: {}".format(ground_imgs_addr))
            os.mkdir(ground_imgs_addr)
            print("create a new folder: {}".format(ground_imgs_addr))
        video_images = get_images_from_video(ground_imgs_addr, video_name, time_F, 'ground') #讀取影片並轉成圖片
        print("All ground films are splilted into .jpg")
        print('img num: {}'.format(len(video_images)))

    if args.tf is not None:
        time_F = args.tfsr#time_F越小，取樣張數越多
        video_name = args.tf #影片名稱
        file_name = video_name.split('.')[0]+'_tfsr_'+str(time_F)
        test_imgs_addr = op_addr+'\\'+file_name
        if not os.path.isdir(test_imgs_addr):
            print("find no folder: {}".format(test_imgs_addr))
            os.mkdir(test_imgs_addr)
            print("create a new folder: {}".format(test_imgs_addr))
        video_images = get_images_from_video(test_imgs_addr, video_name, time_F, 'test') #讀取影片並轉成圖片
        print("All test films are splilted into .jpg")
        print('img num: {}'.format(len(video_images)))
