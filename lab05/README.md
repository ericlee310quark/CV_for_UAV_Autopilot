# Lab05
## 1. Tello EDU control
- Tello EDU control
https://github.com/dji-sdk/Tello-Python
https://github.com/dji-sdk/Tello-Python/tree/master/Tello_Video
    * Written in Python 2.7
### I. Modify /tello.py
* comment ```def _h264_decode``` and ```def _receive_video_thread```
* Add ```import cv2```
* Add the following:
    ```
    def _receive_video_thread(self):
        video_ip = "udp://{}:{}".format("0.0.0.0", 11111)
        video_capture = cv2.VideoCapture(video_ip)
        retval, self.frame = video_capture.read()
        while retval:
            retval, frame = video_capture.read()
            self.frame = frame[..., ::-1] # From BGR to RGB
    ```
* <font color=#FF0000>Notice</font> The unit in this lib is <font color=#FF0000>"Meter"</font>.
* modify /tello_control_ui.py
    ```
    #import Tkinter as tki
    #from Tkinter import Toplevel, Scale

    import tkinter as tki
    from tkinter import Toplevel, Scale
    ```
    ```
    #except RuntimeError, e:
    except RuntimeError as e:
    ```
* Add brackets in all ```print()``` function
### II. Run ```lab5_1.py```
* You can use computer keyboard to control UAV.
## 2. Measure distance with Web camera (use AR Marker)
### I. Camera Calibration
* use ```/lab04``` files to get web camera parameters
* run ```read_xml.py```
