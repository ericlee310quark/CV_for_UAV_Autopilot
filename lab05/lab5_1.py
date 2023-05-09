import tello 
import cv2
import time

def main():
    drone = tello.Tello('', 8889)
    time.sleep(10)

    while True:
        frame = drone.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        cv2.imshow("drone", frame)

        key = cv2.waitKey(1)

        if key!=-1:
            drone.keyboard(key)
    cv2.destoryAllWindows()
if __name__ == '__main__':
    main()