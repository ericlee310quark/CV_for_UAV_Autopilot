def keyboard(self, key):
    print("key:", key)
    distance = 0.9
    degree = 30
    if key == ord('1'):
        self.takeoff()
        if key == ord('2'):
            self.land()
        if key == ord('i'):
            self.move_forward(distance)
            print("forward!!!!")
        if key == ord('k'):
            self.move_backward(distance)
            print("backward!!!!")
        if key == ord('j'):
            self.move_left(distance)
            print("left!!!!")
        if key == ord('l'):
            self.move_right(distance)
            print("right!!!!")
        if key == ord('s'):
            self.move_down(distance)
            print("down!!!!")
        if key == ord('w'):
            self.move_up(distance)
            print("up!!!!")
        if key == ord('a'):
            self.rotate_cw(degree)
            print("rotate!!!!")
        if key == ord('d'):
            self.rotate_ccw(degree)
        print("counter rotate!!!!")
        if key == ord('5'):
            height = self.get_height()
            print(height)
        if key == ord('6'):
            battery = self.get_battery()
            print(battery)
