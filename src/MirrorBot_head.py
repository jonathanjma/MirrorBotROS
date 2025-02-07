#!/usr/bin/env python3
import time
import serial

class head:
    def __init__(self, servo_serial_port='/dev/ttyACM1'):
        self.startup = False
        self.shutdown = None
        self.home_pos = 0
        self.servo_serial_wheel = serial.Serial(servo_serial_port, 9600, timeout=1)
        
    def homes(self):
        print("Mirror Home")
        t_start = time.time()
        while((time.time()-t_start) < 1):
            self.servo_serial_wheel.write((f"MR_X {0}\nMR_Y {0}\nML_X {0}\nML_Y {0}\n").encode())
            time.sleep(0.25)
    
    def mirror_move(self,alpha_r:int,gamma_r:int,alpha_l:int,gamma_l:int):
        t_start = time.time()
        while((time.time()-t_start) < 1):
            self.servo_serial_wheel.write((f"MR_X {alpha_r}\nMR_Y {gamma_r}\nML_X {alpha_l}\nML_Y {gamma_l}\n").encode())
            time.sleep(0.25)

    def mirror_nod(self, r:bool, l:bool, duration:int):
        t_start = time.time()
        nod_by = 30
        while ((time.time()-t_start) < duration):
            self.mirror_move(0, (self.home_pos + nod_by) * int(r), 0, (self.home_pos + nod_by) * int(r))
            time.sleep(0.8)
            self.mirror_move(0, (self.home_pos - nod_by) * int(r), 0, (self.home_pos - nod_by) * int(r))
            time.sleep(0.8)
        self.homes()

if __name__ == "__main__":
    head_test = head('/dev/ttyACM1')
    print("new_pos")
    # head_test.mirror_move(20,45,-20,-45)    
    # time.sleep(6)
    # head_test.homes()
    # time.sleep(3)
    head_test.mirror_nod(1,1,5)




# two mirror both face to me

# then

# both face to other

# then

# switch mirror for reflection

# then

# face to me and myself

# then

# both left mirror down

    