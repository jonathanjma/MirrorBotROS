import time
import serial

class head:
    def __init__(self, LA_serial_port='/dev/ttyACM0', servo_serial_port='/dev/ttyACM1'):
        self.startup = False
        self.shutdown = None
        if LA_serial_port is not None:
            self.LA_serial_wheel = serial.Serial(LA_serial_port, 9600, timeout=1)
        if servo_serial_port is not None:
            self.servo_serial_wheel = serial.Serial(servo_serial_port, 9600, timeout=1)
        if (LA_serial_port is None) and (servo_serial_port is None):
            print("[WARN] NO dev on HEAD")
        
    def start(self):
        self.LA_serial_wheel.write(b'START|\n')
        self.servo_serial_wheel.write(b'START|\n')

    def stop(self):
        self.LA_serial_wheel.write(b'STOP|\n')
        self.servo_serial_wheel.write(b'STOP|\n')
        time.sleep(0.25)

    def LA_up(self, duration: int = 1000):
        command = f'U|{duration}\n'
        self.LA_serial_wheel.write(command.encode())
        time.sleep(duration / 1000)

    def LA_down(self, duration: int = 1000):
        command = f'D|{duration}\n'
        self.LA_serial_wheel.write(command.encode())
        time.sleep(duration / 1000)
    
    def yaw(self, rad_1: int, rad_2: int, timeout=0.1):
        command = f'yaw|{rad_1}|{rad_2}\n'
        self.servo_serial_wheel.write(command.encode())
        time.sleep(timeout)
    
    def tilt(self, rad_1: int, rad_2: int, timeout=0.1):
        command = f'tilt|{rad_1}|{rad_2}\n'
        self.servo_serial_wheel.write(command.encode())
        time.sleep(timeout)


if __name__ == "__main__":
    head_test = head(None,'/dev/ttyACM0')
    head_start_time = time.time()
    head_test.yaw(100,0)
    
