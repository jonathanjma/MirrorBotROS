import serial
import time

class base:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Give some time for the serial connection to initialize

    def send_command(self, pwm1, pwm2):
        """Sends a command to the Arduino to control the base. Serial: B|PWM1|PWM2"""
        if -255 <= pwm1 <= 255 and -255 <= pwm2 <= 255:
            command = f"B|{int(pwm1)}|{int(pwm2)}\n"
            self.serial_connection.write(command.encode())
            print(f"Sent command: {command.strip()}")
        else:
            raise ValueError("PWM values must be between -255 and 255")
    def stop(self):
        """
        stop the base
        """
        self.serial_connection.write(b'X\n')
        time.sleep(0.2)

    def move_with(self, left_PWM, right_PWM):
        command = f'PWM|{left_PWM}|{right_PWM}\n'
        self.serial_connection.write(command.encode())

# Example usage
if __name__ == "__main__":
    # e.g., '/dev/ttyUSB0' on Linux for serial ports
    robot = base(port='/dev/ttyACM0', baudrate=115200)

    # Move the robot
    robot.send_command(150,150)
    time.sleep(2)  # Move forward for 2 seconds
    robot.turn_left(150)
    time.sleep(1)  # Turn left for 1 second
    robot.stop()  # Stop the robot

    # Close the connection
    robot.close()
