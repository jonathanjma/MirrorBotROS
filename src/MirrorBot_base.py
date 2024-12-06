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

    def move_forward(self, speed=200):
        """Moves the robot forward."""
        self.send_command(speed, speed)

    def move_backward(self, speed=200):
        """Moves the robot backward."""
        self.send_command(-speed, -speed)

    def turn_left(self, speed=200):
        """Turns the robot left by setting left motor speed to negative and right motor speed to positive."""
        self.send_command(-speed, speed)

    def turn_right(self, speed=200):
        """Turns the robot right by setting right motor speed to negative and left motor speed to positive."""
        self.send_command(speed, -speed)

    def stop(self):
        """Stops the robot by setting both motor speeds to 0."""
        self.send_command(0, 0)

    def close(self):
        """Closes the serial connection."""
        self.serial_connection.close()
    def start(self):
        pass

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
