import MirrorBot_base
import MirrorBot_head

class MirrorBot():
    def __init__(self,base_serial_port,servo_serial_port) -> None:
        self.base = MirrorBot_base.base(serial_port=base_serial_port)
        self.head = MirrorBot_head.head(servo_serial_port=servo_serial_port)
    
        
