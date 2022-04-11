import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from rclpy.qos import QoSPresetProfiles
from pynput import keyboard

# Flags
POS_X = 0b00000001
NEG_X = 0b00000010
POS_Y = 0b00000100
NEG_Y = 0b00001000
POS_Z = 0b00010000
NEG_Z = 0b00100000
POS_YAW = 0b01000000
NEG_YAW = 0b10000000


class KeyTeleop(Node):
    def __init__(self):
        super().__init__('key_teleop')
        
        self.flag = 0
        self.publisher = self.create_publisher(Int16, 'thrust', QoSPresetProfiles.get_from_short_key('system_default'))
        
    def send(self):
        msg = Int16()
        msg.data = self.flag
        
        self.publisher.publish(msg)
        
    def on_press(self, key):
        
        try:
            if key == keyboard.Key.up:
                self.flag |= POS_X
            elif key == keyboard.Key.down:
                self.flag |= NEG_X
            elif key == keyboard.Key.left:
                self.flag |= POS_Y
            elif key == keyboard.Key.right:
                self.flag |= NEG_Y
            elif key == keyboard.Key.space:
                self.flag |= POS_Z
            elif key == keyboard.Key.shift:
                self.flag |= NEG_Z
            elif key.char == 'r':
                self.flag |= NEG_YAW
            elif key.char == 'e':
                self.flag |= POS_YAW
            elif key.char == 'q':
                self.flag = 0
            else:
                raise AttributeError
        except AttributeError:
            pass
        else:
            self.send()
    
    def on_release(self, key):
        try:
            if key == keyboard.Key.up:
                self.flag &= ~POS_X
            elif key == keyboard.Key.down:
                self.flag &= ~NEG_X
            elif key == keyboard.Key.left:
                self.flag &= ~POS_Y
            elif key == keyboard.Key.right:
                self.flag &= ~NEG_Y
            elif key == keyboard.Key.space:
                self.flag &= ~POS_Z
            elif key == keyboard.Key.shift:
                self.flag &= ~NEG_Z
            elif key.char == 'r':
                self.flag &= ~NEG_YAW
            elif key.char == 'e':
                self.flag &= ~POS_YAW
            elif key.char == 'q':
                self.flag = 0
            else:
                raise AttributeError
        except AttributeError:
            pass
        else:
            self.send()
        
        
def main(args=None):
    rclpy.init(args=args)
    node = KeyTeleop()
    
    keyboard.Listener(on_press=node.on_press, on_release=node.on_release).start()
    rclpy.spin(node)
        
    node.destroy_node()
    rclpy.shutdown()
            

if __name__ == '__main__':
    main()