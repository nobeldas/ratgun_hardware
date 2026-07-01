import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from pynput import keyboard
import sys

class TurretTeleopNode(Node):
    def __init__(self):
        super().__init__('turret_teleop')
        
        self.publisher_ = self.create_publisher(Int32MultiArray, 'turret_commands', 10)
        
        # Initial states
        self.laser_state = 0
        self.pan = 90  # Centered
        self.tilt = 90 # Centered
        
        # Step size (Lowered to 2 for smooth timer-based movement)
        self.step = 1 

        # This set will hold all keys currently being held down
        self.pressed_keys = set()

        self.print_instructions()

        # Create a timer that runs the control loop 20 times per second (20Hz)
        self.timer = self.create_timer(0.01, self.control_loop)

        # Start the pynput keyboard listener in the background
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

    def print_instructions(self):
        print("""
        ------------------------------------
         MANUAL TURRET OVERRIDE ACTIVE 
        ------------------------------------
        UP / DOWN ARROWS    : Tilt Up / Down
        LEFT / RIGHT ARROWS : Pan Left / Right
        [HOLD] SPACEBAR     : Fire Gel Blaster
        
        Q                   : Quit
        ------------------------------------
        """)

    def publish_state(self):
        """Helper function to publish and print the current state."""
        msg = Int32MultiArray()
        msg.data = [self.pan, self.tilt, self.laser_state]
        self.publisher_.publish(msg)
        
        fire_status = " FIRE! " if self.laser_state == 1 else "   SAFE   "
        sys.stdout.write(f"\rAiming -> Pan: {self.pan:3}° | Tilt: {self.tilt:3}° | Blaster: {fire_status}")
        sys.stdout.flush()

    def on_press(self, key):
        # Add the key to our tracker the moment it is pressed
        self.pressed_keys.add(key)
        
        # Q to quit
        if hasattr(key, 'char') and key.char is not None and key.char.lower() == 'q':
            print("\nExiting manual control...")
            rclpy.shutdown()
            return False 

    def on_release(self, key):
        # Remove the key from our tracker the moment it is released
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)

    def control_loop(self):
        """This loop runs constantly in the background and checks the key states."""
        moved = False
        
        # Check Tilt
        if keyboard.Key.up in self.pressed_keys:
            self.tilt = min(111, self.tilt + self.step)
            moved = True
        if keyboard.Key.down in self.pressed_keys:
            self.tilt = max(0, self.tilt - self.step)
            moved = True
            
        # Check Pan (Notice these are 'if', not 'elif', allowing simultaneous evaluation)
        if keyboard.Key.left in self.pressed_keys:
            self.pan = min(180, self.pan + self.step)
            moved = True
        if keyboard.Key.right in self.pressed_keys:
            self.pan = max(0, self.pan - self.step)
            moved = True
            
        # Check Blaster
        if keyboard.Key.space in self.pressed_keys:
            if self.laser_state == 0:
                self.laser_state = 1
                moved = True
        else:
            if self.laser_state == 1:
                self.laser_state = 0
                moved = True

        # Only publish to ROS if something actually changed this frame
        if moved:
            self.publish_state()

def main(args=None):
    rclpy.init(args=args)
    node = TurretTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()