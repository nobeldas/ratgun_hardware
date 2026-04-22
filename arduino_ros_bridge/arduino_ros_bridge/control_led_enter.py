import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading

class LaserToggleNode(Node):
    def __init__(self):
        super().__init__('laser_toggler')
        
        # Publish to the exact same topic the serial bridge is listening to
        self.publisher_ = self.create_publisher(Int32MultiArray, 'turret_commands', 10)
        
        # Initial states
        self.laser_state = 0
        self.pan = 90  # Default centered pan
        self.tilt = 90 # Default centered tilt

        # Start a background thread to listen for the Enter key
        self.input_thread = threading.Thread(target=self.wait_for_enter)
        self.input_thread.daemon = True # Ensures thread dies when the main node is killed
        self.input_thread.start()

        self.get_logger().info("Laser Toggler Node Started.")
        self.get_logger().info("Press [ENTER] to toggle the laser. Press [Ctrl+C] to exit.")

    def wait_for_enter(self):
        # This loop runs forever in the background
        while rclpy.ok():
            try:
                # input() pauses this thread until Enter is pressed
                input() 
                
                # Flip the laser state: If 0 make it 1. If 1 make it 0.
                self.laser_state = 1 if self.laser_state == 0 else 0
                
                # Create and pack the ROS2 message
                msg = Int32MultiArray()
                msg.data = [self.pan, self.tilt, self.laser_state]
                
                # Blast it out to the serial bridge
                self.publisher_.publish(msg)
                
                # Log the output to the terminal so you know it worked
                state_str = "ON" if self.laser_state == 1 else "OFF"
                self.get_logger().info(f"Laser toggled {state_str}. Published: {msg.data}")
                
            except EOFError:
                # Handles edge cases where the terminal closes abruptly
                break

def main(args=None):
    rclpy.init(args=args)
    node = LaserToggleNode()
    
    try:
        # spin() keeps the ROS2 node alive and listening for Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()