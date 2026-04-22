import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct

class TurretSerialBridge(Node):
    def __init__(self):
        super().__init__('arduino_ros_bridge')
        
        # Update this to match your Arduino's port (e.g., /dev/ttyACM0, /dev/ttyUSB0, or COM3)
        port_name = '/dev/ttyACM0' 
        baud_rate = 115200
        
        try:
            self.serial_port = serial.Serial(port_name, baud_rate, timeout=0.01)
            self.get_logger().info(f"Connected to Arduino on {port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise SystemExit

        # We keep the QoS queue size small (10) to drop old packets.
        # In a tracking scenario, we only care about the absolute newest data.
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'turret_commands',
            self.command_callback,
            10 
        )

    def command_callback(self, msg):
        # Expecting an array of exactly 3 integers: [pan, tilt, laser]
        if len(msg.data) == 3:
            pan, tilt, laser = msg.data
            
            # Clamp the values. 
            # We cap at 254 so the data is never accidentally read as the 255 Start Marker.
            pan_byte = max(0, min(254, pan))
            tilt_byte = max(0, min(254, tilt))
            laser_byte = 1 if laser > 0 else 0
            
            # Pack the data into 4 unsigned chars (bytes)
            # Format '4B' means 4 Unsigned Bytes
            packet = struct.pack('4B', 255, pan_byte, tilt_byte, laser_byte)
            
            # Write directly to the hardware
            self.serial_port.write(packet)
        else:
            self.get_logger().warn("Invalid array length. Expected [pan, tilt, laser].")

def main(args=None):
    rclpy.init(args=args)
    bridge_node = TurretSerialBridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.serial_port.close()
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()