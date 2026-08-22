import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyautogui

try:
    from .least_square import filter_coordinate
except ImportError:
    from least_square import filter_coordinate

class MouseTracker(Node):
    def __init__(self):
        super().__init__('mouse_tracker')
        self.publisher_ = self.create_publisher(Point, 'mouse_coords', 10)
        self.timer = self.create_timer(0.01, self.publish_mouse_position)

    def publish_mouse_position(self):
        x, y = pyautogui.position()
        predicted_x, predicted_y = filter_coordinate(float(x), float(y))

        msg = Point()
        msg.x = predicted_x
        msg.y = predicted_y
        msg.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MouseTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
