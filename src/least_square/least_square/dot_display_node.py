import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import tkinter as tk
import signal
import sys

a = 100
b = 100

class DotDisplay(Node):
    def __init__(self):
        super().__init__('dot_display_node')

        self.subscription = self.create_subscription(
            Point,
            'mouse_coords',
            self.listener_callback,
            10
        )

        self.x = 0
        self.y = 0

        self.root = tk.Tk()
        self.root.overrideredirect(True)
        self.root.attributes("-topmost", True)

        size = 10
        self.root.geometry(f"{size}x{size}")
        self.root.configure(bg="red")

        # Safe close bindings
        self.root.bind("<Escape>", self.close_app)
        self.root.bind("<Button-1>", self.close_app)

        self.update_ui()

    def listener_callback(self, msg):
        self.x = int(msg.x)
        self.y = int(msg.y)

    def update_ui(self):
        self.root.geometry(f"+{self.x + a}+{self.y + b}")
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(1, self.update_ui)   # change the number to add delay. 

    def close_app(self, event=None):
        self.cleanup()

    def cleanup(self):
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

        self.root.destroy()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = DotDisplay()

    #  Handle Ctrl+C globally
    def signal_handler(sig, frame):
        node.cleanup()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        node.root.mainloop()
    except KeyboardInterrupt:
        node.cleanup()


if __name__ == '__main__':
    main()