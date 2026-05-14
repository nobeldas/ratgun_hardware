import argparse
import math
import random
import sys
import tkinter as tk

try:
    from .least_square import filter_coordinate
except ImportError:
    from least_square import filter_coordinate


# Quick tuning values. Change these defaults, then run the simulator again.
# Higher LINEAR_SPEED moves faster in a straight line.
# Higher ANGULAR_VELOCITY turns faster.
# Higher NOISE_STD makes the measured path noisier.
LINEAR_SPEED = 3.0
ANGULAR_VELOCITY = 1.5
NOISE_STD = 0.02

# Example slower/smoother settings:
# LINEAR_SPEED = 0.5
# ANGULAR_VELOCITY = 0.25
# NOISE_STD = 0.02

# Example faster/noisier settings:
# LINEAR_SPEED = 2.0
# ANGULAR_VELOCITY = 1.0
# NOISE_STD = 0.15


class DiffDriveSim:
    def __init__(
        self,
        velocity=LINEAR_SPEED,
        angular_velocity=ANGULAR_VELOCITY,
        noise_std=NOISE_STD,
    ):
        self.forward_speed = abs(velocity) 
        self.turn_speed = abs(angular_velocity) 
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.noise_std = noise_std

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.output_x = 0.0
        self.output_y = 0.0
        self.predicted_curve = []
        self.last_time = None

        self.scale = 60.0
        self.origin_x = 360
        self.origin_y = 280

        self.root = tk.Tk()
        self.root.title("Diff Drive Simulation")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.canvas = tk.Canvas(self.root, width=720, height=560, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=4, sticky="nsew")

        self.status_var = tk.StringVar()
        tk.Label(self.root, textvariable=self.status_var, anchor="w").grid(
            row=1, column=0, columnspan=4, sticky="ew", padx=8, pady=4
        )

        self.path = []
        self.noisy_path = []
        self.output_path = []
        self.pressed_keys = set()
        self.bind_teleop_keys()

    def bind_teleop_keys(self):
        self.root.bind_all("<KeyPress>", self.handle_key_press)
        self.root.bind_all("<KeyRelease>", self.handle_key_release)

    def handle_key_press(self, event):
        key = event.keysym.lower()
        if key in ("space", "k"):
            self.pressed_keys.clear()
            self.stop_motion()
            return
        if key in self.teleop_key_map():
            self.pressed_keys.add(key)
            self.update_motion_from_keys()

    def handle_key_release(self, event):
        key = event.keysym.lower()
        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
            self.update_motion_from_keys()

    def update_motion_from_keys(self):
        velocity = 0.0
        angular_velocity = 0.0
        key_map = self.teleop_key_map()
        for key in self.pressed_keys:
            key_velocity, key_angular_velocity = key_map[key]
            velocity += key_velocity
            angular_velocity += key_angular_velocity
        self.set_motion(
            max(-self.forward_speed, min(self.forward_speed, velocity)),
            max(-self.turn_speed, min(self.turn_speed, angular_velocity)),
        )

    def teleop_key_map(self):
        return {
            "up": (self.forward_speed, 0.0),
            "down": (-self.forward_speed, 0.0),
            "left": (0.0, -self.turn_speed),
            "right": (0.0, self.turn_speed),
        }

    def set_motion(self, velocity, angular_velocity):
        self.velocity = velocity
        self.angular_velocity = angular_velocity

    def stop_motion(self):
        self.set_motion(0.0, 0.0)

    def close(self):
        self.root.destroy()

    def run(self):
        self.last_time = self.now()
        self.update()
        self.root.mainloop()

    def update(self):
        current_time = self.now()
        dt = min(current_time - self.last_time, 0.05)
        self.last_time = current_time

        self.step(dt)

        self.draw()

        self.root.after(20, self.update)

    def step(self, dt):
        self.x += self.velocity * math.cos(self.theta) * dt
        self.y += self.velocity * math.sin(self.theta) * dt
        self.theta += self.angular_velocity * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        noisy_x = self.x + random.gauss(0.0, self.noise_std)
        noisy_y = self.y + random.gauss(0.0, self.noise_std)
        self.output_x, self.output_y = self.filter_coordinate(noisy_x, noisy_y)

        self.path.append((self.x, self.y))
        self.noisy_path.append((noisy_x, noisy_y))
        self.output_path.append((self.output_x, self.output_y))

        self.path = self.path[-400:]
        self.noisy_path = self.noisy_path[-400:]
        self.output_path = self.output_path[-400:]

    def filter_coordinate(self, noisy_x, noisy_y):
        filtered = filter_coordinate(noisy_x, noisy_y)
        self.predicted_curve = [
            (float(x), float(y))
            for x, y in getattr(filter_coordinate, "predicted_curve", [])
        ]

        if len(filtered) == 2 and hasattr(filtered[0], "__len__"):
            tracked_state, future_state = filtered
            if not self.predicted_curve:
                self.predicted_curve = [
                    (float(tracked_state[0]), float(tracked_state[1])),
                    (float(future_state[0]), float(future_state[1])),
                ]
            return float(future_state[0]), float(future_state[1])

        output_x, output_y = filtered
        return float(output_x), float(output_y)

    def draw(self):
        self.canvas.delete("all")
        self.draw_grid()
        self.draw_path(self.noisy_path, "#d08020", 1)
        self.draw_path(self.output_path, "#209050", 2)
        self.draw_path(self.predicted_curve, "#c02080", 3)

        true_px, true_py = self.to_screen(self.x, self.y)
        out_px, out_py = self.to_screen(self.output_x, self.output_y)

        self.canvas.create_oval(
            true_px - 7, true_py - 7, true_px + 7, true_py + 7, fill="#2060c0", outline=""
        )
        nose_px = true_px + 16 * math.cos(self.theta)
        nose_py = true_py - 16 * math.sin(self.theta)
        self.canvas.create_line(true_px, true_py, nose_px, nose_py, fill="black", width=2)
        self.canvas.create_oval(
            out_px - 6, out_py - 6, out_px + 6, out_py + 6, fill="#209050", outline=""
        )
        self.canvas.create_text(true_px + 42, true_py - 12, text="true", fill="#2060c0")
        self.canvas.create_text(out_px + 48, out_py + 12, text="output", fill="#209050")

        self.status_var.set(
            f"true=({self.x:.2f}, {self.y:.2f})  "
            f"output=({self.output_x:.2f}, {self.output_y:.2f})  "
            f"v={self.velocity:.2f}  w={self.angular_velocity:.2f}  "
            f"linear speed={self.forward_speed:.2f}  angular speed={self.turn_speed:.2f}"
        )

    def draw_grid(self):
        width = int(self.canvas["width"])
        height = int(self.canvas["height"])
        for x in range(0, width, int(self.scale)):
            self.canvas.create_line(x, 0, x, height, fill="#eeeeee")
        for y in range(0, height, int(self.scale)):
            self.canvas.create_line(0, y, width, y, fill="#eeeeee")
        self.canvas.create_line(self.origin_x, 0, self.origin_x, height, fill="#cccccc")
        self.canvas.create_line(0, self.origin_y, width, self.origin_y, fill="#cccccc")

    def draw_path(self, points, color, width):
        if len(points) < 2:
            return
        screen_points = []
        for x, y in points:
            screen_points.extend(self.to_screen(x, y))
        self.canvas.create_line(*screen_points, fill=color, width=width)

    def to_screen(self, x, y):
        return self.origin_x + x * self.scale, self.origin_y - y * self.scale

    @staticmethod
    def now():
        return time_source()


def time_source():
    return __import__("time").monotonic()


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Simple diff-drive simulator.")
    parser.add_argument("--velocity", "-v", type=float, default=LINEAR_SPEED)
    parser.add_argument("--angular-velocity", "-w", type=float, default=ANGULAR_VELOCITY)
    parser.add_argument("--noise-std", type=float, default=NOISE_STD)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    sim = DiffDriveSim(
        velocity=args.velocity,
        angular_velocity=args.angular_velocity,
        noise_std=args.noise_std,
    )
    sim.run()


if __name__ == "__main__":
    main()
