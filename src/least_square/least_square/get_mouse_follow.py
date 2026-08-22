import pyautogui
import tkinter as tk
import signal
import sys

a = 100
b = 100

root = tk.Tk()
root.overrideredirect(True)
root.attributes("-topmost", True)

size = 10
root.geometry(f"{size}x{size}")
root.configure(bg="red")

#  Close function
def close_app(event=None):
    root.destroy()
    sys.exit(0)

# Bind ESC
root.bind("<Escape>", close_app)

# 🔥 Handle Ctrl+C (SIGINT)
def signal_handler(sig, frame):
    close_app()

signal.signal(signal.SIGINT, signal_handler)

def update_position():
    x, y = pyautogui.position()
    root.geometry(f"+{x + a}+{y + b}")
    root.after(10, update_position)

update_position()

try:
    root.mainloop()
except KeyboardInterrupt:
    close_app()