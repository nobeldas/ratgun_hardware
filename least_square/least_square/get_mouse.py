import pyautogui

print("Move your mouse to see coordinates. Press Ctrl+C to stop.")

try:
    while True:
        x, y = pyautogui.position()
        print(f"X: {x}, Y: {y}", end="\r")
except KeyboardInterrupt:
    print("\nStopped.")