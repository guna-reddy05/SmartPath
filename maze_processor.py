import cv2
import numpy as np
import serial
import time
from pathlib import Path

# === CONFIGURATION ===
BT_PORT = 'COM4'  # Change to your ESP32 Bluetooth COM port
BAUD_RATE = 115200
GRID_ROWS = 11
GRID_COLS = 11

def process_maze_image(image_path):
    img = cv2.imread(image_path)
    if img is None:
        print("Error: Image not found.")
        return None

    # 1. Preprocessing
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    
    # 2. Grid Analysis
    H, W = blurred.shape
    cell_h, cell_w = H / GRID_ROWS, W / GRID_COLS
    wall_mask = (blurred < 100).astype(np.float32)
    
    ratios = np.zeros((GRID_ROWS, GRID_COLS), dtype=np.float32)
    for r in range(GRID_ROWS):
        for c in range(GRID_COLS):
            y1, y2 = int(r * cell_h), int((r + 1) * cell_h)
            x1, x2 = int(c * cell_w), int((c + 1) * cell_w)
            ratios[r, c] = wall_mask[y1:y2, x1:x2].mean()

    # 3. Thresholding & Matrix (0=Path, 1=Wall)
    values = np.sort(np.unique(ratios.flatten()))
    thresh = (values[np.argmax(np.diff(values))] + values[np.argmax(np.diff(values))+1]) / 2
    matrix = (ratios > thresh).astype(int)

    # 4. Identify Start (Top-Leftmost path) and Goal (Bottom-Rightmost path)
    coords = np.argwhere(matrix == 0)
    start = tuple(coords[0])
    goal = tuple(coords[-1])
    
    return matrix, start, goal

def send_to_esp32(matrix, start, goal):
    try:
        ser = serial.Serial(BT_PORT, BAUD_RATE, timeout=2)
        print(f"Connected to {BT_PORT}. Initializing transfer...")
        time.sleep(3) # Wait for BT sync

        payload = "<START>\n"
        for r in range(matrix.shape[0]):
            line = ""
            for c in range(matrix.shape[1]):
                if (r, c) == start: line += "S"
                elif (r, c) == goal: line += "G"
                else: line += str(matrix[r, c])
            payload += line + "\n"
        payload += "<END>\n"

        ser.write(payload.encode('utf-8'))
        print("Maze data sent successfully!")
        
        # Listen for ESP32 confirmation
        while True:
            if ser.in_waiting:
                print("ESP32:", ser.readline().decode().strip())
            time.sleep(0.1)

    except Exception as e:
        print(f"Serial Error: {e}")

if __name__ == "__main__":
    matrix, start, goal = process_maze_image("maze.png")
    if matrix is not None:
        send_to_esp32(matrix, start, goal)
