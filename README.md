# ESP32 Bluetooth Maze Solver Robot

An automated robotics project that processes a physical or digital maze image using **OpenCV**, converts it into a grid matrix, sends it wirelessly via **Bluetooth Serial** to an **ESP32 microcontroller**, and calculates/executes the optimal path using the **A* (A-Star) search algorithm**.

---

## 🚀 Project Components

1. **Python Vision Script (`maze_processor.py`)**:
   - Reads a maze image (`maze.png`).
   - Converts the image to grayscale, applies Gaussian blur, and maps out a grid matrix (`GRID_ROWS` x `GRID_COLS`).
   - Identifies walls (`1`), open paths (`0`), the starting point (`S`), and the goal point (`G`).
   - Connects to the ESP32 via Bluetooth (using PySerial) and transmits the parsed maze data.

2. **ESP32 Firmware (`esp32_maze_bot.ino`)**:
   - Runs on an ESP32 microcontroller using the `BluetoothSerial` library.
   - Dynamically allocates memory to store grid boundaries and obstacles.
   - Implements the **A* Pathfinding Algorithm** to compute the shortest route from the start node to the goal node.
   - Controls dual DC motors via an L298N (or similar) motor driver to navigate the physical maze.

---

## 🛠️ Hardware Requirements

* ESP32 Development Board
* DC Geared Motors (x2) with Motor Driver (e.g., L298N)
* Power Source / Battery pack for motors and ESP32
* Bluetooth-enabled PC or Laptop (for sending the maze matrix)

---

## ⚙️ Pin Configuration (ESP32)

| Component | ESP32 Pin | Description |
| :--- | :--- | :--- |
| **IN1** | GPIO 27 | Motor Driver Input 1 |
| **IN2** | GPIO 26 | Motor Driver Input 2 |
| **IN3** | GPIO 25 | Motor Driver Input 3 |
| **IN4** | GPIO 33 | Motor Driver Input 4 |
| **ENA** | GPIO 14 | Motor A Enable (PWM Speed) |
| **ENB** | GPIO 32 | Motor B Enable (PWM Speed) |

---

## 💻 Setup and Usage

### 1. ESP32 Setup
* Open `esp32_maze_bot.ino` in the Arduino IDE.
* Install the `BluetoothSerial` board libraries if needed.
* Select your ESP32 board and COM port, then upload the code to your ESP32.

### 2. Python Setup
* Install required dependencies via terminal:
  ```bash
  pip install opencv-python numpy pyserial
