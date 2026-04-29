# Multi-Robot Path Planning on a Grid

## Overview
This project is an implementation of a multi-robot path planning algorithm on a 2D grid. The program calculates optimal paths for multiple robots, allowing them to navigate from their respective start positions to their goal positions while visiting an ordered sequence of checkpoints. 

The environment includes static obstacles, directional one-way cells, and limits on the maximum energy/time each robot can expend. To prevent collisions, the planner processes robots sequentially based on a predefined priority, ensuring no two robots occupy the same space at the same time.

## Features
* **Priority-Based Planning:** Robots with higher priority are planned first. Lower priority robots dynamically route around the reserved paths of higher-priority robots.
* **Checkpoint Navigation:** Robots successfully navigate through `K` checkpoints in exact sequential order before reaching their final goal.
* **Environmental Constraints:** Pathfinding respects grid boundaries, static obstacles (`X`), and directional one-way cells (`^`, `v`, `<`, `>`).
* **Wait Actions:** Robots can "wait" in their current cell for a time step to let a higher-priority robot pass, consuming 1 unit of energy.

## Files Included
* `assignment1.py`: The main Python script containing the search algorithm and logic.
* `input.txt` / `input_big.txt` / `exampleInput.txt` / `input_impossible.txt`: Various test cases specifying grid dimensions, layout, and robot configurations.
* `output.txt`: The generated output file containing the sequential path, total time, and total energy for each robot.

## How to Run
1. Ensure you have Python 3 installed on your system. No external libraries are required as the code only utilizes the standard Python library (`collections.deque`).
2. Place your input file in the same directory as the script. **Note:** Ensure the input file you want to test is named `input.txt` and that the script is set to read from it. *(Currently, line 10 in `assignment1.py` reads from `exampleInput.txt` — change this to `input.txt` before final submission to meet assignment requirements!)*
3. Open a terminal or command prompt in the directory.
4. Run the script using the following command:
   ```bash
   python assignment1.py
