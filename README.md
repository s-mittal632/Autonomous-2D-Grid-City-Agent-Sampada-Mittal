# Autonomous-2D-Grid-City-Agent-Sampada-Mittal
1. Problem Statement: Using a dynamic map, describe in brief the difficulty of determining the best or most efficient route for an autonomous agent to take from a starting point to a destination. 2. Project Goal: The goal of this work is to design, implement, and compare different search algorithms for this pathfinding problem.

Project Overview
This project simulates and compares various search algorithms used by an autonomous agent to find optimal paths across a grid map. The environment features heterogeneous terrain (varying movement costs) and dynamic, time-dependent obstacles. The goal is to analyze the trade-offs between path cost (optimality) and computational efficiency (speed).

Key Objectives

Cost Optimization: Find the path with the minimum total travel cost.
Dynamic World Modeling: Navigate environments with time-varying, moving obstacles.
Algorithm Comparison: Analyze and compare four main search strategies based on performance metrics like path cost, nodes expanded, and runtime.

Features

Multi-Cost Terrain: Supports movement costs for terrain types like EMPTY, EXPRESSWAY, PARK, and WATER.
Time-Space Search: Algorithms operate in the state space of (X, Y, Time) to handle moving obstacles.
Performance Metrics: Tracks Path Cost, Path Length, Nodes Expanded, and Computation Time.
Visualization: Uses Matplotlib to plot the calculated path on the grid map.

Available Planners

The project implements the following search algorithms:
Uninformed: Breadth-First Search (BFS) and Uniform-Cost Search (UCS).
Informed: A* Search (with Manhattan and Euclidean heuristics).
Metaheuristic: Simulated Annealing (SA) for path optimization.

Dependencies

The project requires the following standard Python libraries:
numpy: For efficient array and grid manipulation.
matplotlib.pyplot: For path visualization.
heapq: For priority queue implementation (UCS and A*).

Installation
Install the required libraries using pip:

Bash
pip install numpy matplotlib
Usage
Single Run
Execute the main.py script with the map file, coordinates, and desired algorithm.

Bash
# Example: Run A* with visualization
python main.py medium.map 0 0 9 9 --algorithm astar --heuristic manhattan -v
Algorithm Comparison
Use the --compare flag to automatically run all implemented planners on a map and print a performance summary.

Bash
# Example: Compare all algorithms on the dynamic map
python main.py dynamic.map 0 0 4 4 --compare

Experiments and Results

In general, the A* planner offers the best balance of optimality (low cost) and efficiency (low nodes expanded) due to its use of an effective heuristic. UCS guarantees the optimal path but is generally slower than A*. The performance analysis focuses on proving A*'s superiority in balancing computational speed against path quality in this multi-cost environment.

Notes
The script automatically generates test map files (small.map, medium.map, large.map, dynamic.map) if they do not exist when running the default or comparison modes.
