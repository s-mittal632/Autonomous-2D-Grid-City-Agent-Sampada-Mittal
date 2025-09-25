# Autonomous-2D-Grid-City-Agent-Sampada-Mittal
1. Problem Statement: Using a dynamic map, describe in brief the difficulty of determining the best or most efficient route for an autonomous agent to take from a starting point to a destination. 2. Project Goal: The goal of this work is to design, implement, and compare different search algorithms for this pathfinding problem.
Overview

This project implements an Autonomous Delivery Agent capable of finding paths across grid-based maps with different terrains and obstacles. It supports multiple search and optimization algorithms to simulate decision-making in realistic environments such as roads, parks, water bodies, and dynamic obstacles.
Features

Supports multiple algorithms:
Breadth-First Search (BFS) – Explores paths uniformly, ignores terrain cost
Uniform Cost Search (UCS) – Expands paths based on the lowest actual cost.
A* – Uses heuristics (Manhattan / Euclidean) for efficient pathfinding.
Simulated Annealing (SA) – Optimization approach to refine paths.
Handles dynamic moving obstacles.
Visualizes the grid and paths using Matplotlib.
Includes map generation (small, medium, large, dynamic).
Command-line interface with flexible arguments.

Important Libraries

numpy → Grid representation and matrix operations
heapq → Priority queues for UCS and A*
random / math → Random path perturbations and cost calculations
argparse → Command-line interface for custom runs
matplotlib → Visualization of maps and computed paths
enum & typing → Structured terrain types and type hints

Key Insights:

BFS is simple but not cost-efficient and scales poorly.
UCS guarantees optimal paths but explores more nodes than A*.
A* with Manhattan heuristic performs best (fastest + optimal).
SA can refine paths but is slower due to random sampling.

Quick Start

Clone this repository:

git clone https://github.com/your-username/autonomous-delivery-agent.git
cd autonomous-delivery-agent


Run with defaults (small map, A* algorithm):
python main.py


Custom run:
python main.py medium.map 0 0 9 9 -a astar --heuristic manhattan -v
