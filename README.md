# Autonomous-2D-Grid-City-Agent-Sampada-Mittal
1. Problem Statement: Using a dynamic map, describe in brief the difficulty of determining the best or most efficient route for an autonomous agent to take from a starting point to a destination. 2. Project Goal: The goal of this work is to design, implement, and compare different search algorithms for this pathfinding problem.

##Project Overview

This project implements an autonomous pathfinding agent that navigates 2D grid-based maps with varying terrains and dynamic obstacles. The system models both static terrains (roads, parks, water, buildings) and moving obstacles, enabling the agent to find efficient and feasible routes under different constraints.

The project supports multiple search algorithms (uninformed, informed, and metaheuristic) and allows experimental comparison of their performance across maps of different sizes.

##Key Objectives

Model 2D grid environments with terrain costs, static obstacles, and dynamic obstacles.

Implement rational planning strategies that balance cost, path length, and computational efficiency.

Provide visualization of maps and computed paths.

Enable experimental benchmarking of algorithms across different scenarios.

Features

Terrain modeling: empty cells, expressways, parks, water, buildings, and impassable obstacles.

Dynamic obstacle handling with time-based schedules.

Multiple planning algorithms:

Uninformed search: Breadth-First Search (BFS), Uniform Cost Search (UCS)

Informed search: A* (Manhattan, Euclidean heuristics)

Metaheuristic search: Simulated Annealing (path optimization & replanning)

4-connected movement (up, down, left, right).

Built-in map generator for small, medium, large, and dynamic maps.

Visualization with matplotlib to render grids and paths.

Metrics logging: nodes expanded, path cost, computation time.

CLI interface for running custom or comparative experiments.

Installation

Clone the repository and navigate into the project folder:

git clone <repo_url>
cd pathfinding-agent


Create and activate a virtual environment:

python -m venv venv
# On Linux/Mac
source venv/bin/activate
# On Windows
venv\Scripts\activate


Install dependencies:

pip install -r requirements.txt

Usage (CLI)

Run with default settings (auto-generates maps):

python main.py


Run a specific algorithm on a map:

python main.py small.map 0 0 4 4 --algorithm astar --heuristic manhattan --visualize


Compare all algorithms on a map:

python main.py small.map 0 0 4 4 --compare

Available Algorithms

bfs → Breadth-First Search

ucs → Uniform Cost Search

astar → A* Search (Manhattan / Euclidean)

sa → Simulated Annealing

Experiments & Results

Compare BFS, UCS, A*, and Simulated Annealing on multiple maps.

Reported metrics:

Path cost

Path length

Nodes expanded

Computation time

Visualization of grid maps and paths (via --visualize).

Dynamic obstacle tests included (dynamic.map).

Tests

Run unit tests to validate core modules:

pytest tests/


Tests cover:

Map parsing and grid loading.

Terrain and cost assignments.

BFS, UCS, A*, and SA correctness.

Dependencies

Python >= 3.8

numpy

matplotlib
Installation

Clone the repository and navigate into the project folder:

git clone <repo_url>
cd pathfinding-agent


Create and activate a virtual environment:

python -m venv venv
# On Linux/Mac
source venv/bin/activate
# On Windows
venv\Scripts\activate


Install dependencies:

pip install -r requirements.txt

Usage (CLI)

Run with default settings (auto-generates maps):

python main.py


Run a specific algorithm on a map:

python main.py small.map 0 0 4 4 --algorithm astar --heuristic manhattan --visualize


Compare all algorithms on a map:

python main.py small.map 0 0 4 4 --compare

Dependencies

Python >= 3.8

numpy

matplotlib
python -m venv venv
# On Linux/Mac
source venv/bin/activate
# On Windows
venv\Scripts\activate
