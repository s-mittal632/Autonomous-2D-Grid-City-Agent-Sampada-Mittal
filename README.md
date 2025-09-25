# Autonomous-2D-Grid-City-Agent-Sampada-Mittal
1. Problem Statement: Using a dynamic map, describe in brief the difficulty of determining the best or most efficient route for an autonomous agent to take from a starting point to a destination. 2. Project Goal: The goal of this work is to design, implement, and compare different search algorithms for this pathfinding problem.

# Autonomous Pathfinding Agent  

## Project Overview  
This project implements an autonomous pathfinding agent that navigates 2D grid-based maps with varying terrains and dynamic obstacles.  
The system models both static terrains (roads, parks, water, buildings) and moving obstacles, enabling the agent to find efficient and feasible routes under different constraints.  

The project supports multiple search algorithms (uninformed, informed, and metaheuristic) and allows experimental comparison of their performance across maps of different sizes.  

---

## Key Objectives  
- Model 2D grid environments with terrain costs, static obstacles, and dynamic obstacles.  
- Implement rational planning strategies that balance cost, path length, and computational efficiency.  
- Provide visualization of maps and computed paths.  
- Enable experimental benchmarking of algorithms across different scenarios.  

---

## Features  
- Terrain modeling: empty cells, expressways, parks, water, buildings, and impassable obstacles.  
- Dynamic obstacle handling with time-based schedules.  
- Multiple planning algorithms:  
  - Uninformed search: Breadth-First Search (BFS), Uniform Cost Search (UCS)  
  - Informed search: A* (Manhattan, Euclidean heuristics)  
  - Metaheuristic search: Simulated Annealing (path optimization & replanning)  
- 4-connected movement (up, down, left, right).  
- Built-in map generator for small, medium, large, and dynamic maps.  
- Visualization with matplotlib to render grids and paths.  
- Metrics logging: nodes expanded, path cost, computation time.  
- CLI interface for running custom or comparative experiments.  

---

## Installation  

Clone the repository and navigate into the project folder:  

```bash
git clone <repo_url>
cd pathfinding-agent
```

Create and activate a virtual environment:

python -m venv envi_agent
# On Linux/Mac
source envi_agent/bin/activate
# On Windows
envi_agent\Scripts\activate


Install dependencies:
pip install -r requirements.txt

Usage (CLI)
```bash
Run a planner on a single map:
python run.py --planner astar --map maps/small.grid --verbose
```


Run dynamic replanning with obstacle schedules:
```bash
python run.py --planner astar --map maps/dynamic.grid --schedule schedules/dynamic.schedule --dynamic --verbose
```

Available Planners

bfs – Breadth-first search
ucs – Uniform-cost search
astar – A* search with an admissible heuristic
local – Local search / replanning

Options

--verbose / -v : Print detailed logs of the agent's actions and path.
--dynamic : Enable dynamic replanning for environments with moving obstacles.
--schedule / -s : Provide a file path for a dynamic obstacle schedule.

Experiments & Results

Compare planners across multiple maps.
Metrics reported: path cost, nodes expanded, time, and collisions.
Visualize agent trajectories on dynamic maps using --verbose.
Example logs and demo videos are included in demos/demo_dynamic_run.mp4.

Tests
Run unit tests to verify core functionality:
```bash
pytest tests/
```

Tests include:

mapio: Grid and schedule loading, neighbor computation, collision checking.
planners: Path correctness and cost for BFS, UCS, A*, and local search.

Dependencies
Python ≥ 3.8 recommended.
See requirements.md for full dependency list.

Notes
Grids use integer movement costs ≥ 1. Obstacles are impassable.
Agent movement is deterministic; dynamic obstacles follow predefined schedules.
Local search can simulate unpredictable obstacle behavior.


