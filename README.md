# Autonomous-2D-Grid-City-Agent-Sampada-Mittal
1. Problem Statement: Using a dynamic map, describe in brief the difficulty of determining the best or most efficient route for an autonomous agent to take from a starting point to a destination. 2. Project Goal: The goal of this work is to design, implement, and compare different search algorithms for this pathfinding problem.
# Autonomous Delivery Agent

## Project Overview
This project implements an autonomous delivery agent that navigates a 2D grid city to deliver packages efficiently. The agent models both static and dynamic environments, accounting for varying terrain costs and moving obstacles.

#### Key Objectives:

* Model the environment (static obstacles, terrain costs, dynamic obstacles).
* Plan rational actions that optimize delivery efficiency under constraints (time, fuel).
* Implement multiple planning strategies:
    * **Uninformed search:** `BFS`, `Uniform-cost search (UCS)`
    * **Informed search:** `A*` with an admissible heuristic
    * **Local search / replanning:** `Hill-climbing` / `simulated annealing` for dynamic obstacles

The project allows for the experimental comparison of planners on multiple maps and provides visual/log outputs demonstrating the agent's behavior.

## Features
* **4-connected movement** (`up`/`down`/`left`/`right`) on a grid.
* Supports **dynamic obstacle schedules** for replanning scenarios.
* **CLI interface** to run single or batch planning experiments.
* **Logging** and optional verbose **visualization** of agent movement.
* Predefined **test maps**: `small`, `medium`, `large`, and dynamic obstacle scenarios.
* **Unit tests** for map IO and planners, ensuring reproducibility.


## Installation

1.  **Clone the repository** and enter the directory:
    ```bash
    git clone <repo_url>
    cd delivery-agent
    ```

2.  **Create a virtual environment** (recommended):
    ```bash
    python -m venv envi_agent
    ```

3.  **Activate the environment**:
    * On **Linux/Mac**:
        ```bash
        source envi_agent/bin/activate
        ```
    * On **Windows**:
        ```powershell
        envi_agent\Scripts\activate
        ```

4.  **Install dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

## Usage (CLI)

#### Run a planner on a single map:
```bash
python run.py --planner astar --map maps/small.grid --verbose
```

#### Run dynamic replanning with obstacle schedules:

```bash
python run.py --planner astar --map maps/dynamic.grid --schedule schedules/dynamic.schedule --dynamic --verbose
```

### Available Planners:
* `bfs` – Breadth-first search
* `ucs` – Uniform-cost search
* `astar` – A* search with an admissible heuristic
* `local` – Local search / replanning

### Options:
* `--verbose` / `-v`: Print detailed logs of the agent's actions and path.
* `--dynamic`: Enable dynamic replanning for environments with moving obstacles.
* `--schedule` / `-s`: Provide a file path for a dynamic obstacle schedule.

## Experiments & Results
* Compare planners across multiple maps.
* Metrics reported: **path cost**, **nodes expanded**, **time**, and **collisions**.
* Visualize agent trajectories on dynamic maps (optional, via `--verbose`).
* Example log outputs and a demo video are included in `demos/demo_dynamic_run.mp4`.

## Tests
Run unit tests to verify core functionality:
```bash
pytest tests/
```

**Tests include:**

* **mapio**: Grid and schedule loading, neighbor computation, and collision checking.
* **planners**: Path correctness and cost for `BFS`, `UCS`, `A*`, and local search.


## Dependencies
* See `requirements.md` for the detailed dependency list.
* **Python `≥ 3.8`** is recommended.

---

## Notes
* Grids use integer movement costs `≥ 1`. Obstacles are impassable.
* Agent movement is deterministic; dynamic obstacles follow predefined schedules.
* Local search can simulate unpredictable obstacle behavior.
