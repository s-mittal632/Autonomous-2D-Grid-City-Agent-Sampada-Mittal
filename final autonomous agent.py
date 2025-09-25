import numpy as np
import heapq
import random
import math
import time
import argparse
from enum import Enum
from typing import List, Tuple, Dict, Set, Optional, Any
import matplotlib.pyplot as plt
import sys

class TerrainType(Enum):
    EMPTY = 1
    OBSTACLE = 2
    EXPRESSWAY = 3
    PARK = 4
    WATER = 5
    BUILDING = 6

class MapGrid:
    def __init__(self, width: int, height: int, default_cost: int = 1):
        self.width = width
        self.height = height
        self.grid = np.full((height, width), TerrainType.EMPTY)
        self.costs = np.full((height, width), default_cost)
        self.dynamic_obstacles = {}  # Format: {time: set((x, y))}
        
        # Define terrain costs
        self.terrain_costs = {
            TerrainType.EMPTY: 1,
            TerrainType.OBSTACLE: float('inf'),
            TerrainType.EXPRESSWAY: 1,
            TerrainType.PARK: 2,
            TerrainType.WATER: 5,
            TerrainType.BUILDING: float('inf')
        }
    
    def update_cell_terrain(self, x: int, y: int, cell_type: TerrainType):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y, x] = cell_type
            self.costs[y, x] = self.terrain_costs[cell_type]
    
    def calculate_movement_cost(self, x: int, y: int, time: int = 0) -> float:
        if not (0 <= x < self.width and 0 <= y < self.height):
            return float('inf')
        
        # Check for dynamic obstacles at this time
        if time in self.dynamic_obstacles and (x, y) in self.dynamic_obstacles[time]:
            return float('inf')
            
        return self.costs[y, x]
    def register_moving_obstacles(self, positions: Dict[int, Set[Tuple[int, int]]]):
        for time, cells in positions.items():
            if time not in self.dynamic_obstacles:
                self.dynamic_obstacles[time] = set()
            self.dynamic_obstacles[time].update(cells)
    
    def Is_Valid_Position(self, x: int, y: int, time: int = 0) -> bool:
        return (0 <= x < self.width and 0 <= y < self.height and 
                self.calculate_movement_cost(x, y, time) < float('inf'))
    
    @staticmethod
    def load_from_file(filename: str) -> 'MapGrid':
        with open(filename, 'r') as f:
            lines = f.readlines()
        
        # Parse header
        width, height = map(int, lines[0].strip().split(','))
        grid = MapGrid(width, height)
        
        # Parse grid cells
        for y, line in enumerate(lines[1:1+height]):
            for x, char in enumerate(line.strip()):
                if char == '#':  # Obstacle
                    grid.update_cell_terrain(x, y, TerrainType.OBSTACLE)
                elif char == '.':  # Empty
                    grid.update_cell_terrain(x, y, TerrainType.EMPTY)
                elif char == 'E':  # EXPRESSWAY
                    grid.update_cell_terrain(x, y, TerrainType.EXPRESSWAY)
                elif char == 'P':  # PARK
                    grid.update_cell_terrain(x, y, TerrainType.PARK)
                elif char == 'W':  # Water
                    grid.update_cell_terrain(x, y, TerrainType.WATER)
                elif char == 'B':  # Building
                    grid.update_cell_terrain(x, y, TerrainType.BUILDING)
        
        # Parse dynamic obstacles if present
        if len(lines) > 1 + height:
            dynamic_obstacles = {}
            for line in lines[1+height:]:
                if line.strip() and ':' in line:
                    time_str, positions_str = line.strip().split(':', 1)
                    time = int(time_str)
                    positions = set()
                    for pos in positions_str.split(';'):
                        if pos:
                            x, y = map(int, pos.split(','))
                            positions.add((x, y))
                    dynamic_obstacles[time] = positions
            
            grid.register_moving_obstacles(dynamic_obstacles)
        
        return grid

class BaseAgent:
    def __init__(self, grid: MapGrid, fuel_constraint: float = float('inf')):
        self.grid = grid
        self.fuel_constraint = fuel_constraint
        self.nodes_expanded = 0
        self.computation_time = 0
        self.path_cost = 0
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        raise NotImplementedError("Subclasses must implement find_path")
    
    def get_metrics(self) -> Dict[str, Any]:
        return {
            'nodes_expanded': self.nodes_expanded,
            'computation_time': self.computation_time,
            'path_cost': self.path_cost,
            'fuel_used': self.path_cost  # Assuming 1:1 cost:fuel ratio for simplicity
        }
    
    def get_neighbors(self, x: int, y: int, time: int = 0) -> List[Tuple[int, int, float]]:
        neighbors = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:  # 4-connected
            nx, ny = x + dx, y + dy
            if self.grid.Is_Valid_Position(nx, ny, time):
                cost = self.grid.calculate_movement_cost(nx, ny, time)
                neighbors.append((nx, ny, cost))
        return neighbors

class BreadthFirstAgent(BaseAgent):
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        start_time = time.time()
        self.nodes_expanded = 0
        
        queue = [(start[0], start[1], 0)]  # (x, y, time)
        visited = {start: (None, 0)}  # cell -> (parent, time)
        
        while queue:
            x, y, time_step = queue.pop(0)
            self.nodes_expanded += 1
            
            if (x, y) == goal:
                # Reconstruct path
                path = []
                current = (x, y)
                while current != start:
                    path.append(current)
                    current, _ = visited[current]
                path.append(start)
                path.reverse()
                
                self.computation_time = time.time() - start_time
                self.path_cost = len(path) - 1  # Number of moves
                return path
            
            for nx, ny, cost in self.get_neighbors(x, y, time_step):
                if (nx, ny) not in visited:
                    visited[(nx, ny)] = ((x, y), time_step + 1)
                    queue.append((nx, ny, time_step + 1))
        
        self.computation_time = time.time() - start_time
        return []  # No path found

class UniformCostAgent(BaseAgent):
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        start_time = time.time()
        self.nodes_expanded = 0
        
        # Priority queue: (cost, x, y, time, path)
        queue = [(0, start[0], start[1], 0, [])]
        visited = {}  # (x, y) -> best cost
        
        while queue:
            cost, x, y, time_step, path = heapq.heappop(queue)
            self.nodes_expanded += 1
            
            if (x, y) == goal:
                full_path = path + [(x, y)]
                self.computation_time = time.time() - start_time
                self.path_cost = cost
                return full_path
            
            if (x, y) in visited and visited[(x, y)] <= cost:
                continue
                
            visited[(x, y)] = cost
            
            for nx, ny, move_cost in self.get_neighbors(x, y, time_step):
                new_cost = cost + move_cost
                if (nx, ny) not in visited or new_cost < visited[(nx, ny)]:
                    heapq.heappush(queue, (new_cost, nx, ny, time_step + 1, path + [(x, y)]))
        
        self.computation_time = time.time() - start_time
        return []  # No path found

def manhattan_distance(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean_distance(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

class AStarAgent(BaseAgent):
    def __init__(self, grid: MapGrid, heuristic: str = 'manhattan', **kwargs):
        super().__init__(grid, **kwargs)
        self.heuristic = heuristic     
        if heuristic == 'manhattan':
            self.h_func = manhattan_distance
        elif heuristic == 'euclidean':
            self.h_func = euclidean_distance
        else:
            raise ValueError(f"Unknown heuristic: {heuristic}")
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        start_time = time.time()
        self.nodes_expanded = 0
        
        # Priority queue: (f, g, x, y, time, path)
        g_score = {start: 0}
        f_score = {start: self.h_func(start, goal)}
        queue = [(f_score[start], 0, start[0], start[1], 0, [])]
        
        while queue:
            f, g, x, y, time_step, path = heapq.heappop(queue)
            self.nodes_expanded += 1
            
            if (x, y) == goal:
                full_path = path + [(x, y)]
                self.computation_time = time.time() - start_time
                self.path_cost = g
                return full_path
            
            if g > g_score.get((x, y), float('inf')):
                continue
                
            for nx, ny, move_cost in self.get_neighbors(x, y, time_step):
                new_g = g + move_cost
                if new_g < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = new_g
                    f_score = new_g + self.h_func((nx, ny), goal)
                    heapq.heappush(queue, (f_score, new_g, nx, ny, time_step + 1, path + [(x, y)]))
        
        self.computation_time = time.time() - start_time
        return []  # No path found

class SimulatedAnnealingAgent(BaseAgent):
    def __init__(self, grid: MapGrid, initial_temp: float = 1000, cooling_rate: float = 0.95, 
                 max_iterations: int = 1000, **kwargs):
        super().__init__(grid, **kwargs)
        self.initial_temp = initial_temp
        self.cooling_rate = cooling_rate
        self.max_iterations = max_iterations
        self.a_star_agent = AStarAgent(grid, heuristic='manhattan')
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        start_time = time.time()
        
        # First, find an initial path using A*
        initial_path = self.a_star_agent.find_path(start, goal)
        if not initial_path:
            return []
        
        current_path = initial_path
        current_cost = self.calculate_path_cost(current_path)
        best_path = current_path
        best_cost = current_cost
        
        temperature = self.initial_temp
        self.nodes_expanded = self.a_star_agent.nodes_expanded
        
        for i in range(self.max_iterations):
            # Generate a neighbor by perturbing the path
            neighbor_path = self.perturb_path(current_path)
            neighbor_cost = self.calculate_path_cost(neighbor_path)
            
            # Accept the neighbor if it's better or with some probability if it's worse
            if neighbor_cost < current_cost or random.random() < math.exp((current_cost - neighbor_cost) / temperature):
                current_path = neighbor_path
                current_cost = neighbor_cost
                
                if current_cost < best_cost:
                    best_path = current_path
                    best_cost = current_cost
            
            # Cool down
            temperature *= self.cooling_rate
            self.nodes_expanded += 1
        
        self.computation_time = time.time() - start_time
        self.path_cost = best_cost
        return best_path
    
    def calculate_path_cost(self, path: List[Tuple[int, int]]) -> float:
        total_cost = 0
        time_step = 0
        for i in range(len(path) - 1):
            x, y = path[i]
            nx, ny = path[i + 1]
            total_cost += self.grid.calculate_movement_cost(nx, ny, time_step)
            time_step += 1
        return total_cost
    
    def perturb_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        if len(path) <= 2:
            return path[:]
        
        # Randomly select a segment to replan
        start_idx = random.randint(0, len(path) - 2)
        end_idx = random.randint(start_idx + 1, len(path) - 1)
        
        segment_start = path[start_idx]
        segment_goal = path[end_idx]
        
        # Replan this segment using A*
        segment_path = self.a_star_agent.find_path(segment_start, segment_goal)
        
        if segment_path:
            return path[:start_idx] + segment_path + path[end_idx + 1:]
        else:
            return path  # If replanning fails, return original path

def display_mapgrid(grid: MapGrid, path: List[Tuple[int, int]] = None, 
                  title: str = "Grid Visualization"):
    # Create a color map
    color_map = {
        TerrainType.EMPTY: [1, 1, 1],        # White
        TerrainType.OBSTACLE: [0, 0, 0],     # Black
        TerrainType.EXPRESSWAY: [0.8, 0.8, 0.8],   # Light gray
        TerrainType.PARK: [0, 1, 0],        # Green
        TerrainType.WATER: [0, 0, 1],        # Blue
        TerrainType.BUILDING: [0.5, 0.5, 0.5] # Dark gray
    }
    
    # Create image
    img = np.zeros((grid.height, grid.width, 3))
    for y in range(grid.height):
        for x in range(grid.width):
            cell_type = grid.grid[y, x]
            img[y, x] = color_map[cell_type]
    
    # Plot the grid
    plt.figure(figsize=(10, 10))
    plt.imshow(img)
    plt.title(title)
    
    # Plot the path if provided
    if path:
        xs, ys = zip(*path)
        plt.plot(xs, ys, 'r-', linewidth=2)
        plt.plot(xs[0], ys[0], 'go', markersize=10)  # Start
        plt.plot(xs[-1], ys[-1], 'bo', markersize=10)  # Goal
    
    plt.grid(True)
    plt.show()

def create_test_maps():
    """Create test map files for the project"""
    
    # Small map
    with open('small.map', 'w') as f:
        f.write("5,5\n")
        f.write(".....\n")
        f.write(".EEE.\n")
        f.write(".PPP.\n")
        f.write(".WWW.\n")
        f.write(".....\n")
    
    # Medium map
    with open('medium.map', 'w') as f:
        f.write("10,10\n")
        f.write("..........\n")
        f.write(".EEEE.....\n")
        f.write(".PPPP.....\n")
        f.write(".WWWW.....\n")
        f.write("..........\n")
        f.write("..........\n")
        f.write("..........\n")
        f.write("..........\n")
        f.write("..........\n")
        f.write("..........\n")
    
    # Large map
    with open('large.map', 'w') as f:
        f.write("20,20\n")
        f.write("....................\n")
        f.write(".EEEEEEEEEEEEEEEEEE.\n")
        f.write(".PPPPPPPPPPPPPPPPPP.\n")
        f.write(".WWWWWWWWWWWWWWWWWW.\n")
        f.write(".EEEEEEEEEEEEEEEEEE.\n")
        f.write(".PPPPPPPPPPPPPPPPPP.\n")
        f.write(".WWWWWWWWWWWWWWWWWW.\n")
        f.write(".EEEEEEEEEEEEEEEEEE.\n")
        f.write(".PPPPPPPPPPPPPPPPPP.\n")
        f.write(".WWWWWWWWWWWWWWWWWW.\n")
        f.write(".EEEEEEEEEEEEEEEEEE.\n")
        f.write(".PPPPPPPPPPPPPPPPPP.\n")
        f.write(".WWWWWWWWWWWWWWWWWW.\n")
        f.write(".EEEEEEEEEEEEEEEEEE.\n")
        f.write(".PPPPPPPPPPPPPPPPPP.\n")
        f.write(".WWWWWWWWWWWWWWWWWW.\n")
        f.write(".EEEEEEEEEEEEEEEEEE.\n")
        f.write(".PPPPPPPPPPPPPPPPPP.\n")
        f.write(".WWWWWWWWWWWWWWWWWW.\n")
        f.write("....................\n")
    
    # Dynamic map with moving obstacles
    with open('dynamic.map', 'w') as f:
        f.write("5,5\n")
        f.write(".....\n")
        f.write(".EEE.\n")
        f.write(".PPP.\n")
        f.write(".WWW.\n")
        f.write(".....\n")
        f.write("0:1,1;2,2\n")
        f.write("1:2,1;1,2\n")
        f.write("2:3,1;2,2\n")
        f.write("3:2,1;3,2\n")
        f.write("4:1,1;2,2\n")

def compare_algorithms():
    """Compare all algorithms on different map sizes"""
    maps = ['small.map', 'medium.map', 'large.map', 'dynamic.map']
    algorithms = ['bfs', 'ucs', 'astar', 'sa']
    
    results = {}
    
    for map_name in maps:
        results[map_name] = {}
        grid = MapGrid.load_from_file(map_name)
        
        # Set start and goal positions
        start = (0, 0)
        goal = (grid.width-1, grid.height-1)
        
        for algo in algorithms:
            if algo == 'bfs':
                agent = BreadthFirstAgent(grid)
            elif algo == 'ucs':
                agent = UniformCostAgent(grid)
            elif algo == 'astar':
                agent = AStarAgent(grid, heuristic='manhattan')
            elif algo == 'sa':
                agent = SimulatedAnnealingAgent(grid)
            
            path = agent.find_path(start, goal)
            metrics = agent.get_metrics()
            
            results[map_name][algo] = {
                'path_found': len(path) > 0,
                'path_length': len(path) if path else 0,
                'path_cost': metrics['path_cost'],
                'nodes_expanded': metrics['nodes_expanded'],
                'computation_time': metrics['computation_time']
            }
    
    # Print results
    print("Algorithm Comparison Results")
    print("=" * 80)
    for map_name, algo_results in results.items():
        print(f"\nMap: {map_name}")
        print("-" * 40)
        for algo, metrics in algo_results.items():
            print(f"{algo.upper():<6}: Found={metrics['path_found']}, "
                  f"Cost={metrics['path_cost']:.2f}, "
                  f"Nodes={metrics['nodes_expanded']}, "
                  f"Time={metrics['computation_time']:.4f}s")

def run_with_defaults():
    """Run the program with default arguments for VS Code"""
    print("Running with default arguments for VS Code...")
    print("Creating test maps...")
    create_test_maps()
    
    # Default configuration
    map_file = "small.map"
    start_x, start_y, goal_x, goal_y = 0, 0, 4, 4
    algorithm = "astar"
    heuristic = "manhattan"
    visualize = True
    
    print(f"Using map: {map_file}")
    print(f"Start: ({start_x}, {start_y}), Goal: ({goal_x}, {goal_y})")
    print(f"Algorithm: {algorithm}, Heuristic: {heuristic}")
    
    # Load the grid
    grid = MapGrid.load_from_file(map_file)
    start = (start_x, start_y)
    goal = (goal_x, goal_y)
    terrain_counts = {}
    for y in range(grid.height):
        for x in range(grid.width):
            terrain = grid.grid[y, x]
            terrain_counts[terrain] = terrain_counts.get(terrain, 0) + 1

    print(f"  - Traversable cells: {terrain_counts.get(TerrainType.EMPTY, 0) + terrain_counts.get(TerrainType.EXPRESSWAY, 0) + terrain_counts.get(TerrainType.PARK, 0) + terrain_counts.get(TerrainType.WATER, 0)}")
    print(f"  - Obstacle cells: {terrain_counts.get(TerrainType.OBSTACLE, 0) + terrain_counts.get(TerrainType.BUILDING, 0)}")
    print(f"  - Special terrain: Expressway({terrain_counts.get(TerrainType.EXPRESSWAY, 0)}), Park({terrain_counts.get(TerrainType.PARK, 0)}), Water({terrain_counts.get(TerrainType.WATER, 0)})")
    print(f"  - Total cells: {grid.width * grid.height}")
    print()
    
    # Create the appropriate agent
    if algorithm == "bfs":
        agent = BreadthFirstAgent(grid)
    elif algorithm == "ucs":
        agent = UniformCostAgent(grid)
    elif algorithm == "astar":
        agent = AStarAgent(grid, heuristic)
    elif algorithm == "sa":
        agent = SimulatedAnnealingAgent(grid)
    
    # Find the path
    print("Finding path...")
    path = agent.find_path(start, goal)
    
    # Print results
    print(f"Path found: {'Yes' if path else 'No'}")
    
    if path:
        print(f"Path length: {len(path)}")
        print(f"Path cost: {agent.path_cost}")
        print(f"Nodes expanded: {agent.nodes_expanded}")
        print(f"Computation time: {agent.computation_time:.4f} seconds")
        
        if visualize:
            print("Showing visualization...")
            display_mapgrid(grid, path, f"Path found by {algorithm}")
    else:
        print("No path found!")

def main():
    # Check if running without arguments (VS Code case)
    if len(sys.argv) == 1:
        run_with_defaults()
        return
    
    parser = argparse.ArgumentParser(description="Autonomous Delivery Agent")
    parser.add_argument("map_file", help="Path to the map file")
    parser.add_argument("start_x", type=int, help="Start X coordinate")
    parser.add_argument("start_y", type=int, help="Start Y coordinate")
    parser.add_argument("goal_x", type=int, help="Goal X coordinate")
    parser.add_argument("goal_y", type=int, help="Goal Y coordinate")
    parser.add_argument("--algorithm", "-a", choices=["bfs", "ucs", "astar", "sa"], 
                        default="astar", help="Pathfinding algorithm to use")
    parser.add_argument("--heuristic", choices=["manhattan", "euclidean"], 
                        default="manhattan", help="Heuristic for A*")
    parser.add_argument("--visualize", "-v", action="store_true", 
                        help="Visualize the path")
    parser.add_argument("--fuel", "-f", type=float, default=float('inf'),
                        help="Fuel constraint")
    parser.add_argument("--compare", "-c", action="store_true",
                        help="Compare all algorithms on the given map")
    
    args = parser.parse_args()
    
    # Create test maps if they don't exist
    if args.map_file in ['small.map', 'medium.map', 'large.map', 'dynamic.map']:
        try:
            open(args.map_file, 'r').close()
        except FileNotFoundError:
            print(f"Creating test map: {args.map_file}")
            create_test_maps()
    
    # Load the grid
    grid = MapGrid.load_from_file(args.map_file)
    start = (args.start_x, args.start_y)
    goal = (args.goal_x, args.goal_y)
    
    if args.compare:
        compare_algorithms()
        return
    
    # Create the appropriate agent
    if args.algorithm == "bfs":
        agent = BreadthFirstAgent(grid, args.fuel)
    elif args.algorithm == "ucs":
        agent = UniformCostAgent(grid, args.fuel)
    elif args.algorithm == "astar":
        agent = AStarAgent(grid, args.heuristic, fuel_constraint=args.fuel)
    elif args.algorithm == "sa":
        agent = SimulatedAnnealingAgent(grid, fuel_constraint=args.fuel)
    
    # Find the path
    path = agent.find_path(start, goal)
    
    # Print results
    print(f"Algorithm: {args.algorithm}")
    if args.algorithm == "astar":
        print(f"Heuristic: {args.heuristic}")
    print(f"Path found: {'Yes' if path else 'No'}")
    
    if path:
        print(f"Path length: {len(path)}")
        print(f"Path cost: {agent.path_cost}")
        print(f"Nodes expanded: {agent.nodes_expanded}")
        print(f"Computation time: {agent.computation_time:.4f} seconds")
        
        if args.visualize:
            display_mapgrid(grid, path, f"Path found by {args.algorithm}")
    else:
        print("No path found!")

if __name__ == "__main__":
    main()