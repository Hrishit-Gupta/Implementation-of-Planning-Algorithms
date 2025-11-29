'''
Step 1 : define env size and plot obstacles
Step 2 : generate random points and if it lies inside obstacle then discard it
Step 3 : make a graph with each node has k nearest points and their edges should not collide with obstacle
Step 4 : find the shortest path using start and goal point. 
'''
import random
import matplotlib.pyplot as plt
from typing import List
import numpy as np
from scipy.spatial import KDTree
import networkx as nx
import time

fig, ax = plt.subplots(figsize=(12, 8))

def point_inside_obstacle(point: tuple, obstacles: List[tuple]) -> bool:
    for x, y, r in obstacles:
        dist = np.sqrt((point[0] - x)**2 + (point[1] - y)**2)
        if dist < r:
            return True
    return False

def random_point_generator(num: int, env_size: List[float], obstacles: List[tuple]):
    points = []
    while len(points) != num:
        x = random.uniform(0, env_size[0])
        y = random.uniform(0, env_size[1])
        if not point_inside_obstacle((x, y), obstacles):
            points.append((x, y))
            ax.scatter(x, y, c='blue', s=20)
    return points

def draw_obstacles(obstacles: List[tuple]):
    for x, y, r in obstacles:
        circle = plt.Circle((x, y), r, color='green', alpha=0.4)
        ax.add_patch(circle)
    return obstacles

def collision_free(p1: tuple, p2: tuple, obstacles: List[tuple], step_size=0.1) -> bool:
    x1, y1 = p1
    x2, y2 = p2
    dist = np.hypot(x2 - x1, y2 - y1)
    steps = int(dist / step_size)
    for i in range(steps + 1):
        u = i / steps
        x = x1 + u * (x2 - x1)
        y = y1 + u * (y2 - y1)
        if point_inside_obstacle((x, y), obstacles):
            return False
    return True

def build_graph(points: List[tuple], obstacles: List[tuple], start: tuple, goal: tuple, k=5):
    if point_inside_obstacle(start, obstacles) or point_inside_obstacle(goal, obstacles):
        raise ValueError("Start or goal is inside an obstacle!")

    ax.scatter(start[0], start[1], c='red', s=100, marker='*', label='Start')
    ax.scatter(goal[0], goal[1], c='magenta', s=100, marker='*', label='Goal')

    points = points + [start, goal]
    graph = {p: [] for p in points}

    # KDTree for nearest neighbor search
    tree = KDTree(points)

    # Connect each point to k nearest neighbors
    for i, p in enumerate(points):
        dists, idxs = tree.query(p, k=k+1)
        for j in idxs[1:]:
            q = points[j]
            if collision_free(p, q, obstacles):
                graph[p].append(q)
                graph[q].append(p)
                ax.plot([p[0], q[0]], [p[1], q[1]], c='black', alpha=0.5)

    return graph

def compute_path_length(path: List[tuple]) -> float:
    """Compute Euclidean path length"""
    if not path or len(path) < 2:
        return 0.0
    return sum(np.hypot(path[i+1][0]-path[i][0], path[i+1][1]-path[i][1]) for i in range(len(path)-1))

def find_shortest_path(graph, start, goal):
    G = nx.Graph()
    for p, neighbors in graph.items():
        for q in neighbors:
            dist = np.hypot(q[0]-p[0], q[1]-p[1])
            G.add_edge(p, q, weight=dist)

    # Shortest path
    path = nx.shortest_path(G, source=start, target=goal, weight='weight')
    path_length = compute_path_length(path)

    print("Path:", path)
    print(f"Path length: {path_length:.4f}")

    # Plot the path
    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]
    ax.plot(path_x, path_y, c='orange', linewidth=3, label='PRM path')

    return path, path_length



env_size = [30, 30]
obstacles = [(4.5, 3, 2), (3, 12, 2), (15, 15, 3)]  # (x,y,r)
start = (1, 1)
goal = (20, 20)

draw_obstacles(obstacles)

# Measure time
start_time = time.time()

pts = random_point_generator(50, env_size, obstacles)
graph = build_graph(pts, obstacles, start, goal, k=5)
path, path_length = find_shortest_path(graph, start, goal)

sim_time = time.time() - start_time

print(f"Simulation time: {sim_time:.4f} sec")

ax.set_xlim(0, env_size[0])
ax.set_ylim(0, env_size[1])
plt.legend()
plt.show()
