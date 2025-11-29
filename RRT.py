import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import random, time
from typing import List
from scipy.spatial import KDTree

def draw_obstacles(ax, obstacles: List[tuple]):
    for x, y, r in obstacles:
        circle = plt.Circle((x, y), r, color='green', alpha=0.4)
        ax.add_patch(circle)

def point_inside_obstacle(point: tuple, obstacles: List[tuple]) -> bool:
    for x, y, r in obstacles:
        if np.hypot(point[0] - x, point[1] - y) < r:
            return True
    return False

def collision_free(p1: tuple, p2: tuple, obstacles: List[tuple], step_size=0.1) -> bool:
    x1, y1 = p1
    x2, y2 = p2
    dist = np.hypot(x2 - x1, y2 - y1)
    steps = max(int(dist / step_size), 1)
    for i in range(steps + 1):
        u = i / steps
        x = x1 + u * (x2 - x1)
        y = y1 + u * (y2 - y1)
        if point_inside_obstacle((x, y), obstacles):
            return False
    return True


def steer(p1, p2, delta):
    x1, y1 = p1
    x2, y2 = p2
    theta = np.arctan2(y2 - y1, x2 - x1)
    return (x1 + delta * np.cos(theta), y1 + delta * np.sin(theta))

def build_rrt(start, goal, obstacles, env_size, delta=0.5, max_iter=500, goal_sample_rate=0.05):
    points = [start]
    parents = {start: None}
    edges = []

    for i in range(max_iter):
        if random.random() < goal_sample_rate:
            x_rand = goal
        else:
            x_rand = (random.uniform(0, env_size[0]), random.uniform(0, env_size[1]))

        tree = KDTree(points)
        _, idx = tree.query(x_rand)
        nearest = points[idx]

        new_point = steer(nearest, x_rand, delta)

        if collision_free(nearest, new_point, obstacles):
            points.append(new_point)
            parents[new_point] = nearest
            edges.append((nearest, new_point))

            if np.hypot(new_point[0] - goal[0], new_point[1] - goal[1]) < delta:
                parents[goal] = new_point
                points.append(goal)
                edges.append((new_point, goal))
                print("Goal reached in", i, "iterations")
                return points, parents, edges, True

    return points, parents, edges, False

def extract_path(parents, start, goal):
    path = [goal]
    node = goal
    while node != start:
        node = parents[node]
        path.append(node)
    path.reverse()
    return path

def get_path_length(path):
    """Compute Euclidean length of a given path"""
    if not path or len(path) < 2:
        return 0.0
    return sum(np.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1]) for i in range(len(path)-1))


def animate_rrt(start, goal, obstacles, env_size, delta=0.5, max_iter=500):
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_xlim(0, env_size[0])
    ax.set_ylim(0, env_size[1])
    draw_obstacles(ax, obstacles)
    ax.scatter(*start, c='blue', s=80, label="Start")
    ax.scatter(*goal, c='red', s=80, label="Goal")

    # Time measurement starts
    start_time = time.time()
    points, parents, edges, reached = build_rrt(start, goal, obstacles, env_size, delta, max_iter)
    sim_time = time.time() - start_time  # simulation time
    path = extract_path(parents, start, goal) if reached else []
    path_len = get_path_length(path) if reached else None

    print(f"Simulation time: {sim_time:.4f} sec")
    if path_len:
        print(f"Path length: {path_len:.4f}")

    # Plot elements
    edge_lines, = ax.plot([], [], 'g-', linewidth=0.5)
    path_line, = ax.plot([], [], 'r-', linewidth=2)
    nodes_scatter = ax.scatter([], [], c='black', s=10, label="Nodes")

    # Data for animation
    edge_x, edge_y = [], []
    path_x, path_y = [], []
    all_nodes = []

    def init():
        edge_lines.set_data([], [])
        path_line.set_data([], [])
        nodes_scatter.set_offsets(np.empty((0, 2)))
        return edge_lines, path_line, nodes_scatter

    def update(frame):
        nonlocal edge_x, edge_y, path_x, path_y, all_nodes
        if frame < len(edges):
            p1, p2 = edges[frame]
            edge_x.extend([p1[0], p2[0], None])
            edge_y.extend([p1[1], p2[1], None])
            edge_lines.set_data(edge_x, edge_y)

            all_nodes.append(p2)
            nodes_scatter.set_offsets(np.array(all_nodes))
        else:
            idx = frame - len(edges)
            if idx < len(path):
                path_x.append(path[idx][0])
                path_y.append(path[idx][1])
                path_line.set_data(path_x, path_y)

        return edge_lines, path_line, nodes_scatter

    total_frames = len(edges) + len(path)
    ani = animation.FuncAnimation(fig, update, frames=total_frames,
                                  init_func=init, interval=30, blit=True, repeat=False)
    plt.legend()
    plt.show()
    return ani, path_len, sim_time


if __name__ == "__main__":
    env_size = [30, 30]
    obstacles = [(4.5, 3, 2), (3, 12, 2), (15, 15, 3)]
    start = (1, 1)
    goal = (20, 20)

    ani, path_length, sim_time = animate_rrt(start, goal, obstacles, env_size, delta=0.7, max_iter=2000)
    print(f"Final Path Length = {path_length}, Simulation Time = {sim_time:.4f} sec")
