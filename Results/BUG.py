# this is implementation of bug2

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from typing import List
import time


def draw_obstacles(ax, obstacles: List[tuple]):
    for x, y, r in obstacles:
        circle = plt.Circle((x, y), r, color='green', alpha=0.4)
        ax.add_patch(circle)

def draw_start_and_goal(ax, start: tuple, goal: tuple):
    ax.scatter(start[0], start[1], c='blue', s=100, marker='o', label='Start')
    ax.scatter(goal[0], goal[1], c='green', s=100, marker='*', label='Goal')
    ax.plot([start[0], goal[0]], [start[1], goal[1]],
            linestyle='--', color='orange', linewidth=2, label='Line of sight')


def get_hit_and_leave(pos: np.ndarray, goal: tuple, obstacles: List[tuple]):
    (x1, y1), (x2, y2) = pos, goal
    dx, dy = x2 - x1, y2 - y1
    first_hit = None
    first_leave = None
    first_distance = float('inf')
    hit_obstacle = None

    for (xc, yc, r) in obstacles:
        A = dx**2 + dy**2
        B = 2 * (dx*(x1 - xc) + dy*(y1 - yc))
        C = (x1 - xc)**2 + (y1 - yc)**2 - r**2
        disc = B**2 - 4*A*C
        if disc >= 0:
            sqrt_disc = np.sqrt(disc)
            t1 = (-B + sqrt_disc) / (2*A)
            t2 = (-B - sqrt_disc) / (2*A)
            intersections = []
            for t in [t1, t2]:
                if 0 <= t <= 1:
                    xi = x1 + t*dx
                    yi = y1 + t*dy
                    intersections.append((xi, yi))
            if len(intersections) == 2:
                d1 = np.hypot(goal[0]-intersections[0][0], goal[1]-intersections[0][1])
                d2 = np.hypot(goal[0]-intersections[1][0], goal[1]-intersections[1][1])
                if d1 > d2:
                    hit, leave = intersections[0], intersections[1]
                else:
                    hit, leave = intersections[1], intersections[0]
                dist_to_hit = np.hypot(hit[0]-pos[0], hit[1]-pos[1])
                if dist_to_hit < first_distance:
                    first_hit, first_leave = hit, leave
                    first_distance = dist_to_hit
                    hit_obstacle = (xc, yc, r)
    return first_hit, first_leave, hit_obstacle


def simulate_bug2(start, goal, obstacles, step_size=0.1):
    path = []
    hit_points = []
    leave_points = []
    pos = np.array(start, dtype=float)
    goal_arr = np.array(goal, dtype=float)

    while np.linalg.norm(pos - goal_arr) > step_size:
        hit, leave, obstacle = get_hit_and_leave(pos, goal, obstacles)
        if hit is None:
            # Move straight to goal
            while np.linalg.norm(pos - goal_arr) > step_size:
                direction = goal_arr - pos
                direction /= np.linalg.norm(direction)
                pos += step_size * direction
                path.append(pos.copy())
            break

        # Move to hit point
        hit = np.array(hit)
        while np.linalg.norm(pos - hit) > step_size:
            direction = hit - pos
            direction /= np.linalg.norm(direction)
            pos += step_size * direction
            path.append(pos.copy())
        hit_points.append(hit.copy())

        # Trace obstacle boundary (choose shortest path)
        xc, yc, r = obstacle
        leave = np.array(leave)
        leave_points.append(leave.copy())

        angle = np.arctan2(pos[1]-yc, pos[0]-xc)
        leave_angle = np.arctan2(leave[1]-yc, leave[0]-xc)
        clockwise_dist = (angle - leave_angle) % (2*np.pi)
        counter_dist = (leave_angle - angle) % (2*np.pi)

        if counter_dist < clockwise_dist:
            angles = np.linspace(angle, leave_angle, max(int(counter_dist/(step_size/r)),1))
        else:
            angles = np.linspace(angle, leave_angle-2*np.pi, max(int(clockwise_dist/(step_size/r)),1))

        for a in angles:
            x = xc + r*np.cos(a)
            y = yc + r*np.sin(a)
            pos = np.array([x, y])
            path.append(pos.copy())

    path.append(goal_arr.copy())
    return np.array(path), np.array(hit_points), np.array(leave_points)

def compute_path_length(path: np.ndarray) -> float:
    """Compute Euclidean length of a path"""
    if path is None or len(path) < 2:
        return 0.0
    return sum(np.linalg.norm(path[i+1] - path[i]) for i in range(len(path)-1))


def animate_path(path, hit_points, leave_points, env_size=[30,30], obstacles=[], start=None, goal=None):
    fig, ax = plt.subplots(figsize=(12, 8))
    draw_obstacles(ax, obstacles)
    draw_start_and_goal(ax, start, goal)

    agent, = ax.plot([], [], 'ro', markersize=8, label="Agent")
    trace, = ax.plot([], [], 'r-', linewidth=1)
    hit_plot, = ax.plot([], [], 'kx', markersize=10, label='Hit Point')
    leave_plot, = ax.plot([], [], 'cd', markersize=10, label='Leave Point')

    def init():
        agent.set_data([], [])
        trace.set_data([], [])
        hit_plot.set_data([], [])
        leave_plot.set_data([], [])
        return agent, trace, hit_plot, leave_plot

    def update(frame):
        x, y = path[frame]
        agent.set_data([x], [y])
        trace.set_data(path[:frame+1,0], path[:frame+1,1])
        if len(hit_points) > 0:
            hit_plot.set_data(hit_points[:,0], hit_points[:,1])
        if len(leave_points) > 0:
            leave_plot.set_data(leave_points[:,0], leave_points[:,1])
        return agent, trace, hit_plot, leave_plot

    ani = animation.FuncAnimation(fig, update, frames=len(path),
                                  init_func=init, interval=30, blit=True, repeat=False)

    ax.set_xlim(0, env_size[0])
    ax.set_ylim(0, env_size[1])
    ax.legend()
    plt.show()

    return ani


# here the data is according to the question
if __name__ == "__main__":
    env_size = [30, 30]
    obstacles = [(4.5, 3, 2), (3, 12, 2), (15, 15, 3)]
    start = (1, 1)
    goal = (20, 20)

    start_time = time.time()
    path, hit_points, leave_points = simulate_bug2(start, goal, obstacles)
    sim_time = time.time() - start_time

    path_length = compute_path_length(path)

    print(f"Path length: {path_length:.4f}")
    print(f"Simulation time: {sim_time:.4f} sec")

    animate_path(path, hit_points, leave_points, env_size, obstacles, start, goal)
