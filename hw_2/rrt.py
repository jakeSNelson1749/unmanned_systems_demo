import time
import numpy as np

import hw_2.utils as utils
from hw_2.config_space import configuration_space


def sample_point(cs: configuration_space, rng: np.random.Generator) -> tuple[float, float]:
    """Uniform random sample inside the usable (inflated-boundary) region. Returns (north, east)."""
    e_min, e_max, n_min, n_max = cs.inf_bounds
    north = rng.uniform(n_min, n_max)
    east = rng.uniform(e_min, e_max)
    return (north, east)


def nearest_index(tree_points: list[tuple[float, float]], q: tuple[float, float]) -> int:
    """Index of the tree node closest to q."""
    pts = np.asarray(tree_points)
    d2 = (pts[:, 0] - q[0]) ** 2 + (pts[:, 1] - q[1]) ** 2
    
    # argmin gives index of smallest value in array
    return int(np.argmin(d2))


def steer(from_pt: tuple[float, float], to_pt: tuple[float, float], step_size: float) -> tuple[float, float]:
    """Move from from_pt toward to_pt by at most step_size."""
    dist = utils.calc_Euclidean_dist(from_pt, to_pt)
    if dist <= step_size:
        return to_pt
    scale = step_size / dist
    return (from_pt[0] + scale * (to_pt[0] - from_pt[0]),
            from_pt[1] + scale * (to_pt[1] - from_pt[1]))


def rrt(cs: configuration_space,
        start: tuple[float, float],
        goal: tuple[float, float],
        step_size: float,
        goal_tol: float,
        max_iters: int,
        seed: int) -> dict:
    """RRT stops when a node is within goal_tol of the goal
    AND the straight segment node->goal is collision free; the exact goal is then
    appended as the final node so the path ends at the true goal."""
    t0 = time.perf_counter()
    rng = np.random.default_rng(seed)

    # tree stored as parallel lists: points[i] = (north, east), parents[i] = parent index
    points = [start]
    parents = [-1]
    goal_idx = -1

    for _ in range(max_iters):
        q_rand = sample_point(cs, rng)
        near_idx = nearest_index(points, q_rand)
        q_near = points[near_idx]
        q_new = steer(q_near, q_rand, step_size)

        # checks the endpoint (inside obstacle / out of bounds) and the whole segment
        if cs.collision_in_line_of_sight(q_near, q_new):
            continue

        points.append(q_new)
        parents.append(near_idx)
        new_idx = len(points) - 1

        # goal detection
        if (utils.calc_Euclidean_dist(q_new, goal) <= goal_tol
                and not cs.collision_in_line_of_sight(q_new, goal)):
            points.append(goal)
            parents.append(new_idx)
            goal_idx = len(points) - 1
            break

    planning_time = time.perf_counter() - t0
    success = goal_idx != -1

    # path reconstruction: walk parents back to the root
    waypoints = []
    if success:
        i = goal_idx
        while i != -1:
            waypoints.append(points[i])
            i = parents[i]
        waypoints.reverse()

    path_length = (sum(utils.calc_Euclidean_dist(a, b) for a, b in zip(waypoints[:-1], waypoints[1:]))
                   if success else float("inf"))

    return {
        "success": success,
        "waypoints": waypoints,
        "path_length": path_length,
        "planning_time_s": planning_time,
        "tree_size": len(points),
        "tree_points": points,
        "tree_parents": parents,
        "seed": seed,
    }