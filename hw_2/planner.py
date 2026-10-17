import math
import time

import numpy as np

#in-project imports
from hw_2.config_space import configuration_space
import hw_2.utils as utils
import hw_2.plotting as plotting
import hw_2.rrt as rrt

# dubins imports
from src.unmanned_systems_basics.dubins_path import dubins_through_waypoints

# main file for the mission planner 

# scenario parameters
start:tuple[float, float] = (10,10)
goal:tuple[float, float] = (190, 180)

cruise_speed_mps = 12.0
waypoint_accept_rad_m = 5
min_turning_rad_m = 5 # changed to 5m for Dubins
safety_inflation_rad_m = 4

# (north_min, north_max, east_min, east_max)
obstacles = [
    (35, 70, 25, 90),
    (85, 120, 75, 130),
    (130, 165, 20, 85),
    (145, 180, 120, 165)
]

# create configuration space with x: [0,200], y[0,200], resolution 2
cs = configuration_space(grid_east=(0,200), 
                         grid_north=(0,200), 
                         resolution=2, 
                         obstacles=obstacles,
                         safety_rad_m=safety_inflation_rad_m)

# plot both original and inflated graphs
plotting.plot_config_space(cs=cs, 
                           start=start, 
                           goal=goal, 
                           safety_radius=safety_inflation_rad_m)

# calculate straight-line dist, desired course angle, and minimum mission time for start/end
straight_line_dist = utils.calc_Euclidean_dist(start, goal)
desired_course_angle_rad = utils.desired_course_angle_rad(start, goal)
min_mission_time_s = straight_line_dist / cruise_speed_mps 

print(f"Problem 1 (part b) values using start: {start} and goal: {goal}")
print(f"Straight-line distance between start and goal = {straight_line_dist: .3f} m")
print(f"Desired course angle (in degrees) between start and goal = {math.degrees(desired_course_angle_rad):.3f} deg")
print(f"Minimum mission time (12 m/s constant speed): {min_mission_time_s:.2f} seconds\n")

print(f"Direct start-goal path is collision free? {not cs.collision_in_line_of_sight(start, goal)}\n")

# Djikstras and A* ran on configuration space
average_time:float = 0.0
tests = 10
for i in range(tests):
    start_time = time.perf_counter()
    temp = cs.djikstras(start,goal)
    end_time = time.perf_counter()
    
    delta_t = end_time - start_time
    average_time += delta_t

average_time = average_time / tests

djikstra_result:dict = cs.djikstras(start, goal)

print(f"Djikstras results for {start} to {goal}:")
# print(f"path ids: {djikstra_result["path_ids"]}")
print(f"path length: {djikstra_result['path_length']:.2f}")
print(f"average planning time: {average_time:.2f}")
print(f"number of expanded nodes: {djikstra_result['expanded_nodes']}")
print(f"number of raw waypoints: {djikstra_result['num_waypoints']}")
print(f"success: {djikstra_result['success']}\n")

plotting.plot_search_result(cs, 
                            "Djikstra",
                            start,
                            goal, 
                            djikstra_result["explored_ids"],
                            djikstra_result["path_ids"],
                            "Djikstra's Algorithm"
                            )

# Problem 2: A*
average_time:float = 0.0
tests = 10
for i in range(tests):
    start_time = time.perf_counter()
    temp = cs.a_star(start,goal)
    end_time = time.perf_counter()
    
    delta_t = end_time - start_time
    average_time += delta_t

a_star_average_time = average_time / tests

a_star_result:dict = cs.a_star(start, goal)

print(f"a_star results for {start} to {goal}:")
# print(f"path ids: {a_star_result["path_ids"]}")
print(f"path length: {a_star_result['path_length']:.2f}")
print(f"average planning time: {a_star_average_time:.2f}")
print(f"number of expanded nodes: {a_star_result['expanded_nodes']}")
print(f"number of raw waypoints: {a_star_result['num_waypoints']}")
print(f"success: {a_star_result['success']}\n")

plotting.plot_search_result(cs, 
                            "Astar",
                            start,
                            goal, 
                            a_star_result["explored_ids"],
                            a_star_result["path_ids"],
                            "A* Algorithm"
                            )


# PROBLEM 3 - RRT
rrt_step_size_m = 5.0
goal_tol_m = 5.0
max_iterations = 5000
seeds = list(range(10))

rrt_results = [rrt.rrt(cs, start, goal, rrt_step_size_m, goal_tol_m, max_iterations,s) for s in seeds]

print("RRT RESULTS")
# per-run table
print(f"{'seed':>4} {'success':>8} {'length (m)':>11} {'time (ms)':>10} {'tree nodes':>11}")
for result in rrt_results:
    length = f"{result['path_length']:.2f}" if result["success"] else "-"
    print(f"{result['seed']:>4} {str(result['success']):>8} {length:>11} "
            f"{result['planning_time_s']*1000:>10.1f} {result['tree_size']:>11}")

# summary table

# ok = all successful results
ok = [r for r in rrt_results if r["success"]]
print("\nRRT summary (n = %d seeds, step = %.1f m, goal tol = %.1f m, max iters = %d)"
        % (len(rrt_results), rrt_step_size_m, goal_tol_m, max_iterations))
print(f"Success rate:              {len(ok)}/{len(rrt_results)} = {100*len(ok)/len(rrt_results):.0f}%")
if not ok:
    print("No successful runs; increase MAX_ITERS or STEP_SIZE.")
    

lengths = np.array([r["path_length"] for r in ok])
print(f"Mean path length:          {lengths.mean():.2f} m")
print(f"Std dev of path length:    {lengths.std(ddof=1) if len(ok) > 1 else 0.0:.2f} m (sample, ddof=1)")
print(f"Mean planning time:        {1000*np.mean([r['planning_time_s'] for r in rrt_results]):.1f} ms")
print(f"Mean tree size:            {np.mean([r['tree_size'] for r in rrt_results]):.0f} nodes")
print(f"Best path length:          {lengths.min():.2f} m")
print(f"Worst successful length:   {lengths.max():.2f} m")

# representative runs: best, worst successful, closest to median
rrt_best = min(ok, key=lambda r: r["path_length"])
rrt_worst = max(ok, key=lambda r: r["path_length"])
rrt_typical = min(ok, key=lambda r: abs(r["path_length"] - np.median(lengths)))
for tag, result in [("best", rrt_best), ("worst", rrt_worst), ("typical", rrt_typical)]:
    plotting.plot_rrt_result(
        cs, result, start, goal,
        title=f"RRT {tag} run (seed {result['seed']}, {result['path_length']:.1f} m, {result['tree_size']} nodes)",
        filename=f"rrt_{tag}")
print(f"\nRepresentative seeds -> best: {rrt_best['seed']}, worst: {rrt_worst['seed']}, typical: {rrt_typical['seed']}")


# Problem 4 - waypoint reduction

orig_djikstra_waypoints = cs.path_ids_to_waypoints(djikstra_result["path_ids"],start, goal)
orig_astar_waypoints = cs.path_ids_to_waypoints(a_star_result["path_ids"],start, goal)
orig_rrt_waypoints = rrt_best["waypoints"]

red_djikstra_waypoints = cs.waypoint_reduction(orig_djikstra_waypoints)
red_astar_waypoints = cs.waypoint_reduction(orig_astar_waypoints)
red_rrt_waypoints = cs.waypoint_reduction(orig_rrt_waypoints)

planners = {
    "Djikstra": (orig_djikstra_waypoints,red_djikstra_waypoints),
    "A_star":(orig_astar_waypoints, red_astar_waypoints),
    "RRT": (orig_rrt_waypoints, red_rrt_waypoints)
}

# Problem 4(a) table (ai generated formatting)
print("\nPROBLEM 4(a): WAYPOINT REDUCTION")
print(f"{'Metric':<26}{'Dijkstra':>12}{'A*':>12}{'RRT':>12}")
rows = [
    ("Original Waypoints",      lambda o, r: f"{len(o)}"),
    ("Reduced Waypoints",       lambda o, r: f"{len(r)}"),
    ("Original Path Length (m)", lambda o, r: f"{utils.polyline_length(o):.2f}"),
    ("Reduced Path Length (m)",  lambda o, r: f"{utils.polyline_length(r):.2f}"),
    ("Est. Mission Time (s)",    lambda o, r: f"{utils.polyline_length(r)/cruise_speed_mps:.2f}"),
]
for label, fn in rows:
    print(f"{label:<26}" + "".join(f"{fn(*planners[n]):>12}" for n in planners))
    
print("\nReduced waypoints (north, east) in m:")
for name, (orig, red) in planners.items():
    print(f"{name}: " + ", ".join(f"({n:.1f}, {e:.1f})" for n, e in red))
    
    
for name, (orig, red) in planners.items():
    plotting.plot_waypoint_reduction(cs, name, orig, red, start, goal)
    
    
# DUBINS

class CSAdapter:
    """Justins dubin class adapter"""
    def __init__(self, cs):
        self.cs = cs
    def is_point_in_collision(self, p):
        return self.cs.collision_check_point(p)

dub = dubins_through_waypoints(red_astar_waypoints, min_turning_rad_m, CSAdapter(cs), sample_step=0.5)

pl_len = utils.polyline_length(red_astar_waypoints)

print("\nPROBLEM 4(c)/(d): A* piecewise-linear vs. Dubins")
print(f"Piecewise-linear: {pl_len:.2f} m, {pl_len / cruise_speed_mps:.2f} s")
print(f"Dubins (R_min = {min_turning_rad_m:.0f} m): {dub.length:.2f} m, {dub.length / cruise_speed_mps:.2f} s")
print(f"Collision free: {dub.collision_free} ")

plotting.plot_dubins(cs, red_astar_waypoints, dub, start, goal, min_turning_rad_m, "dubins_astar")