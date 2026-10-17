import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.collections import LineCollection
import numpy as np

from hw_2.config_space import configuration_space

def draw_obstacles(ax, obstacles, inflation=0.0):
    for k, (n_min, n_max, e_min, e_max) in enumerate(obstacles):
        if inflation > 0.0:
            ax.add_patch(Rectangle(
                (e_min - inflation, n_min -inflation),
                (e_max - e_min) + 2 * inflation,
                (n_max - n_min) + 2 * inflation,
                facecolor="orange",edgecolor="darkorange",alpha=0.35,
                zorder=2))
        ax.add_patch(Rectangle(
            (e_min, n_min), e_max - e_min, n_max - n_min,
            facecolor="dimgrey",edgecolor="black",
            zorder=3))

def draw_start_goal(ax, start, goal):
    ax.plot(start[1], start[0], "go", markersize=10, label=f"Start {start} m")
    ax.plot(goal[1], goal[0], "r*", markersize=14, label=f"Goal {goal} m")

def format_axes(cs: configuration_space, ax, title):
    ax.set_xlim(cs.grid_east[0],cs.grid_east[1])
    ax.set_ylim(cs.grid_north[0],cs.grid_north[1])
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(title)
    
    # start, stop, step size
    ticks = np.arange(cs.grid_east[0],cs.grid_north[1]+1,10)
    
    # labels = [str(t) if t % 10 == 0 else "" for t in ticks]
    labels = [str(t) for t in ticks]
    
    ax.set_xticks(ticks)
    ax.set_yticks(ticks)
    ax.set_xticklabels(labels)
    ax.set_yticklabels(labels)
    ax.grid(True, alpha=0.3)
    
def plot_search_result(cs, algorithm,start, goal, explored_ids, path_ids, title):
    fig, ax = plt.subplots(figsize=(8, 8))
    draw_obstacles(ax, cs.obstacles, cs.safety_rad)

    # explored nodes: convert IDs -> (N, E) centers, plot East on x, North on y
    pts = np.array([cs.get_point_from_node_id(i) for i in explored_ids])
    sc = ax.scatter(pts[:, 1], pts[:, 0], color="blue",
                    s=6, zorder=1, label=f"Explored ({len(pts)} nodes)")

    # final path on top
    path = np.array([cs.get_point_from_node_id(i) for i in path_ids])
    ax.plot(path[:, 1], path[:, 0], "r-", lw=2, zorder=5, label=f"{algorithm} path")

    draw_start_goal(ax, start, goal)
    format_axes(cs, ax, title)
    ax.legend(loc="lower right")
    
    save_path = f"hw_2/hw_2_plots/plot_{algorithm}.png"
    fig.savefig(save_path, dpi=200,bbox_inches="tight")
    return fig


def plot_config_space(cs:configuration_space, start, goal, safety_radius=0.0):
    fig, axes = plt.subplots(1, 2, figsize=(14,7))
    
    for ax, inflation, title in [(axes[0],0.0,"Original environment"),
                                 (axes[1], safety_radius, "Inflated environment")]:
        draw_obstacles(ax, cs.obstacles, inflation)
        draw_start_goal(ax, start, goal)
        format_axes(cs, ax, title)
        ax.legend(loc="lower right")
    
    # draw obstacles 
    plt.tight_layout()
    
    
    save_path = "hw_2/hw_2_plots/config_space.png"
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    
    #plt.show()
    
    return fig


def plot_rrt_result(cs, result, start, goal, title, filename):
    fig, ax = plt.subplots(figsize=(8, 8))
    draw_obstacles(ax, cs.obstacles, cs.safety_rad)

    pts, par = result["tree_points"], result["tree_parents"]
    # each edge runs from a node to its parent; plotted as (east, north)
    edges = [[(pts[i][1], pts[i][0]), (pts[par[i]][1], pts[par[i]][0])]
             for i in range(1, len(pts))]
    ax.add_collection(LineCollection(edges, colors="steelblue", linewidths=0.6, zorder=1,
                                     label=f"RRT tree ({result['tree_size']} nodes)"))

    wp = np.array(result["waypoints"])
    ax.plot(wp[:, 1], wp[:, 0], "r-", lw=2.5, zorder=5,
            label=f"Path ({result['path_length']:.1f} m)")

    draw_start_goal(ax, start, goal)
    format_axes(cs, ax, title)
    ax.legend(loc="lower right")
    fig.savefig(f"hw_2/hw_2_plots/{filename}.png", dpi=200, bbox_inches="tight")
    return fig


def plot_waypoint_reduction(cs, name, orig, red, start, goal):
    fig, ax = plt.subplots(figsize=(8, 8))
    draw_obstacles(ax, cs.obstacles, cs.safety_rad)
    o, r = np.array(orig), np.array(red)
    ax.plot(o[:, 1], o[:, 0], "b.-", ms=4, lw=1, zorder=4, label=f"Raw ({len(o)} waypoints)")
    ax.plot(r[:, 1], r[:, 0], "r.-", ms=9, lw=2.5, zorder=5, label=f"Reduced ({len(r)} waypoints)")
    draw_start_goal(ax, start, goal)
    format_axes(cs, ax, f"{name}: raw vs. reduced waypoints")
    ax.legend(loc="lower right")
    fig.savefig(f"hw_2/hw_2_plots/wp_reduction_{name}.png", dpi=200, bbox_inches="tight")
    return fig

def plot_dubins(cs, waypoints, dub, start, goal, r_min, filename):
    fig, ax = plt.subplots(figsize=(8, 8))
    draw_obstacles(ax, cs.obstacles, cs.safety_rad)

    w = np.array(waypoints)
    ax.plot(w[:, 1], w[:, 0], "b.--", ms=9, lw=1.5, zorder=4,
            label=f"Piecewise-linear A* ({len(w)} waypoints, {dub_len(w):.1f} m)")

    s = np.array(dub.samples)
    ax.plot(s[:, 1], s[:, 0], "r-", lw=2, zorder=5,
            label=f"Dubins, R_min = {r_min:.0f} m ({dub.length:.1f} m)")

    if dub.collisions:
        c = np.array(dub.collisions)
        ax.plot(c[:, 1], c[:, 0], "kx", ms=6, zorder=6,
                label=f"Colliding samples ({len(c)})")

    draw_start_goal(ax, start, goal)
    format_axes(cs, ax, "A* reduced path vs. curvature-constrained path")
    ax.legend(loc="lower right")
    fig.savefig(f"hw_2/hw_2_plots/{filename}.png", dpi=200, bbox_inches="tight")
    return fig

def dub_len(w):
    return float(np.sum(np.hypot(np.diff(w[:, 0]), np.diff(w[:, 1]))))