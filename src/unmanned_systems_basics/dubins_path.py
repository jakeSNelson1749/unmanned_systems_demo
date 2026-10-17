"""
Curvature-constrained (Dubins) paths through a waypoint list, using the local solver.

Frames and angles
-----------------
Planner points are (north, east) in meters, and course angle chi is measured
from North toward East: chi = atan2(dE, dN).

The standalone solver works in a standard math frame (x, y, theta) with theta measured
counter-clockwise from +x. We map
    x = east, y = north, theta = pi/2 - chi
so a course of 0 (due North) becomes theta = pi/2 (+y).
"""
import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

from . import dubins
# check if we have a .planner if not then define Point as a tuple of floats and ConfigSpace
Point = Tuple[float, float]  # (north, east) in meters
class ConfigSpace:
    def is_point_in_collision(self, point: Point) -> bool:
            return False

DUBINS_TYPES = {dubins.LSL: "LSL", dubins.LSR: "LSR", dubins.RSL: "RSL",
                dubins.RSR: "RSR", dubins.RLR: "RLR", dubins.LRL: "LRL"}


def course_angle(p1: Point, p2: Point) -> float:
    """Desired course chi (rad) from p1 to p2, measured from North toward East."""
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def assign_waypoint_courses(waypoints: Sequence[Point]) -> List[float]:
    """Pick a course angle for every waypoint.

    The first and last waypoints use the direction of their only leg. Interior
    waypoints use the bisector of the incoming and outgoing legs, so the
    vehicle starts turning before the corner instead of overshooting it.
    """
    if len(waypoints) < 2:
        raise ValueError("Need at least two waypoints")
    legs = [course_angle(a, b) for a, b in zip(waypoints[:-1], waypoints[1:])]
    courses = [legs[0]]
    for chi_in, chi_out in zip(legs[:-1], legs[1:]):
        courses.append(math.atan2(math.sin(chi_in) + math.sin(chi_out),
                                  math.cos(chi_in) + math.cos(chi_out)))
    courses.append(legs[-1])
    return courses


def to_dubins_config(point: Point, chi: float) -> Tuple[float, float, float]:
    """(north, east), chi  ->  Dubins (x, y, theta)."""
    return (point[1], point[0], math.pi / 2 - chi)


def from_dubins_config(q: Sequence[float]) -> Point:
    """Dubins (x, y, theta)  ->  (north, east)."""
    return (q[1], q[0])


@dataclass
class DubinsResult:
    waypoints: List[Point]
    courses: List[float]                       # course angle at each waypoint [rad]
    segment_types: List[str]                   # e.g. ["LSL", "RSR", ...], one per leg
    segment_lengths: List[float]               # [m], one per leg
    samples: List[Point]                       # densely sampled (north, east) trajectory
    collisions: List[Point] = field(default_factory=list)

    @property
    def length(self) -> float:
        return sum(self.segment_lengths)

    @property
    def collision_free(self) -> bool:
        return not self.collisions


def dubins_through_waypoints(waypoints: Sequence[Point],
                             turn_radius: float,
                             config_space: ConfigSpace | None = None,
                             sample_step: float = 0.5) -> DubinsResult:
    """Connect consecutive waypoints with shortest Dubins paths of radius `turn_radius`.

    If a ConfigSpace is given, every sample of the trajectory is checked against
    map bounds and inflated obstacles, and any colliding samples are reported.
    The path is NOT modified to avoid them.
    """
    courses = assign_waypoint_courses(waypoints)
    types, lengths, samples = [], [], []
    for i in range(len(waypoints) - 1):
        q0 = to_dubins_config(waypoints[i], courses[i])
        q1 = to_dubins_config(waypoints[i + 1], courses[i + 1])
        path = dubins.shortest_path(q0, q1, turn_radius)
        types.append(DUBINS_TYPES[path.path_type()])
        lengths.append(path.path_length())
        configs, _ = path.sample_many(sample_step)
        samples.extend(from_dubins_config(q) for q in configs)
    samples.append(waypoints[-1])  # sample_many stops just short of the end point

    collisions = []
    if config_space is not None:
        collisions = [p for p in samples if config_space.is_point_in_collision(p)]
    return DubinsResult(list(waypoints), courses, types, lengths, samples, collisions)
