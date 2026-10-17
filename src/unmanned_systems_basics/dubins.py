"""Standalone, standard-library-only Dubins solver in the (x, y, theta) frame."""

import math
from collections.abc import Sequence
from dataclasses import dataclass

LSL, LSR, RSL, RSR, RLR, LRL = range(6)
_MODES = ("LSL", "LSR", "RSL", "RSR", "RLR", "LRL")
type Configuration = tuple[float, float, float]
type Parameters = tuple[float, float, float]


def mod2pi(theta: float) -> float:
    return theta % math.tau


def _sqrt_nonnegative(value: float) -> float | None:
    # Tangent-circle solutions can be slightly negative from roundoff.
    if value < -1e-12:
        return None
    return math.sqrt(max(value, 0.0))


def _lsl(alpha: float, beta: float, d: float) -> Parameters | None:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    p = _sqrt_nonnegative(2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (sa - sb))
    if p is None:
        return None
    tmp = math.atan2(cb - ca, d + sa - sb)
    return mod2pi(-alpha + tmp), p, mod2pi(beta - tmp)


def _lsr(alpha: float, beta: float, d: float) -> Parameters | None:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    p = _sqrt_nonnegative(-2 + d * d + 2 * math.cos(alpha - beta) + 2 * d * (sa + sb))
    if p is None:
        return None
    tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
    return mod2pi(-alpha + tmp), p, mod2pi(-beta + tmp)


def _rsl(alpha: float, beta: float, d: float) -> Parameters | None:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    p = _sqrt_nonnegative(d * d - 2 + 2 * math.cos(alpha - beta) - 2 * d * (sa + sb))
    if p is None:
        return None
    tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
    return mod2pi(alpha - tmp), p, mod2pi(beta - tmp)


def _rsr(alpha: float, beta: float, d: float) -> Parameters | None:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    p = _sqrt_nonnegative(2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (sb - sa))
    if p is None:
        return None
    tmp = math.atan2(ca - cb, d - sa + sb)
    return mod2pi(alpha - tmp), p, mod2pi(-beta + tmp)


def _rlr(alpha: float, beta: float, d: float) -> Parameters | None:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    tmp = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (sa - sb)) / 8
    if abs(tmp) > 1 + 1e-12:
        return None
    p = mod2pi(math.tau - math.acos(max(-1.0, min(1.0, tmp))))
    t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + p / 2)
    return t, p, mod2pi(alpha - beta - t + p)


def _lrl(alpha: float, beta: float, d: float) -> Parameters | None:
    sa, sb = math.sin(alpha), math.sin(beta)
    ca, cb = math.cos(alpha), math.cos(beta)
    tmp = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (-sa + sb)) / 8
    if abs(tmp) > 1 + 1e-12:
        return None
    p = mod2pi(math.tau - math.acos(max(-1.0, min(1.0, tmp))))
    t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2)
    return t, p, mod2pi(beta - alpha - t + p)


_PLANNERS = (_lsl, _lsr, _rsl, _rsr, _rlr, _lrl)


def _configuration(q: Sequence[float]) -> Configuration:
    if len(q) != 3 or not all(math.isfinite(value) for value in q):
        raise ValueError("A configuration must contain three finite values (x, y, theta)")
    return float(q[0]), float(q[1]), mod2pi(q[2])


@dataclass(frozen=True)
class DubinsPath:
    """A forward-only path of straight lines and arcs with turn radius ``rho``.

    Construct with ``shortest_path(q0, q1, rho)``. Positions and sampling
    distances are in meters; headings are counter-clockwise from +x in radians.
    """

    _q0: Configuration
    _rho: float
    _parameters: Parameters
    _word: int

    @classmethod
    def shortest_path(cls, q0: Sequence[float], q1: Sequence[float], rho: float) -> DubinsPath:
        """Choose the shortest feasible one of LSL, LSR, RSL, RSR, RLR, LRL."""
        if not math.isfinite(rho) or rho <= 0:
            raise ValueError("Turn radius rho must be finite and positive")
        start, goal = _configuration(q0), _configuration(q1)
        if start == goal:
            return cls(start, rho, (0.0, 0.0, 0.0), LSL)
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        d = math.hypot(dx, dy) / rho
        if not math.isfinite(d * d):
            raise ValueError("Pose separation relative to turn radius is too large")
        theta = math.atan2(dy, dx)
        alpha, beta = mod2pi(start[2] - theta), mod2pi(goal[2] - theta)
        best: Parameters | None = None
        best_word = LSL
        best_cost = math.inf
        for word, planner in enumerate(_PLANNERS):
            parameters = planner(alpha, beta, d)
            if parameters is not None and sum(parameters) < best_cost:
                best, best_word, best_cost = parameters, word, sum(parameters)
        if best is None:
            raise RuntimeError("No feasible Dubins path found")
        if not math.isfinite(best_cost * rho):
            raise ValueError("Dubins path length exceeds the supported numeric range")
        return cls(start, rho, best, best_word)

    def path_type(self) -> int:
        return self._word

    def path_length(self) -> float:
        return sum(self._parameters) * self._rho

    def segment_length_normalized(self, i: int) -> float:
        if i not in (0, 1, 2):
            raise IndexError("Dubins segment index must be 0, 1, or 2")
        return self._parameters[i]

    def segment_length(self, i: int) -> float:
        return self.segment_length_normalized(i) * self._rho

    def sample(self, distance: float) -> Configuration:
        """Evaluate the exact curve at an arc-length distance, including the end."""
        if not math.isfinite(distance) or not 0 <= distance <= self.path_length():
            raise ValueError("Sample distance must be finite and within the path length")
        remaining = distance / self._rho
        x, y, heading = self._q0
        for mode, length in zip(_MODES[self._word], self._parameters):
            amount = min(remaining, length)
            if mode == "S":
                x += self._rho * amount * math.cos(heading)
                y += self._rho * amount * math.sin(heading)
            else:
                sign = 1 if mode == "L" else -1
                # Exact circle displacement, not Euler steps along the tangent.
                half_turn = sign * amount / 2
                chord = self._rho * (2 * math.sin(amount / 2))
                x += chord * math.cos(heading + half_turn)
                y += chord * math.sin(heading + half_turn)
                heading += sign * amount
            remaining -= amount
            if remaining <= 0:
                break
        return x, y, mod2pi(heading)

    def sample_many(self, step_size: float) -> tuple[list[Configuration], list[float]]:
        """Return (poses, distances) at 0, step_size, ... strictly before the end.

        Like pydubins, an exactly zero-length path returns two empty lists.
        Use ``sample(path_length())`` to evaluate the endpoint.
        """
        if not math.isfinite(step_size) or step_size <= 0:
            raise ValueError("Sample step must be finite and positive")
        length = self.path_length()
        distances = [i * step_size for i in range(math.ceil(length / step_size))
                     if i * step_size < length]
        return [self.sample(distance) for distance in distances], distances


def shortest_path(q0: Sequence[float], q1: Sequence[float], rho: float) -> DubinsPath:
    """Module-level compatibility entry point for ``dubins.shortest_path(...)``."""
    return DubinsPath.shortest_path(q0, q1, rho)
