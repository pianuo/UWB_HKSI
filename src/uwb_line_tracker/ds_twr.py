"""
Utilities for simulating DS-TWR ranging and estimating tag position.
"""

from __future__ import annotations

import math
import random
from typing import Iterable, Sequence, Tuple

from .config import LINE_OPTIONS, SPEED_OF_LIGHT_M_S
from .models import Anchor, RangeMeasurement, TagState, Vector3

Nanoseconds = float


def euclidean_distance(a: Vector3, b: Vector3) -> float:
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def distance_to_tof_ns(distance_m: float) -> Nanoseconds:
    """Convert meters into time-of-flight in nanoseconds."""
    return (distance_m / SPEED_OF_LIGHT_M_S) * 1e9


def simulate_ds_twr(
    anchor: Anchor,
    tag: TagState,
    *,
    noise_std_m: float,
    responder_delay_ns: float,
    clock_drift_ns: float,
    rng: random.Random | None = None,
) -> RangeMeasurement:
    """
    Create a synthetic range measurement for the provided anchor/tag combo.

    The DS-TWR round trip is approximated as:
        RTT = 2 * ToF + responder_delay + clock_drift
    """

    rng = rng or random
    true_distance = euclidean_distance(anchor.position, tag.true_position)
    noisy_distance = true_distance + rng.gauss(0.0, noise_std_m)
    tof_ns = distance_to_tof_ns(noisy_distance) + clock_drift_ns
    rtt_ns = (2.0 * distance_to_tof_ns(noisy_distance)) + responder_delay_ns

    return RangeMeasurement(
        anchor_id=anchor.id,
        tag_id=tag.id,
        tof_ns=tof_ns,
        distance_m=noisy_distance,
        variance_m2=noise_std_m**2,
        round_trip_ns=rtt_ns,
        reply_time_ns=responder_delay_ns,
    )


def _circle_intersection(
    anchor_a: Anchor,
    anchor_b: Anchor,
    dist_a: float,
    dist_b: float,
    *,
    prefer_positive_side: bool,
) -> Tuple[float, float]:
    """
    Solve the intersection of two circles in 2D.

    The anchors are projected onto the XY plane. When two solutions exist we
    pick the one that lies on the requested side of the line defined by A -> B.
    """

    ax, ay, _ = anchor_a.position
    bx, by, _ = anchor_b.position

    dx = bx - ax
    dy = by - ay
    d = math.hypot(dx, dy)
    if d == 0:
        raise ValueError("Anchors must not occupy the same location")

    # Guard numerical edge cases due to noise by clamping to feasible bounds.
    sum_d = dist_a + dist_b
    if sum_d < d:
        scale = (d + 1e-6) / sum_d
        dist_a *= scale
        dist_b *= scale
    diff = abs(dist_a - dist_b)
    if diff > d:
        shrink = (d - 1e-6) / diff
        dist_a *= shrink
        dist_b *= shrink

    # Place A at origin and B on the x-axis.
    ux = dx / d
    uy = dy / d

    x = (dist_a**2 - dist_b**2 + d**2) / (2 * d)
    y_squared = dist_a**2 - x**2
    y = math.sqrt(max(y_squared, 0.0))

    if not prefer_positive_side:
        y = -y

    # Rotate back to world coordinates.
    rx = ax + x * ux - y * uy
    ry = ay + x * uy + y * ux
    return rx, ry


def estimate_tag_position(
    anchors: Sequence[Anchor],
    measurements: Sequence[RangeMeasurement],
    *,
    prefer_positive_side: bool | None = None,
) -> Tuple[float, float]:
    """
    Estimate the tag position in the XY plane using two DS-TWR measurements.

    Returns (x, y). The Z coordinate is assumed to be coplanar with anchors and
    can be inferred from domain knowledge if needed.
    """

    if len(anchors) < 2 or len(measurements) < 2:
        raise ValueError("Need two anchors and two measurements for trilateration")

    anchor_lookup = {anchor.id: anchor for anchor in anchors}
    m_sorted = sorted(measurements, key=lambda m: m.anchor_id)[:2]
    anchor_a = anchor_lookup[m_sorted[0].anchor_id]
    anchor_b = anchor_lookup[m_sorted[1].anchor_id]
    prefer_positive_side = (
        prefer_positive_side
        if prefer_positive_side is not None
        else LINE_OPTIONS["prefer_positive_side"]
    )
    return _circle_intersection(
        anchor_a,
        anchor_b,
        m_sorted[0].distance_m,
        m_sorted[1].distance_m,
        prefer_positive_side=prefer_positive_side,
    )


def side_of_line(anchor_a: Anchor, anchor_b: Anchor, point: Vector3) -> float:
    """Return signed distance from point to the AB line (Z ignored)."""
    ax, ay, _ = anchor_a.position
    bx, by, _ = anchor_b.position
    px, py, _ = point
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax)


def detect_line_crossing(
    anchor_a: Anchor,
    anchor_b: Anchor,
    previous_point: Vector3,
    current_point: Vector3,
) -> bool:
    """True when the tag crosses the virtual start line A->B."""
    return side_of_line(anchor_a, anchor_b, previous_point) == 0 or (
        side_of_line(anchor_a, anchor_b, previous_point)
        * side_of_line(anchor_a, anchor_b, current_point)
        < 0
    )


def signed_distance_to_line(
    anchor_a: Anchor, anchor_b: Anchor, point: Vector3
) -> float:
    """
    Return the signed perpendicular distance (in meters) from the point to line AB.

    Positive values indicate the point lies on the left-hand side of the vector A->B.
    """

    side = side_of_line(anchor_a, anchor_b, point)
    ax, ay, _ = anchor_a.position
    bx, by, _ = anchor_b.position
    denom = math.hypot(bx - ax, by - ay)
    if denom == 0.0:
        return 0.0
    return side / denom





