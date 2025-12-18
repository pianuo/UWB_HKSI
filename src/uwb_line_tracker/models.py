"""
Data models for anchors, tags, measurements, and frames.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Sequence, Tuple

Vector3 = Tuple[float, float, float]


@dataclass
class Anchor:
    """UWB anchor (base station) mounted on a buoy."""

    id: str
    position: Vector3

    @classmethod
    def from_dict(cls, data: dict) -> Anchor:
        return cls(id=data["id"], position=tuple(data["position"]))


@dataclass
class TagState:
    """Current state of the UWB tag on the sailboat."""

    id: str
    true_position: Vector3
    velocity: Vector3


@dataclass
class RangeMeasurement:
    """A single DS-TWR range measurement from anchor to tag."""

    anchor_id: str
    tag_id: str
    tof_ns: float
    distance_m: float
    variance_m2: float
    round_trip_ns: float
    reply_time_ns: float


@dataclass
class Frame:
    """A complete data frame with all anchors, tag state, and measurements."""

    frame_id: int
    timestamp_ns: int
    anchors: Sequence[Anchor]
    tag: TagState
    measurements: Sequence[RangeMeasurement]
    line_crossing: dict | None

    def to_payload(self) -> dict:
        return {
            "frame_id": self.frame_id,
            "timestamp_ns": self.timestamp_ns,
            "anchors": [
                {"id": a.id, "position": list(a.position)} for a in self.anchors
            ],
            "tag": {
                "id": self.tag.id,
                "true_position": list(self.tag.true_position),
                "velocity": list(self.tag.velocity),
            },
            "measurements": [
                {
                    "anchor_id": m.anchor_id,
                    "tag_id": m.tag_id,
                    "tof_ns": m.tof_ns,
                    "distance_m": m.distance_m,
                    "variance_m2": m.variance_m2,
                    "round_trip_ns": m.round_trip_ns,
                    "reply_time_ns": m.reply_time_ns,
                }
                for m in self.measurements
            ],
            "line_crossing": self.line_crossing,
        }


def anchors_from_config(config: Iterable[dict]) -> Tuple[Anchor, ...]:
    return tuple(Anchor.from_dict(item) for item in config)





