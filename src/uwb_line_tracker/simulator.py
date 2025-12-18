"""
DS-TWR signal simulator that mimics a pair of anchors and a sailing tag.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import math
import time
from collections import deque
from typing import Deque, Iterable, List, Sequence

from .config import ANCHORS, NETWORK, SIMULATION
from .ds_twr import detect_line_crossing, simulate_ds_twr
from .models import Anchor, Frame, TagState, anchors_from_config


class RegattaSimulator:
    """Smooth motion model: vertical crossing + semicircle return pattern."""

    def __init__(self, anchors: Sequence[Anchor], settings: dict):
        self.anchors = tuple(anchors)
        if len(self.anchors) < 2:
            raise ValueError("At least two anchors are required")
        self.settings = settings
        self.frame_id = 0
        self._start_time = time.perf_counter()
        self._last_tick = self._start_time
        self._elapsed_s = 0.0
        self._history: Deque[Frame] = deque(maxlen=settings["history_limit"])
        self._line_anchor_a = self.anchors[0]
        self._line_anchor_b = self.anchors[1]

        # Calculate line midpoint and perpendicular direction
        ax, ay, _ = self._line_anchor_a.position
        bx, by, _ = self._line_anchor_b.position
        self._line_midpoint = ((ax + bx) / 2, (ay + by) / 2, 0.0)
        line_dx = bx - ax
        line_dy = by - ay
        line_length = math.hypot(line_dx, line_dy)
        # Perpendicular vector (pointing "up" from the line)
        self._perpendicular = (-line_dy / line_length, line_dx / line_length, 0.0)
        # Left direction (perpendicular to line, pointing left when facing forward)
        self._left_direction = (-line_dy / line_length, line_dx / line_length, 0.0)

        # Initialize state machine
        self._phase = "approach"  # "approach", "crossing", "post_crossing", "semicircle"
        self._phase_start_time = 0.0
        self._phase_start_position = None
        self._semicircle_center = None
        self._semicircle_start_pos = None

        # Start position: approach_distance_m below the line, centered
        start_y_offset = -settings["approach_distance_m"]
        start_position = (
            self._line_midpoint[0] + start_y_offset * self._perpendicular[0],
            self._line_midpoint[1] + start_y_offset * self._perpendicular[1],
            0.0,
        )
        self._tag_state = TagState(
            id=settings["tag_id"],
            true_position=start_position,
            velocity=(0.0, 0.0, 0.0),
        )
        self._previous_position = start_position

    @property
    def history(self) -> Sequence[Frame]:
        return tuple(self._history)

    def _update_tag_state(self, dt: float) -> None:
        settings = self.settings
        current_pos = self._tag_state.true_position
        phase_elapsed = self._elapsed_s - self._phase_start_time

        # Calculate signed distance to line (positive = above line)
        signed_dist = (
            (current_pos[0] - self._line_midpoint[0]) * self._perpendicular[0]
            + (current_pos[1] - self._line_midpoint[1]) * self._perpendicular[1]
        )

        if self._phase == "approach":
            # Move vertically toward the line
            speed = settings["approach_speed_m_s"]
            dy = speed * dt
            new_x = current_pos[0] + dy * self._perpendicular[0]
            new_y = current_pos[1] + dy * self._perpendicular[1]
            new_z = current_pos[2]

            # Check if we've reached the line
            new_signed_dist = (
                (new_x - self._line_midpoint[0]) * self._perpendicular[0]
                + (new_y - self._line_midpoint[1]) * self._perpendicular[1]
            )
            if new_signed_dist >= 0:
                self._phase = "crossing"
                self._phase_start_time = self._elapsed_s
                self._phase_start_position = (new_x, new_y, new_z)

        elif self._phase == "crossing":
            # Continue moving forward after crossing, up to the intersection with anchor line
            speed = settings["approach_speed_m_s"]
            dy = speed * dt
            new_x = current_pos[0] + dy * self._perpendicular[0]
            new_y = current_pos[1] + dy * self._perpendicular[1]
            new_z = current_pos[2]

            # Check if we've reached the intersection with anchor line
            # The intersection is where the vertical start line meets the horizontal anchor line
            # Since start line is vertical and anchor line is horizontal, intersection is at anchor line height
            target_distance = settings.get("crossing_to_anchor_line_m", 12.5)
            if self._phase_start_position:
                dist_traveled = math.hypot(
                    new_x - self._phase_start_position[0],
                    new_y - self._phase_start_position[1],
                )
                if dist_traveled >= target_distance:
                    # Store the actual position when entering semicircle phase (at anchor line intersection)
                    self._semicircle_start_pos = (new_x, new_y, new_z)
                    self._phase = "semicircle"
                    self._phase_start_time = self._elapsed_s
                    self._semicircle_center = None  # Will be calculated on first semicircle update

        elif self._phase == "semicircle":
            # Move in a semicircle from anchor line intersection back to approach start
            # The semicircle is centered near Anchor A, goes left and around, then back to start line below
            if self._semicircle_start_pos is None:
                # Fallback: use current position if start pos not set
                new_x, new_y, new_z = current_pos
            else:
                radius = settings["semicircle_radius_m"]
                speed = settings["semicircle_speed_m_s"]
                
                # Calculate start position (at anchor line intersection, above start line)
                semicircle_start_x, semicircle_start_y, _ = self._semicircle_start_pos
                
                # End position (at approach start, below start line)
                start_y_offset = -settings["approach_distance_m"]
                semicircle_end_x = (
                    self._line_midpoint[0] + start_y_offset * self._perpendicular[0]
                )
                semicircle_end_y = (
                    self._line_midpoint[1] + start_y_offset * self._perpendicular[1]
                )
                
                # Initialize semicircle center if needed
                # Center should be near Anchor A, offset to the left
                if self._semicircle_center is None:
                    # Use Anchor A as reference point
                    anchor_a = self._line_anchor_a
                    ax, ay, _ = anchor_a.position
                    
                    # Center is at Anchor A, offset to the left by some distance
                    # This ensures the arc goes around the start line from the left
                    left_dir_x, left_dir_y, _ = self._left_direction
                    center_offset = radius * 0.3  # Offset to position center for proper arc
                    self._semicircle_center = (
                        ax + center_offset * left_dir_x,
                        ay + center_offset * left_dir_y,
                        0.0
                    )
                
                # Calculate angles and interpolate
                cx, cy, _ = self._semicircle_center
                start_angle = math.atan2(semicircle_start_y - cy, semicircle_start_x - cx)
                end_angle = math.atan2(semicircle_end_y - cy, semicircle_end_x - cx)
                
                # Calculate angular distance - we want to go the longer path (semicircle)
                # Start is above line (positive y relative to center), end is below (negative y)
                # We want to go left (counterclockwise from start to end)
                angle_diff = (end_angle - start_angle) % (2 * math.pi)
                
                # If the direct path is less than 180°, we need to go the long way (semicircle)
                if angle_diff < math.pi:
                    angle_diff = angle_diff - 2 * math.pi  # Go the other way (negative, counterclockwise)
                
                # Calculate progress based on distance traveled
                total_arc_length = radius * abs(angle_diff)
                if total_arc_length > 0:
                    distance_traveled = speed * phase_elapsed
                    progress = min(distance_traveled / total_arc_length, 1.0)
                else:
                    progress = 1.0
                
                if progress >= 1.0:
                    # Completed semicircle, reset to approach phase
                    self._phase = "approach"
                    self._phase_start_time = self._elapsed_s
                    new_x = semicircle_end_x
                    new_y = semicircle_end_y
                    new_z = current_pos[2]
                    self._semicircle_center = None
                    self._semicircle_start_pos = None
                else:
                    # Interpolate angle (going counterclockwise, so subtract)
                    current_angle = start_angle + progress * angle_diff
                    new_x = cx + radius * math.cos(current_angle)
                    new_y = cy + radius * math.sin(current_angle)
                    new_z = current_pos[2]

        new_position = (new_x, new_y, new_z)
        vx = (new_position[0] - current_pos[0]) / max(dt, 1e-3)
        vy = (new_position[1] - current_pos[1]) / max(dt, 1e-3)
        self._tag_state = TagState(
            id=self._tag_state.id,
            true_position=new_position,
            velocity=(vx, vy, 0.0),
        )

    def generate_frame(self) -> Frame:
        now = time.perf_counter()
        dt = now - self._last_tick
        self._last_tick = now
        self._elapsed_s += dt

        self._update_tag_state(dt)

        measurements = tuple(
            simulate_ds_twr(
                anchor,
                self._tag_state,
                noise_std_m=self.settings["distance_noise_std_m"],
                responder_delay_ns=self.settings["responder_processing_ns"],
                clock_drift_ns=self.settings["clock_drift_ns"],
                rng=None,  # No random noise in trajectory
            )
            for anchor in self.anchors
        )

        line_crossing = None
        if detect_line_crossing(
            self._line_anchor_a,
            self._line_anchor_b,
            self._previous_position,
            self._tag_state.true_position,
        ):
            line_crossing = {
                "timestamp_ns": time.time_ns(),
                "tag_position": list(self._tag_state.true_position),
            }

        frame = Frame(
            frame_id=self.frame_id,
            timestamp_ns=time.time_ns(),
            anchors=self.anchors,
            tag=self._tag_state,
            measurements=measurements,
            line_crossing=line_crossing,
        )
        self._previous_position = self._tag_state.true_position
        self._history.append(frame)
        self.frame_id += 1
        return frame


class SimulatorServer:
    """TCP JSON line broadcaster for simulator frames."""

    def __init__(
        self,
        simulator: RegattaSimulator,
        *,
        host: str,
        port: int,
        update_rate_hz: int,
    ):
        self.simulator = simulator
        self.host = host
        self.port = port
        self.update_rate_hz = update_rate_hz
        self._clients: List[asyncio.StreamWriter] = []
        self._server: asyncio.AbstractServer | None = None

    async def start(self) -> None:
        self._server = await asyncio.start_server(
            self._handle_client, self.host, self.port
        )
        addr = ", ".join(str(sock.getsockname()) for sock in self._server.sockets)
        print(f"[simulator] Serving DS-TWR frames on {addr}")
        broadcaster = asyncio.create_task(self._broadcast_loop())
        async with self._server:
            try:
                await self._server.serve_forever()
            finally:
                broadcaster.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await broadcaster

    async def _handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer = writer.get_extra_info("peername")
        print(f"[simulator] Client connected: {peer}")
        self._clients.append(writer)
        try:
            while not reader.at_eof():
                await reader.readline()
        except asyncio.CancelledError:  # pragma: no cover - defensive
            raise
        finally:
            print(f"[simulator] Client disconnected: {peer}")
            self._clients.remove(writer)
            writer.close()
            await writer.wait_closed()

    async def _broadcast_loop(self) -> None:
        interval = 1.0 / self.update_rate_hz
        try:
            while True:
                frame = self.simulator.generate_frame()
                payload = json.dumps(frame.to_payload()) + "\n"
                encoded = payload.encode("utf-8")
                stale_clients: List[asyncio.StreamWriter] = []
                for writer in self._clients:
                    writer.write(encoded)
                for writer in self._clients:
                    try:
                        await writer.drain()
                    except ConnectionResetError:
                        stale_clients.append(writer)
                for writer in stale_clients:
                    self._clients.remove(writer)
                await asyncio.sleep(interval)
        except asyncio.CancelledError:  # pragma: no cover
            pass


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the DS-TWR sailing start-line simulator."
    )
    parser.add_argument("--host", default=NETWORK["host"], help="TCP host to bind")
    parser.add_argument(
        "--port", type=int, default=NETWORK["port"], help="TCP port to bind"
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=SIMULATION["update_rate_hz"],
        help="Frame update rate (Hz)",
    )
    return parser


async def async_main(host: str, port: int, rate: int) -> None:
    anchors = anchors_from_config(ANCHORS)
    simulator = RegattaSimulator(anchors, SIMULATION)
    server = SimulatorServer(
        simulator,
        host=host,
        port=port,
        update_rate_hz=rate,
    )
    await server.start()


def main() -> None:
    args = build_arg_parser().parse_args()
    try:
        asyncio.run(async_main(args.host, args.port, args.rate))
    except KeyboardInterrupt:
        print("\n[simulator] Stopped.")


if __name__ == "__main__":
    main()



