"""
Terminal receiver that visualises DS-TWR simulator output.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
from datetime import datetime, timezone
from pathlib import Path

from .config import NETWORK
from .ds_twr import estimate_tag_position
from .history import DEFAULT_HISTORY_PATH, HistoryStore
from .models import Anchor, RangeMeasurement


def anchors_from_payload(payload: list) -> tuple[Anchor, ...]:
    from .models import Anchor

    anchors: list[Anchor] = []
    for item in payload:
        anchors.append(
            Anchor(
                id=item["id"],
                position=tuple(float(coord) for coord in item["position"]),
            )
        )
    return tuple(anchors)


def measurements_from_payload(payload: list) -> tuple[RangeMeasurement, ...]:
    measurements: list[RangeMeasurement] = []
    for item in payload:
        measurements.append(
            RangeMeasurement(
                anchor_id=item["anchor_id"],
                tag_id=item["tag_id"],
                tof_ns=float(item["tof_ns"]),
                distance_m=float(item["distance_m"]),
                variance_m2=float(item["variance_m2"]),
                round_trip_ns=float(item["round_trip_ns"]),
                reply_time_ns=float(item["reply_time_ns"]),
            )
        )
    return tuple(measurements)


class TerminalClient:
    def __init__(self, host: str, port: int, history_file: Path, max_frames: int | None = None):
        self.host = host
        self.port = port
        self.history = HistoryStore(history_file)
        self._frame_zero_ns: int | None = None
        self._anchors_printed = False
        self._max_frames = max_frames
        self._frames_seen = 0

    async def run(self) -> None:
        print(f"[terminal] Connecting to {self.host}:{self.port} ...")
        reader, writer = await asyncio.open_connection(self.host, self.port)
        print("[terminal] Connected. Listening for frames (press Ctrl+C to stop).")
        try:
            while True:
                line = await reader.readline()
                if not line:
                    raise ConnectionError("Simulator disconnected")
                frame = json.loads(line)
                self._handle_frame(frame)
                self._frames_seen += 1
                if self._max_frames and self._frames_seen >= self._max_frames:
                    print(f"[terminal] Reached frame limit {self._max_frames}, stopping.")
                    break
        finally:
            writer.close()
            await writer.wait_closed()

    def _handle_frame(self, frame: dict) -> None:
        anchors = anchors_from_payload(frame["anchors"])
        measurements = measurements_from_payload(frame["measurements"])

        if not self._anchors_printed:
            print("\nAnchors:")
            for anchor in anchors:
                print(
                    f"  - {anchor.id}: ({anchor.position[0]:.2f}, "
                    f"{anchor.position[1]:.2f}, {anchor.position[2]:.2f}) m"
                )
            print()
            self._anchors_printed = True

        try:
            est_x, est_y = estimate_tag_position(anchors, measurements)
        except ValueError:
            est_x, est_y = math.nan, math.nan

        timestamp_ns = int(frame["timestamp_ns"])
        if self._frame_zero_ns is None:
            self._frame_zero_ns = timestamp_ns
        relative_ms = (timestamp_ns - self._frame_zero_ns) / 1_000_000

        line_status = ""
        if frame.get("line_crossing"):
            ts = datetime.fromtimestamp(
                frame["line_crossing"]["timestamp_ns"] / 1e9, tz=timezone.utc
            )
            line_status = f" | line crossing @ {ts.isoformat()}"

        print(
            f"[{relative_ms:8.1f} ms] Frame {frame['frame_id']:05d} | "
            f"Tag ≈ ({est_x:7.3f}, {est_y:7.3f}) m{line_status}"
        )

        self.history.append(
            {
                "frame_id": frame["frame_id"],
                "timestamp_ns": timestamp_ns,
                "estimate_xy": [est_x, est_y],
                "line_crossing": frame.get("line_crossing"),
            }
        )


def replay_history(history_path: Path, limit: int | None = None) -> None:
    history = HistoryStore(history_path).load()
    if not history:
        print(f"[terminal] No history found at {history_path}")
        return

    limit = limit or len(history)
    print(f"[terminal] Replaying {min(limit, len(history))} / {len(history)} samples\n")
    for record in history[-limit:]:
        ts = datetime.fromtimestamp(record["timestamp_ns"] / 1e9, tz=timezone.utc)
        est_x, est_y = record["estimate_xy"]
        label = " crossing" if record.get("line_crossing") else ""
        print(f"{ts.isoformat()} | ({est_x:7.3f}, {est_y:7.3f}) m{label}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Display DS-TWR anchor/tag updates from the simulator."
    )
    parser.add_argument("--host", default=NETWORK["host"], help="Simulator host")
    parser.add_argument("--port", type=int, default=NETWORK["port"], help="Simulator port")
    parser.add_argument(
        "--history",
        type=Path,
        default=DEFAULT_HISTORY_PATH,
        help="History log file (JSONL)",
    )
    parser.add_argument(
        "--replay",
        action="store_true",
        help="Replay stored history instead of connecting to simulator",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Number of history samples to display when using --replay",
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=None,
        help="Maximum number of frames to process before stopping",
    )
    return parser


async def async_main(host: str, port: int, history_file: Path, max_frames: int | None) -> None:
    client = TerminalClient(host, port, history_file, max_frames)
    await client.run()


def main() -> None:
    args = build_arg_parser().parse_args()
    if args.replay:
        replay_history(args.history, args.limit)
        return

    try:
        asyncio.run(async_main(args.host, args.port, args.history, args.frames))
    except KeyboardInterrupt:
        print("\n[terminal] Interrupted by user.")
    except ConnectionError as exc:
        print(f"[terminal] Connection error: {exc}")


if __name__ == "__main__":
    main()





