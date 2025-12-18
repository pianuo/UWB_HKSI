"""
Tkinter-based visual terminal for the UWB start-line simulator.
"""

from __future__ import annotations

import json
import math
import os
import queue
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Deque, List, Tuple

from .config import ANCHORS, NETWORK, SIMULATION
from .ds_twr import signed_distance_to_line
from .history import DEFAULT_HISTORY_PATH, HistoryStore
from .models import Anchor, anchors_from_config


class SimulatorController:
    """Start/stop the simulator in a background process."""

    def __init__(self) -> None:
        self._process: subprocess.Popen[str] | None = None

    def start(self) -> None:
        if self.is_running:
            return
        env = os.environ.copy()
        package_root = Path(__file__).resolve().parents[1]
        python_path = env.get("PYTHONPATH", "")
        if str(package_root) not in python_path.split(os.pathsep):
            env["PYTHONPATH"] = (
                f"{package_root}{os.pathsep}{python_path}" if python_path else str(package_root)
            )
        self._process = subprocess.Popen(
            [sys.executable, "-m", "uwb_line_tracker.simulator"],
            env=env,
        )

    def stop(self) -> None:
        if not self.is_running:
            return
        assert self._process is not None
        self._process.terminate()
        try:
            self._process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self._process.kill()
        finally:
            self._process = None

    @property
    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None


class FrameReceiver(threading.Thread):
    """Background thread that pulls simulator frames over TCP."""

    def __init__(
        self,
        host: str,
        port: int,
        frame_queue: queue.Queue,
        status_queue: queue.Queue,
    ):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.frame_queue = frame_queue
        self.status_queue = status_queue
        self._stop_event = threading.Event()
        self._socket: socket.socket | None = None

    def stop(self) -> None:
        self._stop_event.set()
        if self._socket:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._socket.close()
            except OSError:
                pass

    def _emit_status(self, message: str) -> None:
        try:
            self.status_queue.put_nowait(message)
        except queue.Full:
            pass

    def run(self) -> None:  # noqa: D401 - threading loop
        while not self._stop_event.is_set():
            try:
                self._emit_status("Connecting …")
                sock = socket.create_connection((self.host, self.port), timeout=5)
                sock.settimeout(1.0)
                self._socket = sock
                self._emit_status("Connected")
                with sock.makefile("r") as stream:
                    while not self._stop_event.is_set():
                        line = stream.readline()
                        if not line:
                            raise ConnectionError("Simulator closed the socket")
                        frame = json.loads(line)
                        try:
                            self.frame_queue.put(frame, timeout=0.1)
                        except queue.Full:
                            pass
            except (OSError, ConnectionError, json.JSONDecodeError) as exc:
                if self._stop_event.is_set():
                    break
                self._emit_status(f"Disconnected: {exc}")
                time.sleep(1.0)
            finally:
                if self._socket:
                    try:
                        self._socket.close()
                    except OSError:
                        pass
                    self._socket = None
        self._emit_status("Stopped")


class VisualizerApp:
    """Tkinter UI to visualise anchors, tag trajectory, and history."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("UWB Start-Line Visualizer")
        self.root.geometry("1024x720")

        self.anchors = anchors_from_config(ANCHORS)
        self.anchor_a = self.anchors[0]
        self.anchor_b = self.anchors[1]
        self.history_store = HistoryStore(DEFAULT_HISTORY_PATH)
        self.sim_controller = SimulatorController()

        self.frame_queue: queue.Queue = queue.Queue(maxsize=200)
        self.status_queue: queue.Queue = queue.Queue(maxsize=20)
        self.receiver = FrameReceiver(
            NETWORK["host"], NETWORK["port"], self.frame_queue, self.status_queue
        )
        self.receiver.start()

        self.canvas = tk.Canvas(self.root, bg="#0f172a", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=(10, 0))

        controls = ttk.Frame(self.root)
        controls.pack(fill=tk.X, padx=10, pady=10)

        self.start_button = ttk.Button(controls, text="Start Simulator", command=self.start_simulator)
        self.start_button.pack(side=tk.LEFT, padx=(0, 5))
        self.stop_button = ttk.Button(
            controls, text="Stop Simulator", command=self.stop_simulator, state=tk.DISABLED
        )
        self.stop_button.pack(side=tk.LEFT, padx=(0, 5))
        self.history_button = ttk.Button(
            controls, text="View History", command=self.show_history_window
        )
        self.history_button.pack(side=tk.LEFT, padx=(0, 5))

        self.status_var = tk.StringVar(value="Waiting for connection...")
        ttk.Label(controls, textvariable=self.status_var).pack(side=tk.RIGHT)

        info_frame = ttk.Frame(self.root)
        info_frame.pack(fill=tk.X, padx=10, pady=(0, 10))

        self.tag_var = tk.StringVar(value="(Waiting for data)")
        self.speed_var = tk.StringVar(value="0.00 m/s")
        self.distance_var = tk.StringVar(value="0.00 m")
        self.crossing_var = tk.StringVar(value="No crossing yet")

        self._add_info_row(info_frame, "Tag Position", self.tag_var, 0)
        self._add_info_row(info_frame, "Instantaneous Speed", self.speed_var, 1)
        self._add_info_row(info_frame, "Distance to Start Line", self.distance_var, 2)
        self._add_info_row(info_frame, "Last Crossing Time", self.crossing_var, 3)

        self.path_points: Deque[Tuple[float, float]] = deque(maxlen=2000)
        self.last_crossing_ns: int | None = None
        self.last_frame_ts: float | None = None

        xs = [anchor.position[0] for anchor in self.anchors]
        ys = [anchor.position[1] for anchor in self.anchors]
        # Expand bounds based on simulation parameters
        approach_dist = SIMULATION.get("approach_distance_m", 15.0)
        post_crossing_dist = SIMULATION.get("post_crossing_distance_m", 8.0)
        semicircle_radius = SIMULATION.get("semicircle_radius_m", 12.0)
        xs.extend([min(xs) - semicircle_radius, max(xs) + semicircle_radius])
        ys.extend([min(ys) - approach_dist - semicircle_radius, max(ys) + post_crossing_dist + semicircle_radius])
        self.world_bounds = {
            "min_x": min(xs) - 5.0,
            "max_x": max(xs) + 5.0,
            "min_y": min(ys) - 5.0,
            "max_y": max(ys) + 5.0,
        }

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self._poll_queues)

    def _add_info_row(self, parent: ttk.Frame, label: str, var: tk.StringVar, row: int) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, padx=(0, 8), pady=2)
        ttk.Label(parent, textvariable=var, font=("Consolas", 11, "bold")).grid(
            row=row, column=1, sticky=tk.W, pady=2
        )

    def start_simulator(self) -> None:
        try:
            self.sim_controller.start()
            self.status_var.set("Simulator started")
            self.start_button.configure(state=tk.DISABLED)
            self.stop_button.configure(state=tk.NORMAL)
        except Exception as exc:  # pragma: no cover - Tk only
            messagebox.showerror("Start Failed", str(exc))

    def stop_simulator(self) -> None:
        try:
            self.sim_controller.stop()
            self.status_var.set("Simulator stopped")
            self.start_button.configure(state=tk.NORMAL)
            self.stop_button.configure(state=tk.DISABLED)
        except Exception as exc:  # pragma: no cover
            messagebox.showerror("Stop Failed", str(exc))

    def _poll_queues(self) -> None:
        while not self.status_queue.empty():
            status = self.status_queue.get_nowait()
            self.status_var.set(status)

        frame_processed = False
        while not self.frame_queue.empty():
            frame = self.frame_queue.get_nowait()
            self._handle_frame(frame)
            frame_processed = True

        if frame_processed:
            self._redraw_canvas()

        self.root.after(50, self._poll_queues)

    def _handle_frame(self, frame: dict) -> None:
        tag_payload = frame["tag"]
        position = tuple(tag_payload["true_position"])
        velocity = tuple(tag_payload.get("velocity", (0.0, 0.0, 0.0)))
        timestamp_ns = frame["timestamp_ns"]
        speed = math.hypot(velocity[0], velocity[1])

        self.tag_var.set(f"({position[0]:7.2f}, {position[1]:7.2f}) m")
        self.speed_var.set(f"{speed:5.2f} m/s")

        signed_dist = signed_distance_to_line(
            self.anchor_a, self.anchor_b, position  # type: ignore[arg-type]
        )
        self.distance_var.set(f"{signed_dist:+6.2f} m")

        if frame.get("line_crossing"):
            self.last_crossing_ns = frame["line_crossing"]["timestamp_ns"]

        if self.last_crossing_ns:
            ts = datetime.fromtimestamp(self.last_crossing_ns / 1e9, tz=timezone.utc)
            self.crossing_var.set(ts.astimezone().strftime("%Y-%m-%d %H:%M:%S"))

        self.path_points.append((position[0], position[1]))
        self._expand_bounds(position[0], position[1])

        self.history_store.append(
            {
                "frame_id": frame["frame_id"],
                "timestamp_ns": timestamp_ns,
                "position_xy": [position[0], position[1]],
                "distance_to_line_m": signed_dist,
                "line_crossing": frame.get("line_crossing"),
            }
        )

    def _expand_bounds(self, x: float, y: float) -> None:
        margin = 2.0
        self.world_bounds["min_x"] = min(self.world_bounds["min_x"], x - margin)
        self.world_bounds["max_x"] = max(self.world_bounds["max_x"], x + margin)
        self.world_bounds["min_y"] = min(self.world_bounds["min_y"], y - margin)
        self.world_bounds["max_y"] = max(self.world_bounds["max_y"], y + margin)

    def _world_to_canvas(self, x: float, y: float) -> Tuple[float, float]:
        width = self.canvas.winfo_width() or 800
        height = self.canvas.winfo_height() or 500
        padding = 40
        min_x = self.world_bounds["min_x"]
        min_y = self.world_bounds["min_y"]
        max_x = self.world_bounds["max_x"]
        max_y = self.world_bounds["max_y"]
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        scale = min((width - 2 * padding) / span_x, (height - 2 * padding) / span_y)
        px = padding + (x - min_x) * scale
        py = height - (padding + (y - min_y) * scale)
        return px, py

    def _redraw_canvas(self) -> None:
        self.canvas.delete("all")
        self.canvas.configure(bg="#0f172a")
        width = self.canvas.winfo_width() or 800
        height = self.canvas.winfo_height() or 500
        self.canvas.create_rectangle(0, 0, width, height, fill="#0f172a", outline="")

        ax, ay, _ = self.anchor_a.position
        bx, by, _ = self.anchor_b.position
        axp, ayp = self._world_to_canvas(ax, ay)
        bxp, byp = self._world_to_canvas(bx, by)
        self.canvas.create_line(
            axp,
            ayp,
            bxp,
            byp,
            fill="#38bdf8",
            width=3,
        )
        self.canvas.create_text(
            (axp + bxp) / 2,
            (ayp + byp) / 2 - 14,
            text="Virtual Start Line",
            fill="#e2e8f0",
            font=("Arial", 12, "bold"),
        )

        path_coords: List[float] = []
        for x, y in self.path_points:
            px, py = self._world_to_canvas(x, y)
            path_coords.extend([px, py])
        if len(path_coords) >= 4:
            self.canvas.create_line(
                *path_coords,
                fill="#a855f7",
                width=2,
                smooth=True,
            )

        for anchor in self.anchors:
            px, py = self._world_to_canvas(anchor.position[0], anchor.position[1])
            self.canvas.create_oval(
                px - 6,
                py - 6,
                px + 6,
                py + 6,
                fill="#22d3ee",
                outline="white",
                width=1,
            )
            self.canvas.create_text(
                px + 12,
                py - 12,
                text=f"Anchor {anchor.id}",
                fill="#e2e8f0",
                anchor=tk.NW,
            )

        if self.path_points:
            x, y = self.path_points[-1]
            px, py = self._world_to_canvas(x, y)
            self.canvas.create_oval(
                px - 10,
                py - 10,
                px + 10,
                py + 10,
                fill="#f97316",
                outline="white",
                width=2,
            )
            self.canvas.create_text(
                px + 12,
                py,
                text="Tag",
                fill="#f1f5f9",
                anchor=tk.W,
                font=("Microsoft Yahei", 11, "bold"),
            )

    def show_history_window(self) -> None:
        records = self.history_store.tail(200)
        if not records:
            messagebox.showinfo("History", "No history records available.")
            return

        window = tk.Toplevel(self.root)
        window.title("History Trajectory & Crossing Times")
        text = tk.Text(window, width=80, height=25, bg="#0f172a", fg="#e2e8f0")
        text.pack(fill=tk.BOTH, expand=True)

        for record in records:
            ts = datetime.fromtimestamp(record["timestamp_ns"] / 1e9, tz=timezone.utc)
            coords = record.get("position_xy") or record.get("estimate_xy") or (math.nan, math.nan)
            dist = record.get("distance_to_line_m")
            label = " crossing" if record.get("line_crossing") else ""
            text.insert(
                tk.END,
                f"{ts.astimezone().strftime('%Y-%m-%d %H:%M:%S')} | "
                f"({coords[0]:7.2f}, {coords[1]:7.2f}) m | "
                f"d={dist if dist is not None else 'n/a'} m{label}\n",
            )
        text.configure(state=tk.DISABLED)

    def on_close(self) -> None:
        self.receiver.stop()
        self.sim_controller.stop()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    app = VisualizerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()



