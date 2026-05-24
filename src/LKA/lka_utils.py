"""
lka_utils.py
RMSE tracker, DNF watchdog, and CSV data logger.
Keeps LKA_Main.py clean of bookkeeping logic.
"""
import math
import csv
import os
import time
from collections import deque
import config as cfg

# File lives at: src/LKA/lka_utils.py  →  results/LKA/ is two levels up
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_RESULTS_LKA_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "..", "results", "LKA"))


class RMSETracker:
    """Rolling-window RMSE over the last N lateral-deviation samples."""

    def __init__(self, window: int = cfg.RMSE_WINDOW):
        self._buf = deque(maxlen=window)

    def update(self, lateral_error_m: float):
        self._buf.append(lateral_error_m ** 2)

    @property
    def rmse(self) -> float:
        if not self._buf:
            return 0.0
        return math.sqrt(sum(self._buf) / len(self._buf))

    @property
    def ilc(self) -> float:
        """In-Lane Coverage: fraction of samples within ±LANE_WIDTH_M."""
        if not self._buf:
            return 1.0
        in_lane = sum(1 for e2 in self._buf if math.sqrt(e2) <= (cfg.LANE_WIDTH_M / 2.0))
        return in_lane / len(self._buf)


class DNFWatchdog:
    """Raises DNFException if car stays out of lane for > DNF_TIME_S seconds."""

    class DNFException(RuntimeError):
        pass

    def __init__(self, limit_s: float = cfg.DNF_TIME_S):
        self._limit   = limit_s
        self._out_since: float | None = None

    def update(self, lateral_error_m: float):
        out_of_lane = abs(lateral_error_m) > cfg.DNF_THRESHOLD
        if out_of_lane:
            if self._out_since is None:
                self._out_since = time.monotonic()
            elif time.monotonic() - self._out_since >= self._limit:
                raise DNFWatchdog.DNFException(
                    f"DNF: car out of lane for {self._limit} s")
        else:
            self._out_since = None

    @property
    def out_lane_seconds(self) -> float:
        if self._out_since is None:
            return 0.0
        return time.monotonic() - self._out_since


class DataLogger:
    """Writes per-tick telemetry to a CSV for post-run analysis."""

    def __init__(self, path: str = None):
        if path is None:
            path = os.path.join(_RESULTS_LKA_DIR, "Data", "Logs", "lka_run.csv")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow([
            "t_s", "lat_error_m", "steer_cmd_deg",
            "rmse_m", "ilc", "lane_detected"
        ])

    def log(self, t_s, lat_error, steer_cmd,
            rmse, ilc, lane_detected):
        self._w.writerow([
            f"{t_s:.3f}", f"{lat_error:.4f}", f"{steer_cmd:.3f}",
            f"{rmse:.4f}", f"{ilc:.4f}", int(lane_detected)
        ])

    def close(self):
        self._f.close()