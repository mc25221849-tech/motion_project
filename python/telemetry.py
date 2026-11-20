"""통신 스레드를 디스크/콘솔 I/O에서 분리하고 지연 통계를 누적한다."""

from collections import Counter, deque
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import time


class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self._data = {"port_open": False, "connected": False, "error": "Waiting"}

    def update(self, **values):
        with self._lock:
            self._data.update(values)

    def snapshot(self):
        with self._lock:
            return self._data.copy()


@dataclass
class Series:
    count: int = 0
    mean: float = 0.0
    m2: float = 0.0
    maximum: float = -math.inf
    minimum: float = math.inf
    recent: deque = field(default_factory=lambda: deque(maxlen=4096))

    def add(self, value):
        self.count += 1
        delta = value - self.mean
        self.mean += delta / self.count
        self.m2 += delta * (value - self.mean)
        self.maximum = max(self.maximum, value)
        self.minimum = min(self.minimum, value)
        self.recent.append(value)

    def summary(self):
        ordered = sorted(self.recent)
        return {"count": self.count, "mean": self.mean, "min": self.minimum,
                "max": self.maximum, "stddev": math.sqrt(self.m2 / self.count),
                "recent_p95": ordered[math.ceil(len(ordered) * 0.95) - 1],
                "recent_count": len(ordered), "unit": "ms"}


class Metrics:
    def __init__(self):
        self._lock = threading.Lock()
        self._series = {}
        self._counters = Counter()

    def observe(self, name, milliseconds):
        with self._lock:
            self._series.setdefault(name, Series()).add(milliseconds)

    def count(self, name, amount=1):
        with self._lock:
            self._counters[name] += amount

    def summary(self):
        with self._lock:
            return {"timings": {k: v.summary() for k, v in self._series.items()},
                    "counters": dict(self._counters)}


class EventLog:
    """버퍼가 차면 로그만 버린다. 센서 최신값/모터 조회는 계속 수행."""
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self._file = (self.directory / "events.jsonl").open("w", encoding="utf-8")
        self._queue = Queue(maxsize=8192)
        self._stop = threading.Event()
        self.dropped = 0  # CPython에서도 복합 갱신이므로 lock으로 보호
        self.error = None
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="event-log", daemon=True)
        self._thread.start()

    def emit(self, kind, **fields):
        event = {"event": kind, "pc_monotonic_s": time.perf_counter(), **fields}
        try:
            self._queue.put_nowait(event)
        except Full:
            with self._lock:
                self.dropped += 1

    def _run(self):
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    event = self._queue.get(timeout=0.1)
                except Empty:
                    self._file.flush()
                    continue
                self._file.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
        except (OSError, ValueError) as exc:
            self.error = str(exc)
        finally:
            try:
                self._file.close()
            except OSError as exc:
                self.error = str(exc)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=3)
        if self._thread.is_alive():
            self.error = "Log writer did not finish; queued records may be lost"


def print_metrics(report):
    print("\nMeasured timings (ms; mean / max / stddev / recent p95):")
    for name, values in sorted(report["timings"].items()):
        print(f"  {name:28} n={values['count']:6d}  "
              f"{values['mean']:.3f} / {values['max']:.3f} / "
              f"{values['stddev']:.3f} / {values['recent_p95']:.3f}")
    print("Counters:", report["counters"])
