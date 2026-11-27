"""Queue capacity and overflow policies shared by the runtime and saving workers."""

import multiprocessing
import queue


DEFAULT_QUEUE_LIMITS = {
    "control": 32, "log": 128, "ui_status": 32, "pulse": 32, "preview": 2,
    "txt_frames": 32, "realtime_frames": 32, "raw_workpieces": 2, "processed_workpieces": 2,
    "save_tasks": 33, "save_acks": 33, "save_status": 64, "stop_results": 1,
}


class QueueLimits:
    def __init__(self, config):
        self._limits = {**DEFAULT_QUEUE_LIMITS, **config.get("queue_limits", {})}
        for name, limit in self._limits.items():
            if type(limit) is not int or limit <= 0:
                raise ValueError(f"queue_limits.{name} 必须为正整数")

    def create(self, name, *, minimum=1, drop_oldest=False, overflow=None, nonblocking=False, local=False):
        capacity = max(minimum, self._limits[name])
        if drop_oldest:
            return DropOldestQueue(capacity)
        if overflow is not None or nonblocking:
            return FaultOnFullQueue(capacity, name, overflow)
        if local:
            return queue.Queue(maxsize=capacity)
        return multiprocessing.Queue(maxsize=capacity)


class _BoundedQueue:
    def __init__(self, capacity):
        self.capacity = capacity
        self._queue = multiprocessing.Queue(maxsize=capacity)

    def get(self, *args, **kwargs):
        return self._queue.get(*args, **kwargs)

    def get_nowait(self):
        return self._queue.get_nowait()

    def empty(self):
        return self._queue.empty()

    def close(self):
        self._queue.close()

    def cancel_join_thread(self):
        self._queue.cancel_join_thread()


class FaultOnFullQueue(_BoundedQueue):
    def __init__(self, capacity, name, overflow):
        super().__init__(capacity)
        self._name = name
        self._overflow = overflow

    def put(self, item):
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            if self._overflow is not None:
                self._overflow.set()
                raise queue.Full(f"实时数据队列 {self._name} 已满，采集已暂停") from None
            raise queue.Full(f"队列 {self._name} 已满") from None

    def put_nowait(self, item):
        self.put(item)


class DropOldestQueue(_BoundedQueue):
    """For snapshots and developer replay packets that can be discarded."""

    def __init__(self, capacity=DEFAULT_QUEUE_LIMITS["txt_frames"]):
        super().__init__(capacity)
        self._lock = multiprocessing.Lock()

    def put(self, item):
        with self._lock:
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                # A packet counted against capacity may still be in the IPC feeder.
                try:
                    oldest = self._queue.get(timeout=0.05)
                except queue.Empty:
                    oldest = None
                try:
                    self._queue.put_nowait(item)
                except queue.Full:
                    return item
                return oldest
        return None

    def get_nowait(self):
        if not self._lock.acquire(False):
            raise queue.Empty
        try:
            return self._queue.get_nowait()
        finally:
            self._lock.release()
