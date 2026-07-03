"""Optional threaded scheduler for real-time PID execution."""

import threading
import time


class Scheduler:
    """Threaded scheduler that calls the PID controller at regular intervals.

    Parameters
    ----------
    pid : PID
        The PID controller instance to execute.
    get_pv : callable
        Function that returns the current process variable value.
        Called each cycle to provide the PV input to the PID.
    interval : float or None
        Execution interval in seconds. If None, uses pid.sample_time.
    on_output : callable or None
        Optional callback called with (output, pid) after each PID execution.
        Useful for sending output to actuators or logging.

    Example
    -------
    >>> from pypid import PID, Scheduler
    >>> pid = PID(Kp=1.0, Ki=0.1, Kd=0.0, setpoint=50.0, output_limits=(0, 100))
    >>> def read_sensor():
    ...     return get_temperature()  # your sensor reading function
    >>> def write_output(output, pid):
    ...     set_heater(output)  # your actuator function
    >>> sched = Scheduler(pid, get_pv=read_sensor, on_output=write_output)
    >>> sched.start()
    >>> # ... controller runs in background ...
    >>> sched.stop()
    """

    def __init__(self, pid, get_pv, interval=None, on_output=None):
        self.pid = pid
        self.get_pv = get_pv
        self.interval = interval if interval is not None else (pid.sample_time or 0.1)
        self.on_output = on_output

        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._running = False
        self._cycle_count = 0

    @property
    def running(self):
        """Whether the scheduler is currently running."""
        return self._running

    @property
    def cycle_count(self):
        """Number of completed execution cycles."""
        return self._cycle_count

    def start(self):
        """Start the scheduler thread."""
        if self._running:
            return

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        self._running = True

    def stop(self, timeout=5.0):
        """Stop the scheduler thread.

        Parameters
        ----------
        timeout : float
            Maximum time to wait for thread to finish.
        """
        if not self._running:
            return

        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
        self._running = False
        self._thread = None

    def _run_loop(self):
        """Internal execution loop."""
        while not self._stop_event.is_set():
            cycle_start = time.monotonic()

            try:
                pv = self.get_pv()

                with self._lock:
                    output = self.pid(pv)

                if self.on_output is not None:
                    self.on_output(output, self.pid)

                self._cycle_count += 1
            except Exception:
                # Don't crash the thread on errors; skip this cycle
                pass

            # Sleep for remainder of interval
            elapsed = time.monotonic() - cycle_start
            sleep_time = self.interval - elapsed
            if sleep_time > 0:
                self._stop_event.wait(timeout=sleep_time)

    def __enter__(self):
        """Context manager support."""
        self.start()
        return self

    def __exit__(self, *args):
        """Context manager support."""
        self.stop()

    def __repr__(self):
        status = "running" if self._running else "stopped"
        return (
            f"Scheduler(interval={self.interval}s, status={status}, "
            f"cycles={self._cycle_count})"
        )
