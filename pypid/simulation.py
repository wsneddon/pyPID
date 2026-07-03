"""First Order Plus Dead Time (FOPDT) simulation module.

Provides a decoupled process simulator for testing PID controllers.
The transfer function modeled is:

    G(s) = K * exp(-theta * s) / (tau * s + 1)

Where:
    K     = process gain
    tau   = time constant
    theta = dead time (transport delay)
"""

import time
from collections import deque


class FOPDTSimulator:
    """First Order Plus Dead Time process simulator.

    Parameters
    ----------
    K : float
        Process gain (steady-state output change / input change).
    tau : float
        Time constant in seconds.
    theta : float
        Dead time (transport delay) in seconds.
    y0 : float
        Initial process output value.
    time_fn : callable or None
        Function returning current time in seconds. If None, uses time.monotonic.
        For simulation loops with explicit dt, this is not used.
    """

    def __init__(self, K=1.0, tau=10.0, theta=0.0, y0=0.0, time_fn=None):
        if tau <= 0:
            raise ValueError(f"tau must be positive, got {tau}")
        if theta < 0:
            raise ValueError(f"theta must be non-negative, got {theta}")

        self.K = K
        self.tau = tau
        self.theta = theta
        self.y = y0
        self._y0 = y0

        if time_fn is not None:
            self.time_fn = time_fn
        else:
            self.time_fn = time.monotonic

        # Dead time buffer: stores (time_available, input_value) tuples
        self._delay_buffer = deque()
        # Initialize current input to maintain steady state: K * u = y0
        self._current_input = y0 / K if K != 0 else 0.0
        self._last_time = None
        self._total_time = 0.0

    def update(self, control_input, dt=None):
        """Advance the simulation by one time step.

        Parameters
        ----------
        control_input : float
            The controller output (manipulated variable) feeding into the process.
        dt : float or None
            Time step in seconds. If None, uses real elapsed time since last call.

        Returns
        -------
        float
            The current process variable (simulated measurement).
        """
        if dt is None:
            now = self.time_fn()
            if self._last_time is None:
                dt = 0.0
            else:
                dt = now - self._last_time
            self._last_time = now
        elif dt < 0:
            raise ValueError(f"dt must be non-negative, got {dt}")

        self._total_time += dt

        # Add new input to the delay buffer with its availability time
        available_time = self._total_time + self.theta
        self._delay_buffer.append((available_time, control_input))

        # Process delayed inputs — use the most recent input that has cleared the dead time
        while self._delay_buffer and self._delay_buffer[0][0] <= self._total_time:
            _, self._current_input = self._delay_buffer.popleft()

        # First-order lag: dy/dt = (K * u - y) / tau
        if dt > 0:
            self.y += (self.K * self._current_input - self.y) * dt / self.tau

        return self.y

    def reset(self, y0=None):
        """Reset the simulator to initial conditions.

        Parameters
        ----------
        y0 : float or None
            New initial output. If None, uses the original y0.
        """
        if y0 is not None:
            self.y = y0
            self._y0 = y0
        else:
            self.y = self._y0
        self._delay_buffer.clear()
        self._current_input = self._y0 / self.K if self.K != 0 else 0.0
        self._last_time = None
        self._total_time = 0.0

    @property
    def steady_state(self):
        """The steady-state output for the current input: K * u."""
        return self.K * self._current_input

    def __repr__(self):
        return (
            f"FOPDTSimulator(K={self.K}, tau={self.tau}, theta={self.theta}, "
            f"y={self.y:.4f})"
        )
