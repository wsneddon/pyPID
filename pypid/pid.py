"""Enhanced PID controller with operating modes, bias, reverse acting, and alarms.

Uses the classic coupled (ISA/standard) form:

    output = Kc * [error + (1/Ti) * ∫error*dt + Td * d(error)/dt]

Where Kc (controller gain) multiplies all three terms. Changing Kc scales
the entire controller response — matching how DCS/PLC controllers behave.

Based on simple-pid by Martin Lundberg (https://github.com/m-lundberg/simple-pid).
"""

import time
from enum import Enum

from .alarms import AlarmConfig, AlarmState, evaluate_alarms


class Mode(Enum):
    """PID controller operating modes."""

    MANUAL = "manual"
    AUTO = "auto"
    CASCADE = "cascade"


def _clamp(value, limits):
    """Clamp value between limits. Either limit can be None."""
    lower, upper = limits
    if value is None:
        return None
    if upper is not None and value > upper:
        return upper
    if lower is not None and value < lower:
        return lower
    return value


class PID:
    """Enhanced PID controller using the coupled (ISA/standard) form.

    The control equation is:

        output = Kc * [error + (1/Ti) * ∫error*dt + Td * d(error)/dt]

    Where:
        Kc = controller gain (dimensionless, affects all terms)
        Ti = integral time. Larger Ti = slower integral action.
             Ti=0 or None disables integral action.
        Td = derivative time. Larger Td = more derivative action.
             Td=0 or None disables derivative action.

    Ti and Td units depend on time_base ('minutes' or 'seconds').
    Default is 'minutes' to match DCS tuning conventions.

    Parameters
    ----------
    Kc : float
        Controller gain. Multiplies all three terms (P, I, D).
    Ti : float or None
        Integral time (minutes per repeat by default). None or 0 disables integral.
    Td : float or None
        Derivative time (minutes by default). None or 0 disables derivative.
    setpoint : float
        Target setpoint for Auto mode.
    time_base : str
        Time units for Ti and Td: 'minutes' (default) or 'seconds'.
        Note: dt (scan time) is always in seconds regardless of time_base.
    sample_time : float or None
        Minimum time between updates in seconds. None = update every call.
    output_limits : tuple of (float or None, float or None)
        (lower, upper) output clamping limits. Prevents integral windup.
    reverse_acting : bool
        If True, error = PV - SP (for cooling loops, etc.).
        If False, error = SP - PV (default, for heating loops).
    proportional_on_measurement : bool
        Calculate P term on measurement change rather than error.
    differential_on_measurement : bool
        Calculate D term on measurement change rather than error.
    time_fn : callable or None
        Function returning current time. Default: time.monotonic.
    starting_output : float
        Initial output / bias value.
    alarm_config : AlarmConfig or None
        Optional alarm configuration.
    """

    def __init__(
        self,
        Kc=1.0,
        Ti=None,
        Td=None,
        setpoint=0.0,
        time_base='minutes',
        sample_time=0.01,
        output_limits=(None, None),
        reverse_acting=False,
        proportional_on_measurement=False,
        differential_on_measurement=True,
        time_fn=None,
        starting_output=0.0,
        alarm_config=None,
    ):
        self.Kc = Kc
        self.Ti = Ti
        self.Td = Td
        self.setpoint = setpoint
        self.sample_time = sample_time
        self.reverse_acting = reverse_acting
        self.proportional_on_measurement = proportional_on_measurement
        self.differential_on_measurement = differential_on_measurement

        # Time base: convert Ti/Td to seconds internally
        if time_base not in ('minutes', 'seconds'):
            raise ValueError(f"time_base must be 'minutes' or 'seconds', got {time_base!r}")
        self.time_base = time_base
        self._time_factor = 60.0 if time_base == 'minutes' else 1.0

        # Alarms
        self.alarm_config = alarm_config
        self._alarm_state = AlarmState()

        # Mode
        self._mode = Mode.AUTO
        self._remote_setpoint = setpoint

        # Output limits
        self._min_output = None
        self._max_output = None

        # Internal state
        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._last_time = None
        self._last_output = None
        self._last_input = None
        self._last_error = None
        self._bias = starting_output
        self._output = starting_output

        # Time function
        if time_fn is not None:
            self.time_fn = time_fn
        else:
            self.time_fn = time.monotonic

        # Set output limits (also clamps integral)
        self.output_limits = output_limits

        # Initialize
        self.reset()
        self._integral = _clamp(starting_output, self.output_limits)
        self._output = starting_output
        self._bias = starting_output

    def __call__(self, input_, dt=None):
        """Update the PID controller.

        Parameters
        ----------
        input_ : float
            Current process variable in engineering units.
        dt : float or None
            Time step override. If None, uses real elapsed time.

        Returns
        -------
        float
            Controller output.
        """
        # PV is already in engineering units (scaling done externally)
        pv = input_

        # Evaluate alarms against PV
        active_sp = self._get_active_setpoint()
        self._alarm_state = evaluate_alarms(pv, active_sp, self.alarm_config)

        # Store PV for inspection
        self._pv = pv

        # In manual mode, just return the manually-set output
        if self._mode == Mode.MANUAL:
            self._last_input = pv
            self._last_time = self.time_fn()
            return self._output

        # Calculate dt
        now = self.time_fn()
        if dt is None:
            if self._last_time is None:
                dt = 0.0
            else:
                dt = now - self._last_time
                if dt <= 0:
                    dt = 1e-16
        elif dt <= 0:
            raise ValueError(f"dt must be positive, got {dt}")

        # Check sample time
        if (
            self.sample_time is not None
            and dt < self.sample_time
            and self._last_output is not None
        ):
            return self._last_output

        # Determine which setpoint to use
        sp = self._get_active_setpoint()

        # Compute error based on acting direction
        if self.reverse_acting:
            error = pv - sp
        else:
            error = sp - pv

        # Compute deltas
        d_input = pv - (self._last_input if self._last_input is not None else pv)
        d_error = error - (self._last_error if self._last_error is not None else error)

        # --- Coupled (ISA) form: output = Kc * [P + I + D] ---
        # dt is always in seconds; convert to time_base units for Ti/Td
        dt_base = dt / self._time_factor  # dt in minutes (or seconds if time_base='seconds')

        # Proportional contribution (before Kc multiplication)
        if not self.proportional_on_measurement:
            self._proportional = error
        else:
            self._proportional -= d_input

        # Integral contribution: (1/Ti) * ∫error*dt
        if self.Ti is not None and self.Ti > 0:
            self._integral += (1.0 / self.Ti) * error * dt_base
        # Clamp integral (in output units, so divide by Kc for clamping)
        # Actually clamp the full output, integral is clamped in output-equivalent units
        if self.Kc != 0:
            int_limits = (
                self._min_output / self.Kc if self._min_output is not None else None,
                self._max_output / self.Kc if self._max_output is not None else None,
            )
            # Handle negative Kc flipping the limits
            if self.Kc < 0:
                int_limits = (int_limits[1], int_limits[0])
            self._integral = _clamp(self._integral, int_limits)

        # Derivative contribution: Td * d(error)/dt
        if self.Td is not None and self.Td > 0:
            if self.differential_on_measurement:
                if dt_base > 0:
                    self._derivative = -self.Td * d_input / dt_base
                else:
                    self._derivative = 0.0
            else:
                if dt_base > 0:
                    self._derivative = self.Td * d_error / dt_base
                else:
                    self._derivative = 0.0
        else:
            self._derivative = 0.0

        # Final output: Kc multiplies all terms
        output = self.Kc * (self._proportional + self._integral + self._derivative)
        output = _clamp(output, self.output_limits)

        # Update state
        self._last_output = output
        self._last_input = pv
        self._last_error = error
        self._last_time = now
        self._output = output
        self._bias = output

        return output

    def _get_active_setpoint(self):
        """Return the active setpoint based on mode."""
        if self._mode == Mode.CASCADE:
            return self._remote_setpoint
        return self.setpoint

    # --- Mode management ---

    @property
    def mode(self):
        """Current operating mode (Mode.MANUAL, Mode.AUTO, or Mode.CASCADE)."""
        return self._mode

    @mode.setter
    def mode(self, new_mode):
        """Set operating mode with bumpless transfer."""
        if not isinstance(new_mode, Mode):
            raise ValueError(f"mode must be a Mode enum, got {new_mode!r}")

        old_mode = self._mode

        if old_mode == new_mode:
            return

        if old_mode == Mode.MANUAL and new_mode in (Mode.AUTO, Mode.CASCADE):
            # Transitioning from Manual to Auto/Cascade: bumpless transfer
            # Set integral so that Kc * integral = current output
            if self.Kc != 0:
                self._integral = self._output / self.Kc
            else:
                self._integral = 0.0
            self._bias = self._output
            self._proportional = 0.0
            self._derivative = 0.0
            self._last_error = None
            self._last_input = self._pv if hasattr(self, '_pv') else None
            self._last_time = self.time_fn()

        elif old_mode in (Mode.AUTO, Mode.CASCADE) and new_mode == Mode.MANUAL:
            # Transitioning to Manual: output holds at last computed value
            pass

        elif old_mode == Mode.AUTO and new_mode == Mode.CASCADE:
            # Auto to Cascade: update remote setpoint to current setpoint for bumpless
            self._remote_setpoint = self.setpoint

        elif old_mode == Mode.CASCADE and new_mode == Mode.AUTO:
            # Cascade to Auto: setpoint stays as is
            pass

        self._mode = new_mode

    # --- Output (writable in Manual mode) ---

    @property
    def output(self):
        """Current controller output."""
        return self._output

    @output.setter
    def output(self, value):
        """Set output manually (only allowed in MANUAL mode)."""
        if self._mode != Mode.MANUAL:
            raise RuntimeError("Output can only be written in MANUAL mode")
        self._output = _clamp(value, self.output_limits)

    # --- Bias ---

    @property
    def bias(self):
        """Current bias term (tracks last output)."""
        return self._bias

    @bias.setter
    def bias(self, value):
        """Write the bias term (not allowed in MANUAL mode).

        In Auto/Cascade mode, this sets the integral term to achieve the
        desired bias on the next scan.
        """
        if self._mode == Mode.MANUAL:
            raise RuntimeError("Bias cannot be written in MANUAL mode")
        self._bias = value
        if self.Kc != 0:
            self._integral = value / self.Kc
        else:
            self._integral = 0.0

    # --- Remote setpoint (for Cascade mode) ---

    @property
    def remote_setpoint(self):
        """Remote setpoint used in CASCADE mode."""
        return self._remote_setpoint

    @remote_setpoint.setter
    def remote_setpoint(self, value):
        """Set the remote setpoint for CASCADE mode."""
        self._remote_setpoint = value

    # --- Alarms ---

    @property
    def alarms(self):
        """Current alarm state (AlarmState object)."""
        return self._alarm_state

    # --- Output limits ---

    @property
    def output_limits(self):
        """Current output limits as (lower, upper)."""
        return self._min_output, self._max_output

    @output_limits.setter
    def output_limits(self, limits):
        """Set output limits for clamping and anti-windup."""
        if limits is None:
            self._min_output, self._max_output = None, None
            return

        min_output, max_output = limits
        if None not in limits and max_output < min_output:
            raise ValueError("lower limit must be less than upper limit")

        self._min_output = min_output
        self._max_output = max_output
        self._last_output = _clamp(self._last_output, self.output_limits)

    # --- Inspection properties ---

    @property
    def components(self):
        """The P, I, and D contributions from the last computation as a tuple.

        These are the raw contributions BEFORE Kc multiplication:
            (proportional, integral, derivative)

        The final output = Kc * (P + I + D), clamped to output_limits.
        """
        return self._proportional, self._integral, self._derivative

    @property
    def tunings(self):
        """Current tunings as (Kc, Ti, Td)."""
        return self.Kc, self.Ti, self.Td

    @tunings.setter
    def tunings(self, tunings):
        """Set PID tunings."""
        self.Kc, self.Ti, self.Td = tunings

    @property
    def pv(self):
        """Last process variable value."""
        return self._pv if hasattr(self, '_pv') else None

    # --- Reset ---

    def reset(self):
        """Reset controller internals."""
        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._last_time = self.time_fn()
        self._last_output = None
        self._last_input = None
        self._last_error = None

    def __repr__(self):
        return (
            f"PID(Kc={self.Kc}, Ti={self.Ti}, Td={self.Td}, "
            f"setpoint={self.setpoint}, mode={self._mode.value}, "
            f"output={self._output})"
        )
