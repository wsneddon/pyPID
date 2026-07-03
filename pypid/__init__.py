"""pyPID - Enhanced PID controller with modes, alarms, scaling, and simulation."""

from .pid import PID, Mode
from .alarms import AlarmConfig, AlarmState, evaluate_alarms
from .scaling import Scaler
from .simulation import FOPDTSimulator
from .scheduler import Scheduler

__version__ = "0.1.0"

__all__ = [
    "PID",
    "Mode",
    "AlarmConfig",
    "AlarmState",
    "evaluate_alarms",
    "Scaler",
    "FOPDTSimulator",
    "Scheduler",
]
