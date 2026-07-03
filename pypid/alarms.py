"""Alarm module for process variable monitoring."""


class AlarmState:
    """Holds the current state of all alarms. All attributes are read-only booleans."""

    __slots__ = ('lll', 'll', 'l', 'h', 'hh', 'hhh', 'yel_dev', 'org_dev')

    def __init__(self):
        self.lll = False
        self.ll = False
        self.l = False
        self.h = False
        self.hh = False
        self.hhh = False
        self.yel_dev = False
        self.org_dev = False

    def as_dict(self):
        """Return alarm states as a dictionary."""
        return {s: getattr(self, s) for s in self.__slots__}

    def any_active(self):
        """Return True if any alarm is active."""
        return any(getattr(self, s) for s in self.__slots__)

    def __repr__(self):
        active = [s for s in self.__slots__ if getattr(self, s)]
        if active:
            return f"AlarmState(active={active})"
        return "AlarmState(all clear)"


class AlarmConfig:
    """Configuration for alarm setpoints.

    Parameters
    ----------
    lllsp : float or None
        Low-low-low setpoint.
    llsp : float or None
        Low-low setpoint.
    lsp : float or None
        Low setpoint.
    hsp : float or None
        High setpoint.
    hhsp : float or None
        High-high setpoint.
    hhhsp : float or None
        High-high-high setpoint.
    yeldev_sp : float or None
        Yellow deviation setpoint (abs(SP - PV) > yeldev_sp).
    orgdev_sp : float or None
        Orange deviation setpoint (abs(SP - PV) > orgdev_sp).
    """

    def __init__(
        self,
        lllsp=None,
        llsp=None,
        lsp=None,
        hsp=None,
        hhsp=None,
        hhhsp=None,
        yeldev_sp=None,
        orgdev_sp=None,
    ):
        self.lllsp = lllsp
        self.llsp = llsp
        self.lsp = lsp
        self.hsp = hsp
        self.hhsp = hhsp
        self.hhhsp = hhhsp
        self.yeldev_sp = yeldev_sp
        self.orgdev_sp = orgdev_sp


def evaluate_alarms(pv, setpoint, config):
    """Evaluate alarm conditions.

    Parameters
    ----------
    pv : float
        Current process variable value.
    setpoint : float
        Current setpoint (used for deviation alarms).
    config : AlarmConfig
        Alarm configuration with setpoints.

    Returns
    -------
    AlarmState
        Current alarm states.
    """
    state = AlarmState()

    if config is None:
        return state

    # Level alarms
    if config.lllsp is not None:
        state.lll = pv < config.lllsp
    if config.llsp is not None:
        state.ll = pv < config.llsp
    if config.lsp is not None:
        state.l = pv < config.lsp
    if config.hsp is not None:
        state.h = pv > config.hsp
    if config.hhsp is not None:
        state.hh = pv > config.hhsp
    if config.hhhsp is not None:
        state.hhh = pv > config.hhhsp

    # Deviation alarms
    deviation = abs(setpoint - pv)
    if config.yeldev_sp is not None:
        state.yel_dev = deviation > config.yeldev_sp
    if config.orgdev_sp is not None:
        state.org_dev = deviation > config.orgdev_sp

    return state
