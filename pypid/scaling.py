"""Engineering units scaling utilities.

Converts raw A/D (analog-to-digital) counts to engineering units.
Default calibration matches a typical 4-20mA input card:
    6400 counts  = 4 mA  (0% of EU range)
    32000 counts = 20 mA (100% of EU range)
"""


class Scaler:
    """Linear scaling between raw A/D counts and engineering units (EU).

    Converts a raw input value (integer counts from an A/D converter) to
    engineering units using linear interpolation:

        eu = eu_min + (counts - raw_lo) * (eu_max - eu_min) / (raw_hi - raw_lo)

    Parameters
    ----------
    raw_lo : int or float
        A/D count value corresponding to the low end of the EU range.
        Default: 6400 (typical 4 mA on a 4-20 mA input card).
    raw_hi : int or float
        A/D count value corresponding to the high end of the EU range.
        Default: 32000 (typical 20 mA on a 4-20 mA input card).
    eu_lo : float
        Engineering units value at raw_lo (e.g., 0.0 PSI, -40.0 °F).
    eu_hi : float
        Engineering units value at raw_hi (e.g., 100.0 PSI, 200.0 °F).
    clamp : bool
        If True, clamp the EU output to [eu_lo, eu_hi]. Default False.
        When False, values outside the raw range will extrapolate.

    Examples
    --------
    >>> # Temperature transmitter: 6400 counts = 0°F, 32000 counts = 200°F
    >>> s = Scaler(eu_lo=0.0, eu_hi=200.0)
    >>> s.to_eu(19200)  # midpoint
    100.0

    >>> # Pressure transmitter: custom A/D range
    >>> s = Scaler(raw_lo=3200, raw_hi=16000, eu_lo=0.0, eu_hi=500.0)
    >>> s.to_eu(9600)  # 50%
    250.0
    """

    def __init__(self, raw_lo=6400, raw_hi=32000, eu_lo=0.0, eu_hi=100.0, clamp=False):
        if raw_hi == raw_lo:
            raise ValueError("raw_lo and raw_hi must be different")
        self.raw_lo = raw_lo
        self.raw_hi = raw_hi
        self.eu_lo = eu_lo
        self.eu_hi = eu_hi
        self.clamp = clamp

    @property
    def raw_span(self):
        """Raw count span (raw_hi - raw_lo)."""
        return self.raw_hi - self.raw_lo

    @property
    def eu_span(self):
        """Engineering units span (eu_hi - eu_lo)."""
        return self.eu_hi - self.eu_lo

    @property
    def counts_per_eu(self):
        """Number of A/D counts per engineering unit."""
        return self.raw_span / self.eu_span if self.eu_span != 0 else float('inf')

    def to_eu(self, counts):
        """Convert raw A/D counts to engineering units.

        Parameters
        ----------
        counts : int or float
            The raw A/D count value.

        Returns
        -------
        float
            The value in engineering units.
        """
        eu = self.eu_lo + (counts - self.raw_lo) * self.eu_span / self.raw_span
        if self.clamp:
            lo = min(self.eu_lo, self.eu_hi)
            hi = max(self.eu_lo, self.eu_hi)
            eu = max(lo, min(hi, eu))
        return eu

    def to_counts(self, eu_value):
        """Convert an engineering units value back to raw A/D counts.

        Parameters
        ----------
        eu_value : float
            The engineering units value.

        Returns
        -------
        float
            The corresponding A/D count value.
        """
        counts = self.raw_lo + (eu_value - self.eu_lo) * self.raw_span / self.eu_span
        if self.clamp:
            lo = min(self.raw_lo, self.raw_hi)
            hi = max(self.raw_lo, self.raw_hi)
            counts = max(lo, min(hi, counts))
        return counts

    # Keep backward-compatible aliases
    to_raw = to_counts

    @property
    def span_raw(self):
        """Alias for raw_span (backward compatibility)."""
        return self.raw_span

    @property
    def span_eu(self):
        """Alias for eu_span (backward compatibility)."""
        return self.eu_span

    def percent(self, counts):
        """Convert raw counts to percent of range (0-100%).

        Parameters
        ----------
        counts : int or float
            The raw A/D count value.

        Returns
        -------
        float
            Percent of range.
        """
        return (counts - self.raw_lo) / self.raw_span * 100.0

    def __repr__(self):
        return (
            f"Scaler(raw=[{self.raw_lo}, {self.raw_hi}] counts, "
            f"eu=[{self.eu_lo}, {self.eu_hi}], clamp={self.clamp})"
        )
