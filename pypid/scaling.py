"""Engineering units scaling utilities."""


class Scaler:
    """Linear scaling between raw and engineering units.

    Converts a raw input value to engineering units (EU) using linear interpolation:
        eu = eu_min + (raw - raw_min) * (eu_max - eu_min) / (raw_max - raw_min)

    Parameters
    ----------
    raw_min : float
        Minimum raw input value (e.g., 4 mA or 0 counts).
    raw_max : float
        Maximum raw input value (e.g., 20 mA or 65535 counts).
    eu_min : float
        Engineering units value corresponding to raw_min.
    eu_max : float
        Engineering units value corresponding to raw_max.
    clamp : bool
        If True, clamp the output to [eu_min, eu_max]. Default False.
    """

    def __init__(self, raw_min=0.0, raw_max=100.0, eu_min=0.0, eu_max=100.0, clamp=False):
        if raw_max == raw_min:
            raise ValueError("raw_min and raw_max must be different")
        self.raw_min = raw_min
        self.raw_max = raw_max
        self.eu_min = eu_min
        self.eu_max = eu_max
        self.clamp = clamp

    @property
    def span_raw(self):
        """Raw input span."""
        return self.raw_max - self.raw_min

    @property
    def span_eu(self):
        """Engineering units span."""
        return self.eu_max - self.eu_min

    def to_eu(self, raw_value):
        """Convert a raw value to engineering units.

        Parameters
        ----------
        raw_value : float
            The raw input value.

        Returns
        -------
        float
            The value in engineering units.
        """
        eu = self.eu_min + (raw_value - self.raw_min) * self.span_eu / self.span_raw
        if self.clamp:
            lo = min(self.eu_min, self.eu_max)
            hi = max(self.eu_min, self.eu_max)
            eu = max(lo, min(hi, eu))
        return eu

    def to_raw(self, eu_value):
        """Convert an engineering units value back to raw.

        Parameters
        ----------
        eu_value : float
            The engineering units value.

        Returns
        -------
        float
            The raw value.
        """
        raw = self.raw_min + (eu_value - self.eu_min) * self.span_raw / self.span_eu
        if self.clamp:
            lo = min(self.raw_min, self.raw_max)
            hi = max(self.raw_min, self.raw_max)
            raw = max(lo, min(hi, raw))
        return raw

    def __repr__(self):
        return (
            f"Scaler(raw=[{self.raw_min}, {self.raw_max}], "
            f"eu=[{self.eu_min}, {self.eu_max}], clamp={self.clamp})"
        )
