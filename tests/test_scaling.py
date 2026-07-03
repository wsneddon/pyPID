"""Unit tests for the scaling module."""

import pytest
from pypid import Scaler, PID


class TestScaler:
    """Test engineering units scaling with A/D counts."""

    def test_default_midpoint(self):
        """Default 6400-32000 counts -> 0-100 EU, midpoint = 19200 -> 50."""
        s = Scaler()
        assert s.to_eu(19200) == pytest.approx(50.0)

    def test_default_endpoints(self):
        s = Scaler()
        assert s.to_eu(6400) == pytest.approx(0.0)
        assert s.to_eu(32000) == pytest.approx(100.0)

    def test_custom_eu_range(self):
        """Temperature: 6400 counts = 0°F, 32000 counts = 200°F."""
        s = Scaler(eu_lo=0.0, eu_hi=200.0)
        assert s.to_eu(6400) == pytest.approx(0.0)
        assert s.to_eu(32000) == pytest.approx(200.0)
        assert s.to_eu(19200) == pytest.approx(100.0)

    def test_custom_count_range(self):
        """Custom A/D card: 3200-16000 counts."""
        s = Scaler(raw_lo=3200, raw_hi=16000, eu_lo=0.0, eu_hi=500.0)
        assert s.to_eu(3200) == pytest.approx(0.0)
        assert s.to_eu(16000) == pytest.approx(500.0)
        assert s.to_eu(9600) == pytest.approx(250.0)

    def test_reverse_scaling(self):
        """Reverse acting: higher counts = lower EU (e.g., vacuum)."""
        s = Scaler(eu_lo=100.0, eu_hi=0.0)
        assert s.to_eu(6400) == pytest.approx(100.0)
        assert s.to_eu(32000) == pytest.approx(0.0)

    def test_to_counts(self):
        s = Scaler(eu_lo=0.0, eu_hi=200.0)
        assert s.to_counts(0.0) == pytest.approx(6400)
        assert s.to_counts(200.0) == pytest.approx(32000)
        assert s.to_counts(100.0) == pytest.approx(19200)

    def test_to_raw_alias(self):
        """to_raw should work as alias for to_counts."""
        s = Scaler(eu_lo=0.0, eu_hi=100.0)
        assert s.to_raw(50.0) == pytest.approx(19200)

    def test_clamp_high(self):
        s = Scaler(eu_lo=0.0, eu_hi=100.0, clamp=True)
        assert s.to_eu(40000) == pytest.approx(100.0)  # above raw_hi, clamped

    def test_clamp_low(self):
        s = Scaler(eu_lo=0.0, eu_hi=100.0, clamp=True)
        assert s.to_eu(0) == pytest.approx(0.0)  # below raw_lo, clamped

    def test_no_clamp_extrapolates(self):
        s = Scaler(eu_lo=0.0, eu_hi=100.0, clamp=False)
        # 0 counts is below raw_lo=6400 by 6400 counts
        # 6400 / 25600 span * 100 EU = 25 EU below zero = -25
        assert s.to_eu(0) == pytest.approx(-25.0)

    def test_same_raw_raises(self):
        with pytest.raises(ValueError):
            Scaler(raw_lo=6400, raw_hi=6400)

    def test_span_properties(self):
        s = Scaler()
        assert s.raw_span == pytest.approx(25600)  # 32000 - 6400
        assert s.eu_span == pytest.approx(100.0)

    def test_counts_per_eu(self):
        s = Scaler(eu_lo=0.0, eu_hi=100.0)
        assert s.counts_per_eu == pytest.approx(256.0)  # 25600 / 100

    def test_percent(self):
        s = Scaler()
        assert s.percent(6400) == pytest.approx(0.0)
        assert s.percent(32000) == pytest.approx(100.0)
        assert s.percent(19200) == pytest.approx(50.0)

    def test_float_counts_accepted(self):
        """A/D counts can be float (e.g., from averaging or filtering)."""
        s = Scaler(eu_lo=0.0, eu_hi=100.0)
        assert s.to_eu(19200.5) == pytest.approx(50.001953125)


class TestScalerInPID:
    """Test scaler used externally before passing PV to PID."""

    def test_scaled_input_to_pid(self):
        """Scale counts to EU externally, then feed to PID."""
        scaler = Scaler(eu_lo=0.0, eu_hi=100.0)
        pid = PID(Kc=1.0, Ti=None, Td=None, setpoint=50.0, sample_time=None)
        # 19200 counts = 50 EU = setpoint, error should be 0
        pv = scaler.to_eu(19200)
        output = pid(pv, dt=1.0)
        assert output == pytest.approx(0.0)

    def test_pid_pv_property_shows_eu(self):
        """pid.pv shows the EU value passed in."""
        scaler = Scaler(eu_lo=0.0, eu_hi=200.0)
        pid = PID(Kc=1.0, setpoint=100.0, sample_time=None)
        pv = scaler.to_eu(19200)  # 19200 counts = 100 EU
        pid(pv, dt=1.0)
        assert pid.pv == pytest.approx(100.0)

    def test_pid_error_from_eu(self):
        """PID computes error in EU after external scaling."""
        scaler = Scaler(eu_lo=0.0, eu_hi=100.0)
        pid = PID(Kc=2.0, Ti=None, Td=None, setpoint=75.0, sample_time=None)
        # 19200 counts = 50 EU, error = 75 - 50 = 25, output = Kc * 25 = 50
        pv = scaler.to_eu(19200)
        output = pid(pv, dt=1.0)
        assert output == pytest.approx(50.0)
