"""Unit tests for the scaling module."""

import pytest
from pypid import Scaler, PID


class TestScaler:
    """Test engineering units scaling."""

    def test_identity_scaling(self):
        s = Scaler(raw_min=0.0, raw_max=100.0, eu_min=0.0, eu_max=100.0)
        assert s.to_eu(50.0) == pytest.approx(50.0)

    def test_4_20ma_to_0_100(self):
        s = Scaler(raw_min=4.0, raw_max=20.0, eu_min=0.0, eu_max=100.0)
        assert s.to_eu(4.0) == pytest.approx(0.0)
        assert s.to_eu(20.0) == pytest.approx(100.0)
        assert s.to_eu(12.0) == pytest.approx(50.0)

    def test_reverse_scaling(self):
        s = Scaler(raw_min=0.0, raw_max=100.0, eu_min=100.0, eu_max=0.0)
        assert s.to_eu(0.0) == pytest.approx(100.0)
        assert s.to_eu(100.0) == pytest.approx(0.0)

    def test_to_raw(self):
        s = Scaler(raw_min=4.0, raw_max=20.0, eu_min=0.0, eu_max=100.0)
        assert s.to_raw(0.0) == pytest.approx(4.0)
        assert s.to_raw(100.0) == pytest.approx(20.0)
        assert s.to_raw(50.0) == pytest.approx(12.0)

    def test_clamp(self):
        s = Scaler(raw_min=4.0, raw_max=20.0, eu_min=0.0, eu_max=100.0, clamp=True)
        assert s.to_eu(0.0) == pytest.approx(0.0)  # clamped to eu_min
        assert s.to_eu(25.0) == pytest.approx(100.0)  # clamped to eu_max

    def test_no_clamp_allows_extrapolation(self):
        s = Scaler(raw_min=4.0, raw_max=20.0, eu_min=0.0, eu_max=100.0, clamp=False)
        assert s.to_eu(0.0) == pytest.approx(-25.0)  # extrapolated

    def test_same_raw_raises(self):
        with pytest.raises(ValueError):
            Scaler(raw_min=10.0, raw_max=10.0, eu_min=0.0, eu_max=100.0)

    def test_span_properties(self):
        s = Scaler(raw_min=4.0, raw_max=20.0, eu_min=0.0, eu_max=100.0)
        assert s.span_raw == pytest.approx(16.0)
        assert s.span_eu == pytest.approx(100.0)


class TestScalerInPID:
    """Test scaler integrated with PID."""

    def test_scaled_input(self):
        scaler = Scaler(raw_min=4.0, raw_max=20.0, eu_min=0.0, eu_max=100.0)
        pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=50.0,
                  scaler=scaler, sample_time=None)
        # 12 mA = 50% = 50 EU, so error = 50 - 50 = 0
        output = pid(12.0, dt=1.0)
        assert output == pytest.approx(0.0)

    def test_scaled_pv_property(self):
        scaler = Scaler(raw_min=4.0, raw_max=20.0, eu_min=0.0, eu_max=100.0)
        pid = PID(Kp=1.0, setpoint=50.0, scaler=scaler, sample_time=None)
        pid(12.0, dt=1.0)
        assert pid.pv == pytest.approx(50.0)
