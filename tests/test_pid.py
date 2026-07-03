"""Unit tests for the PID controller (coupled ISA form)."""

import pytest
from pypid import PID, Mode


class TestBasicPID:
    """Test basic PID computation in coupled form: output = Kc * [e + (1/Ti)*∫e*dt + Td*de/dt]"""

    def test_proportional_only(self):
        pid = PID(Kc=2.0, Ti=None, Td=None, setpoint=10.0, sample_time=None)
        output = pid(0.0, dt=1.0)
        # error = 10 - 0 = 10, output = Kc * error = 2 * 10 = 20
        assert output == pytest.approx(20.0)

    def test_integral_accumulates_seconds(self):
        # Ti=1.0 second, time_base='seconds'
        pid = PID(Kc=1.0, Ti=1.0, Td=None, setpoint=10.0,
                  sample_time=None, time_base='seconds')
        pid(0.0, dt=1.0)
        # After 1st call: P=10, I=(1/1)*10*1=10, output = 1*(10+10) = 20
        output = pid(0.0, dt=1.0)
        # After 2nd call: P=10, I=10+10=20, output = 1*(10+20) = 30
        assert output == pytest.approx(30.0)

    def test_integral_accumulates_minutes(self):
        # Ti=1.0 minute (default), dt=60s = 1 minute
        pid = PID(Kc=1.0, Ti=1.0, Td=None, setpoint=10.0, sample_time=None)
        pid(0.0, dt=60.0)
        # After 1st call: dt_base=60/60=1min, P=10, I=(1/1)*10*1=10, output=20
        output = pid(0.0, dt=60.0)
        # After 2nd call: P=10, I=10+10=20, output=30
        assert output == pytest.approx(30.0)

    def test_longer_ti_slower_integral(self):
        """Larger Ti = slower integral action."""
        pid = PID(Kc=1.0, Ti=10.0, Td=None, setpoint=10.0,
                  sample_time=None, time_base='seconds')
        pid(0.0, dt=1.0)
        # P=10, I=(1/10)*10*1=1.0, output = 1*(10+1) = 11
        output = pid(0.0, dt=1.0)
        # P=10, I=1+1=2, output = 1*(10+2) = 12
        assert output == pytest.approx(12.0)

    def test_derivative(self):
        pid = PID(Kc=1.0, Ti=None, Td=1.0, setpoint=10.0,
                  sample_time=None, time_base='seconds')
        pid(0.0, dt=1.0)  # first call, d_input = 0
        output = pid(5.0, dt=1.0)
        # P = error = 10-5 = 5
        # D = -Td * d_input/dt_base = -1.0 * 5/1.0 = -5
        # output = Kc * (P + D) = 1 * (5 + (-5)) = 0
        assert output == pytest.approx(0.0)

    def test_gain_couples_all_terms(self):
        """Changing Kc should scale P, I, and D proportionally."""
        pid1 = PID(Kc=1.0, Ti=1.0, Td=None, setpoint=10.0,
                   sample_time=None, time_base='seconds')
        pid2 = PID(Kc=3.0, Ti=1.0, Td=None, setpoint=10.0,
                   sample_time=None, time_base='seconds')
        out1 = pid1(0.0, dt=1.0)
        out2 = pid2(0.0, dt=1.0)
        # pid1: Kc*(e + I) = 1*(10+10) = 20
        # pid2: Kc*(e + I) = 3*(10+10) = 60
        assert out1 == pytest.approx(20.0)
        assert out2 == pytest.approx(60.0)
        assert out2 == pytest.approx(3.0 * out1)

    def test_output_clamping(self):
        pid = PID(Kc=10.0, Ti=None, Td=None, setpoint=100.0,
                  output_limits=(0, 50), sample_time=None)
        output = pid(0.0, dt=1.0)
        assert output == 50.0

    def test_anti_windup(self):
        """Integral should not wind up beyond output limits."""
        pid = PID(Kc=1.0, Ti=0.1, Td=None, setpoint=100.0,
                  output_limits=(0, 50), sample_time=None, time_base='seconds')
        # Run several cycles to try to wind up
        for _ in range(100):
            pid(0.0, dt=1.0)
        # Now set setpoint below PV — integral should unwind quickly
        pid.setpoint = 0.0
        output = pid(50.0, dt=1.0)
        # Should be able to go below 50 immediately (no windup)
        assert output < 50.0

    def test_ti_none_disables_integral(self):
        pid = PID(Kc=1.0, Ti=None, Td=None, setpoint=10.0, sample_time=None)
        pid(0.0, dt=1.0)
        pid(0.0, dt=1.0)
        _, i, _ = pid.components
        assert i == pytest.approx(0.0)

    def test_td_none_disables_derivative(self):
        pid = PID(Kc=1.0, Ti=None, Td=None, setpoint=10.0, sample_time=None)
        pid(0.0, dt=1.0)
        pid(5.0, dt=1.0)
        _, _, d = pid.components
        assert d == pytest.approx(0.0)


class TestTimeBase:
    """Test time_base parameter behavior."""

    def test_default_time_base_is_minutes(self):
        pid = PID(Kc=1.0, Ti=1.0)
        assert pid.time_base == 'minutes'

    def test_minutes_vs_seconds_integral(self):
        """Ti=1 minute should give same result as Ti=60 seconds."""
        pid_min = PID(Kc=1.0, Ti=1.0, Td=None, setpoint=10.0,
                      sample_time=None, time_base='minutes')
        pid_sec = PID(Kc=1.0, Ti=60.0, Td=None, setpoint=10.0,
                      sample_time=None, time_base='seconds')
        # Same dt in seconds for both
        out_min = pid_min(0.0, dt=1.0)
        out_sec = pid_sec(0.0, dt=1.0)
        assert out_min == pytest.approx(out_sec)

    def test_minutes_vs_seconds_derivative(self):
        """Td=1 minute should give same result as Td=60 seconds."""
        pid_min = PID(Kc=1.0, Ti=None, Td=1.0, setpoint=10.0,
                      sample_time=None, time_base='minutes')
        pid_sec = PID(Kc=1.0, Ti=None, Td=60.0, setpoint=10.0,
                      sample_time=None, time_base='seconds')
        # First call to establish baseline
        pid_min(0.0, dt=1.0)
        pid_sec(0.0, dt=1.0)
        # Second call with PV change
        out_min = pid_min(5.0, dt=1.0)
        out_sec = pid_sec(5.0, dt=1.0)
        assert out_min == pytest.approx(out_sec)

    def test_invalid_time_base_raises(self):
        with pytest.raises(ValueError):
            PID(Kc=1.0, time_base='hours')

    def test_minutes_integral_rate(self):
        """With Ti=1 min and dt=1s, integral adds (1/1)*(error)*(1/60) per scan."""
        pid = PID(Kc=1.0, Ti=1.0, Td=None, setpoint=10.0, sample_time=None)
        pid(0.0, dt=1.0)
        # P=10, I=(1/1)*10*(1/60) = 0.1667, output = 10.1667
        _, i, _ = pid.components
        assert i == pytest.approx(10.0 / 60.0, rel=1e-6)


class TestReverseActing:
    """Test reverse acting mode."""

    def test_direct_acting(self):
        pid = PID(Kc=1.0, Ti=None, Td=None, setpoint=50.0,
                  reverse_acting=False, sample_time=None)
        output = pid(40.0, dt=1.0)  # error = 50 - 40 = 10
        assert output == pytest.approx(10.0)

    def test_reverse_acting(self):
        pid = PID(Kc=1.0, Ti=None, Td=None, setpoint=50.0,
                  reverse_acting=True, sample_time=None)
        output = pid(40.0, dt=1.0)  # error = 40 - 50 = -10
        assert output == pytest.approx(-10.0)

    def test_reverse_acting_cooling(self):
        """In a cooling loop, PV > SP should increase output."""
        pid = PID(Kc=1.0, Ti=None, Td=None, setpoint=50.0,
                  reverse_acting=True, sample_time=None)
        output = pid(60.0, dt=1.0)  # error = 60 - 50 = 10
        assert output == pytest.approx(10.0)


class TestModes:
    """Test operating mode transitions."""

    def test_default_mode_is_auto(self):
        pid = PID(setpoint=50.0)
        assert pid.mode == Mode.AUTO

    def test_manual_mode_holds_output(self):
        pid = PID(Kc=1.0, Ti=None, Td=None, setpoint=50.0, sample_time=None)
        pid(25.0, dt=1.0)  # output = 25
        pid.mode = Mode.MANUAL
        # Output should stay at last value regardless of PV change
        output = pid(0.0, dt=1.0)
        assert output == pytest.approx(25.0)

    def test_manual_mode_writable(self):
        pid = PID(Kc=1.0, setpoint=50.0, sample_time=None)
        pid.mode = Mode.MANUAL
        pid.output = 42.0
        output = pid(0.0, dt=1.0)
        assert output == pytest.approx(42.0)

    def test_auto_mode_not_writable(self):
        pid = PID(Kc=1.0, setpoint=50.0, sample_time=None)
        pid(25.0, dt=1.0)
        with pytest.raises(RuntimeError):
            pid.output = 99.0

    def test_cascade_uses_remote_setpoint(self):
        pid = PID(Kc=1.0, Ti=None, Td=None, setpoint=50.0, sample_time=None)
        pid.mode = Mode.CASCADE
        pid.remote_setpoint = 70.0
        output = pid(60.0, dt=1.0)  # error = 70 - 60 = 10
        assert output == pytest.approx(10.0)

    def test_bumpless_transfer_manual_to_auto(self):
        """Output should not jump on Manual -> Auto transition."""
        pid = PID(Kc=1.0, Ti=1.0, Td=None, setpoint=50.0,
                  output_limits=(0, 100), sample_time=None)
        pid.mode = Mode.MANUAL
        pid.output = 60.0
        pid(50.0, dt=1.0)  # one scan in manual at PV=SP

        pid.mode = Mode.AUTO
        output = pid(50.0, dt=0.1)  # PV == SP, error = 0
        # Output should be close to 60 (the manual output), not jump to 0
        assert abs(output - 60.0) < 5.0


class TestBias:
    """Test bias term behavior."""

    def test_bias_tracks_output(self):
        pid = PID(Kc=1.0, Ti=None, Td=None, setpoint=50.0, sample_time=None)
        output = pid(40.0, dt=1.0)
        assert pid.bias == pytest.approx(output)

    def test_bias_not_writable_in_manual(self):
        pid = PID(setpoint=50.0)
        pid.mode = Mode.MANUAL
        with pytest.raises(RuntimeError):
            pid.bias = 30.0

    def test_bias_writable_in_auto(self):
        pid = PID(Kc=1.0, Ti=1.0, Td=None, setpoint=50.0, sample_time=None)
        pid(40.0, dt=1.0)  # prime the controller
        pid.bias = 25.0
        assert pid.bias == pytest.approx(25.0)

    def test_bias_overwritten_on_manual_to_auto(self):
        """Bias should equal the manual output after transition."""
        pid = PID(Kc=1.0, Ti=1.0, Td=None, setpoint=50.0,
                  output_limits=(0, 100), sample_time=None)
        pid.mode = Mode.MANUAL
        pid.output = 45.0
        pid(50.0, dt=1.0)
        pid.mode = Mode.AUTO
        assert pid.bias == pytest.approx(45.0)


class TestComponents:
    """Test inspection properties."""

    def test_components_tuple(self):
        pid = PID(Kc=2.0, Ti=1.0, Td=None, setpoint=10.0,
                  sample_time=None, time_base='seconds')
        pid(5.0, dt=1.0)
        p, i, d = pid.components
        # P = error = 5
        assert p == pytest.approx(5.0)
        # I = (1/Ti) * error * dt_base = (1/1) * 5 * 1 = 5
        assert i == pytest.approx(5.0)
        # Output = Kc * (P + I) = 2 * (5 + 5) = 20
        assert pid.output == pytest.approx(20.0)

    def test_tunings_property(self):
        pid = PID(Kc=1.0, Ti=2.0, Td=3.0)
        assert pid.tunings == (1.0, 2.0, 3.0)
        pid.tunings = (4.0, 5.0, 6.0)
        assert pid.Kc == 4.0
        assert pid.Ti == 5.0
        assert pid.Td == 6.0

    def test_pv_property(self):
        pid = PID(Kc=1.0, setpoint=50.0, sample_time=None)
        pid(42.0, dt=1.0)
        assert pid.pv == pytest.approx(42.0)
