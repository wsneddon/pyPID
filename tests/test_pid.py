"""Unit tests for the PID controller."""

import pytest
from pypid import PID, Mode


class TestBasicPID:
    """Test basic PID computation."""

    def test_proportional_only(self):
        pid = PID(Kp=2.0, Ki=0.0, Kd=0.0, setpoint=10.0, sample_time=None)
        output = pid(0.0, dt=1.0)
        # error = 10 - 0 = 10, P = 2 * 10 = 20
        assert output == pytest.approx(20.0)

    def test_integral_accumulates(self):
        pid = PID(Kp=0.0, Ki=1.0, Kd=0.0, setpoint=10.0, sample_time=None)
        pid(0.0, dt=1.0)  # integral = 1.0 * 10 * 1.0 = 10
        output = pid(0.0, dt=1.0)  # integral = 10 + 10 = 20
        assert output == pytest.approx(20.0)

    def test_derivative(self):
        pid = PID(Kp=0.0, Ki=0.0, Kd=1.0, setpoint=10.0, sample_time=None)
        pid(0.0, dt=1.0)  # first call, d_input = 0
        output = pid(5.0, dt=1.0)  # d_input = 5, derivative = -1.0 * 5 / 1.0 = -5
        assert output == pytest.approx(-5.0)

    def test_output_clamping(self):
        pid = PID(Kp=10.0, Ki=0.0, Kd=0.0, setpoint=100.0,
                  output_limits=(0, 50), sample_time=None)
        output = pid(0.0, dt=1.0)
        assert output == 50.0

    def test_anti_windup(self):
        """Integral should not wind up beyond output limits."""
        pid = PID(Kp=0.0, Ki=10.0, Kd=0.0, setpoint=100.0,
                  output_limits=(0, 50), sample_time=None)
        # Run several cycles to try to wind up
        for _ in range(100):
            pid(0.0, dt=1.0)
        # Now set setpoint below PV - integral should unwind quickly
        pid.setpoint = 0.0
        output = pid(50.0, dt=1.0)
        # Should be able to go below 50 immediately (no windup)
        assert output < 50.0


class TestReverseActing:
    """Test reverse acting mode."""

    def test_direct_acting(self):
        pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=50.0,
                  reverse_acting=False, sample_time=None)
        output = pid(40.0, dt=1.0)  # error = 50 - 40 = 10
        assert output == pytest.approx(10.0)

    def test_reverse_acting(self):
        pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=50.0,
                  reverse_acting=True, sample_time=None)
        output = pid(40.0, dt=1.0)  # error = 40 - 50 = -10
        assert output == pytest.approx(-10.0)

    def test_reverse_acting_cooling(self):
        """In a cooling loop, PV > SP should increase output."""
        pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=50.0,
                  reverse_acting=True, sample_time=None)
        output = pid(60.0, dt=1.0)  # error = 60 - 50 = 10
        assert output == pytest.approx(10.0)


class TestModes:
    """Test operating mode transitions."""

    def test_default_mode_is_auto(self):
        pid = PID(setpoint=50.0)
        assert pid.mode == Mode.AUTO

    def test_manual_mode_holds_output(self):
        pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=50.0, sample_time=None)
        pid(25.0, dt=1.0)  # output = 25
        pid.mode = Mode.MANUAL
        # Output should stay at last value regardless of PV change
        output = pid(0.0, dt=1.0)
        assert output == pytest.approx(25.0)

    def test_manual_mode_writable(self):
        pid = PID(Kp=1.0, setpoint=50.0, sample_time=None)
        pid.mode = Mode.MANUAL
        pid.output = 42.0
        output = pid(0.0, dt=1.0)
        assert output == pytest.approx(42.0)

    def test_auto_mode_not_writable(self):
        pid = PID(Kp=1.0, setpoint=50.0, sample_time=None)
        pid(25.0, dt=1.0)
        with pytest.raises(RuntimeError):
            pid.output = 99.0

    def test_cascade_uses_remote_setpoint(self):
        pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=50.0, sample_time=None)
        pid.mode = Mode.CASCADE
        pid.remote_setpoint = 70.0
        output = pid(60.0, dt=1.0)  # error = 70 - 60 = 10
        assert output == pytest.approx(10.0)

    def test_bumpless_transfer_manual_to_auto(self):
        """Output should not jump on Manual -> Auto transition."""
        pid = PID(Kp=1.0, Ki=0.1, Kd=0.0, setpoint=50.0,
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
        pid = PID(Kp=1.0, Ki=0.0, Kd=0.0, setpoint=50.0, sample_time=None)
        output = pid(40.0, dt=1.0)
        assert pid.bias == pytest.approx(output)

    def test_bias_not_writable_in_manual(self):
        pid = PID(setpoint=50.0)
        pid.mode = Mode.MANUAL
        with pytest.raises(RuntimeError):
            pid.bias = 30.0

    def test_bias_writable_in_auto(self):
        pid = PID(Kp=1.0, Ki=0.1, Kd=0.0, setpoint=50.0, sample_time=None)
        pid(40.0, dt=1.0)  # prime the controller
        pid.bias = 25.0
        assert pid.bias == pytest.approx(25.0)

    def test_bias_overwritten_on_manual_to_auto(self):
        """Bias should equal the manual output after transition."""
        pid = PID(Kp=1.0, Ki=0.1, Kd=0.0, setpoint=50.0,
                  output_limits=(0, 100), sample_time=None)
        pid.mode = Mode.MANUAL
        pid.output = 45.0
        pid(50.0, dt=1.0)
        pid.mode = Mode.AUTO
        assert pid.bias == pytest.approx(45.0)


class TestComponents:
    """Test inspection properties."""

    def test_components_tuple(self):
        pid = PID(Kp=1.0, Ki=1.0, Kd=1.0, setpoint=10.0, sample_time=None)
        pid(5.0, dt=1.0)
        p, i, d = pid.components
        assert p == pytest.approx(5.0)  # Kp * error = 1 * 5
        assert i == pytest.approx(5.0)  # Ki * error * dt = 1 * 5 * 1

    def test_tunings_property(self):
        pid = PID(Kp=1.0, Ki=2.0, Kd=3.0)
        assert pid.tunings == (1.0, 2.0, 3.0)
        pid.tunings = (4.0, 5.0, 6.0)
        assert pid.Kp == 4.0
        assert pid.Ki == 5.0
        assert pid.Kd == 6.0

    def test_pv_property(self):
        pid = PID(Kp=1.0, setpoint=50.0, sample_time=None)
        pid(42.0, dt=1.0)
        assert pid.pv == pytest.approx(42.0)
