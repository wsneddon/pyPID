"""Unit tests for the FOPDT simulation module."""

import pytest
from pypid import FOPDTSimulator


class TestFOPDTBasic:
    """Test basic FOPDT behavior."""

    def test_initial_state(self):
        sim = FOPDTSimulator(K=2.0, tau=10.0, theta=0.0, y0=25.0)
        assert sim.y == 25.0

    def test_step_response_no_deadtime(self):
        """Step input should approach K * u with time constant tau."""
        sim = FOPDTSimulator(K=2.0, tau=10.0, theta=0.0, y0=0.0)
        # Apply step input of 1.0 for many time constants
        for _ in range(1000):
            sim.update(1.0, dt=0.1)
        # After ~10 tau, should be very close to K * u = 2.0
        assert sim.y == pytest.approx(2.0, abs=0.01)

    def test_step_response_63_percent(self):
        """After 1 time constant, output should be ~63.2% of final value."""
        sim = FOPDTSimulator(K=1.0, tau=10.0, theta=0.0, y0=0.0)
        steps = int(10.0 / 0.01)  # 1 tau with dt=0.01
        for _ in range(steps):
            sim.update(1.0, dt=0.01)
        # Should be approximately 1 - e^(-1) ≈ 0.632
        assert sim.y == pytest.approx(0.632, abs=0.02)

    def test_dead_time(self):
        """Output should not change until dead time has passed."""
        sim = FOPDTSimulator(K=2.0, tau=10.0, theta=5.0, y0=0.0)
        # Apply step input, check output doesn't move for theta seconds
        for _ in range(49):  # 49 * 0.1 = 4.9s < theta=5.0
            sim.update(1.0, dt=0.1)
        assert sim.y == pytest.approx(0.0, abs=0.001)
        # After dead time passes, output should start moving
        for _ in range(20):
            sim.update(1.0, dt=0.1)
        assert sim.y > 0.01

    def test_zero_input_stays_at_zero(self):
        sim = FOPDTSimulator(K=2.0, tau=10.0, theta=0.0, y0=0.0)
        for _ in range(100):
            sim.update(0.0, dt=0.1)
        assert sim.y == pytest.approx(0.0)

    def test_negative_gain(self):
        """Negative gain should produce inverse response."""
        sim = FOPDTSimulator(K=-1.0, tau=5.0, theta=0.0, y0=0.0)
        for _ in range(500):
            sim.update(1.0, dt=0.1)
        assert sim.y == pytest.approx(-1.0, abs=0.01)


class TestFOPDTValidation:
    """Test input validation."""

    def test_negative_tau_raises(self):
        with pytest.raises(ValueError):
            FOPDTSimulator(K=1.0, tau=-1.0, theta=0.0)

    def test_zero_tau_raises(self):
        with pytest.raises(ValueError):
            FOPDTSimulator(K=1.0, tau=0.0, theta=0.0)

    def test_negative_theta_raises(self):
        with pytest.raises(ValueError):
            FOPDTSimulator(K=1.0, tau=1.0, theta=-1.0)

    def test_negative_dt_raises(self):
        sim = FOPDTSimulator(K=1.0, tau=1.0, theta=0.0)
        with pytest.raises(ValueError):
            sim.update(1.0, dt=-0.1)


class TestFOPDTReset:
    """Test simulator reset."""

    def test_reset_to_original(self):
        sim = FOPDTSimulator(K=2.0, tau=10.0, theta=0.0, y0=25.0)
        for _ in range(100):
            sim.update(1.0, dt=0.1)
        sim.reset()
        assert sim.y == 25.0

    def test_reset_to_new_value(self):
        sim = FOPDTSimulator(K=2.0, tau=10.0, theta=0.0, y0=0.0)
        for _ in range(100):
            sim.update(1.0, dt=0.1)
        sim.reset(y0=50.0)
        assert sim.y == 50.0


class TestFOPDTSteadyState:
    """Test steady state property."""

    def test_steady_state_value(self):
        sim = FOPDTSimulator(K=3.0, tau=5.0, theta=0.0, y0=0.0)
        sim.update(10.0, dt=0.1)
        assert sim.steady_state == pytest.approx(30.0)
