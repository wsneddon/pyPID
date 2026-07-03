"""Unit tests for the alarm module."""

import pytest
from pypid import AlarmConfig, AlarmState, evaluate_alarms, PID


class TestAlarmEvaluation:
    """Test alarm state evaluation."""

    def test_all_clear(self):
        config = AlarmConfig(lsp=10.0, hsp=90.0)
        state = evaluate_alarms(pv=50.0, setpoint=50.0, config=config)
        assert not state.any_active()

    def test_high_alarm(self):
        config = AlarmConfig(hsp=80.0, hhsp=90.0, hhhsp=95.0)
        state = evaluate_alarms(pv=85.0, setpoint=50.0, config=config)
        assert state.h is True
        assert state.hh is False
        assert state.hhh is False

    def test_high_high_alarm(self):
        config = AlarmConfig(hsp=80.0, hhsp=90.0, hhhsp=95.0)
        state = evaluate_alarms(pv=92.0, setpoint=50.0, config=config)
        assert state.h is True
        assert state.hh is True
        assert state.hhh is False

    def test_high_high_high_alarm(self):
        config = AlarmConfig(hsp=80.0, hhsp=90.0, hhhsp=95.0)
        state = evaluate_alarms(pv=96.0, setpoint=50.0, config=config)
        assert state.h is True
        assert state.hh is True
        assert state.hhh is True

    def test_low_alarm(self):
        config = AlarmConfig(lsp=20.0, llsp=10.0, lllsp=5.0)
        state = evaluate_alarms(pv=15.0, setpoint=50.0, config=config)
        assert state.l is True
        assert state.ll is False
        assert state.lll is False

    def test_low_low_alarm(self):
        config = AlarmConfig(lsp=20.0, llsp=10.0, lllsp=5.0)
        state = evaluate_alarms(pv=8.0, setpoint=50.0, config=config)
        assert state.l is True
        assert state.ll is True
        assert state.lll is False

    def test_low_low_low_alarm(self):
        config = AlarmConfig(lsp=20.0, llsp=10.0, lllsp=5.0)
        state = evaluate_alarms(pv=3.0, setpoint=50.0, config=config)
        assert state.l is True
        assert state.ll is True
        assert state.lll is True

    def test_yellow_deviation(self):
        config = AlarmConfig(yeldev_sp=5.0, orgdev_sp=10.0)
        state = evaluate_alarms(pv=44.0, setpoint=50.0, config=config)
        assert state.yel_dev is True  # |50-44| = 6 > 5
        assert state.org_dev is False  # 6 < 10

    def test_orange_deviation(self):
        config = AlarmConfig(yeldev_sp=5.0, orgdev_sp=10.0)
        state = evaluate_alarms(pv=38.0, setpoint=50.0, config=config)
        assert state.yel_dev is True  # |50-38| = 12 > 5
        assert state.org_dev is True  # 12 > 10

    def test_none_config(self):
        state = evaluate_alarms(pv=50.0, setpoint=50.0, config=None)
        assert not state.any_active()


class TestAlarmState:
    """Test AlarmState helper methods."""

    def test_as_dict(self):
        state = AlarmState()
        state.h = True
        d = state.as_dict()
        assert d['h'] is True
        assert d['l'] is False

    def test_repr_clear(self):
        state = AlarmState()
        assert "all clear" in repr(state)

    def test_repr_active(self):
        state = AlarmState()
        state.hh = True
        assert "hh" in repr(state)


class TestAlarmsInPID:
    """Test alarms integrated with PID controller."""

    def test_alarms_evaluated_on_call(self):
        config = AlarmConfig(hsp=60.0, lsp=20.0)
        pid = PID(Kp=1.0, setpoint=50.0, alarm_config=config, sample_time=None)
        pid(70.0, dt=1.0)
        assert pid.alarms.h is True
        assert pid.alarms.l is False

    def test_alarms_update_each_scan(self):
        config = AlarmConfig(hsp=60.0)
        pid = PID(Kp=1.0, setpoint=50.0, alarm_config=config, sample_time=None)
        pid(70.0, dt=1.0)
        assert pid.alarms.h is True
        pid(55.0, dt=1.0)
        assert pid.alarms.h is False
