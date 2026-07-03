# pyPID

An enhanced PID controller for Python, forked from [simple-pid](https://github.com/m-lundberg/simple-pid).

## Features

- **Operating Modes**: Manual, Auto, and Cascade with bumpless transfer
- **Reverse Acting**: Configurable error direction (SP-PV or PV-SP)
- **Bias Term**: Externally writable bias with proper mode transition handling
- **Engineering Units Scaling**: Separate Scaler module for A/D count-to-EU conversion
- **Alarms**: Configurable high/low alarm setpoints with boolean outputs
- **FOPDT Simulation**: Decoupled First Order Plus Dead Time simulator for testing
- **Optional Scheduler**: Threaded execution for real-time applications
- **Jupyter Friendly**: Explicit `dt` parameter for simulation loops, all state inspectable

## Installation

```bash
pip install -e .
```

## Quick Start

```python
from pypid import PID, FOPDTSimulator

# Create controller and simulator (coupled ISA form)
pid = PID(Kc=1.2, Ti=10.0, Td=2.0, setpoint=50.0, output_limits=(0, 100))
sim = FOPDTSimulator(K=2.0, tau=10.0, theta=2.0)

# Run a control loop
pv = 25.0
for i in range(500):
    output = pid(pv, dt=0.1)
    pv = sim.update(output, dt=0.1)
    print(f"t={i*0.1:.1f}  PV={pv:.2f}  OUT={output:.2f}")
```

## Scaling (I/O Layer)

Scaling is handled separately from the PID — convert raw A/D counts to engineering
units before passing to the controller. This mirrors how real DCS/PLC systems work.

```python
from pypid import Scaler

# Default: 6400 counts (4mA) to 32000 counts (20mA)
scaler = Scaler(eu_lo=0.0, eu_hi=200.0)  # 0-200°F

raw_counts = 19200  # from your A/D card
pv = scaler.to_eu(raw_counts)  # → 100.0°F

# Custom A/D range
scaler = Scaler(raw_lo=0, raw_hi=65535, eu_lo=-40.0, eu_hi=300.0)
```

## Modes

```python
from pypid import PID, Mode

pid = PID(Kc=1.0, Ti=10.0, Td=0.0, setpoint=50.0)

# Manual mode - output is writable
pid.mode = Mode.MANUAL
pid.output = 25.0

# Auto mode - output computed by PID
pid.mode = Mode.AUTO

# Cascade mode - uses remote setpoint
pid.mode = Mode.CASCADE
pid.remote_setpoint = 55.0
```

## Alarms

```python
from pypid import PID, AlarmConfig

alarms = AlarmConfig(hsp=80.0, hhsp=90.0, lsp=20.0, llsp=10.0, yeldev_sp=5.0)
pid = PID(Kc=1.0, Ti=10.0, setpoint=50.0, alarm_config=alarms)

pid(85.0, dt=0.1)
print(pid.alarms.h)    # True — PV > hsp
print(pid.alarms.hh)   # False — PV < hhsp
```

## License

MIT License. Based on [simple-pid](https://github.com/m-lundberg/simple-pid) by Martin Lundberg.
