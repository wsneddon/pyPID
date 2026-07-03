# pyPID

An enhanced PID controller for Python, forked from [simple-pid](https://github.com/m-lundberg/simple-pid).

## Features

- **Operating Modes**: Manual, Auto, and Cascade with bumpless transfer
- **Reverse Acting**: Configurable error direction (SP-PV or PV-SP)
- **Bias Term**: Externally writable bias with proper mode transition handling
- **Engineering Units Scaling**: Optional raw-to-EU conversion on inputs
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

# Create controller and simulator
pid = PID(Kp=1.2, Ki=0.5, Kd=0.1, setpoint=50.0, output_limits=(0, 100))
sim = FOPDTSimulator(K=2.0, tau=10.0, theta=2.0)

# Run a control loop
pv = 25.0
for i in range(500):
    output = pid(pv, dt=0.1)
    pv = sim.update(output, dt=0.1)
    print(f"t={i*0.1:.1f}  PV={pv:.2f}  OUT={output:.2f}")
```

## Modes

```python
from pypid import PID, Mode

pid = PID(Kp=1.0, Ki=0.1, Kd=0.0, setpoint=50.0)

# Manual mode - output is writable
pid.mode = Mode.MANUAL
pid.output = 25.0

# Auto mode - output computed by PID
pid.mode = Mode.AUTO

# Cascade mode - uses remote setpoint
pid.mode = Mode.CASCADE
pid.remote_setpoint = 55.0
```

## License

MIT License. Based on [simple-pid](https://github.com/m-lundberg/simple-pid) by Martin Lundberg.
