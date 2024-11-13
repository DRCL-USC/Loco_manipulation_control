## Quadruped_Control
This software package is designed for Unitree robots, including A1, AlienGo, and Go1. Note that slight modifications may be necessary for compatibility with other Unitree robots.

## Code Structure of the quadruped_control Package
- **Common**: Contains the leg controller, state estimation, and swing foot trajectory generator.
- **BalanceController**: Features a QP-based balance controller as described by [Focchi et al., 2017](https://iit-dlslab.github.io/papers/focchi2016.pdf).
- **ConvexMPC**: Implements MPC for both locomotion and loco-manipulation.
- **FSM**: Includes the finite state machine and FSM states for mode switching (passive, PD-stand, QP-stand, walking, and loco-manipulation).
