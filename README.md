# Control Track

This track contains the practical ROS 2 + Aerostack2 control exercises for the drone racing environment.
The progression is designed to move from direct command execution to full autonomous control over a gate circuit.

Before starting, follow the setup guide in [docs/INSTALLATION.md](docs/INSTALLATION.md).
For API links and message references, use [docs/REFERENCES.md](docs/REFERENCES.md).

## Exercises

### Exercise 1: Position Control Fundamentals

Statement: [exercises/Exercise1.md](exercises/Exercise1.md)

You start with the basic control workflow: setting position mode, sending pose references from CLI, and implementing the same behavior in your own node.
The goal is to understand the control interface and complete the gate circuit with position commands.

### Exercise 2: Speed Control and Controller Tuning

Statement: [exercises/Exercise2.md](exercises/Exercise2.md)

This exercise moves to speed mode and `TwistStamped` commands.
You implement closed-loop velocity control, then improve performance by tuning control gains so the drone can traverse the circuit more effectively.

### Exercise 3: Smooth Trajectory Through Gates

Statement: [exercises/Exercise3.md](exercises/Exercise3.md)

In this final control exercise, you build a smoother strategy to cross all gates using speed commands and trajectory planning ideas.
You should integrate stable tracking, consistent gate crossing, and clean overall flight behavior.

## Code and Workspace

Python templates are available at [python_interface/exercise_1/exercise_1.py](python_interface/exercise_1/exercise_1.py) and [python_interface/exercise_2/exercise_2.py](python_interface/exercise_2/exercise_2.py).
The ROS 2 workspace for this track is `drone_course_ws`.
Use [launch_as2.bash](launch_as2.bash) to start the environment and [stop.bash](stop.bash) to stop all sessions.

## Quick Start

From repo root:

```bash
cd control
./launch_as2.bash
```

Stop:

```bash
./stop.bash
```
