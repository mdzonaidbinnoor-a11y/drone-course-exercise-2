# Exercise 3: Smooth Trajectory Speed Control Through Gates

## Introduction

In this exercise, you will build a controller to fly through all gates using speed commands and a smooth trajectory.

Main objective:

1. Use `SPEED` control mode.
2. Generate a smooth trajectory that crosses all gates in order.
3. Track that trajectory with velocity commands until the circuit is completed.

<img src="../docs/resources/rviz_view.png" alt="RViz simulation view" width="500" />

Simulation view.

<img src="../docs/resources/alphanumeric.png" alt="Simulation data view" width="500" />

Simulation data view (e.g., control mode and drone state).

## Guide for Section 3.1: Build Your Own Node

Objective: implement a speed-control over a smooth trajectory workflow in your own node.
Code goal: complete the circuit by passing through all gates with a smooth trajectory and speed control.

Reference note: you can use `control/exercises/Exercise2.md` and `control/python_interface/exercise_2/exercise_2.py` as an example and starting point.

### Step 1. Launch the simulation

Start the simulation environment:

```bash
./launch_as2.bash
```

### Step 2. Set speed control mode

Your node must call `/drone0/controller/set_control_mode` and configure:

- `control_mode = SPEED`
- `yaw_mode = YAW_SPEED`
- `reference_frame = LOCAL_ENU_FRAME`

### Step 3. Get the gate sequence

Get the gate positions from the provided path service and build an ordered list of waypoints to cross all gates.

### Step 4. Generate a smooth trajectory

Create a continuous trajectory from the gate waypoints. You can use any valid method, for example:

- pure pursuit
- cubic or quintic interpolation,
- spline-based trajectory,
- waypoint smoothing with heading continuity.

Requirements:

- no sharp direction changes at gates,
- bounded speed and acceleration,
- stable behavior near corners.

### Step 5. Track the trajectory with speed commands

Publish `geometry_msgs/msg/TwistStamped` to `/drone0/motion_reference/twist`.

Minimum control requirements:

1. Closed-loop tracking based on current drone state.
2. Error reduction along the full path (not only at isolated waypoints).
3. Reliable gate crossing in sequence.

## Guide for Section 3.2: Validation

You must validate that the trajectory is smooth and the circuit is completed:

1. Show the drone crossing all gates.
2. Show control/state information during the run.
3. Show that the drone remains stable (no oscillatory or abrupt behavior).

## Running Your Implementation

Launch simulation:

```bash
./launch_as2.bash
```

Run your node with your chosen package and executable.

## Submission

Submit your work in **one** of these formats:

- A link to a GitHub repository containing this repository with your exercise solution.
- A `.zip` file of this repository including your exercise solution.

Do not submit individual source files separately.

Also include:

- A short description of your trajectory-generation method.
- A video showing the full circuit with smooth gate crossing.
