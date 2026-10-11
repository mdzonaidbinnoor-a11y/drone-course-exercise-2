# Installation and Prerequisites

First, clone this repository and initialize the workspace:

```bash
cd ~
git clone https://github.com/cvar-upm-drone-team/project_cvar_upm_drone_course.git
```

You can choose between two setups:

- [Native Ubuntu + ROS 2 Humble](#native)
- [Docker](#docker)

Then, you can test the installation by following the instructions in the [Running the Course Environment](#run-the-course-environment) section below.

## Native

Video tutorial with Aerostack2 by source installation:

[![Installation with Aerostack2](resources/video_control_installation_source_w_as2.jpg)](https://youtu.be/WP4XgzKsdxE)

Video tutorial using `rosdep` for dependencies installation:

[![Installation using rosdep](resources/video_control_installation_source_w_rosdep.jpg)](https://youtu.be/jkoXc7SVJuk)

### 1. Prerequisites

- You need Ubuntu 22.04 with ROS 2 Humble installed: [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- You need also `tmux` and `tmuxinator` available:
```bash
sudo apt update
sudo apt install -y tmux tmuxinator
```

_Optional: You can also install `Aerostack2` from source following the instructions in the [Aerostack2 documentation](https://aerostack2.github.io/)._


### 2. Building the Workspace

1. Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

2. Install workspace dependencies:

```bash
cd ~/project_cvar_upm_drone_course/control
cd drone_course_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

3. Build workspace:

```bash
cd ~/project_cvar_upm_drone_course/control
cd drone_course_ws
colcon build --symlink-install
source install/setup.bash
```

4. (Optional) Add sourcing to `.bashrc`:

```bash
echo "source ~/project_cvar_upm_drone_course/control/drone_course_ws/install/setup.bash" >> ~/.bashrc
```

## Docker

Video tutorial:

[![Installation using docker](resources/video_control_installation_docker.jpg)](https://youtu.be/tPer5hfMhLU)

### 1. Prerequisites

- You need Docker installed: [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/).
- You need `docker-compose` installed: [Docker Compose Installation Guide](https://docs.docker.com/compose/install/) (if not included with Docker).


### 2. Building and Running the Container

1. Navigate to the `docker` directory:

```bash
cd ~/project_cvar_upm_drone_course/control
```

2. Build the Docker image:

```bash
docker compose build
```

3. Start the container:

```bash
docker compose up -d
```

4. Connect to the container using `docker exec` in new terminal:

```bash
docker exec -it project_cvar_upm_drone_course_control bash
```

5. Build the workspace inside the container:

```bash
cd ~/project_cvar_upm_drone_course/control
cd drone_course_ws
colcon build --symlink-install
source install/setup.bash
```

_Note: You can use [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/) to connect to the container's desktop environment. The default VNC port is `5901`._

_Note: You may use `xhost +` to allow the container to access your host's display._

## Run the Course Environment

Go to the control directory:

```bash
cd ~/project_cvar_upm_drone_course/control
```

### Launch simulation

```bash
./launch_as2.bash
```

Platform options:

- `-p ms`: Multirotor Simulator (default)
- `-p gz`: Gazebo

Examples:

```bash
./launch_as2.bash -p ms
./launch_as2.bash -p gz
```

A window like this should appear:

<img src="resources/rviz_view.png" alt="RViz simulation view" width="500" />


## Stop simulation
Launch stop script in tmuxinator session or in a new terminal:

```bash
./stop.bash
```
