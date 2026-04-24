# ksp_bridge

ROS2 package for Kerbal Space Program based on the kRPC mod.

Works with:  

+ KSP 1.12.5

+ kRPC 0.5.2

+ ROS2 Humble

+ Ubuntu 22.04

## Installation

### Install kRPC mod

Install the [kRPC mod](https://github.com/nullprofile/krpc) and it's dependencies.

### Clone repository and build the docker container

``` bash
git clone https://github.com/clausqr/ksp_bridge
cd ksp_bridge
docker build -t ksp_bridge:humble .
```

## Usage

### 1. Server settings inside KSP

![up_and_down_assembly](doc/img/server_settings.jpg)

![Arianne 5 landing with ksp_bridge connected](doc/img/ksp-arianne-5-landing.png)

**Note**: *Max. time per update* is required to be high, but it affects the framerate.

### 2. Running the docker container

``` bash
xhost +local:docker
docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ksp_bridge:humble
```

### 3. inside the docker container

Resource monitor example:
``` bash
ros2 launch ksp_bridge_examples resource_monitor.launch.py
```

or 

Up and Down Launch example:
``` bash
ros2 launch ksp_bridge_examples up_and_down.launch.py
``` 

### 4. Development

(for fast reference, your workflow may vary)

Mount the repository inside the docker container:
``` bash
docker run -it --rm --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd):/ws/ksp_bridge ksp_bridge:humble
```

Code your custom packets outside the container and build and test inside the container with:

``` bash
cd /ws/ksp_bridge
colcon build
source install/setup.bash
rqt &
ros2 run ksp_bridge_<your_custom_packet> <your_custom_launch_file>.launch.py
```

## Contributing

This repository uses a **gitflow** branching model:

- `main` — release branch. Tagged versions ship from here. Never commit directly.
- `develop` — integration branch. All feature work merges here first.
- `feature/<slug>` — one branch per feature or work package, branched from `develop`, PR'd back to `develop`.
- `fix/<slug>` — same pattern for non-release bug fixes.
- `release/<version>` — short-lived branch cut from `develop` when stabilising a release; merges into both `main` and `develop`.
- `hotfix/<slug>` — branched from `main`, merges into both `main` and `develop`.

**Workflow:**

```bash
git switch develop && git pull
git switch -c feature/my-thing
# ... work, commit ...
git push -u origin feature/my-thing
gh pr create --base develop
```

For parallel work on multiple features, use git worktrees so each branch has its own checkout:

```bash
git worktree add ../ksp_bridge.wt/my-thing feature/my-thing
```

Keep worktrees outside the `ros2_ws/src/` tree so `colcon` does not double-build the package.

