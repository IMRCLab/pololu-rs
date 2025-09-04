# Ground Robots ROS Workspace

This repository contains a ROS 2 workspace for ground robots.  
Some packages (e.g. [`crazyswarm2`](https://github.com/IMRCLab/crazyswarm2) and [`motion_capture_tracking`](https://github.com/IMRCLab/motion_capture_tracking)) are included as **git submodules**.

---

## How to Clone

Since this repository uses **submodules**, you must clone it recursively:

```bash
git clone --recurse-submodules -b ground_robots_ros <REPO_URL>
cd ground_robots_ws
```

If you already cloned without `--recurse-submodules`, run:
```bash
git submodule update --init --recursive
```

## How to Build
From the workspace root:
```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
Source the environment before running any nodes:
```bash
source install/setup.bash
```

## Notes on Submodules

- Submodules are pinned to specific commits to ensure reproducibility.
- If you need to update a submodule to a newer commit:

```bash
cd src/crazyswarm2
git checkout <new_commit_or_branch>
git pull
cd ../../
git add src/crazyswarm2
git commit -m "Update crazyswarm2 submodule"
```

Do the same for `motion_capture_tracking` if needed.