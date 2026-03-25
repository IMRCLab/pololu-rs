# Dependencies

This directory contains external dependencies as git submodules.

## wmr-simulator

The `wmr-simulator` provides the controller and estimator implementations:
- **Repository**: https://github.com/IMRCLab/wmr-simulator
- **Used by**: `wmr_controller_node.py`
- **Components**:
  - `controller.py` - Differential drive controller with pose control + wheel speed control
  - `estimator.py` - Dead reckoning and Kalman filter estimator

## Setup

After cloning this repository, initialize the submodules:

```bash
cd /home/charlotte/IMRCLab/worksation_ros
git submodule update --init --recursive
```

