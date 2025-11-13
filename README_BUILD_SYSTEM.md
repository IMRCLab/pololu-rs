# Pololu 3Pi/Zumo Robot Build System Documentation

This document provides brief documentation for building and running the [Pololu 3Pi+ 2040 robot firmware](README.md) with multiple robot support and different firmware tasks. It covers the `./run` build script usage, feature flag system, and multi-robot configuration workflow.

> **📋 For complete project information**: See [README.md](README.md) for hardware details, project structure, and comprehensive documentation.

## 🚀 Quick Startlolu 3Pi/Zumo Robot Build System Documentation


## � Quick Start

### Using the `./run` Script (Recommended)

The `./run` script provides a convenient way to build and flash different robot configurations.

```bash
# Make executable (first time only)
chmod +x run

# Examples:
./run                             # Default config, main binary
./run zumo                        # Zumo config, main binary
./run zumo teleop                 # Zumo config, teleop_control binary
./run 3pi menu                    # 3Pi config, menu binary
./run 3pi trajectory_following    # 3Pi config, trajectory_following binary
```

### Using Cargo Directly

```bash
# Default configuration
cargo run --release

# Zumo robot
cargo run --release --features zumo

# 3Pi robot  
cargo run --release --features 3pi

# Specific binaries
cargo run --release --bin programm_entrance --features 3pi
cargo run --release --bin teleop_control --features zumo
cargo run --release --bin trajectory_following --features 3pi
```

## � Complete `./run` Script Reference

### Script Syntax
```bash
./run [robot_type] [target]
```

**Parameters:**
- `robot_type` (optional): `zumo`, `3pi`, or empty for default
- `target` (optional): `teleop`, `trajectory`, `build`, or empty for main binary

### All Available Commands

| Command                           | Robot Config | Binary               | Description                              |
| --------------------------------- | ------------ | -------------------- | ---------------------------------------- |
| `./run`                           | Default      | Main                 | Default joystick_control.rs, main binary |
| `./run zumo`                      | Zumo         | Main                 | Zumo-specific config, main binary        |
| `./run 3pi`                       | 3Pi          | Main                 | 3Pi-specific config, main binary         |
| `./run zumo menu`                 | Zumo         | programm_entrance    | Zumo config, menu binary                 |
| `./run 3pi menu`                  | 3Pi          | programm_entrance    | 3Pi config, menu binary                  |
| `./run zumo teleop`               | Zumo         | teleop_control       | Zumo config, teleop binary               |
| `./run 3pi teleop`                | 3Pi          | teleop_control       | 3Pi config, teleop binary                |
| `./run zumo trajectory_following` | Zumo         | trajectory_following | Zumo config, trajectory binary           |
| `./run 3pi trajectory_following`  | 3Pi          | trajectory_following | 3Pi config, trajectory binary            |
| `./run zumo build`                | Zumo         | -                    | Build only (no flash)                    |
| `./run 3pi build`                 | 3Pi          | -                    | Build only (no flash)                    |

### Equivalent Cargo Commands

| `./run` Command                  | Equivalent Cargo Command                                        |
| -------------------------------- | --------------------------------------------------------------- |
| `./run`                          | `cargo run --release`                                           |
| `./run zumo`                     | `cargo run --release --features zumo`                           |
| `./run 3pi`                      | `cargo run --release --features 3pi`                            |
| `./run 3pi menu`                 | `cargo run --release --bin programm_entrance --features 3pi`    |
| `./run zumo teleop`              | `cargo run --release --bin teleop_control --features zumo`      |
| `./run 3pi trajectory_following` | `cargo run --release --bin trajectory_following --features 3pi` |
| `./run zumo build`               | `cargo build --release --features zumo`                         |

## 🔍 Technical Implementation

The build system uses Rust's conditional compilation with feature flags to select robot-specific modules at compile time.

### Feature Flag System

1. **Feature Selection**: Robot type determined by `--features zumo` or `--features 3pi`
2. **Module Selection**: `lib.rs` conditionally compiles the appropriate joystick control module
3. **Re-export**: Selected module is re-exported as `joystick_control` for transparent usage
4. **Compilation**: Main code imports from `joystick_control` without knowing the underlying implementation

### Code Structure

#### `lib.rs` Conditional Compilation
```rust
// Default joystick control (for testing)
#[cfg(not(any(feature = "zumo", feature = "3pi")))]
pub mod joystick_control;

// Robot-specific modules
#[cfg(feature = "zumo")]
pub mod joystick_control_zumo;
#[cfg(feature = "zumo")]
pub use joystick_control_zumo as joystick_control;

#[cfg(feature = "3pi")]
pub mod joystick_control_3pi;
#[cfg(feature = "3pi")]
pub use joystick_control_3pi as joystick_control;
```

#### `Cargo.toml` Feature Definition
```toml
[features]
default = []    # No default robot (uses joystick_control.rs)
zumo = []       # Empty feature flag for Zumo
3pi = []        # Empty feature flag for 3Pi
```

### Key Differences Between Robot Modules

| Aspect                 | Zumo                                       | 3Pi                                      | Default               |
| ---------------------- | ------------------------------------------ | ---------------------------------------- | --------------------- |
| **Module File**        | `joystick_control_zumo.rs`                 | `joystick_control_3pi.rs`                | `joystick_control.rs` |
| **Motor Control**      | `motor.set_speed(-duty_left, -duty_right)` | `motor.set_speed(duty_left, duty_right)` | Zumo parameters       |
| **Encoder CPR**        | 909.72                                     | 358.32                                   | 909.72                |
| **Physical Constants** | Zumo values                                | 3Pi values                               | Zumo values           |

## 🛠️ Development Workflows

### Adding a New Robot Configuration

1. Create new joystick control module (e.g., `joystick_control_newrobot.rs`)
2. Add feature flag in `Cargo.toml`:
   ```toml
   [features]
   newrobot = []
   ```
3. Update `lib.rs`:
   ```rust
   #[cfg(feature = "newrobot")]
   pub mod joystick_control_newrobot;
   #[cfg(feature = "newrobot")]
   pub use joystick_control_newrobot as joystick_control;
   ```
4. Update `./run` script validation logic

### Testing Different Configurations

```bash
# Verify all configurations compile
cargo check                    # Default
cargo check --features zumo    # Zumo
cargo check --features 3pi     # 3Pi

# Build without flashing
./run zumo build
./run 3pi build
```

## 🚨 Troubleshooting

### Common Issues & Solutions

**Script not executable:**
```bash
chmod +x run
```

**Invalid robot type error:**
```
Error: Invalid robot type 'typo'. Use 'zumo', '3pi', or leave empty for default
```
**Solution**: Check spelling and use exactly `zumo` or `3pi`

**Invalid target error:**
```
Error: Invalid target 'typo'. Use 'teleop', 'trajectory', 'build', or leave empty for main binary
```
**Solution**: Use valid targets: `teleop`, `trajectory`, `build`, or leave empty

**Feature compilation errors:**
```bash
# Test individual configurations
cargo check                    # Default config
cargo check --features zumo    # Zumo config
cargo check --features 3pi     # 3Pi config
```

**Script execution issues:**
```bash
# Make sure you're in the project root
ls -la | grep run    # Should show the run script

# Check script permissions
ls -la run           # Should show executable permissions (x)
```

### Verification Commands

```bash
# Build verification (without flashing)
./run zumo build      # Should compile Zumo config
./run 3pi build       # Should compile 3Pi config
./run build          # Invalid - will show error

# Feature flag verification
cargo build --release --features zumo
cargo build --release --features 3pi
cargo build --release    # Default config
```

## 📝 Notes & Best Practices

- **Release Mode**: The `./run` script always builds in `--release` mode for optimized embedded code
- **Backward Compatibility**: Default configuration maintains compatibility with existing code
- **Module Interface**: All robot configurations expose the same public interface for seamless switching
- **Testing**: Use default configuration (`./run`) for rapid development and testing
- **Production**: Use specific robot features (`./run zumo` or `./run 3pi`) for deployment
