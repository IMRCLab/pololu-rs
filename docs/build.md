# Pololu 3Pi/Zumo Robot Build System Documentation

This document provides brief documentation for building and running the [Pololu 3Pi+ 2040 robot firmware](https://github.com/IMRCLab/pololu3pi2040-rs) with multiple robot support and different firmware tasks. It covers the `./run` build script usage, feature flag system, and multi-robot configuration workflow.

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Quick Start

---

### Hardware Preperation
* You will need a Raspberry Pi Debug Probe(for debugging) and a USB-C Cabel(for flashing).
* connect the probe properly to the Pololu. If the official probe is used, the connection should be:
  
        Yellow(SWDIO) -> SWDIO
        Green(SWCLK)  -> SWCLK
        Black(GND)    -> GND

    Then connect the Pololu with the USB-C cabel to your PC, and the debug probe as well and uncomment in `config.toml`
        
        runner = "probe-rs run --chip RP2040 --protocol swd"

---

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

---

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

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Complete `./run` Script Reference

---

### Script Syntax
```bash
./run [robot_type] [target]
```

**Parameters:**
- `robot_type` (optional): `zumo`, `3pi`, or empty for default
- `target` (optional): `teleop`, `trajectory`, `build`, or empty for main binary

---

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

---

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

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Technical Implementation

The build system uses Rust's conditional compilation with feature flags to select robot-specific modules at compile time.

---

### Feature Flag System

1. **Feature Selection**: Robot type determined by `--features zumo` or `--features 3pi`
2. **Module Selection**: `lib.rs` conditionally compiles the appropriate joystick control module
3. **Re-export**: Selected module is re-exported as `joystick_control` for transparent usage
4. **Compilation**: Main code imports from `joystick_control` without knowing the underlying implementation

---

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

---

### Key Differences Between Robot Modules

| Aspect                 | Zumo                                       | 3Pi                                      | Default               |
| ---------------------- | ------------------------------------------ | ---------------------------------------- | --------------------- |
| **Module File**        | `joystick_control_zumo.rs`                 | `joystick_control_3pi.rs`                | `joystick_control.rs` |
| **Motor Control**      | `motor.set_speed(-duty_left, -duty_right)` | `motor.set_speed(duty_left, duty_right)` | Zumo parameters       |
| **Encoder CPR**        | 909.72                                     | 358.32                                   | 909.72                |
| **Physical Constants** | Zumo values                                | 3Pi values                               | Zumo values           |

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Development Workflows

---

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

---

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

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Troubleshooting

---

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

---

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

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Notes

- **Release Mode**: The `./run` script always builds in `--release` mode for optimized embedded code
- **Backward Compatibility**: Default configuration maintains compatibility with existing code
- **Module Interface**: All robot configurations expose the same public interface for seamless switching
- **Testing**: Use default configuration (`./run`) for rapid development and testing
- **Production**: Use specific robot features (`./run zumo` or `./run 3pi`) for deployment
- **Version Conflict**: The newest version `1.89.0` of `rustc` is released on 4th Aug 2025. However, the flashing with `elf2uf2-rs -d` might suffers some temporary issues. For example, an error `unregonized ABI` occurs because of the generated elf header doesn't match the requirements of the elf2uf2 runner. (The generated 8th bit of the header is `03`, which indicates that the OS/ABI type is `UNIX - GNU`, but actually should be `00`, which indicates `UNIX - System V`). There are 2 ways to solve this issue:

    - If you really need the newest rustc, then each time after you build the project you should enter you target folder and do:

            printf '\x00' | dd of=teleop_control bs=1 seek=7 count=1 conv=notrunc

        This will change the 8th bit from `03` to `00` and then you can flash this as usual (don't build it again).

    - The other way is to downgrade rustc to `1.87.0` by using:
            
            rustup toolchain install 1.87.0
            rustup default 1.87.0
            rustup component add rust-src --toolchain 1.87.0
            rustup target add thumbv6m-none-eabi --toolchain 1.87.0
