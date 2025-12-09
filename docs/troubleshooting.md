# Troubleshooting
<!-- Add like the acutal debugging protocoll-->
A collection of errors we encountered and how we fixed them.

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## General Troubleshooting Tips

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Various Errors we encouterd 

### Probe Rs being unable to connect to the probe

When trying to connect to the probe we received this error:

        Error: Failed to open probe: Failed to open the debug probe.

        Caused by:
            0: The debug probe could not be created.
            1: An error specific with the selected probe occurred.
            2: The firmware on the probe is outdated, and not supported by probe-rs. The minimum supported firmware version is 2.2.0.
This is caused by too old probe firmware, you can find the updated probe firmware [here](https://github.com/raspberrypi/debugprobe/releases/tag/debugprobe-v2.2.3)

### Computer and robot state getting mismatched

As of now there is only a one way communication implemented. This can lead to a mismatch in the robot state and the state displayed on the computer (e.g., in the "Menu" functionality).

To fix this you can restart the process.




### Motor Directional Inversion
- In case of self assembly motors might get mixed up
- Zumo motors may run in **reversed direction** (requires negated duty cycle)  
- 3pi motors are **normal direction**
- Needs to be checked before use.
