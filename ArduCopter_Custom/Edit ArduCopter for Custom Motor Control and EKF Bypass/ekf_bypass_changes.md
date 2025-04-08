# Changes Required to Bypass EKF Checks in guided_nogps Mode

Based on the analysis of the ArduPilot codebase, here are the specific changes needed to bypass EKF checks in guided_nogps mode:

## 1. Modify `ekf_check.cpp`

The main EKF failsafe mechanism is implemented in `ekf_check.cpp`. To bypass these checks, we need to modify the `ekf_check()` function:

```cpp
// File: /ArduCopter/ekf_check.cpp
void Copter::ekf_check()
{
    // Add a bypass for guided_nogps mode
    if (flightmode->mode_number() == Mode::Number::GUIDED_NOGPS) {
        // Clear any existing EKF failsafe
        ekf_check_state.fail_count = 0;
        ekf_check_state.bad_variance = false;
        AP_Notify::flags.ekf_bad = ekf_check_state.bad_variance;
        failsafe_ekf_off_event();
        return;
    }

    // Rest of the original function remains unchanged
    // ...
}
```

## 2. Modify `AP_Arming_Copter.h` and its implementation

The pre-arm checks in `AP_Arming_Copter.h` include EKF attitude checks. We need to modify the `pre_arm_ekf_attitude_check()` function in the implementation file:

```cpp
// File: /ArduCopter/AP_Arming_Copter.cpp
bool AP_Arming_Copter::pre_arm_ekf_attitude_check()
{
    // If we're planning to use guided_nogps, bypass this check
    if (copter.flightmode->mode_number() == Mode::Number::GUIDED_NOGPS) {
        return true;
    }

    // Rest of the original function remains unchanged
    // ...
}
```

## 3. Modify `failsafe.cpp`

To ensure that EKF failsafe doesn't trigger during guided_nogps mode:

```cpp
// File: /ArduCopter/failsafe.cpp
// Add this check to the failsafe_ekf_event() function
void Copter::failsafe_ekf_event()
{
    // Skip EKF failsafe in guided_nogps mode
    if (flightmode->mode_number() == Mode::Number::GUIDED_NOGPS) {
        return;
    }

    // Rest of the original function remains unchanged
    // ...
}
```

## 4. Modify `mode_guided_nogps.cpp`

Update the guided_nogps mode to explicitly disable EKF checks:

```cpp
// File: /ArduCopter/mode_guided_nogps.cpp
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // Always ignore checks in guided_nogps mode
    ignore_checks = true;
    
    // start in angle control mode
    ModeGuided::angle_control_start();
    return true;
}
```

## 5. Modify `Copter.h`

You may need to add a flag to indicate that EKF checks should be bypassed:

```cpp
// File: /ArduCopter/Copter.h
// Add to the appropriate section, such as the AP_State structure
struct {
    // existing flags...
    bool bypass_ekf_checks;   // true if EKF checks should be bypassed
} ap;
```

Then set this flag in the guided_nogps mode initialization.
