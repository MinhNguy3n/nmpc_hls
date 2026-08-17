# XPso_fsm Driver Test Suite

## Overview

This test suite validates the **PSO FSM (Particle Swarm Optimization Finite State Machine)** driver functions for the Sniff quadrotor drone. The PSO FSM is an FPGA-based optimization core that computes optimal control inputs based on the current drone state and reference trajectory.

### Key Features

- **7 comprehensive test cases** covering all driver functions
- **Modeled after MATLAB `test_model_cpp.m`** for consistency
- **Hardware-aware** with graceful fallback for software-only testing
- **Multi-step simulation** showing FSM integration in control loops
- **Detailed logging** for debugging and validation

---

## Test Structure

### Test 1: Device Initialization and Status Checks
- **Functions tested:** `XPso_fsm_Initialize()`, `XPso_fsm_IsIdle()`, `XPso_fsm_IsReady()`
- **Purpose:** Verify driver can initialize the FPGA PSO FSM device
- **Notes:**
  - Runs on Linux UIO (Userspace I/O) interface
  - Gracefully handles missing hardware (for simulation/development)

### Test 2: Parameter Setting and Getting
- **Functions tested:**
  - `XPso_fsm_Set_u_curr()` / `XPso_fsm_Get_u_curr()` - Control input
  - `XPso_fsm_Set_x_curr()` / `XPso_fsm_Get_x_curr()` - Current state
  - `XPso_fsm_Set_xref()` / `XPso_fsm_Get_xref()` - Reference state
- **Purpose:** Verify register read/write operations
- **Data Flow:** Host ↔ FPGA registers

### Test 3: Fitness and Optimization Parameters
- **Functions tested:**
  - `XPso_fsm_Set_last_best()` / `XPso_fsm_Get_last_best()`
  - `XPso_fsm_Set_new_best()` / `XPso_fsm_Get_new_best()`
  - `XPso_fsm_Set_bestfitness_top()` / `XPso_fsm_Get_bestfitness_top()`
- **Purpose:** Verify fitness value tracking for PSO iterations
- **PSO Context:** These values guide particle swarm movement

### Test 4: Control Flow (Start, Wait, Done)
- **Functions tested:**
  - `XPso_fsm_Start()` - Initiate computation
  - `XPso_fsm_IsDone()` - Poll completion status
  - `XPso_fsm_IsIdle()` - Check idle state
  - `XPso_fsm_Get_return()` - Get computation result
- **Purpose:** Validate FSM execution flow and status signaling
- **Timing:** Includes timeout handling and iteration counting

### Test 5: Multi-Step Simulation Loop
- **Purpose:** Simulate drone control loop similar to MATLAB `test_model_cpp.m`
- **Scenario:**
  - Initial state: hover at position (7, 10, 1) m
  - Reference state: (10, 12, 2) m
  - 10 time steps with Ts = 0.1 s
  - Proportional control law for trajectory tracking
- **Output:** Position tracking over time
- **Works:** With or without hardware (uses simple proportional control if no FSM)

### Test 6: Interrupt Management (Optional)
- **Functions tested:**
  - `XPso_fsm_InterruptGlobalEnable()` / `XPso_fsm_InterruptGlobalDisable()`
  - `XPso_fsm_InterruptEnable()` / `XPso_fsm_InterruptDisable()`
  - `XPso_fsm_InterruptGetStatus()`
- **Purpose:** Validate interrupt control for asynchronous operation

### Test 7: Auto-Restart Mode
- **Functions tested:**
  - `XPso_fsm_EnableAutoRestart()` / `XPso_fsm_DisableAutoRestart()`
- **Purpose:** Test continuous operation mode for real-time control

---

## Comparison with MATLAB Test (`test_model_cpp.m`)

### MATLAB Test Structure
```matlab
% Parameters
drone = struct with I, l, kf, kM, m, g, b

% Data loading
load ../data_line.mat  % Loads xHistory, uHistory, time, xRef_out

% Simulation loop
for i=1:size(uHistory,1)
    state_dot = drone_model(xHistory(i,:), uHistory(i,:))
    xHistory(i+1,:) = xHistory(i,:) + state_dot * Ts
end

% Visualization
animateQuadrotor(...)
```

### C Test Structure (Parallel Implementation)
```c
// Parameters
DroneParams = {Ixx, Iyy, Izz, l, kf, kM, m, g, b, Ts}

// Initial conditions
DroneState = {x, y, z, phi, theta, psi, xdot, ydot, zdot, phidot, thetadot, psidot}

// Simulation loop
for (step = 0; step < num_steps; step++) {
    // Compute error to reference
    error = reference - current

    // Proportional control (FSM would optimize this)
    control = proportional_feedback(error)

    // State integration
    state = state + state_dot * Ts
}

// Test output
print_results()
```

---

## Building and Running

### Quick Start

```bash
# Navigate to test directory
cd /home/minh/nmpc_hls/app/orb_extract_platform/hw/sdt/drivers/pso_fsm_v1_0/src

# Make script executable (first time only)
chmod +x build_and_test.sh

# Build and run all tests
./build_and_test.sh all

# Or just build
./build_and_test.sh build

# Or just run (if already built)
./build_and_test.sh run

# Clean build artifacts
./build_and_test.sh clean
```

### Using Make Directly

```bash
# Build test executable
make -f Makefile.test build

# Run tests
make -f Makefile.test run

# Clean
make -f Makefile.test clean

# Show help
make -f Makefile.test help
```

### Build Output

- **Executable:** `./build/bin/test_xpso_fsm`
- **Library:** `./build/lib/libpso_fsm.a`
- **Objects:** `./build/*.o`

---

## Expected Output

### With Hardware (FPGA PSO FSM Available)

```
╔═══════════════════════════════════════════════════════════╗
║        XPSO_FSM Driver Test Suite                         ║
║        PSO-Based Control Optimization for Sniff Drone     ║
╚═══════════════════════════════════════════════════════════╝

═══════════════════════════════════════════════════════════
TEST 1: Device Initialization and Status Checks
═══════════════════════════════════════════════════════════
Initializing XPso_fsm device...
[PASS] Device initialized successfully
Device is idle: YES
Device is ready: YES

[... additional tests ...]

═══════════════════════════════════════════════════════════
TEST 5: Multi-Step Simulation Loop
═══════════════════════════════════════════════════════════
Step      x[m]        y[m]        z[m]        err_norm
  0       7.0000      10.0000     1.0000      1.8481
  1       7.0000      10.0000     1.0000      1.8481
  ...
```

### Without Hardware (Software-Only Mode)

```
═══════════════════════════════════════════════════════════
TEST 1: Device Initialization and Status Checks
═══════════════════════════════════════════════════════════
Initializing XPso_fsm device...
[WARN] Device initialization failed
       This may be expected if hardware is not available.
       Continuing with other tests...

[INFO] Proceeding with software-only tests
       (Hardware tests require FPGA with PSO FSM IP core)

═══════════════════════════════════════════════════════════
TEST 5: Multi-Step Simulation Loop
═══════════════════════════════════════════════════════════
[... simulation results ...]
```

---

## Drone Model Parameters

Defined in both `test_model_cpp.m` and `test_xpso_fsm.c`:

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Inertia (XX) | Ixx | 1.2 | kg⋅m² |
| Inertia (YY) | Iyy | 1.2 | kg⋅m² |
| Inertia (ZZ) | Izz | 2.3 | kg⋅m² |
| Arm Length | l | 0.25 | m |
| Thrust Coefficient | kf | 1.0 | Ns² |
| Drag Coefficient | kM | 0.2 | Nms² |
| Mass | m | 2.0 | kg |
| Gravity | g | 9.81 | m/s² |
| Speed Offset | b | 4.9050 | rad²/s² |
| Sampling Time | Ts | 0.1 | s |

---

## Drone State Vector

```
x[0:2]   = Position (x, y, z) [m]
x[3:5]   = Attitude angles (phi, theta, psi) [rad]
x[6:8]   = Linear velocities (xdot, ydot, zdot) [m/s]
x[9:11]  = Angular velocities (phidot, thetadot, psidot) [rad/s]
```

---

## Control Input Vector

```
u[0] = Thrust (T)           [N]
u[1] = Roll moment (tx)     [Nm]
u[2] = Pitch moment (ty)    [Nm]
u[3] = Yaw moment (tz)      [Nm]
```

---

## Hardware Requirements

To run full hardware tests:

1. **FPGA with PSO FSM IP Core**
   - Xilinx Zynq or Zynq UltraScale+
   - PSO FSM IP generated by Vitis HLS
   - Bitstream programmed to FPGA

2. **Device Tree / UIO Driver**
   - PSO FSM device exposed in `/sys/class/uio/`
   - Readable/writable memory-mapped registers

3. **Linux System**
   - Kernel compiled with CONFIG_UIO=y
   - Standard C library (glibc)

### Checking Hardware Availability

```bash
# List UIO devices
ls -la /sys/class/uio/

# Check for PSO FSM device
cat /sys/class/uio/uio0/name  # Should contain "pso_fsm"

# Check UIO device memory maps
cat /sys/class/uio/uio0/maps/map0/addr
cat /sys/class/uio/uio0/maps/map0/size
```

---

## File Organization

```
pso_fsm_v1_0/src/
├── xpso_fsm.h              (Driver header - register definitions)
├── xpso_fsm.c              (Driver functions - register R/W)
├── xpso_fsm_hw.h           (Hardware address constants)
├── xpso_fsm_linux.c        (Linux UIO interface)
├── xpso_fsm_sinit.c        (Static initialization)
├── test_xpso_fsm.c         (Test suite - THIS FILE)
├── Makefile.test           (Build configuration)
└── build_and_test.sh       (Build and run script)
```

---

## Extending the Tests

### Adding New Test Cases

1. Create a new function: `int test_new_feature(XPso_fsm* device)`
2. Add to main() in appropriate section
3. Follow existing pattern:
   ```c
   printf("\n" "="*70 "\n");
   printf("TEST N: Description\n");
   printf("="*70 "\n");

   if (!device) {
       printf("[INFO] Skipping (device not available)\n");
       return 0;
   }

   // Test code here
   return 0;
   ```

### Running Specific Tests

Modify `main()` to selectively enable/disable tests:

```c
test_device_initialization();
// Comment out tests you don't want:
// test_parameter_operations();
test_simulation_loop(NULL);  // Software-only test
```

---

## Troubleshooting

### Build Errors

**Error:** `xpso_fsm.h: No such file or directory`
- **Solution:** Ensure you're in the correct directory with test file

**Error:** `undefined reference to XPso_fsm_*`
- **Solution:** Verify driver source files are being compiled (check Makefile.test)

### Runtime Errors

**Error:** `Device initialization failed: -1`
- **Cause:** UIO device not available
- **Solution:**
  - Check if FPGA is programmed
  - Verify kernel has UIO support
  - Run `ls /sys/class/uio/` to see available devices

**Error:** `Computation timeout`
- **Cause:** FSM taking too long or stuck
- **Solution:**
  - Increase timeout value
  - Check FPGA clock frequency
  - Verify bitstream integrity

---

## References

### Related Files
- MATLAB Test: `/home/minh/nmpc_hls/app/matlab_ws/model_cpp/test_model_cpp.m`
- Drone Model C: `/home/minh/nmpc_hls/app/drone_simulation/src/drone_model.c`
- Drone Model MATLAB: `/home/minh/nmpc_hls/app/matlab_ws/model_cpp/model_drone.m`

### Documentation
- Vitis HLS Driver Generation
- Xilinx UIO Framework
- PSO Algorithm for Nonlinear MPC

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-04-29 | Initial test suite creation |

---

## License

This test suite is part of the NMPC HLS project.
See LICENSE file in repository root for details.

---

## Contact & Support

For issues or questions about this test suite:
1. Check the troubleshooting section above
2. Review hardware requirements
3. Verify file structure and dependencies
4. Check FPGA bitstream and UIO driver status

---
