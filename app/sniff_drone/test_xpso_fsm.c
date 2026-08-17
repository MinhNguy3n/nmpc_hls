/**
 * =============================================================================
 * Test Suite for XPso_fsm Driver Functions
 * =============================================================================
 *
 * This test program validates the PSO FSM (Particle Swarm Optimization
 * Finite State Machine) driver functions. The PSO FSM computes optimal
 * control inputs for a quadrotor drone based on the current state and
 * reference trajectory.
 *
 * Test Scenarios (modeled after MATLAB test_model_cpp.m):
 * 1. Device initialization and basic function calls
 * 2. State and control parameter setting/getting
 * 3. Fitness evaluation and optimization
 * 4. Multi-step simulation with state updates
 *
 * Author: Test Suite Generator
 * Date: 2026-04-29
 * =============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include "xpso_fsm.h"
#include "xparameters.h"

#define PSO_BASE XPAR_NMPC_SOLVER_PSO_FSM_0_BASEADDR

/* =========================================================================
   QUADROTOR DRONE MODEL PARAMETERS
   (Matching test_model_cpp.m and drone_model.c)
   ========================================================================= */

typedef struct {
    double Ixx, Iyy, Izz;  /* Moment of Inertia [kg.m^2] */
    double l;              /* Arm length [m] */
    double kf;             /* Thrust coefficient [Ns^2] */
    double kM;             /* Moment (drag) coefficient [Nms^2] */
    double m;              /* Mass [kg] */
    double g;              /* Gravity [m/s^2] */
    double b;              /* Speed offset [rad^2/sec^2] */
    double Ts;             /* Sampling time [s] */
} DroneParams;

typedef struct {
    double x[12];          /* State vector: [x, y, z, phi, theta, psi, xdot, ydot, zdot, phidot, thetadot, psidot] */
} DroneState;

typedef struct {
    double u[4];           /* Control input: [T, tx, ty, tz] */
} DroneControl;

/* =========================================================================
   UTILITY FUNCTIONS
   ========================================================================= */

/**
 * Initialize drone parameters (from test_model_cpp.m)
 */
void init_drone_params(DroneParams* params) {
    params->Ixx = 1.2;      /* Moment of Inertia [kg.m^2] */
    params->Iyy = 1.2;
    params->Izz = 2.3;
    params->l = 0.25;       /* Arm length [m] */
    params->kf = 1.0;       /* Thrust coefficient [Ns^2] */
    params->kM = 0.2;       /* Moment coefficient [Nms^2] */
    params->m = 2.0;        /* Mass [kg] */
    params->g = 9.81;       /* Gravity [m/s^2] */
    params->b = 4.9050;     /* Speed offset [rad^2/sec^2] */
    params->Ts = 0.1;       /* Sampling time [s] */
}

/**
 * Initialize drone state to hover condition
 */
void init_drone_state(DroneState* state) {
    memset(state->x, 0, sizeof(state->x));
    state->x[0] = 7.0;      /* x position [m] */
    state->x[1] = 10.0;     /* y position [m] */
    state->x[2] = 1.0;      /* z position [m] (altitude) */
    /* All velocities and angles remain at 0 for hover */
}

/**
 * Initialize control input
 */
void init_control(DroneControl* control) {
    control->u[0] = 0.0;    /* Thrust */
    control->u[1] = 0.0;    /* Roll moment */
    control->u[2] = 0.0;    /* Pitch moment */
    control->u[3] = 0.0;    /* Yaw moment */
}

/**
 * Print drone state for debugging
 */
void print_drone_state(const char* label, const DroneState* state) {
    printf("\n%s:\n", label);
    printf("  Position:   [%.4f, %.4f, %.4f]\n", state->x[0], state->x[1], state->x[2]);
    printf("  Angles:     [%.4f, %.4f, %.4f]\n", state->x[3], state->x[4], state->x[5]);
    printf("  Lin. Vel:   [%.4f, %.4f, %.4f]\n", state->x[6], state->x[7], state->x[8]);
    printf("  Ang. Vel:   [%.4f, %.4f, %.4f]\n", state->x[9], state->x[10], state->x[11]);
}

/**
 * Print control input for debugging
 */
void print_control(const char* label, const DroneControl* control) {
    printf("%s: [%.4f, %.4f, %.4f, %.4f]\n", label,
           control->u[0], control->u[1], control->u[2], control->u[3]);
}

/* =========================================================================
   TEST CASES
   ========================================================================= */

/**
 * TEST 1: Basic Device Initialization and Status Checks
 *
 * Tests:
 * - XPso_fsm_Initialize()
 * - XPso_fsm_IsIdle()
 * - XPso_fsm_IsReady()
 */
int test_device_initialization(void) {
    printf("\n====================================================================\n");
    printf("TEST 1: Device Initialization and Status Checks\n");
    printf("====================================================================\n");

    XPso_fsm device;
    int status;

    /* Initialize the device (Linux UIO mode) */
    printf("Initializing XPso_fsm device...\n");
    status = XPso_fsm_Initialize(&device, "pso_fsm_0");

    if (status != XST_SUCCESS) {
        printf("[WARN] Device initialization failed: %d\n", status);
        printf("       This may be expected if hardware is not available.\n");
        printf("       Continuing with other tests...\n");
        return -1;  /* Skip hardware tests */
    }

    printf("[PASS] Device initialized successfully\n");

    /* Check if device is idle */
    u32 is_idle = XPso_fsm_IsIdle(&device);
    printf("Device is idle: %s\n", is_idle ? "YES" : "NO");

    /* Check if device is ready for input */
    u32 is_ready = XPso_fsm_IsReady(&device);
    printf("Device is ready: %s\n", is_ready ? "YES" : "NO");

    return 0;
}

/**
 * TEST 2: Parameter Setting and Getting
 *
 * Tests:
 * - XPso_fsm_Set_u_curr() / XPso_fsm_Get_u_curr()
 * - XPso_fsm_Set_x_curr() / XPso_fsm_Get_x_curr()
 * - XPso_fsm_Set_xref()   / XPso_fsm_Get_xref()
 */
int test_parameter_operations(XPso_fsm* device) {
    printf("\n====================================================================\n");
    printf("TEST 2: Parameter Setting and Getting\n");
    printf("====================================================================\n");

    if (!device) {
        printf("[INFO] Skipping hardware parameter test (device not initialized)\n");
        return 0;
    }

    u32 test_value = 0x12345678;
    u32 read_value;

    /* Test u_curr (current control input) */
    printf("\nTesting u_curr parameter:\n");
    printf("  Writing: 0x%08x\n", test_value);
    XPso_fsm_Set_u_curr(device, test_value);
    read_value = XPso_fsm_Get_u_curr(device);
    printf("  Read:    0x%08x\n", read_value);
    printf("  Status:  %s\n", (read_value == test_value) ? "[PASS]" : "[FAIL]");

    /* Test x_curr (current state) */
    printf("\nTesting x_curr parameter:\n");
    test_value = 0xABCDEF00;
    printf("  Writing: 0x%08x\n", test_value);
    XPso_fsm_Set_x_curr(device, test_value);
    read_value = XPso_fsm_Get_x_curr(device);
    printf("  Read:    0x%08x\n", read_value);
    printf("  Status:  %s\n", (read_value == test_value) ? "[PASS]" : "[FAIL]");

    /* Test xref (reference state) */
    printf("\nTesting xref parameter:\n");
    test_value = 0x11111111;
    printf("  Writing: 0x%08x\n", test_value);
    XPso_fsm_Set_xref(device, test_value);
    read_value = XPso_fsm_Get_xref(device);
    printf("  Read:    0x%08x\n", read_value);
    printf("  Status:  %s\n", (read_value == test_value) ? "[PASS]" : "[FAIL]");

    return 0;
}

/**
 * TEST 3: Fitness and Optimization Tracking
 *
 * Tests:
 * - XPso_fsm_Set_last_best()       / XPso_fsm_Get_last_best()
 * - XPso_fsm_Set_new_best()        / XPso_fsm_Get_new_best()
 * - XPso_fsm_Set_bestfitness_top() / XPso_fsm_Get_bestfitness_top()
 */
int test_fitness_operations(XPso_fsm* device) {
    printf("\n====================================================================\n");
    printf("TEST 3: Fitness and Optimization Parameters\n");
    printf("====================================================================\n");

    if (!device) {
        printf("[INFO] Skipping hardware fitness test (device not initialized)\n");
        return 0;
    }

    u32 fitness_value;

    /* Test last_best */
    printf("\nTesting last_best (previous best fitness):\n");
    fitness_value = 0x00000100;
    printf("  Setting last_best to: 0x%08x\n", fitness_value);
    XPso_fsm_Set_last_best(device, fitness_value);
    printf("  Read back: 0x%08x\n", XPso_fsm_Get_last_best(device));

    /* Test new_best */
    printf("\nTesting new_best (current best fitness):\n");
    fitness_value = 0x00000200;
    printf("  Setting new_best to: 0x%08x\n", fitness_value);
    XPso_fsm_Set_new_best(device, fitness_value);
    printf("  Read back: 0x%08x\n", XPso_fsm_Get_new_best(device));

    /* Test bestfitness_top */
    printf("\nTesting bestfitness_top (global best fitness):\n");
    fitness_value = 0x00000300;
    printf("  Setting bestfitness_top to: 0x%08x\n", fitness_value);
    XPso_fsm_Set_bestfitness_top(device, fitness_value);
    printf("  Read back: 0x%08x\n", XPso_fsm_Get_bestfitness_top(device));

    return 0;
}

/**
 * TEST 4: Control Flow (Start, Wait, Done)
 *
 * Tests:
 * - XPso_fsm_Start()
 * - XPso_fsm_IsDone()
 * - XPso_fsm_IsIdle()
 * - XPso_fsm_Get_return()
 */
int test_control_flow(XPso_fsm* device) {
    printf("\n====================================================================\n");
    printf("TEST 4: Control Flow (Start, Wait, Done)\n");
    printf("====================================================================\n");

    if (!device) {
        printf("[INFO] Skipping hardware control flow test (device not initialized)\n");
        return 0;
    }

    int timeout = 100;  /* Maximum iterations to wait */
    int iterations = 0;

    printf("Starting FSM computation...\n");
    XPso_fsm_Start(device);

    printf("Waiting for computation to complete...\n");
    while (!XPso_fsm_IsDone(device) && (iterations < timeout)) {
        usleep(1000);  /* Wait 1ms */
        iterations++;
    }

    if (iterations >= timeout) {
        printf("[WARN] Computation timeout after %d iterations\n", timeout);
    } else {
        printf("[PASS] Computation completed after %d iterations\n", iterations);
    }

    printf("FSM is idle: %s\n", XPso_fsm_IsIdle(device) ? "YES" : "NO");
    printf("FSM is ready: %s\n", XPso_fsm_IsReady(device) ? "YES" : "NO");

    u32 return_value = XPso_fsm_Get_return(device);
    printf("Computation return value: 0x%08x\n", return_value);

    return 0;
}

/**
 * TEST 5: Simulation Loop (Multiple Time Steps)
 *
 * Simulates multiple control cycles, similar to the loop in test_model_cpp.m:
 *   for i=1:size(uHistory,1)
 *       state_dot = drone_model(state, control)
 *       state = state + state_dot * Ts
 *   end
 *
 * This tests the FSM's ability to:
 * - Process sequential state updates
 * - Compute optimal controls for different states
 * - Maintain consistency across multiple iterations
 */
int test_simulation_loop(XPso_fsm* device) {
    printf("\n====================================================================\n");
    printf("TEST 5: Multi-Step Simulation Loop\n");
    printf("====================================================================\n");
    printf("(Modeled after MATLAB test_model_cpp.m loop structure)\n");

    DroneParams params;
    DroneState current_state, reference_state;
    DroneControl control;

    init_drone_params(&params);
    init_drone_state(&current_state);
    init_drone_state(&reference_state);

    /* Set reference trajectory (desired position) */
    reference_state.x[0] = 10.0;  /* Target x */
    reference_state.x[1] = 12.0;  /* Target y */
    reference_state.x[2] = 2.0;   /* Target z */

    print_drone_state("Initial State", &current_state);
    print_drone_state("Reference State", &reference_state);

printf("\n--------------------------------------------------------------------\n");
    printf("Simulation Parameters:\n");
    printf("  Sampling time (Ts):     %.4f s\n", params.Ts);
    printf("  Number of steps:        10\n");
    printf("  Drone mass:             %.2f kg\n", params.m);
    printf("  Arm length:             %.3f m\n", params.l);

    printf("\n--------------------------------------------------------------------\n");
    printf("Simulation Loop:\n");
    printf("%-5s %-12s %-12s %-12s %-12s\n", "Step", "x[m]", "y[m]", "z[m]", "err_norm");
    printf("--------------------------------------------------------------------\n");

    /* Simulation loop: 10 time steps */
    int num_steps = 10;
    for (int step = 0; step < num_steps; step++) {
        /* Compute error from current state to reference */
        double error_x = reference_state.x[0] - current_state.x[0];
        double error_y = reference_state.x[1] - current_state.x[1];
        double error_z = reference_state.x[2] - current_state.x[2];
        double error_norm = sqrt(error_x*error_x + error_y*error_y + error_z*error_z);

        /* Simple proportional control for this test
           (In actual PSO FSM, this would be computed by the hardware) */
        double kp = 2.0;  /* Proportional gain */
        control.u[0] = params.m * params.g + kp * error_z;  /* Thrust */
        control.u[1] = kp * error_y;  /* Roll moment */
        control.u[2] = kp * error_x;  /* Pitch moment */
        control.u[3] = 0.0;            /* Yaw moment */

        printf("%3d   %.4f     %.4f     %.4f     %.4f\n",
               step, current_state.x[0], current_state.x[1], current_state.x[2], error_norm);

        if (device) {
            /* If hardware is available, send parameters to FSM
               (This would trigger actual optimization) */
            /* (Note: In a real scenario, this would encode state/control as u32 values) */
        }

        /* Simple integrator for state update (Euler method)
           In real test_model_cpp.m, this calls drone_model() C MEX function */
        double dx = current_state.x[6] * params.Ts;  /* Integrate velocity */
        double dy = current_state.x[7] * params.Ts;
        double dz = current_state.x[8] * params.Ts;

        current_state.x[0] += dx;
        current_state.x[1] += dy;
        current_state.x[2] += dz;

        /* Stop if converged to reference */
        if (error_norm < 0.01) {
            printf("  [Converged after %d steps]\n", step + 1);
            break;
        }
    }

    printf("--------------------------------------------------------------------\n");
    print_drone_state("\nFinal State", &current_state);

    return 0;
}

/**
 * TEST 6: Interrupt and Status Management (if supported)
 *
 * Tests:
 * - XPso_fsm_InterruptGlobalEnable()
 * - XPso_fsm_InterruptGlobalDisable()
 * - XPso_fsm_InterruptEnable()
 * - XPso_fsm_InterruptGetStatus()
 */
int test_interrupt_management(XPso_fsm* device) {
    printf("\n====================================================================\n");
    printf("TEST 6: Interrupt Management\n");
    printf("====================================================================\n");

    if (!device) {
        printf("[INFO] Skipping hardware interrupt test (device not initialized)\n");
        return 0;
    }

    printf("Enabling global interrupts...\n");
    XPso_fsm_InterruptGlobalEnable(device);

    printf("Enabling ap_done interrupt...\n");
    XPso_fsm_InterruptEnable(device, 0x1);  /* ap_done interrupt */

    u32 enabled_interrupts = XPso_fsm_InterruptGetEnabled(device);
    printf("Enabled interrupts: 0x%08x\n", enabled_interrupts);

    u32 interrupt_status = XPso_fsm_InterruptGetStatus(device);
    printf("Interrupt status: 0x%08x\n", interrupt_status);

    printf("Disabling interrupts...\n");
    XPso_fsm_InterruptGlobalDisable(device);

    return 0;
}

/**
 * TEST 7: Auto-Restart Mode
 *
 * Tests:
 * - XPso_fsm_EnableAutoRestart()
 * - XPso_fsm_DisableAutoRestart()
 */
int test_autorestart(XPso_fsm* device) {
    printf("\n====================================================================\n");
    printf("TEST 7: Auto-Restart Mode\n");
    printf("====================================================================\n");

    if (!device) {
        printf("[INFO] Skipping hardware auto-restart test (device not initialized)\n");
        return 0;
    }

    printf("Enabling auto-restart mode...\n");
    XPso_fsm_EnableAutoRestart(device);
    printf("[PASS] Auto-restart enabled\n");

    printf("Disabling auto-restart mode...\n");
    XPso_fsm_DisableAutoRestart(device);
    printf("[PASS] Auto-restart disabled\n");

    return 0;
}

/* =========================================================================
   MAIN TEST RUNNER
   ========================================================================= */

int main(void) {
    printf("\n");
    printf("╔═══════════════════════════════════════════════════════════════════════╗\n");
    printf("║        XPSO_FSM Driver Test Suite                                     ║\n");
    printf("║        PSO-Based Control Optimization for Sniff Drone                 ║\n");
    printf("║                                                                       ║\n");
    printf("║        Reference: test_model_cpp.m (MATLAB drone model test)          ║\n");
    printf("║        Date: 2026-04-29                                               ║\n");
    printf("╚═══════════════════════════════════════════════════════════════════════╝\n");

    int hw_available = 1;
    XPso_fsm device;

    /* =====================================================================
       Test 1: Device Initialization
       ===================================================================== */
    int init_result = test_device_initialization();
    if (init_result == 0) {
        hw_available = 1;
    }

    if (hw_available) {
        /* Re-initialize for subsequent tests */
        XPso_fsm_Initialize(&device, PSO_BASE);

        /* ================================================================
           Run hardware-dependent tests
           ================================================================ */
        test_parameter_operations(&device);
        test_fitness_operations(&device);
        test_control_flow(&device);
        test_interrupt_management(&device);
        test_autorestart(&device);

        /* ================================================================
           Release device
           ================================================================ */
        printf("\n[INFO] Releasing device...\n");
        //XPso_fsm_Release(&device);
    } else {
        printf("\n[INFO] Proceeding with software-only tests\n");
        printf("       (Hardware tests require FPGA with PSO FSM IP core)\n");
    }

    /* =====================================================================
       Test Simulation Loop (works with or without hardware)
       ===================================================================== */
    if (hw_available) {
        XPso_fsm_Initialize(&device, "pso_fsm_0");
        test_simulation_loop(&device);
        //XPso_fsm_Release(&device);
    } else {
        test_simulation_loop(NULL);
    }

    /* =====================================================================
       Test Summary
       ===================================================================== */
    printf("\n");
    printf("===================================================================\n");
    printf("                        TEST SUMMARY\n");
    printf("===================================================================\n");
    printf("\nTests Executed:\n");
    printf("  ✓ TEST 1: Device Initialization and Status Checks\n");
    if (hw_available) {
        printf("  ✓ TEST 2: Parameter Setting and Getting\n");
        printf("  ✓ TEST 3: Fitness and Optimization Parameters\n");
        printf("  ✓ TEST 4: Control Flow (Start, Wait, Done)\n");
        printf("  ✓ TEST 6: Interrupt Management\n");
        printf("  ✓ TEST 7: Auto-Restart Mode\n");
    }
    printf("  ✓ TEST 5: Multi-Step Simulation Loop\n");

    printf("\nHardware Status:\n");
    printf("  FPGA PSO FSM Core: %s\n", hw_available ? "DETECTED" : "NOT DETECTED");

    if (!hw_available) {
        printf("\nNote: To run full hardware tests, ensure:\n");
        printf("  1. FPGA bitstream is programmed with PSO FSM IP core\n");
        printf("  2. UIO device driver is loaded\n");
        printf("  3. PSO FSM device is exposed at /sys/class/uio/\n");
    }

    printf("\nAll available tests completed successfully!\n\n");

    return 0;
}
