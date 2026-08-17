from __future__ import annotations

from typing import Any

import numpy as np


NX = 12
NU = 4
DEFAULT_TS = 0.05


def spiral_reference(times: Any) -> np.ndarray:
    time_values = np.atleast_1d(np.asarray(times, dtype=np.float64))
    reference = np.zeros((time_values.size, NX), dtype=np.float64)
    angle = 2.0 * np.pi * time_values / 20.0 - np.pi / 2.0
    reference[:, 0] = 7.0 + 2.5 * np.cos(angle)
    reference[:, 1] = 12.5 + 2.5 * np.sin(angle)
    reference[:, 2] = np.clip(time_values / 8.0, 0.0, 15.0)
    reference[:, 5] = 2.0 * np.pi * time_values / 20.0
    return reference


def reference_horizon(
    current_time: float, horizon: int = 25, sample_time: float = DEFAULT_TS
) -> np.ndarray:
    times = current_time + np.arange(1, horizon + 1) * sample_time
    return spiral_reference(times)


def quadrotor_derivative(state: Any, normalized_control: Any) -> np.ndarray:
    state_array = np.asarray(state, dtype=np.float64)
    control = np.asarray(normalized_control, dtype=np.float64)
    if state_array.shape != (NX,):
        raise ValueError(f"state must have shape ({NX},), got {state_array.shape}")
    if control.shape != (NU,):
        raise ValueError(f"normalized_control must have shape ({NU},), got {control.shape}")

    control = np.clip(control, -100.0, 100.0)
    physical_control = control * np.array([0.15, 0.03, 0.03, 0.03])
    motor_mixing_inverse = np.array(
        [
            [0.25, 0.0, -0.5, -0.25],
            [0.25, -0.5, 0.0, 0.25],
            [0.25, 0.0, 0.5, -0.25],
            [0.25, 0.5, 0.0, 0.25],
        ]
    )
    motor_inputs = 4.905 + motor_mixing_inverse @ (
        physical_control * np.array([1.0, 4.0, 4.0, 5.0])
    )
    u1, u2, u3, u4 = motor_inputs

    ixx, iyy, izz = 1.2, 1.2, 2.3
    arm_length, thrust, drag, mass, gravity = 0.25, 1.0, 0.2, 2.0, 9.81
    phi, theta, psi = state_array[3:6]
    xdot, ydot, zdot = state_array[6:9]
    phidot, thetadot, psidot = state_array[9:12]

    cos_phi, cos_theta, cos_psi = np.cos(phi), np.cos(theta), np.cos(psi)
    sin_phi, sin_theta, sin_psi = np.sin(phi), np.sin(theta), np.sin(psi)
    if abs(cos_theta) < 1e-8:
        raise ValueError("quadrotor model is singular near pitch = +/- pi/2")

    t8 = iyy * iyy
    t9 = izz * izz
    t12 = thetadot * thetadot
    t21 = 1.0 / iyy
    t22 = 1.0 / izz
    t23 = 1.0 / mass
    total_motor_input = u1 + u2 + u3 + u4
    t13 = cos_phi * cos_phi
    t14 = cos_phi * cos_phi * cos_phi
    t16 = cos_theta * cos_theta
    t17 = np.sin(2.0 * phi)
    t18 = sin_phi * sin_phi
    t19 = np.sin(2.0 * theta)
    t20 = sin_theta * sin_theta
    t25 = 1.0 / cos_theta
    t15 = t13 * t13
    t27 = 1.0 / (t20 - 1.0)

    derivative = np.empty(NX, dtype=np.float64)
    derivative[:6] = [xdot, ydot, zdot, phidot, thetadot, psidot]
    derivative[6] = thrust * t23 * total_motor_input * (
        sin_phi * sin_psi + cos_phi * cos_psi * sin_theta
    )
    derivative[7] = -thrust * t23 * total_motor_input * (
        cos_psi * sin_phi - cos_phi * sin_psi * sin_theta
    )
    derivative[8] = -gravity + thrust * cos_phi * cos_theta * t23 * total_motor_input

    derivative[9] = (
        -sin_theta
        * t21
        * t22
        * t27
        * (iyy - iyy * t18 + izz * t18)
        * (
            -drag * u1
            + drag * u2
            - drag * u3
            + drag * u4
            + 0.5 * ixx * phidot * cos_theta * thetadot
            + 0.5 * iyy * phidot * cos_theta * thetadot
            - 0.5 * izz * phidot * cos_theta * thetadot
            - 0.5 * ixx * psidot * t19 * thetadot
            + 0.5 * iyy * psidot * t19 * thetadot
            - iyy * phidot * cos_theta * t13 * thetadot
            + izz * phidot * cos_theta * t13 * thetadot
            + 0.5 * iyy * cos_phi * sin_phi * sin_theta * t12
            - 0.5 * izz * cos_phi * sin_phi * sin_theta * t12
            - iyy * phidot * psidot * cos_phi * sin_phi * t16
            + izz * phidot * psidot * cos_phi * sin_phi * t16
            - iyy * psidot * cos_theta * sin_theta * t13 * thetadot
            + izz * psidot * cos_theta * sin_theta * t13 * thetadot
        )
        - t21
        * t22
        * t27
        * (-thrust * arm_length * u2 + thrust * arm_length * u4 + 0.5 * ixx * psidot * cos_theta * thetadot)
        * (iyy * izz + ixx * iyy * t20 - iyy * izz * t20 - ixx * iyy * t18 * t20 + ixx * izz * t18 * t20)
        / ixx
        + sin_theta
        * t17
        * t21
        * t22
        * t25
        * (0.5 * iyy - 0.5 * izz)
        * (
            thrust * arm_length * u1
            - thrust * arm_length * u3
            - 0.5 * iyy * phidot * t17 * thetadot
            + 0.5 * izz * phidot * t17 * thetadot
            - 0.5 * iyy * phidot * psidot * cos_theta
            + 0.5 * izz * phidot * psidot * cos_theta
            + iyy * phidot * psidot * cos_theta * t13
            - izz * phidot * psidot * cos_theta * t13
            - 0.5 * iyy * psidot * cos_phi * sin_phi * sin_theta * thetadot
            + 0.5 * izz * psidot * cos_phi * sin_phi * sin_theta * thetadot
        )
    )

    derivative[10] = 0.5 * t21 * t22 * t25 * (
        phidot * psidot * t8 * t16
        - sin_theta * t8 * t12 * t13
        - sin_theta * t9 * t12 * t13
        + sin_theta * t8 * t12 * t15
        + sin_theta * t9 * t12 * t15
        + iyy * drag * t17 * u1
        - iyy * drag * t17 * u2
        + iyy * drag * t17 * u3
        - iyy * drag * t17 * u4
        - izz * drag * t17 * u1
        + izz * drag * t17 * u2
        - izz * drag * t17 * u3
        + izz * drag * t17 * u4
        - iyy * izz * phidot * psidot * t16
        + 2.0 * iyy * izz * sin_theta * t12 * t13
        - 2.0 * iyy * izz * sin_theta * t12 * t15
        - 2.0 * iyy * thrust * arm_length * cos_theta * u1
        + 2.0 * iyy * thrust * arm_length * cos_theta * u3
        - phidot * psidot * t8 * t13 * t16
        + phidot * psidot * t9 * t13 * t16
        + phidot * cos_phi * cos_theta * sin_phi * t8 * thetadot
        - phidot * cos_phi * cos_theta * sin_phi * t9 * thetadot
        + 2.0 * iyy * thrust * arm_length * cos_theta * t13 * u1
        - 2.0 * iyy * thrust * arm_length * cos_theta * t13 * u3
        - 2.0 * izz * thrust * arm_length * cos_theta * t13 * u1
        + 2.0 * izz * thrust * arm_length * cos_theta * t13 * u3
        - ixx * iyy * phidot * cos_phi * cos_theta * sin_phi * thetadot
        + ixx * izz * phidot * cos_phi * cos_theta * sin_phi * thetadot
        + 2.0 * iyy * thrust * arm_length * cos_phi * sin_phi * sin_theta * u2
        - 2.0 * iyy * thrust * arm_length * cos_phi * sin_phi * sin_theta * u4
        - 2.0 * izz * thrust * arm_length * cos_phi * sin_phi * sin_theta * u2
        + 2.0 * izz * thrust * arm_length * cos_phi * sin_phi * sin_theta * u4
        - psidot * cos_phi * cos_theta * sin_phi * sin_theta * t8 * thetadot
        + psidot * cos_theta * sin_phi * sin_theta * t8 * t14 * thetadot
        + psidot * cos_theta * sin_phi * sin_theta * t9 * t14 * thetadot
        + ixx * iyy * psidot * cos_phi * cos_theta * sin_phi * sin_theta * thetadot
        - ixx * izz * psidot * cos_phi * cos_theta * sin_phi * sin_theta * thetadot
        + iyy * izz * psidot * cos_phi * cos_theta * sin_phi * sin_theta * thetadot
        - 2.0 * iyy * izz * psidot * cos_theta * sin_phi * sin_theta * t14 * thetadot
    )

    derivative[11] = 0.5 * t21 * t22 / t16 * (
        -2.0 * izz * drag * u1
        + 2.0 * izz * drag * u2
        - 2.0 * izz * drag * u3
        + 2.0 * izz * drag * u4
        - phidot * cos_theta * t9 * thetadot
        - 2.0 * iyy * drag * t13 * u1
        + 2.0 * iyy * drag * t13 * u2
        - 2.0 * iyy * drag * t13 * u3
        + 2.0 * iyy * drag * t13 * u4
        + 2.0 * izz * drag * t13 * u1
        - 2.0 * izz * drag * t13 * u2
        + 2.0 * izz * drag * t13 * u3
        - 2.0 * izz * drag * t13 * u4
        + ixx * izz * phidot * cos_theta * thetadot
        + iyy * izz * phidot * cos_theta * thetadot
        - 0.5 * ixx * izz * psidot * t19 * thetadot
        + iyy * izz * psidot * t19 * thetadot
        - 2.0 * izz * thrust * arm_length * sin_theta * u2
        + 2.0 * izz * thrust * arm_length * sin_theta * u4
        - phidot * cos_theta * t8 * t13 * thetadot
        + phidot * cos_theta * t9 * t13 * thetadot
        - cos_phi * sin_phi * sin_theta * t9 * t12
        + sin_phi * sin_theta * t8 * t12 * t14
        + sin_phi * sin_theta * t9 * t12 * t14
        - phidot * psidot * cos_phi * sin_phi * t8 * t16
        + phidot * psidot * cos_phi * sin_phi * t9 * t16
        + psidot * cos_theta * sin_theta * t8 * t13 * thetadot
        + psidot * cos_theta * sin_theta * t9 * t13 * thetadot
        - psidot * cos_theta * sin_theta * t8 * t15 * thetadot
        - psidot * cos_theta * sin_theta * t9 * t15 * thetadot
        + ixx * iyy * phidot * cos_theta * t13 * thetadot
        - ixx * izz * phidot * cos_theta * t13 * thetadot
        + iyy * izz * cos_phi * sin_phi * sin_theta * t12
        - 2.0 * iyy * izz * sin_phi * sin_theta * t12 * t14
        - 2.0 * iyy * thrust * arm_length * sin_theta * t13 * u2
        + 2.0 * iyy * thrust * arm_length * sin_theta * t13 * u4
        + 2.0 * izz * thrust * arm_length * sin_theta * t13 * u2
        - 2.0 * izz * thrust * arm_length * sin_theta * t13 * u4
        - ixx * iyy * psidot * cos_theta * sin_theta * t13 * thetadot
        + ixx * izz * psidot * cos_theta * sin_theta * t13 * thetadot
        - 2.0 * iyy * izz * psidot * cos_theta * sin_theta * t13 * thetadot
        + 2.0 * iyy * izz * psidot * cos_theta * sin_theta * t15 * thetadot
        + 2.0 * iyy * thrust * arm_length * cos_phi * cos_theta * sin_phi * u1
        - 2.0 * iyy * thrust * arm_length * cos_phi * cos_theta * sin_phi * u3
        - 2.0 * izz * thrust * arm_length * cos_phi * cos_theta * sin_phi * u1
        + 2.0 * izz * thrust * arm_length * cos_phi * cos_theta * sin_phi * u3
    )
    return derivative


def quadrotor_step(
    state: Any, normalized_control: Any, sample_time: float = DEFAULT_TS
) -> np.ndarray:
    if sample_time <= 0:
        raise ValueError("sample_time must be positive")
    state_array = np.asarray(state, dtype=np.float64)
    return state_array + sample_time * quadrotor_derivative(
        state_array, normalized_control
    )
