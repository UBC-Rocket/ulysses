import casadi as ca

from .geometry.quaternion import quat2rot, quatconj, quatexp, quatlog, quatmul
from .geometry.so3 import right_jacobian, skewsym
from .state import IMU, ErrorState, ProcessNoise, State, dt, dx, n, u, x


def boxplus(x: ca.SX, dx: ca.SX) -> ca.SX:
    assert x.shape == (State.DIM, 1)
    assert dx.shape == (ErrorState.DIM, 1)

    out = ca.SX.zeros(State.DIM, 1)

    out[State.POS] = x[State.POS] + dx[ErrorState.POS]
    out[State.VEL] = x[State.VEL] + dx[ErrorState.VEL]
    out[State.QUAT] = quatmul(x[State.QUAT], quatexp(dx[ErrorState.ATT]))
    out[State.GYRO_BIAS] = x[State.GYRO_BIAS] + dx[ErrorState.GYRO_BIAS]
    out[State.ACCEL_BIAS] = x[State.ACCEL_BIAS] + dx[ErrorState.ACCEL_BIAS]
    out[State.BARO_BIAS] = x[State.BARO_BIAS] + dx[ErrorState.BARO_BIAS]
    out[State.GND_HEIGHT] = x[State.GND_HEIGHT] + dx[ErrorState.GND_HEIGHT]

    return out


def boxminus(x1: ca.SX, x2: ca.SX) -> ca.SX:
    """Compute the 17D error between two 18D nominal states."""
    assert x1.shape == (State.DIM, 1)
    assert x2.shape == (State.DIM, 1)

    out = ca.SX.zeros(ErrorState.DIM, 1)

    out[ErrorState.POS] = x1[State.POS] - x2[State.POS]
    out[ErrorState.VEL] = x1[State.VEL] - x2[State.VEL]
    out[ErrorState.ATT] = quatlog(quatmul(quatconj(x2[State.QUAT]), x1[State.QUAT]))
    out[ErrorState.GYRO_BIAS] = x1[State.GYRO_BIAS] - x2[State.GYRO_BIAS]
    out[ErrorState.ACCEL_BIAS] = x1[State.ACCEL_BIAS] - x2[State.ACCEL_BIAS]
    out[ErrorState.BARO_BIAS] = x1[State.BARO_BIAS] - x2[State.BARO_BIAS]
    out[ErrorState.GND_HEIGHT] = x1[State.GND_HEIGHT] - x2[State.GND_HEIGHT]

    return out


def tangent_state_model() -> ca.Function:
    # Nominal state
    q = x[State.QUAT]
    bw = x[State.GYRO_BIAS]
    ba = x[State.ACCEL_BIAS]

    dp = dx[ErrorState.POS]
    dv = dx[ErrorState.VEL]
    dtheta = dx[ErrorState.ATT]
    dbw = dx[ErrorState.GYRO_BIAS]
    dba = dx[ErrorState.ACCEL_BIAS]
    dbb = dx[ErrorState.BARO_BIAS]
    dhg = dx[ErrorState.GND_HEIGHT]

    nw = n[ProcessNoise.GYRO]
    na = n[ProcessNoise.ACCEL]
    nbw = n[ProcessNoise.GYRO_BIAS]
    nba = n[ProcessNoise.ACCEL_BIAS]
    nbb = n[ProcessNoise.BARO_BIAS]
    nhg = n[ProcessNoise.GND_HEIGHT]

    # Bias-corrected IMU measurements
    w_nom = u[IMU.GYRO] - bw
    a_nom = u[IMU.ACCEL] - ba

    phi = w_nom * dt

    # Midpoint rotation
    q_mid = quatmul(q, quatexp(phi / 2))
    R_mid = quat2rot(q_mid)

    # SO(3) Jacobians
    R_half_inv = quat2rot(quatexp(-phi / 2))
    R_full_inv = quat2rot(quatexp(-phi))

    Jr_half = right_jacobian(phi / 2)
    Jr_full = right_jacobian(phi)

    # Midpoint attitude error
    dtheta_mid = R_half_inv @ dtheta - Jr_half @ (dbw + nw) * dt / 2

    # World-frame acceleration error
    da_world = -R_mid @ (skewsym(a_nom) @ dtheta_mid + dba + na)

    # Error-state propagation
    dx_next = ca.SX.zeros(ErrorState.DIM, 1)
    dx_next[ErrorState.POS] = dp + dv * dt + 0.5 * da_world * dt**2
    dx_next[ErrorState.VEL] = dv + da_world * dt
    dx_next[ErrorState.ATT] = R_full_inv @ dtheta - Jr_full @ (dbw + nw) * dt
    dx_next[ErrorState.GYRO_BIAS] = dbw + nbw
    dx_next[ErrorState.ACCEL_BIAS] = dba + nba
    dx_next[ErrorState.BARO_BIAS] = dbb + nbb
    dx_next[ErrorState.GND_HEIGHT] = dhg + nhg

    # Jacobians
    F = ca.jacobian(dx_next, dx)
    G = ca.jacobian(dx_next, n)

    return ca.Function(
        "error_state_jacobians",
        [x, u, dt],
        [ca.densify(F), ca.densify(G)],
        ["x", "u", "dt"],
        ["F", "G"],
    )
