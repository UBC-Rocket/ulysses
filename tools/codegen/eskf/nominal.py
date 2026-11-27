import casadi as ca

from .geometry import so3
from .geometry.quaternion import quat2rot, quatmul, quatnormalize
from .state import IMU, State, dt, g, u, x


def nominal_state_model() -> ca.Function:
    p = x[State.POS]
    v = x[State.VEL]
    q = x[State.QUAT]
    bw = x[State.GYRO_BIAS]
    ba = x[State.ACCEL_BIAS]
    bb = x[State.BARO_BIAS]
    hg = x[State.GND_HEIGHT]

    w_meas = u[IMU.GYRO]
    a_meas = u[IMU.ACCEL]

    w_nom = w_meas - bw
    a_nom = a_meas - ba

    q_mid = quatmul(q, so3.exp_map(w_nom * dt / 2))
    q_next = quatnormalize(quatmul(q, so3.exp_map(w_nom * dt)))

    a_world_mid = quat2rot(q_mid) @ a_nom + g

    x_next = ca.SX.zeros(State.DIM, 1)
    x_next[State.POS] = p + v * dt + 0.5 * a_world_mid * dt**2
    x_next[State.VEL] = v + a_world_mid * dt
    x_next[State.QUAT] = q_next
    x_next[State.GYRO_BIAS] = bw
    x_next[State.ACCEL_BIAS] = ba
    x_next[State.BARO_BIAS] = bb
    x_next[State.GND_HEIGHT] = hg
    

    return ca.Function(
        "nominal_propagate",
        [x, u, g, dt],
        [x_next],
        ["x", "u", "g", "dt"],
        ["x_next"],
    )
