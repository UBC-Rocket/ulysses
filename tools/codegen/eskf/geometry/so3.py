import casadi as ca

ANGLE_EPS = 1e-4


def skewsym(v: ca.SX) -> ca.SX:
    assert v.shape == (3, 1)

    x, y, z = ca.vertsplit(v)

    return ca.vertcat(
        ca.horzcat(0, -z, y),
        ca.horzcat(z, 0, -x),
        ca.horzcat(-y, x, 0),
    )


def right_jacobian(phi: ca.SX) -> ca.SX:
    assert phi.shape == (3, 1)

    theta_sq = ca.dot(phi, phi)
    small = theta_sq < ANGLE_EPS**2

    safe_theta_sq = ca.if_else(small, ANGLE_EPS**2, theta_sq)
    theta = ca.sqrt(safe_theta_sq)

    A = ca.if_else(
        small,
        0.5 - theta_sq / 24 + theta_sq**2 / 720,
        (1 - ca.cos(theta)) / safe_theta_sq,
    )

    B = ca.if_else(
        small,
        1 / 6 - theta_sq / 120 + theta_sq**2 / 5040,
        (theta - ca.sin(theta)) / (safe_theta_sq * theta),
    )

    K = skewsym(phi)

    return ca.SX.eye(3) - A * K + B * (K @ K)
