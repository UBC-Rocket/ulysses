import casadi as ca

ANGLE_EPS = 1e-4

def exp_map(theta: ca.SX) -> ca.SX:
    assert theta.shape == (3, 1)

    angle_sq = ca.dot(theta, theta)
    small = angle_sq < ANGLE_EPS**2

    # Avoid sqrt(0) in the exact branch
    safe_angle_sq = ca.if_else(small, ANGLE_EPS**2, angle_sq)
    angle = ca.sqrt(safe_angle_sq)

    scalar = ca.if_else(
        small,
        1 - angle_sq / 8 + angle_sq**2 / 384,  # taylor approx
        ca.cos(angle / 2),  # exact
    )

    scale = ca.if_else(
        small,
        1 / 2 - angle_sq / 48 + angle_sq**2 / 3840,  # taylor approx
        ca.sin(angle / 2) / angle,  # exact
    )

    return ca.vertcat(scalar, scale * theta)


def log_map(q: ca.SX) -> ca.SX:
    assert q.shape == (4, 1)

    w = q[0]
    v = q[1:4]

    # Choose the shortest rotation
    sign = ca.if_else(w < 0, -1, 1)
    w = sign * w
    v = sign * v

    norm_sq = ca.dot(v, v)
    small = norm_sq < ANGLE_EPS**2

    # Avoid singularities in the unused branch
    safe_norm = ca.sqrt(ca.if_else(small, ANGLE_EPS**2, norm_sq))

    # General case
    scale = ca.if_else(
        small,
        2 + norm_sq / 3 + 3 * norm_sq**2 / 20,
        2 * ca.atan2(safe_norm, w) / safe_norm,
    )

    return scale * v


def skew_sym(v: ca.SX) -> ca.SX:
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

    K = skew_sym(phi)

    return ca.SX.eye(3) - A * K + B * (K @ K)
