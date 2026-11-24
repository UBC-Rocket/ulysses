import casadi as ca

ANGLE_EPS = 1e-4


def quatnormalize(q: ca.SX) -> ca.SX:
    assert q.shape == (4, 1)

    return q / ca.sqrt(ca.sumsqr(q))


def quatconj(q: ca.SX) -> ca.SX:
    assert q.shape == (4, 1)

    return ca.vertcat(q[0], -q[1:4])


def quatmul(q1: ca.SX, q2: ca.SX) -> ca.SX:
    assert q1.shape == (4, 1)
    assert q2.shape == (4, 1)

    s1, v1 = q1[0], q1[1:4]
    s2, v2 = q2[0], q2[1:4]

    return ca.vertcat(
        s1 * s2 - ca.dot(v1, v2),
        s1 * v2 + s2 * v1 + ca.cross(v1, v2),
    )


def quat2rot(q: ca.SX) -> ca.SX:
    assert q.shape == (4, 1)

    w, x, y, z = ca.vertsplit(q)

    return ca.vertcat(
        ca.horzcat(1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        ca.horzcat(2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        ca.horzcat(2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def quatexp(theta: ca.SX) -> ca.SX:
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


def quatlog(q: ca.SX) -> ca.SX:
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
