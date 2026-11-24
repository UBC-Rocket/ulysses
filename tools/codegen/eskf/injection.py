import casadi as ca

from .geometry.quaternion import quatnormalize
from .geometry.so3 import right_jacobian
from .state import ErrorState, State, dx, x
from .tangent import boxplus


def injection_model() -> ca.Function:
    x_injected = boxplus(x, dx)
    x_injected[State.QUAT] = quatnormalize(x_injected[State.QUAT])

    G_reset = ca.SX.eye(ErrorState.DIM)
    G_reset[ErrorState.ATT, ErrorState.ATT] = right_jacobian(dx[ErrorState.ATT])

    return ca.Function(
        "error_state_inject",
        [x, dx],
        [ca.densify(x_injected), ca.densify(G_reset)],
        ["x", "dx"],
        ["x_injected", "G_reset"],
    )
