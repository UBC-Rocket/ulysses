import casadi as ca


class State:
    POS = slice(0, 3)
    VEL = slice(3, 6)
    QUAT = slice(6, 10)
    GYRO_BIAS = slice(10, 13)
    ACCEL_BIAS = slice(13, 16)
    BARO_BIAS = 16
    GND_HEIGHT = 17

    DIM = 18


class ErrorState:
    POS = slice(0, 3)
    VEL = slice(3, 6)
    ATT = slice(6, 9)
    GYRO_BIAS = slice(9, 12)
    ACCEL_BIAS = slice(12, 15)
    BARO_BIAS = 16
    GND_HEIGHT = 17

    DIM = 17


class IMU:
    GYRO = slice(0, 3)
    ACCEL = slice(3, 6)

    DIM = 6


class ProcessNoise:
    GYRO = slice(0, 3)
    ACCEL = slice(3, 6)
    GYRO_BIAS = slice(6, 9)
    ACCEL_BIAS = slice(9, 12)
    BARO_BIAS = 12
    GND_HEIGHT = 13

    DIM = 14


# Nominal state
u = ca.SX.sym("u", IMU.DIM)

x = ca.SX.sym("x", State.DIM)
dx = ca.SX.sym("dx", ErrorState.DIM)

n = ca.SX.sym("n", ProcessNoise.DIM)

# Parameters
g = ca.SX.sym("g", 3)
dt = ca.SX.sym("dt")
