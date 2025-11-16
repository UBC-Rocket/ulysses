import ctypes
import csv
import math
import matplotlib.pyplot as plt

lib = ctypes.CDLL("./fusion_test.so")

# ----------------------------------------------------------
# C TYPES
# ----------------------------------------------------------

Float4 = ctypes.c_float * 4
Float3 = ctypes.c_float * 3
Float4x4 = (ctypes.c_float * 4) * 4
Float3x3 = (ctypes.c_float * 3) * 3

# ----------------------------------------------------------
# FUNCTION SIGNATURES
# ----------------------------------------------------------

# void init_ekf(float Q[4][4], float R[3][3])
lib.init_ekf.argtypes = [
    ctypes.POINTER(Float4x4),
    ctypes.POINTER(Float3x3)
]
lib.init_ekf.restype = None

# void tick_ekf(float dt, float gyro[3], float accel[3])
lib.tick_ekf.argtypes = [
    ctypes.c_float,
    ctypes.POINTER(Float3),
    ctypes.POINTER(Float3)
]
lib.tick_ekf.restype = None

# void get_state_x(float out[4])
lib.get_state_x.argtypes = [
    ctypes.POINTER(ctypes.c_float)
]
lib.get_state_x.restype = None

def quaternion_to_euler(w, x, y, z):
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  
    else:
        pitch = math.asin(sinp)

    return [roll * 180 / math.pi, pitch * 180 / math.pi]

def test_file(f, N):
    time = [0 for i in range(N)]
    gyro = [[0,0,0] for i in range(N)]
    accel = [[0,0,0] for i in range(N)]
    real = [[0,0,0] for i in range(N)]

    with open(f, newline='') as csvfile:
        csv_reader = csv.reader(csvfile)    
        i = -1

        for row in csv_reader:
            if (i==-1):
                i=0
                continue
            
            time[i] = float(row[0])
            gyro[i][0] = float(row[1]) * math.pi / 180
            gyro[i][1] = float(row[2]) * math.pi / 180
            gyro[i][2] = float(row[3]) * math.pi / 180
            accel[i][0] = float(row[4])
            accel[i][1] = float(row[5])
            accel[i][2] = float(row[6])

            real[i][0] = float(row[7])
            real[i][1] = float(row[8])
            real[i][2] = float(row[9])

            
            i+=1

            if (i >= N):
                break

    # Build Q (4x4 identity * small noise)
    Q = Float4x4()
    for i in range(4):
        for j in range(4):
            Q[i][j] = 1e-6 if i == j else 0.0

    # Build R (3x3 diagonal)
    R = Float3x3()
    R[0][0] = 0.01
    R[1][1] = 0.01
    R[2][2] = 0.02
    for i in range(3):
        for j in range(3):
            if i != j:
                R[i][j] = 0.0

    # Initialize EKF
    lib.init_ekf(ctypes.byref(Q), ctypes.byref(R))

    deltas = []

    for i in range(N):
        gyro_in = Float3(gyro[i][0], gyro[i][1], gyro[i][2])
        accel_in = Float3(accel[i][0], accel[i][1], accel[i][2])

        print(accel_in[0])

        if (i==0):
            dt = 0.01
        else:
            dt = time[i] - time[i-1]

        # Tick with dt = 0.01
        lib.tick_ekf(ctypes.c_float(dt),
                    ctypes.byref(gyro_in),
                    ctypes.byref(accel_in))

        # Read back state quaternion
        state = Float4()
        lib.get_state_x(state)

        quat = state[:]
        euler = quaternion_to_euler(quat[0], quat[1], quat[2], quat[3])

        print("DATA FOR STEP",i)
        print("Quaternion:", list(state[:]))
        print(f"Measured Roll: {euler[0]}\nMeasured Pitch: {euler[1]}")
        print(f"Real Roll: {real[i][0]}\nReal Pitch: {real[i][1]}")

        delta = [abs(euler[0] - real[i][0]), abs(euler[1] - real[i][1])]

        deltas.append(delta)
    
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))

    deltaRoll = [deltas[i][0] for i in range(N)]
    deltaPitch = [deltas[i][1] for i in range(N)]

    axes[0, 0].plot(time, deltaRoll, color = "green")
    axes[0, 0].set_title("Delta Roll (x)")

    axes[0, 1].plot(time, deltaPitch, color = "blue")
    axes[0, 1].set_title("Delta Pitch (y)")

    axes[1, 0].plot(time, deltaPitch, color = "blue")
    axes[1, 0].plot(time, deltaRoll, color = "green")
    axes[1, 0].set_title("Superimposed")





    plt.show()



test_file("test-data/test2.csv", 1000)