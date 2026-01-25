#!/usr/bin/env python3
"""
EKF C Code Test Setup with Mock Headers and Build System

This script creates mock headers for missing dependencies and builds
the EKF library for testing.
"""

import ctypes
import numpy as np
import os
import subprocess
import sys
from pathlib import Path

class EKFTestSetup:
    """Handles building and setting up the EKF test environment"""
    
    def __init__(self, source_dir="."):
        self.source_dir = Path(source_dir)
        self.test_dir = Path("ekf_test_build")
        self.lib_name = "ekf.so" if sys.platform != "win32" else "ekf.dll"
    
    def check_required_files(self):
        """Check if required source files exist"""
        required_files = ['ekf.c', 'ekf.h']
        optional_files = ['quaternion.c', 'body.c', 'matrix.c', 'matrix.h', 
                         'quaternion.h', 'body.h', 'state.c', 'state.h']
        
        missing_required = []
        available_optional = []
        
        for f in required_files:
            if not (self.source_dir / f).exists():
                missing_required.append(f)
        
        for f in optional_files:
            if (self.source_dir / f).exists():
                available_optional.append(f)
        
        if missing_required:
            print(f"ERROR: Missing required files: {missing_required}")
            return False
        
        print(f"Found source files: {required_files + available_optional}")
        return True, available_optional
    
    def build_library(self, source_files):
        """Compile the EKF library"""
        
        print("\nBuilding EKF library...")
        
        self.test_dir.mkdir(exist_ok=True)
        
        # Prepare compilation command - compile directly from source directory
        c_files = [f for f in source_files if f.endswith('.c')]
        
        compile_cmd = [
            'gcc',
            '-shared',
            '-fPIC',
            '-I' + str(self.source_dir),  # Use actual source directory
            '-o', str(self.test_dir / self.lib_name),
            *[str(self.source_dir / f) for f in c_files],
            '-lm',
            '-std=c99'
        ]
        
        print(f"Compile command: {' '.join(compile_cmd)}")
        
        try:
            result = subprocess.run(compile_cmd, capture_output=True, text=True)
            if result.returncode != 0:
                print("COMPILATION FAILED:")
                print(result.stderr)
                return False
            print(f"✓ Successfully built {self.lib_name}")
            return True
        except FileNotFoundError:
            print("ERROR: gcc compiler not found. Please install gcc.")
            return False
        except Exception as e:
            print(f"ERROR during compilation: {e}")
            return False
    
    def setup(self):
        """Run complete setup process"""
        result = self.check_required_files()
        if not result:
            return False
        
        success, source_files = result
        all_files = ['ekf.c', 'ekf.h'] + source_files
        
        if not self.build_library(all_files):
            return False
        
        return True
    
    def get_library_path(self):
        """Get path to compiled library"""
        return str(self.test_dir / self.lib_name)


class EKFTester:
    """Python wrapper for testing the EKF C implementation"""
    
    def __init__(self, lib_path):
        """Initialize the EKF tester by loading the shared library"""
        if not os.path.exists(lib_path):
            raise FileNotFoundError(f"Library not found at {lib_path}")
        
        self.lib = ctypes.CDLL(lib_path)
        self._setup_function_signatures()
    
    def _setup_function_signatures(self):
        """Define C function signatures for ctypes"""
        
        # init_ekf_orientation
        self.lib.init_ekf_orientation.argtypes = [
            ctypes.POINTER(ctypes.c_float * 4),  # process_noise[4][4]
            ctypes.POINTER(ctypes.c_float * 3),  # measurement_noise[3][3]
            ctypes.POINTER(ctypes.c_float)        # expected_g[3]
        ]
        self.lib.init_ekf_orientation.restype = None
        
        # init_ekf_body
        self.lib.init_ekf_body.argtypes = [
            ctypes.POINTER(ctypes.c_float * 6),  # process_noise[6][6]
            ctypes.POINTER(ctypes.c_float * 3)   # measurement_noise[3][3]
        ]
        self.lib.init_ekf_body.restype = None
        
        # tick_ekf_orientation
        self.lib.tick_ekf_orientation.argtypes = [
            ctypes.c_float,                      # deltaTime
            ctypes.POINTER(ctypes.c_float),      # gyro[3]
            ctypes.POINTER(ctypes.c_float)       # accel[3]
        ]
        self.lib.tick_ekf_orientation.restype = None
        
        # tick_ekf_body
        self.lib.tick_ekf_body.argtypes = [
            ctypes.c_float,                      # deltaTime
            ctypes.POINTER(ctypes.c_float),      # accel[3]
            ctypes.POINTER(ctypes.c_float)       # gps_pos[3]
        ]
        self.lib.tick_ekf_body.restype = None
        
        # get_state
        self.lib.get_state.argtypes = [
            ctypes.POINTER(ctypes.c_float),      # quaternion[4]
            ctypes.POINTER(ctypes.c_float),      # position[3]
            ctypes.POINTER(ctypes.c_float)       # velocity[3]
        ]
        self.lib.get_state.restype = None
    
    def init_orientation(self, process_noise, measurement_noise, expected_g):
        """Initialize orientation EKF"""
        process_noise = np.array(process_noise, dtype=np.float32)
        measurement_noise = np.array(measurement_noise, dtype=np.float32)
        expected_g = np.array(expected_g, dtype=np.float32)
        
        process_c = ((ctypes.c_float * 4) * 4)()
        for i in range(4):
            for j in range(4):
                process_c[i][j] = process_noise[i][j]
        
        measure_c = ((ctypes.c_float * 3) * 3)()
        for i in range(3):
            for j in range(3):
                measure_c[i][j] = measurement_noise[i][j]
        
        gravity_c = expected_g.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        
        self.lib.init_ekf_orientation(process_c, measure_c, gravity_c)
    
    def init_body(self, process_noise, measurement_noise):
        """Initialize body state EKF"""
        process_noise = np.array(process_noise, dtype=np.float32)
        measurement_noise = np.array(measurement_noise, dtype=np.float32)
        
        process_c = ((ctypes.c_float * 6) * 6)()
        for i in range(6):
            for j in range(6):
                process_c[i][j] = process_noise[i][j]
        
        measure_c = ((ctypes.c_float * 3) * 3)()
        for i in range(3):
            for j in range(3):
                measure_c[i][j] = measurement_noise[i][j]
        
        self.lib.init_ekf_body(process_c, measure_c)
    
    def tick_orientation(self, dt, gyro, accel):
        """Update orientation EKF"""
        gyro = np.array(gyro, dtype=np.float32)
        accel = np.array(accel, dtype=np.float32)
        
        gyro_c = gyro.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        accel_c = accel.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        
        self.lib.tick_ekf_orientation(ctypes.c_float(dt), gyro_c, accel_c)
    
    def tick_body(self, dt, accel, gps_pos):
        """Update body state EKF"""
        accel = np.array(accel, dtype=np.float32)
        gps_pos = np.array(gps_pos, dtype=np.float32)
        
        accel_c = accel.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        gps_c = gps_pos.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        
        self.lib.tick_ekf_body(ctypes.c_float(dt), accel_c, gps_c)
    
    def get_state(self):
        """Get current EKF state"""
        quat = np.zeros(4, dtype=np.float32)
        pos = np.zeros(3, dtype=np.float32)
        vel = np.zeros(3, dtype=np.float32)
        
        quat_c = quat.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        pos_c = pos.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        vel_c = vel.ctypes.data_as(ctypes.POINTER(ctypes.c_float))
        
        self.lib.get_state(quat_c, pos_c, vel_c)
        
        return quat, pos, vel


# ============================================
# TEST CASES
# ============================================

def test_orientation_static(ekf):
    """Test orientation EKF with static readings"""
    print("\n" + "="*60)
    print("TEST 1: Orientation EKF - Static (No Rotation)")
    print("="*60)
    
    process_noise = np.eye(4, dtype=np.float32) * 0.001
    measurement_noise = np.eye(3, dtype=np.float32) * 0.1
    expected_g = np.array([0.0, 0.0, 1], dtype=np.float32)
    
    ekf.init_orientation(process_noise, measurement_noise, expected_g)
    ekf.init_body(np.eye(6) * 0.01, np.eye(3) * 0.1)
    
    dt = 0.01
    gyro = np.array([0.0, 0.0, 0.0])
    accel = np.array([0.0, 0.0, 1])
    
    print(f"Simulating {100} steps with dt={dt}s")
    print(f"Gyro: {gyro}, Accel: {accel}\n")
    
    for i in range(100):
        ekf.tick_orientation(dt, gyro, accel)
        
        if i % 25 == 0:
            quat, _, _ = ekf.get_state()
            norm = np.linalg.norm(quat)
            print(f"Step {i:3d}: quat=[{quat[0]:7.4f}, {quat[1]:7.4f}, "
                  f"{quat[2]:7.4f}, {quat[3]:7.4f}], norm={norm:.6f}")
    
    quat, _, _ = ekf.get_state()
    print(f"\n✓ Final quaternion norm: {np.linalg.norm(quat):.6f} (should be ~1.0)")


def test_body_motion(ekf):
    """Test body state EKF with constant acceleration"""
    print("\n" + "="*60)
    print("TEST 2: Body State EKF - Constant Acceleration")
    print("="*60)
    
    process_noise = np.eye(6, dtype=np.float32) * 0.01
    measurement_noise = np.eye(3, dtype=np.float32) * 1.0
    
    ekf.init_orientation(np.eye(4) * 0.001, np.eye(3) * 0.1, np.array([0, 0, -1]))
    ekf.init_body(process_noise, measurement_noise)
    
    dt = 0.01
    accel = np.array([1.0, 0.0, 0.0])
    
    print(f"Simulating motion with acceleration: {accel} m/s²")
    print(f"Time step: {dt}s, Total steps: 200\n")
    
    for i in range(200):
        true_pos = np.array([0.5 * accel[0] * (i * dt)**2, 0.0, 0.0])
        noise = np.random.randn(3) * 0.1
        gps_pos = true_pos + noise
        
        ekf.tick_body(dt, accel, gps_pos)
        
        if i % 50 == 0:
            _, pos, vel = ekf.get_state()
            print(f"Step {i:3d}: pos=[{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}], "
                  f"vel=[{vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}]")
    
    _, final_pos, final_vel = ekf.get_state()
    expected_vel = accel * (200 * dt)
    print(f"\nFinal velocity:   [{final_vel[0]:6.3f}, {final_vel[1]:6.3f}, {final_vel[2]:6.3f}]")
    print(f"Expected velocity: [{expected_vel[0]:6.3f}, {expected_vel[1]:6.3f}, {expected_vel[2]:6.3f}]")


def test_rotation(ekf):
    """Test orientation EKF with rotation"""
    print("\n" + "="*60)
    print("TEST 3: Orientation EKF - Z-Axis Rotation")
    print("="*60)
    
    process_noise = np.eye(4, dtype=np.float32) * 0.001
    measurement_noise = np.eye(3, dtype=np.float32) * 0.1
    expected_g = np.array([0.0, 0.0, 1], dtype=np.float32)
    
    ekf.init_orientation(process_noise, measurement_noise, expected_g)
    ekf.init_body(np.eye(6) * 0.01, np.eye(3) * 0.1)
    
    dt = 0.01
    omega_z = 0.5  # rad/s
    
    print(f"Rotating around Z-axis at {omega_z} rad/s ({omega_z*180/np.pi:.1f}°/s)")
    print(f"Time step: {dt}s, Total steps: 200\n")
    
    for i in range(200):
        gyro = np.array([0.0, 0.0, omega_z])
        accel = np.array([0.0, 0.0, 1])
        
        ekf.tick_orientation(dt, gyro, accel)
        
        if i % 50 == 0:
            quat, _, _ = ekf.get_state()
            angle = 2 * np.arccos(np.clip(quat[0], -1, 1)) * 180/np.pi
            print(f"Step {i:3d}: quat=[{quat[0]:7.4f}, {quat[1]:7.4f}, "
                  f"{quat[2]:7.4f}, {quat[3]:7.4f}], angle={angle:6.2f}°")
    
    quat, _, _ = ekf.get_state()
    total_angle = 2 * np.arccos(np.clip(quat[0], -1, 1)) * 180/np.pi
    expected_angle = omega_z * 200 * dt * 180/np.pi
    print(f"\nTotal rotation: {total_angle:.2f}° (expected ~{expected_angle:.2f}°)")


def main():
    print("="*60)
    print("EKF C Code Test Suite with Auto-Build")
    print("="*60)
    
    # Setup and build
    setup = EKFTestSetup()
    
    if not setup.setup():
        print("\nERRRR.")
        return 1
    
    print("\n✓ Build successful!")
    
    # Run tests
    try:
        ekf = EKFTester(setup.get_library_path())
        
        test_orientation_static(ekf)
        test_body_motion(ekf)
        test_rotation(ekf)
        
        print("\n" + "="*60)
        print("chilling")
        print("="*60)
        
        return 0
        
    except Exception as e:
        print(f"\nTest failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())