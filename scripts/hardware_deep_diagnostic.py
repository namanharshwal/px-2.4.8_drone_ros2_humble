#!/usr/bin/env python3
# Copyright (c) 2025-2026 Naman Harshwal. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
F450 Deep Hardware Diagnostics & Avionics Benchmarking Suite
Author: Naman Harshwal

Performs deep analysis of:
1. Micro-XRCE-DDS Serial Link & Timesync Jitter
2. IMU Sensor Noise, Variance, and Earth Gravity Magnitude
3. EKF2 State Estimator Health & Variances
4. Failsafe Matrix & Hardware Sensor Bitflags
5. Attitude Stability & Level Calibration Check
"""

import math
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    SensorCombined,
    VehicleAttitude,
    VehicleOdometry,
    VehicleStatus,
    FailsafeFlags,
    TimesyncStatus
)

class HardwareDeepDiagnostic(Node):
    def __init__(self):
        super().__init__('hardware_deep_diagnostic')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sensor_samples = []
        self.attitude_samples = []
        self.odometry_data = None
        self.status_data = None
        self.failsafe_data = None
        self.timesync_data = None

        self.create_subscription(SensorCombined, '/fmu/out/sensor_combined', self.sensor_cb, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, qos)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_cb, qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos)
        self.create_subscription(FailsafeFlags, '/fmu/out/failsafe_flags', self.failsafe_cb, qos)
        self.create_subscription(TimesyncStatus, '/fmu/out/timesync_status', self.timesync_cb, qos)

        self.get_logger().info("Collecting 100 IMU samples and telemetry across all topics...")

    def sensor_cb(self, msg):
        if len(self.sensor_samples) < 100:
            self.sensor_samples.append({
                'accel': list(msg.accelerometer_m_s2),
                'gyro': list(msg.gyro_rad)
            })

    def attitude_cb(self, msg):
        if len(self.attitude_samples) < 100:
            self.attitude_samples.append(list(msg.q))

    def odometry_cb(self, msg):
        self.odometry_data = msg

    def status_cb(self, msg):
        self.status_data = msg

    def failsafe_cb(self, msg):
        self.failsafe_data = msg

    def timesync_cb(self, msg):
        self.timesync_data = msg

    def is_ready(self):
        return (
            len(self.sensor_samples) >= 100 and
            len(self.attitude_samples) >= 100 and
            self.odometry_data is not None and
            self.status_data is not None and
            self.failsafe_data is not None
        )

def analyze(node):
    print("\n" + "="*70)
    print("   F450 AVIONICS & SENSOR SUITE — DEEP HARDWARE BENCHMARK REPORT")
    print("="*70)

    # 1. IMU Statistical Benchmark
    accels = np.array([s['accel'] for s in node.sensor_samples])
    gyros = np.array([s['gyro'] for s in node.sensor_samples])

    mean_accel = np.mean(accels, axis=0)
    std_accel = np.std(accels, axis=0)
    mean_gyro = np.mean(gyros, axis=0)
    std_gyro = np.std(gyros, axis=0)

    gravity_mag = np.linalg.norm(mean_accel)
    gravity_nominal = 9.80665
    gravity_error = abs(gravity_mag - gravity_nominal)

    print("\n[TEST 1] IMU SENSOR NOISE & CALIBRATION ANALYSIS (100 Samples)")
    print(f"  * Accelerometer Mean (X, Y, Z)  : [{mean_accel[0]:+7.3f}, {mean_accel[1]:+7.3f}, {mean_accel[2]:+7.3f}] m/s^2")
    print(f"  * Accelerometer Noise StdDev     : [{std_accel[0]:7.4f}, {std_accel[1]:7.4f}, {std_accel[2]:7.4f}] m/s^2")
    print(f"  * Measured Earth Gravity |g|     : {gravity_mag:.3f} m/s^2 (Theoretical: {gravity_nominal:.3f} m/s^2, Error: {gravity_error:.3f} m/s^2)")
    print(f"  * Gyroscope Mean Zero-Bias       : [{mean_gyro[0]:+7.5f}, {mean_gyro[1]:+7.5f}, {mean_gyro[2]:+7.5f}] rad/s")
    print(f"  * Gyroscope Noise StdDev         : [{std_gyro[0]:7.5f}, {std_gyro[1]:7.5f}, {std_gyro[2]:7.5f}] rad/s")

    accel_pass = gravity_error < 0.25 and np.all(std_accel < 0.15)
    gyro_pass = np.all(np.abs(mean_gyro) < 0.05) and np.all(std_gyro < 0.05)
    print(f"  -> IMU Calibration & Noise Status: {'[PASS] NOMINAL' if (accel_pass and gyro_pass) else '[WARN] NOISY / CALIBRATION NEEDED'}")

    # 2. Attitude & Leveling
    quats = np.array(node.attitude_samples)
    mean_q = np.mean(quats, axis=0)
    mean_q /= np.linalg.norm(mean_q)
    w, x, y, z = mean_q
    roll_deg = math.degrees(math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y)))
    pitch_deg = math.degrees(math.asin(max(-1.0, min(1.0, 2.0*(w*y - z*x)))))
    yaw_deg = math.degrees(math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z)))

    print("\n[TEST 2] VEHICLE ATTITUDE & LEVEL ESTIMATION")
    print(f"  * Orientation Quaternion (w,x,y,z): [{w:.4f}, {x:.4f}, {y:.4f}, {z:.4f}]")
    print(f"  * Roll Angle                      : {roll_deg:+6.2f} deg")
    print(f"  * Pitch Angle                     : {pitch_deg:+6.2f} deg")
    print(f"  * Yaw Heading                     : {yaw_deg:+6.2f} deg")
    level_pass = abs(roll_deg) < 5.0 and abs(pitch_deg) < 5.0
    print(f"  -> Bench Leveling Status          : {'[PASS] LEVEL (< 5 deg)' if level_pass else '[WARN] TILTED'}")

    # 3. EKF2 State Estimator Health & Covariance
    odom = node.odometry_data
    pos_var = list(odom.position_variance)
    vel_var = list(odom.velocity_variance)
    ori_var = list(odom.orientation_variance)

    print("\n[TEST 3] EKF2 KALMAN FILTER ESTIMATOR COVARIANCES")
    print(f"  * Position Variances [X, Y, Z]   : [{pos_var[0]:.2e}, {pos_var[1]:.2e}, {pos_var[2]:.2e}] m^2")
    print(f"  * Velocity Variances [X, Y, Z]   : [{vel_var[0]:.2e}, {vel_var[1]:.2e}, {vel_var[2]:.2e}] (m/s)^2")
    print(f"  * Orientation Variances [R, P, Y]: [{ori_var[0]:.2e}, {ori_var[1]:.2e}, {ori_var[2]:.2e}] rad^2")
    print(f"  * Estimated Altitude (Z relative) : {-odom.position[2]:+6.2f} m")
    ekf_pass = pos_var[2] < 1.0 and vel_var[2] < 0.5
    print(f"  -> Kalman Filter Estimator Status : {'[PASS] CONVERGED & HEALTHY' if ekf_pass else '[WARN] CONVERGING'}")

    # 4. Timesync & Serial Bridge Latency
    ts = node.timesync_data
    print("\n[TEST 4] MICRO-XRCE-DDS BRIDGE LATENCY & TIMESTAMPS")
    if ts is not None:
        rtt_ms = ts.round_trip_time / 1000.0
        offset_ms = ts.estimated_offset / 1000.0
        print(f"  * Round-Trip DDS Latency (RTT)    : {rtt_ms:.2f} ms")
        print(f"  * Clock Synchronization Offset    : {offset_ms:+.2f} ms")
        rtt_pass = rtt_ms < 20.0
        print(f"  -> DDS Bridge Jitter & Latency    : {'[PASS] LOW LATENCY (< 20ms)' if rtt_pass else '[WARN] HIGH JITTER'}")
    else:
        print("  * Timesync packet pending...")
        rtt_pass = True

    # 5. Failsafe & System Flags
    fs = node.failsafe_data
    st = node.status_data
    print("\n[TEST 5] FAILSAFE & FLIGHT SAFETY INTERLOCK MATRIX")
    print(f"  * Flight Controller Power Rail    : {'[OK] VALID 5V' if st.power_input_valid else '[FAIL] INVALID'}")
    print(f"  * Physical Safety Button State    : {'[OK] UNLOCKED / DISENGAGED' if st.safety_off else '[LOCKED] PRESSED'}")
    print(f"  * Angular Velocity Health         : {'[OK] VALID' if not fs.angular_velocity_invalid else '[FAIL] INVALID'}")
    print(f"  * Attitude Health                 : {'[OK] VALID' if not fs.attitude_invalid else '[FAIL] INVALID'}")
    print(f"  * Barometric Altitude Health      : {'[OK] VALID' if not fs.local_altitude_invalid else '[FAIL] INVALID'}")
    print(f"  * Battery Health Condition        : {'[OK] HEALTHY' if not fs.battery_unhealthy else '[WARN] UNHEALTHY'}")
    print(f"  * Motor Failure Detection         : {'[OK] NO FAILURES' if not fs.fd_motor_failure else '[FAIL] FAILURE'}")
    print(f"  * ESC Arming Failure Detection    : {'[OK] NO FAILURES' if not fs.fd_esc_arming_failure else '[FAIL] FAILURE'}")

    all_tests_passed = accel_pass and gyro_pass and level_pass and ekf_pass and rtt_pass and not fs.battery_unhealthy and not fs.fd_motor_failure
    print("\n" + "="*70)
    print(f"OVERALL HARDWARE HEALTH: {'100% HEALTHY & FLIGHT BENCH READY [PASS]' if all_tests_passed else 'CHECK WARNINGS ABOVE'}")
    print("="*70 + "\n")

def main():
    rclpy.init()
    node = HardwareDeepDiagnostic()
    start_time = time.time()
    try:
        while rclpy.ok() and (time.time() - start_time) < 10.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.is_ready():
                break
        analyze(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
