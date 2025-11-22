#!/usr/bin/env python3
"""
Side-by-side comparison of Mavros and my implementation.
This script numerically verifies that both approaches produce identical results.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_from_rpy(roll, pitch, yaw):
    """
    Create quaternion from roll-pitch-yaw Euler angles.
    Matches Mavros's quaternion_from_rpy function.
    Returns [w, x, y, z]
    """
    r = R.from_euler('xyz', [roll, pitch, yaw])
    q = r.as_quat()  # scipy returns [x, y, z, w]
    return np.array([q[3], q[0], q[1], q[2]])  # convert to [w, x, y, z]


class MavrosImplementation:
    """Replicates Mavros's coordinate transformation logic."""
    
    def __init__(self):
        # Static quaternion for NED <-> ENU
        # Roll=π, Pitch=0, Yaw=π/2
        self.NED_ENU_Q = quaternion_from_rpy(np.pi, 0.0, np.pi/2)
        
        # Static quaternion for AIRCRAFT (FRD) <-> BASELINK (FLU)
        # Roll=π, Pitch=0, Yaw=0
        self.AIRCRAFT_BASELINK_Q = quaternion_from_rpy(np.pi, 0.0, 0.0)
        
        print("Mavros Implementation Initialized")
        print(f"  NED_ENU_Q: {self.NED_ENU_Q}")
        print(f"  AIRCRAFT_BASELINK_Q: {self.AIRCRAFT_BASELINK_Q}")
    
    def transform_position_enu_to_ned(self, pos_enu):
        """
        Transform position using reflection (Mavros approach).
        Steps: 1) Z-reflection, 2) XY-permutation
        """
        # Step 1: Z-reflection (1, 1, -1)
        pos_temp = np.array([pos_enu[0], pos_enu[1], -pos_enu[2]])
        
        # Step 2: XY-permutation (swap X and Y)
        pos_ned = np.array([pos_temp[1], pos_temp[0], pos_temp[2]])
        
        return pos_ned
    
    def transform_orientation_world_frame(self, q_in):
        """
        Transform orientation for world frame change (ENU->NED).
        Left multiply: NED_ENU_Q * q_in
        """
        r_ned_enu = R.from_quat([self.NED_ENU_Q[1], self.NED_ENU_Q[2], 
                                 self.NED_ENU_Q[3], self.NED_ENU_Q[0]])
        r_in = R.from_quat([q_in[1], q_in[2], q_in[3], q_in[0]])
        
        r_out = r_ned_enu * r_in
        q_out = r_out.as_quat()
        return np.array([q_out[3], q_out[0], q_out[1], q_out[2]])
    
    def transform_orientation_body_frame(self, q_in):
        """
        Transform orientation for body frame change (FLU->FRD).
        Right multiply: q_in * AIRCRAFT_BASELINK_Q
        """
        r_in = R.from_quat([q_in[1], q_in[2], q_in[3], q_in[0]])
        r_aircraft_baselink = R.from_quat([self.AIRCRAFT_BASELINK_Q[1], 
                                           self.AIRCRAFT_BASELINK_Q[2],
                                           self.AIRCRAFT_BASELINK_Q[3], 
                                           self.AIRCRAFT_BASELINK_Q[0]])
        
        r_out = r_in * r_aircraft_baselink
        q_out = r_out.as_quat()
        return np.array([q_out[3], q_out[0], q_out[1], q_out[2]])
    
    def transform_complete(self, pos_enu, q_enu_flu):
        """
        Complete transformation: ENU/FLU -> NED/FRD
        Two-step approach like Mavros.
        """
        # Step 1: Transform position
        pos_ned = self.transform_position_enu_to_ned(pos_enu)
        
        # Step 2: Transform orientation - world frame
        q_ned_flu = self.transform_orientation_world_frame(q_enu_flu)
        
        # Step 3: Transform orientation - body frame
        q_ned_frd = self.transform_orientation_body_frame(q_ned_flu)
        
        return pos_ned, q_ned_frd


class MyImplementation:
    """My single-step transformation implementation."""
    
    def __init__(self):
        # World frame transformation matrix: ENU -> NED
        self.R_world = np.array([
            [0,  1,  0],   # X_ned = Y_enu (North)
            [1,  0,  0],   # Y_ned = X_enu (East)
            [0,  0, -1]    # Z_ned = -Z_enu (Down)
        ])
        
        # Body frame transformation matrix: FLU -> FRD
        self.R_body = np.array([
            [1,  0,  0],   # X_frd = X_flu (Forward)
            [0, -1,  0],   # Y_frd = -Y_flu (Right = -Left)
            [0,  0, -1]    # Z_frd = -Z_flu (Down = -Up)
        ])
        
        # Convert to quaternions
        self.q_world = R.from_matrix(self.R_world).as_quat()
        self.q_world = np.array([self.q_world[3], self.q_world[0], 
                                self.q_world[1], self.q_world[2]])
        
        self.q_body = R.from_matrix(self.R_body).as_quat()
        self.q_body = np.array([self.q_body[3], self.q_body[0], 
                               self.q_body[1], self.q_body[2]])
        
        print("\nMy Implementation Initialized")
        print(f"  q_world_transform: {self.q_world}")
        print(f"  q_body_transform: {self.q_body}")
    
    def transform_position(self, pos_enu):
        """Transform position using matrix multiplication."""
        return self.R_world @ pos_enu
    
    def transform_orientation(self, q_in):
        """
        Single-step orientation transformation.
        q_out = q_world * q_in * q_body
        """
        r_world = R.from_quat([self.q_world[1], self.q_world[2], 
                              self.q_world[3], self.q_world[0]])
        r_in = R.from_quat([q_in[1], q_in[2], q_in[3], q_in[0]])
        r_body = R.from_quat([self.q_body[1], self.q_body[2], 
                             self.q_body[3], self.q_body[0]])
        
        r_out = r_world * r_in * r_body
        q_out = r_out.as_quat()
        return np.array([q_out[3], q_out[0], q_out[1], q_out[2]])
    
    def transform_complete(self, pos_enu, q_enu_flu):
        """
        Complete transformation: ENU/FLU -> NED/FRD
        Single-step approach.
        """
        pos_ned = self.transform_position(pos_enu)
        q_ned_frd = self.transform_orientation(q_enu_flu)
        return pos_ned, q_ned_frd


def compare_quaternions(q1, q2, tolerance=1e-6):
    """
    Compare two quaternions, accounting for the q == -q equivalence.
    Returns True if they represent the same rotation.
    """
    # Check if q1 ≈ q2 or q1 ≈ -q2
    diff1 = np.linalg.norm(q1 - q2)
    diff2 = np.linalg.norm(q1 + q2)
    return min(diff1, diff2) < tolerance


def test_case(name, pos_enu, q_enu_flu, mavros, my_impl):
    """Run a single test case and compare results."""
    print(f"\n{'='*70}")
    print(f"Test Case: {name}")
    print(f"{'='*70}")
    
    # Input
    print("\n[INPUT - ENU/FLU]")
    print(f"  Position: {pos_enu}")
    print(f"  Quaternion: {q_enu_flu}")
    
    # Convert to Euler for interpretation
    r_in = R.from_quat([q_enu_flu[1], q_enu_flu[2], q_enu_flu[3], q_enu_flu[0]])
    euler_in = r_in.as_euler('xyz', degrees=True)
    print(f"  Euler (deg): roll={euler_in[0]:.2f}, pitch={euler_in[1]:.2f}, yaw={euler_in[2]:.2f}")
    
    # Mavros result
    pos_mavros, q_mavros = mavros.transform_complete(pos_enu, q_enu_flu)
    print(f"\n[MAVROS OUTPUT - NED/FRD]")
    print(f"  Position: {pos_mavros}")
    print(f"  Quaternion: {q_mavros}")
    
    r_mavros = R.from_quat([q_mavros[1], q_mavros[2], q_mavros[3], q_mavros[0]])
    euler_mavros = r_mavros.as_euler('xyz', degrees=True)
    print(f"  Euler (deg): roll={euler_mavros[0]:.2f}, pitch={euler_mavros[1]:.2f}, yaw={euler_mavros[2]:.2f}")
    
    # My implementation result
    pos_mine, q_mine = my_impl.transform_complete(pos_enu, q_enu_flu)
    print(f"\n[MY OUTPUT - NED/FRD]")
    print(f"  Position: {pos_mine}")
    print(f"  Quaternion: {q_mine}")
    
    r_mine = R.from_quat([q_mine[1], q_mine[2], q_mine[3], q_mine[0]])
    euler_mine = r_mine.as_euler('xyz', degrees=True)
    print(f"  Euler (deg): roll={euler_mine[0]:.2f}, pitch={euler_mine[1]:.2f}, yaw={euler_mine[2]:.2f}")
    
    # Comparison
    print(f"\n[COMPARISON]")
    pos_diff = np.linalg.norm(pos_mavros - pos_mine)
    print(f"  Position difference: {pos_diff:.10f}")
    
    quat_match = compare_quaternions(q_mavros, q_mine)
    quat_diff = min(np.linalg.norm(q_mavros - q_mine), 
                   np.linalg.norm(q_mavros + q_mine))
    print(f"  Quaternion difference: {quat_diff:.10f}")
    print(f"  Quaternions match: {quat_match}")
    
    # Verdict
    if pos_diff < 1e-6 and quat_match:
        print(f"\n  ✅ PASS - Results are identical!")
        return True
    else:
        print(f"\n  ❌ FAIL - Results differ!")
        return False


def main():
    print("="*70)
    print("MAVROS vs MY IMPLEMENTATION - COMPARISON TEST")
    print("="*70)
    
    # Initialize both implementations
    mavros = MavrosImplementation()
    my_impl = MyImplementation()
    
    # Check if transformation quaternions match
    print("\n" + "="*70)
    print("TRANSFORMATION QUATERNIONS COMPARISON")
    print("="*70)
    
    print(f"\nNED_ENU_Q (Mavros):    {mavros.NED_ENU_Q}")
    print(f"q_world_transform (Mine): {my_impl.q_world}")
    q_world_match = compare_quaternions(mavros.NED_ENU_Q, my_impl.q_world)
    print(f"Match: {q_world_match} ✅" if q_world_match else f"Match: {q_world_match} ❌")
    
    print(f"\nAIRCRAFT_BASELINK_Q (Mavros): {mavros.AIRCRAFT_BASELINK_Q}")
    print(f"q_body_transform (Mine):      {my_impl.q_body}")
    q_body_match = compare_quaternions(mavros.AIRCRAFT_BASELINK_Q, my_impl.q_body)
    print(f"Match: {q_body_match} ✅" if q_body_match else f"Match: {q_body_match} ❌")
    
    # Run test cases
    test_results = []
    
    # Test 1: Identity quaternion (no rotation)
    q_identity = np.array([1.0, 0.0, 0.0, 0.0])
    result = test_case(
        "Identity Quaternion",
        np.array([1.0, 2.0, 3.0]),
        q_identity,
        mavros, my_impl
    )
    test_results.append(result)
    
    # Test 2: 90 degree yaw
    r_90yaw = R.from_euler('z', 90, degrees=True)
    q_90yaw = r_90yaw.as_quat()
    q_90yaw = np.array([q_90yaw[3], q_90yaw[0], q_90yaw[1], q_90yaw[2]])
    result = test_case(
        "90° Yaw Rotation",
        np.array([5.0, 3.0, 1.5]),
        q_90yaw,
        mavros, my_impl
    )
    test_results.append(result)
    
    # Test 3: 45 degree roll, pitch, yaw
    r_complex = R.from_euler('xyz', [45, 30, 60], degrees=True)
    q_complex = r_complex.as_quat()
    q_complex = np.array([q_complex[3], q_complex[0], q_complex[1], q_complex[2]])
    result = test_case(
        "Complex Rotation (45° roll, 30° pitch, 60° yaw)",
        np.array([2.5, 1.8, 2.2]),
        q_complex,
        mavros, my_impl
    )
    test_results.append(result)
    
    # Test 4: Near-gimbal-lock (pitch ≈ 90°)
    r_gimbal = R.from_euler('xyz', [10, 85, 20], degrees=True)
    q_gimbal = r_gimbal.as_quat()
    q_gimbal = np.array([q_gimbal[3], q_gimbal[0], q_gimbal[1], q_gimbal[2]])
    result = test_case(
        "Near Gimbal Lock (10° roll, 85° pitch, 20° yaw)",
        np.array([0.5, -1.2, 3.8]),
        q_gimbal,
        mavros, my_impl
    )
    test_results.append(result)
    
    # Test 5: Random orientation
    r_random = R.random(random_state=42)
    q_random = r_random.as_quat()
    q_random = np.array([q_random[3], q_random[0], q_random[1], q_random[2]])
    result = test_case(
        "Random Orientation",
        np.array([-2.3, 4.1, -0.8]),
        q_random,
        mavros, my_impl
    )
    test_results.append(result)
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"\nTotal tests: {len(test_results)}")
    print(f"Passed: {sum(test_results)}")
    print(f"Failed: {len(test_results) - sum(test_results)}")
    
    if all(test_results):
        print("\n🎉 ALL TESTS PASSED! 🎉")
        print("Mavros and my implementation produce IDENTICAL results!")
    else:
        print("\n⚠️ SOME TESTS FAILED")
        print("There are differences between the implementations.")
    
    print("\n" + "="*70)
    print("CONCLUSION")
    print("="*70)



if __name__ == "__main__":
    main()