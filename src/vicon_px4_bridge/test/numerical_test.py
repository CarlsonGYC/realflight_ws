#!/usr/bin/env python3
"""
Verification script for coordinate frame transformations.
This script helps verify that the C++ transformations are correct.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

def get_transformation_matrix(from_frame, to_frame):
    """
    Get transformation matrix from one frame to another.
    Returns 3x3 rotation matrix.
    """
    if from_frame == to_frame:
        return np.eye(3)
    
    transformations = {
        ('ENU', 'NED'): np.array([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1]
        ]),
        ('NED', 'ENU'): np.array([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1]
        ]),
        ('FLU', 'FRD'): np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ]),
        ('FRD', 'FLU'): np.array([
            [1,  0,  0],
            [0, -1,  0],
            [0,  0, -1]
        ]),
    }
    
    key = (from_frame, to_frame)
    if key in transformations:
        return transformations[key]
    else:
        raise ValueError(f"Unknown transformation: {from_frame} -> {to_frame}")


def transform_pose(position, quaternion, 
                   input_world_frame, output_world_frame,
                   input_body_frame, output_body_frame):
    """
    Transform a pose from input frames to output frames.
    
    Args:
        position: [x, y, z] position in input world frame
        quaternion: [w, x, y, z] orientation quaternion
        input_world_frame: e.g., "ENU"
        output_world_frame: e.g., "NED"
        input_body_frame: e.g., "FLU"
        output_body_frame: e.g., "FRD"
    
    Returns:
        position_out, quaternion_out
    """
    
    # Get transformation matrices
    R_world = get_transformation_matrix(input_world_frame, output_world_frame)
    R_body = get_transformation_matrix(input_body_frame, output_body_frame)
    
    # Transform position
    position_out = R_world @ position
    
    # Transform orientation
    # Create rotation objects
    r_in = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])  # scipy uses [x,y,z,w]
    r_world = R.from_matrix(R_world)
    r_body = R.from_matrix(R_body)
    
    # Apply double transformation: q_out = q_world * q_in * q_body
    r_out = r_world * r_in * r_body
    
    # Convert back to quaternion [w, x, y, z]
    quat_scipy = r_out.as_quat()  # [x, y, z, w]
    quaternion_out = np.array([quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]])  # [w, x, y, z]
    
    return position_out, quaternion_out


def test_identity():
    """Test that identity orientation stays as frame transformation only."""
    print("\n=== Test 1: Identity Quaternion ===")
    
    position = np.array([1.0, 2.0, 3.0])
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Identity [w, x, y, z]
    
    pos_out, quat_out = transform_pose(
        position, quaternion,
        "ENU", "NED",
        "FLU", "FRD"
    )
    
    print(f"Input position (ENU): {position}")
    print(f"Output position (NED): {pos_out}")
    print(f"Expected: [2.0, 1.0, -3.0] (Y, X, -Z)")
    
    print(f"\nInput quaternion (FLU): {quaternion}")
    print(f"Output quaternion (FRD): {quat_out}")
    
    # For identity input, output should be the combined frame transformation
    R_expected = get_transformation_matrix("ENU", "NED") @ get_transformation_matrix("FLU", "FRD")
    r_expected = R.from_matrix(R_expected)
    quat_expected_scipy = r_expected.as_quat()
    quat_expected = np.array([quat_expected_scipy[3], quat_expected_scipy[0], 
                             quat_expected_scipy[1], quat_expected_scipy[2]])
    print(f"Expected quaternion: {quat_expected}")
    
    return np.allclose(pos_out, [2.0, 1.0, -3.0]) and np.allclose(quat_out, quat_expected)


def test_90deg_yaw():
    """Test 90 degree yaw rotation."""
    print("\n=== Test 2: 90° Yaw Rotation ===")
    
    position = np.array([1.0, 0.0, 0.0])
    
    # 90 degree rotation about Z axis in ENU/FLU
    # This is [cos(45°), 0, 0, sin(45°)] = [0.707, 0, 0, 0.707]
    r = R.from_euler('z', 90, degrees=True)
    quat_scipy = r.as_quat()
    quaternion = np.array([quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]])
    
    pos_out, quat_out = transform_pose(
        position, quaternion,
        "ENU", "NED",
        "FLU", "FRD"
    )
    
    print(f"Input position (ENU): {position}")
    print(f"Output position (NED): {pos_out}")
    
    print(f"\nInput quaternion (FLU, 90° yaw): {quaternion}")
    print(f"Output quaternion (FRD): {quat_out}")
    
    # Convert output quaternion to euler angles for interpretation
    r_out = R.from_quat([quat_out[1], quat_out[2], quat_out[3], quat_out[0]])
    euler_out = r_out.as_euler('xyz', degrees=True)
    print(f"Output as Euler angles (degrees): roll={euler_out[0]:.1f}, pitch={euler_out[1]:.1f}, yaw={euler_out[2]:.1f}")


def test_custom_pose(position, roll, pitch, yaw, 
                     input_world="ENU", output_world="NED",
                     input_body="FLU", output_body="FRD"):
    """Test a custom pose with given Euler angles."""
    print(f"\n=== Custom Test: {input_world}/{input_body} -> {output_world}/{output_body} ===")
    
    # Create quaternion from Euler angles
    r = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    quat_scipy = r.as_quat()
    quaternion = np.array([quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]])
    
    print(f"Input position ({input_world}): {position}")
    print(f"Input orientation ({input_body}): roll={roll}°, pitch={pitch}°, yaw={yaw}°")
    print(f"Input quaternion: {quaternion}")
    
    pos_out, quat_out = transform_pose(
        position, quaternion,
        input_world, output_world,
        input_body, output_body
    )
    
    print(f"\nOutput position ({output_world}): {pos_out}")
    print(f"Output quaternion ({output_body}): {quat_out}")
    
    # Convert to Euler for interpretation
    r_out = R.from_quat([quat_out[1], quat_out[2], quat_out[3], quat_out[0]])
    euler_out = r_out.as_euler('xyz', degrees=True)
    print(f"Output as Euler angles: roll={euler_out[0]:.1f}°, pitch={euler_out[1]:.1f}°, yaw={euler_out[2]:.1f}°")
    
    return pos_out, quat_out


def verify_transformation_properties():
    """Verify that transformation matrices have proper rotation properties."""
    print("\n=== Verifying Transformation Matrix Properties ===")
    
    frames = [("ENU", "NED"), ("FLU", "FRD")]
    
    for from_f, to_f in frames:
        R_mat = get_transformation_matrix(from_f, to_f)
        
        print(f"\n{from_f} -> {to_f}:")
        print(f"Matrix:\n{R_mat}")
        
        # Check determinant = 1 (proper rotation)
        det = np.linalg.det(R_mat)
        print(f"Determinant: {det:.6f} (should be 1.0)")
        
        # Check orthogonality: R^T * R = I
        identity_check = R_mat.T @ R_mat
        is_orthogonal = np.allclose(identity_check, np.eye(3))
        print(f"Orthogonal: {is_orthogonal}")
        
        # Check if inverse equals transpose
        R_inv = np.linalg.inv(R_mat)
        is_inverse_transpose = np.allclose(R_inv, R_mat.T)
        print(f"Inverse = Transpose: {is_inverse_transpose}")


def compare_with_old_method(position, quaternion,
                            input_world="ENU", output_world="NED",
                            input_body="FLU", output_body="FRD"):
    """
    Compare correct method with the old incorrect method.
    """
    print("\n=== Comparing Correct vs Incorrect Methods ===")
    
    # Correct method
    pos_correct, quat_correct = transform_pose(
        position, quaternion,
        input_world, output_world,
        input_body, output_body
    )
    
    # Old incorrect method (similarity transformation)
    R_transform = get_transformation_matrix(input_world, output_world)
    r_transform = R.from_matrix(R_transform)
    r_in = R.from_quat([quaternion[1], quaternion[2], quaternion[3], quaternion[0]])
    
    # Old method: q_out = q_transform * q_in * q_transform^-1
    r_out_old = r_transform * r_in * r_transform.inv()
    quat_scipy_old = r_out_old.as_quat()
    quat_old = np.array([quat_scipy_old[3], quat_scipy_old[0], 
                        quat_scipy_old[1], quat_scipy_old[2]])
    
    print("Input:")
    print(f"  Position: {position}")
    print(f"  Quaternion: {quaternion}")
    
    print("\nCorrect Method (double transformation):")
    print(f"  Position: {pos_correct}")
    print(f"  Quaternion: {quat_correct}")
    
    print("\nOld Incorrect Method (similarity transformation):")
    print(f"  Position: {R_transform @ position}")
    print(f"  Quaternion: {quat_old}")
    
    print("\nDifference:")
    print(f"  Quaternion diff: {quat_correct - quat_old}")

def rotation_matrix_to_euler(R):
    assert R.shape == (3, 3), "Input matrix must be 3x3"

    # Check numerical stability (clamp value)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    singular = sy < 1e-6
    if not singular:
        roll  = np.arctan2(R[2, 1], R[2, 2])    # around x
        pitch = np.arctan2(-R[2, 0], sy)        # around y
        yaw   = np.arctan2(R[1, 0], R[0, 0])    # around z
    else:
        # Gimbal lock
        roll  = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = 0

    # Normalize to [-pi, pi]
    def wrap_to_pi(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    roll  = wrap_to_pi(roll)
    pitch = wrap_to_pi(pitch)
    yaw   = wrap_to_pi(yaw)

    return roll, pitch, yaw

def rotation_matrix_to_quaternion(R):
    assert R.shape == (3, 3), "Input matrix must be 3x3"

    trace = np.trace(R)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    q = np.array([qw, qx, qy, qz])
    q /= np.linalg.norm(q)  # Normalize
    return q


if __name__ == "__main__":
    R =  np.eye(3)
    roll, pitch, yaw = rotation_matrix_to_euler(R)
    print(f"Roll: {np.degrees(roll):.2f}°, Pitch: {np.degrees(pitch):.2f}°, Yaw: {np.degrees(yaw):.2f}°")
    theta = -np.pi / 2  # 顺时针90° = -90°
    Rz = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1]
    ])
    roll, pitch, yaw = rotation_matrix_to_euler(Rz)
    print(f"Roll: {np.degrees(roll):.2f}°, Pitch: {np.degrees(pitch):.2f}°, Yaw: {np.degrees(yaw):.2f}°")

    R_ENU2NED = get_transformation_matrix("ENU", "NED")
    R_FLU2FRD = get_transformation_matrix("FLU", "FRD")
    R_total = R_ENU2NED @ Rz @ R_FLU2FRD
    roll, pitch, yaw = rotation_matrix_to_euler(R_total)
    print(f"Roll: {np.degrees(roll):.2f}°, Pitch: {np.degrees(pitch):.2f}°, Yaw: {np.degrees(yaw):.2f}°")
    quat_correct = rotation_matrix_to_quaternion(R_total)
    print(f"Quaternion: {quat_correct}")