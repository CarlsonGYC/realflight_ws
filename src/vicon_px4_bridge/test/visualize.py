#!/usr/bin/env python3
"""
Visualization script for coordinate frame transformations.
Requires: matplotlib, numpy, scipy
Install: pip install matplotlib numpy scipy
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
# Get transformation matrices
from numerical_test import get_transformation_matrix, transform_pose


def plot_frame(ax, origin, rotation_matrix, frame_name, scale=1.0, alpha=1.0):
    """
    Plot a coordinate frame as three axes (RGB = XYZ).
    
    Args:
        ax: matplotlib 3D axis
        origin: [x, y, z] origin of the frame
        rotation_matrix: 3x3 rotation matrix
        frame_name: Name to display
        scale: Scale of the axes
        alpha: Transparency
    """
    colors = ['r', 'g', 'b']
    labels = ['X', 'Y', 'Z']
    
    for i, (color, label) in enumerate(zip(colors, labels)):
        direction = rotation_matrix[:, i] * scale
        ax.quiver(origin[0], origin[1], origin[2],
                 direction[0], direction[1], direction[2],
                 color=color, alpha=alpha, linewidth=2,
                 arrow_length_ratio=0.3)
        
        # Add label at the end of arrow
        end_point = origin + direction
        ax.text(end_point[0], end_point[1], end_point[2],
               f'{frame_name}-{label}', color=color, fontsize=8)


def visualize_transformation(input_world, output_world, 
                            input_body, output_body,
                            position, roll, pitch, yaw):
    """
    Visualize the complete transformation process.
    """
    fig = plt.figure(figsize=(15, 10))
    
    R_world = get_transformation_matrix(input_world, output_world)
    R_body = get_transformation_matrix(input_body, output_body)
    
    # Create rotation from Euler angles
    r_euler = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    quat_scipy = r_euler.as_quat()
    quaternion = np.array([quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]])
    
    # Get input rotation matrix
    R_in = r_euler.as_matrix()
    
    # Calculate output
    pos_out, quat_out = transform_pose(
        position, quaternion,
        input_world, output_world,
        input_body, output_body
    )
    
    # Get output rotation matrix
    r_out = R.from_quat([quat_out[1], quat_out[2], quat_out[3], quat_out[0]])
    R_out = r_out.as_matrix()
    
    # --- Plot 1: Input World Frame and Body Frame ---
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.set_title(f'Input: {input_world} World + {input_body} Body')
    
    # Plot world frame at origin
    plot_frame(ax1, np.zeros(3), np.eye(3), input_world, scale=2.0)
    
    # Plot body frame at position with input orientation
    plot_frame(ax1, position, R_in, input_body, scale=1.5, alpha=0.8)
    
    # Plot position vector
    ax1.plot([0, position[0]], [0, position[1]], [0, position[2]], 
            'k--', linewidth=1, label='Position')
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.legend()
    set_axes_equal(ax1)
    
    # --- Plot 2: World Frame Transformation ---
    ax2 = fig.add_subplot(222, projection='3d')
    ax2.set_title(f'After World Transform: {output_world}')
    
    # Plot transformed world frame
    plot_frame(ax2, np.zeros(3), R_world, output_world, scale=2.0)
    
    # Show original world frame (faded)
    plot_frame(ax2, np.zeros(3), np.eye(3), f'{input_world} (old)', 
              scale=2.0, alpha=0.3)
    
    # Transformed position (world transform only)
    pos_world_only = R_world @ position
    plot_frame(ax2, pos_world_only, R_world @ R_in, 
              'Partial', scale=1.5, alpha=0.8)
    
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    set_axes_equal(ax2)
    
    # --- Plot 3: Body Frame Transformation ---
    ax3 = fig.add_subplot(223, projection='3d')
    ax3.set_title(f'After Body Transform: {output_body}')
    
    # Plot output world frame
    plot_frame(ax3, np.zeros(3), R_world, output_world, scale=2.0)
    
    # Complete transformation
    R_complete = R_world @ R_in @ R_body
    plot_frame(ax3, pos_out, R_complete, output_body, scale=1.5, alpha=0.8)
    
    # Show intermediate (faded)
    plot_frame(ax3, pos_world_only, R_world @ R_in, 
              'Before body', scale=1.5, alpha=0.3)
    
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    set_axes_equal(ax3)
    
    # --- Plot 4: Complete Comparison ---
    ax4 = fig.add_subplot(224, projection='3d')
    ax4.set_title('Complete Transformation')
    
    # Input system (faded)
    plot_frame(ax4, np.zeros(3), np.eye(3), f'{input_world} (in)', 
              scale=2.0, alpha=0.3)
    plot_frame(ax4, position, R_in, f'{input_body} (in)', 
              scale=1.5, alpha=0.3)
    
    # Output system (solid)
    plot_frame(ax4, np.zeros(3), R_world, f'{output_world} (out)', 
              scale=2.0, alpha=1.0)
    plot_frame(ax4, pos_out, R_complete, f'{output_body} (out)', 
              scale=1.5, alpha=1.0)
    
    # Position vectors
    ax4.plot([0, position[0]], [0, position[1]], [0, position[2]], 
            'k--', linewidth=1, alpha=0.3, label='Input pos')
    ax4.plot([0, pos_out[0]], [0, pos_out[1]], [0, pos_out[2]], 
            'k-', linewidth=2, label='Output pos')
    
    ax4.set_xlabel('X')
    ax4.set_ylabel('Y')
    ax4.set_zlabel('Z')
    ax4.legend()
    set_axes_equal(ax4)
    
    plt.tight_layout()
    
    # Print numerical results
    print("\n" + "="*60)
    print("TRANSFORMATION RESULTS")
    print("="*60)
    print(f"\nInput Configuration:")
    print(f"  World Frame: {input_world}")
    print(f"  Body Frame: {input_body}")
    print(f"  Position: {position}")
    print(f"  Orientation (Euler): roll={roll}°, pitch={pitch}°, yaw={yaw}°")
    print(f"  Orientation (Quat): {quaternion}")
    
    print(f"\nOutput Configuration:")
    print(f"  World Frame: {output_world}")
    print(f"  Body Frame: {output_body}")
    print(f"  Position: {pos_out}")
    print(f"  Orientation (Quat): {quat_out}")
    
    euler_out = r_out.as_euler('xyz', degrees=True)
    print(f"  Orientation (Euler): roll={euler_out[0]:.1f}°, pitch={euler_out[1]:.1f}°, yaw={euler_out[2]:.1f}°")
    
    print("\n" + "="*60)
    
    plt.show()


def set_axes_equal(ax):
    """Set 3D plot axes to equal scale."""
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    
    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])


def visualize_frame_comparison():
    """Visualize different frame conventions side by side."""
    fig = plt.figure(figsize=(15, 5))
    
    frames = [
        ("ENU", np.eye(3), "East-North-Up"),
        ("NED", np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]]), "North-East-Down"),
        ("FLU", np.eye(3), "Forward-Left-Up"),
        ("FRD", np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]), "Forward-Right-Down"),
    ]
    
    for idx, (name, R_mat, description) in enumerate(frames):
        ax = fig.add_subplot(1, 4, idx+1, projection='3d')
        ax.set_title(f'{name}\n{description}')
        
        plot_frame(ax, np.zeros(3), R_mat, name, scale=1.0)
        
        # Add axis labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        # Set equal aspect ratio
        ax.set_xlim([-1.5, 1.5])
        ax.set_ylim([-1.5, 1.5])
        ax.set_zlim([-1.5, 1.5])
        
        # Add text annotations
        if name == "ENU":
            ax.text(1.2, 0, -1.2, "X = East", fontsize=8)
            ax.text(0, 1.2, -1.2, "Y = North", fontsize=8)
            ax.text(-1.2, 0, 1.2, "Z = Up", fontsize=8)
        elif name == "NED":
            ax.text(1.2, 0, -1.2, "X = North", fontsize=8)
            ax.text(0, 1.2, -1.2, "Y = East", fontsize=8)
            ax.text(-1.2, 0, 1.2, "Z = Down", fontsize=8)
        elif name == "FLU":
            ax.text(1.2, 0, -1.2, "X = Forward", fontsize=8)
            ax.text(0, 1.2, -1.2, "Y = Left", fontsize=8)
            ax.text(-1.2, 0, 1.2, "Z = Up", fontsize=8)
        elif name == "FRD":
            ax.text(1.2, 0, -1.2, "X = Forward", fontsize=8)
            ax.text(0, 1.2, -1.2, "Y = Right", fontsize=8)
            ax.text(-1.2, 0, 1.2, "Z = Down", fontsize=8)
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    print("Coordinate Frame Transformation Visualization")
    print("=" * 60)
    
    # First, show all frame conventions
    print("\n1. Showing all frame conventions...")
    visualize_frame_comparison()
    
    # Then show a complete transformation example
    print("\n2. Showing complete transformation: ENU/FLU -> NED/FRD")
    print("   Position: [2, 3, 1.5]")
    print("   Orientation: roll=10°, pitch=15°, yaw=45°")
    
    visualize_transformation(
        input_world="ENU",
        output_world="NED",
        input_body="FLU",
        output_body="FRD",
        position=np.array([2.0, 3.0, 1.5]),
        roll=10.0,
        pitch=15.0,
        yaw=45.0
    )
    
    print("\nVisualization complete!")