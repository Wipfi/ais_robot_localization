import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from tkinter import filedialog
import dill
import os
from evo.core.trajectory import PoseTrajectory3D
import mathUtils

# File Utilities
def load_trajectory(file_path=None):
    """Load a trajectory file using dill."""
    if not file_path:
        file_path = filedialog.askopenfilename(
            title="Select Trajectory File",
            filetypes=[("Dill files", "*.dill"), ("All files", "*.*")]
        )
    
    with open(file_path, 'rb') as file:
        trajectory = dill.load(file)
        name = os.path.splitext(os.path.basename(file.name))[0]
    
    return trajectory, name

def export_trajectory(trajectory, file_path=None):
    """Export a trajectory to a dill file."""
    if not file_path:
        file_path = filedialog.asksaveasfilename(
            title="Save Trajectory File",
            defaultextension=".dill",
            filetypes=[("Dill files", "*.dill"), ("All files", "*.*")]
        )

    with open(file_path, 'wb') as file:
        dill.dump(trajectory, file)

# Visualization Utilities
def plot_quivers(ax, x, y, yaw, spacing=5.0):
    """Plot quivers (arrows) to represent yaw on a trajectory."""
    subsampled_indices = [0]
    accumulated_distance = 0.0

    for i in range(1, len(x)):
        dx = x[i] - x[i - 1]
        dy = y[i] - y[i - 1]
        accumulated_distance += np.sqrt(dx**2 + dy**2)
        if accumulated_distance >= spacing:
            subsampled_indices.append(i)
            accumulated_distance = 0.0

    quiver_x = np.array(x)[subsampled_indices]
    quiver_y = np.array(y)[subsampled_indices]
    quiver_yaw = np.array(yaw)[subsampled_indices]

    dx = np.cos(np.radians(quiver_yaw))
    dy = np.sin(np.radians(quiver_yaw))

    ax.quiver(quiver_x, quiver_y, dx, dy, angles='xy', scale_units='xy', scale=0.5, color='orange')
    #ax.quiver(quiver_x, quiver_y, dx, dy, angles='xy', scale_units='xy', scale=0.1, color='orange', width = 0.019)

def plot_velocity_profile_(trajectory):
    """Plot velocity profile for the trajectory."""
    fig, ax = plt.subplots()
    plot_velocity_profile(ax=ax, trajectory=trajectory)
    plt.show()


def plot_height_profile_(trajectory):
    """Plot height profile for the trajectory."""
    fig, ax = plt.subplots()
    plot_height_profile(ax=ax, trajectory=trajectory)
    plt.show()

def plot_attitude_3d(ax_trajectory, x_, y_, z_, yaw_, roll_, pitch_, spacing):

    # Subsample for attitude frames
    subsampled_indices = [0]
    accumulated_distance = 0.0
    for i in range(1, len(x_)):
        dx = x_[i] - x_[i - 1]
        dy = y_[i] - y_[i - 1]
        dz = z_[i] - z_[i - 1]
        accumulated_distance += np.sqrt(dx**2 + dy**2 + dz**2)
        if accumulated_distance >= spacing:
            subsampled_indices.append(i)
            accumulated_distance = 0.0

    for idx in subsampled_indices:
        x, y, z = x_[idx], y_[idx], z_[idx]
        yaw, roll, pitch = np.radians(yaw_[idx]), np.radians(roll_[idx]), np.radians(pitch_[idx])

        R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                          [np.sin(yaw), np.cos(yaw), 0],
                          [0, 0, 1]])
        R_pitch = np.array([[1, 0, 0],
                            [0, np.cos(pitch), -np.sin(pitch)],
                            [0, np.sin(pitch), np.cos(pitch)]])
        R_roll = np.array([[np.cos(roll), 0, np.sin(roll)],
                           [0, 1, 0],
                           [-np.sin(roll), 0, np.cos(roll)]])
        R = R_yaw @ R_pitch @ R_roll

        origin = np.array([x, y, z])
        x_axis = origin + R @ np.array([1, 0, 0])
        y_axis = origin + R @ np.array([0, 1, 0])
        z_axis = origin + R @ np.array([0, 0, 1])

        # Draw axes
        ax_trajectory.plot([origin[0], x_axis[0]], [origin[1], x_axis[1]], [origin[2], x_axis[2]], color='red')
        ax_trajectory.plot([origin[0], y_axis[0]], [origin[1], y_axis[1]], [origin[2], y_axis[2]], color='green')
        ax_trajectory.plot([origin[0], z_axis[0]], [origin[1], z_axis[1]], [origin[2], z_axis[2]], color='blue')


def plot_velocity_profile(ax, trajectory):
    if "Name" in  trajectory.meta:
        name =  trajectory.meta["Name"]
    else:
        name = None
    velocities = np.sqrt(np.sum(np.diff(trajectory.positions_xyz, axis=0)**2, axis=1)) / np.diff(trajectory.timestamps)
    ax.plot(trajectory.timestamps[:-1], velocities, label=name)
    setup_velocity_plot(ax)

def plot_height_profile(ax, trajectory):
    if "Name" in  trajectory.meta:
        name =  trajectory.meta["Name"]
    else:
        name = None
    ax.plot(trajectory.timestamps, trajectory.positions_xyz[:, 2], label=name)
    setup_height_plot(ax)

# Toggle between 2D and 3D plot
def plot_trajectory(fig, ax, trajectory: PoseTrajectory3D, is_3d_mode, quiver_spacing, autoscale = False, linestyle = 'solid'):
    """Toggle between 2D and 3D visualization of the trajectory."""

    # Extract data from trajectory
    poses = trajectory.poses_se3
    orientations = np.rad2deg(trajectory.get_orientations_euler())
    timestamps = trajectory.timestamps

    t_smooth = timestamps
    smooth_x, smooth_y, smooth_z = zip(*[pose[:3, 3] for pose in poses])
    
    smooth_yaw = [orientation[2] for orientation in orientations] 
    smooth_roll = [orientation[1] for orientation in orientations] 
    smooth_pitch = [orientation[0] for orientation in orientations] 

    if not "Name" in  trajectory.meta:
        trajectory_name = "Trajectory"
    else:
        trajectory_name = trajectory.meta["Name"]
    
    if is_3d_mode:
        setup_trajectory_3d_plot(ax, autoscale = autoscale)
        if linestyle != None:
            ax.plot(
                trajectory.positions_xyz[:, 0],
                trajectory.positions_xyz[:, 1],
                trajectory.positions_xyz[:, 2],
                label=trajectory_name, 
                linestyle= linestyle
            )
        if "base_points" in  trajectory.meta:
            ax.scatter(*zip(*[(p[0], p[1], p[2]) for p in  trajectory.meta["base_points"]]))
        plot_attitude_3d(ax, smooth_x, smooth_y, smooth_z, smooth_yaw, smooth_roll, smooth_pitch, quiver_spacing)

    else:
        setup_trajectory_2d_plot(ax, autoscale=autoscale)
        ax.plot(
            trajectory.positions_xyz[:, 0],
            trajectory.positions_xyz[:, 1],
            label=trajectory_name
        )
        if "base_points" in  trajectory.meta:
            ax.scatter(*zip(*[(p[0], p[1]) for p in  trajectory.meta["base_points"]]), zorder=5)
        plot_quivers(ax, smooth_x, smooth_y, smooth_yaw, quiver_spacing)

    #ax.legend()
    fig.canvas.draw()
    return ax

def setup_height_plot(ax):
    """Set up the height profile plot."""
    ax.set_title("Height Profile")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Height (m)")
    ax.legend()


def setup_velocity_plot(ax):
    """Set up the velocity profile plot."""
    ax.set_title("Velocity Profile")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")



def setup_trajectory_2d_plot(ax, autoscale = False):
    """Set up the 2D trajectory plot."""
    ax.set_title("2D Trajectory Visualization")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_xlim(-10,200)
    ax.set_ylim(-10,200)
    ax.set_aspect('equal', adjustable='box')

    if autoscale:
        ax.autoscale()

    ax.grid(True)


def setup_trajectory_3d_plot(ax, autoscale = False):
    """Set up the 3D trajectory plot."""
    #ax.set_title("3D Trajectory Visualization")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_xlim(-10, 200)
    ax.set_ylim(-10, 200)
    ax.set_zlim(-10, 60)
    ax.set_aspect('equal', adjustable='box')

    if autoscale:
        ax.autoscale()


def generate_random_trajectory(grid_size=(190, 190, 50), spacing=10, num_points=25, min_velocity=0.5, max_velocity=3):
    """
    Generates a random trajectory with straight sections
    - grid_size: Dimensions of the 3D grid
    - spacing: Minimum distance between points
    - num_points: Number of points to sample for the trajectory
    """
    # 1. Create grid points
    x_vals = np.arange(0, grid_size[0], spacing)
    y_vals = np.arange(0, grid_size[1], spacing)
    z_vals = np.arange(0, grid_size[2], spacing / 10)
    grid_points = np.array(np.meshgrid(x_vals, y_vals, z_vals)).T.reshape(-1, 3)
    
    # 2. Initialize trajectory points
    base_points = []
    points = []
    start_point = grid_points[np.random.randint(len(grid_points))]
    prev_dir = np.array([1, 0, 0])  # Initial direction (arbitrary)
    current_velocity = np.random.uniform(min_velocity, max_velocity)
    base_points.append((start_point, current_velocity))
    points.append(start_point)


    # 3. Select random points with momentum constraint
    for _ in range(num_points - 1):
        # Filter candidate points
        candidates = [p for p in grid_points if np.linalg.norm(p - points[-1]) >= spacing]

        # Exclude points where only the z value changes or very high z value changes
        candidates = [p for p in candidates if not np.array_equal(p[:2], points[-1][:2])]

        candidates = [p for p in candidates if np.abs(p[2]-points[-1][2]) < 5.0]

        # Bias selection towards the previous direction
        def direction_bias(point):
            dir_vec = point - points[-1]
            dir_vec = dir_vec.astype(float)  # Convert to float for safe division
            dir_vec /= np.linalg.norm(dir_vec)  # Normalize

            return np.dot(dir_vec, prev_dir)   # Cosine similarity with prev_dir

        # Randomly decide whether to ignore the bias
        if np.random.random() < 0.2:  # 20% chance (1/5) to ignore bias
            next_point = candidates[np.random.randint(0, len(candidates))]  # Random choice
        else:
            # Apply the direction bias
            candidates.sort(key=direction_bias, reverse=True)
            next_point = candidates[np.random.randint(0, min(5, len(candidates)))]

        points.append(next_point)
        
        # Adjust velocity with momentum (favor similar velocities)
        velocity_change = np.random.uniform(-0.2, 0.2)  # Small random perturbation
        current_velocity = max(min_velocity, min(max_velocity, current_velocity + velocity_change))

        base_points.append((next_point, current_velocity))
        
        # Update direction
        prev_dir = next_point - points[-2]
        prev_dir = prev_dir.astype(float)
        prev_dir /= np.linalg.norm(prev_dir)


    return base_points


