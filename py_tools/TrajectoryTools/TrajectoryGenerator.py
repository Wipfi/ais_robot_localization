import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import dill
from tkinter import Tk, filedialog
import utils
from evo.core.trajectory import PoseTrajectory3D


# Parameters
MAX_VELOCITY = 3.0  # m/s
#MAX_ACCELERATION = 1.0  # m/s²
SAMPLING_RATE = 10  # Hz
QUIVER_SPACING = 5.0  # Spacing between quiver arrows in meters
is_3d_mode = False  # Start in 2D mode

# Global variables to store clicked points
trajectory: PoseTrajectory3D = None
points = []  # Stores (x, y, z, yaw, roll, pitch, velocity) points
z_value = 0.0  # Default Z value for new points
yaw_value = 0.0  # Default yaw value for new points
roll_value = 0.0  # Default roll value for new points
pitch_value = 0.0  # Default pitch value for new points
velocity_value = MAX_VELOCITY  # Default velocity for new points

# Initialize ax_trajectory globally
ax_trajectory = None  # Set to None initially


def update_z_value(text):
    """Update the Z value for the next point."""
    global z_value
    try:
        z_value = float(text)
    except ValueError:
        print("Invalid Z value entered. Using previous Z value.")


def update_yaw_value(text):
    """Update the yaw value for the next point."""
    global yaw_value
    try:
        yaw_value = float(text)
    except ValueError:
        print("Invalid yaw value entered. Using previous yaw value.")


def update_roll_value(text):
    """Update the roll value for the next point."""
    global roll_value
    try:
        roll_value = float(text)
    except ValueError:
        print("Invalid roll value entered. Using previous roll value.")


def update_pitch_value(text):
    """Update the pitch value for the next point."""
    global pitch_value
    try:
        pitch_value = float(text)
    except ValueError:
        print("Invalid pitch value entered. Using previous pitch value.")


def toggle_view_mode(event):
    global trajectory
    global is_3d_mode
    global ax_trajectory

    if trajectory is None:
        return

    is_3d_mode = not is_3d_mode

    ax_trajectory.clear()
    fig.delaxes(ax_trajectory)

    if is_3d_mode:
        ax_trajectory = fig.add_subplot(grid[0:2, 0], projection='3d')
    else:
        ax_trajectory = fig.add_subplot(grid[0:2, 0])

    
    utils.plot_trajectory(fig, ax_trajectory, trajectory, is_3d_mode, QUIVER_SPACING)

    fig.canvas.draw()



# Add a global flag for yaw input usage
use_yaw_input = True  # Default is to use yaw input
use_pitch_input = True  # Default is to use yaw input

def toggle_yaw_input(event):
    """Toggle the usage of yaw input."""
    global use_yaw_input
    use_yaw_input = not use_yaw_input
    state = "enabled" if use_yaw_input else "disabled"
    print(f"Yaw input {state}")

def toggle_pitch_input(event):
    """Toggle the usage of pitch input."""
    global use_pitch_input
    use_pitch_input = not use_pitch_input
    state = "enabled" if use_pitch_input else "disabled"
    print(f"Pitch input {state}")



# Function to update velocity
def update_velocity_value(text):
    """Update the velocity value for the next point."""
    global velocity_value
    try:
        velocity_value = float(text)
        if velocity_value > MAX_VELOCITY:
            print(f"Warning: Velocity exceeds MAX_VELOCITY ({MAX_VELOCITY} m/s). Capping to MAX_VELOCITY.")
            velocity_value = MAX_VELOCITY
    except ValueError:
        print("Invalid velocity value entered. Using previous velocity value.")

def onclick(event):
    """Callback to capture points when clicked on the plot."""
    if is_3d_mode:
        return  # Disable adding points in 3D mode

    if event.button == 1 and event.dblclick:  # Add points only on double-click
        if event.inaxes == ax_trajectory:  # Ensure the click is on the trajectory plot
            x, y = event.xdata, event.ydata

            # Add point with the current Z, yaw, roll, pitch, and velocity values
            points.append((x, y, z_value, yaw_value, roll_value, pitch_value, velocity_value))
            print(f"Added point: ({x:.2f}, {y:.2f}, {z_value:.2f}, {yaw_value:.2f}, {roll_value:.2f}, {pitch_value:.2f}, {velocity_value:.2f} m/s)")

            # Save current axis limits
            xlim = ax_trajectory.get_xlim()
            ylim = ax_trajectory.get_ylim()

            # Plot the new point
            ax_trajectory.scatter(x, y, color='red', zorder=5)  # Mark the point on the 2D plot

            # Restore axis limits
            ax_trajectory.set_xlim(xlim)
            ax_trajectory.set_ylim(ylim)

            fig.canvas.draw()

def clear_plots():
    ax_trajectory.clear()
    ax_velocity.clear()
    ax_height.clear()

def update_plots():
    clear_plots()
    utils.plot_trajectory(fig, ax_trajectory, trajectory, is_3d_mode=is_3d_mode, quiver_spacing=QUIVER_SPACING)
    utils.plot_velocity_profile(ax=ax_velocity, trajectory=trajectory)
    utils.plot_height_profile(ax=ax_height, trajectory=trajectory)


def generate_trajectory(event):
    """Generate the smooth trajectory and plot."""
    global trajectory, points, use_pitch_input, use_yaw_input

    trajectory = utils.mathUtils.trajectory_generation(points, use_yaw_input=use_yaw_input, use_pitch_input=use_pitch_input, sampling_rate=SAMPLING_RATE)

    update_plots()

    print(trajectory.get_statistics())
    print(trajectory.get_infos())

    fig.canvas.draw()


def clear_points(event):
    """Clear all points and trajectories."""
    global points, trajectory, is_3d_mode

    if is_3d_mode:
        toggle_view_mode(None)

    clear_plots() 

    utils.setup_trajectory_2d_plot(ax=ax_trajectory)

    points = []
    trajectory = None

    print("Points cleared")

    fig.canvas.draw()


def export_trajectory():
    """Export the trajectory as an evo PoseTrajectory3D object and save it using dill."""
    global trajectory
    if trajectory is None:
        print("No trajectory to export.")
        return

    # File save dialog
    root = Tk()
    root.withdraw()  # Hide the main tkinter window
    file_path = filedialog.asksaveasfilename(
        defaultextension=".dill",
        filetypes=[("Dill files", "*.dill"), ("All files", "*.*")],
        title="Save Trajectory"
    )
    if file_path:
        with open(file_path, 'wb') as file:
            dill.dump(trajectory, file)
        print(f"Trajectory saved to {file_path}.")
    else:
        print("Export canceled.")


def generate_random_points(event):
    global points

    random_points = utils.generate_random_trajectory()

    for point, velocity_value in random_points:
        points.append((point[0], point[1], point[2], yaw_value, roll_value, pitch_value, velocity_value))


    # Save current axis limits
    xlim = ax_trajectory.get_xlim()
    ylim = ax_trajectory.get_ylim()

    # Plot the new point
    ax_trajectory.scatter(*zip(*[(p[0], p[1]) for p in  points]), zorder=5)

    # Restore axis limits
    ax_trajectory.set_xlim(xlim)
    ax_trajectory.set_ylim(ylim)

    fig.canvas.draw()



# Create the Matplotlib figure and subplots
fig = plt.figure(figsize=(14, 8))
grid = fig.add_gridspec(3, 2, width_ratios=[2, 1], height_ratios=[2, 1, 1])

ax_trajectory = fig.add_subplot(grid[0:2, 0], projection='3d' if is_3d_mode else None)
ax_velocity = fig.add_subplot(grid[0, 1])
ax_height = fig.add_subplot(grid[1, 1])
#ax_elements = fig.add_subplot(grid[2, 0])


# Add buttons and inputs
btn_toggle_view = Button(plt.axes([0.6, 0.15, 0.15, 0.04]), 'Toggle 2D/3D')
btn_random_points = Button(plt.axes([0.6, 0.08, 0.15, 0.04]), 'Generate Random Points')
btn_generate = Button(plt.axes([0.6, 0.02, 0.15, 0.04]), 'Generate')
btn_clear = Button(plt.axes([0.8, 0.02, 0.15, 0.04]), 'Clear')
btn_toggle_yaw = Button(plt.axes([0.8, 0.08, 0.15, 0.04]), 'Toggle Yaw Input')
btn_toggle_pitch = Button(plt.axes([0.8, 0.12, 0.15, 0.04]), 'Toggle Pitch Input')
text_box_z = TextBox(plt.axes([0.4, 0.02, 0.15, 0.04]), "Set Height (Z):", initial=str(z_value))
text_box_yaw = TextBox(plt.axes([0.15, 0.02, 0.15, 0.04]), "Set Yaw (°):", initial=str(yaw_value))
text_box_roll = TextBox(plt.axes([0.15, 0.07, 0.15, 0.04]), "Set Roll (°):", initial=str(roll_value))
text_box_pitch = TextBox(plt.axes([0.4, 0.07, 0.15, 0.04]), "Set Pitch (°):", initial=str(pitch_value))
text_box_velocity = TextBox(plt.axes([0.4, 0.12, 0.15, 0.04]), "Set Velocity (m/s):", initial=str(velocity_value))
btn_export = Button(plt.axes([0.6, 0.2, 0.15, 0.04]), 'Export')


# Connect callbacks
btn_toggle_view.on_clicked(toggle_view_mode)
btn_toggle_yaw.on_clicked(toggle_yaw_input)
btn_toggle_pitch.on_clicked(toggle_pitch_input)
btn_generate.on_clicked(generate_trajectory)
btn_clear.on_clicked(clear_points)
btn_random_points.on_clicked(generate_random_points)
btn_export.on_clicked(lambda event: export_trajectory())
text_box_z.on_submit(update_z_value)
text_box_yaw.on_submit(update_yaw_value)
text_box_roll.on_submit(update_roll_value)
text_box_pitch.on_submit(update_pitch_value)
text_box_velocity.on_submit(update_velocity_value)
fig.canvas.mpl_connect('button_press_event', onclick)

utils.setup_trajectory_2d_plot(ax=ax_trajectory)
utils.setup_velocity_plot(ax=ax_velocity)
utils.setup_height_plot(ax=ax_height)

plt.tight_layout()
plt.show()
