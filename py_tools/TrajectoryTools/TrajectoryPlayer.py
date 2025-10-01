import rospy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, QuaternionStamped, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from utils import load_trajectory, plot_trajectory, plot_velocity_profile, plot_height_profile, plot_attitude_3d   # Import utilities
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, CheckButtons, TextBox, RadioButtons
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import threading
import argparse
from tkinter import Tk
import time

# Global variables
trajectories = []
is_publishing = False

QUIVER_SPACING = 5.0
is_3d_mode = False  # Toggle between 2D and 3D plots
selected_trajectory_index = None  # Index of the selected trajectory

# Initialize ROS Node on script start
rospy.init_node("trajectory_publisher", anonymous=True)

# Set up Transform Broadcaster
def setup_transform_broadcaster():
    """Create and return a TransformBroadcaster."""
    return TransformBroadcaster()

def start_odom_publisher(odom_topic_name):
    """Set up and return an Odometry publisher."""
    return rospy.Publisher(odom_topic_name, Odometry, queue_size=10)

def start_quaternion_publisher(imu_topic_name):
    """Set up and return an Odometry publisher."""
    return rospy.Publisher(imu_topic_name, QuaternionStamped, queue_size=10)

def update_odom_topic_name(text):
    """Update the odom topic name."""
    global selected_trajectory_index
    global trajectories
    trajectories[selected_trajectory_index].meta["topic"] = text.strip()
    print(f"Odom topic name set to: {trajectories[selected_trajectory_index].meta['topic']}")


def update_frame_id(text):
    """Update the frame_id of the selected trajectory."""
    global selected_trajectory_index
    global trajectories
    if selected_trajectory_index is not None:
        trajectories[selected_trajectory_index].meta["frame_id"] = text.strip()
        print(f"Updated frame_id to: {text.strip()}")

def update_child_frame_id(text):
    """Update the child_frame_id of the selected trajectory."""
    global selected_trajectory_index
    global trajectories
    if selected_trajectory_index is not None:
        trajectories[selected_trajectory_index].meta["child_frame_id"] = text.strip()
        print(f"Updated child_frame_id to: {text.strip()}")

def update_selected_trajectory(label):
    """Update the selected trajectory based on the dropdown selection."""
    global selected_trajectory_index
    global trajectories
    for idx, traj in enumerate(trajectories):
        if traj.meta["Name"] == label:
            selected_trajectory_index = idx
            print(f"Selected trajectory: {label}")
            update_settings()
            return

def publish_trajectory_loop(odom_pub, imu_pub, trajectory, start_time, rate=10):
    global is_publishing, numbers_publishing, numbers_succ_publishing

    tf_broadcaster = setup_transform_broadcaster()
    interval = 1.0 / rate  # Time interval between poses

    remaining_time = 0

    print("Start publishing " + trajectory.meta["topic"] + f"time {rospy.Time.now().to_time()}")
    try:
        while is_publishing:
            # Calculate elapsed time
            elapsed_time = (rospy.Time.now() - start_time).to_sec()

            # Determine the current pose index

            if(elapsed_time < 0):
                pose_index = 0
            else:
                pose_index = int(elapsed_time / interval)

            # Stop if we've reached the end of the trajectory
            if pose_index >= len(trajectory.positions_xyz):
                print("Finished publishing trajectory: " + trajectory.meta["Name"])
                break

            #simulate outage
            if "dead_reck_span" in  trajectory.meta:
                if trajectory.timestamps[pose_index] > trajectory.meta["dead_reck_span"][0] and trajectory.timestamps[pose_index] < trajectory.meta["dead_reck_span"][1]:
                    if remaining_time > 0:
                        rospy.sleep(remaining_time)
                        continue

            # Fetch the current pose and orientation
            pose = trajectory.positions_xyz[pose_index]
            orientation = trajectory.orientations_quat_wxyz[pose_index]

            # Create and populate the Odometry message
            orientation_xyzw = Quaternion(orientation[1], orientation[2], orientation[3], orientation[0])
            time_stamp = rospy.Time.now()

            odom_msg = Odometry()
            odom_msg.header.stamp = time_stamp
            odom_msg.header.frame_id = trajectory.meta["frame_id"]
            odom_msg.child_frame_id = trajectory.meta["child_frame_id"]
            odom_msg.pose.pose.position.x = pose[0]
            odom_msg.pose.pose.position.y = pose[1]
            odom_msg.pose.pose.position.z = pose[2]
            odom_msg.pose.pose.orientation = orientation_xyzw
            if(elapsed_time >= 0):
                odom_pub.publish(odom_msg)

            # Publish transform if enabled
            if trajectory.meta["publish_transform"]:
                transform = TransformStamped()
                transform.header.stamp = time_stamp
                transform.header.frame_id = trajectory.meta["frame_id"]
                transform.child_frame_id = trajectory.meta["child_frame_id"]
                transform.transform.translation.x = pose[0]
                transform.transform.translation.y = pose[1]
                transform.transform.translation.z = pose[2]
                transform.transform.rotation = orientation_xyzw
                if(elapsed_time >= 0):
                    tf_broadcaster.sendTransform(transform)

            # Publish orientation if enabled
            if trajectory.meta["publish_orientation"]:
                orientation_msg = QuaternionStamped()
                orientation_msg.header.stamp = time_stamp
                orientation_msg.header.frame_id = trajectory.meta["child_frame_id"]
                orientation_msg.quaternion = orientation_xyzw
                if(elapsed_time >= 0):
                    imu_pub.publish(orientation_msg)


            # Wait until the next expected time
            if elapsed_time < 0:
                next_time = start_time
                print("frist run finished publishing " + trajectory.meta["topic"] + f"time {rospy.Time.now().to_time()}")
            else:
                next_time = start_time + rospy.Duration((pose_index + 1) * interval)

            remaining_time = (next_time - rospy.Time.now()).to_sec()

            if remaining_time > 0:
                rospy.sleep(remaining_time)

    except rospy.ROSInterruptException:
        pass
    finally:
        print("Publishing finished " + trajectory.meta["Name"])

def start_publishing(event):
    global trajectories, is_publishing
    if not trajectories:
        print("Load a trajectory first.")
        return
    
    is_publishing = True

    start_time = rospy.Time.now() + rospy.Duration(7)  # Record the start time
    print("setup publishers and wait some time")

    for traj in trajectories:
        threading.Thread(target=publish_trajectory_loop, args=(start_odom_publisher(traj.meta["topic"]), start_quaternion_publisher(traj.meta["topic"]+"_orientation"), traj, start_time, traj.meta["frequency"])).start()

def stop_publishing(event):
    global is_publishing
    is_publishing = False

def toggle_view_mode(event):
    global trajectories
    global is_3d_mode
    global ax_trajectory

    if not trajectories:
        return

    is_3d_mode = not is_3d_mode

    ax_trajectory.clear()
    fig.delaxes(ax_trajectory)

    if is_3d_mode:
        ax_trajectory = fig.add_subplot(grid[0:2, 0], projection='3d')
    else:
        ax_trajectory = fig.add_subplot(grid[0:2, 0])

    for traj in trajectories:
        plot_trajectory(fig, ax_trajectory, traj, is_3d_mode, QUIVER_SPACING, autoscale=True)

    fig.canvas.draw()

def toggle_transform_publishing(event):
    global trajectories
    global selected_trajectory_index

    traj = trajectories[selected_trajectory_index]

    traj.meta["publish_transform"] = not traj.meta["publish_transform"]

    state = "enabled" if traj.meta["publish_transform"] else "disabled"
    print(f"Transform publishing for {traj.meta['Name']} {state}.")

def toggle_orientation_publishing(event):
    global trajectories
    global selected_trajectory_index

    traj = trajectories[selected_trajectory_index]

    traj.meta["publish_orientation"] = not traj.meta["publish_orientation"]

    state = "enabled" if traj.meta["publish_orientation"] else "disabled"
    print(f"IMU publishing for {traj.meta['Name']} {state}.")

def update_plots():
    global ax_trajectory
    global trajectories

    if not trajectories:
        return

    traj = trajectories[-1]

    plot_trajectory(fig, ax_trajectory, traj, is_3d_mode ,QUIVER_SPACING,autoscale=True)

    plot_velocity_profile(ax=ax_velocity, trajectory=traj)

    plot_height_profile(ax=ax_height, trajectory=traj)

    fig.canvas.draw()

def load_and_plot_file(file = None, GUI_used = True):
    if file is None:
        trajectory, name = load_trajectory()
    else:
        trajectory, name = load_trajectory(file)

    if trajectory is not None:
        if not "Name" in  trajectory.meta:
            trajectory.meta["Name"] = name

        if not "topic" in trajectory.meta:
            trajectory.meta["topic"] = "/odom_" + name
        
        if not "frame_id" in  trajectory.meta:
            trajectory.meta["frame_id"] = "odom_" + name

        if not "child_frame_id" in  trajectory.meta:
            trajectory.meta["child_frame_id"] = "base_link_" + name
        
        if not "publish_transform" in  trajectory.meta:
            trajectory.meta["publish_transform"] = False

        if not "publish_orientation" in trajectory.meta:
            trajectory.meta["publish_orientation"] = False

        if not "frequency" in trajectory.meta:
            trajectory.meta["frequency"] = 10

        trajectories.append(trajectory)

        if GUI_used:
            update_plots()
            update_selected_trajectory(trajectory.meta["Name"])
            update_trajectory_dropdown()

        print("loaded Trajectory "+ trajectories[-1].meta["Name"])

def load_and_plot(event):
    global trajectories
    
    root = Tk()
    root.withdraw()  # Hide the main tkinter window

    load_and_plot_file(None)


def clear_trajectories(event):

    global trajectories
    trajectories.clear()

    ax_trajectory.clear()
    ax_velocity.clear()
    ax_height.clear()
    fig.canvas.draw()


def update_trajectory_dropdown():
    """Update the radio button options with loaded trajectories."""
    global trajectories
    global selected_trajectory_index

    if selected_trajectory_index is None and trajectories:
        selected_trajectory_index = 0

    labels = [traj.meta["Name"] for traj in trajectories]
    radio_buttons.ax.clear()
    radio_buttons.__init__(ax_select, labels, active=selected_trajectory_index, activecolor='green')  # Reinitialize with new options
    radio_buttons.on_clicked(update_selected_trajectory)
    fig.canvas.draw()

def update_settings():
    global selected_trajectory_index
    global trajectories
    global text_box_odom
    global text_box_frame_id
    global text_box_child_frame_id
    global checkBoxButton_publish_trans

    if selected_trajectory_index is not None and trajectories:
        traj = trajectories[selected_trajectory_index]
        # Add TextBox for odom topic name

        text_box_odom.set_val(traj.meta["topic"])
        text_box_frame_id.set_val(traj.meta["frame_id"])
        text_box_child_frame_id.set_val(traj.meta["child_frame_id"])

        print(selected_trajectory_index)
        print(trajectories[selected_trajectory_index].meta["Name"])
        print(traj.meta["Name"])
        print(checkBoxButton_publish_trans.get_status()[0])
        print(traj.meta["publish_transform"])
        print(traj.meta["publish_orientation"])

        if(checkBoxButton_publish_trans.get_status()[0] != traj.meta["publish_transform"]):
            checkBoxButton_publish_trans.eventson = False
            checkBoxButton_publish_trans.set_active(index=0)
            checkBoxButton_publish_trans.eventson = True

        if(checkBoxButton_publish_orientation.get_status()[0] != traj.meta["publish_orientation"]):
            checkBoxButton_publish_orientation.eventson = False
            checkBoxButton_publish_orientation.set_active(index=0)
            checkBoxButton_publish_orientation.eventson = True


        print(traj.meta["Name"] + ": Settings loaded")
        #fig.canvas.draw()


def init_cli():
    parser = argparse.ArgumentParser(description="Command-line interface for the Trajectory Player")

    # CLI arguments
    parser.add_argument('--load', type=str, nargs='+', help="Paths to trajectory files to load and plot")
    parser.add_argument('--play', action='store_true', help="Start publishing loaded trajectories")

    args = parser.parse_args()

    print(args.load)

    #Load and plot trajectories if provided
    if args.load:
        for file in args.load:
            load_and_plot_file(file, GUI_used=False)

    #time.sleep(2)

    # Start publishing if requested
    if args.play:
        if not trajectories:
            print("No trajectories loaded. Use --load to specify trajectory files.")
        else:
            print("Starting trajectory publishing...")
            threading.Thread(target=start_publishing, args=(None,)).start()





# Plotting setup
fig = plt.figure(figsize=(14, 8))
grid = fig.add_gridspec(3, 5, width_ratios=[2, 0.1, 1, 0.1, 1.0], height_ratios=[2, 1, 1])

ax_trajectory = fig.add_subplot(grid[0:2, 0])
ax_velocity = fig.add_subplot(grid[0, 2], )
ax_height = fig.add_subplot(grid[0, 4])
ax_select = fig.add_subplot(grid[2:, 2], frameon=False)

# Button setup
btn_load = Button(plt.axes([0.1, 0.15, 0.15, 0.05]), 'Load Trajectory')
btn_toggle_view = Button(plt.axes([0.3, 0.15, 0.15, 0.05]), 'Toggle 2D/3D')
btn_publish = Button(plt.axes([0.1, 0.02, 0.15, 0.05]), 'Start Publishing')
btn_stop = Button(plt.axes([0.1, 0.08, 0.15, 0.05]), 'Stop Publishing')
btn_clear = Button(plt.axes([0.3, 0.02, 0.15, 0.05]), 'Clear')

text_box_odom = TextBox(plt.axes([0.76, 0.24, 0.2, 0.05]), "Odom Topic:", "test")
text_box_odom.on_submit(update_odom_topic_name)


btn_load.on_clicked(load_and_plot)
btn_toggle_view.on_clicked(toggle_view_mode)
btn_publish.on_clicked(start_publishing)
btn_stop.on_clicked(stop_publishing)
btn_clear.on_clicked(clear_trajectories)

radio_buttons = RadioButtons(ax_select, ["none"])
#ax_select.set_title("Settings Selector")

text_box_odom = TextBox(plt.axes([0.76, 0.24, 0.2, 0.05]), "Odom Topic:", "None")
text_box_odom.on_submit(update_odom_topic_name)
text_box_frame_id = TextBox(plt.axes([0.76, 0.15, 0.2, 0.05]), "frame_id:", "None")
text_box_frame_id.on_submit(update_frame_id)
text_box_child_frame_id = TextBox(plt.axes([0.76, 0.08, 0.2, 0.05]), "child_frame_id:", "None")
text_box_child_frame_id.on_submit(update_child_frame_id)

# Add CheckButton for publishing transform
checkBoxButton_publish_trans = CheckButtons(plt.axes([0.76, 0.3, 0.1, 0.05]), ["Pub Trans"])
checkBoxButton_publish_trans.on_clicked(toggle_transform_publishing)

checkBoxButton_publish_orientation = CheckButtons(plt.axes([0.86, 0.3, 0.1, 0.05]), ["Pub IMU"])
checkBoxButton_publish_orientation.on_clicked(toggle_orientation_publishing)

init_cli()

plt.show()

