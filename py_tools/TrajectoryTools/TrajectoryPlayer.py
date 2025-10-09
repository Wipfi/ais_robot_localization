import argparse
import threading
import time
from tkinter import Tk

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib.widgets import Button, CheckButtons, RadioButtons, TextBox
import rclpy
from geometry_msgs.msg import Quaternion, QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from utils import (
    load_trajectory,
    plot_height_profile,
    plot_trajectory,
    plot_velocity_profile,
)


class TrajectoryPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_publisher")
        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_publishers = {}
        self._imu_publishers = {}

    def get_transform_broadcaster(self) -> TransformBroadcaster:
        return self._tf_broadcaster

    def get_odom_publisher(self, topic: str):
        if topic not in self._odom_publishers:
            self._odom_publishers[topic] = self.create_publisher(Odometry, topic, 10)
        return self._odom_publishers[topic]

    def get_orientation_publisher(self, topic: str):
        if topic not in self._imu_publishers:
            self._imu_publishers[topic] = self.create_publisher(QuaternionStamped, topic, 10)
        return self._imu_publishers[topic]


trajectories = []
is_publishing = False

QUIVER_SPACING = 5.0
is_3d_mode = False
selected_trajectory_index = None


rclpy.init(args=None)
ros_node = TrajectoryPublisherNode()


def now_as_float() -> float:
    current_time = ros_node.get_clock().now()
    seconds, nanoseconds = divmod(current_time.nanoseconds, int(1e9))
    return float(seconds) + float(nanoseconds) / 1e9


def update_odom_topic_name(text):
    global selected_trajectory_index
    global trajectories
    trajectories[selected_trajectory_index].meta["topic"] = text.strip()
    print(f"Odom topic name set to: {trajectories[selected_trajectory_index].meta['topic']}")


def update_frame_id(text):
    global selected_trajectory_index
    global trajectories
    if selected_trajectory_index is not None:
        trajectories[selected_trajectory_index].meta["frame_id"] = text.strip()
        print(f"Updated frame_id to: {text.strip()}")


def update_child_frame_id(text):
    global selected_trajectory_index
    global trajectories
    if selected_trajectory_index is not None:
        trajectories[selected_trajectory_index].meta["child_frame_id"] = text.strip()
        print(f"Updated child_frame_id to: {text.strip()}")


def update_selected_trajectory(label):
    global selected_trajectory_index
    global trajectories
    for idx, traj in enumerate(trajectories):
        if traj.meta["Name"] == label:
            selected_trajectory_index = idx
            print(f"Selected trajectory: {label}")
            update_settings()
            return


def publish_trajectory_loop(trajectory, start_wall_time, rate=10):
    global is_publishing

    if rate <= 0:
        rate = 10

    interval = 1.0 / rate

    odom_pub = ros_node.get_odom_publisher(trajectory.meta["topic"])
    orientation_topic = trajectory.meta.get("orientation_topic", trajectory.meta["topic"] + "_orientation")
    imu_pub = ros_node.get_orientation_publisher(orientation_topic)
    tf_broadcaster = ros_node.get_transform_broadcaster()

    print("Start publishing " + trajectory.meta["topic"] + f" time {now_as_float():.3f}")

    while is_publishing:
        now_wall = time.monotonic()
        elapsed_time = now_wall - start_wall_time

        if elapsed_time < 0:
            pose_index = 0
        else:
            pose_index = int(elapsed_time / interval)

        if pose_index >= len(trajectory.positions_xyz):
            print("Finished publishing trajectory: " + trajectory.meta["Name"])
            break

        if "dead_reck_span" in trajectory.meta:
            span_start, span_end = trajectory.meta["dead_reck_span"]
            if span_start < trajectory.timestamps[pose_index] < span_end:
                sleep_time = start_wall_time + (pose_index + 1) * interval - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue

        pose = trajectory.positions_xyz[pose_index]
        orientation = trajectory.orientations_quat_wxyz[pose_index]

        orientation_xyzw = Quaternion()
        orientation_xyzw.x = orientation[1]
        orientation_xyzw.y = orientation[2]
        orientation_xyzw.z = orientation[3]
        orientation_xyzw.w = orientation[0]

        time_stamp = ros_node.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = time_stamp
        odom_msg.header.frame_id = trajectory.meta["frame_id"]
        odom_msg.child_frame_id = trajectory.meta["child_frame_id"]
        odom_msg.pose.pose.position.x = pose[0]
        odom_msg.pose.pose.position.y = pose[1]
        odom_msg.pose.pose.position.z = pose[2]
        odom_msg.pose.pose.orientation = orientation_xyzw

        if elapsed_time >= 0:
            odom_pub.publish(odom_msg)

        if trajectory.meta["publish_transform"] and elapsed_time >= 0:
            transform = TransformStamped()
            transform.header.stamp = time_stamp
            transform.header.frame_id = trajectory.meta["frame_id"]
            transform.child_frame_id = trajectory.meta["child_frame_id"]
            transform.transform.translation.x = pose[0]
            transform.transform.translation.y = pose[1]
            transform.transform.translation.z = pose[2]
            transform.transform.rotation = orientation_xyzw
            tf_broadcaster.sendTransform(transform)

        if trajectory.meta["publish_orientation"] and elapsed_time >= 0:
            orientation_msg = QuaternionStamped()
            orientation_msg.header.stamp = time_stamp
            orientation_msg.header.frame_id = trajectory.meta["child_frame_id"]
            orientation_msg.quaternion = orientation_xyzw
            imu_pub.publish(orientation_msg)

        next_time = start_wall_time if elapsed_time < 0 else start_wall_time + (pose_index + 1) * interval
        remaining_time = next_time - time.monotonic()
        if remaining_time > 0:
            time.sleep(remaining_time)

    print("Publishing finished " + trajectory.meta["Name"])


def start_publishing(event):
    global trajectories, is_publishing
    if not trajectories:
        print("Load a trajectory first.")
        return

    is_publishing = True

    delay = trajectories[0].meta.get("start_delay", 7.0)
    start_time = time.monotonic() + delay
    print("Setup publishers and wait some time")

    for traj in trajectories:
        frequency = traj.meta.get("frequency", 10)
        threading.Thread(
            target=publish_trajectory_loop,
            args=(traj, start_time, frequency),
            daemon=True,
        ).start()


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
        ax_trajectory = fig.add_subplot(grid[0:2, 0], projection="3d")
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

    plot_trajectory(fig, ax_trajectory, traj, is_3d_mode, QUIVER_SPACING, autoscale=True)

    plot_velocity_profile(ax=ax_velocity, trajectory=traj)

    plot_height_profile(ax=ax_height, trajectory=traj)

    fig.canvas.draw()


def load_and_plot_file(file=None, GUI_used=True):
    if file is None:
        trajectory, name = load_trajectory()
    else:
        trajectory, name = load_trajectory(file)

    if trajectory is not None:
        if "Name" not in trajectory.meta:
            trajectory.meta["Name"] = name

        if "topic" not in trajectory.meta:
            trajectory.meta["topic"] = "/odom_" + name

        if "frame_id" not in trajectory.meta:
            trajectory.meta["frame_id"] = "odom_" + name

        if "child_frame_id" not in trajectory.meta:
            trajectory.meta["child_frame_id"] = "base_link_" + name

        if "publish_transform" not in trajectory.meta:
            trajectory.meta["publish_transform"] = False

        if "publish_orientation" not in trajectory.meta:
            trajectory.meta["publish_orientation"] = False

        if "frequency" not in trajectory.meta:
            trajectory.meta["frequency"] = 10

        trajectories.append(trajectory)

        if GUI_used:
            update_plots()
            update_selected_trajectory(trajectory.meta["Name"])
            update_trajectory_dropdown()

        print("loaded Trajectory " + trajectories[-1].meta["Name"])


def load_and_plot(event):
    global trajectories

    root = Tk()
    root.withdraw()

    load_and_plot_file(None)


def clear_trajectories(event):
    global trajectories
    trajectories.clear()

    ax_trajectory.clear()
    ax_velocity.clear()
    ax_height.clear()
    fig.canvas.draw()


def update_trajectory_dropdown():
    global trajectories
    global selected_trajectory_index

    if selected_trajectory_index is None and trajectories:
        selected_trajectory_index = 0

    labels = [traj.meta["Name"] for traj in trajectories]
    radio_buttons.ax.clear()
    radio_buttons.__init__(ax_select, labels, active=selected_trajectory_index, activecolor="green")
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

        text_box_odom.set_val(traj.meta["topic"])
        text_box_frame_id.set_val(traj.meta["frame_id"])
        text_box_child_frame_id.set_val(traj.meta["child_frame_id"])

        if checkBoxButton_publish_trans.get_status()[0] != traj.meta["publish_transform"]:
            checkBoxButton_publish_trans.eventson = False
            checkBoxButton_publish_trans.set_active(index=0)
            checkBoxButton_publish_trans.eventson = True

        if checkBoxButton_publish_orientation.get_status()[0] != traj.meta["publish_orientation"]:
            checkBoxButton_publish_orientation.eventson = False
            checkBoxButton_publish_orientation.set_active(index=0)
            checkBoxButton_publish_orientation.eventson = True

        print(traj.meta["Name"] + ": Settings loaded")


def init_cli():
    parser = argparse.ArgumentParser(description="Command-line interface for the Trajectory Player")

    parser.add_argument("--load", type=str, nargs="+", help="Paths to trajectory files to load and plot")
    parser.add_argument("--play", action="store_true", help="Start publishing loaded trajectories")

    args = parser.parse_args()

    if args.load:
        for file in args.load:
            load_and_plot_file(file, GUI_used=False)

    if args.play:
        if not trajectories:
            print("No trajectories loaded. Use --load to specify trajectory files.")
        else:
            print("Starting trajectory publishing...")
            threading.Thread(target=start_publishing, args=(None,), daemon=True).start()


fig = plt.figure(figsize=(14, 8))
grid = fig.add_gridspec(3, 5, width_ratios=[2, 0.1, 1, 0.1, 1.0], height_ratios=[2, 1, 1])

ax_trajectory = fig.add_subplot(grid[0:2, 0])
ax_velocity = fig.add_subplot(grid[0, 2])
ax_height = fig.add_subplot(grid[0, 4])
ax_select = fig.add_subplot(grid[2:, 2], frameon=False)

btn_load = Button(plt.axes([0.1, 0.15, 0.15, 0.05]), "Load Trajectory")
btn_toggle_view = Button(plt.axes([0.3, 0.15, 0.15, 0.05]), "Toggle 2D/3D")
btn_publish = Button(plt.axes([0.1, 0.02, 0.15, 0.05]), "Start Publishing")
btn_stop = Button(plt.axes([0.1, 0.08, 0.15, 0.05]), "Stop Publishing")
btn_clear = Button(plt.axes([0.3, 0.02, 0.15, 0.05]), "Clear")

btn_load.on_clicked(load_and_plot)
btn_toggle_view.on_clicked(toggle_view_mode)
btn_publish.on_clicked(start_publishing)
btn_stop.on_clicked(stop_publishing)
btn_clear.on_clicked(clear_trajectories)

radio_buttons = RadioButtons(ax_select, ["none"])

text_box_odom = TextBox(plt.axes([0.76, 0.24, 0.2, 0.05]), "Odom Topic:", "None")
text_box_odom.on_submit(update_odom_topic_name)
text_box_frame_id = TextBox(plt.axes([0.76, 0.15, 0.2, 0.05]), "frame_id:", "None")
text_box_frame_id.on_submit(update_frame_id)
text_box_child_frame_id = TextBox(plt.axes([0.76, 0.08, 0.2, 0.05]), "child_frame_id:", "None")
text_box_child_frame_id.on_submit(update_child_frame_id)

checkBoxButton_publish_trans = CheckButtons(plt.axes([0.76, 0.3, 0.1, 0.05]), ["Pub Trans"])
checkBoxButton_publish_trans.on_clicked(toggle_transform_publishing)

checkBoxButton_publish_orientation = CheckButtons(plt.axes([0.86, 0.3, 0.1, 0.05]), ["Pub IMU"])
checkBoxButton_publish_orientation.on_clicked(toggle_orientation_publishing)

init_cli()

try:
    plt.show()
finally:
    ros_node.destroy_node()
    rclpy.shutdown()
