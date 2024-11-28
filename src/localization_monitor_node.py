#!/usr/bin/env python3
import rospy
import threading
import numpy as np
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from evo.core.trajectory import PoseTrajectory3D
from localization_monitor.msg import LocalizationMonitorResult
from bisect import bisect_left
from ais_robot_localization.utils import odom_to_se3, se3_to_posestamped, calculate_euclidean_distance, snake_alignment, calculate_rpe_data

class OdometryComparisonNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('odometry_comparison_node')

        # Parameters
        self.global_odom_topic = rospy.get_param('~global_odom_topic', '/odometry/gps_with_orientation')
        self.local_odom_topic = rospy.get_param('~local_odom_topic', '/odom_dlo')
        self.dist_cum_threshold = rospy.get_param('~dist_cum_threshold', 15.0)
        self.max_poses_threshold = rospy.get_param('~max_poses_threshold', 500) 
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)
        self.time_based = rospy.get_param('~time_based', False)
        self.publish_gnss_with_scaled_covariance = rospy.get_param('~publish_gnss_with_scaled_covariance', False)

        # Lock for synchronization
        self.lock = threading.Lock()

        # Error Publisher
        self.feature_pub = rospy.Publisher('RPE_values', Float32MultiArray, queue_size=10)

        # Publishers for RViz visualization
        self.global_odom_path_pub = rospy.Publisher('/global_odom_path', Path, queue_size=10)
        self.local_odom_path_pub = rospy.Publisher('/local_odom_path', Path, queue_size=10)
        self.localization_result_pub = rospy.Publisher('/localization_result', LocalizationMonitorResult, queue_size=10)

        # Data storage
        self.global_odom_poses = []
        self.local_odom_poses = []
        self.global_odom_timestamps = []
        self.local_odom_timestamps = []
        self.cumulative_distance = 0.0
        self.cumulative_length = 0.0

        # Initialize Path messages
        self.global_odom_path = Path()
        self.local_odom_path = Path()
        #self.global_odom_path_selected = Path()
        #self.local_odom_selected = Path()

        # Set up throttling for path publishing
        self.last_publish_time = rospy.Time.from_sec(0.0)
        self.global_frame = ""
        self.local_frame = ""
        self.child_frame = ""
        self.time_newest_global_pose = rospy.Time.from_sec(0.0)

        # Subscribers
        self.ref_sub = rospy.Subscriber(self.global_odom_topic, Odometry, self.global_odom_callback)
        self.local_odom_sub = rospy.Subscriber(self.local_odom_topic, Odometry, self.local_odom_callback)

        rospy.loginfo("OdometryComparisonNode initialized")

    def global_odom_callback(self, msg):
        rospy.logdebug("Received message on local_odom topic")
        with self.lock:
            try:
                pose_se3 = odom_to_se3(msg.pose.pose)
                self.global_odom_poses.append(pose_se3)
                self.global_odom_timestamps.append(msg.header.stamp)
                self.time_newest_global_pose = msg.header.stamp
                self.global_frame = msg.header.frame_id
                self.child_frame =  msg.child_frame_id
            except Exception as e:
                rospy.logerr(f"Error in global_odom_callback: {e}")
        rospy.logdebug("Received message on global_odom topic")

    def local_odom_callback(self, msg):
        with self.lock:
            try:
                pose_se3 = odom_to_se3(msg.pose.pose)
                timestamp = msg.header.stamp

                if self.local_odom_poses:
                    last_pose_se3 = self.local_odom_poses[-1]
                    dist = calculate_euclidean_distance(last_pose_se3, pose_se3)
                    if(dist < 0.2): #ToDo How to handle this?? evtl update newest?
                        return 
                    self.cumulative_distance += dist
                    self.cumulative_length += dist

                self.local_frame = msg.header.frame_id

                # Convert SE(3) to PoseStamped and update the global_odom path
                pose_stamped = se3_to_posestamped(pose_se3, msg.header.stamp, z_to_zero=False)
                pose_stamped.header = msg.header
                self.local_odom_path.poses.append(pose_stamped)

                # Add the new pose and timestamp
                self.local_odom_poses.append(pose_se3)
                self.local_odom_timestamps.append(timestamp)

                # Throttle the publication rate
                current_time = timestamp
                if (current_time - self.last_publish_time).to_sec() >= (1.0 / self.publish_rate) and (self.cumulative_distance > 1.0 or self.time_based):
                    rospy.loginfo("Recalculate.....")

                    self.check_cumulative_distance()
                    self.cleanup_global_odom_poses()
                    self.find_corresponding_poses()
                    self.align_local_to_global_odom_and_calc_errors(msg.header)

                    self.publish_global_odom_path(msg.header)
                    self.publish_local_odom_path(msg.header)
                    
                    rospy.loginfo(f"Paths published with ref {len(self.global_odom_path.poses)}, comp {len(self.local_odom_path.poses)}")
                    self.last_publish_time = current_time
            except Exception as e:
                rospy.logerr(f"Error in local_odom_callback: {e}")

    def publish_results(self, rpe_avg_trans=float('inf'), rpe_avg_rot=float('inf'), rpe_max_trans=float('inf'), rpe_max_rot=float('inf')):
        feature_msg = Float32MultiArray()
        feature_msg.data = [rpe_avg_trans, rpe_avg_rot, rpe_max_trans, rpe_max_rot]
        rospy.loginfo(f"Publishing RPE values: RPE_avg_trans={rpe_avg_trans}, RPE_avg_rot={rpe_avg_rot}, RPE_max_trans={rpe_max_trans}, RPE_avg_rot={rpe_max_rot} at length {self.cumulative_distance}")
        self.feature_pub.publish(feature_msg)

        #include center pose of trajectories
        used_index =  len(self.global_odom_poses)-1 #int(len(self.global_odom_poses) / 2)
        pose_stamped_global = se3_to_posestamped(self.global_odom_poses[used_index], self.local_odom_timestamps[used_index], frame_id=self.global_frame, z_to_zero=False)
        pose_stamped_global.header.stamp = self.local_odom_timestamps[used_index]

        pose_stamped_local = se3_to_posestamped(self.local_odom_poses[used_index], self.local_odom_timestamps[used_index], frame_id=self.local_frame, z_to_zero=False)
        pose_stamped_local.header.stamp = self.local_odom_timestamps[used_index]

        result = LocalizationMonitorResult()
        result.float_array = feature_msg.data
        result.pose_global = pose_stamped_global
        result.pose_local = pose_stamped_local
        result.cumulative_length = self.cumulative_length
        self.localization_result_pub.publish(result)

        
        #if(rpe_avg_trans < 1.0):
        #    used_index = int(len(self.global_odom_poses) / 2)
        #    pose_stamped = se3_to_posestamped(self.global_odom_poses[used_index], self.local_odom_timestamps[used_index], z_to_zero=False)
        #    pose_stamped.header.stamp = self.local_odom_timestamps[used_index]
        #    pose_stamped.header.frame_id = self.global_frame
        #    self.global_odom_path_selected.poses.append(pose_stamped)
            
        #    # publish path
        #    self.global_odom_path_selected.header.frame_id = self.global_frame
        #    self.global_odom_path_selected.header.stamp = self.time_newest_global_pose
        #    # ToDo: Add child frame?
        #    self.global_odom_path_selected_pub.publish(self.global_odom_path_selected)




    def calculate_and_publish_results(self, global_odom_traj, local_odom_trajectory):
        if len(global_odom_traj.poses_se3) < 2 or self.cumulative_distance < 2.0:
            self.publish_results()
            return

        max_trans_error = float('inf')
        max_rot_error = float('inf')
        
        ref_pose_start = global_odom_traj.poses_se3[0]
        ref_pose_target = global_odom_traj.poses_se3[1]
        comp_pose_start = local_odom_trajectory.poses_se3[0]
        comp_pose_target = local_odom_trajectory.poses_se3[1]

        rpe_curr, error_curr_trans, error_curr_rot, ds = calculate_rpe_data(
            ref_pose_start, ref_pose_target, ref_pose_start, comp_pose_start, comp_pose_target
        )

        total_error_trans = error_curr_trans * ds
        total_error_rot = error_curr_rot * ds

        if error_curr_trans < max_trans_error:
            max_trans_error = error_curr_trans
        if error_curr_rot < max_rot_error:
            max_rot_error = error_curr_rot

        error_prev_trans = error_curr_trans
        error_prev_rot = error_curr_rot
        path_length = ds

        for i in range(2, len(global_odom_traj.poses_se3)):
            ref_pose_target = global_odom_traj.poses_se3[i]
            comp_pose_target = local_odom_trajectory.poses_se3[i]
            ref_pose_one_before_target = global_odom_traj.poses_se3[i-1] 

            rpe_curr, error_curr_trans, error_curr_rot, ds = calculate_rpe_data(
                ref_pose_start, ref_pose_target, ref_pose_one_before_target, comp_pose_start, comp_pose_target
            )

            error_avg_trans = (error_prev_trans + error_curr_trans) / 2
            total_error_trans += error_avg_trans * ds
            error_avg_rot = (error_prev_rot + error_curr_rot) / 2
            total_error_rot += error_avg_rot * ds

            error_prev_trans = error_curr_trans
            error_prev_rot = error_curr_rot
            path_length += ds

        self.publish_results(total_error_trans / path_length, total_error_rot / path_length, max_trans_error, max_rot_error)

    def align_local_to_global_odom_and_calc_errors(self, header):
        rospy.loginfo("Align local´odom path to global odom path")
        self.local_odom_path = Path()
        local_odom_trajectory = PoseTrajectory3D(poses_se3=np.copy(self.local_odom_poses), timestamps=self.local_odom_timestamps)
        global_odom_trajectory = PoseTrajectory3D(poses_se3=np.copy(self.global_odom_poses), timestamps=self.global_odom_timestamps)

        #R, t = kabsch_algorithm(local_odom_trajectory, global_odom_trajectory)
        #local_odom_trajectory.align(traj_ref=global_odom_trajectory)
        trans = snake_alignment(local_odom_trajectory, global_odom_trajectory)
        self.calculate_and_publish_results(global_odom_trajectory, local_odom_trajectory)

        for i, pose_se3 in enumerate(local_odom_trajectory.poses_se3):
            pose_stamped = se3_to_posestamped(pose_se3, self.local_odom_timestamps[i], z_to_zero=False)
            pose_stamped.header = header
            pose_stamped.header.frame_id = self.global_frame
            self.local_odom_path.poses.append(pose_stamped)

    def publish_local_odom_path(self, header):
        self.local_odom_path.header.frame_id = self.global_frame
        self.local_odom_path.header.stamp = self.time_newest_global_pose
        # ToDo: Add child frame?
        self.local_odom_path_pub.publish(self.local_odom_path)
    
    def publish_global_odom_path(self, header):
        self.global_odom_path.header.frame_id = self.global_frame
        self.global_odom_path.header.stamp = self.time_newest_global_pose
        # ToDo: Add child frame?
        self.global_odom_path_pub.publish(self.global_odom_path)

    def find_corresponding_poses(self):
        rospy.loginfo("Synchronizing local_odom poses with global_odom poses")
        if not self.local_odom_poses or not self.global_odom_poses:
            return
        aligned_global_odom_poses = []
        aligned_global_odom_timestamps = []
        self.global_odom_path.poses.clear()
        for ref_time in self.local_odom_timestamps:
            nearest_idx = self.find_nearest_time(ref_time)
            pose_se3 = self.global_odom_poses[nearest_idx]
            timestamp = self.global_odom_timestamps[nearest_idx]
            aligned_global_odom_poses.append(pose_se3)
            aligned_global_odom_timestamps.append(timestamp)

            #create path to publish
            pose_stamped = se3_to_posestamped(pose_se3, timestamp, z_to_zero=False)
            pose_stamped.header.frame_id = self.global_frame
            self.global_odom_path.poses.append(pose_stamped)

        self.global_odom_poses = aligned_global_odom_poses
        self.global_odom_timestamps = aligned_global_odom_timestamps
        rospy.loginfo("Synchronized %d poses from local odom with %d aligned poses from global odom", len(self.local_odom_poses), len(aligned_global_odom_poses))

    def check_cumulative_distance(self):
        rospy.loginfo("Checking cumulative distance")
        while (self.cumulative_distance > self.dist_cum_threshold and len(self.local_odom_poses) > 1) or len(self.local_odom_poses) > self.max_poses_threshold:
            first_pose_se3 = self.local_odom_poses[0] 
            second_pose_se3 = self.local_odom_poses[1]
            dist_removed = calculate_euclidean_distance(first_pose_se3, second_pose_se3)
            self.local_odom_poses.pop(0)
            self.local_odom_timestamps.pop(0)
            self.cumulative_distance -= dist_removed
        rospy.loginfo("Finished checking cumulative distance")

    def cleanup_global_odom_poses(self):
        rospy.loginfo("Cleaning up old comparison poses")
        if not self.local_odom_timestamps:
            return

        oldest_local_odom_timestamp = self.local_odom_timestamps[0]
        while self.global_odom_timestamps and self.global_odom_timestamps[0] < oldest_local_odom_timestamp:
            self.global_odom_poses.pop(0)
            self.global_odom_timestamps.pop(0)
        rospy.logdebug("Finished cleaning up old comparison poses")

    def find_nearest_time(self, ref_time):
        pos = bisect_left(self.global_odom_timestamps, ref_time)
        if pos == 0:
            return 0
        if pos == len(self.global_odom_timestamps):
            return len(self.global_odom_timestamps) - 1
        before = self.global_odom_timestamps[pos - 1]
        after = self.global_odom_timestamps[pos]
        return pos if abs(after - ref_time) < abs(before - ref_time) else pos - 1

if __name__ == '__main__':
    try:
        node = OdometryComparisonNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("OdometryComparisonNode has been interrupted.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
