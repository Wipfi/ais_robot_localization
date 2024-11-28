import numpy as np
import math
from scipy.linalg import svd
from scipy.spatial.transform import Rotation as rot
from tf.transformations import quaternion_from_matrix, quaternion_matrix
from geometry_msgs.msg import PoseStamped
from evo.core.lie_algebra import so3_log_angle
from evo.core.trajectory import PoseTrajectory3D

def odom_to_se3(pose):
    """
    Convert ROS Pose (position and orientation) to SE(3) 4x4 transformation matrix.
    """
    se3_matrix = np.eye(4)

    # Fill in the translation part
    se3_matrix[0, 3] = pose.position.x
    se3_matrix[1, 3] = pose.position.y
    se3_matrix[2, 3] = pose.position.z

    # Convert quaternion to a 3x3 rotation matrix
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rotation_matrix = quaternion_matrix(quaternion)[:3, :3]

    # Place the rotation matrix into the SE(3) matrix
    se3_matrix[:3, :3] = rotation_matrix

    return se3_matrix

def se3_to_posestamped(se3_matrix, stamp, frame_id="", z_to_zero = False):
    """
    Convert SE(3) 4x4 matrix to a PoseStamped message.
    """
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = stamp
    pose_stamped.header.frame_id = frame_id

    # Fill in the position
    pose_stamped.pose.position.x = se3_matrix[0, 3]
    pose_stamped.pose.position.y = se3_matrix[1, 3]

    if(z_to_zero):
        pose_stamped.pose.position.z = 0.0
    else:
        pose_stamped.pose.position.z = se3_matrix[2, 3]
        

    # Extract rotation matrix and convert to quaternion
    quat = quaternion_from_matrix(se3_matrix)

    pose_stamped.pose.orientation.x = quat[0]
    pose_stamped.pose.orientation.y = quat[1]
    pose_stamped.pose.orientation.z = quat[2]
    pose_stamped.pose.orientation.w = quat[3]

    return pose_stamped


def calculate_euclidean_distance(se3_matrix1, se3_matrix2):
    """Calculate the Euclidean distance between the translations of two SE(3) matrices."""
    dx = se3_matrix2[0, 3] - se3_matrix1[0, 3]
    dy = se3_matrix2[1, 3] - se3_matrix1[1, 3]
    dz = se3_matrix2[2, 3] - se3_matrix1[2, 3]
    return math.sqrt(dx**2 + dy**2 + dz**2)


def se3(R: np.ndarray = np.eye(3),
        t: np.ndarray = np.array([0, 0, 0])) -> np.ndarray:  
    """
    :param r: SO(3) rotation matrix
    :param t: 3x1 translation vector
    :return: SE(3) transformation matrix
    """
    se3 = np.eye(4)
    se3[:3, :3] = R
    se3[:3, 3] = t
    return se3


def kabsch_algorithm(traj_a_poses_se3, traj_b_poses_se3, weights = None):
    """
    Perform the Kabsch algorithm to find the optimal rotation matrix and translation vector
    that aligns two PoseTrajectory3D objects.
    
    Args:
        traj_a (PoseTrajectory3D): The first trajectory (source).
        traj_b (PoseTrajectory3D): The second trajectory (target).
    
    Returns:
        R (np.ndarray): The optimal rotation matrix (3x3).
        t (np.ndarray): The optimal translation vector (3x1).
    """
    
    #assert len(traj_a.poses_se3) == len(traj_b.poses_se3), "Trajectories must have the same number of poses."
    
    # Extract the positions from the PoseTrajectory3D objects
    positions_a = traj_a_poses_se3.positions_xyz.T
    positions_b = traj_b_poses_se3.positions_xyz.T

    if weights is None:
        # Compute the centroids of each set of positions
        centroid_a = np.mean(positions_a, axis=1, keepdims=True)  # Shape (3, 1)
        centroid_b = np.mean(positions_b, axis=1, keepdims=True)  # Shape (3, 1)
    
    else:
    # Compute the weighted centroids
        weights = np.array(weights)  # Ensure weights is a NumPy array
        weights_normalized = weights / np.sum(weights)  # Normalize weights to sum to 1

        centroid_a = np.sum(positions_a * weights_normalized, axis=1, keepdims=True)  # Shape (3, 1)
        centroid_b = np.sum(positions_b * weights_normalized, axis=1, keepdims=True)  # Shape (3, 1)

    # Center the positions by subtracting the centroids
    positions_a_centered = positions_a - centroid_a  # Shape (3, N)
    positions_b_centered = positions_b - centroid_b  # Shape (3, N)

    R,_ = rot.align_vectors(positions_b_centered.T, positions_a_centered.T, weights)

    R = R.as_matrix()

    
    # Compute the translation vector t
    t = centroid_b - np.dot(R, centroid_a)

    return R, t.T


def transfrom_path(se3_poses, R: np.ndarray = np.eye(3),
        t: np.ndarray = np.array([0, 0, 0])):
    
    transformation = se3(R, t)

    se3_poses_transformed = []

    for pose in se3_poses:
        se3_poses_transformed.append(transformation @ pose)

    return se3_poses_transformed


def frobenius_norm(matrix):
    """Calculate the Frobenius norm of a matrix."""
    return np.linalg.norm(matrix, 'fro')

def translation_norm(matrix):
    """Calculate the norm of the translation part of a transformation matrix."""
    translation_vector = matrix[:3, 3]
    return np.linalg.norm(translation_vector)

def rotation_norm(matrix):
    """Calculate the rotation angle from the rotation part of a transformation matrix using the evo library."""
    rotation_matrix = matrix[:3, :3]
    return so3_log_angle(rotation_matrix)


def calculate_rpe_data(ref_pose_start, ref_pose_target, ref_pose_one_bevore_target, comp_pose_start, comp_pose_target):
    ref_transform_curr = np.linalg.inv(ref_pose_start) @ ref_pose_target
    comp_transform_curr = np.linalg.inv(comp_pose_start) @ comp_pose_target

    rpe_curr = np.linalg.inv(ref_transform_curr) @ comp_transform_curr

    error_curr_trans = translation_norm(rpe_curr)
    error_curr_rot = rotation_norm(rpe_curr)

    # Pathsegment Length
    ds = calculate_euclidean_distance(ref_pose_start, ref_pose_target)

    return rpe_curr, error_curr_trans, error_curr_rot, ds

def snake_alignment(trajectory: PoseTrajectory3D, reference_trajectory: PoseTrajectory3D, weights = None):
    make_snake(trajectory=trajectory)
    make_snake(trajectory=reference_trajectory)

    rotation, translation = kabsch_algorithm(trajectory, reference_trajectory, weights)
    print("---------------------------")
    print('Translation')
    print(rotation)
    print('Rotation')
    print(translation)


    #rotation, translation, s = trajectory.align(traj_ref=reference_trajectory)
    #Initialize a 4x4 identity matrix
    transform = np.eye(4)
    # Insert rotation and translation into the transformation matrix
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    trajectory.transform(transform)
    return transform




def make_snake(trajectory: PoseTrajectory3D, fatness = 5.0):
    if(len(trajectory.poses_se3) < 3):
        raise Exception("To short to be a snake :(")
    
    translation_vector = np.array([0, fatness, 0, 1]) 

    for i in range(0, len(trajectory.poses_se3) -1):
        # Convert quaternion to rotation matrix
        pose = trajectory.poses_se3[i]
        rotation = np.array(pose)
        rotation[:3, 3] = np.array([0,0,0])  #remove translation
        point_translation = rotation @ translation_vector
        if(i%2) == 0:
            point_translation *= -1.0

        #print(trajectory.poses_se3[i][:3, 3])
        #print(point_translation[0:3])
        trajectory.poses_se3[i] = pose
        trajectory.poses_se3[i][:3, 3] += point_translation[0:3]


def gaussian_weight(errors, half_life):
    """
    Calculate Gaussian-based weights based on a list of error values and a half-life distance.

    Parameters:
    - errors: A list of estimated translation errors for each point.
    - half_life: The error distance at which the function should return a weight of 0.5.

    Returns:
    - weights: A list of calculated weights based on the Gaussian function.
    """

    # Calculate the standard deviation (sigma) for the Gaussian function
    sigma = half_life / math.sqrt(2 * math.log(2))
    
    # Calculate the Gaussian weight for each error
    weights = [math.exp(-0.5 * (error / sigma) ** 2) for error in errors]
    return weights


def euclidean_distance(pose1: PoseStamped, pose2: PoseStamped) -> float:
    """
    Calculate the Euclidean distance between two SE(3) poses given as PoseStamped messages.

    Parameters:
    - pose1: The first PoseStamped message.
    - pose2: The second PoseStamped message.

    Returns:
    - The Euclidean distance between the two poses.
    """
    # Extract positions from the PoseStamped messages
    x1, y1, z1 = pose1.position.x, pose1.position.y, pose1.position.z
    x2, y2, z2 = pose2.position.x, pose2.position.y, pose2.position.z
    
    # Calculate the Euclidean distance
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
    return distance