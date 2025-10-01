
import numpy as np
from scipy.interpolate import CubicSpline
from evo.core.trajectory import PoseTrajectory3D
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm


def calculate_yaw_from_points(x, y):
    """Calculate yaw based on the tangent of the trajectory."""
    dx = np.diff(x)
    dy = np.diff(y)
    yaw = np.arctan2(dy, dx) * 180 / np.pi  # Convert radians to degrees
    yaw = np.append(yaw, yaw[-1])  # Repeat the last yaw for array alignment
    return yaw

def calculate_pitch_from_points(x, y, z):
    """Calculate pitch based on the slope of the trajectory."""
    dx = np.diff(x)
    dy = np.diff(y)
    dz = np.diff(z)
    horizontal_distance = np.sqrt(dx**2 + dy**2)  # Distance in the horizontal plane
    pitch = np.arctan2(dz, horizontal_distance) * 180 / np.pi  # Convert radians to degrees
    pitch = np.append(pitch, pitch[-1])  # Repeat the last pitch for array alignment
    return -pitch


def trajectory_generation(points, use_yaw_input, use_pitch_input, sampling_rate) -> PoseTrajectory3D:
    if len(points) < 2:
        print("At least two points are required to generate a trajectory.")
        return

    # Extract x, y, z, yaw, roll, pitch, and velocity from points
    x, y, z, yaw, roll, pitch, velocity = zip(*points)

    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2)  # Approximate distances
    time_intervals = distances / np.minimum(velocity[:-1], velocity[1:])  # Use the smaller velocity at each segment
    t = np.insert(np.cumsum(time_intervals), 0, 0)  # Cumulative time

    # Smooth interpolation with cubic splines (zero velocity constraint)
    spline_x = CubicSpline(t, x, bc_type=((1, 0.0), (1, 0.0)))
    spline_y = CubicSpline(t, y, bc_type=((1, 0.0), (1, 0.0)))
    spline_z = CubicSpline(t, z, bc_type=((1, 0.0), (1, 0.0)))
    spline_roll = CubicSpline(t, roll, bc_type=((1, 0.0), (1, 0.0)))

    if use_yaw_input:
        spline_yaw = CubicSpline(t, yaw, bc_type=((1, 0.0), (1, 0.0)))
    if use_pitch_input:
        spline_pitch = CubicSpline(t, pitch, bc_type=((1, 0.0), (1, 0.0)))

    # Sample at 10Hz
    total_time = t[-1]
    num_samples = max(int(sampling_rate * total_time), 2)  # Ensure at least two samples
    t_smooth = np.linspace(0, total_time, num_samples)
    smooth_x = spline_x(t_smooth)
    smooth_y = spline_y(t_smooth)
    smooth_z = spline_z(t_smooth)
    smooth_roll = spline_roll(t_smooth)

    if use_yaw_input:
        smooth_yaw = spline_yaw(t_smooth)
    else:
        smooth_yaw = calculate_yaw_from_points(smooth_x, smooth_y)

    if use_pitch_input:
        smooth_pitch = spline_pitch(t_smooth)
    else:
        smooth_pitch = calculate_pitch_from_points(smooth_x, smooth_y, smooth_z)

    # Calculate velocities
    #velocity_x = spline_x.derivative()(t_smooth)
    #velocity_y = spline_y.derivative()(t_smooth)
    #velocity_z = spline_z.derivative()(t_smooth)
    #velocities = np.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)

    traj = create_evo_trajecory(smooth_x, smooth_y, smooth_z, smooth_yaw, smooth_pitch, smooth_roll, t_smooth)
    
    traj.meta["base_points"] = points

    return traj


def create_evo_trajecory(x,y,z,yaw,pitch,roll,t) -> PoseTrajectory3D:
    
    # Prepare SE3 poses and timestamps
    poses = []
    for i in range(len(t)):
        # Translation vector
        translation = np.array([x[i], y[i], z[i]])

        # Compute rotation matrix from yaw, pitch, and roll
        yaw_, pitch_, roll_ = (
            np.radians(yaw[i]),
            np.radians(pitch[i]),
            np.radians(roll[i]),
        )
        R_yaw = np.array([
            [np.cos(yaw_), -np.sin(yaw_), 0],
            [np.sin(yaw_),  np.cos(yaw_), 0],
            [0,            0,           1],
        ])
        R_pitch = np.array([
            [np.cos(pitch_), 0, np.sin(pitch_)],
            [0,             1, 0           ],
            [-np.sin(pitch_), 0, np.cos(pitch_)],
        ])
        R_roll = np.array([
            [1, 0,            0           ],
            [0, np.cos(roll_), -np.sin(roll_)],
            [0, np.sin(roll_),  np.cos(roll_)],
        ])
        rotation_matrix = R_yaw @ R_pitch @ R_roll

        # Combine translation and rotation into SE3 matrix
        se3_pose = np.eye(4)
        se3_pose[:3, :3] = rotation_matrix
        se3_pose[:3, 3] = translation

        poses.append(se3_pose)

    # Create PoseTrajectory3D
    trajectory = PoseTrajectory3D(poses_se3=poses, timestamps=t)

    return trajectory


def compute_systematic_error_transform(rotation_vector = np.zeros(3), trans_offset = np.zeros(3)):
    """
    Parameters:
    - drift_coeff_rot (3x1 numpy array): 
    - drift_coeff_trans (3x1 numpy array): 

    Returns:
    - T_offset (4x4 numpy array): SE(3) transformation representing the sensor offset
    """

    # Convert back to rotation matrix
    R_offset = R.from_rotvec(rotation_vector).as_matrix()

    # ---- CONSTRUCT DRIFT SE(3) TRANSFORMATION ----
    T_offset = np.eye(4)
    T_offset[:3, :3] = R_offset
    T_offset[:3, 3] = trans_offset

    return T_offset


def compute_corrected_transformation(T_increment, R_offset, t_offset):
    """
    Computes the corrected SE(3) transformation accounting for both:
    - A sensor translation offset
    - A sensor rotation offset
    
    Parameters:
    - T_increment (4x4 numpy array): The ideal transformation (without drift).
    - R_offset (3x3 numpy array): Rotation matrix for the sensor's misalignment.
    - t_offset (3x1 numpy array): Translation offset of the sensor from expected position.
    
    Returns:
    - T_corrected (4x4 numpy array): SE(3) transformation with both translation & rotation drift.
    """
    # Extract original transformation components
    R_increment = T_increment[:3, :3]
    t_increment = T_increment[:3, 3]

    # Compute the corrected rotation
    R_corrected = R_increment @ R_offset  # Apply the offset rotation

    # Compute the induced translation error
    t_induced = (R_corrected - np.eye(3)) @ t_offset

    # Compute the final corrected translation
    t_corrected = t_increment + t_induced

    # Construct the final corrected SE(3) transformation
    T_corrected = np.eye(4)
    T_corrected[:3, :3] = R_corrected
    T_corrected[:3, 3] = t_corrected

    return T_corrected

#def compute_noise_transform(translation_std, rotation_std):
    """
 #   Generates a stochastic noise transformation with different rotational noise per axis.

 #   Parameters:
 #   - translation_std (array-like of shape (3,)): Standard deviations for translation noise in X, Y, Z (meters).
 #   - rotation_std (array-like of shape (3,)): Standard deviations for roll (X), pitch (Y), and yaw (Z) noise (radians).

 #   Returns:
 #    - T_noise (4x4 numpy array): SE(3) transformation representing only the noise.
 #   """
    # ---- STOCHASTIC ROTATIONAL NOISE ----
    #rotation_std = np.asarray(rotation_std)  # Ensure it's a NumPy array
    #omega_noise = np.random.normal(0, rotation_std, size=3)  # Vectorized noise generation

    # Convert to rotation matrix
    #R_noise = R.from_rotvec(omega_noise).as_matrix()

    # ---- STOCHASTIC TRANSLATIONAL NOISE ----
 #   translation_std = np.asarray(translation_std)  # Ensure it's a NumPy array
 #   t_noise = np.random.normal(0, translation_std, size=3)  # Vectorized noise generation

    # ---- CONSTRUCT NOISY SE(3) TRANSFORMATION ----
#   T_noise = np.eye(4)
#    T_noise[:3, :3] = R_noise
#    T_noise[:3, 3] = t_noise

#    return T_noise 


def skew_symmetric(omega):
    """ Converts a 3D rotation vector into a skew-symmetric matrix """
    return np.array([
        [0, -omega[2], omega[1]],
        [omega[2], 0, -omega[0]],
        [-omega[1], omega[0], 0]
    ])

def compute_noise_transform(covariance):
    """
    Generates a stochastic noise transformation in SE(3) using the exponential map.
    Concept according to http://asrl.utias.utoronto.ca/~tdb/bib/barfoot_ser17.pdf , page 270

    ExpM calculation accrding to https://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf

    Parameters:
    - translation_std: Standard deviations for translation noise in X, Y, Z.
    - rotation_std: Standard deviations for rotation noise in roll (X), pitch (Y), and yaw (Z).

    Returns:
    - T_noise (4x4 numpy array): SE(3) transformation matrix representing noise.
    """

     # Ensure covariance matrix is symmetric positive semi-definite
    assert covariance.shape == (6, 6), "Covariance matrix must be 6x6."

    # Generate correlated noise vector (6D: [t_x, t_y, t_z, omega_x, omega_y, omega_z])
    correlated_noise = np.random.multivariate_normal(mean=np.zeros(6), cov=covariance)

    # Extract translation and rotation noise
    t_noise = correlated_noise[:3]  # First 3 values are translation noise
    omega_noise = correlated_noise[3:]  # Last 3 values are rotation noise


    # Compute the SO(3) rotation exponential
    theta = np.linalg.norm(omega_noise)

    if theta > 1e-6:  # Avoid division by zero for small angles
        omega_hat = skew_symmetric(omega_noise)
        omega_hat_sq = np.dot(omega_hat, omega_hat)

        #according to  https://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf, page 44
        V = (
            np.eye(3)
            + (1 - np.cos(theta)) / theta**2 * omega_hat
            + (theta - np.sin(theta)) / theta**3 * omega_hat_sq
        )

        R_noise = expm(omega_hat)  # Rodrigues' formula for SO(3)
        t_noise_transformed = np.dot(V, t_noise)  # Apply V to translation
    else:
        R_noise = np.eye(3)  # No rotation for very small angles
        t_noise_transformed = t_noise  # No transformation needed

    # Construct SE(3) noise transformation
    T_noise = np.eye(4)
    T_noise[:3, :3] = R_noise
    T_noise[:3, 3] = t_noise_transformed

    return T_noise

