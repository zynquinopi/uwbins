"""
********************************************************************************
IMU-GNSS Sensor-Fusion on the KITTI Dataset
********************************************************************************
Goals of this script:

- apply the UKF for estimating the 3D pose, velocity and sensor biases of a
  vehicle on real data.

- efficiently propagate the filter when one part of the Jacobian is already
  known. 

- efficiently update the system for GNSS position.

*We assume the reader is already familiar with the approach described in the
tutorial and in the 2D SLAM example.*

This script proposes an UKF to estimate the 3D attitude, the velocity, and the
position of a rigid body in space from inertial sensors and position
measurement.

We use the KITTI data that can be found in the `iSAM repo
<https://github.com/borglab/gtsam/blob/develop/matlab/gtsam_examples/IMUKittiExampleGNSS.m>`_
(examples folder).
"""

################################################################################
# Import
# ==============================================================================
from scipy.linalg import block_diag
import ukfm
import numpy as np
import matplotlib.pyplot as plt
# ukfm.set_matplotlib_config()

################################################################################
# Model and Data
# ==============================================================================
# This script uses the :meth:`~ukfm.IMUGNSS` model that loads the KITTI data
# from text files. The model is the standard 3D kinematics model based on
# inertial inputs.

MODEL = ukfm.IMUGNSS
# observation frequency (Hz)
GNSS_freq = 1
# load data
omegas, ys, one_hot_ys, t = MODEL.load(GNSS_freq)
N = t.shape[0]

ys_plot = np.zeros((N, 3))
cnt = 0
for i in range(len(one_hot_ys)):
    if one_hot_ys[i] == 1:
      print(f"ys[{cnt}] = {ys[cnt]}")x
      ys_plot[i] = ys[cnt]
      cnt += 1
    else:
      ys_plot[i] = np.array([np.nan, np.nan, np.nan])

# print(f"Number of measurements: {N}")
# gyro = np.array([omegas[i].gyro for i in range(N)])
# acc = np.array([omegas[i].acc for i in range(N)])
# print(t.shape)
# print(t)
# print(1/(t[-1] - t[-2]))
# fig, ax = plt.subplots(3, 1, figsize=(10, 6))
# ax[0].plot(t, gyro[:, 0], label='Gyro X')
# ax[0].plot(t, gyro[:, 1], label='Gyro Y')
# ax[0].plot(t, gyro[:, 2], label='Gyro Z')
# ax[0].set_title('Gyro Data')
# ax[0].set_xlabel('Time [s]')
# ax[0].set_ylabel('Gyro [rad/s]')
# ax[0].legend()
# ax[1].plot(t, acc[:, 0], label='Acc X')
# ax[1].plot(t, acc[:, 1], label='Acc Y')
# ax[1].plot(t, acc[:, 2], label='Acc Z')
# ax[1].set_title('Accelerometer Data')
# ax[1].set_xlabel('Time [s]')
# ax[1].set_ylabel('Accelerometer [m/s^2]')
# ax[1].legend()
# ax[2].plot(ys[:, 0], label='GNSS X')
# ax[2].plot(ys[:, 1], label='GNSS Y')
# ax[2].plot(ys[:, 2], label='GNSS Z')
# sub_ax = ax[2].twinx()
# sub_ax.plot(t, one_hot_ys, label='GNSS Measurement', color='gray', alpha=0.5)
# ax[2].set_title('GNSS Data')
# ax[2].set_xlabel('Time [s]')
# ax[2].set_ylabel('GNSS [m]')
# ax[2].legend()
# plt.show()

# IMU noise standard deviation (noise is isotropic)
imu_std = np.array([0.01,     # gyro (rad/s)
                    0.05,     # accelerometer (m/s^2)
                    0.000001, # gyro bias (rad/s^2)
                    0.0001])  # accelerometer bias (m/s^3)
# GNSS noise standard deviation (m)
GNSS_std = 0.05

################################################################################
# The state and the input contain the following variables:
#
# .. highlight:: python
# .. code-block:: python
#
#    states[n].Rot     # 3d orientation (matrix)
#    states[n].v       # 3d velocity
#    states[n].p       # 3d position
#    states[n].b_gyro  # gyro bias
#    states[n].b_acc   # accelerometer bias
#    omegas[n].gyro    # vehicle angular velocities
#    omegas[n].acc     # vehicle specific forces
#
# A measurement ``ys[k]`` contains a GNSS (position) measurement.

################################################################################
# Filter Design and Initialization
# ------------------------------------------------------------------------------
# We now design the UKF on parallelizable manifolds. This script embeds the
# state in :math:`SO(3) \times \mathbb{R}^{12}`, such that:
#
# * the retraction :math:`\varphi(.,.)` is the :math:`SO(3)` exponential for
#   orientation, and the vector addition for the remaining part of the
#   state.
#
# * the inverse retraction :math:`\varphi^{-1}_.(.)` is the :math:`SO(3)`
#   logarithm for orientation and the vector subtraction for the remaining part
#   of the state.
#
# Remaining parameter setting is standard.

# propagation noise covariance matrix
Q = block_diag(imu_std[0]**2*np.eye(3), imu_std[1]**2*np.eye(3),
               imu_std[2]**2*np.eye(3), imu_std[3]**2*np.eye(3))
# measurement noise covariance matrix
R = GNSS_std**2 * np.eye(3)

################################################################################
# We use the UKF that is able to infer Jacobian to speed up the update step, see
# the 2D SLAM example.

# sigma point parameters
alpha = np.array([1e-3, 1e-3, 1e-3, 1e-3, 1e-3])
# for propagation we need the all state
red_idxs = np.arange(15)  # indices corresponding to the full state in P
# for update we need only the state corresponding to the position
up_idxs = np.array([6, 7, 8])

################################################################################
# We initialize the state with zeros biases. The initial covariance is non-null
# as the state is not perfectly known.

# initial uncertainty matrix
P0 = block_diag(0.01*np.eye(3), 1*np.eye(3), 1*np.eye(3),
                0.001*np.eye(3), 0.001*np.eye(3))
# initial state
state0 = MODEL.STATE(
    Rot=np.eye(3),
    v=np.zeros(3),
    p=np.zeros(3),
    b_gyro=np.zeros(3),
    b_acc=np.zeros(3))

################################################################################
# As the noise affecting the dynamic of the biases is trivial (it is the
# identity), we set our UKF with a reduced propagation noise covariance, and
# manually set the remaining part of the Jacobian.

# create the UKF
ukf = ukfm.JUKF(state0=state0, P0=P0, f=MODEL.f, h=MODEL.h, Q=Q[:6, :6],
                phi=MODEL.phi, alpha=alpha, red_phi=MODEL.phi,
                red_phi_inv=MODEL.phi_inv, red_idxs=red_idxs,
                up_phi=MODEL.up_phi, up_idxs=up_idxs)
# set variables for recording estimates along the full trajectory
ukf_states = [state0]
ukf_Ps = np.zeros((N, 15, 15))
ukf_Ps[0] = P0
# the part of the Jacobian that is already known.
G_const = np.zeros((15, 6))
G_const[9:] = np.eye(6)

################################################################################
# Filtering
# ==============================================================================
# The UKF proceeds as a standard Kalman filter with a for loop.

# measurement iteration number
k = 1
end_idx = -1
t = t[:end_idx]
N = t.shape[0]
for n in range(1, N):
    print(f"Processing measurement {n}/{N}")
    # propagation
    dt = t[n]-t[n-1]
    ukf.state_propagation(omegas[n-1], dt)
    ukf.F_num(omegas[n-1], dt)
    # we assert the reduced noise covariance for computing Jacobian.
    ukf.Q = Q[:6, :6]
    ukf.G_num(omegas[n-1], dt)
    # concatenate Jacobian
    ukf.G = np.hstack((ukf.G, G_const*dt))
    # we assert the full noise covariance for uncertainty propagation.
    ukf.Q = Q
    ukf.cov_propagation()
    # update only if a measurement is received
    if one_hot_ys[n] == 1:
        ukf.update(ys[k], R)
        k = k + 1
    # save estimates
    ukf_states.append(ukf.state)
    ukf_Ps[n] = ukf.P

################################################################################
# Results
# ------------------------------------------------------------------------------
# We plot the estimated trajectory.
# fig, ax = plt.subplots(3, 1, figsize=(10, 6))
# ax[0].plot(ukf_states[1:].p[0], label='UKF Position')
# # ax[0].plot(ys[:, 0], ys[:, 1], label='GNSS Position', color='gray', alpha=0.5)
# ax[0].set_title('Position vs Time')
# ax[0].set_xlabel('Time [s]')
# ax[0].set_ylabel('Position [m]')
# ax[0].legend()
# ax[1].plot(ukf_states[1:], label='UKF Velocity')
# ax[1].set_title('Velocity vs Time')
# ax[1].set_xlabel('Time [s]')
# ax[1].set_ylabel('Velocity [m/s]')
# ax[1].legend()
# ax[2].plot(ukf_states[1:], label='UKF Orientation')
# ax[2].set_title('Orientation vs Time')
# ax[2].set_xlabel('Time [s]')
# ax[2].set_ylabel('Orientation [rad]')
fig, ax = plt.subplots(3, 2, figsize=(10, 6))
ax[0,0].plot(t[:], [state.p[0] for state in ukf_states], label='Estimated X')
ax[0,0].scatter(t[:], ys_plot[:end_idx, 0], label='GNSS X', color='gray', alpha=0.5)
ax[0,0].set_title('Position vs Time')
ax[0,0].set_xlabel('Time [s]')
ax[0,0].set_ylabel('Position [m]')
ax[1,0].plot(t[:], [state.p[1] for state in ukf_states], label='Estimated Y')
ax[1,0].scatter(t[:], ys_plot[:end_idx, 1], label='GNSS Y', color='gray', alpha=0.5)
ax[1,0].set_title('Position vs Time')
ax[1,0].set_xlabel('Time [s]')
ax[1,0].set_ylabel('Position [m]')
ax[2,0].plot(t[:], [state.p[2] for state in ukf_states], label='Estimated Z')
ax[2,0].scatter(t[:], ys_plot[:end_idx, 2], label='GNSS Z', color='gray', alpha=0.5)
ax[2,0].set_title('Position vs Time')
ax[2,0].set_xlabel('Time [s]')
ax[2,0].set_ylabel('Position [m]')
ax[0,1].plot(t[:], [state.b_gyro[0] for state in ukf_states], label='Estimated Gyro Bias X')
ax[0,1].plot(t[:], [state.b_gyro[1] for state in ukf_states], label='Estimated Gyro Bias Y')
ax[0,1].plot(t[:], [state.b_gyro[2] for state in ukf_states], label='Estimated Gyro Bias Z')
ax[0,1].set_title('Gyro Bias vs Time')
ax[0,1].set_xlabel('Time [s]')
ax[0,1].set_ylabel('Gyro Bias [rad/s]')
ax[1,1].plot(t[:], [state.b_acc[0] for state in ukf_states], label='Estimated Acc Bias X')
ax[1,1].plot(t[:], [state.b_acc[1] for state in ukf_states], label='Estimated Acc Bias Y')
ax[1,1].plot(t[:], [state.b_acc[2] for state in ukf_states], label='Estimated Acc Bias Z')
ax[1,1].set_title('Acc Bias vs Time')
ax[1,1].set_xlabel('Time [s]')
ax[1,1].set_ylabel('Acc Bias [m/s^2]')
plt.show()


# ax[0,0].plot(uwb_timestamps / 1000000, ys[:, 0], label='GNSS X')
# ax_0 = ax[0,0].twinx()
# ax_0.plot(t, one_hot_ys, label='GNSS Measurement', color='gray', alpha=0.5)
plt.show()

MODEL.plot_results(ukf_states, ys)

################################################################################
# Results are coherent with the GNSS. As the GNSS is used in the filter, it
# makes no sense to compare the filter outputs to the same measurement.

################################################################################
# Conclusion
# ==============================================================================
# This script implements an UKF for sensor-fusion of an IMU with GNSS. The UKF
# is efficiently implemented, as some part of the Jacobian are known and not
# computed. Results are satisfying.
#
# You can now:
#
# * increase the difficulties of the example by reduced the GNSS frequency or
#   adding noise to position measurements.
#
# * implement the UKF with different uncertainty representations, as viewing the
#   state as an element :math:`\boldsymbol{\chi} \in SE_2(3) \times
#   \mathbb{R}^6`. We yet provide corresponding retractions and inverse
#   retractions.
