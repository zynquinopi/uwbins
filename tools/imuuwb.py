import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.linalg import block_diag
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import ukfm


def trilateration(anchor_positions, distances, initial_guess=None):
    def residuals(pos):
        return np.linalg.norm(anchor_positions - pos, axis=1) - distances

    result = least_squares(residuals, initial_guess)
    return result.x

################################################################################
# Model and Data
# ==============================================================================
# This script uses the :meth:`~ukfm.IMUGNSS` model that loads the KITTI data
# from text files. The model is the standard 3D kinematics model based on
# inertial inputs.

# MODEL = ukfm.IMUGNSS
# # observation frequency (Hz)
# GNSS_freq = 1
# # load data
# omegas, ys, one_hot_ys, t = MODEL.load(GNSS_freq)
# N = t.shape[0]

MODEL = ukfm.IMUGNSS
df_imu = pd.read_csv("C:/Users/tysir/Documents/Projects/uwbins/data/imu.txt")
df_uwb = pd.read_csv("C:/Users/tysir/Documents/Projects/uwbins/data/uwb.txt")
# anchor_poses = np.array([
#     [1.69, 0.0, 2.03],
#     [1.69, 0.0, 0.0],
#     [0.0, 0.0, 0.0],
#     [0.0, 0.0, 2.03]
# ])
anchor_poses = np.array([
    [0.0, 1.69, 2.03],
    [0.0, 1.69, 0.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 2.03]
])
N = len(df_imu)

# preprocess UWB
uwb_timestamps = df_uwb[df_uwb['anchor_id'] == 0]['timestamp[us]'].to_numpy()
uwbs = []
for i in range(0, 4):
  uwbs.append(df_uwb[df_uwb['anchor_id'] == i]['distance[m]'].tolist())
uwbs = np.array(uwbs).T

uwb_positions = []
first_position = np.array([1.0, 0.845, 1.015])
# first_position = np.array([0.0, -0.845, 1.015])              # coord conversion
for i in range(len(uwbs)):
  uwb_positions.append(trilateration(anchor_poses, uwbs[i], first_position))
  # tmp = trilateration(anchor_poses, uwbs[i], first_position) # coord conversion
  # uwb_positions.append([tmp[1], -tmp[0], tmp[2]])            # coord conversion
  first_position = uwb_positions[-1]
ys = np.array(uwb_positions)

# preprocess UWB
imu_timestamps = df_imu['timestamp[us]'].to_numpy()
# imu_acc = df_imu[['ax[m/s^2]', 'ay[m/s^2]', 'az[m/s^2]']].to_numpy()
# imu_gyro = df_imu[['gx[rad/s]', 'gy[rad/s]', 'gz[rad/s]']].to_numpy()
imu_acc_raw = df_imu[['ax[m/s^2]', 'ay[m/s^2]', 'az[m/s^2]']].to_numpy()
imu_gyro_raw = df_imu[['gx[rad/s]', 'gy[rad/s]', 'gz[rad/s]']].to_numpy()
imu_acc = np.stack([
    imu_acc_raw[:, 1],          # 新しい ax ← 元の ay
    -imu_acc_raw[:, 0],         # 新しい ay ← -元の ax（符号反転）
    imu_acc_raw[:, 2]           # azはそのまま
], axis=1)
imu_gyro = np.stack([
    imu_gyro_raw[:, 1],        # 新しい gx ← 元の gy
    -imu_gyro_raw[:, 0],       # 新しい gy ← -元の gx（符号反転）
    imu_gyro_raw[:, 2]         # gzはそのまま
], axis=1)

omegas =[]
for i in range(N):
    omegas.append(MODEL.INPUT(
        gyro=np.array([imu_gyro[i][0], imu_gyro[i][1], imu_gyro[i][2]]),
        acc=np.array([imu_acc[i][0], imu_acc[i][1], imu_acc[i][2]])
    ))

# round the UWB timestamps to the nearest IMU timestamp
rounded_uwb_ts = np.array([imu_timestamps[np.abs(imu_timestamps - ts).argmin()] for ts in uwb_timestamps])
one_hot_ys = df_imu['timestamp[us]'].apply(lambda ts: 1 if ts in rounded_uwb_ts else 0).to_list()


################################################################################
# IMU noise standard deviation (noise is isotropic)
imu_std = np.array([0.01,     # gyro (rad/s)                                   # TODO:
                    0.05,     # accelerometer (m/s^2)                          # TODO:
                    0.000001, # gyro bias (rad/s^2)                            # TODO:
                    0.0001])  # accelerometer bias (m/s^3)                     # TODO:
# GNSS noise standard deviation (m)
GNSS_std = 0.001                                                                 # TODO:

Q = block_diag(imu_std[0]**2*np.eye(3), imu_std[1]**2*np.eye(3),
               imu_std[2]**2*np.eye(3), imu_std[3]**2*np.eye(3))
R = GNSS_std**2 * np.eye(3)

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


# ################################################################################ final check
# gyro = np.array([omegas[i].gyro for i in range(N)])
# acc = np.array([omegas[i].acc for i in range(N)])
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

# ################################################################################
# Filtering
# ==============================================================================
# The UKF proceeds as a standard Kalman filter with a for loop.
# measurement iteration number
t = imu_timestamps / 1e6 # convert to seconds
k = 1
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

# ################################################################################
# dump the results
with open("pose.txt", "w") as f:
    f.write("timestamp[us],x[m],y[m],z[m],qx,qy,qz,qw\n")
    for i in range(len(ukf_states)):
        state = ukf_states[i]
        timestamp = imu_timestamps[i]
        x, y, z = state.p
        q = Rotation.from_matrix(state.Rot).as_quat()
        f.write(f"{timestamp},{x},{y},{z},{q[0]},{q[1]},{q[2]},{q[3]}\n")
################################################################################
# Results
# ------------------------------------------------------------------------------
# We plot the estimated trajectory.
fig, ax = plt.subplots(3, 2, figsize=(10, 6))
ax[0,0].plot(t, [state.p[0] for state in ukf_states], label='Estimated X')
ax[0,0].plot(uwb_timestamps / 1000000, ys[:, 0], label='GNSS X')
ax_0 = ax[0,0].twinx()
ax_0.plot(t, one_hot_ys, label='GNSS Measurement', color='gray', alpha=0.5)

ax[1,0].plot(t, [state.p[1] for state in ukf_states], label='Estimated Y')
ax[1,0].plot(uwb_timestamps / 1000000, ys[:, 1], label='GNSS Y')
ax_1 = ax[1,0].twinx()
ax_1.plot(t, one_hot_ys, label='GNSS Measurement', color='gray', alpha=0.5)

ax[2,0].plot(t, [state.p[2] for state in ukf_states], label='Estimated Z')
ax[2,0].plot(uwb_timestamps / 1000000, ys[:, 2], label='GNSS Z')
ax_2 = ax[2,0].twinx()
ax_2.plot(t, one_hot_ys, label='GNSS Measurement', color='gray', alpha=0.5)

ax[0,0].set_title('Estimated Trajectory')
ax[0,0].set_xlabel('Time [s]')
ax[0,0].set_ylabel('X [m]')
ax[1,0].set_xlabel('Time [s]')
ax[1,0].set_ylabel('Y [m]')
ax[2,0].set_xlabel('Time [s]')
ax[2,0].set_ylabel('Z [m]')
ax[0,0].legend()
ax[1,0].legend()
ax[2,0].legend()

ax[0, 1].plot(t, [state.b_gyro[0] for state in ukf_states], label='Estimated Gyro Bias X')
ax[0, 1].plot(t, [state.b_gyro[1] for state in ukf_states], label='Estimated Gyro Bias Y')
ax[0, 1].plot(t, [state.b_gyro[2] for state in ukf_states], label='Estimated Gyro Bias Z')

ax[1, 1].plot(t, [state.b_acc[0] for state in ukf_states], label='Estimated Accel Bias X')
ax[1, 1].plot(t, [state.b_acc[1] for state in ukf_states], label='Estimated Accel Bias Y')
ax[1, 1].plot(t, [state.b_acc[2] for state in ukf_states], label='Estimated Accel Bias Z')

plt.show()