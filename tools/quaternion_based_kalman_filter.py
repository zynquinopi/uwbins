import pickle
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pandas as pd
from scipy.linalg import block_diag
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

anchor_poses = np.array([
    [0.0, 1.69, 2.03],
    [0.0, 1.69, 0.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 2.03]
])

def skew_symmetric(v):
    v = np.asarray(v).flatten()
    return np.array(
        [[0, -v[2], v[1]],
         [v[2], 0, -v[0]],
         [-v[1], v[0], 0]]
    )

def trilateration(anchor_positions, distances, initial_guess=None):
    def residuals(pos):
        return np.linalg.norm(anchor_positions - pos, axis=1) - distances

    result = least_squares(residuals, initial_guess)
    return result.x

class Quaternion():
    def __init__(self, w=1., x=0., y=0., z=0., axis_angle=None, euler=None):
        if axis_angle is None and euler is None:
            self.w = w; self.x = x; self.y = y; self.z = z

        elif axis_angle is not None:
            axis_angle = np.array(axis_angle)
            norm = np.linalg.norm(axis_angle)
            self.w = np.cos(norm / 2)
            if norm < 1e-10:
                self.x = 0; self.y = 0; self.z = 0
            else:
                imag = axis_angle / norm * np.sin(norm / 2)
                self.x = imag[0].item(); self.y = imag[1].item(); self.z = imag[2].item()
        else:
            roll  = euler[0]; pitch = euler[1]; yaw   = euler[2]

            cy = np.cos(yaw * 0.5); sy = np.sin(yaw * 0.5)
            cr = np.cos(roll * 0.5); sr = np.sin(roll * 0.5)
            cp = np.cos(pitch * 0.5); sp = np.sin(pitch * 0.5)

            self.w = cr * cp * cy + sr * sp * sy
            self.x = sr * cp * cy - cr * sp * sy
            self.y = cr * sp * cy + sr * cp * sy
            self.z = cr * cp * sy - sr * sp * cy

    def to_mat(self):
        v = np.array([self.x, self.y, self.z]).reshape(3,1)
        return (self.w ** 2 - v.T @ v) * np.eye(3) + \
               2 * v @ v.T + 2 * self.w * skew_symmetric(v)

    def quat_mult(self, q):
        v = np.array([self.x, self.y, self.z]).reshape(3, 1)
        sum_term = np.zeros([4,4])
        sum_term[0,1:]   = -v[:,0]
        sum_term[1:, 0]  = v[:,0]
        sum_term[1:, 1:] = -skew_symmetric(v)
        sigma = self.w * np.eye(4) + sum_term

        return sigma @ q

# sys.path.append('./data')
# with open('data/data.pickle', 'rb') as file:
#     data = pickle.load(file)

df_imu = pd.read_csv("./data/imu.txt")
df_uwb = pd.read_csv("./data/uwb.txt")

N = len(df_imu)

# preprocess UWB
uwb_timestamps = df_uwb[df_uwb['anchor_id'] == 0]['timestamp[us]'].to_numpy()
uwbs = []
for i in range(0, 4):
  uwbs.append(df_uwb[df_uwb['anchor_id'] == i]['distance[m]'].tolist())
uwbs = np.array(uwbs).T
uwb_positions = []
first_position = np.array([1.0, 0.845, 1.015])
for i in range(len(uwbs)):
  uwb_positions.append(trilateration(anchor_poses, uwbs[i], first_position))
  first_position = uwb_positions[-1]
uwb_positions = np.array(uwb_positions)

# preprocess UWB
imu_timestamps = df_imu['timestamp[us]'].to_numpy()
imu_acc = df_imu[['ax[m/s^2]', 'ay[m/s^2]', 'az[m/s^2]']].to_numpy()
imu_gyro = df_imu[['gx[rad/s]', 'gy[rad/s]', 'gz[rad/s]']].to_numpy()

# round the UWB timestamps to the nearest IMU timestamp
rounded_uwb_ts = np.array([imu_timestamps[np.abs(imu_timestamps - ts).argmin()] for ts in uwb_timestamps])
one_hot_ys = df_imu['timestamp[us]'].apply(lambda ts: 1 if ts in rounded_uwb_ts else 0).to_numpy()


imu_timestamps = imu_timestamps * 1e-6

var_imu_a = 0.01
var_imu_w = 0.01
var_gnss = 0.1
gravity = 9.81

g = np.array([0, 0, -gravity])
lJac = np.zeros([9, 6])         # motion model noise jacobian
lJac[3:, :] = np.eye(6)
hJac = np.zeros([3, 9])         # measurement model jacobian
hJac[:, :3] = np.eye(3)

pEst = np.zeros([imu_acc.shape[0], 3])     # position estimates
vEst = np.zeros([imu_acc.shape[0], 3])     # velocity estimates
qEst = np.zeros([imu_acc.shape[0], 4])     # orientation estimates as quaternions
pCov = np.zeros([imu_acc.shape[0], 9, 9])  # covariance matrices

# Initial values
pEst[0] = uwb_positions[0]
vEst[0] = np.array([0.0, 0.0, 0.0])
qEst[0] = np.array([1.0, 0.0, 0.0, 0.0])
pCov[0] = np.eye(9)

def predict_update(pCovTemp, pTemp, vTemp, qTemp, delta_t, imu_a,  imu_w):

    Rotation_Mat = Quaternion(*qTemp).to_mat()
    pEst = pTemp + delta_t * vTemp + 0.5 * (delta_t ** 2) * (Rotation_Mat @ imu_a - g)
    vEst = vTemp + delta_t * (Rotation_Mat @ imu_a - g)
    qEst = Quaternion(euler = delta_t * imu_w).quat_mult(qTemp)

    F = np.eye(9)
    imu = imu_a.reshape((3, 1))
    F[0:3, 3:6] = delta_t * np.eye(3)
    F[3:6, 6:9] = Rotation_Mat @ (-skew_symmetric(imu)) * delta_t

    Q = np.eye(6)
    Q[0:3, 0:3] = var_imu_a * Q[0:3, 0:3]
    Q[3:6, 3:6] = var_imu_w * Q[3:6, 3:6]
    Q = Q * (delta_t ** 2)
    pCov = F @ pCovTemp @ F.T + lJac @ Q @ lJac.T
    return pEst, vEst, qEst, pCov

def measurement_update(pCovTemp, y_k, pTemp, vTemp, qTemp):

    RCov = var_gnss * np.eye(3)
    K = pCovTemp @ hJac.T @ np.linalg.inv(hJac @ pCovTemp @ hJac.T + RCov)

    delta_x = K @ (y_k - pTemp)

    pTemp = pTemp + delta_x[:3]
    vTemp = vTemp + delta_x[3:6]
    qTemp = Quaternion(axis_angle = delta_x[6:]).quat_mult(qTemp)

    pCovTemp = (np.eye(9) - K @ hJac) @ pCovTemp

    return pTemp, vTemp, qTemp, pCovTemp

def main():
    print(__file__ + " start!!")

    i_gnss = 0

    hz = np.zeros((3,1))
    est_traj_fig = plt.figure()
    ax = est_traj_fig.add_subplot(111, projection='3d')


    for k in range(1, imu_acc.shape[0]):

        delta_t = imu_timestamps[k] - imu_timestamps[k - 1]
        pEst[k], vEst[k], qEst[k], pCov[k] \
            = predict_update(pCov[k-1], pEst[k-1], vEst[k-1], qEst[k-1], delta_t, imu_acc[k - 1], imu_gyro[k - 1])

        if one_hot_ys[k] == 1:
            uwb = uwb_positions[i_gnss]
            pEst[k], vEst[k], qEst[k], pCov[k] = measurement_update(pCov[k], uwb, pEst[k], vEst[k], qEst[k])
            hz = np.hstack((hz, uwb.reshape(3,1)))
            i_gnss += 1
        # if i_gnss < gnss_t.shape[0] and abs(gnss_t[i_gnss] - imu_a_t[k]) < 0.01:
        #     pEst[k], vEst[k], qEst[k], pCov[k] = measurement_update(pCov[k],
        #                                                 gnss[i_gnss], pEst[k], vEst[k], qEst[k])
        #     hz = np.hstack((hz, gnss[i_gnss].reshape(3,1)))
        #     i_gnss += 1

        # if (k % 5 == 0):
        #     plt.cla()
        #     ax.plot(hz[0,:], hz[1,:], hz[2,:], ".g", label='GPS')
        #     ax.plot(pEst[:,0], pEst[:,1], pEst[:,2], '.r', label='Estimated', markersize=1)
        #     # ax.plot(gt_p[:,0], gt_p[:,1], gt_p[:,2], '-b', label='Ground Truth')
        #     ax.set_xlabel('x [m]')
        #     ax.set_ylabel('y [m]')
        #     ax.set_zlabel('z [m]')
        #     ax.set_title('Estimated Trajectory')
        #     ax.legend()
        #     ax.set_zlim(-1, 1)
        #     plt.pause(0.1)

    with open('pose.txt', 'w') as f:
        f.write('timestamp_us,x[m],y[m],z[m],qx,qy,qz,qw\n')
        
        for k in range(imu_acc.shape[0]):
            t_us = int(imu_timestamps[k] * 1e6)
            pos = pEst[k]
            quat = qEst[k] # w x y z
            f.write(f"{t_us},{pos[0]},{pos[1]},{pos[2]},{quat[1]},{quat[2]},{quat[3]},{quat[0]}\n")

if __name__ == '__main__':
            main()
