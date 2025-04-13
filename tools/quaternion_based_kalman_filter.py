import numpy as np
import pandas as pd
from scipy.linalg import block_diag
from scipy.optimize import least_squares

from kf import KalmanFilter3D, State


anchor_poses = np.array([
    [0.0, 1.69, 2.03],
    [0.0, 1.69, 0.0],
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 2.03]
])


def trilateration(anchor_positions, distances, initial_guess=None):
    def residuals(pos):
        return np.linalg.norm(anchor_positions - pos, axis=1) - distances

    result = least_squares(residuals, initial_guess)
    return result.x


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
acc = df_imu[['ax[m/s^2]', 'ay[m/s^2]', 'az[m/s^2]']].to_numpy()
gyro = df_imu[['gx[rad/s]', 'gy[rad/s]', 'gz[rad/s]']].to_numpy()

# round the UWB timestamps to the nearest IMU timestamp
rounded_uwb_ts = np.array([imu_timestamps[np.abs(imu_timestamps - ts).argmin()] for ts in uwb_timestamps])
one_hot_ys = df_imu['timestamp[us]'].apply(lambda ts: 1 if ts in rounded_uwb_ts else 0).to_numpy()


imu_timestamps = imu_timestamps * 1e-6

def main():
    N = len(imu_timestamps)
    kf = KalmanFilter3D()
    states = [None] * N

    states[0] = State(p=uwb_positions[0])

    i_gnss = 0
    for k in range(1, N):
        dt = imu_timestamps[k] - imu_timestamps[k - 1]
        states[k] = kf.predict(states[k - 1], dt, acc[k - 1], gyro[k - 1])

        if one_hot_ys[k] == 1:
            states[k] = kf.update(states[k], uwb_positions[i_gnss])
            i_gnss += 1

    with open('./data/pose.txt', 'w') as f:
        f.write('timestamp[us],x[m],y[m],z[m],qx,qy,qz,qw\n')
        for k in range(acc.shape[0]):
            t_us = int(imu_timestamps[k] * 1e6)
            pos = states[k].p
            quat = states[k].q
            f.write(f"{t_us},{pos[0]},{pos[1]},{pos[2]},{quat[0]},{quat[1]},{quat[2]},{quat[3]}\n")


if __name__ == '__main__':
    main()
