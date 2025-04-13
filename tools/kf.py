import numpy as np
from scipy.spatial.transform import Rotation as R


def skew_symmetric(v):
    v = v.flatten()
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


class State:
    def __init__(self, p=None, v=None, q=None, cov=None):
        self.p = np.zeros(3) if p is None else p
        self.v = np.zeros(3) if v is None else v
        self.q = np.array([0.0, 0.0, 0.0, 1.0]) if q is None else q
        self.cov = np.eye(9) if cov is None else cov

    def copy(self):
        return State(
            p =   self.p.copy(),
            v =   self.v.copy(),
            q =   self.q.copy(),
            cov = self.cov.copy()
        )


class KalmanFilter3D:
    def __init__(self, var_imu_a=0.01, var_imu_w=0.01, var_gnss=0.01, gravity=9.81):
        self.var_imu_a = var_imu_a
        self.var_imu_w = var_imu_w
        self.var_gnss = var_gnss
        self.g = np.array([0, 0, gravity])

        self.lJac = np.zeros((9, 6))
        self.lJac[3:, :] = np.eye(6)

        self.hJac = np.zeros((3, 9))
        self.hJac[:, :3] = np.eye(3)


    def predict(self, state: State, dt, acc, gyro) -> State:
        R_mat = R.from_quat(state.q).as_matrix()
        p = state.p + dt * state.v + 0.5 * dt ** 2 * (R_mat @ acc - self.g)
        v = state.v + dt * (R_mat @ acc - self.g)
        q = (R.from_rotvec(dt * gyro) * R.from_quat(state.q)).as_quat()

        F = np.eye(9)
        F[0:3, 3:6] = dt * np.eye(3)
        F[3:6, 6:9] = R_mat @ (-skew_symmetric(acc.reshape(3, 1))) * dt

        Q = np.zeros((6, 6))
        Q[:3, :3] = self.var_imu_a * np.eye(3)
        Q[3:, 3:] = self.var_imu_w * np.eye(3)
        Q = Q * dt ** 2

        cov = F @ state.cov @ F.T + self.lJac @ Q @ self.lJac.T

        return State(p, v, q, cov)


    def update(self, state: State, measurement) -> State:
        R_cov = self.var_gnss * np.eye(3)
        K = state.cov @ self.hJac.T @ np.linalg.inv(self.hJac @ state.cov @ self.hJac.T + R_cov)
        delta_x = K @ (measurement - state.p)

        p = state.p + delta_x[:3]
        v = state.v + delta_x[3:6]
        q = (R.from_rotvec(delta_x[6:]) * R.from_quat(state.q)).as_quat()
        cov = (np.eye(9) - K @ self.hJac) @ state.cov

        return State(p, v, q, cov)