import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import block_diag
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


COLORS = ("red", "green", "blue", "yellow")

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


def main(args):
    df = pd.read_csv(args.input_file)
    df_pose = pd.read_csv("./data/pose.txt")

    timestamp = df[df['anchor_id'] == 0]['timestamp[us]'].to_numpy()

    uwbs = []
    for i in range(0, len(anchor_poses)):
        uwbs.append(df[df['anchor_id'] == i]['distance[m]'].tolist())
    uwbs = np.array(uwbs).T

    positions = []
    first_position = np.array([1.0, 0.845, 1.015])
    for i in range(len(uwbs)):
        positions.append(trilateration(anchor_poses, uwbs[i], first_position))
        first_position = positions[-1]
    positions = np.array(positions)
    position_norms = np.linalg.norm(positions, axis=1)

    uwb_std = uwbs.std(axis=0)
    uwb_mean = uwbs.mean(axis=0)
    position_std = positions.std(axis=0)
    positions_mean = positions.mean(axis=0)
    position_norms_std = position_norms.std()
    position_norms_mean = position_norms.mean()

    pose_timestamp = df_pose['timestamp[us]'].to_numpy()
    pose_positions = df_pose[['x[m]', 'y[m]', 'z[m]']].to_numpy()

    fix, ax = plt.subplots(1, 1, figsize=(30, 15))
    for i, (axis,color) in enumerate(zip("xyz", "rgb")):
        ax.plot(timestamp, positions[:, i], label=axis, color=color)
        ax.plot(pose_timestamp, pose_positions[:, i], label=f"pose_{axis}", color=color, linestyle='dashed')
        ax.set_xlabel('Timestamp [us]')
        ax.set_ylabel('Position [m]')
        ax.legend()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="UWBデータの解析とプロット")
    parser.add_argument('input_file', type=str, help="UWBデータが保存されたテキストファイル")
    args = parser.parse_args()

    main(args)
