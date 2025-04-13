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

    fix, ax = plt.subplots(3, 2, figsize=(15, 10))
    for i, color in enumerate(COLORS):
        ax[0, 0].plot(timestamp, uwbs[:, i] - uwb_mean[i], label=f"distance_{i}", color=color)
        ax[0, 0].set_xlabel('Timestamp [us]')
        ax[0, 0].set_ylabel('Distance [m]')
        ax[0, 0].legend()
        ax[0, 1].hist(uwbs[:, i] - uwb_mean[i], bins=30, color=color, alpha=0.7)
        ax[0, 1].set_xlabel(f'distance [m]')
        ax[0, 1].text(0.1, 0.9 - i*0.1, f"std_{i}: {uwb_std[i]:.7f} m", transform=ax[0, 1].transAxes)

    for i, (axis,color) in enumerate(zip("xyz", "rgb")):
        ax[1, 0].plot(timestamp, positions[:, i], label=axis, color=color)
        ax[1, 0].set_xlabel('Timestamp [us]')
        ax[1, 0].set_ylabel('Position [m]')
        ax[1, 0].legend()
        ax[1, 1].hist(positions[:, i] - positions_mean[i], bins=30, color=color, alpha=0.7)
        ax[1, 1].set_xlabel(f'position [m]')
        ax[1, 1].text(0.1, 0.9 - i*0.1, f"std_{axis}: {position_std[i]:.7f} m", transform=ax[1, 1].transAxes)
        
        ax[2, 0].plot(timestamp, position_norms, label=f"position_norm", color=color)
        ax[2, 0].set_xlabel('Timestamp [us]')
        ax[2, 0].set_ylabel('Position Norm [m]')
        ax[2, 0].legend()
        ax[2, 1].hist(position_norms, bins=30, color=color, alpha=0.7)
        ax[2, 1].set_xlabel(f'position_norm [m]')
        ax[2, 1].text(0.1, 0.9, f"std: {position_norms_std:.7f} m", transform=ax[2, 1].transAxes)
        ax[2, 1].text(0.1, 0.8, f"mean: {position_norms_mean:.7f} m", transform=ax[2, 1].transAxes)

        
        # ax[i+1, 0].hist(acc[:, i], bins=100, color=color, alpha=0.7)
        # ax[i+1, 0].set_xlabel(f'{axis} [m/s^2]')
        # ax[i+1, 0].text(0.1, 0.9, f"{axis} std: {acc_std[i]:.7f}", transform=ax[i + 1, 0].transAxes)
    
        # ax[0, 1].plot(timestamp, gyr[:, i], label=axis, color=color)
        # ax[0, 1].set_xlabel('Timestamp [us]')
        # ax[0, 1].set_ylabel('Gyroscope [rad/s]')
        # ax[0, 1].legend()
        # ax[i+1, 1].hist(gyr[:, i], bins=50, color=color, alpha=0.7)
        # ax[i+1, 1].set_xlabel(f'{axis} [rad/s]')
        # ax[i+1, 1].text(0.1, 0.9, f"{axis} std: {gyr_std[i]:.7f}", transform=ax[i + 1, 1].transAxes)
        # ax[i+1, 1].text(0.1, 0.8, f"{axis} mean: {gyr_mean[i]:.7f}", transform=ax[i + 1, 1].transAxes)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="UWBデータの解析とプロット")
    parser.add_argument('input_file', type=str, help="UWBデータが保存されたテキストファイル")
    args = parser.parse_args()

    main(args)
