import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

COLORS = ("red", "green", "blue")


def main(args):
    df = pd.read_csv(args.input_file)

    acc = df[['ax[m/s^2]', 'ay[m/s^2]', 'az[m/s^2]']].to_numpy()
    gyr = df[['gx[rad/s]', 'gy[rad/s]', 'gz[rad/s]']].to_numpy()
    timestamp = df['timestamp[us]'].to_numpy()
    acc_std = acc.std(axis=0)
    gyr_std = gyr.std(axis=0)
    gyr_mean = gyr.mean(axis=0)
    

    fix, ax = plt.subplots(4, 2, figsize=(15, 10))
    for i, (axis, color) in enumerate(zip("xyz", COLORS)):
        ax[0, 0].plot(timestamp, acc[:, i], label=axis, color=color)
        ax[0, 0].set_xlabel('Timestamp [us]')
        ax[0, 0].set_ylabel('Acceleration [m/s^2]')
        ax[0, 0].legend()
        ax[i+1, 0].hist(acc[:, i], bins=100, color=color, alpha=0.7)
        ax[i+1, 0].set_xlabel(f'{axis} [m/s^2]')
        ax[i+1, 0].text(0.1, 0.9, f"{axis} std: {acc_std[i]:.7f}", transform=ax[i + 1, 0].transAxes)
    
        ax[0, 1].plot(timestamp, gyr[:, i], label=axis, color=color)
        ax[0, 1].set_xlabel('Timestamp [us]')
        ax[0, 1].set_ylabel('Gyroscope [rad/s]')
        ax[0, 1].legend()
        ax[i+1, 1].hist(gyr[:, i], bins=50, color=color, alpha=0.7)
        ax[i+1, 1].set_xlabel(f'{axis} [rad/s]')
        ax[i+1, 1].text(0.1, 0.9, f"{axis} std: {gyr_std[i]:.7f}", transform=ax[i + 1, 1].transAxes)
        ax[i+1, 1].text(0.1, 0.8, f"{axis} mean: {gyr_mean[i]:.7f}", transform=ax[i + 1, 1].transAxes)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="IMUデータの解析とプロット")
    parser.add_argument('input_file', type=str, help="IMUデータが保存されたテキストファイル")
    args = parser.parse_args()

    main(args)
