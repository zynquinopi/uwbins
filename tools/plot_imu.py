import argparse
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_histogram(df):
    # タイムスタンプの差分を計算
    df['timestamp_diff'] = df['timestamp[us]'].diff()  # 時間差分を計算
    plt.figure(figsize=(10, 6))
    plt.hist(df['timestamp_diff'].dropna(), bins=50, color='skyblue', edgecolor='black')
    plt.title('Timestamp Difference Histogram')
    plt.xlabel('Timestamp Difference [us]')
    plt.ylabel('Frequency')
    plt.grid(True)
    plt.show()

def plot_temperature(df):
    # 温度の時系列プロット
    plt.figure(figsize=(10, 6))
    plt.plot(df['timestamp[us]'], df['temp[celsius]'], color='orange')
    plt.title('Temperature vs Time')
    plt.xlabel('Timestamp [us]')
    plt.ylabel('Temperature [Celsius]')
    plt.grid(True)
    plt.show()

def plot_acceleration(df):
    # 加速度の時系列プロット
    plt.figure(figsize=(10, 6))
    plt.plot(df['timestamp[us]'], df['ax[m/s^2]'], label='ax', color='blue')
    plt.plot(df['timestamp[us]'], df['ay[m/s^2]'], label='ay', color='green')
    plt.plot(df['timestamp[us]'], df['az[m/s^2]'], label='az', color='red')
    plt.title('Acceleration vs Time')
    plt.xlabel('Timestamp [us]')
    plt.ylabel('Acceleration [m/s^2]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.show()

def plot_gyro(df):
    # ジャイロの時系列プロット
    plt.figure(figsize=(10, 6))
    plt.plot(df['timestamp[us]'], df['gx[rad/s]'], label='gx', color='purple')
    plt.plot(df['timestamp[us]'], df['gy[rad/s]'], label='gy', color='brown')
    plt.plot(df['timestamp[us]'], df['gz[rad/s]'], label='gz', color='pink')
    plt.title('Gyroscope vs Time')
    plt.xlabel('Timestamp [us]')
    plt.ylabel('Gyroscope [rad/s]')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.show()

def main(args):
    # ファイルを読み込む
    df = pd.read_csv(args.input_file)

    # ヒストグラム
    plot_histogram(df)

    # 温度の時系列プロット
    plot_temperature(df)

    # 加速度の時系列プロット
    plot_acceleration(df)

    # ジャイロの時系列プロット
    plot_gyro(df)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="IMUデータの解析とプロット")
    parser.add_argument('input_file', type=str, help="IMUデータが保存されたテキストファイル")
    args = parser.parse_args()

    main(args)
