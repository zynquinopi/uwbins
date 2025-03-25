import math
import time
import csv

import serial
import numpy as np
import pandas as pd
import rerun as rr
import rerun.blueprint as rrb


GRAY = np.array([0.5, 0.5, 0.5])
DARK_GRAY = np.array([0.3, 0.3, 0.3])
F = 500
W = 640
H = 480
M = np.array([
    [F, 0, W/2],
    [0, F, H/2],
    [0, 0, 1]
])


SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
TIMEOUT = 10


def add_grid(entity_path: str, xlim: np.ndarray, ylim: np.ndarray,
             is_major: bool = None) -> None:
    """Log xy grid to Rerun viewer.

    Parameters
    ----------
    entity_path : str
        Path to the entity in the space hierarchy.
    xlim : (2,) shaped array like object of float
        Min and max value for grid edges in the x-axis.
    ylim : (2,) shaped array like object of float
        Min and max value for grid edges in the y-axis.
    is_major : bool, optional
        Set to True to draw major grid lines at one-meter intervals, or False for
        minor lines at one-tenth-meter intervals. If not given, draw both grid lines.
    """
    if is_major is None:
        add_grid(entity_path, xlim, ylim, is_major=True)
        add_grid(entity_path, xlim, ylim, is_major=False)
        return
    elif is_major:
        step = 1.0
        radii = 0.002
        entity_path = entity_path + "/major"
    else:
        step = 0.1
        radii = 0.001
        entity_path = entity_path + "/minor"

    xlim = (np.floor(xlim[0]), np.ceil(xlim[1]))
    ylim = (np.floor(ylim[0]), np.ceil(ylim[1]))
    xrange = np.arange(xlim[0], xlim[1] + step, step)
    yrange = np.arange(ylim[0], ylim[1] + step, step)
    xlines = [[[xlim[0], y, 0], [xlim[1], y, 0]] for y in yrange]
    ylines = [[[x, ylim[0], 0], [x, ylim[1], 0]] for x in xrange]
    lines = rr.LineStrips3D(np.array(xlines + ylines), colors=DARK_GRAY, radii=radii)
    rr.log(entity_path, lines, static=True)


def main():

    my_blueprint = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Vertical(
            rrb.TimeSeriesView(origin="/sensor/acc"),
            rrb.TimeSeriesView(origin="/sensor/gyro"),
            # rrb.TimeSeriesView(origin="/gt/position"),
            # rrb.TimeSeriesView(origin="/gt/rotation"),
            rrb.TimeSeriesView(origin="/sensor/uwb"),
        ),
        rrb.Vertical(
            rrb.Spatial3DView(
                origin="/3d",
            ),
        ),
        column_shares=[4, 6],
    ),
    rrb.BlueprintPanel(state="collapsed"),
    rrb.SelectionPanel(state="collapsed"),
    rrb.TimePanel(state="expanded"),
    )
    rr.init("rerun_example_my_data", spawn=True)
    rr.serve_web()
    rr.send_blueprint(my_blueprint, make_active=True)

    #plot coordinate arrow on 3D view
    rr.log("gt", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        "gt/xyz",
        rr.Arrows3D(
            vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        ),
    )

    anchor_pos_list = [
        [0, 0, 0, 0],
        [1, 0, 1, 0],
        [2, 1, 1, 0],
        [3, 1, 0, 0],
    ]

    anchor_rot_list = [
        [0, 0, 0, 0, 1],
        [1, 0, 0, 0, 1],
        [2, 0, 0, 0, 1],
        [3, 0, 0, 0, 1],
    ]
    anchor_pos = pd.DataFrame(anchor_pos_list, columns=["ID", "X", "Y", "Z"])
    anchor_rot = pd.DataFrame(anchor_rot_list, columns=["ID", "Qx", "Qy", "Qz", "Qw"])

    #plot anchor positions
    # rr.log("/gt/anchor",
    #     rr.Boxes3D(centers=anchor_pos[["X", "Y", "Z"]].values,
    #                 half_sizes=np.tile([0.01, 0.5, 0.1], (4, 1)),
    #                 quaternions=[rr.Quaternion.identity(), rr.Quaternion(xyzw=anchor_rot[["Qx", "Qy", "Qz", "Qw"]].values)]),
    #                 static=True)
    
    add_grid("/gt", np.array([-10, 10]), np.array([-10, 10]))


    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud...")

            while True:
                data = ser.readline().decode("utf-8", errors="ignore").strip()
                print(data)
                if data.startswith("ts="):
                    data_dict = {key: value for key, value in (item.split("=") for item in data.split(","))}
                    timestamp = int(data_dict["ts"])
                    temperature = float(data_dict["temp"])
                    ax, ay, az = float(data_dict["ax"]), float(data_dict["ay"]), float(data_dict["az"])
                    gx, gy, gz = float(data_dict["gx"]), float(data_dict["gy"]), float(data_dict["gz"])

                    rr.log("/sensor/acc/acc_x", rr.Scalar(ax))
                    rr.log("/sensor/acc/acc_y", rr.Scalar(ay))
                    rr.log("/sensor/acc/acc_z", rr.Scalar(az))
                    rr.log("/sensor/gyro/gyro_x", rr.Scalar(gx))
                    rr.log("/sensor/gyro/gyro_y", rr.Scalar(gy))
                    rr.log("/sensor/gyro/gyro_z", rr.Scalar(gz))
                    # rr.log(
                    #     "/sensor/acc/acc_x",
                    #     times=[rr.TimeSecondsColumn("step", timestamp)],
                    #     components=[rr.components.Scalar(x)],
                    # )
                elif data.startswith("sensor_type=uwb"):
                    data_dict = {key: value for key, value in (item.split("=") for item in data.split(","))}
                    timestamp = int(data_dict["ts"])
                    anchor_id = int(data_dict["anchor_id"])
                    nlos = int(data_dict["nlos"])
                    distance = float(data_dict["distance"])
                    azimuth = float(data_dict["azimuth"])
                    elevation = float(data_dict["elevation"])
                    rr.log(f"/sensor/uwb/distance/{anchor_id}", rr.Scalar(distance))

                
                    center = np.array([0, 0, 0])
                    sigmas = np.array([5, 5, 5])
                    color_pallet = [
                        [255, 0, 0],
                        [0, 255, 0],
                        [0, 0, 255],
                        [255, 255, 0],
                    ]
                    rr.log(
                        f"/3d/{anchor_id}",
                        # rr.Capsules3D(
                        #     lengths=[0.0],
                        #     radii=[distance/100],
                        #     colors=[
                        #         (10, 255, 0, 0),
                        #     ],
                        #     translations=[
                        #         tuple(anchor_pos_list[anchor_id][1:]),
                        #     ],
                        #     rotation_axis_angles=[
                        #         rr.RotationAxisAngle([1.0, 0.0, 0.0], rr.Angle(deg=0))
                        #     ],
                        # ),
                        # rr.Capsules3D(
                        #     lengths=0.0,
                        #     radii=distance/100,
                        #     colors=(0.0, 1.0, 0.0, 0.8),
                        #     translations=tuple(anchor_pos_list[anchor_id][1:]),
                        #     rotation_axis_angles=rr.RotationAxisAngle([1.0, 0.0, 0.0], rr.Angle(deg=0))
                        # ),
                        rr.Ellipsoids3D(
                        #    centers=[center, center],
                        #     half_sizes=[sigmas, 3 * sigmas],
                        #     colors=[[255, 255, 0], [64, 64, 0]],
                            centers=[anchor_pos_list[anchor_id][1:]],
                            half_sizes=[np.array([distance/100, distance/100, distance/100])],
                            colors=[color_pallet[anchor_id]],
                            fill_mode=2,
                            line_radii=0.002,
                        )
                    )

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":

    main()
