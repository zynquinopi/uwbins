import math
import time
import csv

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

def read_anchor_pose(file_path):
    positions = []
    quaternions = []
    with open(file_path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) == 4:
                positions.append(row)
            elif len(row) == 5:
                quaternions.append(row)
    df_positions = pd.DataFrame(positions, columns=["ID", "X", "Y", "Z"])
    df_quaternions = pd.DataFrame(quaternions, columns=["ID", "Qx", "Qy", "Qz", "Qw"])
    return df_positions, df_quaternions


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


# load csv using numpy
data = np.genfromtxt("/home/refsys/work/uwb-imu/ext/util-uwb-dataset/dataset/flight-dataset/csv-data/const1/const1-trial1-tdoa2.csv", delimiter=",", skip_header=1, filling_values=0)
anchor_pos, anchor_rot = read_anchor_pose("/home/refsys/work/uwb-imu/ext/util-uwb-dataset/dataset/flight-dataset/survey-results/anchor_const1_survey.txt")
my_blueprint = rrb.Blueprint(
    rrb.Horizontal(
        rrb.Vertical(
            rrb.TimeSeriesView(origin="/sensor/acc"),
            rrb.TimeSeriesView(origin="/sensor/gyro"),
            rrb.TimeSeriesView(origin="/gt/position"),
            rrb.TimeSeriesView(origin="/gt/rotation"),
        ),
        rrb.Vertical(
            rrb.Spatial3DView(
                origin="/gt",
            ),
        ),
        column_shares=[4, 6],
    ),
    rrb.BlueprintPanel(state="collapsed"),
    rrb.SelectionPanel(state="collapsed"),
    rrb.TimePanel(state="expanded"),
)
rr.init("rerun_example_my_data", spawn=True)
rr.serve()
rr.send_blueprint(my_blueprint, make_active=True)





acc_timestamp = data[:, 4]
accs = data[:, 5:8]
gyro_timestamp = data[:, 8]
gyro = data[:, 9:12]
pose_timestamp = data[:, 19]
positions = data[:, 20:23]
rotation = data[:, 23:27]
tdoa_timestamp = data[:, 0]
tdoa = data[:, 1:4]

# for ts, acc in zip(timestamp, acc):
#     rr.set_time_sequence("step", ts)
#     rr.log("scalar", rr.Scalar(acc[0]))
#     time.sleep(0.01)

# for i in range(len(data)):
#     rr.set_time_seconds("step", timestamp[i])
#     rr.log("acc/1", rr.Scalar(acc[i, 0]))
#     rr.log("acc/2", rr.Scalar(acc[i, 1]))
#     rr.log("acc/3", rr.Scalar(acc[i, 2]))
#     rr.log("gyro/1", rr.Scalar(gyro[i, 0]))
#     rr.log("gyro/2", rr.Scalar(gyro[i, 1]))
#     rr.log("gyro/3", rr.Scalar(gyro[i, 2]))
#     time.sleep(timestamp[i+1] - timestamp[i])


rr.send_columns(
    "/sensor/acc/acc_x",
    times=[rr.TimeSecondsColumn("step", acc_timestamp)],
    components=[rr.components.ScalarBatch(accs[:, 0])],
)
rr.send_columns(
    "/sensor/acc/acc_y",
    times=[rr.TimeSecondsColumn("step", acc_timestamp)],
    components=[rr.components.ScalarBatch(accs[:, 1])],
)
rr.send_columns(
    "/sensor/acc/acc_z",
    times=[rr.TimeSecondsColumn("step", acc_timestamp)],
    components=[rr.components.ScalarBatch(accs[:, 2])],
)

rr.send_columns(
    "/sensor/gyro/gyro_x",
    times=[rr.TimeSecondsColumn("step", acc_timestamp)],
    components=[rr.components.ScalarBatch(gyro[:, 0])],
)
rr.send_columns(
    "/sensor/gyro/gyro_y",
    times=[rr.TimeSecondsColumn("step", acc_timestamp)],
    components=[rr.components.ScalarBatch(gyro[:, 1])],
)
rr.send_columns(
    "/sensor/gyro/gyro_z",
    times=[rr.TimeSecondsColumn("step", acc_timestamp)],
    components=[rr.components.ScalarBatch(gyro[:, 2])],
)

# rr.send_columns(
#     "/gt/position/pos_x",
#     times=[rr.TimeSecondsColumn("step", pose_timestamp)],
#     components=[rr.components.ScalarBatch(positions[:, 0])],
# )
# rr.send_columns(
#     "/gt/position/pos_y",
#     times=[rr.TimeSecondsColumn("step", pose_timestamp)],
#     components=[rr.components.ScalarBatch(positions[:, 1])],
# )
# rr.send_columns(
#     "/gt/position/pos_z",
#     times=[rr.TimeSecondsColumn("step", pose_timestamp)],
#     components=[rr.components.ScalarBatch(positions[:, 2])],
# )

# rr.send_columns(
#     "/gt/rotation/rot_x",
#     times=[rr.TimeSecondsColumn("step", pose_timestamp)],
#     components=[rr.components.ScalarBatch(rotation[:, 0])],
# )
# rr.send_columns(
#     "/gt/rotation/rot_y",
#     times=[rr.TimeSecondsColumn("step", pose_timestamp)],
#     components=[rr.components.ScalarBatch(rotation[:, 1])],
# )
# rr.send_columns(
#     "/gt/rotation/rot_z",
#     times=[rr.TimeSecondsColumn("step", pose_timestamp)],
#     components=[rr.components.ScalarBatch(rotation[:, 2])],
# )
# rr.send_columns(
#     "/gt/rotation/rot_w",
#     times=[rr.TimeSecondsColumn("step", pose_timestamp)],
#     components=[rr.components.ScalarBatch(rotation[:, 3])],
# )
rr.log("/gt/frustum", rr.Pinhole(image_from_camera=M, resolution=[W, H]), static=True)
rr.send_columns(
    "/gt/frustum",
    times=[rr.TimeSecondsColumn("step", pose_timestamp)],
    components=[
        rr.components.Translation3DBatch(positions),
        rr.components.PoseRotationQuatBatch(rotation)
    ]
)


rr.send_columns(
    "/gt/position",
    times=[rr.TimeSecondsColumn("step", pose_timestamp)],
    components=[
        rr.Points3D.indicator(),
        rr.components.Position3DBatch(positions),
    ],
)


#plot coordinate arrow
rr.log("gt", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
rr.log(
    "gt/xyz",
    rr.Arrows3D(
        vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
    ),
)

#plot tdoa

# rr.log("/gt/tdoa_arrow",
#        rr.Arrows3D(vectors=tdoa, colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]]),
#        static=True)

#plot anchor positions
rr.log("/gt/anchor",
       rr.Boxes3D(centers=anchor_pos[["X", "Y", "Z"]].values,
                  half_sizes=np.tile([0.01, 0.5, 0.1], (8, 1)),
                  quaternions=[rr.Quaternion.identity(), rr.Quaternion(xyzw=anchor_rot[["Qx", "Qy", "Qz", "Qw"]].values)]),
                  static=True)

rr.log("/gt/position", rr.LineStrips3D(positions, radii=0.005), static=True)

xlim = np.array([np.nanmin(anchor_pos[["X"]].values.astype(float)),
                 np.nanmax(anchor_pos[["X"]].values.astype(float))])
ylim = np.array([np.nanmin(anchor_pos[["Y"]].values.astype(float)),
                 np.nanmax(anchor_pos[["Y"]].values.astype(float))])
add_grid("/gt", xlim, ylim)


for pos in positions:
    rr.log("/world/point1", rr.Points3D(pos, radii=0.01))
    time.sleep(0.001)