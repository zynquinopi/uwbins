from enum import Enum

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from scipy.optimize import least_squares

from .utils import *


F = 500
W = 640
H = 480
M = np.array([
    [F, 0, W/2],
    [0, F, H/2],
    [0, 0, 1]
])

class AxisColor(Enum):
    X = (255, 0, 0)
    Y = (0, 255, 0)
    Z = (0, 0, 255)

UwbAnchorColor = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0)
]

UwbAnchorBoxSize = np.array([0.05, 0.001, 0.1])


def trilateration(anchor_positions, distances, initial_guess=None):
    def residuals(pos):
        return np.linalg.norm(anchor_positions - pos, axis=1) - distances

    result = least_squares(residuals, initial_guess)
    return result.x


class UwbInsViewer:
    def __init__(self, anchor_poses: list[AnchorPose]) -> None:
        self.anchor_poses = np.array(anchor_poses)
        self.distances = np.array([0.0, 0.0, 0.0, 0.0])
        self.latest_position = np.array([0.845, 0.0, 1.015])

        rr.init("UWBINS rerun viewer")
        rr.serve_web()
        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Vertical(
                    rrb.TimeSeriesView(origin="/sensor/acc"),
                    rrb.TimeSeriesView(origin="/sensor/gyro"),
                    rrb.TimeSeriesView(origin="/sensor/uwb/distance"),
                    # rrb.TimeSeriesView(origin="/sensor/uwb/azimuth"),
                    # rrb.TimeSeriesView(origin="/sensor/uwb/elevation"),
                ),
                rrb.Vertical(
                    rrb.Spatial3DView(origin = "/3d", background=[30, 30, 30]),
                ),
                column_shares=[4, 6],
            ),
            rrb.BlueprintPanel(state="expanded"),
            rrb.SelectionPanel(state="collapsed"),
            rrb.TimePanel(state="collapsed"),
        )
        rr.send_blueprint(blueprint, make_active=True, make_default=True)
        self.setup_static_components()

    def setup_static_components(self) -> None:
        rr.log("/3d", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        rr.log("3d/xyz",
               rr.Arrows3D(
                   vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                   colors=[AxisColor.X.value, AxisColor.Y.value, AxisColor.Z.value],
               ),
               static=True)
        rr.log("/3d/frustum", rr.Pinhole(image_from_camera=M, resolution=[W, H]), static=True)

        for axis, color in zip("xyz", AxisColor):
            rr.log(f"/sensor/acc/{axis}", rr.SeriesLine(color=color.value), static=True)
            rr.log(f"/sensor/gyro/{axis}", rr.SeriesLine(color=color.value), static=True)

        for anchor_id, color in zip(range(len(self.anchor_poses)), UwbAnchorColor):
            rr.log(
                f"/3d/uwb/box/{anchor_id}",
                rr.Boxes3D(
                    centers=self.anchor_poses[anchor_id].position,
                    half_sizes=np.tile(UwbAnchorBoxSize, (4, 1)),
                    quaternions=[rr.Quaternion.identity(), rr.Quaternion(xyzw=self.anchor_poses[anchor_id].rotation)],
                    colors=color,
                ),
                static=True
            )
            rr.log(f"/sensor/uwb/distance/{anchor_id}", rr.SeriesLine(color=color), static=True)
            # rr.log(f"/sensor/uwb/azimuth/{anchor_id}", rr.SeriesLine(color=color), static=True)
            # rr.log(f"/sensor/uwb/elevation/{anchor_id}", rr.SeriesLine(color=color), static=True)

    def draw_data(self, data_type: DataType, data: dict) -> None:
        if data_type == DataType.IMU:
            imu_data = Imu(float(data["timestamp"]),
                           float(data["temperature"]),
                           np.array([float(data["ax"]), float(data["ay"]), float(data["az"])]),
                           np.array([float(data["gx"]), float(data["gy"]), float(data["gz"])])
            )
            self._draw_imu(imu_data)

        elif data_type == DataType.UWB:
            uwb_data = Uwb(float(data["timestamp"]),
                           int(data["anchor_id"]),
                           int(data["nlos"]),
                           float(data["distance"]),
                           float(data["azimuth"]),
                           float(data["elevation"])
            )
            self.distances[data["anchor_id"]] = float(data["distance"])
            if data["anchor_id"] == 3:
                poses = np.array([self.anchor_poses[i].position for i in range(len(self.anchor_poses))])
                position = trilateration(poses, self.distances, self.latest_position)
                self.latest_position = position
            self._draw_uwb(uwb_data)

        elif data_type == DataType.POSE:
            pose = Pose(
                float(data["timestamp"]),
                # position = np.array([0.0, 0.0, 0.0]),
                position = np.array(self.latest_position),
                rotation=np.array([float(data["qx"]), float(data["qy"]), float(data["qz"]), float(data["qw"])]),
            )
            self._draw_pose(pose)

    def _draw_imu(self, imu: Imu) -> None:
        for axis, acc, gyro in zip("xyz", imu.acc, imu.gyro):
            rr.log(f"/sensor/acc/{axis}", rr.Scalar(acc))
            rr.log(f"/sensor/gyro/{axis}", rr.Scalar(gyro))

    def _draw_uwb(self, uwb: Uwb) -> None:
        rr.log(f"/sensor/uwb/distance/{uwb.anchor_id}", rr.Scalar(uwb.distance))
        # rr.log(f"/sensor/uwb/azimuth/{anchor_id}", rr.Scalar(uwb.azimuth))
        # rr.log(f"/sensor/uwb/elevation/{anchor_id}", rr.Scalar(uwb.elevation))
        rr.log(
            f"/3d/uwb/{uwb.anchor_id}",
            rr.Ellipsoids3D(
                centers=self.anchor_poses[uwb.anchor_id].position,
                half_sizes=[np.array([uwb.distance, uwb.distance, uwb.distance])],
                colors=UwbAnchorColor[uwb.anchor_id],
                fill_mode=2,
                line_radii=0.002,
            )
        )

    def _draw_pose(self, pose: Pose) -> None:
        rr.log(
            "/3d/frustum",
            rr.Transform3D(translation=pose.position, rotation=rr.Quaternion(xyzw=pose.rotation)),
        )


    def draw_data_batch(self, data_type: DataType, data: list[Imu | Uwb]) -> None:
        if data_type == DataType.IMU:
            self._draw_imu_batch(np.array(data))

        elif data_type == DataType.UWB:
            self._draw_uwb_batch(np.array(data))
        #     # if uwbs[0].anchor_id == 3:
        #     #     poses = np.array([self.anchor_poses[i].position for i in range(len(self.anchor_poses))])
        #     #     position = trilateration(poses, self.distances, self.latest_position)
        #     #     self.latest_position = position
        #     self._draw_uwb_batch(np.array(uwbs))

        elif data_type == DataType.POSE:
            self.draw_pose(np.array(data))


    def _draw_imu_batch(self, imus: np.ndarray) -> None:
        timestamps = np.array([imu.timestamp for imu in imus])
        accs = np.stack([imu.acc for imu in imus], axis=0)
        gyros = np.stack([imu.gyro for imu in imus], axis=0)
        for i, axis in enumerate("xyz"):
            rr.send_columns(
                f"/sensor/acc/{axis}",
                indexes=[rr.TimeSecondsColumn("time", timestamps)],
                columns=rr.Scalar.columns(scalar=accs[:, i]),
            )
            rr.send_columns(
                f"/sensor/gyro/{axis}",
                indexes=[rr.TimeSecondsColumn("time", timestamps)],
                columns=rr.Scalar.columns(scalar=gyros[:, i]),
            )


    def _draw_uwb_batch(self, uwbs: np.ndarray) -> None:
        anchor_ids = set(uwb.anchor_id for uwb in uwbs)
        for id in anchor_ids:
            timestamps = np.array([uwb.timestamp for uwb in uwbs if uwb.anchor_id == id])
            distances = np.array([uwb.distance for uwb in uwbs if uwb.anchor_id == id])
            # azimuths = np.array([uwb.azimuth for uwb in uwbs if uwb.anchor_id == id])
            # elevations = np.array([uwb.elevation for uwb in uwbs if uwb.anchor_id == id])
            rr.send_columns(
                f"/sensor/uwb/distance/{id}",
                indexes=[rr.TimeSecondsColumn("time", timestamps)],
                columns=rr.Scalar.columns(scalar=distances),
            )
            # rr.send_columns(
            #     f"/sensor/uwb/azimuth/{id}",
            #     indexes=[rr.TimeSecondsColumn("time", timestamps)],
            #     columns=rr.Scalar.columns(scalar=azimuths),
            # )
            # rr.send_columns(
            #     f"/sensor/uwb/elevation/{id}",
            #     indexes=[rr.TimeSecondsColumn("time", timestamps)],
            #     columns=rr.Scalar.columns(scalar=elevations),
            # )
            rr.send_columns(
                f"/3d/uwb/{id}",
                indexes=[rr.TimeSecondsColumn("time", timestamps)],
                columns=rr.Ellipsoids3D.columns(
                    centers=[self.anchor_poses[id].position] * len(timestamps),
                    half_sizes=np.array([distances, distances, distances]).T,
                    colors=[UwbAnchorColor[id]] * len(timestamps),
                    fill_mode=[rr.components.FillMode.DenseWireframe] * len(timestamps),
                    line_radii=[0.002] * len(timestamps),
                ),
            )


    def draw_pose(self, poses: np.ndarray) -> None:
        timestamps = np.array([pose.timestamp for pose in poses])
        positions = np.array([pose.position for pose in poses])
        rotations = np.array([pose.rotation for pose in poses])
        rr.send_columns(
            "/3d/frustum",
            indexes=[rr.TimeSecondsColumn("time", timestamps)],
            columns=rr.Transform3D.columns(
                translation=positions,
                quaternion=[rr.Quaternion(xyzw=rot) for rot in rotations],
            ),
        )