from enum import Enum

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

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


class UwbInsViewer:
    def __init__(self, anchor_poses: list[AnchorPose]) -> None:
        self.anchor_poses = np.array(anchor_poses)

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
            self._draw_uwb(uwb_data)

        elif data_type == DataType.POSE:
            pose = Pose(
                position = np.array([0.0, 0.0, 0.0]),
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