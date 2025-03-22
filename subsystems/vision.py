import concurrent.futures
import math
from enum import Enum

from phoenix6 import utils
from wpilib import DataLogManager
from wpimath.geometry import Pose3d, Pose2d

from constants import Constants
from lib.limelight import PoseEstimate, LimelightHelpers
from subsystems import StateSubsystem
from subsystems.swerve import SwerveSubsystem


class VisionSubsystem(StateSubsystem):
    """
    Handles all camera calculations on the robot.
    This is primarily used for combining MegaTag pose estimates and ensuring no conflicts between Limelights.

    Our vision system consists of:
    - 1 Limelight 4 (back of the funnel, horizontal)
    - 1 Limelight 4 (under the pivot, 20-degree inclination)
    - 2 Limelight 3As (front swerve covers, 15-degree outward incline)

    We use the starting position in auto to determine our robot heading to calibrate our cameras.
    """

    class SubsystemState(Enum):
        ALL_ESTIMATES = list(range(1, 23))
        """ Enables MegaTag 2 pose estimates to the robot from all tags."""

        REEF_ESTIMATES = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22]
        """Only allows pose estimates from tags from the reef."""

        NO_ESTIMATES = []
        """ Ignores all Limelight pose estimates. """

    def __init__(self, swerve: SwerveSubsystem, *cameras: str):
        super().__init__("Vision", self.SubsystemState.ALL_ESTIMATES)

        self._swerve = swerve
        self._cameras = tuple(cameras)

        if not all(isinstance(cam, str) for cam in self._cameras):
            raise TypeError(f"All cameras must be strings! Given: {self._cameras}")

        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=len(self._cameras))

        self._visible_tags_pub = self.get_network_table().getStructArrayTopic("Visible Tags", Pose3d).publish()
        self._final_measurement_pub = self.get_network_table().getStructTopic("Best Measurement", Pose2d).publish()
        self._vision_measurements = self.get_network_table().getStructArrayTopic("All Measurements", Pose2d).publish()

    def periodic(self):
        super().periodic()

        state = self._subsystem_state
        if state is self.SubsystemState.NO_ESTIMATES or abs(self._swerve.pigeon2.get_angular_velocity_z_world().value) > 720:
            return

        futures = [
            self._executor.submit(self._process_camera, cam)
            for cam in self._cameras
        ]

        best_estimate = None
        visible_tags = []
        all_measurements = []
        for future in concurrent.futures.as_completed(futures):
            try:
                camera, estimate = future.result()
                if not estimate or estimate.tag_count == 0:
                    continue

                visible_tags.extend(
                    Constants.FIELD_LAYOUT.getTagPose(f.id) for f in estimate.raw_fiducials
                )
                all_measurements.append(estimate.pose)

                if best_estimate is None or self._is_better_estimate(estimate, best_estimate):
                    best_estimate = estimate
            except Exception as e:
                DataLogManager.log(f"Vision processing failed for a camera: {e}")

        if best_estimate:
            self._swerve.add_vision_measurement(
                best_estimate.pose,
                utils.fpga_to_current_time(best_estimate.timestamp_seconds),
                self._get_dynamic_std_devs(best_estimate),
            )
            self._final_measurement_pub.set(best_estimate.pose)
        else:
            self._final_measurement_pub.set(Pose2d())

        self._vision_measurements.set(all_measurements)
        self._visible_tags_pub.set(list(visible_tags))

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        for camera in self._cameras:
            LimelightHelpers.set_fiducial_id_filters_override(camera, desired_state.value)

    def _process_camera(self, camera: str) -> tuple[str, PoseEstimate | None]:
        state = self._swerve.get_state_copy()
        LimelightHelpers.set_robot_orientation(
            camera,
            state.pose.rotation().degrees(),
            state.speeds.omega_dps, 0, 0, 0, 0
        )
        pose = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(camera)

        if pose is None or pose.tag_count == 0:
            return camera, None

        return camera, pose

    @staticmethod
    def _is_better_estimate(new_estimate: PoseEstimate, current_best: PoseEstimate) -> bool:
        if new_estimate.tag_count > current_best.tag_count:
            return True
        if new_estimate.tag_count == current_best.tag_count:
            new_ambiguity = sum(f.ambiguity for f in new_estimate.raw_fiducials) / new_estimate.tag_count
            current_ambiguity = sum(f.ambiguity for f in current_best.raw_fiducials) / current_best.tag_count
            return new_ambiguity < current_ambiguity
        return False

    @staticmethod
    def _get_dynamic_std_devs(estimate: PoseEstimate) -> tuple[float, float, float]:
        """Computes dynamic standard deviations based on tag count and distance."""
        if estimate.tag_count == 0:
            return 0.5, 0.5, 0.5

        avg_dist = sum(f.dist_to_camera for f in estimate.raw_fiducials) / estimate.tag_count
        factor = 1 + (avg_dist ** 2 / 30)

        return 0.5 * factor, 0.5 * factor, math.inf if estimate.is_megatag_2 else (0.5 * factor)
