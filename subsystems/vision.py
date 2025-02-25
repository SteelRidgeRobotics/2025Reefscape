import math
import concurrent.futures
from enum import Enum, auto

from phoenix6 import utils
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

    We use MegaTag 1 before the match starts to set the heading and switch to MegaTag 2 during gameplay.
    """

    class SubsystemState(Enum):
        MEGA_TAG_1 = auto()
        """ Uses MegaTag 1 pose estimates to determine the robot position. """

        MEGA_TAG_2 = auto()
        """ Uses MegaTag 2 pose estimates to determine the robot position. """

        DISABLE_ESTIMATES = auto()
        """ Ignores all Limelight pose estimates. """

    def __init__(self, swerve: SwerveSubsystem, *cameras: str):
        super().__init__("Vision", self.SubsystemState.MEGA_TAG_1)

        self._swerve = swerve
        self._cameras = tuple(cameras)

        if not all(isinstance(cam, str) for cam in self._cameras):
            raise TypeError(f"All cameras must be strings! Given: {self._cameras}")

    def periodic(self):
        super().periodic()

        state = self._subsystem_state

        if abs(self._swerve.pigeon2.get_angular_velocity_z_world().value) > 720 or state == self.SubsystemState.DISABLE_ESTIMATES:
            return

        pigeon_values = self._get_pigeon_values()

        with concurrent.futures.ThreadPoolExecutor() as executor:
            futures = {
                executor.submit(self._process_camera, cam, state, pigeon_values): cam
                for cam in self._cameras
            }

            for future in concurrent.futures.as_completed(futures):
                estimate = future.result()
                if estimate and estimate.tag_count > 0 and state is not self.SubsystemState.DISABLE_ESTIMATES:
                    self._swerve.add_vision_measurement(
                        estimate.pose,
                        utils.fpga_to_current_time(estimate.timestamp_seconds),
                        self._get_dynamic_std_devs(estimate),
                    )

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

    def _process_camera(self, camera: str, state: SubsystemState, pigeon_values: dict) -> PoseEstimate | None:
        """ Retrieves pose estimate for a single camera. """
        pose = None

        if state == self.SubsystemState.MEGA_TAG_2:
            self._update_camera_orientation(camera, pigeon_values)
            pose = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(camera)
        elif state == self.SubsystemState.MEGA_TAG_1:
            pose = LimelightHelpers.get_botpose_estimate_wpiblue(camera)

        return pose if pose and pose.tag_count > 0 else None

    @staticmethod
    def _update_camera_orientation(camera: str, pigeon_values: dict):
        """ Updates the camera with the latest robot orientation from the IMU. """
        LimelightHelpers.set_robot_orientation(
            camera,
            pigeon_values["yaw"],
            pigeon_values["ang_vel_z"],
            0, 0,  # Pitch and pitch velocity set to 0
            0, 0   # Roll and roll velocity set to 0
        )

    def _get_pigeon_values(self) -> dict:
        """ Fetches and stores all Pigeon IMU values at once to reduce redundant calls. """
        pigeon = self._swerve.pigeon2
        return {
            "yaw": pigeon.get_yaw().value,
            "ang_vel_z": pigeon.get_angular_velocity_z_world().value,
        }

    @staticmethod
    def _get_dynamic_std_devs(estimate: PoseEstimate) -> tuple[float, float, float]:
        """ Computes dynamic standard deviations based on tag count and distance. """
        if estimate.tag_count == 0:
            return 0.7, 0.7, 0.7

        avg_dist = sum(f.dist_to_camera for f in estimate.raw_fiducials) / estimate.tag_count
        factor = 1 + (avg_dist ** 2 / 30)

        return 0.7 * factor, 0.7 * factor, math.inf if estimate.is_megatag_2 else (0.7 * factor)
