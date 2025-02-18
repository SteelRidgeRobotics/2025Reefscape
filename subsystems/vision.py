import math
from enum import auto, Enum

from phoenix6 import utils

from lib.limelight import PoseEstimate, LimelightHelpers
from subsystems import StateSubsystem
from subsystems.swerve import SwerveSubsystem


class VisionSubsystem(StateSubsystem):
    """
    Handles all camera calculations on the robot.
    This is primarily used for combining MegaTag pose estimates and ensuring no conflicts between Limelights.

    Our vision system is composed of the following:
    - 2 Limelight 4s mounted on the back of the funnel and under the pivot
    - 2 Limelight 3As mounted on the back swerve covers, facing outward from the center of the robot at a 30 degree incline
    - 1 Limelight 3 (with a Google Coral) mounted upside down on the front left swerve cover, facing outward from the center of the robot at a 15 degree incline
    - 1 Limelight 2 mounted upside down on the front right swerve cover, facing outward from the center of the robot at a 15 degree incline

    We use MegaTag 1 before the match starts to ensure our robot's heading is correct. We switch to MegaTag 2 for the remainder of the match.
    """

    class SubsystemState(Enum):
        MEGA_TAG_1 = auto()
        """
        Uses MegaTag 1 pose estimates to determine the robot position.
        """
        MEGA_TAG_2 = auto()
        """
        Uses MegaTag 2 pose estimates to determine the robot position.
        """
        HELEN_KELLER = auto()
        """
        Ignores all Limelight pose estimates.
        """

    def __init__(self, swerve: SwerveSubsystem, *args):
        super().__init__("Vision", self.SubsystemState.MEGA_TAG_1)

        self._swerve = swerve

        # noinspection PyTypeChecker
        self._cameras: tuple[str] = args
        for camera in self._cameras:
            if not isinstance(camera, str):
                raise TypeError(f"Camera must be a string!\nGiven cameras: {args}")

    def periodic(self):
        super().periodic()

        if not abs(self._swerve.pigeon2.get_angular_velocity_z_world().value) <= 720:
            return

        valid_pose_estimates: list[PoseEstimate] = []
        match self._subsystem_state:
            case self.SubsystemState.MEGA_TAG_2:
                for camera in self._cameras:
                    LimelightHelpers.set_robot_orientation(
                        camera,
                        self._swerve.pigeon2.get_yaw().value,
                        self._swerve.pigeon2.get_angular_velocity_z_world().value,
                        self._swerve.pigeon2.get_pitch().value,
                        self._swerve.pigeon2.get_angular_velocity_y_world().value,
                        self._swerve.pigeon2.get_roll().value,
                        self._swerve.pigeon2.get_angular_velocity_x_world().value
                    )
                    estimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(camera)
                    if estimate.is_megatag_2 and estimate.tag_count > 0:
                        valid_pose_estimates.append(estimate)

            case self.SubsystemState.MEGA_TAG_1:
                for camera in self._cameras:
                    estimate = LimelightHelpers.get_botpose_estimate_wpiblue(camera)
                    if not estimate.is_megatag_2 and estimate.tag_count > 0:
                        valid_pose_estimates.append(estimate)

            case self.SubsystemState.HELEN_KELLER:
                return

        if len(valid_pose_estimates) == 0:
            return

        for estimate in valid_pose_estimates:
            self._swerve.add_vision_measurement(estimate.pose, utils.fpga_to_current_time(estimate.timestamp_seconds), self.get_dynamic_std_devs(estimate))

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if self.is_frozen():
            return
        self._subsystem_state = desired_state

    @staticmethod
    def get_dynamic_std_devs(estimate: PoseEstimate) -> tuple[float, float, float]:
        default = (0.7, 0.7, 0.7)
        if estimate.tag_count == 0:
            return default

        avg_dist = 0
        for fiducial in estimate.raw_fiducials:
            avg_dist += fiducial.dist_to_camera
        avg_dist /= estimate.tag_count

        return (
            default[0] * (1 + (avg_dist ** 2 / 30)),
            default[1] * (1 + (avg_dist ** 2 / 30)),
            math.inf if estimate.is_megatag_2 else (default[2] * (1 + (avg_dist ** 2 / 30)))
        )
