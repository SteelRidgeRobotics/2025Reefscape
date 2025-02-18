import math
from enum import auto, Enum
from typing import TYPE_CHECKING

from lib.limelight import PoseEstimate
from subsystems import StateSubsystem
from subsystems.vision.util import Limelight

if TYPE_CHECKING:
    from robot_state import RobotState


class VisionSubsystem(StateSubsystem):
    """
    Handles all camera calculations on the robot.
    This is primarily used for combining MegaTag pose estimates and ensuring no conflicts between Limelights.

    Our vision system is composed of the following:
    - 2 Limelight 4s mounted on the back of the funnel and under the pivot
    - 2 Limelight 3As mounted on the back swerve covers, facing outward from the center of the robot at a 30 degree incline
    - 1 Limelight 3 (with a Google Coral) mounted upside down on the front left swerve cover, facing outward from the center of the robot at a 15 degree incline
    - 1 Limelight 2 mounted upside down on the front right swerve cover, facing outward from the center of the robot at a 15 degree incline
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

    def __init__(self, robot_state: 'RobotState', *args):
        super().__init__("Vision")

        self._robot_state = robot_state

        # noinspection PyTypeChecker
        self._cameras: tuple[Limelight] = args
        for camera in self._cameras:
            if not isinstance(camera, Limelight):
                raise TypeError(f"Camera must be Limelight instance!\nGiven cameras: {args}")

    def periodic(self):
        super().periodic()

        if not self._robot_state.should_use_vision_measurements():
            return

        match self._subsystem_state:
            case self.SubsystemState.MEGA_TAG_2:
                self.add_all_pose_estimates(True)
            case self.SubsystemState.MEGA_TAG_1:
                self.add_all_pose_estimates(False)
            case self.SubsystemState.HELEN_KELLER:
                pass

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if self.is_frozen():
            return
        self._subsystem_state = desired_state

    def add_all_pose_estimates(self, megatag_2: bool) -> None:
        valid_pose_estimates: list[tuple[PoseEstimate, Limelight]] = []
        for camera in self._cameras:
            pose_estimate = camera.get_global_pose_estimate(megatag_2)
            if pose_estimate.tag_count > 0:
                valid_pose_estimates.append((pose_estimate, camera))

        for estimate in valid_pose_estimates:
            self._robot_state.add_vision_measurements(
                estimate[1],
                estimate[0],
                self.get_dynamic_std_devs(estimate[1], estimate[0])
            )

    @staticmethod
    def get_dynamic_std_devs(limelight: Limelight, estimate: PoseEstimate) -> tuple[float, float, float]:
        default = limelight.default_std_devs()
        if estimate.tag_count == 0:
            return default
        else:
            avg_dist = 0
            for fiducial in estimate.raw_fiducials:
                avg_dist += fiducial.dist_to_camera
            avg_dist /= estimate.tag_count

            return (
                default[0] * (1 + (avg_dist ** 2 / 30)),
                default[1] * (1 + (avg_dist ** 2 / 30)),
                math.inf if estimate.is_megatag_2 else (default[2] * (1 + (avg_dist ** 2 / 30)))
            )
