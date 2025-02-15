import math
from enum import Enum, auto

from ntcore import NetworkTableInstance
from phoenix6 import StatusCode, utils
from phoenix6.swerve import Translation2d, SwerveModule, SwerveControlParameters, Pose2d
from phoenix6.swerve.requests import SwerveRequest, ForwardPerspectiveValue, FieldCentric, FieldCentricFacingAngle
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController
from phoenix6.units import *
from wpilib import DriverStation
from wpimath.geometry import Rotation2d, Pose3d, Rotation3d
from wpimath.units import degreesToRadians

from constants import Constants
from lib.limelight import LimelightHelpers, RawFiducial


class FieldCentricReefAlign(SwerveRequest):

    class BranchSide(Enum):
        NO_TARGET = auto()
        LEFT = auto()
        RIGHT = auto()

    _blue_branch_targets = [
        Pose2d(3.091, 4.181, degreesToRadians(0)),  # A
        Pose2d(3.091, 3.863, degreesToRadians(0)),  # B
        Pose2d(3.656, 2.916, degreesToRadians(60)),  # C
        Pose2d(3.956, 2.748, degreesToRadians(60)),  # D
        Pose2d(5.023, 2.772, degreesToRadians(120)),  # E
        Pose2d(5.323, 2.928, degreesToRadians(120)),  # F
        Pose2d(5.850, 3.851, degreesToRadians(180)),  # G
        Pose2d(5.862, 4.187, degreesToRadians(180)),  # H
        Pose2d(5.347, 5.134, degreesToRadians(-120)),  # I
        Pose2d(5.047, 5.290, degreesToRadians(-120)),  # J
        Pose2d(3.932, 5.302, degreesToRadians(-60)),  # K
        Pose2d(3.668, 5.110, degreesToRadians(-60)),  # L
    ]

    _red_branch_targets = [
        Pose2d(
            Constants.FIELD_LAYOUT.getFieldLength() - pose.X(),
            Constants.FIELD_LAYOUT.getFieldWidth() - pose.Y(),
            pose.rotation() + Rotation2d.fromDegrees(180)
        ) for pose in _blue_branch_targets
    ]

    def __init__(self) -> None:
        self.velocity_x: meters_per_second = 0
        """
        The velocity in the X direction, in m/s. X is defined as forward according to
        WPILib convention, so this determines how fast to travel forward.
        """
        self.velocity_y: meters_per_second = 0
        """
        The velocity in the Y direction, in m/s. Y is defined as to the left according
        to WPILib convention, so this determines how fast to travel to the left.
        """
        self.rotational_rate: radians_per_second = 0
        """
        The angular rate to rotate at, in radians per second. Angular rate is defined as
        counterclockwise positive, so this determines how fast to turn counterclockwise.
        """
        self.deadband: meters_per_second = 0
        """
        The allowable deadband of the request, in m/s.
        """
        self.rotational_deadband: radians_per_second = 0
        """
        The rotational deadband of the request, in radians per second.
        """
        self.center_of_rotation: Translation2d = Translation2d()
        """
        The center of rotation the robot should rotate around. This is (0,0) by default,
        which will rotate around the center of the robot.
        """
        self.drive_request_type: SwerveModule.DriveRequestType = SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        """
        The type of control request to use for the drive motor.
        """
        self.steer_request_type: SwerveModule.SteerRequestType = SwerveModule.SteerRequestType.POSITION
        """
        The type of control request to use for the drive motor.
        """
        self.desaturate_wheel_speeds: bool = True
        """
        Whether to desaturate wheel speeds before applying. For more information, see
        the documentation of SwerveDriveKinematics.desaturateWheelSpeeds.
        """
        self.forward_perspective: ForwardPerspectiveValue = ForwardPerspectiveValue.OPERATOR_PERSPECTIVE
        """
        The perspective to use when determining which direction is forward.
        """
        self.direction: FieldCentricReefAlign.BranchSide = FieldCentricReefAlign.BranchSide.RIGHT
        """
        The branch to target.
        """

        self._tag_target_pose = Pose3d()
        self._tag_target_id = -1

        self._field_centric = FieldCentric()
        self._field_centric_facing_angle = FieldCentricFacingAngle()

        self.heading_controller = self._field_centric_facing_angle.heading_controller
        self.translation_x_controller = PhoenixPIDController(0.0, 0.0, 0.0)
        self.translation_y_controller = PhoenixPIDController(0.0, 0.0, 0.0)

        self._table = NetworkTableInstance.getDefault().getTable("Telemetry").getSubTable("Auto Align")
        self._tag_target_pub = self._table.getStructTopic("Tag Target", Pose3d).publish()
        self._reef_target_pub = self._table.getStructTopic("Reef Target", Pose2d).publish()

    def with_velocity_x(self, new_velocity_x: meters_per_second) -> 'FieldCentricReefAlign':
        """
        Modifies the velocity_x parameter and returns itself.

        The velocity in the X direction, in m/s. X is defined as forward according to
        WPILib convention, so this determines how fast to travel forward.

        :param new_velocity_x: Parameter to modify
        :type new_velocity_x: meters_per_second
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.velocity_x = new_velocity_x
        return self

    def with_velocity_y(self, new_velocity_y: meters_per_second) -> 'FieldCentricReefAlign':
        """
        Modifies the velocity_y parameter and returns itself.

        The velocity in the Y direction, in m/s. Y is defined as to the left according
        to WPILib convention, so this determines how fast to travel to the left.

        :param new_velocity_y: Parameter to modify
        :type new_velocity_y: meters_per_second
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.velocity_y = new_velocity_y
        return self

    def with_rotational_rate(self, new_rotational_rate: radians_per_second) -> 'FieldCentricReefAlign':
        """
        Modifies the rotational_rate parameter and returns itself.

        The angular rate to rotate at, in radians per second. Angular rate is defined as
        counterclockwise positive, so this determines how fast to turn counterclockwise.

        :param new_rotational_rate: Parameter to modify
        :type new_rotational_rate: radians_per_second
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.rotational_rate = new_rotational_rate
        return self

    def with_deadband(self, new_deadband: meters_per_second) -> 'FieldCentricReefAlign':
        """
        Modifies the deadband parameter and returns itself.

        The allowable deadband of the request, in m/s.

        :param new_deadband: Parameter to modify
        :type new_deadband: meters_per_second
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.deadband = new_deadband
        return self

    def with_rotational_deadband(self, new_rotational_deadband: radians_per_second) -> 'FieldCentricReefAlign':
        """
        Modifies the rotational_deadband parameter and returns itself.

        The rotational deadband of the request, in radians per second.

        :param new_rotational_deadband: Parameter to modify
        :type new_rotational_deadband: radians_per_second
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.rotational_deadband = new_rotational_deadband
        return self

    def with_center_of_rotation(self, new_center_of_rotation: Translation2d) -> 'FieldCentricReefAlign':
        """
        Modifies the center_of_rotation parameter and returns itself.

        The center of rotation the robot should rotate around. This is (0,0) by default,
        which will rotate around the center of the robot.

        :param new_center_of_rotation: Parameter to modify
        :type new_center_of_rotation: Translation2d
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.center_of_rotation = new_center_of_rotation
        return self

    def with_drive_request_type(self, new_drive_request_type: SwerveModule.DriveRequestType) -> 'FieldCentricReefAlign':
        """
        Modifies the drive_request_type parameter and returns itself.

        The type of control request to use for the drive motor.

        :param new_drive_request_type: Parameter to modify
        :type new_drive_request_type: SwerveModule.DriveRequestType
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.drive_request_type = new_drive_request_type
        return self

    def with_steer_request_type(self, new_steer_request_type: SwerveModule.SteerRequestType) -> 'FieldCentricReefAlign':
        """
        Modifies the steer_request_type parameter and returns itself.

        The type of control request to use for the drive motor.

        :param new_steer_request_type: Parameter to modify
        :type new_steer_request_type: SwerveModule.SteerRequestType
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.steer_request_type = new_steer_request_type
        return self

    def with_desaturate_wheel_speeds(self, new_desaturate_wheel_speeds: bool) -> 'FieldCentricReefAlign':
        """
        Modifies the desaturate_wheel_speeds parameter and returns itself.

        Whether to desaturate wheel speeds before applying. For more information, see
        the documentation of SwerveDriveKinematics.desaturateWheelSpeeds.

        :param new_desaturate_wheel_speeds: Parameter to modify
        :type new_desaturate_wheel_speeds: bool
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.desaturate_wheel_speeds = new_desaturate_wheel_speeds
        return self

    def with_forward_perspective(self, new_forward_perspective: ForwardPerspectiveValue) -> 'FieldCentricReefAlign':
        """
        Modifies the forward_perspective parameter and returns itself.

        The perspective to use when determining which direction is forward.

        :param new_forward_perspective: Parameter to modify
        :type new_forward_perspective: ForwardPerspectiveValue
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.forward_perspective = new_forward_perspective
        return self

    def with_direction(self, new_direction: BranchSide) -> 'FieldCentricReefAlign':
        """
        Modified the direction parameter and returns itself.

        The branch to target.

        :param new_direction: Parameter to modify
        :type new_direction: Direction
        :returns: this object
        :rtype: FieldCentricReefAlign
        """

        self.direction = new_direction
        return self

    def apply(self, parameters: SwerveControlParameters, modules_to_apply: list[SwerveModule]) -> StatusCode:
        current_pose = parameters.current_pose

        self._update_tag_target(parameters)
        if self._tag_target_id == -1 or self._tag_target_pose == Pose3d() or self.direction is self.BranchSide.NO_TARGET:
            # Don't align if we don't have a valid target or haven't specified a direction.
            return self._give_up(parameters, modules_to_apply)

        # Get the current desired target
        reef_targets = self._get_targets_from_id(self._tag_target_id)
        if self.direction is self.BranchSide.LEFT:
            target = reef_targets[0]
        else:
            target = reef_targets[1]
        self._reef_target_pub.set(target)

        current_translation_x = current_pose.translation().X()
        target_translation_x = target.translation().X()
        translation_x_output = -self.translation_x_controller.calculate(
            current_translation_x,
            target_translation_x,
            parameters.timestamp
        )

        current_translation_y = current_pose.translation().Y()
        target_translation_y = target.translation().Y()
        translation_y_output = -self.translation_y_controller.calculate(
            current_translation_y,
            target_translation_y,
            parameters.timestamp
        )
        if abs(translation_x_output) < abs(self.velocity_x):
            velocity_x_output = self.velocity_x
        else:
            velocity_x_output = translation_x_output
        if abs(translation_y_output) < abs(self.velocity_y):
            velocity_y_output = self.velocity_y
        else:
            velocity_y_output = translation_y_output

        return (
            self._field_centric_facing_angle
            .with_velocity_x(velocity_x_output)
            .with_velocity_y(velocity_y_output)
            .with_target_direction(target.rotation() + Rotation2d.fromDegrees(180))
            .with_deadband(self.deadband)
            .with_rotational_deadband(self.rotational_deadband)
            .with_center_of_rotation(self.center_of_rotation)
            .with_drive_request_type(self.drive_request_type)
            .with_steer_request_type(self.steer_request_type)
            .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
            .with_forward_perspective(self.forward_perspective)
            .apply(parameters, modules_to_apply)
        )

    def _update_tag_target(self, parameters: SwerveControlParameters) -> None:
        valid_ids = self._get_valid_ids()

        if not utils.is_simulation():
            # When we're not in simulation, grab current raw fiducials for tag checks
            valid_fiducials: list[RawFiducial] = []
            for fiducial in LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(Constants.VisionConstants.CENTER_FRONT_NAME).raw_fiducials:
                if fiducial.id in valid_ids:
                    valid_fiducials.append(fiducial)
        else:
            # When in simulation, check to see which valid tags we're able to see. NOTE: This sucks :3
            robot_3d_pose = Pose3d(
                parameters.current_pose.X(),
                parameters.current_pose.Y(),
                0.0, # Assume we're not flying (unlikely),
                Rotation3d(parameters.current_pose.rotation())
            )
            half_fov = degreesToRadians(82) / 2 # 82 degrees = LL4 horizontal FOV
            valid_fiducials: list[RawFiducial] = []
            for tag in Constants.FIELD_LAYOUT.getTags():
                tag_to_robot = tag.pose.relativeTo(robot_3d_pose)
                if -half_fov <= math.atan2(tag_to_robot.y, tag_to_robot.x) <= half_fov and tag.ID in valid_ids: # Check if valid tag is in our camera FOV
                    valid_fiducials.append(
                        RawFiducial(id=tag.ID, dist_to_robot=tag_to_robot.translation().norm())
                    )

        old_pose = self._tag_target_pose
        if len(valid_fiducials) == 0:
            # No valid tags found, don't change current value.
            return
        elif len(valid_fiducials) > 1:
            # If we can see multiple tags, set the target to the closest one.
            closest_fiducial = RawFiducial(dist_to_robot=math.inf) # Create fiducial at edge of universe
            for fiducial in valid_fiducials:
                if fiducial.dist_to_robot < closest_fiducial.dist_to_robot:
                    closest_fiducial = fiducial
            self._tag_target_pose = Constants.FIELD_LAYOUT.getTagPose(closest_fiducial.id)
        else:
            # If we only see one tag, set that as the new target
            self._tag_target_pose = Constants.FIELD_LAYOUT.getTagPose(valid_fiducials[0].id)

        if old_pose != self._tag_target_pose:
            # If the target changed, update self._tag_target_id
            for tag in Constants.FIELD_LAYOUT.getTags():
                if tag.pose == self._tag_target_pose:
                    self._tag_target_id = tag.ID
            self._tag_target_pub.set(self._tag_target_pose)

    def _give_up(self, parameters: SwerveControlParameters, modules_to_apply: list[SwerveModule]) -> StatusCode:
        """Gives up on auto aligning."""
        return (
            self._field_centric
            .with_velocity_x(self.velocity_x)
            .with_velocity_y(self.velocity_y)
            .with_rotational_rate(self.rotational_rate)
            .with_deadband(self.deadband)
            .with_rotational_deadband(self.rotational_deadband)
            .with_center_of_rotation(self.center_of_rotation)
            .with_drive_request_type(self.drive_request_type)
            .with_steer_request_type(self.steer_request_type)
            .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
            .with_forward_perspective(self.forward_perspective)
            .apply(parameters, modules_to_apply)
        )

    @staticmethod
    def _get_valid_ids() -> list[int]:
        """Get the correct AprilTag ids depending on our alliance."""
        if (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed: # red alliance check
            return [6, 7, 8, 9, 10, 11]
        return [17, 18, 19, 20, 21, 22]

    @classmethod
    def _get_targets_from_id(cls, tag_id: int) -> list[Pose2d]:
        """Match Case Monolith."""
        match tag_id:
            # Red Reef
            case 6:
                return [cls._red_branch_targets[10], cls._red_branch_targets[11]]
            case 7:
                return [cls._red_branch_targets[0], cls._red_branch_targets[1]]
            case 8:
                return [cls._red_branch_targets[2], cls._red_branch_targets[3]]
            case 9:
                return [cls._red_branch_targets[4], cls._red_branch_targets[5]]
            case 10:
                return [cls._red_branch_targets[6], cls._red_branch_targets[7]]
            case 11:
                return [cls._red_branch_targets[8], cls._red_branch_targets[9]]

            # Blue Reef
            case 17:
                return [cls._red_branch_targets[10], cls._red_branch_targets[11]]
            case 18:
                return [cls._red_branch_targets[0], cls._red_branch_targets[1]]
            case 19:
                return [cls._red_branch_targets[2], cls._red_branch_targets[3]]
            case 20:
                return [cls._red_branch_targets[4], cls._red_branch_targets[5]]
            case 21:
                return [cls._red_branch_targets[6], cls._red_branch_targets[7]]
            case 22:
                return [cls._red_branch_targets[8], cls._red_branch_targets[9]]

            case _:
                return [Pose2d(), Pose2d()]

