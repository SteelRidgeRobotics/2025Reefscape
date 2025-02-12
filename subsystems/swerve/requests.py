from enum import Enum, auto

from phoenix6 import StatusCode
from phoenix6.swerve import Translation2d, SwerveModule, SwerveControlParameters
from phoenix6.swerve.requests import SwerveRequest, ForwardPerspectiveValue, FieldCentric
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController
from phoenix6.units import *
from robotpy_ext.autonomous.selector_tests import test_all_autonomous

from robot_state import RobotState


class FieldCentricReefAlign(SwerveRequest):

    class BranchSide(Enum):
        CLOSEST = auto()
        RIGHT = auto()
        LEFT = auto()

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

        self._field_centric = FieldCentric()

        self.translation_x_controller = PhoenixPIDController(0.0, 0.0, 0.0)
        self.translation_y_controller = PhoenixPIDController(0.0, 0.0, 0.0)

        return

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


        # Get nearest reef side targets (2 poses per side) (O(2n))
        closest_index, closest_target = min(enumerate(RobotState.get_reef_targets()), key=lambda p: p[1].translation().distance(current_pose.translation()))
        if closest_index % 2 == 0:
            closest_direction = self.BranchSide.LEFT
        else:
            closest_direction = self.BranchSide.RIGHT

        side_target = None
        for target in RobotState.get_reef_targets():
            if abs(target.rotation().radians() - target.rotation().radians()) <= 1e-6 and target is not closest_target:
                side_target = target
                break
        if closest_direction is self.BranchSide.LEFT:
            side_direction = self.BranchSide.RIGHT
        else:
            side_direction = self.BranchSide.LEFT

        # Depending on the direction parameter, select the target
        if self.direction is self.BranchSide.CLOSEST:
            target = closest_target
        elif self.direction is closest_direction:
            target = closest_target
        else:
            target = side_target

        # Check if we should enable the PID controllers
        should_align = True
        if abs(target.rotation().degrees() - current_pose.rotation().degrees()) > 20:
            should_align = False
        if target.translation().distance(current_pose.translation()) > 0.5:
            should_align = False

        if should_align and target is not None:
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

            velocity_x_output = translation_x_output
            velocity_y_output = translation_y_output
        else:
            velocity_x_output = self.velocity_x
            velocity_y_output = self.velocity_y

        return (
            self._field_centric
            .with_velocity_x(velocity_x_output)
            .with_velocity_y(velocity_y_output)
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








