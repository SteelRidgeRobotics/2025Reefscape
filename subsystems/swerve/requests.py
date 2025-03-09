from typing import Self

from pathplannerlib.config import RobotConfig
from pathplannerlib.util import DriveFeedforwards
from subsystems.swerve.util import SwerveSetpointGenerator, SwerveSetpoint
from phoenix6 import StatusCode
from phoenix6.swerve import SwerveModule, SwerveControlParameters, Translation2d
from phoenix6.swerve.requests import SwerveRequest, ApplyRobotSpeeds, ForwardPerspectiveValue, FieldCentric
from phoenix6.units import meters_per_second
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.units import radians_per_second


class ApplyRobotSetpointSpeeds(SwerveRequest):
    """
    Accepts a generic robot-centric ChassisSpeeds to apply PathPlanner's SwerveSetpointGenerator to allow for more path following control.
    """

    def __init__(self, config: RobotConfig, max_steer_velocity: radians_per_second) -> None:
        self.desired_speeds: ChassisSpeeds = ChassisSpeeds()
        """
        The desired robot-centric chassis speeds to apply to the drivetrain.
        """
        self.drive_request_type: SwerveModule.DriveRequestType = SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        """
        The type of control request to use for the drive motor.
        """
        self.steer_request_type: SwerveModule.SteerRequestType = SwerveModule.SteerRequestType.POSITION
        """
        The type of control request to use for the steer motor.
        """
        self.desaturate_wheel_speeds: bool = True
        """
        Whether to desaturate wheel speeds before applying. For more information, see
        the documentation of SwerveDriveKinematics.desaturateWheelSpeeds.
        """

        self._setpoint_generator = SwerveSetpointGenerator(config, max_steer_velocity)
        self._prev_setpoint = SwerveSetpoint(
            ChassisSpeeds(),
            [SwerveModuleState() for _ in range(config.numModules)],
            DriveFeedforwards.zeros(config.numModules)
        )

        self.__apply_robot_speeds = ApplyRobotSpeeds()

    def with_speeds(self, new_speeds: ChassisSpeeds) -> Self:
        """
        Modifies the speeds parameter and returns itself.

        The desired robot-centric chassis speeds to apply to the drivetrain.

        :param new_speeds: Parameter to modify
        :type new_speeds: ChassisSpeeds
        :returns: this object
        :rtype: ApplyRobotSetpointSpeeds
        """

        self.desired_speeds = new_speeds
        return self

    def with_drive_request_type(self, new_drive_request_type: SwerveModule.DriveRequestType) -> Self:
        """
        Modifies the drive request type and returns itself.

        The type of control request to use for the drive motor.

        :param new_drive_request_type: The desired drive request type to set.
        :type new_drive_request_type: SwerveModule.DriveRequestType
        :returns: this object
        :rtype: ApplyRobotSetpointSpeeds
        """
        self.drive_request_type = new_drive_request_type
        return self

    def with_steer_request_type(self, new_steer_request_type: SwerveModule.SteerRequestType) -> Self:
        """
        Modifies the steer request type and returns itself.

        The type of control request to use for the steer motor.

        :param new_steer_request_type: The desired steer request type to set.
        :type new_steer_request_type: SwerveModule.SteerRequestType
        :returns: this object
        :rtype: ApplyRobotSetpointSpeeds
        """
        self.steer_request_type = new_steer_request_type
        return self

    def with_desaturate_wheel_speeds(self, desaturate: bool) -> Self:
        """
        Modifies the desaturate wheel speeds property and returns itself.

        :param desaturate: Whether to enable desaturating wheel speeds
        :type desaturate: bool
        :returns: this object
        :rtype: ApplyRobotSetpointSpeeds
        """
        self.desaturate_wheel_speeds = desaturate
        return self

    def apply(self, parameters: SwerveControlParameters, modules_to_apply: list[SwerveModule]) -> StatusCode:
        print(parameters.update_period)
        self._prev_setpoint = self._setpoint_generator.generateSetpoint(self._prev_setpoint, self.desired_speeds, parameters.update_period)

        return (self.__apply_robot_speeds
                .with_drive_request_type(self.drive_request_type)
                .with_steer_request_type(self.steer_request_type)
                .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
                .with_speeds(self._prev_setpoint.robot_relative_speeds)
                .with_wheel_force_feedforwards_x(self._prev_setpoint.feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(self._prev_setpoint.feedforwards.robotRelativeForcesYNewtons)
                .apply(parameters, modules_to_apply)
                )


class SetpointFieldCentric(SwerveRequest):
    """
    Drives the swerve drivetrain in a field-centric manner.

    When users use this request, they specify the direction the robot should travel
    oriented against the field, and the rate at which their robot should rotate
    about the center of the robot.

    An example scenario is that the robot is oriented to the east, the VelocityX is
    +5 m/s, VelocityY is 0 m/s, and RotationRate is 0.5 rad/s. In this scenario, the
    robot would drive northward at 5 m/s and turn counterclockwise at 0.5 rad/s.
    """

    def __init__(self, config: RobotConfig, max_steer_velocity: radians_per_second):
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

        self._setpoint_generator = SwerveSetpointGenerator(config, max_steer_velocity)
        self._prev_setpoint = SwerveSetpoint(
            ChassisSpeeds(),
            [SwerveModuleState() for _ in range(config.numModules)],
            DriveFeedforwards.zeros(config.numModules)
        )

        self.__field_centric = FieldCentric()

        return

    def with_velocity_x(self, new_velocity_x: meters_per_second) -> 'SetpointFieldCentric':
        """
        Modifies the velocity_x parameter and returns itself.

        The velocity in the X direction, in m/s. X is defined as forward according to
        WPILib convention, so this determines how fast to travel forward.

        :param new_velocity_x: Parameter to modify
        :type new_velocity_x: meters_per_second
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.velocity_x = new_velocity_x
        return self

    def with_velocity_y(self, new_velocity_y: meters_per_second) -> 'SetpointFieldCentric':
        """
        Modifies the velocity_y parameter and returns itself.

        The velocity in the Y direction, in m/s. Y is defined as to the left according
        to WPILib convention, so this determines how fast to travel to the left.

        :param new_velocity_y: Parameter to modify
        :type new_velocity_y: meters_per_second
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.velocity_y = new_velocity_y
        return self

    def with_rotational_rate(self, new_rotational_rate: radians_per_second) -> 'SetpointFieldCentric':
        """
        Modifies the rotational_rate parameter and returns itself.

        The angular rate to rotate at, in radians per second. Angular rate is defined as
        counterclockwise positive, so this determines how fast to turn counterclockwise.

        :param new_rotational_rate: Parameter to modify
        :type new_rotational_rate: radians_per_second
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.rotational_rate = new_rotational_rate
        return self

    def with_deadband(self, new_deadband: meters_per_second) -> 'SetpointFieldCentric':
        """
        Modifies the deadband parameter and returns itself.

        The allowable deadband of the request, in m/s.

        :param new_deadband: Parameter to modify
        :type new_deadband: meters_per_second
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.deadband = new_deadband
        return self

    def with_rotational_deadband(self, new_rotational_deadband: radians_per_second) -> 'SetpointFieldCentric':
        """
        Modifies the rotational_deadband parameter and returns itself.

        The rotational deadband of the request, in radians per second.

        :param new_rotational_deadband: Parameter to modify
        :type new_rotational_deadband: radians_per_second
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.rotational_deadband = new_rotational_deadband
        return self

    def with_center_of_rotation(self, new_center_of_rotation: Translation2d) -> 'SetpointFieldCentric':
        """
        Modifies the center_of_rotation parameter and returns itself.

        The center of rotation the robot should rotate around. This is (0,0) by default,
        which will rotate around the center of the robot.

        :param new_center_of_rotation: Parameter to modify
        :type new_center_of_rotation: Translation2d
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.center_of_rotation = new_center_of_rotation
        return self

    def with_drive_request_type(self, new_drive_request_type: SwerveModule.DriveRequestType) -> 'SetpointFieldCentric':
        """
        Modifies the drive_request_type parameter and returns itself.

        The type of control request to use for the drive motor.

        :param new_drive_request_type: Parameter to modify
        :type new_drive_request_type: SwerveModule.DriveRequestType
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.drive_request_type = new_drive_request_type
        return self

    def with_steer_request_type(self, new_steer_request_type: SwerveModule.SteerRequestType) -> 'SetpointFieldCentric':
        """
        Modifies the steer_request_type parameter and returns itself.

        The type of control request to use for the drive motor.

        :param new_steer_request_type: Parameter to modify
        :type new_steer_request_type: SwerveModule.SteerRequestType
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.steer_request_type = new_steer_request_type
        return self

    def with_desaturate_wheel_speeds(self, new_desaturate_wheel_speeds: bool) -> 'SetpointFieldCentric':
        """
        Modifies the desaturate_wheel_speeds parameter and returns itself.

        Whether to desaturate wheel speeds before applying. For more information, see
        the documentation of SwerveDriveKinematics.desaturateWheelSpeeds.

        :param new_desaturate_wheel_speeds: Parameter to modify
        :type new_desaturate_wheel_speeds: bool
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.desaturate_wheel_speeds = new_desaturate_wheel_speeds
        return self

    def with_forward_perspective(self, new_forward_perspective: ForwardPerspectiveValue) -> 'SetpointFieldCentric':
        """
        Modifies the forward_perspective parameter and returns itself.

        The perspective to use when determining which direction is forward.

        :param new_forward_perspective: Parameter to modify
        :type new_forward_perspective: ForwardPerspectiveValue
        :returns: this object
        :rtype: SetpointFieldCentric
        """

        self.forward_perspective = new_forward_perspective
        return self

    def apply(self, parameters: 'SwerveControlParameters', modules_to_apply: list[SwerveModule]) -> StatusCode:
        desired_speeds = ChassisSpeeds(self.velocity_x, self.velocity_y, self.rotational_rate)
        self._prev_setpoint = self._setpoint_generator.generateSetpoint(self._prev_setpoint, desired_speeds, parameters.update_period)

        speeds = self._prev_setpoint.robot_relative_speeds

        return (self.__field_centric
                .with_velocity_x(speeds.vx)
                .with_velocity_y(speeds.vy)
                .with_rotational_rate(speeds.omega)
                .with_deadband(self.deadband)
                .with_rotational_deadband(self.rotational_deadband)
                .with_center_of_rotation(self.center_of_rotation)
                .with_drive_request_type(self.drive_request_type)
                .with_steer_request_type(self.steer_request_type)
                .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
                .with_forward_perspective(self.forward_perspective)
                .apply(parameters, modules_to_apply)
                )