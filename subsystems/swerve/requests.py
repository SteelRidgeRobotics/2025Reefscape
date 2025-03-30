from typing import Self

from phoenix6 import StatusCode
from phoenix6.swerve import SwerveModule, SwerveControlParameters, Translation2d
from phoenix6.swerve.requests import FieldCentricFacingAngle, ForwardPerspectiveValue, SwerveRequest
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController
from phoenix6.units import meters_per_second, radians_per_second
from wpilib import DriverStation
from wpimath.geometry import Pose2d


class DriverAssist(SwerveRequest):

    def __init__(self) -> None:
        self.velocity_x: meters_per_second = 0  # Velocity forward/back
        self.velocity_y: meters_per_second = 0  # Velocity left/right

        self.target_pose = Pose2d()  # The target pose we align to

        self.deadband: meters_per_second = 0  # Deadband on linear velocity
        self.rotational_deadband: radians_per_second = 0  # Deadband on angular velocity

        self.drive_request_type: SwerveModule.DriveRequestType = SwerveModule.DriveRequestType.VELOCITY  # Control velocity of drive motor directly
        self.steer_request_type: SwerveModule.SteerRequestType = SwerveModule.SteerRequestType.POSITION  # Steer motor uses position control

        self.desaturate_wheel_speeds: bool = True  # This ensures no wheel speed is above the maximum speed

        self.forward_perspective: ForwardPerspectiveValue = ForwardPerspectiveValue.OPERATOR_PERSPECTIVE  # Operator perspective is forward

        self.target_pose: Pose2d = Pose2d()

        self.translation_controller = PhoenixPIDController(0.0, 0.0, 0.0)  # PID controller for translation

        self._field_centric_facing_angle = FieldCentricFacingAngle()
        self.heading_controller = self._field_centric_facing_angle.heading_controller

    def apply(self, parameters: SwerveControlParameters, modules: list[SwerveModule]) -> StatusCode:
        current_pose = parameters.current_pose
        target_rotation = self.target_pose.rotation() + parameters.operator_forward_direction

        y_error = current_pose.translation().rotateBy(-target_rotation).Y() - self.target_pose.translation().rotateBy(-target_rotation).Y()
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            y_error *= -1

        field_relative_velocity = Translation2d(
            self.velocity_x * target_rotation.cos() + self.velocity_y * target_rotation.sin(),
            self.translation_controller.calculate(y_error, 0, parameters.timestamp) * 0.333
        ).rotateBy(target_rotation)

        return (
            self._field_centric_facing_angle
            .with_velocity_x(field_relative_velocity.X())
            .with_velocity_y(field_relative_velocity.Y())
            .with_target_direction(target_rotation)
            .with_deadband(self.deadband)
            .with_rotational_deadband(self.rotational_deadband / field_relative_velocity.norm())
            .with_drive_request_type(self.drive_request_type)
            .with_steer_request_type(self.steer_request_type)
            .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
            .with_forward_perspective(self.forward_perspective)
            .apply(parameters, modules)
        )

    def with_target_pose(self, new_target_pose: Pose2d) -> Self:
        """
        Modifies the pose to align with.
        :param new_target_pose: New target pose
        :type new_target_pose: Pose2d
        :return: This request
        :rtype: DriverAssist
        """
        self.target_pose = new_target_pose
        return self

    def with_velocity_x(self, velocity_x: meters_per_second) -> Self:
        """
        Modifies the velocity we travel forwards and returns this request for method chaining.
        
        :param velocity_x: The velocity we travel forwards
        :type velocity_x: meters_per_second
        :returns: This request
        :rtype: DriverAssist
        """
        self.velocity_x = velocity_x
        return self

    def with_velocity_y(self, velocity_y: meters_per_second) -> Self:
        """
        Modifies the velocity we travel right and returns this request for method chaining.

        :param velocity_y: The velocity we travel right
        :type velocity_y: meters_per_second
        :returns: This request
        :rtype: DriverAssist
        """
        self.velocity_y = velocity_y
        return self

    def with_drive_request_type(self, new_drive_request_type: SwerveModule.DriveRequestType) -> Self:
        """
        Modifies the drive_request_type parameter and returns itself.

        The type of control request to use for the drive motor.

        :param new_drive_request_type: Parameter to modify
        :type new_drive_request_type: SwerveModule.DriveRequestType
        :returns: this object
        :rtype: DriverAssist
        """
        self.drive_request_type = new_drive_request_type
        return self

    def with_steer_request_type(self, new_steer_request_type: SwerveModule.SteerRequestType) -> Self:
        """
        Modifies the steer_request_type parameter and returns itself.

        The type of control request to use for the drive motor.

        :param new_steer_request_type: Parameter to modify
        :type new_steer_request_type: SwerveModule.SteerRequestType
        :returns: this object
        :rtype: DriverAssist
        """
        self.steer_request_type = new_steer_request_type
        return self

    def with_translation_pid(self, p: float, i: float, d: float) -> Self:
        """
        Modifies the translation PID gains and returns this request for method chaining.
        
        :param p: The proportional gain
        :type p: float
        :param i: The integral gain
        :type i: float
        :param d: The derivative gain
        :type d: float
        :returns: This request
        :rtype: DriverAssist
        """
        self.translation_controller.setPID(p, i, d)
        return self

    def with_heading_pid(self, p: float, i: float, d: float) -> Self:
        """
        Modifies the heading PID gains and returns this request for method chaining.
        
        :param p: The proportional gain
        :type p: float
        :param i: The integral gain
        :type i: float
        :param d: The derivative gain
        :type d: float
        :returns: This request
        :rtype: DriverAssist
        """
        self.heading_controller.setPID(p, i, d)
        return self

    def with_deadband(self, deadband: float) -> Self:
        """
        Modifies the velocity deadband and returns this request for method chaining.
        
        :param deadband: The velocity deadband
        :type deadband: float
        :returns: This request
        :rtype: DriverAssist
        """
        self.deadband = deadband
        return self

    def with_rotational_deadband(self, rotational_deadband: float) -> Self:
        """
        Modifies the rotational deadband and returns this request for method chaining.

        :param rotational_deadband: The rotational deadband
        :type rotational_deadband: float
        :returns: This request
        :rtype: DriverAssist
        """
        self.rotational_deadband = rotational_deadband
        return self
