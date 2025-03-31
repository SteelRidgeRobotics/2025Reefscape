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

        self._target_pose = Pose2d()  # The target pose we align to

        self.deadband: meters_per_second = 0  # Deadband on linear velocity
        self.rotational_deadband: radians_per_second = 0  # Deadband on angular velocity

        self.drive_request_type: SwerveModule.DriveRequestType = SwerveModule.DriveRequestType.VELOCITY  # Control velocity of drive motor directly
        self.steer_request_type: SwerveModule.SteerRequestType = SwerveModule.SteerRequestType.POSITION  # Steer motor uses position control

        self.desaturate_wheel_speeds: bool = True  # This ensures no wheel speed is above the maximum speed

        self.forward_perspective: ForwardPerspectiveValue = ForwardPerspectiveValue.OPERATOR_PERSPECTIVE  # Operator perspective is forward

        self._target_pose: Pose2d = Pose2d()

        self.translation_controller = PhoenixPIDController(0.0, 0.0, 0.0)  # PID controller for translation

        self._field_centric_facing_angle = FieldCentricFacingAngle()
        self.heading_controller = self._field_centric_facing_angle.heading_controller

        self._target_pose = Pose2d()

    def apply(self, parameters: SwerveControlParameters, modules: list[SwerveModule]) -> StatusCode:
        current_pose = parameters.current_pose
        target_pose = self._target_pose
        op_dir = parameters.operator_forward_direction
        vel_x = self.velocity_x
        vel_y = self.velocity_y
        controller = self.translation_controller
        timestamp = parameters.timestamp

        target_rot = target_pose.rotation() + op_dir
        neg_rot = -target_rot

        current_trans = current_pose.translation()
        target_trans = target_pose.translation()
        rotated_current_y = current_trans.rotateBy(neg_rot).Y()
        rotated_target_y = target_trans.rotateBy(neg_rot).Y()
        y_error = rotated_current_y - rotated_target_y

        y_error *= -1 if DriverStation.getAlliance() == DriverStation.Alliance.kRed else 1

        cos_rot = target_rot.cos()
        sin_rot = target_rot.sin()

        controller_output = controller.calculate(y_error, 0, timestamp) * 0.333
        frv_x = vel_x * cos_rot + vel_y * sin_rot
        frv_y = controller_output
        field_relative_velocity = Translation2d(frv_x, frv_y).rotateBy(target_rot)

        builder = self._field_centric_facing_angle
        return (
            builder.with_velocity_x(field_relative_velocity.X())
            .with_velocity_y(field_relative_velocity.Y())
            .with_target_direction(target_rot)
            .with_deadband(self.deadband)
            .with_rotational_deadband(self.rotational_deadband)
            .with_drive_request_type(self.drive_request_type)
            .with_steer_request_type(self.steer_request_type)
            .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
            .with_forward_perspective(self.forward_perspective)
            .apply(parameters, modules)
        )

    @property
    def target_pose(self) -> Pose2d:
        return self._target_pose

    @target_pose.setter
    def target_pose(self, value: Pose2d) -> None:
        self._target_pose = value

    def with_target_pose(self, new_target_pose: Pose2d) -> Self:
        """
        Modifies the pose to align with.
        :param new_target_pose: New target pose
        :type new_target_pose: Pose2d
        :return: This request
        :rtype: DriverAssist
        """
        self._target_pose = new_target_pose
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
