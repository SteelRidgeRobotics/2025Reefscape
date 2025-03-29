import math
from typing import Callable, Self

from phoenix6 import StatusCode
from phoenix6.swerve import SwerveModule, SwerveControlParameters, Translation2d
from phoenix6.swerve.requests import FieldCentricFacingAngle, ForwardPerspectiveValue, SwerveRequest
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController
from phoenix6.units import meters_per_second, radians_per_second
from wpilib import DriverStation
from wpimath.geometry import Pose2d

from constants import Constants


class DriverAssist(SwerveRequest):

    def __init__(self) -> None:
        self.velocity_x: meters_per_second = 0  # Velocity forward/back
        self.velocity_y: meters_per_second = 0  # Velocity left/right
        self.rotational_rate: radians_per_second = 0  # Angular rate, CCW positive

        self.target_pose = Pose2d()  # The target pose we align to

        self.deadband: meters_per_second = 0  # Deadband on linear velocity
        self.rotational_deadband: radians_per_second = 0  # Deadband on angular velocity

        self.drive_request_type: SwerveModule.DriveRequestType = SwerveModule.DriveRequestType.VELOCITY  # Control velocity of drive motor directly
        self.steer_request_type: SwerveModule.SteerRequestType = SwerveModule.SteerRequestType.POSITION  # Steer motor uses position control

        self.desaturate_wheel_speeds: bool = True  # This ensures no wheel speed is above the maximum speed

        self.forward_perspective: ForwardPerspectiveValue = ForwardPerspectiveValue.OPERATOR_PERSPECTIVE  # Operator perspective is forward

        self.target_pose: Pose2d = Pose2d()

        self.translation_controller = PhoenixPIDController(0.0, 0.0, 0.0)  # PID controller for translation

        self.elevator_up_function = lambda: False  # Callback for whether the elevator is up or not

        self._field_centric_facing_angle = FieldCentricFacingAngle()
        self.heading_controller = self._field_centric_facing_angle.heading_controller

    def apply(self, parameters: SwerveControlParameters, modules: list[SwerveModule]) -> StatusCode:
        current_pose = parameters.current_pose
        alliance = DriverStation.getAlliance()

        target_direction = self.target_pose.rotation() + parameters.operator_forward_direction

        cos_theta = target_direction.cos()
        sin_theta = target_direction.sin()

        rotated_velocity = Translation2d(
            self.velocity_x * cos_theta + self.velocity_y * sin_theta,
            -self.velocity_x * sin_theta + self.velocity_y * cos_theta
        )

        current_transformed = current_pose.translation().rotateBy(-target_direction)
        target_transformed = self.target_pose.translation().rotateBy(-target_direction)

        y_error = current_transformed.Y() - target_transformed.Y()

        if alliance == DriverStation.Alliance.kRed:
            y_error = -y_error

        horizontal_velocity = self.translation_controller.calculate(y_error, 0, parameters.timestamp)

        if self.elevator_up_function():
            horizontal_velocity *= 0.3333

        corrected_velocity = Translation2d(rotated_velocity.X(), horizontal_velocity)

        field_relative_velocity = corrected_velocity.rotateBy(target_direction)
        magnitude = math.sqrt(self.velocity_x ** 2 + self.velocity_y ** 2)

        return (
            self._field_centric_facing_angle
            .with_velocity_x(field_relative_velocity.X() * magnitude)
            .with_velocity_y(field_relative_velocity.Y() * magnitude)
            .with_target_direction(
                target_direction if abs(target_direction.degrees() - current_pose.rotation().degrees()) >= Constants.AutoAlignConstants.HEADING_TOLERANCE
                else current_pose.rotation()
            )
            .with_deadband(self.deadband)
            .with_rotational_deadband(self.rotational_deadband)
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

    def with_rotational_rate(self, rotational_rate: radians_per_second) -> Self:
        """
        Modifies the angular velocity we travel at and returns this request for method chaining.

        :param rotational_rate: The angular velocity we travel at
        :type rotational_rate: radians_per_second
        :returns: This request
        :rtype: DriverAssist
        """
        self.rotational_rate = rotational_rate
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

    def with_elevator_up_function(self, elevator_up_function: Callable[[], bool]) -> Self:
        """
        Modifies the function that returns whether the elevator is up and returns this request for method chaining.

        :param elevator_up_function: The function for whether the elevator is up or not
        :type elevator_up_function: Callable
        :returns: This request
        :rtype: DriverAssist
        """
        self.elevator_up_function = elevator_up_function
        return self
