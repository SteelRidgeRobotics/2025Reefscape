from typing import Callable, Self

from phoenix6 import StatusCode
from phoenix6.swerve import SwerveModule, SwerveControlParameters
from phoenix6.swerve.requests import FieldCentric, FieldCentricFacingAngle, ForwardPerspectiveValue, SwerveRequest
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController
from phoenix6.units import meters_per_second, meter, radians_per_second
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Translation2d

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

        self.fallback: FieldCentric = FieldCentric()  # Fallback if we are too far from the target pose
        self.max_distance: meter = 0  # Max distance we can be from the target pose

        self.target_pose: Pose2d = Pose2d()

        self.translation_controller = PhoenixPIDController(0.0, 0.0, 0.0)  # PID controller for translation

        self.elevator_up_function = lambda: False  # Callback for whether the elevator is up or not

        self._field_centric_facing_angle = FieldCentricFacingAngle()
        self.heading_controller = self._field_centric_facing_angle.heading_controller

    @staticmethod
    def _get_distance_to_pose(robot_pose: Pose2d, target_pose: Pose2d) -> float:
        """
        Get distance from the robot to a pose. This is used to determine whether we should align or not.
        """
        return (target_pose.X() - robot_pose.X()) ** 2 + (target_pose.Y() - robot_pose.Y()) ** 2

    def apply(self, parameters: SwerveControlParameters, modules: list[SwerveModule]) -> StatusCode:
        current_pose = parameters.current_pose
        alliance = DriverStation.getAlliance()

        distance_to_target = self._get_distance_to_pose(current_pose, self.target_pose)
        if distance_to_target > self.max_distance ** 2:
            return self.fallback.with_velocity_x(self.velocity_x).with_velocity_y(self.velocity_y).with_rotational_rate(self.rotational_rate).apply(parameters, modules)

        target_direction = self.target_pose.rotation() + parameters.operator_forward_direction
        rotated_velocity = Translation2d(self.velocity_x, self.velocity_y).rotateBy(-target_direction)

        velocity_towards_pose = rotated_velocity.X()

        current_y = current_pose.translation().rotateBy(-target_direction).Y()
        target_y = self.target_pose.translation().rotateBy(-target_direction).Y()

        # Adjust for red alliance to ensure consistent motion correction
        horizontal_velocity = self.translation_controller.calculate(
            -current_y if alliance == DriverStation.Alliance.kRed else current_y,
            -target_y if alliance == DriverStation.Alliance.kRed else target_y,
            parameters.timestamp
        )

        if self.elevator_up_function():
            horizontal_velocity *= 0.3333

        field_relative_velocity = Translation2d(velocity_towards_pose, horizontal_velocity).rotateBy(target_direction)

        return (
            self._field_centric_facing_angle
            .with_velocity_x(field_relative_velocity.X())
            .with_velocity_y(field_relative_velocity.Y())
            .with_target_direction(target_direction if abs(target_direction.degrees() - current_pose.rotation().degrees()) >= Constants.AutoAlignConstants.HEADING_TOLERANCE else current_pose.rotation())
            .with_deadband(self.deadband)
            .with_rotational_deadband(self.rotational_deadband)
            .with_drive_request_type(self.drive_request_type)
            .with_steer_request_type(self.steer_request_type)
            .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
            .with_forward_perspective(self.forward_perspective)
            .apply(parameters, modules)
        )

    def with_fallback(self, fallback) -> Self:
        """
        Modifies the fallback request and returns this request for method chaining.

        :param fallback: The fallback request
        :type fallback: SwerveRequest
        :returns: This request
        :rtype: DriverAssist
        """
        self.fallback = fallback
        return self

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

    def with_max_distance(self, max_distance: meter) -> Self:
        """
        Modifies the maximum distance we can be away from the target pose to be considered "close enough" (in meters) and returns this request for method chaining.

        :param max_distance: The maximum distance we can be away from the target pose to be considered "close enough"
        :type max_distance: meter
        :returns: This request
        :rtype: DriverAssist
        """
        self.max_distance = max_distance
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
