from enum import Enum, auto
import math
from typing import Callable, overload, Self

from constants import Constants
from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from ntcore import NetworkTableInstance
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.logging import PathPlannerLogging
from phoenix6 import swerve, units, utils, SignalLogger, StatusCode
from phoenix6.swerve import SwerveModule, SwerveControlParameters
from phoenix6.swerve.requests import ApplyRobotSpeeds, FieldCentric, FieldCentricFacingAngle, ForwardPerspectiveValue, SwerveRequest
from phoenix6.swerve.swerve_drivetrain import DriveMotorT, SteerMotorT, EncoderT
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController
from phoenix6.units import meters_per_second, meter, radians_per_second
from wpilib import DriverStation, Notifier, RobotController, Field2d, SmartDashboard
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.units import degreesToRadians
from wpiutil import Sendable, SendableBuilder

class DriverAssist(SwerveRequest):

    class BranchSide(Enum):
        """
        Enum that determines which side of the reef we score on, left branch or right branch.
        """
        LEFT = auto()
        RIGHT = auto()

    # All poses on the blue side of the reef
    _blue_branch_left_targets = [
        Pose2d(3.091, 4.181, degreesToRadians(0)),  # A
        Pose2d(3.656, 2.916, degreesToRadians(60)),  # C
        Pose2d(5.023, 2.772, degreesToRadians(120)),  # E
        Pose2d(5.850, 3.851, degreesToRadians(180)),  # G
        Pose2d(5.347, 5.134, degreesToRadians(240)),  # I
        Pose2d(3.932, 5.302, degreesToRadians(300)),  # K
    ]  # 4.4705

    _blue_branch_right_targets = [
        Pose2d(3.091, 3.863, degreesToRadians(0)),  # B
        Pose2d(3.956, 2.748, degreesToRadians(60)),  # D
        Pose2d(5.323, 2.928, degreesToRadians(120)),  # F
        Pose2d(5.862, 4.187, degreesToRadians(180)),  # H
        Pose2d(5.047, 5.290, degreesToRadians(240)),  # J
        Pose2d(3.668, 5.110, degreesToRadians(300)),  # L
    ]  # 4.4765

    # TODO: We should be rotating these poses by 180, but a logic error later in the code means we need to keep them the same way to align correctly. This error should be found.
    # To find the poses on the red side of the reef, we mirror each pose and rotate by 180 degrees.
    _red_branch_left_targets = [
        Pose2d(
            Constants.FIELD_LAYOUT.getFieldLength() - pose.X(),
            Constants.FIELD_LAYOUT.getFieldWidth() - pose.Y(),
            pose.rotation() + Rotation2d.fromDegrees(180)
        ) for pose in _blue_branch_left_targets
    ]

    _red_branch_right_targets = [
        Pose2d(
            Constants.FIELD_LAYOUT.getFieldLength() - pose.X(),
            Constants.FIELD_LAYOUT.getFieldWidth() - pose.Y(),
            pose.rotation() + Rotation2d.fromDegrees(180)
        ) for pose in _blue_branch_right_targets
    ]

    _branch_targets = {
        DriverStation.Alliance.kBlue: {
            BranchSide.LEFT: _blue_branch_left_targets,
            BranchSide.RIGHT: _blue_branch_right_targets,
        },
        DriverStation.Alliance.kRed: {
            BranchSide.LEFT: _red_branch_left_targets,
            BranchSide.RIGHT: _red_branch_right_targets,
        }
    }

    def __init__(self) -> None:
        self.velocity_x: meters_per_second = 0 # Velocity forward/back
        self.velocity_y: meters_per_second = 0 # Velocity left/right
        self.rotational_rate: radians_per_second = 0 # Angular rate, CCW positive
        
        self.target_pose = Pose2d() # The target pose we align to

        self.deadband: meters_per_second = 0 # Deadband on linear velocity
        self.rotational_deadband: radians_per_second = 0 # Deadband on angular velocity
        
        self.drive_request_type: SwerveModule.DriveRequestType = SwerveModule.DriveRequestType.VELOCITY # Control velocity of drive motor directly
        self.steer_request_type: SwerveModule.SteerRequestType = SwerveModule.SteerRequestType.POSITION # Steer motor uses position control
        
        self.desaturate_wheel_speeds: bool = True # This ensures no wheel speed is above the maximum speed
        
        self.forward_perspective: ForwardPerspectiveValue = ForwardPerspectiveValue.OPERATOR_PERSPECTIVE # Operator perspective is forward
        
        self.fallback: FieldCentric = FieldCentric()  # Fallback if we are too far from the target pose
        self.max_distance: meter = 0 # Max distance we can be from the target pose
        
        self.change_target_pose: bool = True

        self.branch_side: DriverAssist.BranchSide = DriverAssist.BranchSide.LEFT # Which side to align to on the reef
        
        self.translation_controller = PhoenixPIDController(0.0, 0.0, 0.0) # PID controller for translation
        
        self.elevator_up_function = lambda: False # Callback for whether the elevator is up or not

        self._field_centric_facing_angle = FieldCentricFacingAngle() # This request utilizes FCFA to align
        self.heading_controller = self._field_centric_facing_angle.heading_controller # Storing this makes setting the PID easier later

        self._current_alliance: DriverStation.Alliance = DriverStation.getAlliance() # Store current alliance to reduce calls later

        # self._target_pose_pub = NetworkTableInstance.getDefault().getTable("Telemetry").getStructTopic("Target Pose", Pose2d).publish()
        # self._horizontal_velocity_pub = NetworkTableInstance.getDefault().getTable("Telemetry").getDoubleTopic("Horizontal Velocity").publish()

    @staticmethod
    def _get_distance_to_pose(robot_pose: Pose2d, target_pose: Pose2d) -> float:
        """
        Get distance from the robot to a pose. This is used to determine whether we should align or not.
        """
        return math.sqrt((target_pose.X() - robot_pose.X()) ** 2 + (target_pose.Y() - robot_pose.Y()) ** 2)

    @staticmethod
    def _get_distance_to_line(robot_pose: Pose2d, target_pose: Pose2d):
        """
        Find the distance from the robot to the line emanating from the target pose.
        """

        # To accomplish this, we need to find the intersection point of the line 
        # emanating from the target pose and the line perpendicular to it that passes through the robot pose.
        # If theta is the target rotation, tan(theta) is the slope of the line from the pose. We can just call that S for the sake of solving this.
        # Where S is the slope, (t_x, t_y) is the target position, and (r_x, r_y) is the robot position:
        # S(x - t_x) + t_y = -1/S(x - r_x) + r_y
        # Sx - St_x + t_y = -x/S + r_x/S + r_y
        # Sx - St_x + t_y - r_y = (r_x - x)/S
        # S^2x - S^2t_x + St_y - Sr_y = r_x - x
        # S^2x + x = r_x + S^2t_x - St_y + Sr_y
        # x(S^2 + 1) = r_x + S^2t_x - St_y + Sr_y
        # x = (r_x + S^2t_x - St_y + Sr_y)/(S^2 + 1)
        # We can then plug in x into our first equation to find y. This will give us the intersection point, which is the pose we want to find distance to.

        slope = math.tan(target_pose.rotation().radians())

        x = (robot_pose.X() + slope ** 2 * target_pose.X() - slope * target_pose.Y() + slope * robot_pose.Y()) / (slope ** 2 + 1)
        y = slope * (x - target_pose.X()) + target_pose.Y()

        # this is a possible pose. if it's on the wrong side of the reef it's a false pose.
        possible_pose = Pose2d(x, y, target_pose.rotation())

        # If we are on the blue alliance, this is the reef X. Otherwise, mirror this x coordinate for the red alliance.
        reef_x = 4.4735 if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else Constants.FIELD_LAYOUT.getFieldLength() - 4.4735

        # here we check if the reef pose is on the same side as our possible pose. if it is, we return the distance between the robot and the possible pose.
        if (target_pose.X() - reef_x <= 0) == (x - reef_x <= 0):
            return math.sqrt((possible_pose.X() - robot_pose.X()) ** 2 + (possible_pose.Y() - robot_pose.Y()) ** 2)
        else:
            # Returning infinity ensures this pose won't be considered
            return math.inf

    def _find_closest_pose(self, robot_pose: Pose2d, list_of_poses: list[Pose2d]) -> Pose2d:
        """
        Find the closest pose to the robot in a list of poses
        """
        closest_pose = Pose2d(math.inf, math.inf, Rotation2d.fromDegrees(0))
        closest_distance = math.inf

        # Iterate through poses, finding which is closest
        for pose in list_of_poses:
            pose_distance = self._get_distance_to_line(robot_pose, pose)
            if pose_distance < closest_distance:
                closest_pose = pose
                closest_distance = pose_distance

        return closest_pose

    def apply(self, parameters: SwerveControlParameters, modules: list[SwerveModule]) -> StatusCode:

        current_pose = parameters.current_pose

        if self.change_target_pose:
            
            self.target_pose = self._find_closest_pose(current_pose, self._branch_targets[DriverStation.getAlliance()][self.branch_side])

        if self._get_distance_to_pose(current_pose, self.target_pose) <= self.max_distance:

            target_direction = self.target_pose.rotation() + parameters.operator_forward_direction

            # New X and Y axis in the direction of the target pose
            rotated_coordinate = Translation2d(self.velocity_x, self.velocity_y).rotateBy(-target_direction)

            # Ignore the Y value because we only care about the component in the direction of the target pose to get our velocity towards the pose
            velocity_towards_pose = rotated_coordinate.X()

            # We need to do the same thing to find the velocity in the Y direction, but this time we'll use a PID controller rather than the driver input.

            rotated_current_pose = current_pose.rotateBy(-target_direction)
            rotated_target_pose = self.target_pose.rotateBy(-target_direction)

            # Find horizontal velocity (relative to pose) using our PID controller
            horizontal_velocity = self.translation_controller.calculate(rotated_current_pose.Y(), rotated_target_pose.Y(), parameters.timestamp)

            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                horizontal_velocity *= -1

            # Take these velocities and rotate it back into the field coordinate system
            field_relative_velocity = Translation2d(velocity_towards_pose, horizontal_velocity).rotateBy(target_direction)

            if self.elevator_up_function():
                field_relative_velocity *= 0.25

            return (
                self._field_centric_facing_angle
                .with_velocity_x(field_relative_velocity.X())
                .with_velocity_y(field_relative_velocity.Y())
                .with_target_direction(
                    target_direction if abs(target_direction.degrees() - current_pose.rotation().degrees()) >= Constants.AutoAlignConstants.HEADING_TOLERANCE else current_pose.rotation()
                )
                .with_deadband(self.deadband)
                .with_rotational_deadband(self.rotational_deadband)
                .with_drive_request_type(self.drive_request_type)
                .with_steer_request_type(self.steer_request_type)
                .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
                .with_forward_perspective(self.forward_perspective)
                .apply(parameters, modules)
            )

        else:
            return (self.fallback
                    .with_velocity_x(self.velocity_x)
                    .with_velocity_y(self.velocity_y)
                    .with_rotational_rate(self.rotational_rate)
                    .apply(parameters, modules))

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

    def with_branch_side(self, branch_side: BranchSide) -> Self:
        """
        Modifies the branch we target and returns this request for method chaining.

        :param branch_side: The branch to align
        :type branch_side: DriverAssist.BranchSide
        :returns: This request
        :rtype: DriverAssist
        """

        self.branch_side = branch_side
        return self
    
    def with_change_target_pose(self, change_target_pose: bool) -> Self:
        """
        Modifies whether we change the target pose and returns this request for method chaining.

        :param change_target_pose: Whether we change the target pose
        :type change_target_pose: bool
        :returns: This request
        :rtype: DriverAssist
        """

        self.change_target_pose = change_target_pose
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

    def with_elevator_up_function(self, elevator_up_function: Callable[[], bool]):
        """
        Modifies the function that returns whether the elevator is up and returns this request for method chaining.

        :param elevator_up_function: The function for whether the elevator is up or not
        :type elevator_up_function: Callable
        :returns: This request
        :rtype: DriverAssist
        """

        self.elevator_up_function = elevator_up_function
        return self