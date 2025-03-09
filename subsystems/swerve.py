from enum import Enum, auto
import math
from typing import Callable, overload, Self

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6.units import *

from constants import Constants
from ntcore import NetworkTableInstance
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.logging import PathPlannerLogging
from phoenix6 import swerve, units, utils, SignalLogger, StatusCode
from phoenix6.swerve import SwerveModule, SwerveDrivetrain
from phoenix6.swerve.requests import ApplyRobotSpeeds, SwerveRequest, FieldCentric, FieldCentricFacingAngle, ForwardPerspectiveValue
from phoenix6.swerve.swerve_drivetrain import DriveMotorT, SteerMotorT, EncoderT, SwerveControlParameters
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController
from wpilib import DriverStation, Notifier, RobotController, Field2d, SmartDashboard
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.units import degreesToRadians
from wpiutil import Sendable, SendableBuilder


class SwerveSubsystem(Subsystem, swerve.SwerveDrivetrain):
    """
   Class that extends the Phoenix 6 SwerveDrivetrain class and implements
   Subsystem so it can easily be used in command-based projects.
   """

    _SIM_LOOP_PERIOD: units.second = 0.005

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

    @overload
    def __init__(
            self,
            drive_motor_type: type,
            steer_motor_type: type,
            encoder_type: type,
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:     Type of the drive motor
        :type drive_motor_type:      type
        :param steer_motor_type:     Type of the steer motor
        :type steer_motor_type:      type
        :param encoder_type:         Type of the azimuth encoder
        :type encoder_type:          type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:  swerve.SwerveDrivetrainConstants
        :param modules:              Constants for each specific module
        :type modules:               list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
            self,
            drive_motor_type: type,
            steer_motor_type: type,
            encoder_type: type,
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            odometry_update_frequency: units.hertz,
            modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
            self,
            drive_motor_type: type,
            steer_motor_type: type,
            encoder_type: type,
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            odometry_update_frequency: units.hertz,
            odometry_standard_deviation: tuple[float, float, float],
            vision_standard_deviation: tuple[float, float, float],
            modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    # noinspection PyTypeChecker
    def __init__(
            self,
            drive_motor_type: type[DriveMotorT],
            steer_motor_type: type[SteerMotorT],
            encoder_type: type[EncoderT],
            drivetrain_constants: swerve.SwerveDrivetrainConstants,
            arg0=None,
            arg1=None,
            arg2=None,
            arg3=None,
    ):
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(
            self, drive_motor_type, steer_motor_type, encoder_type,
            drivetrain_constants, arg0, arg1, arg2, arg3
        )

        self.pigeon2.reset()

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        # Keep track if we've ever applied the operator perspective before or not
        self._has_applied_operator_perspective = False

        # Telemetry
        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)
        self._field.setRobotPose(Pose2d())

        self._table = NetworkTableInstance.getDefault().getTable("Telemetry")

        class SwerveModuleSendable(Sendable):
            def __init__(self, modules: list[SwerveModule[DriveMotorT, SteerMotorT, EncoderT]], rotation_getter: Callable[[], float]):
                super().__init__()
                self._modules = modules
                self._get_rotation = rotation_getter

            def initSendable(self, builder: SendableBuilder):
                builder.setSmartDashboardType("SwerveDrive")

                for i, name in enumerate(["Front Left", "Front Right", "Back Left", "Back Right"]):
                    builder.addDoubleProperty(f"{name} Angle", lambda: self._modules[i].get_current_state().angle.radians(), lambda _: None)
                    builder.addDoubleProperty(f"{name} Velocity", lambda: self._modules[i].get_current_state().speed, lambda _: None)
                builder.addDoubleProperty("Robot Angle", self._get_rotation, lambda _: None)
        SmartDashboard.putData("Swerve Modules", SwerveModuleSendable(self.modules, lambda: (self.get_state_copy().pose.rotation() + self.get_operator_forward_direction()).radians()))

        self._pose_pub = self._table.getStructTopic("current_pose", Pose2d).publish()
        self._speeds_pub = self._table.getStructTopic("chassis_speeds", ChassisSpeeds).publish()
        self._odom_freq = self._table.getDoubleTopic("odometry_frequency").publish()
        self._module_states_pub = self._table.getStructArrayTopic("module_states", SwerveModuleState).publish()
        self._module_targets_pub = self._table.getStructArrayTopic("module_targets", SwerveModuleState).publish()

        self._auto_target_pub = self._table.getStructTopic("auto_target", Pose2d).publish()
        self._auto_path_pub = self._table.getStructArrayTopic("auto_path", Pose2d).publish()
        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self._auto_target_pub.set(pose))
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self._auto_path_pub.set(poses))

        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing translation. This is used to find PID gains for the drive motors."""

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    # output is actually radians per second, but SysId only supports "volts"
                    self.set_control(
                        self._rotation_characterization.with_rotational_rate(output)
                    ),
                    # also log the requested output for SysId
                    SignalLogger.write_double("Rotational_Rate", output),
                ),
                lambda log: None,
                self,
            ),
        )
        """
        SysId routine for characterizing rotation.
        This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        """

        self._sys_id_routine_to_apply = self._sys_id_routine_translation
        """The SysId routine to test"""

        if utils.is_simulation():
            self._start_sim_thread()
        self._configure_auto_builder()
    
    def _configure_auto_builder(self) -> None:
        """
        Method to configure the auto builder
        """

        #Create config from GUI settings
        config = RobotConfig.fromGUISettings()
        self._apply_robot_speeds = ApplyRobotSpeeds()
        AutoBuilder.configure(
            lambda: self.get_state().pose,  # Supplier of current robot pose
            self.reset_pose,  # Consumer for seeding pose against auto
            lambda: self.get_state().speeds,  # Supplier of current robot speeds
            # Consumer of ChassisSpeeds and feedforwards to drive the robot
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds
                .with_speeds(speeds)
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed, # If getAlliance() is None (maybe the robot doesn't know its alliance yet), it defaults to blue. This returns True if the alliance is red, and False otherwise
            self
        )

    def apply_request(
            self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.
        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine_to_apply.dynamic(direction)

    def periodic(self) -> None:
        """
        Method to run the swerve drive periodically
        """
        
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

        state = self.get_state_copy()
        self._field.setRobotPose(state.pose)
        self._pose_pub.set(state.pose)
        self._odom_freq.set(1.0 / state.odometry_period)
        self._module_states_pub.set(state.module_states)
        self._module_targets_pub.set(state.module_targets)
        self._speeds_pub.set(state.speeds)

    def _start_sim_thread(self) -> None:
        """
        Start the simulation thread
        """
        def _sim_periodic():

            # the current timestamp, then find change from last time update.
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())
            
        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

class DriverAssist(SwerveRequest):

    class BranchSide(Enum):
        """
        Enum that determines which side of the reef we score on, left branch or right branch.
        """
        LEFT = auto()
        RIGHT = auto()

    # All poses on the blue side of the reef
    _blue_branch_left_targets = [
        Pose2d(3.091, 4.181, degreesToRadians(0)), # A
        Pose2d(3.656, 2.916, degreesToRadians(60)), # C
        Pose2d(5.023, 2.772, degreesToRadians(120)), # E
        Pose2d(5.850, 3.851, degreesToRadians(180)), # G
        Pose2d(5.347, 5.134, degreesToRadians(240)), # I
        Pose2d(3.932, 5.302, degreesToRadians(300)), # K
    ] #4.4705

    _blue_branch_right_targets = [
        Pose2d(3.091, 3.863, degreesToRadians(0)), # B
        Pose2d(3.956, 2.748, degreesToRadians(60)), # D
        Pose2d(5.323, 2.928, degreesToRadians(120)), # F
        Pose2d(5.862, 4.187, degreesToRadians(180)), # H
        Pose2d(5.047, 5.290, degreesToRadians(240)), # J
        Pose2d(3.668, 5.110, degreesToRadians(300)), # L
    ] #4.4765

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
        self.drive_request_type: SwerveModule.DriveRequestType = SwerveModule.DriveRequestType.VELOCITY
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
        self.fallback: FieldCentric = FieldCentric() # TODO: As of now, this fallback doesn't work. James will be fixing this in a later PR. The functionality of auto aligning is not affected by it.
        """
        The swerve request to fallback to if further than the max distance.
        """
        self.max_distance: meter = 0
        """
        The max distance to allow aligning to a target. 
        """
        self.branch_side: DriverAssist.BranchSide = DriverAssist.BranchSide.LEFT
        """
        The branch side to align to.
        """
        self.translation_controller = PhoenixPIDController(0.0, 0.0, 0.0)
        """
        The PID controller used to maintain the desired horizontal translation, relative to the robot's chassis.
        Users can specify the PID gains to change how aggressively to maintain
        position.
        
        This PID controller operates on meters and outputs a target
        translation rate in meters per second.
        """
        self.__field_centric_facing_angle = FieldCentricFacingAngle()
        self.heading_controller = self.__field_centric_facing_angle.heading_controller
        """
        The PID controller used to maintain the desired heading.
        Users can specify the PID gains to change how aggressively to maintain
        heading.

        This PID controller operates on heading radians and outputs a target
        rotational rate in radians per second. Note that continuous input should
        be enabled on the range [-pi, pi].
        """

        self.__current_alliance: DriverStation.Alliance = DriverStation.getAlliance()

        self._target_pose = Pose2d()

        self._target_pose_pub = NetworkTableInstance.getDefault().getTable("Telemetry").getStructTopic("Target Pose", Pose2d).publish()
        self._horizontal_velocity_pub = NetworkTableInstance.getDefault().getTable("Telemetry").getDoubleTopic("Horizontal Velocity").publish()

    @staticmethod
    def _get_distance_to_pose(robot_pose: Pose2d, target_pose: Pose2d) -> float:
        return math.sqrt((target_pose.X() - robot_pose.X()) ** 2 + (target_pose.Y() - robot_pose.Y()) ** 2)

    @staticmethod
    def _get_distance_to_line(robot_pose: Pose2d, target_pose: Pose2d):
        """Find the distance from the robot to the line emanating from the target pose."""

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

        reef_x = 4.4735 if DriverStation.getAlliance() == DriverStation.Alliance.kBlue else Constants.FIELD_LAYOUT.getFieldLength() - 4.4735

        # here we check if the reef pose is on the same side as our possible pose. if it is, we return the distance between the robot and the possible pose.
        if (target_pose.X() - reef_x <= 0) == (x - reef_x <= 0):
            return math.sqrt((possible_pose.X() - robot_pose.X()) ** 2 + (possible_pose.Y() - robot_pose.Y()) ** 2)
        else:
            return math.inf
    
    def _find_closest_pose(self, robot_pose: Pose2d, list_of_poses: list[Pose2d]) -> Pose2d:
        closest_pose = Pose2d(math.inf, math.inf, Rotation2d.fromDegrees(0))
        closest_distance = math.inf

        for pose in list_of_poses:
            pose_distance = self._get_distance_to_line(robot_pose, pose)
            if pose_distance < closest_distance:
                closest_pose = pose
                closest_distance = pose_distance
        
        return closest_pose

    def apply(self, parameters: SwerveControlParameters, modules: list[SwerveModule]) -> StatusCode:
        current_pose = parameters.current_pose

        self._target_pose = self._find_closest_pose(current_pose, self._branch_targets[DriverStation.getAlliance()][self.branch_side])

        self._target_pose_pub.set(self._target_pose)

        if self._get_distance_to_pose(current_pose, self._target_pose) <= self.max_distance:

            target_direction = self._target_pose.rotation() + parameters.operator_forward_direction

            # New X and Y axis in the direction of the target pose
            rotated_coordinate = Translation2d(self.velocity_x, self.velocity_y).rotateBy(-target_direction)

            # Ignore the Y value because we only care about the component in the direction of the target pose to get our velocity towards the pose
            velocity_towards_pose = rotated_coordinate.X()

            # We need to do the same thing to find the velocity in the Y direction, but this time we'll use a PID controller rather than the driver input.

            rotated_current_pose = current_pose.rotateBy(-target_direction)
            rotated_target_pose = self._target_pose.rotateBy(-target_direction)

            # Find horizontal velocity (relative to pose) using our PID controller
            horizontal_velocity = self.translation_controller.calculate(rotated_current_pose.Y(), rotated_target_pose.Y(), parameters.timestamp)

            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                horizontal_velocity *= -1
            
            self._horizontal_velocity_pub.set(horizontal_velocity)

            # Take these velocities and rotate it back into the field coordinate system
            field_relative_velocity = Translation2d(velocity_towards_pose, horizontal_velocity).rotateBy(target_direction)

            return (
                self.__field_centric_facing_angle
                .with_velocity_x(field_relative_velocity.X())
                .with_velocity_y(field_relative_velocity.Y())
                .with_target_direction(target_direction)
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
