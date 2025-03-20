import math
from typing import Callable, overload

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from ntcore import NetworkTableInstance
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.logging import PathPlannerLogging
from phoenix6 import swerve, units, utils, SignalLogger
from phoenix6.swerve import SwerveModule
from phoenix6.swerve.requests import ApplyRobotSpeeds
from phoenix6.swerve.swerve_drivetrain import DriveMotorT, SteerMotorT, EncoderT
from wpilib import DriverStation, Notifier, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
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

        self._table = NetworkTableInstance.getDefault().getTable("Telemetry")

        # class SwerveModuleSendable(Sendable):
        #     def __init__(self, modules: list[SwerveModule[DriveMotorT, SteerMotorT, EncoderT]], rotation_getter: Callable[[], float]):
        #         super().__init__()
        #         self._modules = modules
        #         self._get_rotation = rotation_getter
        #
        #     def initSendable(self, builder: SendableBuilder):
        #         builder.setSmartDashboardType("SwerveDrive")
        #
        #         for i, name in enumerate(["Front Left", "Front Right", "Back Left", "Back Right"]):
        #             builder.addDoubleProperty(f"{name} Angle", lambda: self._modules[i].get_current_state().angle.radians(), lambda _: None)
        #             builder.addDoubleProperty(f"{name} Velocity", lambda: self._modules[i].get_current_state().speed, lambda _: None)
        #         builder.addDoubleProperty("Robot Angle", self._get_rotation, lambda _: None)
        # SmartDashboard.putData("Swerve Modules", SwerveModuleSendable(self.modules, lambda: (self.get_state_copy().pose.rotation() + self.get_operator_forward_direction()).radians()))

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

        self._sys_id_routine_to_apply = self._sys_id_routine_steer
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
                PIDConstants(7.0, 0.0, 0.0),
                PIDConstants(7.0, 0.0, 0.0)
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