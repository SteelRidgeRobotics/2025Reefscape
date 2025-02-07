"""
File: swerve.py
Description: The swerve subsystem, plus method to add MegaTag2 pose estimate.
Main Author: Caden Dalley
Co-Authors: James Haddix
"""

import math
from importlib import metadata
from typing import Callable, overload

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.util import DriveFeedforwards
from pathplannerlib.util.swerve import SwerveSetpointGenerator, SwerveSetpoint
from phoenix6 import swerve, units, utils, SignalLogger
from phoenix6.swerve.requests import ApplyRobotSpeeds
from phoenix6.swerve.swerve_drivetrain import DriveMotorT, SteerMotorT, EncoderT
from wpilib import DriverStation, Notifier, RobotController, DataLogManager
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import rotationsToRadians

import robot
from lib.limelight import LimelightHelpers


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

    _MAX_STEERING_VELOCITY: units.radians_per_second = rotationsToRadians(15)

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
        

        # Swerve request to apply during path following
        self._apply_robot_speeds = ApplyRobotSpeeds()

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

        # If PathPlanner is outdated, warn the user.
        if robot.has_outdated_pathplanner():
            DataLogManager.log(f"WARN: robotpy-pathplannerlib is version {metadata.version("robotpy-pathplannerlib")}. "
                               "PathPlanner must be greater than 2025.2.1 in order to use SetpointGenerator, defaulting"
                               " to standard control.")
        self._configure_auto_builder()
    
    def _configure_auto_builder(self) -> None:
        """
        Method to configure the auto builder
        """

        #Create config from GUI settings
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            lambda: self.get_state().pose,  # Supplier of current robot pose
            self.reset_pose,  # Consumer for seeding pose against auto
            lambda: self.get_state().speeds,  # Supplier of current robot speeds
            # Consumer of ChassisSpeeds and feedforwards to drive the robot
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds_from_setpoint(speeds, feedforwards)
            ),
            PPHolonomicDriveController(
                PIDConstants(10.0, 0.0, 0.0),
                PIDConstants(7.0, 0.0, 0.0)
            ),
            config,
            lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed, # If getAlliance() is None (maybe the robot doesn't know its alliance yet), it defaults to blue. This returns True if the alliance is red, and False otherwise
            self
        )

        # create set point generator
        self._setpoint_generator = SwerveSetpointGenerator(config, self._MAX_STEERING_VELOCITY)

        # set up the first setpoint using the robot's current state
        state = self.get_state()
        self._prev_setpoint = SwerveSetpoint(state.speeds, state.module_states, DriveFeedforwards.zeros(config.numModules))

    def _apply_robot_speeds_from_setpoint(self, speeds: ChassisSpeeds, feedforwards: DriveFeedforwards) -> ApplyRobotSpeeds:
        # get new previous setpoint using the current setpoint and given speeds
        self._prev_setpoint = self._setpoint_generator.generateSetpoint(
            self._prev_setpoint,
            speeds,
            0.02
        )

        # apply the speeds and feedforwards we get from this setpoint
        return (self._apply_robot_speeds
            .with_speeds(self._prev_setpoint.robot_relative_speeds)
            .with_wheel_force_feedforwards_x(self._prev_setpoint.feedforwards.robotRelativeForcesXNewtons)
            .with_wheel_force_feedforwards_y(self._prev_setpoint.feedforwards.robotRelativeForcesYNewtons)
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

        # if we are not in simulation, add vision measurement
        if not utils.is_simulation():
            self._add_vision_measurements()

    def _add_vision_measurements(self) -> None:
        """
        Add vision measurement to MegaTag2
        """

        LimelightHelpers.set_robot_orientation(
            "",
            self.pigeon2.get_yaw().value,
            self.pigeon2.get_angular_velocity_z_world().value,
            self.pigeon2.get_pitch().value,
            self.pigeon2.get_angular_velocity_y_world().value,
            self.pigeon2.get_roll().value,
            self.pigeon2.get_angular_velocity_x_world().value
        )

        # get botpose estimate with origin on blue side of field
        mega_tag2 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("")
        
        #if we are spinning slower than 720 deg/sec and we see tags
        if abs(self.pigeon2.get_angular_velocity_z_world().value) <= 720 and mega_tag2.tag_count > 0:
            
            # set and add vision measurement
            self.set_vision_measurement_std_devs((0.7, 0.7, 9999999))
            self.add_vision_measurement(mega_tag2.pose, utils.fpga_to_current_time(mega_tag2.timestamp_seconds))
    
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
