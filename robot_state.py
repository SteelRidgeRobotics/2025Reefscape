import math

from ntcore import NetworkTableInstance
from pathplannerlib.logging import PathPlannerLogging
from phoenix6 import swerve, utils
from wpilib import DataLogManager, DriverStation, Field2d, SmartDashboard, Mechanism2d, Color8Bit
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.units import degreesToRadians

from constants import Constants
from subsystems.elevator import ElevatorSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.swerve import SwerveSubsystem


class RobotState:

    _blue_reef_targets = [
        Pose2d(3.091, 4.181, degreesToRadians(0)), # A
        Pose2d(3.091, 3.863, degreesToRadians(0)), # B
        Pose2d(3.656, 2.916, degreesToRadians(60)), # C
        Pose2d(3.956, 2.748, degreesToRadians(60)), # D
        Pose2d(5.023, 2.772, degreesToRadians(120)), # E
        Pose2d(5.323, 2.928, degreesToRadians(120)), # F
        Pose2d(5.850, 3.851, degreesToRadians(180)), # G
        Pose2d(5.862, 4.187, degreesToRadians(180)), # H
        Pose2d(5.347, 5.134, degreesToRadians(-120)), # I
        Pose2d(5.047, 5.290, degreesToRadians(-120)), # J
        Pose2d(3.932, 5.302, degreesToRadians(-60)), # K
        Pose2d(3.668, 5.110, degreesToRadians(-60)), # L
    ]

    _red_reef_targets = [
        Pose2d(
            Constants.FIELD_LAYOUT.getFieldLength() - pose.X(),
            Constants.FIELD_LAYOUT.getFieldWidth() - pose.Y(),
            pose.rotation() + Rotation2d.fromDegrees(180)
        ) for pose in _blue_reef_targets
    ]

    def __init__(self, drivetrain: SwerveSubsystem, pivot: PivotSubsystem, elevator: ElevatorSubsystem):
        self._swerve = drivetrain
        self._pivot = pivot
        self._elevator = elevator

        DriverStation.startDataLog(DataLogManager.getLog())

        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)
        self._field.setRobotPose(Pose2d())

        # Robot speeds for general checking
        self._table = NetworkTableInstance.getDefault().getTable("Telemetry")
        self._current_pose = self._table.getStructTopic("current_pose", Pose2d).publish()
        self._chassis_speeds = self._table.getStructTopic("chassis_speeds", ChassisSpeeds).publish()
        self._odom_freq = self._table.getDoubleTopic("odometry_frequency").publish()
        self._teleop_speed = self._table.getDoubleTopic("current_speed").publish()

        # Additional swerve info
        self._module_states = self._table.getStructArrayTopic("module_states", SwerveModuleState).publish()
        self._module_targets = self._table.getStructArrayTopic("module_targets", SwerveModuleState).publish()

        # Swerve Data
        self._swerve_data = self._table.getSubTable("Swerve Data")
        self._swerve_data.getEntry(".type").setString("SwerveDrive")  # Tells Elastic what widget this is

        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self._field.getObject("targetPose").setPose(pose))
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self._field.getObject("activePath").setPoses(poses[::3]))

        if utils.is_simulation():
            self._superstructure_mechanism = Mechanism2d(0.5334, 2.286, Color8Bit("#000058"))
            self._root = self._superstructure_mechanism.getRoot("Root", 0.5334 / 2, 0.125)

            self._elevator_mech = self._root.appendLigament("Elevator", 0.2794, 90, 5, Color8Bit("#FFFFFF"))
            self._pivot_mech = self._elevator_mech.appendLigament("Pivot", 0.635, 0, 4, Color8Bit("#FEFEFE"))

            SmartDashboard.putData("Superstructure Mechanism", self._superstructure_mechanism)

    def log_swerve_state(self, state: swerve.SwerveDrivetrain.SwerveDriveState) -> None:
        """
        Logs desired info with the given swerve state. Called by the
        Phoenix 6 drivetrain method every time the odometry thread is
        updated.
        """

        self._field.setRobotPose(state.pose)
        self._current_pose.set(state.pose)

        self._odom_freq.set(1.0 / state.odometry_period)

        self._module_states.set(state.module_states)
        self._module_targets.set(state.module_targets)
        self._chassis_speeds.set(state.speeds)

        self._teleop_speed.set(abs(math.sqrt(state.speeds.vx ** 2 + state.speeds.vy ** 2)))

        self._swerve_data.getEntry("Front Left Angle").setDouble(state.module_states[0].angle.radians())
        self._swerve_data.getEntry("Front Left Velocity").setDouble(state.module_states[0].speed)
        self._swerve_data.getEntry("Front Right Angle").setDouble(state.module_states[1].angle.radians())
        self._swerve_data.getEntry("Front Right Velocity").setDouble(state.module_states[1].speed)
        self._swerve_data.getEntry("Back Left Angle").setDouble(state.module_states[2].angle.radians())
        self._swerve_data.getEntry("Back Left Velocity").setDouble(state.module_states[2].speed)
        self._swerve_data.getEntry("Back Right Angle").setDouble(state.module_states[3].angle.radians())
        self._swerve_data.getEntry("Back Right Velocity").setDouble(state.module_states[3].speed)
        self._swerve_data.getEntry("Robot Angle").setDouble((self._swerve.get_operator_forward_direction() + state.pose.rotation()).radians())

        NetworkTableInstance.getDefault().flush()

    @staticmethod
    def get_reef_targets() -> list[Pose2d]:
        """Returns all reef scoring poses."""
        if (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed:
            return RobotState._red_reef_targets
        return RobotState._blue_reef_targets

    def update_mechanisms(self) -> None:
        self._elevator_mech.setLength(self._elevator.get_height())
        self._pivot_mech.setAngle(self._pivot.get_angle())

    def get_current_pose(self) -> Pose2d:
        """Returns the current pose of the robot on the field (blue-side origin)."""
        return self._swerve.get_state().pose

    def should_pivot_move(self) -> bool:
        return self._elevator.is_at_setpoint()

    def get_latency_compensated_pose(self, dt: float) -> Pose2d:
        """Returns the current pose of the robot on the field (blue-side origin),
        compensated for latency.

        :param dt: The amount of time in seconds since the last
            update.
        :type dt: float
        :return: The current pose of the robot on the field with
            latency compensation.
        :rtype: Pose2d
        """
        state = self._swerve.get_state()
        speeds = state.speeds
        pose = state.pose

        return Pose2d(
            pose.X() + speeds.vx * dt,
            pose.Y() + speeds.vy * dt,
            pose.rotation() + speeds.omega * dt
            )