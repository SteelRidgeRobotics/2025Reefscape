import math

from ntcore import NetworkTableInstance
from pathplannerlib.logging import PathPlannerLogging
from phoenix6 import swerve, utils
from wpilib import DataLogManager, DriverStation, Field2d, SmartDashboard, Mechanism2d, Color8Bit
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState

from subsystems.elevator import ElevatorSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.swerve import SwerveSubsystem


class RobotState:

    def __init__(self, drivetrain: SwerveSubsystem, pivot: PivotSubsystem, elevator: ElevatorSubsystem):
        self._swerve = drivetrain
        self._pivot = pivot
        self._elevator = elevator

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
            self._pivot_mech = self._elevator_mech.appendLigament("Pivot", 0.635, 90, 4, Color8Bit("#FEFEFE"))

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

    def update_mechanisms(self) -> None:
        self._elevator_mech.setLength(self._elevator.get_height())
        self._pivot_mech.setAngle(self._pivot.get_angle() - 90)

    def get_current_pose(self) -> Pose2d:
        """Returns the current pose of the robot on the field (blue-side origin)."""
        return self._swerve.get_state().pose

    def should_pivot_move(self) -> bool:
        return self._elevator.is_at_setpoint()
