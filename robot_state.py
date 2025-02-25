from commands2 import Subsystem
from commands2 import Subsystem
from ntcore import NetworkTableInstance
from pathplannerlib.logging import PathPlannerLogging
from phoenix6 import swerve, utils
from wpilib import Field2d, SmartDashboard, Mechanism2d, Color8Bit
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState

from subsystems.elevator import ElevatorSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.swerve import SwerveSubsystem


class RobotState(Subsystem):

    def __init__(self, drivetrain: SwerveSubsystem, pivot: PivotSubsystem, elevator: ElevatorSubsystem):
        super().__init__()
        self._swerve = drivetrain
        self._pivot = pivot
        self._elevator = elevator

        self._swerve_state = self._swerve.get_state()

        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)
        self._field.setRobotPose(Pose2d())

        self._table = NetworkTableInstance.getDefault().getTable("Telemetry")

        # Network table topics
        self._current_pose = self._table.getStructTopic("current_pose", Pose2d).publish()
        self._chassis_speeds = self._table.getStructTopic("chassis_speeds", ChassisSpeeds).publish()
        self._odom_freq = self._table.getDoubleTopic("odometry_frequency").publish()
        self._teleop_speed = self._table.getDoubleTopic("current_speed").publish()
        self._module_states = self._table.getStructArrayTopic("module_states", SwerveModuleState).publish()
        self._module_targets = self._table.getStructArrayTopic("module_targets", SwerveModuleState).publish()
        self._swerve_data = self._table.getSubTable("Swerve Data")
        self._swerve_data.getEntry(".type").setString("SwerveDrive")  # Tells Elastic what widget this is

        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self._field.getObject("targetPose").setPose(pose))
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self._field.getObject("activePath").setPoses(poses[::3]))

        self._superstructure_mechanism = None
        self._setup_simulation_mechanisms()

    def _setup_simulation_mechanisms(self):
        if utils.is_simulation():
            self._superstructure_mechanism = Mechanism2d(1, 5, Color8Bit(0, 0, 105))
            self._root = self._superstructure_mechanism.getRoot("Root", 1 / 2, 0.125)
            self._elevator_mech = self._root.appendLigament("Elevator", 0.2794, 90, 5, Color8Bit(194, 194, 194))
            self._pivot_mech = self._elevator_mech.appendLigament("Pivot", 0.635, 90, 4, Color8Bit(19, 122, 127))
            SmartDashboard.putData("Superstructure Mechanism", self._superstructure_mechanism)

    def periodic(self) -> None:
            state = self._swerve_state
            self._log_swerve_state(state)

    def simulationPeriodic(self) -> None:
        self.update_mechanisms()

    def _log_swerve_state(self, state: swerve.SwerveDrivetrain.SwerveDriveState) -> None:
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

        for i, module_state in enumerate(state.module_states):
            self._swerve_data.getEntry(f"Module {i} Angle").setDouble(module_state.angle.radians())
            self._swerve_data.getEntry(f"Module {i} Velocity").setDouble(module_state.speed)

        robot_angle = (self._swerve.get_operator_forward_direction() + state.pose.rotation()).radians()
        self._swerve_data.getEntry("Robot Angle").setDouble(robot_angle)

    def update_swerve_state(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        self._swerve_state = state

    def update_mechanisms(self) -> None:
        if self._superstructure_mechanism:
            self._elevator_mech.setLength(self._elevator.get_height())
            self._pivot_mech.setAngle(self._pivot.get_angle() - 90)

    def should_pivot_move(self) -> bool:
        return self._elevator.is_at_setpoint()
