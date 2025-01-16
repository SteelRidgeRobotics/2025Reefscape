import math

from ntcore import NetworkTableInstance
from pathplannerlib.logging import PathPlannerLogging
from phoenix6 import swerve
from wpilib import DataLogManager, DriverStation, Field2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpiutil import Sendable

from subsystems.swerve import SwerveSubsystem


class RobotState:

    # noinspection PyTypeChecker
    def __init__(self, drivetrain: SwerveSubsystem):
        self.swerve = drivetrain

        DriverStation.startDataLog(DataLogManager.getLog())

        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)
        self._field.setRobotPose(Pose2d())

        # Robot speeds for general checking
        self._table = NetworkTableInstance.getDefault().getTable("Telemetry")
        self._current_pose = self._table.getStructTopic("currentPose", Pose2d).publish()
        self._chassis_speeds = self._table.getStructTopic("chassisSpeeds", ChassisSpeeds).publish()
        self._odom_freq = self._table.getDoubleTopic("Odometry Frequency").publish()
        self._teleop_speed = self._table.getDoubleTopic("Current Speed").publish()

        # Additional swerve info
        self._module_states = self._table.getStructArrayTopic("moduleStates", SwerveModuleState).publish()
        self._module_targets = self._table.getStructArrayTopic("moduleTargets", SwerveModuleState).publish()

        class SendableSwerveDrive(Sendable):
            def __init__(self):
                super().__init__()

            def initSendable(self, builder):
                state = drivetrain.get_state()
                builder.setSmartDashboardType("SwerveDrive")
                builder.addDoubleProperty("Front Left Angle", lambda: state.module_states[0].angle.radians(), lambda _: None)
                builder.addDoubleProperty("Front Left Velocity", lambda: state.module_states[0].speed, lambda _: None)
                builder.addDoubleProperty("Front Right Angle", lambda: state.module_states[1].angle.radians(), lambda _: None)
                builder.addDoubleProperty("Front Right Velocity", lambda: state.module_states[1].speed, lambda _: None)
                builder.addDoubleProperty("Back Left Angle", lambda: state.module_states[2].angle.radians(), lambda _: None)
                builder.addDoubleProperty("Back Left Velocity", lambda: state.module_states[2].speed, lambda _: None)
                builder.addDoubleProperty("Back Right Angle", lambda: state.module_states[3].angle.radians(), lambda _: None)
                builder.addDoubleProperty("Back Right Velocity", lambda: state.module_states[3].speed, lambda _: None)
                builder.addDoubleProperty("Robot Angle", lambda: state.pose.rotation().radians(), lambda _: None)

        SmartDashboard.putData("Swerve Drive", SendableSwerveDrive())

        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self._field.getObject("targetPose").setPose(pose))
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self._field.getObject("activePath").setPoses(poses[::3]))

    def log_swerve_state(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
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

        self._teleop_speed.set(abs(math.sqrt(state.speeds.vx**2+state.speeds.vy**2)))

    def get_current_pose(self) -> Pose2d:
        
        """Returns the current pose of the robot on the field (blue-side origin)."""
        return self.swerve.get_state().pose
    
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
        state = self.swerve.get_state()
        speeds = state.speeds
        pose = state.pose

        return Pose2d(pose.X() + speeds.vx * dt,
                      pose.Y() + speeds.vy * dt,
                      pose.rotation() + speeds.omega * dt)
    