import commands2
import commands2.button
from commands2 import cmd
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder, NamedCommands, PathPlannerAuto
from phoenix6 import SignalLogger, swerve, utils
from wpilib import DriverStation, SmartDashboard, DataLogManager
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.units import rotationsToRadians

from constants import Constants
from generated.tuner_constants import TunerConstants
from robot_state import RobotState
from subsystems.climber import ClimberSubsystem
from subsystems.elevator import ElevatorSubsystem
from subsystems.funnel import FunnelSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.superstructure import Superstructure
from subsystems.vision import VisionSubsystem


def flip_pose_if_on_red_alliance(pose: Pose2d) -> Pose2d:
    if (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed:
        flipped_x = Constants.apriltag_layout.getFieldLength() - pose.X()
        flipped_y = Constants.apriltag_layout.getFieldWidth() - pose.Y()
        flipped_rotation = Rotation2d(pose.rotation().radians()) + Rotation2d.fromDegrees(180)
        return Pose2d(flipped_x, flipped_y, flipped_rotation)
    return pose


class RobotContainer:
    def __init__(self) -> None:
        self._max_speed = TunerConstants.speed_at_12_volts
        self._max_angular_rate = rotationsToRadians(1)

        self._driver_controller = commands2.button.CommandXboxController(0)
        self._function_controller = commands2.button.CommandXboxController(1)
        self.drivetrain = TunerConstants.create_drivetrain()

        self.climber = ClimberSubsystem()
        self.pivot = PivotSubsystem()
        self.intake = IntakeSubsystem()
        self.elevator = ElevatorSubsystem()
        self.funnel = FunnelSubsystem()
        self.vision = VisionSubsystem(
            self.drivetrain,
            Constants.VisionConstants.FRONT_RIGHT,
            Constants.VisionConstants.FRONT_CENTER,
            Constants.VisionConstants.FRONT_LEFT,
            Constants.VisionConstants.BACK_CENTER,
        )

        self.robot_state = RobotState(self.drivetrain, self.pivot, self.elevator)
        self.drivetrain.register_telemetry(
            lambda state: self.robot_state.log_swerve_state(state)
        )
        self.superstructure = Superstructure(
            self.drivetrain, self.pivot, self.elevator, self.funnel, self.vision
        )

        self._setup_swerve_requests()
        self._pathplanner_setup()
        self._setup_controller_bindings()

    def _pathplanner_setup(self):
        for goal in Superstructure.Goal:
            name = goal.name.title().replace("_", " ")
            NamedCommands.registerCommand(
                name,
                self.superstructure.set_goal_command(goal),
            )
            DataLogManager.log(f"Added PathPlanner NamedCommand: '{name}'")

        for state in IntakeSubsystem.SubsystemState:
            name = state.name.title().replace("_", " ")
            NamedCommands.registerCommand(
                name,
                self.intake.set_desired_state_command(state),
            )
            DataLogManager.log(f"Added PathPlanner NamedCommand: '{name}'")

        self._auto_chooser = AutoBuilder.buildAutoChooser("Auto Chooser")
        self._auto_chooser.onChange(
            lambda auto: self.drivetrain.reset_pose(flip_pose_if_on_red_alliance(auto._startingPose)) if utils.is_simulation() and isinstance(auto, PathPlannerAuto) else None
        )
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

    def _setup_swerve_requests(self):
        common_settings = lambda req: req.with_deadband(self._max_speed * 0.01).with_rotational_deadband(self._max_angular_rate * 0.01).with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self._field_centric = common_settings(swerve.requests.FieldCentric())
        self._robot_centric = common_settings(swerve.requests.RobotCentric())
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

    def _setup_controller_bindings(self) -> None:
        hid = self._driver_controller.getHID()
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: self._field_centric
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
            )
        )

        self._driver_controller.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._driver_controller.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(Rotation2d(-hid.getLeftY(), -hid.getLeftX()))
            )
        )

        self._driver_controller.leftBumper().onTrue(self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric()))

        self._setup_sysid_bindings(
            self._driver_controller, self.drivetrain,
            self._driver_controller.y(), self._driver_controller.a()
        )

        self._setup_sysid_bindings(
            self._function_controller, self.elevator,
            self._function_controller.y(), self._function_controller.a()
        )

        self._setup_sysid_bindings(
            self._function_controller, self.pivot,
            self._function_controller.b(), self._function_controller.x()
        )

        goal_bindings = {
            self._function_controller.y(): self.superstructure.Goal.L4_SCORING,
            self._function_controller.x(): self.superstructure.Goal.L3_SCORING,
            self._function_controller.b(): self.superstructure.Goal.L2_SCORING,
            self._function_controller.a(): self.superstructure.Goal.L1_SCORING,
            self._function_controller.leftStick(): self.superstructure.Goal.DEFAULT,
        }

        for button, goal in goal_bindings.items():
            button.onTrue(self.superstructure.set_goal_command(goal))

        self._function_controller.leftBumper().whileTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.FUNNEL_INTAKE),
                self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_INTAKE),
            )
        )

        (self._function_controller.leftBumper() & self._function_controller.back()).whileTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.GROUND_INTAKE),
                self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_INTAKE),
            )
        )

        self._function_controller.rightBumper().whileTrue(
            self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_OUTPUT)
        ).onFalse(
            self.intake.set_desired_state_command(self.intake.SubsystemState.DEFAULT)
        )

    def _setup_sysid_bindings(self, controller, subsystem, forward_btn, reverse_btn):
        forward_dynamic = subsystem.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        reverse_dynamic = subsystem.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        forward_quasistatic = subsystem.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        reverse_quasistatic = subsystem.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)

        # Dynamic Tests
        forward_btn.onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(forward_dynamic.onlyIf(lambda: not DriverStation.isFMSAttached() and DriverStation.isTest()))
        reverse_btn.onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(reverse_dynamic.onlyIf(lambda: not DriverStation.isFMSAttached() and DriverStation.isTest()))

        # Quasistatic Tests (POV Up for forward, POV Down for reverse)
        controller.back().and_(forward_btn).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(forward_quasistatic.onlyIf(lambda: not DriverStation.isFMSAttached() and DriverStation.isTest()))
        controller.back().and_(reverse_btn).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(reverse_quasistatic.onlyIf(lambda: not DriverStation.isFMSAttached() and DriverStation.isTest()))

    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()
