import commands2
import commands2.button
from commands2 import cmd
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder, NamedCommands
from phoenix6 import SignalLogger, swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
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

        self._register_named_commands()
        self._setup_swerve_requests()
        self._setup_auto_chooser()
        self._setup_controller_bindings()

    def _register_named_commands(self):
        for goal in Superstructure.Goal:
            NamedCommands.registerCommand(
                goal.name.replace("_", " "),
                self.superstructure.set_goal_command(goal),
            )

        for state in IntakeSubsystem.SubsystemState:
            NamedCommands.registerCommand(
                state.name.replace("_", " "),
                self.intake.set_desired_state_command(state),
            )

    def _setup_swerve_requests(self):
        common_settings = lambda req: req.with_deadband(self._max_speed * 0.01).with_rotational_deadband(self._max_angular_rate * 0.01).with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self._field_centric = common_settings(swerve.requests.FieldCentric())
        self._robot_centric = common_settings(swerve.requests.RobotCentric())
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

    def _setup_auto_chooser(self):
        self._auto_chooser = AutoBuilder.buildAutoChooser("Auto Chooser")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

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

        self._setup_signal_logging_bindings(self._driver_controller, self.drivetrain)
        self._setup_signal_logging_bindings(self._function_controller, self.elevator, self.pivot)

        goal_bindings = {
            self._function_controller.y(): self.superstructure.Goal.L4_SCORING,
            self._function_controller.x(): self.superstructure.Goal.L3_SCORING,
            self._function_controller.b(): self.superstructure.Goal.L2_SCORING,
            self._function_controller.a(): self.superstructure.Goal.L1_SCORING,
        }

        for button, goal in goal_bindings.items():
            button.onTrue(self.superstructure.set_goal_command(goal))

        self._function_controller.leftBumper().whileTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.FUNNEL_INTAKE),
                self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_INTAKING),
            )
        )

        (self._function_controller.leftBumper() & self._function_controller.back()).whileTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.GROUND_INTAKE),
                self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_INTAKING),
            )
        )

        self._function_controller.rightBumper().whileTrue(
            self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_OUTPUTTING)
        ).onFalse(
            self.intake.set_desired_state_command(self.intake.SubsystemState.DEFAULT)
        )

    def _setup_signal_logging_bindings(self, controller, *subsystems):
        for direction, method in [(SysIdRoutine.Direction.kForward, "sys_id_dynamic"), (SysIdRoutine.Direction.kReverse, "sys_id_quasistatic")]:
            for btn, sub in zip([controller.y(), controller.a()], subsystems):
                (controller.povUp() & btn).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(getattr(sub, method)(direction).onlyIf(lambda: not DriverStation.isFMSAttached()))

    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()
