import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints, PathPlannerPath
from phoenix6 import SignalLogger, swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from generated.tuner_constants import TunerConstants
from robot_state import RobotState
from subsystems.superstructure import Superstructure

from subsystems.climber import ClimberSubsystem
from subsystems.pivot import Pivot
from subsystems.intake import Intake


class RobotContainer:

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            1
        )  # 3/4 of a rotation per second max angular velocity

        self._driver_controller = commands2.button.CommandXboxController(0)
        self._function_controller = commands2.button.CommandXboxController(1)
        self.path_constraints = PathConstraints(1, 1, 1, 1, unlimited=False)
        self.trigger_margin = .75

        self.drivetrain = TunerConstants.create_drivetrain()

        self.climber = ClimberSubsystem()
        self.superstructure = Superstructure(self.drivetrain)

        self.pivot = Pivot()
        self.intake = Intake()

        self.superstructure = Superstructure(self.drivetrain, self.pivot)

        self._robot_state = RobotState(self.drivetrain)

        # Setting up bindings for necessary control of the swerve drive platform
        self._field_centric = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.01)
            .with_rotational_deadband(
                self._max_angular_rate * 0.01
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._robot_centric = (
            swerve.requests.RobotCentric()
            .with_deadband(self._max_speed * 0.01)
            .with_rotational_deadband(
                self._max_angular_rate * 0.01
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Auto Chooser")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Configure the button bindings
        self.configure_button_bindings()

    def configure_button_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self._field_centric.with_velocity_x(
                        -self._driver_controller.getLeftY() * self._max_speed
                    )
                    .with_velocity_y(
                        -self._driver_controller.getLeftX() * self._max_speed
                    )
                    .with_rotational_rate(
                        -self._driver_controller.getRightX() * self._max_angular_rate
                    )
                )
            )
        )

        self._driver_controller.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._driver_controller.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._driver_controller.getLeftY(), -self._driver_controller.getLeftX())
                )
            )
        )

        (self._driver_controller.back() & self._driver_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._driver_controller.back() & self._driver_controller.x()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._driver_controller.start() & self._driver_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._driver_controller.start() & self._driver_controller.x()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )

        self._driver_controller.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        path_state = "DEFAULT"
        if self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin and self._driver_controller.getRightTriggerAxis() >= self.trigger_margin:
            path_state = "CORALSTATION"
        elif self._driver_controller.getLeftTriggerAxis() >= self.trigger_margin:
            path_state = "LEFT"
        elif self._driver_controller.getRightTriggerAxis() >= self.trigger_margin:
            path_state = "RIGHT"


        match path_state:
            case "LEFT":
                self._driver_controller.y().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral A"), self.path_constraints)
                )
                self._driver_controller.x().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral C"), self.path_constraints)
                )
                self._driver_controller.a().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral E"), self.path_constraints)
                )
                self._driver_controller.b().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral G"), self.path_constraints)
                )
                self._driver_controller.rightBumper().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral I"), self.path_constraints)
                )
                self._driver_controller.leftBumper().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral K"), self.path_constraints)
                )
            case "RIGHT":
                self._driver_controller.y().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral B"), self.path_constraints)
                )
                self._driver_controller.x().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral D"), self.path_constraints)
                )
                self._driver_controller.a().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral F"), self.path_constraints)
                )
                self._driver_controller.b().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral H"), self.path_constraints)
                )
                self._driver_controller.b().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral J"), self.path_constraints)
                )
                self._driver_controller.leftBumper().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral L"), self.path_constraints)
                )
            case "CORALSTATION":
                self._driver_controller.leftBumper().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral Station 1"), self.path_constraints)
                )
                self._driver_controller.rightBumper().whileTrue(
                    AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("Coral Station 2"), self.path_constraints)
                )
            case _:
                pass

        

        self._driver_controller.rightBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._robot_centric.with_velocity_x(
                        -self._driver_controller.getLeftY() * self._max_speed
                    )
                    .with_velocity_y(
                        -self._driver_controller.getLeftX() * self._max_speed
                    )
                    .with_rotational_rate(
                        -self._driver_controller.getRightX() * self._max_angular_rate
                    )
                )
            )
        )
        self._function_controller.y().whileTrue(
            self.climber.set_desired_state_command(self.climber.SubsystemState.CLIMB_POSITIVE)).onFalse(
            self.climber.set_desired_state_command(self.climber.SubsystemState.STOP)
        )
        
        self._function_controller.x().whileTrue(
            self.climber.set_desired_state_command(self.climber.SubsystemState.CLIMB_NEGATIVE)).onFalse(
            self.climber.set_desired_state_command(self.climber.SubsystemState.STOP)
        )
        self.drivetrain.register_telemetry(
            lambda state: self._robot_state.log_swerve_state(state)
        )

    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()