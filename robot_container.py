import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from phoenix6 import SignalLogger, swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from generated.tuner_constants import TunerConstants
from robot_state import RobotState
from subsystems.superstructure import Superstructure
from subsystems.pivot import Pivot


class RobotContainer:

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            1
        )  # 3/4 of a rotation per second max angular velocity

        self._driver_controller = commands2.button.CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()
        self.pivot = Pivot()

        self.superstructure = Superstructure(self.drivetrain, self.pivot)
        self._robot_state = RobotState(self.drivetrain)

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

        # Path follower
        #self._auto_chooser = AutoBuilder.buildAutoChooser("Auto Chooser")
        #SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Configure the button bindings
        self.configure_button_bindings()

    def configure_button_bindings(self) -> None:
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
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

        self.drivetrain.register_telemetry(
            lambda state: self._robot_state.log_swerve_state(state)
        )

    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()
