import os
from typing import Callable

import commands2
import commands2.button
from commands2 import cmd, InstantCommand
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from phoenix6 import SignalLogger, swerve
from wpilib import DriverStation, SendableChooser, XboxController, SmartDashboard, getDeployDirectory
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.units import rotationsToRadians

from constants import Constants
from generated.tuner_constants import TunerConstants
from subsystems.elevator import ElevatorSubsystem
from subsystems.funnel import FunnelSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.superstructure import Superstructure
from subsystems.climber import ClimberSubsystem
from subsystems.swerve.requests import DriverAssist
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

        self.superstructure = Superstructure(
            self.drivetrain, self.pivot, self.elevator, self.funnel, self.vision, self.climber
        )

        self._setup_swerve_requests()
        self._pathplanner_setup()
        self._setup_controller_bindings()

    def _pathplanner_setup(self):
        # Register NamedCommands
        NamedCommands.registerCommand("Default", self.superstructure.set_goal_command(Superstructure.Goal.DEFAULT))
        NamedCommands.registerCommand("L4 Coral", self.superstructure.set_goal_command(Superstructure.Goal.L4_CORAL))
        NamedCommands.registerCommand("L3 Coral", self.superstructure.set_goal_command(Superstructure.Goal.L3_CORAL))
        NamedCommands.registerCommand("L2 Coral", self.superstructure.set_goal_command(Superstructure.Goal.L2_CORAL))
        NamedCommands.registerCommand("L1 Coral", self.superstructure.set_goal_command(Superstructure.Goal.L1_CORAL))
        NamedCommands.registerCommand("L2 Algae", self.superstructure.set_goal_command(Superstructure.Goal.L2_ALGAE))
        NamedCommands.registerCommand("L3 Algae", self.superstructure.set_goal_command(Superstructure.Goal.L3_ALGAE))
        NamedCommands.registerCommand("Processor", self.superstructure.set_goal_command(Superstructure.Goal.PROCESSOR))
        NamedCommands.registerCommand("Net", self.superstructure.set_goal_command(Superstructure.Goal.NET))
        NamedCommands.registerCommand("Funnel", self.superstructure.set_goal_command(Superstructure.Goal.FUNNEL))
        NamedCommands.registerCommand("Floor", self.superstructure.set_goal_command(Superstructure.Goal.FLOOR))

        NamedCommands.registerCommand("Hold", self.intake.set_desired_state_command(IntakeSubsystem.SubsystemState.HOLD))
        NamedCommands.registerCommand("Coral Intake", self.intake.set_desired_state_command(IntakeSubsystem.SubsystemState.CORAL_INTAKE))
        NamedCommands.registerCommand("Coral Output", self.intake.set_desired_state_command(IntakeSubsystem.SubsystemState.CORAL_OUTPUT))
        NamedCommands.registerCommand("Funnel Intake", self.intake.set_desired_state_command(IntakeSubsystem.SubsystemState.FUNNEL_INTAKE).until(self.intake.has_coral))
        NamedCommands.registerCommand("Algae Intake", self.intake.set_desired_state_command(IntakeSubsystem.SubsystemState.ALGAE_INTAKE))
        NamedCommands.registerCommand("Algae Output", self.intake.set_desired_state_command(IntakeSubsystem.SubsystemState.ALGAE_OUTPUT))

        # Build AutoChooser
        self._auto_chooser = SendableChooser()

        for auto in os.listdir(os.path.join(getDeployDirectory(), 'pathplanner', 'autos')):
            auto = auto.removesuffix(".auto")
            if auto ==".DS_Store":
                continue
            self._auto_chooser.addOption(auto, PathPlannerAuto(auto, False))
            self._auto_chooser.addOption(auto + " (Mirrored)", PathPlannerAuto(auto, True))
        self._auto_chooser.setDefaultOption("None", cmd.none())
        self._auto_chooser.onChange(
            lambda _: self._set_correct_swerve_position()
        )
        # Add basic leave
        self._auto_chooser.addOption("Basic Leave",
            self.drivetrain.apply_request(lambda: self._robot_centric.with_velocity_x(1)).withTimeout(1.0)
        )
        SmartDashboard.putData("Selected Auto", self._auto_chooser)

    def _set_correct_swerve_position(self) -> None:
        chooser_selected = self._auto_chooser.getSelected()
        try:
            self.drivetrain.reset_pose(self._flip_pose_if_needed(chooser_selected._startingPose))
            self.drivetrain.reset_rotation(chooser_selected._startingPose.rotation() + self.drivetrain.get_operator_forward_direction())
        except AttributeError:
            pass

    @staticmethod
    def _flip_pose_if_needed(pose: Pose2d) -> Pose2d:
        if (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed:
            flipped_x = Constants.FIELD_LAYOUT.getFieldLength() - pose.X()
            flipped_y = Constants.FIELD_LAYOUT.getFieldWidth() - pose.Y()
            flipped_rotation = Rotation2d(pose.rotation().radians()) + Rotation2d.fromDegrees(180)
            return Pose2d(flipped_x, flipped_y, flipped_rotation)
        return pose

    def _setup_swerve_requests(self):
        common_settings: Callable[[swerve.requests.SwerveRequest], swerve.requests.SwerveRequest] = lambda req: req.with_deadband(self._max_speed * 0.01).with_rotational_deadband(self._max_angular_rate * 0.01).with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.VELOCITY
        ).with_steer_request_type(swerve.SwerveModule.SteerRequestType.MOTION_MAGIC_EXPO)
        self._field_centric: swerve.requests.FieldCentric = common_settings(swerve.requests.FieldCentric())
        self._robot_centric: swerve.requests.RobotCentric = common_settings(swerve.requests.RobotCentric())

        self._driver_assist: DriverAssist = common_settings(
            DriverAssist()
            .with_translation_pid(Constants.AutoAlignConstants.TRANSLATION_P, Constants.AutoAlignConstants.TRANSLATION_I, Constants.AutoAlignConstants.TRANSLATION_D)
            .with_heading_pid(Constants.AutoAlignConstants.HEADING_P, Constants.AutoAlignConstants.HEADING_I, Constants.AutoAlignConstants.HEADING_D)
            .with_max_distance(Constants.AutoAlignConstants.MAX_DISTANCE)
            .with_elevator_up_function(lambda: not self.elevator.get_current_state() == self.elevator.SubsystemState.DEFAULT)
            )
        
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()

    @staticmethod
    def rumble_command(controller: CommandXboxController, duration: float, intensity: float):
        return cmd.sequence(
            InstantCommand(lambda: controller.setRumble(XboxController.RumbleType.kBothRumble, intensity)),
            cmd.waitSeconds(duration),
            InstantCommand(lambda: controller.setRumble(XboxController.RumbleType.kBothRumble, 0))
        )

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

        
        self._driver_controller.leftBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._robot_centric
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
            )
        )

        self._driver_controller.rightBumper().whileTrue(
            self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_OUTPUT)
        ).onFalse(
            self.intake.set_desired_state_command(self.intake.SubsystemState.HOLD)
        )

        self._driver_controller.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._driver_controller.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(Rotation2d(-hid.getLeftY(), -hid.getLeftX()))
            )
        )

        Trigger(lambda: self._driver_controller.getLeftTriggerAxis() > 0.75).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._driver_assist
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
                .with_fallback(self._field_centric)
                .with_target_pose(self.drivetrain.get_closest_branch(self.drivetrain.BranchSide.LEFT))
            )
        )

        Trigger(lambda: self._driver_controller.getRightTriggerAxis() > 0.75).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._driver_assist
                .with_velocity_x(-hid.getLeftY() * self._max_speed)
                .with_velocity_y(-hid.getLeftX() * self._max_speed)
                .with_rotational_rate(-self._driver_controller.getRightX() * self._max_angular_rate)
                .with_fallback(self._field_centric)
                .with_target_pose(self.drivetrain.get_closest_branch(self.drivetrain.BranchSide.RIGHT))
            )
        )

        self._driver_controller.start().onTrue(self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric()))

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
            self._function_controller.y(): self.superstructure.Goal.L4_CORAL,
            self._function_controller.x(): self.superstructure.Goal.L3_CORAL,
            self._function_controller.b(): self.superstructure.Goal.L2_CORAL,
            self._function_controller.a(): self.superstructure.Goal.DEFAULT,
            self._function_controller.y() & self._function_controller.start(): self.superstructure.Goal.NET,
            self._function_controller.x() & self._function_controller.start(): self.superstructure.Goal.L3_ALGAE,
            self._function_controller.b() & self._function_controller.start(): self.superstructure.Goal.L2_ALGAE,
            self._function_controller.a() & self._function_controller.start(): self.superstructure.Goal.PROCESSOR,
            self._function_controller.leftStick(): self.superstructure.Goal.L1_CORAL
        }

        for button, goal in goal_bindings.items():
            if goal is self.superstructure.Goal.L3_ALGAE or goal is self.superstructure.Goal.NET or goal is self.superstructure.Goal.L2_ALGAE or goal is self.superstructure.Goal.PROCESSOR:
                (button.whileTrue(
                    self.superstructure.set_goal_command(goal)
                    .alongWith(self.intake.set_desired_state_command(self.intake.SubsystemState.ALGAE_INTAKE)))
                    .onFalse(self.intake.set_desired_state_command(self.intake.SubsystemState.ALGAE_HOLD)))
            else:
                button.onTrue(self.superstructure.set_goal_command(goal))

        self._function_controller.leftBumper().onTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.FUNNEL),
                self.intake.set_desired_state_command(self.intake.SubsystemState.FUNNEL_INTAKE),
            )
        ).onFalse(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.DEFAULT),
                self.intake.set_desired_state_command(self.intake.SubsystemState.HOLD)
            )
        )

        (self._function_controller.leftBumper() & self._function_controller.back()).whileTrue(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.FLOOR),
                self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_INTAKE),
            )
        ).onFalse(
            cmd.parallel(
                self.superstructure.set_goal_command(self.superstructure.Goal.DEFAULT),
                self.intake.set_desired_state_command(self.intake.SubsystemState.HOLD),
            )
        )

      
        self._function_controller.povLeft().onTrue(
            cmd.parallel(
                self.climber.set_desired_state_command(self.climber.SubsystemState.CLIMB_OUT),
                self.superstructure.set_goal_command(self.superstructure.Goal.CLIMBING)
            )

        ).onFalse(self.climber.set_desired_state_command(self.climber.SubsystemState.STOP))

        self._function_controller.povRight().onTrue(
            cmd.parallel(
                self.climber.set_desired_state_command(self.climber.SubsystemState.CLIMB_IN),
                self.superstructure.set_goal_command(self.superstructure.Goal.CLIMBING)
            )
        ).onFalse(self.climber.set_desired_state_command(self.climber.SubsystemState.STOP))
    

        self._function_controller.rightBumper().whileTrue(
            self.intake.set_desired_state_command(self.intake.SubsystemState.CORAL_OUTPUT)
        ).onFalse(
            self.intake.set_desired_state_command(self.intake.SubsystemState.HOLD)
        )

        (self._function_controller.rightBumper() & self._function_controller.start()).onTrue(
            self.intake.set_desired_state_command(self.intake.SubsystemState.L1_OUTPUT)
        ).onFalse(
            self.intake.set_desired_state_command(self.intake.SubsystemState.HOLD)
        )

    def _setup_sysid_bindings(self, controller, subsystem, forward_btn, reverse_btn):
        forward_dynamic = subsystem.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        reverse_dynamic = subsystem.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        forward_quasistatic = subsystem.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        reverse_quasistatic = subsystem.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)

        # Dynamic Tests
        forward_btn.whileTrue(forward_dynamic.onlyIf(lambda: not DriverStation.isFMSAttached() and DriverStation.isTest()))
        reverse_btn.whileTrue(reverse_dynamic.onlyIf(lambda: not DriverStation.isFMSAttached() and DriverStation.isTest()))

        # Quasistatic Tests (POV Up for forward, POV Down for reverse)
        controller.back().and_(forward_btn).whileTrue(forward_quasistatic.onlyIf(lambda: not DriverStation.isFMSAttached() and DriverStation.isTest()))
        controller.back().and_(reverse_btn).whileTrue(reverse_quasistatic.onlyIf(lambda: not DriverStation.isFMSAttached() and DriverStation.isTest()))

    def get_autonomous_command(self) -> commands2.Command:
        return self._auto_chooser.getSelected()
