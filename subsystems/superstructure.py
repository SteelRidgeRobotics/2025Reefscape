from enum import auto, Enum

from commands2 import Command, Subsystem, cmd
from wpilib import DriverStation, SmartDashboard

from subsystems.swerve import SwerveSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.elevator import ElevatorSubsystem

class Superstructure(Subsystem):
        
    class Goal(Enum):
        DEFAULT = auto()
        L4_SCORING = auto()
        L3_SCORING = auto()
        L2_SCORING = auto()
        L1_SCORING = auto()
        L2_ALGAE_INTAKE = auto()
        L3_ALGAE_INTAKE = auto()
        ALGAE_SCORING_PROCESSOR = auto()
        ALGAE_SCORING_NET = auto()
        FUNNEL_INTAKE = auto()
        GROUND_INTAKE = auto()
        
    def __init__(self, drivetrain: SwerveSubsystem, pivot: PivotSubsystem, elevator: ElevatorSubsystem) -> None:
        super().__init__()

        self.drivetrain = drivetrain
        self.pivot = pivot
        self.elevator = elevator
        
        self._goal = self.Goal.DEFAULT
        self._last_goal = self.Goal.DEFAULT

    def periodic(self):
        if DriverStation.isDisabled():
            default_command = self.set_goal_command(self.Goal.DEFAULT)
            default_command.addRequirements(self)
            self.setDefaultCommand(default_command)

        self._last_goal = self._goal
        SmartDashboard.putString("Superstructure Goal", self._goal.name)
        
        match self._goal:
            case self.Goal.DEFAULT:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.STOW)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.DEFAULT)

            case self.Goal.L4_SCORING:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.HIGH_SCORING)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.L4)

            case self.Goal.L3_SCORING:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.MID_SCORING)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.L3)

            case self.Goal.L2_SCORING:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.MID_SCORING)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.L2)

            case self.Goal.L1_SCORING:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.LOW_SCORING)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.L1)

            case self.Goal.FUNNEL_INTAKE:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.FUNNEL_INTAKE)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.DEFAULT)

            case self.Goal.GROUND_INTAKE:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.GROUND_INTAKE)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.DEFAULT)

            case self.Goal.ALGAE_SCORING_NET:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.NET_SCORING)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.NET)

            case self.Goal.ALGAE_SCORING_PROCESSOR:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.PROCESSOR_SCORING)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.DEFAULT)

            case self.Goal.L2_ALGAE_INTAKE:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.ALGAE_INTAKE)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.L2_ALGAE)

            case self.Goal.L3_ALGAE_INTAKE:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.ALGAE_INTAKE)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.L3_ALGAE)

    def _set_goal(self, goal: Goal) -> None:
        if goal is self._goal:
            return
        self._goal = goal
        
    def set_goal_command(self, goal: Goal) -> Command:
        return cmd.startEnd(
            lambda: self._set_goal(goal),
            lambda: self._set_goal(self.Goal.DEFAULT),
            self
        )
