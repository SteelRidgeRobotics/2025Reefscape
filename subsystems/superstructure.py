from enum import auto, Enum

from commands2 import Command, Subsystem, cmd
from wpilib import DriverStation, SmartDashboard

from subsystems.elevator import ElevatorSubsystem
from subsystems.funnel import FunnelSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.swerve import SwerveSubsystem
from subsystems.vision import VisionSubsystem


class Superstructure(Subsystem):
    """
    The Superstructure is in charge of handling all subsystems to ensure no conflicts between them.
    """

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

    # Map each goal to each subsystem state to reduce code complexion
    _goal_to_states = {
        Goal.DEFAULT: (PivotSubsystem.SubsystemState.STOW, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L4_SCORING: (PivotSubsystem.SubsystemState.HIGH_SCORING, ElevatorSubsystem.SubsystemState.L4, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L3_SCORING: (PivotSubsystem.SubsystemState.MID_SCORING, ElevatorSubsystem.SubsystemState.L3, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L2_SCORING: (PivotSubsystem.SubsystemState.MID_SCORING, ElevatorSubsystem.SubsystemState.L2, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L1_SCORING: (PivotSubsystem.SubsystemState.LOW_SCORING, ElevatorSubsystem.SubsystemState.L1, FunnelSubsystem.SubsystemState.DOWN),
        Goal.FUNNEL_INTAKE: (PivotSubsystem.SubsystemState.FUNNEL_INTAKE, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.UP),
        Goal.GROUND_INTAKE: (PivotSubsystem.SubsystemState.GROUND_INTAKE, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN),
        Goal.ALGAE_SCORING_NET: (PivotSubsystem.SubsystemState.NET_SCORING, ElevatorSubsystem.SubsystemState.NET, FunnelSubsystem.SubsystemState.DOWN),
        Goal.ALGAE_SCORING_PROCESSOR: (PivotSubsystem.SubsystemState.PROCESSOR_SCORING, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L2_ALGAE_INTAKE: (PivotSubsystem.SubsystemState.ALGAE_INTAKE, ElevatorSubsystem.SubsystemState.L2_ALGAE, FunnelSubsystem.SubsystemState.DOWN),
        Goal.L3_ALGAE_INTAKE: (PivotSubsystem.SubsystemState.ALGAE_INTAKE, ElevatorSubsystem.SubsystemState.L3_ALGAE, FunnelSubsystem.SubsystemState.DOWN),
    }
        
    def __init__(self, drivetrain: SwerveSubsystem, pivot: PivotSubsystem, elevator: ElevatorSubsystem, funnel: FunnelSubsystem, vision: VisionSubsystem) -> None:
        """
        Constructs the superstructure using instance of each subsystem.

        :param drivetrain: Swerve drive base
        :type drivetrain: SwerveSubsystem
        :param pivot: Pivot that rotates our intake
        :type pivot: PivotSubsystem
        :param elevator: Elevator that moves the intake up and down
        :type elevator: ElevatorSubsystem
        :param funnel: Pivot for funnel structure
        :type funnel: FunnelSubsystem
        :param vision: Handles all vision estimates
        :type vision: VisionSubsystem
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.pivot = pivot
        self.elevator = elevator
        self.funnel = funnel
        self.vision = vision

        self._goal = self.Goal.DEFAULT
        self.set_goal_command(self._goal)
        self._goal_commands = {}
    
    def periodic(self):
        if DriverStation.isTest():
            return

        if self.pivot.is_in_elevator() and not self.elevator.is_at_setpoint():
            # Wait for Pivot to leave elevator
            self.pivot.set_desired_state(PivotSubsystem.SubsystemState.AVOID_ELEVATOR)
            self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.IDLE)
            self.pivot.freeze()
            self.elevator.freeze()

        # Unfreeze subsystems if safe
        if not self.pivot.is_in_elevator() and self.pivot.get_current_state() is PivotSubsystem.SubsystemState.AVOID_ELEVATOR:
            self.elevator.unfreeze()
        if self.elevator.is_at_setpoint() and self.pivot.get_current_state() is PivotSubsystem.SubsystemState.AVOID_ELEVATOR:
            self.pivot.unfreeze()

        # Use MegaTag 1 before the match starts
        if DriverStation.isEnabled():
            self.vision.set_desired_state(VisionSubsystem.SubsystemState.MEGA_TAG_2)

    def _set_goal(self, goal: Goal) -> None:
        # if the goal is already set to this goal, return, otherwise set our goal
        current_goal = self._goal
        if goal is current_goal and not self.elevator.is_frozen() and not self.pivot.is_frozen():
            return
        current_goal = self._goal = goal

        pivot_state, elevator_state, funnel_state = self._goal_to_states.get(current_goal, (None, None, None))
        if pivot_state:
            self.pivot.set_desired_state(pivot_state)
        if elevator_state:
            self.elevator.set_desired_state(elevator_state)
        if funnel_state:
            self.funnel.set_desired_state(funnel_state)

        SmartDashboard.putString("Superstructure Goal", current_goal.name)

    def set_goal_command(self, goal: Goal) -> Command:
        """
        Return a command that sets the superstructure goal to whatever the desired goal is.

        :param goal: The desired goal
        :type goal:  Goal
        :return:     A command that will set the desired goal
        :rtype:      Command
        """
        if goal in self._goal_commands:
            return self._goal_commands[goal]

        command = cmd.startEnd(
            lambda: self._set_goal(goal),
            lambda: self._set_goal(self.Goal.DEFAULT),
            self
        )
        self._goal_commands[goal] = command
        return command
