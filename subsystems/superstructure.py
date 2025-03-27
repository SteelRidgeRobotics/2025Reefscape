from enum import auto, Enum
from typing import Optional

from commands2 import Command, Subsystem, cmd
from ntcore import NetworkTableInstance
from phoenix6 import utils
from wpilib import DriverStation, Mechanism2d, Color8Bit, SmartDashboard

from constants import Constants
from subsystems.elevator import ElevatorSubsystem
from subsystems.funnel import FunnelSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.swerve import SwerveSubsystem
from subsystems.vision import VisionSubsystem
from subsystems.climber import ClimberSubsystem


class Superstructure(Subsystem):
    """
    The Superstructure is in charge of handling all subsystems to ensure no conflicts between them.
    """

    class Goal(Enum):
        DEFAULT = auto()
        L4_CORAL = auto()
        L3_CORAL = auto()
        L2_CORAL = auto()
        L1_CORAL = auto()
        L2_ALGAE = auto()
        L3_ALGAE = auto()
        PROCESSOR = auto()
        NET = auto()
        CLIMBING = auto()
        FUNNEL = auto()
        FLOOR = auto()

    # Map each goal to each subsystem state to reduce code complexity
    _goal_to_states: dict[Goal,
            tuple[
                Optional[PivotSubsystem.SubsystemState],
                Optional[ElevatorSubsystem.SubsystemState],
                Optional[FunnelSubsystem.SubsystemState],
                Optional[VisionSubsystem.SubsystemState]
            ]] = {
        Goal.DEFAULT: (PivotSubsystem.SubsystemState.STOW, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.ALL_ESTIMATES),
        Goal.L4_CORAL: (PivotSubsystem.SubsystemState.HIGH_SCORING, ElevatorSubsystem.SubsystemState.L4, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.REEF_ESTIMATES),
        Goal.L3_CORAL: (PivotSubsystem.SubsystemState.L3_CORAL, ElevatorSubsystem.SubsystemState.L3, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.REEF_ESTIMATES),
        Goal.L2_CORAL: (PivotSubsystem.SubsystemState.L2_CORAL, ElevatorSubsystem.SubsystemState.L2, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.REEF_ESTIMATES),
        Goal.L1_CORAL: (PivotSubsystem.SubsystemState.LOW_SCORING, ElevatorSubsystem.SubsystemState.L1, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.REEF_ESTIMATES),
        Goal.L2_ALGAE: (PivotSubsystem.SubsystemState.ALGAE_INTAKE, ElevatorSubsystem.SubsystemState.L2_ALGAE, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.REEF_ESTIMATES),
        Goal.L3_ALGAE: (PivotSubsystem.SubsystemState.ALGAE_INTAKE, ElevatorSubsystem.SubsystemState.L3_ALGAE, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.REEF_ESTIMATES),
        Goal.PROCESSOR: (PivotSubsystem.SubsystemState.PROCESSOR_SCORING, ElevatorSubsystem.SubsystemState.PROCESSOR, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.ALL_ESTIMATES),
        Goal.NET: (PivotSubsystem.SubsystemState.NET_SCORING, ElevatorSubsystem.SubsystemState.NET, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.ALL_ESTIMATES),
        Goal.FUNNEL: (PivotSubsystem.SubsystemState.FUNNEL_INTAKE, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.UP, VisionSubsystem.SubsystemState.ALL_ESTIMATES),
        Goal.FLOOR: (PivotSubsystem.SubsystemState.GROUND_INTAKE, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.ALL_ESTIMATES),
        Goal.CLIMBING: (PivotSubsystem.SubsystemState.AVOID_CLIMBER, ElevatorSubsystem.SubsystemState.DEFAULT, FunnelSubsystem.SubsystemState.DOWN, VisionSubsystem.SubsystemState.NO_ESTIMATES),
    }

    def __init__(self, drivetrain: SwerveSubsystem, pivot: PivotSubsystem, elevator: ElevatorSubsystem, funnel: FunnelSubsystem, vision: VisionSubsystem, climber: ClimberSubsystem) -> None:
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
        self.climber = climber

        self._goal = self.Goal.DEFAULT
        self.set_goal_command(self._goal)

        table = NetworkTableInstance.getDefault().getTable("Superstructure")
        self._current_goal_pub = table.getStringTopic("Current Goal").publish()

        if utils.is_simulation():
            self._superstructure_mechanism = Mechanism2d(1, 5, Color8Bit(0, 0, 105))
            self._superstructure_root = self._superstructure_mechanism.getRoot("Root", 1 / 2, 0.125)
            self._elevator_mech = self._superstructure_root.appendLigament("Elevator", 0.2794, 90, 5, Color8Bit(194, 194, 194))
            self._pivot_mech = self._elevator_mech.appendLigament("Pivot", 0.635, 90, 4, Color8Bit(19, 122, 127))
            SmartDashboard.putData("Superstructure Mechanism", self._superstructure_mechanism)

    def periodic(self):
        if DriverStation.isDisabled():
            return

        # Unfreeze subsystems if safe
        if self.elevator.is_frozen() and not self.pivot.is_in_elevator():
            self.elevator.unfreeze()
            self.elevator.set_desired_state(self._desired_elevator_state)

        # If the elevator reaches its setpoint or the pivot is outside the elevator and can reach its target safely, unfreeze the pivot
        if self.pivot.get_current_state() is PivotSubsystem.SubsystemState.AVOID_ELEVATOR and not self.pivot.is_in_elevator() and (
                self.elevator.is_at_setpoint() or self._desired_pivot_state.value < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE):
            self.pivot.unfreeze()
            self.pivot.set_desired_state(self._desired_pivot_state)

        #If climber motor position is at the top position (1 is the placeholder for what the value would actually be), it will go to the full climb state
        if self.climber._climb_motor.get_position().value > 1 and self.climber.get_current_state() is ClimberSubsystem.SubsystemState.CLIMB_IN:
            self.climber.set_desired_state(ClimberSubsystem.SubsystemState.CLIMB_IN_FULL)

    def simulationPeriodic(self) -> None:
        self._elevator_mech.setLength(self.elevator.get_height())
        self._pivot_mech.setAngle(self.pivot.get_position() * 360 - 90)

    def _set_goal(self, goal: Goal) -> None:
        self._goal = goal

        pivot_state, elevator_state, funnel_state, vision_state = self._goal_to_states.get(goal, (None, None, None, None))
        safety_checks = self._should_enable_safety_checks(pivot_state)
        if pivot_state:
            self._desired_pivot_state = pivot_state
            if safety_checks:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.AVOID_ELEVATOR)
                self.pivot.freeze()
            else:
                self.pivot.set_desired_state(pivot_state)
        if elevator_state:
            self._desired_elevator_state = elevator_state
            if pivot_state and safety_checks:
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.IDLE)
                self.elevator.freeze()
            else:
                self.elevator.set_desired_state(elevator_state)
        if funnel_state:
            self.funnel.set_desired_state(funnel_state)
        if vision_state:
            self.vision.set_desired_state(vision_state)

        self._current_goal_pub.set(goal.name)

    def _should_enable_safety_checks(self, pivot_state: PivotSubsystem.SubsystemState) -> bool:
        """Safety checks are always activated, unless we're already outside the elevator and the new state is also outside the elevator."""
        return not (
                self.pivot.get_current_state().value < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE
                and pivot_state.value < Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE
        )

    def set_goal_command(self, goal: Goal) -> Command:
        """
        Return a command that sets the superstructure goal to whatever the desired goal is.

        :param goal: The desired goal
        :type goal:  Goal
        :return:     A command that will set the desired goal
        :rtype:      Command
        """
        return cmd.runOnce(lambda: self._set_goal(goal), self)
