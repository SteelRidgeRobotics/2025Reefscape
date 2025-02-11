"""
File: swerve.py
Description: The swerve subsystem, plus method to add MegaTag2 pose estimate.
Main Author: Caden Dalley
Co-Authors: Micah Nguyen, James Haddix
"""

from enum import auto, Enum

from commands2 import Command, Subsystem, cmd
from wpilib import DriverStation, SmartDashboard

from subsystems.swerve import SwerveSubsystem
from subsystems.pivot import PivotSubsystem
from subsystems.elevator import ElevatorSubsystem

class Superstructure(Subsystem):
    """
    The Superstructure is in charge of controlling the elevator and the end effector.
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
        
    def __init__(self, drivetrain: SwerveSubsystem, pivot: PivotSubsystem, elevator: ElevatorSubsystem) -> None:
        """
        Constructs the superstructure using instance of each subsystem.

        :param drivetrain: Swerve drive base
        :type drivetrain:  SwerveSubsystem
        :param pivot:      Pivot that rotates our intake
        :type pivot:       PivotSubsystem
        :param elevator:   Elevator that moves the intake up and down
        :type elevator:    ElevatorSubsystem
        """

        # __init__ from Subsystem class
        super().__init__()

        # create subsystems
        self.drivetrain = drivetrain
        self.pivot = pivot
        self.elevator = elevator

        # set up goal with the default goal being the current goal as well as the last goal
        self._goal = self.Goal.DEFAULT
        self._last_goal = self.Goal.DEFAULT
    
    def periodic(self):
        # Do nothing if in test mode
        if DriverStation.isTest():
            return

        # if the driver station is disabled, set the default goal with the requirement of the superstructure.
        if DriverStation.isDisabled():
            default_command = self.set_goal_command(self.Goal.DEFAULT)
            default_command.addRequirements(self)
            self.setDefaultCommand(default_command)

        # periodically the goal becomes the last goal (we save what the last goal was)
        self._last_goal = self._goal

        # put the goal's name on SmartDashboard
        SmartDashboard.putString("Superstructure Goal", self._goal.name)

        # match the goal against each possible goal for the superstructure
        match self._goal:
            
            # the default goal: the pivot is stowed, the elevator is lowered
            case self.Goal.DEFAULT:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.STOW)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.DEFAULT)
            
            # goals for each scoring level on the reef (L1-L4)
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

            # goal for intaking through our funnel: the pivot is oriented for funnel intaking, the elevator is lowered
            case self.Goal.FUNNEL_INTAKE:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.FUNNEL_INTAKE)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.DEFAULT)

            # goal for ground intaking: the pivot is oriented for ground intake, the elevator is lowered
            case self.Goal.GROUND_INTAKE:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.GROUND_INTAKE)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.DEFAULT)

            # goal for scoring in the net: the pivot is oriented for scoring in the net, the elevator is raised to net level
            case self.Goal.ALGAE_SCORING_NET:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.NET_SCORING)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.NET)

            # goal for scoring algae in the processor: pivot oriented for processor, elevator is lowered
            case self.Goal.ALGAE_SCORING_PROCESSOR:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.PROCESSOR_SCORING)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.DEFAULT)

            # goals for intaking algae from L2 and L3
            case self.Goal.L2_ALGAE_INTAKE:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.ALGAE_INTAKE)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.L2_ALGAE)

            case self.Goal.L3_ALGAE_INTAKE:
                self.pivot.set_desired_state(PivotSubsystem.SubsystemState.ALGAE_INTAKE)
                self.elevator.set_desired_state(ElevatorSubsystem.SubsystemState.L3_ALGAE)


    def _set_goal(self, goal: Goal) -> None:
        # if the goal is already set to this goal, return, otherwise set our goal
        if goal is self._goal:
            return
        self._goal = goal


    def set_goal_command(self, goal: Goal) -> Command:
        """
        Return a command that sets the superstructure goal to whatever the desired goal is.

        :param goal: The desired goal
        :type goal:  Goal
        :return:     A command that will set the desired goal
        :rtype:      Command
        """
        return cmd.startEnd(
            lambda: self._set_goal(goal),
            lambda: self._set_goal(self.Goal.DEFAULT),
            self
        )