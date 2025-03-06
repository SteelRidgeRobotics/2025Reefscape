from commands2 import Subsystem
from wpimath.geometry import Pose2d

from subsystems.climber import ClimberSubsystem
from subsystems.elevator import ElevatorSubsystem
from subsystems.pivot import PivotSubsystem


class RobotState(Subsystem):

    starting_pose: Pose2d | None = None

    def __init__(self, pivot: PivotSubsystem, elevator: ElevatorSubsystem, climber: ClimberSubsystem):
        super().__init__()
        self._pivot = pivot
        self._elevator = elevator
        self._climber = climber
