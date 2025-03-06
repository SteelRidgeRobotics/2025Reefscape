from commands2 import Subsystem
from phoenix6 import utils
from wpilib import SmartDashboard, Mechanism2d, Color8Bit
from wpimath import units
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

        self._climber_mechanism = None
        if utils.is_simulation():
            self._setup_simulation_mechanisms()

    def _setup_simulation_mechanisms(self):
        self._climber_mechanism = Mechanism2d(1, 1)
        self._climber_root = self._climber_mechanism.getRoot("Root", 1/2, 0)
        self._climber_base = self._climber_root.appendLigament("Base", units.inchesToMeters(18.25), 90, 5, Color8Bit(194, 194, 194))
        self._climber_arm = self._climber_base.appendLigament("Arm", units.inchesToMeters(9.424631), 0, 3, Color8Bit(100, 100, 100))
        SmartDashboard.putData("Climber Mechanism", self._climber_mechanism)

    def simulationPeriodic(self) -> None:
        self.update_mechanisms()

    def update_mechanisms(self) -> None:
        if self._climber_mechanism:
            self._climber_arm.setAngle(self._climber.get_position() * 360)
