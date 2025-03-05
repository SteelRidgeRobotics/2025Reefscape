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

        self._superstructure_mechanism = None
        self._climber_mechanism = None
        if utils.is_simulation():
            self._setup_simulation_mechanisms()

    def _setup_simulation_mechanisms(self):
        self._superstructure_mechanism = Mechanism2d(1, 5, Color8Bit(0, 0, 105))
        self._superstructure_root = self._superstructure_mechanism.getRoot("Root", 1 / 2, 0.125)
        self._elevator_mech = self._superstructure_root.appendLigament("Elevator", 0.2794, 90, 5, Color8Bit(194, 194, 194))
        self._pivot_mech = self._elevator_mech.appendLigament("Pivot", 0.635, 90, 4, Color8Bit(19, 122, 127))
        SmartDashboard.putData("Superstructure Mechanism", self._superstructure_mechanism)

        self._climber_mechanism = Mechanism2d(1, 1)
        self._climber_root = self._climber_mechanism.getRoot("Root", 1/2, 0)
        self._climber_base = self._climber_root.appendLigament("Base", units.inchesToMeters(18.25), 90, 5, Color8Bit(194, 194, 194))
        self._climber_arm = self._climber_base.appendLigament("Arm", units.inchesToMeters(9.424631), 0, 3, Color8Bit(100, 100, 100))
        SmartDashboard.putData("Climber Mechanism", self._climber_mechanism)

    def simulationPeriodic(self) -> None:
        self.update_mechanisms()

    def update_mechanisms(self) -> None:
        if self._superstructure_mechanism:
            self._elevator_mech.setLength(self._elevator.get_height())
            self._pivot_mech.setAngle(self._pivot.get_angle() - 90)

        if self._climber_mechanism:
            self._climber_arm.setAngle(self._climber.get_position() * 360)
