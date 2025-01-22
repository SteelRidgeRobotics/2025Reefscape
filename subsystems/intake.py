from enum import auto, Enum

from subsystems import StateSubsystem
from wpilib import SmartDashboard
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityDutyCycle
from constants import Constants

class Intake(StateSubsystem):

    class SubsystemState(Enum):
        DEFAULT = auto()
        INTAKING = auto()
        OUTPUTING = auto()

    def __init__(self) -> None:

        super().__init__("Intake")
    
        self._subsystem_state = self.SubsystemState.DEFAULT

        self.intakeMotor = TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID)

    def periodic(self):
        return super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:

        # this subsytem is separated from the superstructure
        match desired_state:

            case self.SubsystemState.DEFAULT:
                self.intakeMotor.set_control(VelocityDutyCycle(0))

            case self.SubsystemState.INTAKING:
                self.intakeMotor.set_control(VelocityDutyCycle(Constants.IntakeConstants.INTAKE_SPEED))

            case self.SubsystemState.OUTPUTING:
                self.intakeMotor.set_control(VelocityDutyCycle(Constants.IntakeConstants.OUTPUT_SPEED))

        # update information for the state
        self._subsystem_state = desired_state
        SmartDashboard.putString("Intake State", self._subsystem_state.name)