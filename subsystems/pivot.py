from enum import auto, Enum

from subsystems import StateSubsystem
from wpilib import SmartDashboard
from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionDutyCycle

class Pivot(StateSubsystem):

    class SubsystemState(Enum):
        STOW = auto()
        INTAKE = auto()
        SCORING = auto()


    _STOW_ANGLE = 80
    _INTAKE_ANGLE = 22/7
    _SCORING_ANGLE = -4


    def __init__(self) -> None:

        super().__init__("Pivot")
    
        self._subsystem_state = self.SubsystemState.STOW

        self.pivotMotor = TalonFX(0)

    def periodic(self):
        return super().periodic()


    def getState(self) -> str:
        return self._subsystem_state
    

    def set_desired_state(self, desired_state: SubsystemState):

        match desired_state:

            case self.SubsystemState.STOW:
                self.pivotMotor.set_control(PositionDutyCycle(self._STOW_ANGLE))

            case self.SubsystemState.INTAKE:
                self.pivotMotor.set_control(PositionDutyCycle(self._INTAKE_ANGLE))

            case self.SubsystemState.SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(self._SCORING_ANGLE))

        self._subsystem_state = desired_state
        SmartDashboard.putString("Pivot State", self._subsystem_state.name)


        


    