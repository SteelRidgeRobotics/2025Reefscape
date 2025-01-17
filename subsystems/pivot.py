from enum import auto, Enum

from subsystems import StateSubsystem
from wpilib import SmartDashboard
from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionDutyCycle
from constants import PivotConstants

class Pivot(StateSubsystem):

    class SubsystemState(Enum):
        STOW = auto()
        GROUND_INTAKE = auto()
        FUNNEL_INTAKE = auto()
        HIGH_SCORING = auto()
        MID_SCORING = auto()
        LOW_SCORING = auto()

    def __init__(self) -> None:

        super().__init__("Pivot")
    
        self._subsystem_state = self.SubsystemState.STOW

        self.pivotMotor = TalonFX(PivotConstants.PIVOT_MOTOR_ID)

    def periodic(self):
        return super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:

        # move motor accordingly to set state in superstructure
        match desired_state:

            case self.SubsystemState.STOW:
                self.pivotMotor.set_control(PositionDutyCycle(PivotConstants.STOW_ANGLE))

            case self.SubsystemState.GROUND_INTAKE:
                self.pivotMotor.set_control(PositionDutyCycle(PivotConstants.GROUND_INTAKE_ANGLE))

            case self.SubsystemState.FUNNEL_INTAKE:
                self.pivotMotor.set_control(PositionDutyCycle(PivotConstants.FUNNEL_INTAKE_ANGLE))

            case self.SubsystemState.HIGH_SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(PivotConstants.HIGH_SCORING_ANGLE))

            case self.SubsystemState.MID_SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(PivotConstants.MID_SCORING_ANGLE))

            case self.SubsystemState.LOW_SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(PivotConstants.LOW_SCORING_ANGLE))

        # update information for the state
        self._subsystem_state = desired_state
        SmartDashboard.putString("Pivot State", self._subsystem_state.name)