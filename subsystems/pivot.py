from enum import auto, Enum

from subsystems import StateSubsystem
from wpilib import SmartDashboard
from phoenix6.hardware import TalonFX
from phoenix6.controls import PositionDutyCycle
from constants import Constants

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

        self.pivotMotor = TalonFX(Constants.MotorIDs.PIVOT_MOTOR)

    def periodic(self):
        return super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:

        # move motor accordingly to set state in superstructure
        match desired_state:

            case self.SubsystemState.STOW:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.STOW_ANGLE))

            case self.SubsystemState.GROUND_INTAKE:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.GROUND_INTAKE_ANGLE))

            case self.SubsystemState.FUNNEL_INTAKE:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.FUNNEL_INTAKE_ANGLE))

            case self.SubsystemState.HIGH_SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.HIGH_SCORING_ANGLE))

            case self.SubsystemState.MID_SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.MID_SCORING_ANGLE))

            case self.SubsystemState.LOW_SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.LOW_SCORING_ANGLE))

        # update information for the state
        self._subsystem_state = desired_state
        SmartDashboard.putString("Pivot State", self._subsystem_state.name)