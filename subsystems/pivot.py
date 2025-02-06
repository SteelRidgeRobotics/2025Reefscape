from enum import auto, Enum

from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import PositionDutyCycle
from wpilib import SmartDashboard
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem

class PivotSubsystem(StateSubsystem):

    class SubsystemState(Enum):
        STOW = auto()
        GROUND_INTAKE = auto()
        FUNNEL_INTAKE = auto()
        ALGAE_INTAKE = auto()
        HIGH_SCORING = auto()
        MID_SCORING = auto()
        LOW_SCORING = auto()
        NET_SCORING = auto()
        PROCESSOR_SCORING = auto()

    _master_config = TalonFXConfiguration()
    _master_config.feedback.with_sensor_to_mechanism_ratio(Constants.PivotConstants.GEAR_RATIO)
    _master_config.with_slot0(Constants.PivotConstants.GAINS)

    def __init__(self) -> None:

        super().__init__("Pivot")
    
        self._subsystem_state = self.SubsystemState.STOW

        self._pivot_motor = TalonFX(Constants.MotorIDs.PIVOT_MOTOR)
        self._pivot_motor.configurator.apply(self._master_config)
        self._add_talon_sim_model(self._pivot_motor, DCMotor.krakenX60FOC(2), Constants.PivotConstants.GEAR_RATIO)

    def periodic(self):
        return super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:

        # move motor accordingly to set state in superstructure
        match desired_state:

            case self.SubsystemState.STOW:
                self._pivot_motor.set_control(PositionDutyCycle(Constants.PivotConstants.STOW_ANGLE))

            case self.SubsystemState.GROUND_INTAKE:
                self._pivot_motor.set_control(PositionDutyCycle(Constants.PivotConstants.GROUND_INTAKE_ANGLE))

            case self.SubsystemState.FUNNEL_INTAKE:
                self._pivot_motor.set_control(PositionDutyCycle(Constants.PivotConstants.FUNNEL_INTAKE_ANGLE))

            case self.SubsystemState.HIGH_SCORING:
                self._pivot_motor.set_control(PositionDutyCycle(Constants.PivotConstants.HIGH_SCORING_ANGLE))

            case self.SubsystemState.MID_SCORING:
                self._pivot_motor.set_control(PositionDutyCycle(Constants.PivotConstants.MID_SCORING_ANGLE))

            case self.SubsystemState.LOW_SCORING:
                self._pivot_motor.set_control(PositionDutyCycle(Constants.PivotConstants.LOW_SCORING_ANGLE))

            case self.SubsystemState.NET_SCORING:
                self._pivot_motor.set_control(PositionDutyCycle(Constants.PivotConstants.NET_SCORING_ANGLE))

            case self.SubsystemState.PROCESSOR_SCORING:
                self._pivot_motor.set_control(PositionDutyCycle(Constants.PivotConstants.PROCESSOR_SCORING_ANGLE))

            case self.SubsystemState.ALGAE_INTAKE:
                self._pivot_motor.set_control(PositionDutyCycle(Constants.PivotConstants.ALGAE_INTAKE_ANGLE))

        # update information for the state
        self._subsystem_state = desired_state
        SmartDashboard.putString("Pivot State", self._subsystem_state.name)

    def get_angle(self) -> float:
        """Returns the current angle of the pivot, in degrees."""
        return self._pivot_motor.get_position().value * 360