import commands2
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX


from enum import auto, Enum

from constants import Constants
from subsystems import StateSubsystem

class ClimberSubsystem(StateSubsystem):
    class SubsystemState(Enum):
        STOP = 1
        CLIMB_POSITIVE = 2
        CLIMB_NEGATIVE = 3

    def __init__(self) -> None:
        super().__init__("Climber")

        self.climbMotor = TalonFX(Constants.MotorIDs.CLIMB_MOTOR)
        climbing_config = TalonFXConfiguration()
        climbing_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        climbing_config.feedback.with_sensor_to_mechanism_ratio(Constants.ClimberConstants.GEAR_RATIO)
        climbing_config.with_slot0(Constants.ClimberConstants.GAINS)

    def periodic(self):
        super().periodic()

    def _handle_desired_state(self) -> None:
        match self._subsystem_state:

            case self.SubsystemState.STOP:
                self._climb_stop()
            case self.SubsystemState.CLIMB_POSITIVE:
                self._climb_positive()
            case self.SubsystemState.CLIMB_NEGATIVE:
                self._climb_negative()

    def _climb_positive(self) -> None:
        self.climbMotor.set_control(DutyCycleOut(0.5))

    def _climb_stop(self) -> None:
        self.climbMotor.set_control(DutyCycleOut(0))

    def _climb_negative(self) -> None:
         self.climbMotor.set_control(DutyCycleOut(-0.5))
