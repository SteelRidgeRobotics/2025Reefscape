from enum import Enum

from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX

from wpilib import Servo


from constants import Constants
from subsystems import StateSubsystem


class ClimberSubsystem(StateSubsystem):
    """
    The ClimberSubsystem is responsible for controlling the robot's climber mechanism.
    """

    class SubsystemState(Enum):
        STOP = 1
        CLIMB_POSITIVE = 2
        CLIMB_NEGATIVE = 3

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.ClimberConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ClimberConstants.GEAR_RATIO))
                     )

    def __init__(self) -> None:
        super().__init__("Climber")

        self.climbServo = Servo(0)
        self._climb_motor = TalonFX(Constants.CanIDs.CLIMB_TALON)
        self._climb_motor.configurator.apply(self._motor_config)
        
        self._climb_request = DutyCycleOut(0)

    def periodic(self):
        super().periodic()

    def _handle_desired_state(self) -> None:
        match self._subsystem_state:
            case self.SubsystemState.STOP:
                self._climb_request.output = 0
                self.climbServo.setAngle(180)
            case self.SubsystemState.CLIMB_POSITIVE:
                self._climb_request.output = 0.5
                self.climbServo.setAngle(0)
            case self.SubsystemState.CLIMB_NEGATIVE:
                self._climb_request.output = -0.5
                self.climbServo.setAngle(0)

        self._climb_motor.set_control(self._climb_request)