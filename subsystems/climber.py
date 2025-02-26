from enum import Enum

from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from wpilib import Servo
from wpimath import units
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


class ClimberSubsystem(StateSubsystem):
    """
    The ClimberSubsystem is responsible for controlling the robot's climber mechanism.
    In order to stay in the air after being disabled, the climber mechanism utilizes a ratchet powered by a servo.
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

    _state_configs: dict[SubsystemState, tuple[int, units.degrees]] = {
        SubsystemState.STOP: (0, 180),
        SubsystemState.CLIMB_POSITIVE: (4, 0),
        SubsystemState.CLIMB_NEGATIVE: (-4, 180),
    }

    def __init__(self) -> None:
        super().__init__("Climber", self.SubsystemState.STOP)

        self._climb_servo = Servo(0)
        self._climb_motor = TalonFX(Constants.CanIDs.CLIMB_TALON)
        self._climb_motor.configurator.apply(self._motor_config)
        self._add_talon_sim_model(self._climb_motor, DCMotor.falcon500FOC(1), Constants.ClimberConstants.GEAR_RATIO)
        
        self._climb_request = VoltageOut(0)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        output, servo_angle = self._state_configs.get(desired_state, (0, 0))
        self._climb_request.output = output
        self._climb_servo.setAngle(servo_angle)

        self._climb_motor.set_control(self._climb_request)

    def get_position(self) -> float:
        return self._climb_motor.get_position().value
