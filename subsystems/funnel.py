from enum import auto, Enum

from phoenix6.configs import TalonFXConfiguration, MotionMagicConfigs
from phoenix6.controls import DutyCycleOut, MotionMagicDutyCycle
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, FeedbackSensorSourceValue, NeutralModeValue
from wpilib import DriverStation
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


class FunnelSubsystem(StateSubsystem):

    class SubsystemState(Enum):
        UP = auto()
        DOWN = auto()

    _funnel_config = TalonFXConfiguration()
    (_funnel_config.feedback
     .with_rotor_to_sensor_ratio(Constants.FunnelConstants.GEAR_RATIO)
     .with_feedback_sensor_source(FeedbackSensorSourceValue.FUSED_CANCODER)
    )
    _funnel_config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    _funnel_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
    _funnel_config.with_slot0(Constants.FunnelConstants.GAINS)
    _funnel_config.with_motion_magic(MotionMagicConfigs().with_motion_magic_cruise_velocity(Constants.FunnelConstants.CRUISE_VELOCITY).with_motion_magic_acceleration(Constants.FunnelConstants.MM_ACCELERATION))

    def __init__(self) -> None:
        super().__init__("Funnel")

        self._funnel_motor = TalonFX(Constants.CanIDs.FUNNEL_TALON)

        self._funnel_motor.configurator.apply(self._funnel_config)

        self._add_talon_sim_model(self._funnel_motor, DCMotor.falcon500FOC(), Constants.FunnelConstants.GEAR_RATIO)


        self._position_request = MotionMagicDutyCycle(0)
        self._brake_request = DutyCycleOut(0)

    def periodic(self):
        super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if DriverStation.isTest() or self.is_frozen():
            return

        match desired_state:
            case self.SubsystemState.UP:
                self._position_request.position = Constants.FunnelConstants.CORAL_STATION_POSITION

            case self.SubsystemState.DOWN:
                self._position_request.position = Constants.FunnelConstants.STOWED_POSITION
        
        self._funnel_motor.set_control(self._position_request)

        self._subsystem_state = desired_state
