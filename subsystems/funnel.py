from enum import auto, Enum


from phoenix6 import utils, BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration, MotionMagicConfigs
from phoenix6.controls import DutyCycleOut, MotionMagicDutyCycle
from phoenix6.hardware import TalonFX
from phoenix6.signals import InvertedValue, FeedbackSensorSourceValue, NeutralModeValue
from wpilib import DriverStation
from wpimath.filter import Debouncer
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


class FunnelPivotSubsystem(StateSubsystem):

    class SubsystemState(Enum):

        UP = auto()
        DOWN = auto()

    _master_config = TalonFXConfiguration()
    (_master_config.feedback
     .with_rotor_to_sensor_ratio(Constants.FunnelConstants.GEAR_RATIO)
     .with_feedback_sensor_source(FeedbackSensorSourceValue.FUSED_CANCODER)
    )
    _master_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
    _master_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
    _master_config.with_slot0(Constants.PivotConstants.GAINS)
    _master_config.with_motion_magic(MotionMagicConfigs().with_motion_magic_cruise_velocity(Constants.FunnelConstants.CRUSIE_VELOCITY).with_motion_magic_acceleration(Constants.FunnelConstants.MM_ACCELERATION))

    def __init__(self) -> None:
        super().__init__("Funnel")
        self._master_motor = TalonFX(Constants.CanIDs.FUNNEL_PIVOT_TALON)

        self._master_motor.configurator.apply(self._master_config)

        self._add_talon_sim_model(self._master_motor, DCMotor.krakenX60FOC(2), Constants.PivotConstants.GEAR_RATIO)

        self._at_setpoint_debounce = Debouncer(0.1, Debouncer.DebounceType.kRising)
        self._at_setpoint = True

        self._position_request = MotionMagicDutyCycle(0)
        self._brake_request = DutyCycleOut(0)

        self._master_motor.set_position(self._encoder.get_position().value)

    def periodic(self):
        super().periodic()

        latency_compensated_position = BaseStatusSignal.get_latency_compensated_value(
            self._master_motor.get_position(), self._master_motor.get_velocity()
        )
        self._at_setpoint = self._at_setpoint_debounce.calculate(abs(latency_compensated_position - self._position_request.position) <= Constants.FunnelConstants.SETPOINT_TOLERANCE)

        if utils.is_simulation():
            talon_sim = self._sim_models[0][0]

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if DriverStation.isTest() or self.is_frozen():
            return

        match desired_state:
            case self.SubsystemState.UP:
                self._position_request.position = Constants.FunnelConstants.CORAL_STATION_POSITION

            case self.SubsystemState.DOWN:
                self._position_request.position = Constants.FunnelConstants.STOWED_POSITION


    def is_at_setpoint(self) -> bool:
        return self._at_setpoint

    def get_angle(self) -> float:
        """Returns the current angle of the pivot, in degrees."""
        return self._master_motor.get_position().value * 360
