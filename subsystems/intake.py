from enum import Enum

from commands2 import Command, cmd
from phoenix6 import utils
from phoenix6.configs import CANrangeConfiguration, TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs, HardwareLimitSwitchConfigs, ProximityParamsConfigs, CurrentLimitsConfigs
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX, CANrange
from phoenix6.signals import NeutralModeValue, ForwardLimitValue, ForwardLimitSourceValue
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


class IntakeSubsystem(StateSubsystem):
    """
    The IntakeSubsystem is responsible for controlling the end effector's compliant wheels.
    """

    class SubsystemState(Enum):
        IDLE = (Constants.IntakeConstants.IDLE_SPEED, False)
        CORAL_INTAKE = (Constants.IntakeConstants.CORAL_INTAKE_SPEED, False)
        FUNNEL_INTAKE = (Constants.IntakeConstants.FUNNEL_INTAKE_SPEED, False)
        CORAL_OUTPUT = (Constants.IntakeConstants.CORAL_OUTPUT_SPEED, True)
        ALGAE_INTAKE = (Constants.IntakeConstants.ALGAE_INTAKE_SPEED, False)
        ALGAE_OUTPUT = (Constants.IntakeConstants.ALGAE_OUTPUT_SPEED, True)
        L1_OUTPUT = (Constants.IntakeConstants.L1_OUTPUT_SPEED, True)

    _canrange_config = (CANrangeConfiguration().with_proximity_params(ProximityParamsConfigs().with_proximity_threshold(Constants.IntakeConstants.PROXIMITY_THRESHOLD)))

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.IntakeConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ElevatorConstants.GEAR_RATIO))
                     .with_current_limits(CurrentLimitsConfigs().with_supply_current_limit_enable(True).with_supply_current_limit(Constants.IntakeConstants.IDLE_SPEED))
                     )

    _limit_switch_config = HardwareLimitSwitchConfigs()
    _limit_switch_config.forward_limit_remote_sensor_id = Constants.CanIDs.INTAKE_CANRANGE
    _limit_switch_config.forward_limit_source = ForwardLimitSourceValue.REMOTE_CANRANGE # Top Limit Switch

    def __init__(self) -> None:
        super().__init__("Intake", self.SubsystemState.IDLE)

        self._intake_motor = TalonFX(Constants.CanIDs.INTAKE_TALON)
        _motor_config = self._motor_config
        if not utils.is_simulation():
            _motor_config.hardware_limit_switch = self._limit_switch_config
        self._intake_motor.configurator.apply(self._motor_config)

        self._add_talon_sim_model(self._intake_motor, DCMotor.falcon500FOC(1), Constants.IntakeConstants.GEAR_RATIO)

        self._canrange = CANrange(Constants.CanIDs.INTAKE_CANRANGE)
        self._canrange.configurator.apply(self._canrange_config)

        self._velocity_request = DutyCycleOut(0)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        output, ignore_limits = desired_state.value
        self._velocity_request.output = output
        self._velocity_request.ignore_hardware_limits = ignore_limits

        self._intake_motor.set_control(self._velocity_request)

    def set_desired_state_command(self, state: SubsystemState) -> Command:
        return cmd.runOnce(lambda: self.set_desired_state(state), self)

    def has_coral(self) -> bool:
        return self._intake_motor.get_forward_limit().value is ForwardLimitValue.CLOSED_TO_GROUND

    