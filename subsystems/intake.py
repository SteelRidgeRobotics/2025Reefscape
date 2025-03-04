from enum import auto, Enum

import commands2.cmd
from commands2 import Command, cmd
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs
from phoenix6.controls import VelocityDutyCycle, DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue

from constants import Constants
from subsystems import StateSubsystem


class IntakeSubsystem(StateSubsystem):
    """
    The IntakeSubsystem is responsible for controlling the end effector's compliant wheels.
    """

    class SubsystemState(Enum):
        HOLD = auto()
        CORAL_INTAKE = auto()
        FUNNEL_INTAKE = auto()
        CORAL_OUTPUT = auto()
        ALGAE_INTAKE = auto()
        ALGAE_OUTPUT = auto()

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.IntakeConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ElevatorConstants.GEAR_RATIO))
                     )

    _state_configs: dict[SubsystemState, tuple[int, bool]] = {
        SubsystemState.HOLD: (0, False),
        SubsystemState.CORAL_INTAKE: (Constants.IntakeConstants.CORAL_INTAKE_SPEED, False),
        SubsystemState.FUNNEL_INTAKE: (Constants.IntakeConstants.FUNNEL_INTAKE_SPEED, False),
        SubsystemState.CORAL_OUTPUT: (Constants.IntakeConstants.CORAL_OUTPUT_SPEED, True),
        SubsystemState.ALGAE_INTAKE: (Constants.IntakeConstants.ALGAE_INTAKE_SPEED, False),
        SubsystemState.ALGAE_OUTPUT: (Constants.IntakeConstants.ALGAE_OUTPUT_SPEED, True),
    }

    def __init__(self) -> None:
        super().__init__("Intake", self.SubsystemState.HOLD)

        self._intake_motor = TalonFX(Constants.CanIDs.INTAKE_TALON)
        self._intake_motor.configurator.apply(self._motor_config)

        self._velocity_request = DutyCycleOut(0)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        output, ignore_limits = self._state_configs.get(desired_state, (0, False))

        self._velocity_request.output = output
        self._velocity_request.ignore_hardware_limits = ignore_limits

        self._intake_motor.set_control(self._velocity_request)

    def set_desired_state_command(self, state: SubsystemState) -> Command:
        return cmd.runOnce(lambda: self.set_desired_state(state), self)