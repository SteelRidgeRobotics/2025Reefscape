from enum import auto, Enum

import commands2.cmd
from commands2 import Command
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs
from phoenix6.controls import VelocityDutyCycle
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue

from constants import Constants
from subsystems import StateSubsystem


class IntakeSubsystem(StateSubsystem):
    """
    The IntakeSubsystem is responsible for controlling the end effector's compliant wheels.
    It uses a VelocityDutyCycle request to control the speed of the wheels.
    """

    class SubsystemState(Enum):
        DEFAULT = auto()
        INTAKING = auto()
        OUTPUTTING = auto()

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.IntakeConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ElevatorConstants.GEAR_RATIO))
                     )

    def __init__(self) -> None:
        super().__init__("Intake")

        self._intake_motor = TalonFX(Constants.MotorIDs.INTAKE_MOTOR)
        self._intake_motor.configurator.apply(self._motor_config)

        self._velocity_request = VelocityDutyCycle(0)

    def periodic(self):
        super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        # TODO: Add beam break sensors to help determine correct state.
        match desired_state:
            case self.SubsystemState.DEFAULT:
                self._velocity_request.velocity = 0
            case self.SubsystemState.INTAKING:
                self._velocity_request.velocity = Constants.IntakeConstants.INTAKE_SPEED
            case self.SubsystemState.OUTPUTTING:
                self._velocity_request.velocity = Constants.IntakeConstants.OUTPUT_SPEED

        self._subsystem_state = desired_state
        self._intake_motor.set_control(self._velocity_request)

    def set_desired_state_command(self, state: SubsystemState) -> Command:
        return commands2.cmd.startEnd(
            lambda: self.set_desired_state(state),
            lambda: self.set_desired_state(self.SubsystemState.DEFAULT),
            self
        )
