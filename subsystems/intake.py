from enum import auto, Enum

from phoenix6.configs import TalonFXConfiguration

from subsystems import StateSubsystem
from wpilib import SmartDashboard
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityDutyCycle
from constants import Constants

class IntakeSubsystem(StateSubsystem):
    """
    The IntakeSubsystem is responsible for controlling the end effector's compliant wheels.
    It uses a VelocityDutyCycle request to control the speed of the wheels.
    """

    class SubsystemState(Enum):
        DEFAULT = auto()
        INTAKING = auto()
        OUTPUTTING = auto()

    def __init__(self) -> None:

        super().__init__("Intake")
    
        self._subsystem_state = self.SubsystemState.DEFAULT

        self.intakeMotor = TalonFX(Constants.MotorIDs.INTAKE_MOTOR)
        intake_config = TalonFXConfiguration()
        intake_config.with_slot0(Constants.IntakeConstants.GAINS)
        intake_config.feedback.with_sensor_to_mechanism_ratio(Constants.IntakeConstants.GEAR_RATIO)
        self.intakeMotor.configurator.apply(intake_config)

    def periodic(self):
        super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:

        # This subsystem is separated from the superstructure
        match desired_state:

            case self.SubsystemState.DEFAULT:
                self.intakeMotor.set_control(VelocityDutyCycle(0))

            case self.SubsystemState.INTAKING:
                self.intakeMotor.set_control(VelocityDutyCycle(Constants.IntakeConstants.INTAKE_SPEED))

            case self.SubsystemState.OUTPUTTING:
                self.intakeMotor.set_control(VelocityDutyCycle(Constants.IntakeConstants.OUTPUT_SPEED))

        # Update information for the state
        self._subsystem_state = desired_state
        SmartDashboard.putString("Intake State", self._subsystem_state.name)