from enum import auto, Enum

from phoenix6 import utils
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from wpilib import Servo, Mechanism2d, Color8Bit, SmartDashboard
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
        STOP = auto()
        CLIMB_IN = auto()
        CLIMB_IN_FULL = auto()
        CLIMB_OUT = auto()

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.ClimberConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ClimberConstants.GEAR_RATIO))
                     )
    _winch_motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.ClimberConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.COAST))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ClimberConstants.GEAR_RATIO))
                     )

    _state_configs: dict[SubsystemState, tuple[int, units.degrees]] = {
        SubsystemState.STOP: (0, Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE),
        SubsystemState.CLIMB_IN: (Constants.ClimberConstants.VOLTAGE_INWARDS, Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE),
        SubsystemState.CLIMB_IN_FULL: (Constants.ClimberConstants.VOLTAGE_INWARDS, Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE),
        SubsystemState.CLIMB_OUT: (Constants.ClimberConstants.CLIMB_STALL_VOLTAGE, Constants.ClimberConstants.SERVO_ENGAGED_ANGLE),
    }

    def __init__(self) -> None:
        super().__init__("Climber", self.SubsystemState.STOP)

        self._climb_servo = Servo(Constants.ClimberConstants.SERVO_PORT)
        self._climb_motor = TalonFX(Constants.CanIDs.CLIMB_TALON)
        self._climb_motor.configurator.apply(self._motor_config)

        self._winch_motor = TalonFX(Constants.CanIDs.WINCH_TALON)
        self._winch_motor.configurator.apply(self._winch_motor_config)

        self._add_talon_sim_model(self._climb_motor, DCMotor.falcon500FOC(1), Constants.ClimberConstants.GEAR_RATIO)
        self._servo_desired_angle_pub = self.get_network_table().getFloatTopic("Servo Desired Angle").publish()
        
        self._climb_request = VoltageOut(0)
        self._winch_request = VoltageOut(0)

        if utils.is_simulation():
            self._climber_mechanism = Mechanism2d(1, 1)
            self._climber_root = self._climber_mechanism.getRoot("Root", 1 / 2, 0)
            self._climber_base = self._climber_root.appendLigament("Base", units.inchesToMeters(18.25), 90, 5, Color8Bit(194, 194, 194))
            self._climber_arm = self._climber_base.appendLigament("Arm", units.inchesToMeters(9.424631), 0, 3, Color8Bit(100, 100, 100))
            SmartDashboard.putData("Climber Mechanism", self._climber_mechanism)

    def periodic(self):
        super().periodic()
        self._servo_desired_angle_pub.set(self._climb_servo.getAngle())

    def simulationPeriodic(self) -> None:
        self._climber_arm.setAngle(self.get_position() * 360)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        output, servo_angle = self._state_configs.get(desired_state, (0, Constants.ClimberConstants.SERVO_ENGAGED_ANGLE))
        self._climb_request.output = output
        self._climb_servo.setAngle(servo_angle)

        self._climb_motor.set_control(self._climb_request)
        self._winch_motor.set_control(self._winch_request)

    def get_position(self) -> float:
        return self._climb_motor.get_position().value