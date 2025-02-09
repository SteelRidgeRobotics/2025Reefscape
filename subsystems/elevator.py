import math
from enum import Enum, auto

from commands2 import Command
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs, CANdiConfiguration, QuadratureConfigs, DigitalInputsConfigs
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import Follower, VoltageOut
from phoenix6.controls import PositionDutyCycle, DutyCycleOut
from phoenix6.hardware import CANdi, TalonFX
from phoenix6.signals import S1CloseStateValue
from wpilib import DriverStation
from wpilib.sysid import SysIdRoutineLog
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


class ElevatorSubsystem(StateSubsystem):
    """
    The ElevatorSubsystem is responsible for controlling the elevator mechanism.
    It manages motor positions and uses MotionMagic to smoothly transition between set points.
    """

    class SubsystemState(Enum):
        DEFAULT = auto()
        L1 = auto()
        L2 = auto()
        L3 = auto()
        L4 = auto()
        L2_ALGAE = auto()
        L3_ALGAE = auto()
        NET = auto()

    _candi_config = CANdiConfiguration()

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.ElevatorConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ElevatorConstants.GEAR_RATIO))
                     )

    def __init__(self) -> None:
        super().__init__("Elevator")

        self._master_motor = TalonFX(Constants.CanIDs.LEFT_ELEVATOR_TALON)
        self._master_motor.configurator.apply(self._motor_config)

        self._follower_motor = TalonFX(Constants.CanIDs.RIGHT_ELEVATOR_TALON)
        self._follower_motor.configurator.apply(self._motor_config)

        self._candi = CANdi(Constants.CanIDs.ELEVATOR_CANDI)
        self._candi.configurator.apply(self._candi_config)

        self._position_request = PositionDutyCycle(0)
        self._brake_request = DutyCycleOut(0)
        self._sys_id_request = VoltageOut(0)

        self._master_motor.set_control(self._brake_request)
        self._follower_motor.set_control(Follower(self._master_motor.device_id, False))

        self._add_talon_sim_model(self._master_motor, DCMotor.krakenX60FOC(2), Constants.ElevatorConstants.GEAR_RATIO)

        self._sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdElevator_State", SysIdRoutineLog.stateEnumToString(state)
                )  # Log to .hoot for ease of access
            ),
            SysIdRoutine.Mechanism(
                lambda output: self._master_motor.set_control(self._sys_id_request.with_output(output)),
                lambda log: None,
                self,
            )
        )

    def periodic(self) -> None:
        super().periodic()

        match self._subsystem_state:
            case self.SubsystemState.DEFAULT:
                self._position_request.position = Constants.ElevatorConstants.DEFAULT_POSITION
            case self.SubsystemState.L1:
                self._position_request.position = Constants.ElevatorConstants.L1_SCORE_POSITION
            case self.SubsystemState.L2:
                self._position_request.position = Constants.ElevatorConstants.L2_SCORE_POSITION
            case self.SubsystemState.L3:
                self._position_request.position = Constants.ElevatorConstants.L3_SCORE_POSITION
            case self.SubsystemState.L4:
                self._position_request.position = Constants.ElevatorConstants.L4_SCORE_POSITION
            case self.SubsystemState.L2_ALGAE:
                self._position_request.position = Constants.ElevatorConstants.L2_ALGAE_POSITION
            case self.SubsystemState.L3_ALGAE:
                self._position_request.position = Constants.ElevatorConstants.L3_ALGAE_POSITION
            case self.SubsystemState.NET:
                self._position_request.position = Constants.ElevatorConstants.NET_SCORE_POSITION

        if not DriverStation.isTest():
            self._master_motor.set_control(self._position_request)

    def stop(self) -> Command:
        return self.runOnce(lambda: self._master_motor.set_control(self._brake_request))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction).andThen(self.stop())

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction).andThen(self.stop())

    def get_height(self) -> float:
        """Returns the height of the elevator, in meters."""
        return (self._master_motor.get_position().value / Constants.ElevatorConstants.GEAR_RATIO) * (2 * math.pi * 0.508)
