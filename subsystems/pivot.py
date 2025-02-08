from enum import auto, Enum

from commands2 import Command
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import PositionDutyCycle, VoltageOut
from wpilib import SmartDashboard, DriverStation
from wpilib.sysid import SysIdRoutineLog
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem

class PivotSubsystem(StateSubsystem):
    """
    The PivotSubsystem is responsible for controlling the end effector's current rotation.
    It uses a PositionDutyCycle to quickly transition between set points and is tuned through the use of SysId routines.
    """

    class SubsystemState(Enum):
        STOW = auto()
        GROUND_INTAKE = auto()
        FUNNEL_INTAKE = auto()
        ALGAE_INTAKE = auto()
        HIGH_SCORING = auto()
        MID_SCORING = auto()
        LOW_SCORING = auto()
        NET_SCORING = auto()
        PROCESSOR_SCORING = auto()

    _master_config = TalonFXConfiguration()
    _master_config.feedback.with_rotor_to_sensor_ratio(Constants.PivotConstants.GEAR_RATIO)
    _master_config.with_slot0(Constants.PivotConstants.GAINS)

    _follower_config = TalonFXConfiguration
    _follower_config.feedback.with_rotor_to_sensor_ratio(Constants.PivotConstants.GEAR_RATIO)
    _follower_config.with_slot0(Constants.PivotConstants.GAINS)
    _follower_config.motor_output = InvertedValue.CLOCKWISE_POSITIVE

    def __init__(self) -> None:
        super().__init__("Pivot")

        self._pivot_motor = TalonFX(Constants.MotorIDs.PIVOT_MOTOR)
        self._pivot_motor.configurator.apply(self._master_config)
        self._add_talon_sim_model(self._pivot_motor, DCMotor.krakenX60FOC(2), Constants.PivotConstants.GEAR_RATIO)

        self._position_request = PositionDutyCycle(0)
        self._sys_id_request = VoltageOut(0)

        self._sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdPivot_State", SysIdRoutineLog.stateEnumToString(state)
                )  # Log to .hoot for ease of access
            ),
            SysIdRoutine.Mechanism(
                lambda output: self._pivot_motor.set_control(self._sys_id_request.with_output(output)),
                lambda log: None,
                self,
            ),
        )

        self._master_pivot_motor = TalonFX(Constants.MotorIDs.LEFT_PIVOT_MOTOR)
        self._follower_pivot_motor = TalonFX(Constants.MotorIDs.RIGHT_PIVOT_MOTOR)

        self._master_pivot_motor.configurator.apply(self._master_config)
        self._follower_pivot_motor.configurator.apply(self._follower_config)

        self._add_talon_sim_model(self._pivot_motor, DCMotor.krakenX60FOC(2), Constants.PivotConstants.GEAR_RATIO)

        self._position_request = PositionDutyCycle(0)
        self._sys_id_request = VoltageOut(0)

        self._follower_motor.set_control(Follower(self._master_pivot_motor.device_id, False))

        self._sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdPivot_State", SysIdRoutineLog.stateEnumToString(state)
                )  # Log to .hoot for ease of access
            ),
            SysIdRoutine.Mechanism(
                lambda output: self._pivot_motor.set_control(self._sys_id_request.with_output(output)),
                lambda log: None,
                self,
            ),
        )
    def periodic(self):
        return super().periodic()

    def set_desired_state(self, desired_state: SubsystemState) -> None:

        if DriverStation.isTest():
            return

        match desired_state:
            case self.SubsystemState.STOW:
                self._position_request.position = Constants.PivotConstants.STOW_ANGLE
            case self.SubsystemState.GROUND_INTAKE:
                self._position_request.position = Constants.PivotConstants.GROUND_INTAKE_ANGLE
            case self.SubsystemState.FUNNEL_INTAKE:
                self._position_request.position = Constants.PivotConstants.FUNNEL_INTAKE_ANGLE
            case self.SubsystemState.HIGH_SCORING:
                self._position_request.position = Constants.PivotConstants.HIGH_SCORING_ANGLE
            case self.SubsystemState.MID_SCORING:
                self._position_request.position = Constants.PivotConstants.MID_SCORING_ANGLE
            case self.SubsystemState.LOW_SCORING:
                self._position_request.position = Constants.PivotConstants.LOW_SCORING_ANGLE
            case self.SubsystemState.NET_SCORING:
                self._position_request.position = Constants.PivotConstants.NET_SCORING_ANGLE
            case self.SubsystemState.PROCESSOR_SCORING:
                self._position_request.position = Constants.PivotConstants.PROCESSOR_SCORING_ANGLE
            case self.SubsystemState.ALGAE_INTAKE:
                self._position_request.position = Constants.PivotConstants.ALGAE_INTAKE_ANGLE

        self._subsystem_state = desired_state
        self._pivot_motor.set_control(self._position_request)

    def stop(self) -> Command:
        return self.runOnce(lambda: self._pivot_motor.set_control(self._sys_id_request.with_output(0)))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction).andThen(self.stop())

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction).andThen(self.stop())

    def get_angle(self) -> float:
        """Returns the current angle of the pivot, in degrees."""
        return self._pivot_motor.get_position().value * 360