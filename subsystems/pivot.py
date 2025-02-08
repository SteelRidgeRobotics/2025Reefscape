from enum import auto, Enum

from subsystems import StateSubsystem
from wpilib import SmartDashboard
from phoenix6.hardware import TalonFX

from phoenix6.configs import TalonFXConfiguration
from phoenix6.signals import InvertedValue
from phoenix6.controls import PositionDutyCycle, VoltageOut, Follower
from wpilib import SmartDashboard, DriverStation
from wpilib.sysid import SysIdRoutineLog
from wpimath.system.plant import DCMotor

from constants import Constants

class Pivot(StateSubsystem):

    class SubsystemState(Enum):
        STOW = auto()
        GROUND_INTAKE = auto()
        FUNNEL_INTAKE = auto()
        HIGH_SCORING = auto()
        MID_SCORING = auto()
        LOW_SCORING = auto()

    _follower_config = TalonFXConfiguration
    _follower_config.feedback.with_rotor_to_sensor_ratio(Constants.PivotConstants.GEAR_RATIO)
    _follower_config.with_slot0(Constants.PivotConstants.GAINS)
    _follower_config.motor_output = InvertedValue.CLOCKWISE_POSITIVE

    def __init__(self) -> None:

        super().__init__("Pivot")
    
        self._subsystem_state = self.SubsystemState.STOW

        self.pivotMotor = TalonFX(Constants.MotorIDs.PIVOT_MOTOR)

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

        # move motor accordingly to set state in superstructure
        match desired_state:

            case self.SubsystemState.STOW:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.STOW_ANGLE))

            case self.SubsystemState.GROUND_INTAKE:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.GROUND_INTAKE_ANGLE))

            case self.SubsystemState.FUNNEL_INTAKE:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.FUNNEL_INTAKE_ANGLE))

            case self.SubsystemState.HIGH_SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.HIGH_SCORING_ANGLE))

            case self.SubsystemState.MID_SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.MID_SCORING_ANGLE))

            case self.SubsystemState.LOW_SCORING:
                self.pivotMotor.set_control(PositionDutyCycle(Constants.PivotConstants.LOW_SCORING_ANGLE))

        # update information for the state
        self._subsystem_state = desired_state
        SmartDashboard.putString("Pivot State", self._subsystem_state.name)