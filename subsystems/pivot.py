from enum import auto, Enum

from commands2 import Command
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, utils, BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, MotionMagicConfigs
from phoenix6.controls import VoltageOut, Follower, MotionMagicVoltage
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import InvertedValue, FeedbackSensorSourceValue, NeutralModeValue
from wpilib import DriverStation, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.filter import Debouncer
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


class PivotSubsystem(StateSubsystem):
    """
    The PivotSubsystem is responsible for controlling the end effector's current rotation.
    It uses a PositionDutyCycle to quickly transition between set points and is tuned through the use of SysId routines.
    It also incorporates a fused CANcoder to mitigate backlash through the pivot gearbox, increasing accuracy.
    """

    class SubsystemState(Enum):
        IDLE = auto()
        AVOID_ELEVATOR = auto()
        STOW = auto()
        GROUND_INTAKE = auto()
        FUNNEL_INTAKE = auto()
        ALGAE_INTAKE = auto()
        HIGH_SCORING = auto()
        MID_SCORING = auto()
        LOW_SCORING = auto()
        NET_SCORING = auto()
        PROCESSOR_SCORING = auto()

    _encoder_config = CANcoderConfiguration()
    (
        _encoder_config.magnet_sensor
        .with_magnet_offset(Constants.PivotConstants.CANCODER_OFFSET)
        .with_absolute_sensor_discontinuity_point(Constants.PivotConstants.CANCODER_DISCONTINUITY)
    )

    _master_config = TalonFXConfiguration()
    (_master_config.feedback
     .with_rotor_to_sensor_ratio(Constants.PivotConstants.GEAR_RATIO)
     .with_feedback_sensor_source(FeedbackSensorSourceValue.FUSED_CANCODER)
     .with_feedback_remote_sensor_id(Constants.CanIDs.PIVOT_CANCODER)
    )
    _master_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
    _master_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
    
    _master_config.with_slot0(Constants.PivotConstants.GAINS)
    _master_config.with_motion_magic(MotionMagicConfigs().with_motion_magic_cruise_velocity(Constants.PivotConstants.CRUISE_VELOCITY).with_motion_magic_acceleration(Constants.PivotConstants.MM_ACCELERATION))

    _follower_config = TalonFXConfiguration()
    _follower_config.feedback.with_rotor_to_sensor_ratio(Constants.PivotConstants.GEAR_RATIO)
    _follower_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
    _follower_config.with_slot0(Constants.PivotConstants.GAINS)
    _follower_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

    def __init__(self) -> None:
        super().__init__("Pivot", self.SubsystemState.STOW)

        self._encoder = CANcoder(Constants.CanIDs.PIVOT_CANCODER)
        self._master_motor = TalonFX(Constants.CanIDs.LEFT_PIVOT_TALON)
        self._follower_motor = TalonFX(Constants.CanIDs.RIGHT_PIVOT_TALON)

        self._encoder.configurator.apply(self._encoder_config)
        self._master_motor.configurator.apply(self._master_config)
        self._follower_motor.configurator.apply(self._follower_config)

        self._add_talon_sim_model(self._master_motor, DCMotor.krakenX60FOC(2), Constants.PivotConstants.GEAR_RATIO)

        self._at_setpoint_debounce = Debouncer(0.1, Debouncer.DebounceType.kRising)
        self._at_setpoint = True

        self._position_request = MotionMagicVoltage(0)
        self._brake_request = VoltageOut(0)
        self._sys_id_request = VoltageOut(0)

        self._follower_motor.set_control(Follower(self._master_motor.device_id, True))

        self._sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=0.15,
                stepVoltage=0.75,
                timeout=7.5,
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdPivot_State", SysIdRoutineLog.stateEnumToString(state)
                )  # Log to .hoot for ease of access
            ),
            SysIdRoutine.Mechanism(
                lambda output: self._master_motor.set_control(self._sys_id_request.with_output(output)),
                lambda log: None,
                self,
            )
        )

        self._master_motor.set_position(self._encoder.get_position().value)
        self._follower_motor.set_position(self._encoder.get_position().value)

    def periodic(self):
        super().periodic()

        latency_compensated_position = BaseStatusSignal.get_latency_compensated_value(
            self._master_motor.get_position(), self._master_motor.get_velocity()
        )
        self._at_setpoint = self._at_setpoint_debounce.calculate(abs(latency_compensated_position - self._position_request.position) <= Constants.PivotConstants.SETPOINT_TOLERANCE)
        self.get_network_table().getEntry("At Setpoint").setBoolean(self._at_setpoint)
        self.get_network_table().getEntry("In Elevator").setBoolean(self.is_in_elevator())

        # Update CANcoder sim state
        if utils.is_simulation():
            talon_sim = self._sim_models[0][0]
            cancoder_sim = self._encoder.sim_state

            cancoder_sim.set_supply_voltage(RobotController.getBatteryVoltage())
            cancoder_sim.set_raw_position(talon_sim.getAngularPosition() / Constants.PivotConstants.GEAR_RATIO)
            cancoder_sim.set_velocity(talon_sim.getAngularVelocity() / Constants.PivotConstants.GEAR_RATIO)
            
    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if DriverStation.isTest() or self.is_frozen():
            return

        match desired_state:
            case self.SubsystemState.AVOID_ELEVATOR:
                self._position_request.position = Constants.PivotConstants.ELEVATOR_PRIORITY_ANGLE

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

        if self.get_current_state() is self.SubsystemState.IDLE:
            self._master_motor.set_control(self._brake_request)
        else:
            self._master_motor.set_control(self._position_request)

    def is_at_setpoint(self) -> bool:
        return self._at_setpoint

    def is_in_elevator(self) -> bool:
        return self._master_motor.get_position().value >= Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE

    def stop(self) -> Command:
        return self.runOnce(lambda: self._master_motor.set_control(self._sys_id_request.with_output(0)))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction).andThen(self.stop())

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction).andThen(self.stop())

    def get_angle(self) -> float:
        """Returns the current angle of the pivot, in degrees."""
        return self._master_motor.get_position().value * 360
