from enum import auto, Enum

from commands2 import Command
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, utils, BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, MotionMagicConfigs
from phoenix6.controls import VoltageOut, Follower, MotionMagicVoltage
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import InvertedValue, FeedbackSensorSourceValue, NeutralModeValue, ForwardLimitValue
from phoenix6.sim import ChassisReference
from wpilib import DriverStation, RobotBase, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.filter import Debouncer
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


class PivotSubsystem(StateSubsystem):
    """
    The PivotSubsystem is responsible for controlling the end effector's angle.
    It uses Motion Magic with Voltage gains to quickly transition between set points.
    It also incorporates a remote CANcoder to mitigate backlash through the pivot gearbox, increasing accuracy.
    """

    class SubsystemState(Enum):
        IDLE = None
        AVOID_ELEVATOR = Constants.PivotConstants.ELEVATOR_PRIORITY_ANGLE
        STOW = Constants.PivotConstants.STOW_ANGLE
        GROUND_INTAKE = Constants.PivotConstants.GROUND_INTAKE_ANGLE
        FUNNEL_INTAKE = Constants.PivotConstants.FUNNEL_INTAKE_ANGLE
        ALGAE_INTAKE = Constants.PivotConstants.ALGAE_INTAKE_ANGLE
        HIGH_SCORING = Constants.PivotConstants.HIGH_SCORING_ANGLE
        L3_CORAL = Constants.PivotConstants.MID_SCORING_ANGLE
        L2_CORAL = Constants.PivotConstants.MID_SCORING_ANGLE
        LOW_SCORING = Constants.PivotConstants.LOW_SCORING_ANGLE
        NET_SCORING = Constants.PivotConstants.NET_SCORING_ANGLE
        PROCESSOR_SCORING = Constants.PivotConstants.PROCESSOR_SCORING_ANGLE
        AVOID_CLIMBER = Constants.PivotConstants.CLIMBER_PRIORITY_ANGLE

    _encoder_config = CANcoderConfiguration()
    (
        _encoder_config.magnet_sensor
        .with_magnet_offset(Constants.PivotConstants.CANCODER_OFFSET)
        .with_absolute_sensor_discontinuity_point(Constants.PivotConstants.CANCODER_DISCONTINUITY)
    )

    _master_config = TalonFXConfiguration()
    (_master_config.feedback
     .with_rotor_to_sensor_ratio(Constants.PivotConstants.GEAR_RATIO)
     .with_feedback_sensor_source(FeedbackSensorSourceValue.REMOTE_CANCODER)
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
        self._master_motor.sim_state.orientation = ChassisReference.Clockwise_Positive
        self._follower_motor = TalonFX(Constants.CanIDs.RIGHT_PIVOT_TALON)

        self._encoder.configurator.apply(self._encoder_config)
        self._master_motor.configurator.apply(self._master_config)
        self._follower_motor.configurator.apply(self._follower_config)

        self._add_talon_sim_model(self._master_motor, DCMotor.krakenX60FOC(2), Constants.PivotConstants.GEAR_RATIO, 0.0807378172)

        self._at_setpoint_debounce = Debouncer(0.1, Debouncer.DebounceType.kRising)
        self._at_setpoint = True

        self._position_request = MotionMagicVoltage(0)
        self._brake_request = VoltageOut(0)
        self._sys_id_request = VoltageOut(0)

        self._follower_motor.set_control(Follower(self._master_motor.device_id, True))

        self._sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
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
            self._master_motor.get_position(False), self._master_motor.get_velocity(False)
        )
        self._at_setpoint = self._at_setpoint_debounce.calculate(abs(latency_compensated_position - self._position_request.position) <= Constants.PivotConstants.SETPOINT_TOLERANCE)
        self.get_network_table().getEntry("At Setpoint").setBoolean(self._at_setpoint)
        self.get_network_table().getEntry("In Elevator").setBoolean(self.is_in_elevator())

        # Update CANcoder sim state
        if utils.is_simulation() and not RobotBase.isReal():
            talon_sim = self._sim_models[0][0]
            cancoder_sim = self._encoder.sim_state

            cancoder_sim.set_supply_voltage(RobotController.getBatteryVoltage())
            cancoder_sim.set_raw_position(talon_sim.getAngularPosition() / Constants.PivotConstants.GEAR_RATIO)
            cancoder_sim.set_velocity(talon_sim.getAngularVelocity() / Constants.PivotConstants.GEAR_RATIO)
            
    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        position = desired_state.value
        if position is None:
            self._master_motor.set_control(self._brake_request)
            return

        self._position_request.position = position
        self._master_motor.set_control(self._position_request)

    def is_at_setpoint(self) -> bool:
        return self._at_setpoint

    def is_in_elevator(self, position=None) -> bool:
        if not position:
            position = self._master_motor.get_position(True).value
        return position >= Constants.PivotConstants.INSIDE_ELEVATOR_ANGLE
    
    def get_setpoint(self) -> float:
        return self._position_request.position

    def stop(self) -> Command:
        return self.runOnce(lambda: self._master_motor.set_control(self._sys_id_request.with_output(0)))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction).andThen(self.stop())

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction).andThen(self.stop())

    def get_position(self) -> float:
        """Returns the current angle of the pivot, in degrees."""
        return self._master_motor.get_position().value
