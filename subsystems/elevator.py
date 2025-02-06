import math
from enum import Enum, auto

from phoenix6 import StatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import Follower
from phoenix6.controls import PositionDutyCycle, DutyCycleOut
from phoenix6.hardware import TalonFX
from wpimath.system.plant import DCMotor

from constants import Constants
from subsystems import StateSubsystem


# Elevator Subsystem class inheriting from StateSubsystem
class ElevatorSubsystem(StateSubsystem):

    # All possible states the elevator can be in
    class SubsystemState(Enum):
        L1 = auto()
        L2 = auto()
        L3 = auto()
        L4 = auto()
        L2_ALGAE = auto()
        L3_ALGAE = auto()
        NET = auto()

        DEFAULT = auto()

    # Initializes the Elevator Subsystem
    def __init__(self):

        # Initialize the subsystem with the StateSubsystem parent
        super().__init__("Elevator")

        # Creates the main configuration object
        self._master_config = TalonFXConfiguration()

        # Setting the neutral mode to brake
        self._master_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)

        # Set the mechanism gear ratio
        self._master_config.feedback.with_sensor_to_mechanism_ratio(Constants.ElevatorConstants.GEAR_RATIO)

        # Set PID and feedforward gains
        self._master_config.with_slot0(Constants.ElevatorConstants.GAINS)

        # Creates the master lift motor (left) and applies the configuration
        self._master_motor = TalonFX(Constants.MotorIDs.LEFT_LIFT_MOTOR)
        self._master_motor.configurator.apply(self._master_config)

        # Creates the follower lift motor (right) and applies the configuration
        self._follower_motor = TalonFX(Constants.MotorIDs.RIGHT_LIFT_MOTOR)
        self._follower_motor.configurator.apply(self._master_config)

        # Sets the default subsystem state to default/ground
        self._subsystem_state = self.SubsystemState.DEFAULT

        # Creating a default position request and a brake request
        self._position_request = PositionDutyCycle(0)
        self._brake_request = DutyCycleOut(0)

        # Sets the default master motor request to brake
        self._master_motor.set_control(self._brake_request)

        # Follower motor follows the control of the master motor
        self._follower_motor.set_control(Follower(self._master_motor.device_id, False))

        self._add_talon_sim_model(self._master_motor, DCMotor.krakenX60FOC(2), Constants.ElevatorConstants.GEAR_RATIO)
    
    # Runs periodically                                             
    def periodic(self):

        super().periodic()

        # Handles all the possible subsystem states
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
            
        # Sets the control of the motor to the position request
        self._master_motor.set_control(self._position_request)

    def get_height(self) -> float:
        """Returns the height of the elevator, in meters."""
        return (self._master_motor.get_position().value / Constants.ElevatorConstants.GEAR_RATIO) * (2 * math.pi * 0.508)
