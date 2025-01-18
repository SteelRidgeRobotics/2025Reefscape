from enum import Enum, auto

from phoenix6.controls import Follower
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import PositionDutyCycle, DutyCycleOut
from phoenix6.hardware import TalonFX

from subsystems import StateSubsystem

from constants import Constants

# Elevator Subsystem class inhereting from StateSubsystem
class ElevatorSubsystem(StateSubsystem):

    # All possible states the elevator can be in
    class SubsystemState(Enum):
        L1 = auto()
        L2 = auto()
        L3 = auto()
        L4 = auto()

        DEFAULT = auto()

    # Initializes the Elevator Subsystem
    def __init__(self):

        # Initialize the subsystem with the StateSubsystem parent
        super().__init__("Elevator")

        # Creates the main configuration object
        self._master_config = TalonFXConfiguration()

        # Setting the neutral mode to brake
        self._master_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)

        # Creates the master lift motor (left) and applies the configuration
        self._master_motor = TalonFX(Constants.MotorIDs.LEFT_LIFT_MOTOR)
        self._master_motor.configurator.apply(self._master_config)

        # Creates the follower lift motor (right) and applies the configuration
        self._follower_motor = TalonFX(Constants.MotorIDs.RIGHT_LIFT_MOTOR)
        self._follower_motor.configurator.apply(self._master_config)

        # Sets the default subsystem state to intake/ground
        self._subsystem_state = self.SubsystemState.INTAKE

        # Creating a default position request and a brake request
        self._position_request = PositionDutyCycle(0)
        self._brake_request = DutyCycleOut(0)

        # Sets the default master motor request to brake
        self._master_motor.set_control(self._brake_request)

        # Follower motor follows the control of the master motor
        self._follower_motor.set_control(Follower(self._master_motor.device_id))
    

    # Runs periodically                                             
    def periodic(self):

        super().periodic()

        # Handles all of the possible subsystem states
        match self._subsystem_state:

            case self.SubsystemState.INTAKE:
                self._position_request.position = Constants.LiftConstants.INTAKE_POSITION    

            case self.SubsystemState.L1:
                self._position_request.position = Constants.LiftConstants.L1_SCORE_POSITION     

            case self.SubsystemState.L2:
                self._position_request.position = Constants.LiftConstants.L2_SCORE_POSITION
                
            case self.SubsystemState.L3:
                self._position_request.position = Constants.LiftConstants.L3_SCORE_POSITION
                
            case self.SubsystemState.L4:
                self._position_request.position = Constants.LiftConstants.L4_SCORE_POSITION
            
        # Sets the control of the motor to the position request
        self._master_motor.set_control(self._position_request)
                