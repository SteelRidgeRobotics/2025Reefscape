from enum import Enum, auto

from phoenix6.controls import Follower
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import DifferentialSensorSourceValue, DifferentialSensorsConfigs, NeutralModeValue
from phoenix6.controls import CoastOut, DifferentialFollower, DutyCycleOut, PositionDutyCycle
from phoenix6.hardware import TalonFX

from subsystems import StateSubsystem

from constants import Constants

class ElevatorSubsystem(StateSubsystem):

    class SubsystemState(Enum):
        L1 = auto()
        L2 = auto()
        L3 = auto()
        L4 = auto()

        INTAKE = auto()


    def __init__(self):

        super.__init__("Elevator")

        self._master_config = TalonFXConfiguration()

        self._master_motor = TalonFX(Constants.MotorIDs.LEFT_LIFT_MOTOR)
        self._master_motor.configurator.apply(self._master_config)

        self._follower_motor = TalonFX(Constants.MotorIDs.RIGHT_LIFT_MOTOR)
        self._follower_motor.configurator.apply(self._master_config)

        self._subsystem_state = self.SubsystemState.INTAKE

        self._position_request = PositionDutyCycle(0)
        self._stop_request = CoastOut()

        self._master_motor.set_control(self._stop_request)
        self._follower_motor.set_control(Follower(self._master_motor.device_id))
    

    def get_current_state(self):
        return self._subsystem_state


    def periodic(self):

        super().periodic()

        match self._subsystem_state:
            
            case self.SubsystemState.INTAKE:
                self._position_request.position = Constants.LiftConstants.INTAKE_POSITION
                self._master_motor.set_control(self._position_request)

            case self.SubsystemState.L1:
                self._position_request.position = Constants.LiftConstants.L1_SCORE_POSITION
                self._master_motor.set_control(self._position_request)

            case self.SubsystemState.L2:
                self._position_request.position = Constants.LiftConstants.L2_SCORE_POSITION
                self._master_motor.set_control(self._position_request)

            case self.SubsystemState.L3:
                self._position_request.position = Constants.LiftConstants.L3_SCORE_POSITION
                self._master_motor.set_control(self._position_request)

            case self.SubsystemState.L4:
                self._position_request.position = Constants.LiftConstants.L4_SCORE_POSITION
                self._master_motor.set_control(self._position_request)

    
        
        
