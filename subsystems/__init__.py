from abc import ABC, ABCMeta
from enum import Enum

from commands2 import Command, InstantCommand
from commands2.subsystem import Subsystem
from ntcore import *
from phoenix6 import utils
from phoenix6.hardware import TalonFX
from wpilib import RobotController, DriverStation
from wpilib.simulation import DCMotorSim
from wpimath import units
from wpimath.system.plant import DCMotor, LinearSystemId


class StateSubsystemMeta(ABCMeta, type(Subsystem)):
    pass


class StateSubsystem(Subsystem, ABC, metaclass=StateSubsystemMeta):
    """
    Subsystem class for handling subsystem state transitions and motor sim models.
    """

    class SubsystemState(Enum):
        OFF = 0

    def __init__(self, name: str, starting_state: SubsystemState):
        """
        Initializes all subsystem logging and sets the default state.
        
        :param name: Name of the subsystem
        :type name: str
        :param starting_state: Starting state of the subsystem
        :type starting_state:
        """
        super().__init__()
        self.setName(name.title())

        self._frozen = False
        self._subsystem_state = None # This allows set_desired_state to succeed
        self.__starting_state = starting_state

        # Create NT folder for organization
        self._network_table = NetworkTableInstance.getDefault().getTable(name.title())
        self._nt_publishers = []
        current_state_nt = self._network_table.getStringTopic("Current State")
        self._current_state_pub = current_state_nt.publish()
        self._current_state_pub.set("INITIALIZING")

        frozen_nt = self._network_table.getBooleanTopic("Frozen")
        self._frozen_pub = frozen_nt.publish()

        self._sim_models: list[tuple[DCMotorSim, TalonFX]] = []

    def set_desired_state(self, desired_state: SubsystemState) -> bool: # type: ignore
        """
        Sets the desired state of the subsystem.
        It's recommended to override this function in order to update objects such as control requests.
        """
        current_state = self._subsystem_state
        if current_state is desired_state or DriverStation.isTest() or self.is_frozen():
            return False
        self._subsystem_state = desired_state
        self._current_state_pub.set(self.get_state_name())
        return True

    def periodic(self):
        # We call set_desired_state once in periodic to ensure the state is correctly set on startup
        if self._subsystem_state is None:
            self.set_desired_state(self.__starting_state)
        if not utils.is_simulation():
            return
        for model in self._sim_models:
            sim = model[1].sim_state
            sim.set_supply_voltage(RobotController.getBatteryVoltage())
            model[0].setInputVoltage(sim.motor_voltage)
            model[0].update(0.02)

            sim.set_raw_rotor_position(units.radiansToRotations(model[0].getAngularPosition())
                                       * model[0].getGearing())
            sim.set_rotor_velocity(units.radiansToRotations(model[0].getAngularVelocity())
                                       * model[0].getGearing())
            sim.set_rotor_acceleration(units.radiansToRotations(model[0].getAngularAcceleration())
                                       * model[0].getGearing())

    def freeze(self) -> None:
        """Prevents new state changes."""
        self._frozen = True
        self._frozen_pub.set(True)

    def unfreeze(self) -> None:
        """Allows state changes."""
        self._frozen = False
        self._frozen_pub.set(False)

    def is_frozen(self) -> bool:
        return self._frozen
            
    def get_current_state(self) -> SubsystemState | None:
        state = self._subsystem_state
        return state

    def get_state_name(self) -> str:
        """Returns the name of the current state."""
        return self._subsystem_state.name.title().replace("_", " ")

    def get_network_table(self) -> NetworkTable:
        return self._network_table

    def _add_talon_sim_model(self, talon: TalonFX, motor: DCMotor, gearing: float, 
                             moi: float=0.001) -> None:
        """Creates a DCMotorSim that updates periodically during 
        simulation. This also logs the talon's status signals to Network
        Tables, regardless if simulated.

        :param talon: The TalonFX to simulate.
        :type talon: TalonFX
        :param motor: The motor of the TalonFX on the physical robot.
        :type motor: DCMotor
        :param gearing: The gearing from the TalonFX to the mechanism.
        This will depend on whether the mechanism is relying on the
        motor's internal sensor or an absolute encoder.
        :type gearing: float
        :param moi: The moment of interia for the simulated mechanism, 
        defaults to 0.001
        :type moi: float, optional
        """
        self._sim_models.append(
            (DCMotorSim(
                LinearSystemId.DCMotorSystem(
                    motor,
                    moi,
                    gearing
                ),
                motor
            ),
            talon)
        )
    
    def set_desired_state_command(self, state: SubsystemState) -> Command:
        return InstantCommand(lambda: self.set_desired_state(state))
