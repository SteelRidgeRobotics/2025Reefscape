from abc import ABC, ABCMeta
from enum import Enum

from commands2 import Command, InstantCommand
from commands2.subsystem import Subsystem
from ntcore import *
from phoenix6 import utils
from phoenix6.hardware import TalonFX
from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath import units
from wpimath.system.plant import DCMotor, LinearSystemId


class StateSubsystemMeta(ABCMeta, type(Subsystem)):
    pass


class StateSubsystem(Subsystem, ABC, metaclass=StateSubsystemMeta):
    """A helper class designed to help make logging easier, as well as
    make simulation easier.
    """

    class SubsystemState(Enum):
        OFF = 0

    def __init__(self, name: str, current_state: SubsystemState=SubsystemState(0)):
        super().__init__()
        self.setName(name.title())

        self._subsystem_state = current_state

        # Create NT folder for organization
        self._network_table = NetworkTableInstance.getDefault().getTable(name.title())
        self._nt_publishers = []
        self._current_state_pub = self._network_table.getStringTopic("Current State").publish()

        self._sim_models: list[tuple[DCMotorSim, TalonFX]] = []

    def set_desired_state(self, desired_state: SubsystemState) -> None: # type: ignore
        """Override this method to handle desired state handling for
        your subsystem!
        """
        if self._subsystem_state is desired_state:
            return
        self._subsystem_state = desired_state

    def periodic(self):
        self._current_state_pub.set(self._subsystem_state.name.title().replace("_", " "))

        # Update sim models
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
            
    def get_current_state(self) -> SubsystemState:
        return self._subsystem_state

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
