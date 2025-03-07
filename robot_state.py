from typing import Self

from wpimath import units

from subsystems import StateSubsystem
from subsystems.elevator import ElevatorSubsystem
from subsystems.pivot import PivotSubsystem

class RobotState:
    """Singleton for getting subsystem positions and measurements. Handles all subsystem "communication"."""
    _instance = None

    def __new__(cls, pivot: PivotSubsystem, elevator: ElevatorSubsystem):
        if cls._instance is None:
            cls._instance = super(RobotState, cls).__new__(cls)
            cls._instance._pivot = pivot
            cls._instance._elevator = elevator
        return cls._instance

    @classmethod
    def get_instance(cls) -> Self:
        """:returns: the singleton instance, or raises an error if it has not been initialized."""
        if cls._instance is None:
            raise RuntimeError("RobotState has not been initialized.")
        return cls._instance

    def get_pivot_position(self) -> units.degrees:
        """:returns: The pivot position, in rotations. An position of 0 means the pivot is fully extended horizontally."""
        return self._pivot.get_position()

    def get_pivot_state(self) -> StateSubsystem.SubsystemState:
        """:returns: the current state of the pivot."""
        return self._pivot.get_current_state()

    def is_pivot_in_elevator(self) -> bool:
        """:returns: True if the pivot is in the elevator."""
        return self._pivot.is_in_elevator()

    def is_elevator_at_setpoint(self) -> bool:
        """:returns: True if the elevator is at its desired setpoint."""
        return self._elevator.is_at_setpoint()

    def get_elevator_state(self) -> StateSubsystem.SubsystemState:
        """:returns: the current state of the elevator."""
        return self._elevator.get_current_state()
