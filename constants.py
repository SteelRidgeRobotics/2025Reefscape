from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from enum import Enum, auto

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

class Constants:

    class MotorIDs():

        LEFT_LIFT_MOTOR = 0 # Placeholders
        RIGHT_LIFT_MOTOR = 1


    class ElevatorConstants():

        L1_SCORE_POSITION = 0 # Placeholders
        L2_SCORE_POSITION = 0
        L3_SCORE_POSITION = 0
        L4_SCORE_POSITION = 0

        DEFAULT_POSITION = 0