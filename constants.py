from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from enum import Enum, auto

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

class Constants:

    class MotorIDs(Enum):

        LEFT_LIFT_MOTOR = auto()
        RIGHT_LIFT_MOTOR = auto()


    class LiftConstants(Enum):

        L1_SCORE_POSITION = 0
        L2_SCORE_POSITION = 0
        L3_SCORE_POSITION = 0
        L4_SCORE_POSITION = 0

        INTAKE_POSITION = 0