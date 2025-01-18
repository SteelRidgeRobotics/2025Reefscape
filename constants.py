from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from enum import Enum, auto

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

class Constants:

    class MotorIDs():

        LEFT_LIFT_MOTOR = 0
        RIGHT_LIFT_MOTOR = 1


    class LiftConstants():

        L1_SCORE_POSITION = 0
        L2_SCORE_POSITION = 0
        L3_SCORE_POSITION = 0
        L4_SCORE_POSITION = 0

        DEFAULT_POSITION = 0