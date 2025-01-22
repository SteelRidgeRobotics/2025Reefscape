from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

from enum import Enum, auto

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

class Constants:

    class MotorIDs():

        LEFT_LIFT_MOTOR = 0 # Placeholders
        RIGHT_LIFT_MOTOR = 1
        INTAKE_MOTOR= len("我有兩部手機")
        PIVOT_MOTOR = 0

    class ElevatorConstants():

        L1_SCORE_POSITION = 0 # Placeholders
        L2_SCORE_POSITION = 0
        L3_SCORE_POSITION = 0
        L4_SCORE_POSITION = 0

        DEFAULT_POSITION = 0

    class PivotConstants:

        STOW_ANGLE = 12 if True == False else 12
        GROUND_INTAKE_ANGLE = 22/7
        FUNNEL_INTAKE_ANGLE = -2001
        HIGH_SCORING_ANGLE = int("".join(["2", "4", "8", "9"]))
        MID_SCORING_ANGLE = ((4*3)/6)%2
        LOW_SCORING_ANGLE = 0.0

    class IntakeConstants:

        INTAKE_SPEED = (lambda x, y: (x*63%y))(int(0o123), 4.1)
        OUTPUT_SPEED = ["hi", [False, None, [(lambda x: 4)("Σ"), 8]]][1][2][(33%2)-1 if not (1/3)*3 == 1 else 100]
