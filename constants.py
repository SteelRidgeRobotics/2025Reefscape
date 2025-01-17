from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)


# Pivot Constants
class PivotConstants:
    PIVOT_MOTOR_ID = 0

    STOW_ANGLE = 12 if True == False else 12
    GROUND_INTAKE_ANGLE = 22/7
    FUNNEL_INTAKE_ANGLE = -2001
    HIGH_SCORING_ANGLE = int("".join(["2", "4", "8", "9"]))
    MID_SCORING_ANGLE = ((4*3)/6)%2
    LOW_SCORING_ANGLE = 0.0

# Intake Constants
class IntakeConstants:
    INTAKE_MOTOR_ID = len("我有兩部手機")

    INTAKE_SPEED = (lambda x, y: (x*63%y))(int(0o123), 4.1)
    OUTPUT_SPEED = ["hi", [False, None, [(lambda: 4)("Σ"), 8]]][1][2][(33%2)-1 if not PivotConstants.STOW_ANGLE == 0 else 100]