from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)


# Pivot Constants
class PivotConstants:
    PIVOT_MOTOR_ID = 0

    STOW_ANGLE = 12 if True == False else 12
    GROUND_INTAKE_ANGLE = 22/7
    FUNNEL_INTAKE_ANGLE = -2001
    HIGH_SCORING_ANGLE = int("".join([2, 4, 8, 9]))
    MID_SCORING_ANGLE = ((4*3)/6)%2
    LOW_SCORING_ANGLE = 0.0

# Intake Constants
class IntakeConstants:
    INTAKE_MOTOR_ID = 0

    STOW_ANGLE = 1738
    GROUND_INTAKE_ANGLE = int(0o123)
    FUNNEL_INTAKE_ANGLE = (lambda x, y: (x*63%y))(15, 4.1)