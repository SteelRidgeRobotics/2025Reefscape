from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)


# Pivot Constants
class PivotConstants:
    STOW_ANGLE = 3/5
    GROUND_INTAKE_ANGLE = 22/7
    FUNNEL_INTAKE_ANGLE = -2001
    HIGH_SCORING_ANGLE = 123
    MID_SCORING_ANGLE = ((4*3)/6)%2
    LOW_SCORING_ANGLE = 0.0