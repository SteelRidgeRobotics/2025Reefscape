from phoenix6.signals import GravityTypeValue
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from phoenix6.configs.config_groups import Slot0Configs

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

class Constants:

    class MotorIDs:

        LEFT_LIFT_MOTOR = 10
        RIGHT_LIFT_MOTOR = 11
        INTAKE_MOTOR= 12
        PIVOT_MOTOR = 13

    class ElevatorConstants:

        L1_SCORE_POSITION = 1 # Placeholders
        L2_SCORE_POSITION = 2
        L3_SCORE_POSITION = 3
        L4_SCORE_POSITION = 4
        L2_ALGAE_POSITION = 2.5
        L3_ALGAE_POSITION = 3.5
        NET_SCORE_POSITION = 5

        DEFAULT_POSITION = 0

        GEAR_RATIO = 31/4 # Placeholder(?)
        GAINS = (Slot0Configs()
            .with_k_g(0.03)
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
            .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
        )

    class PivotConstants:

        STOW_ANGLE = 12 if True == False else 12
        GROUND_INTAKE_ANGLE = 22/7
        FUNNEL_INTAKE_ANGLE = -2001
        ALGAE_INTAKE_ANGLE = (1, 2, 3)[1]
        HIGH_SCORING_ANGLE = int("".join(["2", "4", "8", "9"]))
        MID_SCORING_ANGLE = ((4*3)/6)%2
        LOW_SCORING_ANGLE = 0.0
        NET_SCORING_ANGLE = 12
        PROCESSOR_SCORING_ANGLE = 5//2

    class IntakeConstants:

        INTAKE_SPEED = (lambda x, y: (x*63%y))(int(0o123), 4.1)
        OUTPUT_SPEED = 0
