from phoenix6.signals import GravityTypeValue
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from phoenix6.configs.config_groups import Slot0Configs

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

class Constants:

    class MotorIDs:

        LEFT_LIFT_MOTOR = 10
        RIGHT_LIFT_MOTOR = 11
        INTAKE_MOTOR = 12
        PIVOT_MOTOR = 13
        CLIMB_MOTOR = 14

    class ClimberConstants:
        GEAR_RATIO = 100
        GAINS = (Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
        )

    class ElevatorConstants:

        L1_SCORE_POSITION = 1 # Placeholders
        L2_SCORE_POSITION = 2
        L3_SCORE_POSITION = 3
        L4_SCORE_POSITION = 4
        L2_ALGAE_POSITION = 2.5
        L3_ALGAE_POSITION = 3.5
        NET_SCORE_POSITION = 5

        DEFAULT_POSITION = 0

        GEAR_RATIO = 31/4 # Placeholder
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

        STOW_ANGLE = 0
        GROUND_INTAKE_ANGLE = 0.25
        FUNNEL_INTAKE_ANGLE = 0
        ALGAE_INTAKE_ANGLE = 0.25
        HIGH_SCORING_ANGLE = 0.15
        MID_SCORING_ANGLE = 0.25
        LOW_SCORING_ANGLE = 0.25
        NET_SCORING_ANGLE = 0.15
        PROCESSOR_SCORING_ANGLE = 0.25

        GEAR_RATIO = 50 # Placeholder
        GAINS = (Slot0Configs()
                 .with_k_g(0.03)
                 .with_k_p(1.0)
                 .with_k_i(0.0)
                 .with_k_d(0.0)
                 .with_k_s(0.0)
                 .with_k_v(0.0)
                 .with_k_a(0.0)
                 .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

    class IntakeConstants:

        INTAKE_SPEED = 1
        OUTPUT_SPEED = 1

        GEAR_RATIO = 4
        GAINS = (Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
        )
