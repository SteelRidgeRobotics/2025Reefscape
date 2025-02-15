from phoenix6.signals import GravityTypeValue
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from phoenix6.configs.config_groups import Slot0Configs
from wpimath import units

apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

class Constants:

    class CanIDs:
        LEFT_ELEVATOR_TALON = 10
        RIGHT_ELEVATOR_TALON = 11
        INTAKE_TALON = 12
        LEFT_PIVOT_TALON = 13
        RIGHT_PIVOT_TALON = 14
        CLIMB_TALON = 15

        ELEVATOR_CANDI = 20
        PIVOT_CANCODER = 21


    class ClimberConstants:
        GEAR_RATIO = 15376/45
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
        GROUND_INTAKE_ANGLE = units.degreesToRotations(90)
        FUNNEL_INTAKE_ANGLE = 0
        ALGAE_INTAKE_ANGLE = units.degreesToRotations(90)
        HIGH_SCORING_ANGLE = units.degreesToRotations(54)
        MID_SCORING_ANGLE = units.degreesToRotations(90)
        LOW_SCORING_ANGLE = units.degreesToRotations(90)
        NET_SCORING_ANGLE = units.degreesToRotations(54)
        PROCESSOR_SCORING_ANGLE = units.degreesToRotations(90)

        GEAR_RATIO = 961/36
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
        CANCODER_OFFSET = 0.0069

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