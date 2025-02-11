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
            .with_k_p(2.5)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
            .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
        )

        SETPOINT_TOLERANCE = 0.1

    class PivotConstants:
        INSIDE_ELEVATOR_ANGLE = units.degreesToRotations(-20) # Used for subsystem collision checking
        ELEVATOR_PRIORITY_ANGLE = units.degreesToRotations(45) # We move the pivot to this position until the elevator has reached its setpoint.
        STOW_ANGLE = 0
        GROUND_INTAKE_ANGLE = units.degreesToRotations(90)
        FUNNEL_INTAKE_ANGLE = units.degreesToRotations(-30)
        ALGAE_INTAKE_ANGLE = units.degreesToRotations(90)
        HIGH_SCORING_ANGLE = units.degreesToRotations(-45)
        MID_SCORING_ANGLE = units.degreesToRotations(-60)
        LOW_SCORING_ANGLE = units.degreesToRotations(-60)
        NET_SCORING_ANGLE = units.degreesToRotations(-60)
        PROCESSOR_SCORING_ANGLE = units.degreesToRotations(90)

        GEAR_RATIO = 961/36
        GAINS = (Slot0Configs()
                 .with_k_g(0.03)
                 .with_k_p(5.0)
                 .with_k_i(0.0)
                 .with_k_d(0.0)
                 .with_k_s(0.0)
                 .with_k_v(0.0)
                 .with_k_a(0.0)
                 .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        CANCODER_DISCONTINUITY = 0.8
        CANCODER_OFFSET = 0.44775390625

        SETPOINT_TOLERANCE = 0.03125

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