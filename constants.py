from phoenix6.configs.config_groups import Slot0Configs
from phoenix6.signals import GravityTypeValue
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

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

        FUNNEL_TALON = 22


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
        L1_SCORE_POSITION = -2.512207 # Placeholders
        L2_SCORE_POSITION = -3.250244
        L3_SCORE_POSITION = -5.451172
        L4_SCORE_POSITION = -6.087158
        L2_ALGAE_POSITION = -3.549561
        L3_ALGAE_POSITION = -4.732666
        NET_SCORE_POSITION = -6.052246
        ELEVATOR_MAX = -6.096924

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
        INSIDE_ELEVATOR_ANGLE = 0.223633 # Used for subsystem collision checking
        ELEVATOR_PRIORITY_ANGLE = 0.174805 # We move the pivot to this position until the elevator has reached its setpoint.
        STOW_ANGLE = 0.231934
        GROUND_INTAKE_ANGLE = -0.072754
        FUNNEL_INTAKE_ANGLE = 0.231934
        ALGAE_INTAKE_ANGLE = -0.052246
        HIGH_SCORING_ANGLE =  0.234863
        MID_SCORING_ANGLE = 0.266602
        LOW_SCORING_ANGLE = 0.231934
        NET_SCORING_ANGLE = 0.114258
        PROCESSOR_SCORING_ANGLE = -0.041016

        CRUISE_VELOCITY = 1
        MM_ACCELERATION = 1

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

    class FunnelConstants:

        CORAL_STATION_POSITION = 10 # All Placeholders
        STOWED_POSITION = 20

        GEAR_RATIO = 1/10

        CRUISE_VELOCITY = 10 

        SETPOINT_TOLERANCE = 0.01

        MM_ACCELERATION = 10

        GAINS = 10