from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


from enum import Enum, auto

from phoenix6.configs.config_groups import Slot0Configs
from wpimath import units


apriltag_layout = AprilTagFieldLayout.loadField(AprilTagField.k2025Reefscape)

class Constants:


    class MotorIDs:

        LEFT_LIFT_MOTOR = 10
        RIGHT_LIFT_MOTOR = 11
        INTAKE_MOTOR = 12
        LEFT_PIVOT_MOTOR = 13
        RIGHT_PIVOT_MOTOR = 14
        CLIMB_MOTOR = 15

    class ClimberConstants:
        GEAR_RATIO = 15376/135
        GAINS = (Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
        )


    class ElevatorConstants():

        L1_SCORE_POSITION = 0 # Placeholders
        L2_SCORE_POSITION = 0
        L3_SCORE_POSITION = 0
        L4_SCORE_POSITION = 0

        DEFAULT_POSITION = 0

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

    class IntakeConstants:

        INTAKE_SPEED = (lambda x, y: (x*63%y))(int(0o123), 4.1)
        OUTPUT_SPEED = 0
