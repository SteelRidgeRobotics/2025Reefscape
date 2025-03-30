from phoenix6.configs.config_groups import Slot0Configs
from phoenix6.signals import GravityTypeValue
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpilib import RobotBase


class Constants:

    FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)

    class CanIDs:
        LEFT_ELEVATOR_TALON = 10
        RIGHT_ELEVATOR_TALON = 11
        INTAKE_TALON = 12
        LEFT_PIVOT_TALON = 13
        CLIMB_TALON = 15
        FUNNEL_TALON = 22

        ELEVATOR_CANDI = 20
        PIVOT_CANCODER = 21

        INTAKE_CANRANGE = 23

    class ClimberConstants:
        GEAR_RATIO = 48
        GAINS = (Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
        )

        VOLTAGE_INWARDS = 4
        VOLTAGE_OUTWARDS = -5
        CLIMB_IN_VOLTAGE = 2.5

        CLIMB_FULL_THRESHOLD = 0.44

        SERVO_PORT = 0
        SERVO_ENGAGED_ANGLE = 0
        SERVO_DISENGAGED_ANGLE = 90

    class ElevatorConstants:
        L1_SCORE_POSITION = 2.208
        L2_SCORE_POSITION = 1.841
        L3_SCORE_POSITION = 3.576
        L4_SCORE_POSITION = 6.087158
        L2_ALGAE_POSITION = 3.198
        L3_ALGAE_POSITION = 5
        NET_SCORE_POSITION = 6.052246
        PROCESSOR_SCORE_POSITION = 0.8205
        ELEVATOR_MAX = 6.096924

        DEFAULT_POSITION = 0

        CRUISE_VELOCITY = 9.5
        MM_JERK = 6000
        MM_UPWARD_ACCELERATION = 65
        MM_BRAKE_ACCELERATION = 24
        MM_DOWNWARD_ACCELERATION = 12
        EXPO_K_V = 10
        EXPO_K_A = 4

        GEAR_RATIO = 31/4
        GAINS = (Slot0Configs()
            .with_k_g(0.352)
            .with_k_p(40)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.062)
            .with_k_v(0.0)
            .with_k_a(0.0)
            .with_gravity_type(GravityTypeValue.ELEVATOR_STATIC)
        )

        SETPOINT_TOLERANCE = 0.1

    class PivotConstants:
        INSIDE_ELEVATOR_ANGLE = 0.2 # Used for subsystem collision checking
        ELEVATOR_PRIORITY_ANGLE = 0.123535 # We move the pivot to this position until the elevator has reached its setpoint.
        STOW_ANGLE = 0.188
        GROUND_INTAKE_ANGLE = -0.081543
        FUNNEL_INTAKE_ANGLE = 0.28
        ALGAE_INTAKE_ANGLE = -0.05
        HIGH_SCORING_ANGLE =  0.21
        MID_SCORING_ANGLE = 0.22
        LOW_SCORING_ANGLE = -0.081543
        NET_SCORING_ANGLE = 0.131
        PROCESSOR_SCORING_ANGLE = 0.001
        CLIMBER_PRIORITY_ANGLE = 0.201943

        MINIMUM_ANGLE = -0.091
        MAXIMUM_ANGLE = 0.392822

        CRUISE_VELOCITY = 3
        MM_ACCELERATION = 3

        GEAR_RATIO = 961/36
        GAINS = (Slot0Configs()
                 .with_k_g(0.27)
                 .with_k_p(30 if RobotBase.isReal() else 60)
                 .with_k_i(0.0)
                 .with_k_d(0.6343)
                 .with_k_s(0.19)
                 .with_k_v(0)
                 .with_k_a(0)
                 .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        CANCODER_DISCONTINUITY = 0.5
        CANCODER_OFFSET = 0.380126953125

        SETPOINT_TOLERANCE = 0.03125

    class IntakeConstants:

        CORAL_INTAKE_SPEED = 0.4*1.2*1.1
        FUNNEL_INTAKE_SPEED = 0.8*0.75
        CORAL_OUTPUT_SPEED = 0.6
        L1_OUTPUT_SPEED = -0.4

        ALGAE_HOLD = 0.125
        ALGAE_INTAKE_SPEED = 0.75
        ALGAE_OUTPUT_SPEED = -1

        SUPPLY_CURRENT = 35

        GEAR_RATIO = 4
        GAINS = (Slot0Configs()
            .with_k_p(1.0)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
        )

    class VisionConstants:
        FRONT_LEFT = "limelight-fl"
        FRONT_RIGHT = "limelight-fr"
        FRONT_CENTER = "limelight-front"
        BACK_CENTER = "limelight-back"

    class FunnelConstants:

        CORAL_STATION_POSITION = 0.24
        STOWED_POSITION = 0

        GEAR_RATIO = 12

        CRUISE_VELOCITY = 1 

        SETPOINT_TOLERANCE = 0.01

        MM_ACCELERATION = 3.5

        GAINS = (Slot0Configs()
            .with_k_p(45)
            .with_k_i(0.0)
            .with_k_d(0.0)
            .with_k_s(0.0)
            .with_k_v(0.0)
            .with_k_a(0.0)
            .with_k_g(0.25)
            .with_gravity_type(GravityTypeValue.ARM_COSINE)
        )

        SUPPLY_CURRENT = 20
        STATOR_CURRENT = 50
    
    class AutoAlignConstants:
        TRANSLATION_P = 9
        TRANSLATION_I = 0
        TRANSLATION_D = 0.1
        
        HEADING_P = 1
        HEADING_I = 0
        HEADING_D = 0.2
