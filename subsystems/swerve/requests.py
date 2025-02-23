from typing import Self

from pathplannerlib.config import RobotConfig
from pathplannerlib.util import DriveFeedforwards
from pathplannerlib.util.swerve import SwerveSetpointGenerator, SwerveSetpoint
from phoenix6 import StatusCode
from phoenix6.swerve import SwerveModule, SwerveControlParameters
from phoenix6.swerve.requests import SwerveRequest, ApplyRobotSpeeds
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.units import radians_per_second


class ApplyRobotSetpointSpeeds(SwerveRequest):
    """
    Accepts a generic robot-centric ChassisSpeeds to apply PathPlanner's SwerveSetpointGenerator to allow for more path following control.
    """

    def __init__(self, config: RobotConfig, max_steer_velocity: radians_per_second) -> None:
        self.desired_speeds: ChassisSpeeds = ChassisSpeeds()
        """
        The desired robot-centric chassis speeds to apply to the drivetrain.
        """
        self.drive_request_type: SwerveModule.DriveRequestType = SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        """
        The type of control request to use for the drive motor.
        """
        self.steer_request_type: SwerveModule.SteerRequestType = SwerveModule.SteerRequestType.POSITION
        """
        The type of control request to use for the steer motor.
        """
        self.desaturate_wheel_speeds: bool = True
        """
        Whether to desaturate wheel speeds before applying. For more information, see
        the documentation of SwerveDriveKinematics.desaturateWheelSpeeds.
        """

        self._setpoint_generator = SwerveSetpointGenerator(config, max_steer_velocity)
        self._prev_setpoint = SwerveSetpoint(
            ChassisSpeeds(),
            [SwerveModuleState() for _ in range(config.numModules)],
            DriveFeedforwards.zeros(config.numModules)
        )

        self.__apply_robot_speeds = ApplyRobotSpeeds()

    def with_desired_speeds(self, new_speeds: ChassisSpeeds) -> Self:
        """
        Modifies the speeds parameter and returns itself.
    
        The robot-centric chassis speeds to apply to the drivetrain.
    
        :param new_speeds: Parameter to modify
        :type new_speeds: ChassisSpeeds
        :returns: this object
        :rtype: ApplyRobotSetpointSpeeds
        """

        self.desired_speeds = new_speeds
        return self

    def with_drive_request_type(self, new_drive_request_type: SwerveModule.DriveRequestType) -> Self:
        """
        Modifies the drive request type and returns itself.
    
        The type of control request to use for the drive motor.
    
        :param new_drive_request_type: The desired drive request type to set.
        :type new_drive_request_type: SwerveModule.DriveRequestType
        :returns: this object
        :rtype: ApplyRobotSetpointSpeeds
        """
        self.drive_request_type = new_drive_request_type
        return self

    def with_steer_request_type(self, new_steer_request_type: SwerveModule.SteerRequestType) -> Self:
        """
        Modifies the steer request type and returns itself.
    
        The type of control request to use for the steer motor.
    
        :param new_steer_request_type: The desired steer request type to set.
        :type new_steer_request_type: SwerveModule.SteerRequestType
        :returns: this object
        :rtype: ApplyRobotSetpointSpeeds
        """
        self.steer_request_type = new_steer_request_type
        return self

    def with_desaturate_wheel_speeds(self, desaturate: bool) -> Self:
        """
        Modifies the desaturate wheel speeds property and returns itself.
    
        :param desaturate: Whether to enable desaturating wheel speeds
        :type desaturate: bool
        :returns: this object
        :rtype: ApplyRobotSetpointSpeeds
        """
        self.desaturate_wheel_speeds = desaturate
        return self

    def apply(self, parameters: SwerveControlParameters, modules_to_apply: list[SwerveModule]) -> StatusCode:
        self._prev_setpoint = self._setpoint_generator.generateSetpoint(self._prev_setpoint, self.desired_speeds, 0.02)

        return (self.__apply_robot_speeds
                .with_drive_request_type(self.drive_request_type)
                .with_steer_request_type(self.steer_request_type)
                .with_desaturate_wheel_speeds(self.desaturate_wheel_speeds)
                .with_speeds(self._prev_setpoint.robot_relative_speeds)
                .with_wheel_force_feedforwards_x(self._prev_setpoint.feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(self._prev_setpoint.feedforwards.robotRelativeForcesYNewtons)
                .apply(parameters, modules_to_apply)
        )
